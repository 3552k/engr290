/*
 * ENGR 290 Team 6 hovercraft - Ella's part
 * Front ultrasonic sensor + steering servo + thrust fan, in plain AVR C
 * (ATmega328P on the Arduino Nano controller board, 16 MHz, no Arduino library).
 *
 * Plan v3, stage 1 (safe version). IMU functions come from imu.h (Iphi);
 * the driver is in imu.c. Build with "make", flash with "make upload".
 *
 * Behaviour:
 *   WAIT      2 s start delay, fans off, IMU calibrated here
 *   CRUISE    hold heading (servo = 90 + IMU_Steer()), ping the front sensor
 *   SCAN      wall ahead: cut thrust, sweep the sensor across SCAN_ANGLES,
 *             remember the distance at each angle
 *   TURN      steer toward the angle with the most room; ends when the IMU
 *             says the turn is done, or after TURN_TIMEOUT_MS as a safety
 *   RECENTRE  straighten the servo, then back to CRUISE
 *   STOPPED   fans off (bar found, or time limit reached)
 *
 * Wiring (Controller_connectors_pinout_1_12.pdf, hardware v3.11):
 *   Front ultrasonic  P6   Echo = PD2 (INT0), Trig = PB3
 *   Servo             P9   PB1 (OC1A), 200 ohm series resistor on the board
 *   Thrust fan        P4   PD6 (OC0A, "PWM1")
 *   Lift fan          P3   PD5 (OC0B, "PWM2")
 *
 * Timers:
 *   Timer0  fan PWM (about 977 Hz)
 *   Timer1  servo pulse, 20 ms period, 0.5 us resolution
 *   Timer2  1 ms system tick (also gives 4 us timing for the echo pulse)
 *
 * Assumptions to check on the real craft:
 *   - the front sensor is mounted on the servo and rotates with the thrust fan
 *   - servo 90 = straight ahead, larger angle = left
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#include "tick.h"
#include "imu.h"

 /* ------------------------------ tuning ------------------------------ */
 /* All of these are starting guesses. Tune them on the real craft.      */

#define WALL_CM           40      /* start a scan when a wall is this close   */
#define WALL_HITS         2       /* readings in a row needed to believe it   */
#define NO_ECHO_CM        400     /* value used when nothing is in range      */

#define SERVO_CENTRE      90
#define SERVO_MIN_ANGLE   20      /* mechanical limits of the mount           */
#define SERVO_MAX_ANGLE   160

#define THRUST_CRUISE     200     /* fan duty, 0-255                          */
#define THRUST_TURN       160
#define LIFT_DUTY         255     /* set to 0 if someone else's code runs lift */

#define PING_PERIOD_MS    60      /* gap between pings (echoes need to die out) */
#define PING_TIMEOUT_MS   40      /* give up on an echo after this long       */
#define TURN_TIMEOUT_MS   3000    /* safety: end a turn even if the IMU never says done */
#define START_DELAY_MS    2000    /* hands clear before the fans start        */
#define RUN_LIMIT_MS      120000UL /* course time limit: 2 minutes            */

#define DEBUG_UART        1       /* 1 = print status at 9600 baud            */

/* Angles looked at during a scan, left side first. */
static const uint8_t SCAN_ANGLES[] = { 150, 120, 60, 30 };
#define SCAN_COUNT (sizeof(SCAN_ANGLES) / sizeof(SCAN_ANGLES[0]))

/* --------------------------- system tick ---------------------------- */

static volatile uint32_t g_ms = 0;

ISR(TIMER2_COMPA_vect)
{
    g_ms++;
}

static void tick_init(void)
{
    /* CTC mode, prescaler 64: 16 MHz / 64 = 250 kHz, 250 counts = 1 ms */
    TCCR2A = _BV(WGM21);
    TCCR2B = _BV(CS22);
    OCR2A = 249;
    TIMSK2 = _BV(OCIE2A);
}

uint32_t millis_now(void)
{
    uint32_t ms;
    uint8_t sreg = SREG;
    cli();
    ms = g_ms;
    SREG = sreg;
    return ms;
}

/* Microseconds since start, 4 us steps. Call with interrupts disabled. */
static uint32_t micros_locked(void)
{
    uint32_t ms = g_ms;
    uint8_t  t = TCNT2;
    /* a tick happened but its interrupt has not run yet */
    if ((TIFR2 & _BV(OCF2A)) && t < 125) {
        ms++;
    }
    return ms * 1000UL + (uint32_t)t * 4UL;
}

/* ----------------------------- debug UART --------------------------- */

#if DEBUG_UART
static void uart_init(void)
{
    UBRR0 = 103;                        /* 9600 baud at 16 MHz */
    UCSR0B = _BV(TXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);  /* 8 data bits, 1 stop bit */
}

static void uart_char(char c)
{
    while (!(UCSR0A & _BV(UDRE0))) {}
    UDR0 = (uint8_t)c;
}

static void uart_text(const char* s)
{
    while (*s) {
        uart_char(*s++);
    }
}

static void uart_number(uint16_t n)
{
    char buf[6];
    uint8_t i = 0;
    do {
        buf[i++] = (char)('0' + n % 10);
        n /= 10;
    } while (n && i < sizeof(buf));
    while (i) {
        uart_char(buf[--i]);
    }
}
#else
#define uart_init()
#define uart_text(s)
#define uart_number(n)
#endif

/* ------------------------------- fans ------------------------------- */

static void fans_init(void)
{
    DDRD |= _BV(PD6) | _BV(PD5);
    PORTD &= (uint8_t)~(_BV(PD6) | _BV(PD5));
    /* Fast PWM, prescaler 64. Outputs are connected only while duty > 0. */
    TCCR0A = _BV(WGM01) | _BV(WGM00);
    TCCR0B = _BV(CS01) | _BV(CS00);
}

static void thrust_set(uint8_t duty)
{
    if (duty == 0) {
        TCCR0A &= (uint8_t)~_BV(COM0A1);   /* pin back to plain low = fully off */
    }
    else {
        OCR0A = duty;
        TCCR0A |= _BV(COM0A1);
    }
}

static void lift_set(uint8_t duty)
{
    if (duty == 0) {
        TCCR0A &= (uint8_t)~_BV(COM0B1);
    }
    else {
        OCR0B = duty;
        TCCR0A |= _BV(COM0B1);
    }
}

/* ------------------------------- servo ------------------------------ */

static uint8_t g_servo_angle = SERVO_CENTRE;

static void servo_write(uint8_t angle)
{
    uint16_t pulse_us;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    g_servo_angle = angle;
    /* HS-311: about 10 us per degree, 1500 us = centre */
    pulse_us = (uint16_t)(600 + (uint16_t)angle * 10);
    OCR1A = (uint16_t)(pulse_us * 2);      /* timer counts in 0.5 us steps */
}

static void servo_init(void)
{
    DDRB |= _BV(PB1);
    /* Fast PWM with ICR1 as top, prescaler 8: 40000 counts = 20 ms.
     * The mode is set first because ICR1 can only be written in this mode. */
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
    ICR1 = 39999;
    servo_write(SERVO_CENTRE);
}

/* Time the servo needs to travel between two angles (HS-311 is roughly
 * 3 ms per degree; a margin is added so the sensor is still when we ping). */
static uint16_t servo_travel_ms(uint8_t from, uint8_t to)
{
    uint8_t delta = (from > to) ? (uint8_t)(from - to) : (uint8_t)(to - from);
    return (uint16_t)(150 + (uint16_t)delta * 4);
}

/* ---------------------------- ultrasonic ---------------------------- */

static volatile uint32_t g_echo_start_us = 0;
static volatile uint32_t g_echo_width_us = 0;
static volatile uint8_t  g_echo_done = 0;

/* Echo pin changed: rising edge starts the stopwatch, falling edge stops it. */
ISR(INT0_vect)
{
    uint32_t now = micros_locked();
    if (PIND & _BV(PD2)) {
        g_echo_start_us = now;
    }
    else {
        g_echo_width_us = now - g_echo_start_us;
        g_echo_done = 1;
    }
}

static uint8_t  g_ping_waiting = 0;
static uint32_t g_ping_sent_ms = 0;

static void us_init(void)
{
    DDRB |= _BV(PB3);                    /* trig = output, low */
    PORTB &= (uint8_t)~_BV(PB3);
    DDRD &= (uint8_t)~_BV(PD2);          /* echo = input */
    EICRA |= _BV(ISC00);                  /* INT0 on any edge */
    EIMSK |= _BV(INT0);
}

/* Throw away any ping in progress (used after the servo moves). */
static void us_restart(void)
{
    g_ping_waiting = 0;
    g_ping_sent_ms = millis_now();
}

/*
 * Call this every pass of the main loop. It sends pings on its own schedule.
 * Returns 1 and fills *cm when a new distance is ready, otherwise returns 0.
 */
static uint8_t us_update(uint16_t* cm)
{
    uint32_t now = millis_now();

    if (!g_ping_waiting) {
        /* wait for the gap, and never ping while the echo line is still high */
        if (now - g_ping_sent_ms >= PING_PERIOD_MS && !(PIND & _BV(PD2))) {
            cli();
            g_echo_done = 0;
            sei();
            PORTB |= _BV(PB3);
            _delay_us(10);
            PORTB &= (uint8_t)~_BV(PB3);
            g_ping_sent_ms = now;
            g_ping_waiting = 1;
        }
        return 0;
    }

    if (g_echo_done) {
        uint32_t width;
        uint16_t d;
        cli();
        width = g_echo_width_us;
        g_echo_done = 0;
        sei();
        g_ping_waiting = 0;
        d = (uint16_t)(width / 58UL);     /* 58 us per cm, there and back */
        *cm = (d == 0 || d > NO_ECHO_CM) ? NO_ECHO_CM : d;
        return 1;
    }

    if (now - g_ping_sent_ms >= PING_TIMEOUT_MS) {
        g_ping_waiting = 0;
        *cm = NO_ECHO_CM;                 /* nothing came back */
        return 1;
    }
    return 0;
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

/* ------------------- hooks for the rest of the team ------------------ */

/* Rachel: return 1 when the upward sensor sees the bar. */
static uint8_t bar_detected(void)
{
    return 0;
}

/* Keep an angle difference in -180..+180. */
static float wrap180(float a)
{
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* Servo angle that holds the IMU's target heading. */
static uint8_t heading_hold_angle(void)
{
    float a = (float)SERVO_CENTRE + IMU_Steer();
    if (a < 0.0f)   a = 0.0f;
    if (a > 255.0f) a = 255.0f;
    return (uint8_t)(a + 0.5f);           /* servo_write clamps to the mount limits */
}

/* --------------------------- state machine --------------------------- */

#define ST_WAIT      0
#define ST_CRUISE    1
#define ST_SCAN      2
#define ST_TURN      3
#define ST_RECENTRE  4
#define ST_STOPPED   5

static uint8_t  g_state = ST_WAIT;
static uint32_t g_state_start_ms = 0;
static uint16_t g_state_wait_ms = 0;      /* servo settling time for this state */

static uint8_t  g_wall_hits = 0;
static uint8_t  g_scan_index = 0;
static uint8_t  g_scan_samples = 0;
static uint16_t g_scan_sample[3];
static uint16_t g_scan_cm[SCAN_COUNT];
static uint8_t  g_turn_angle = SERVO_CENTRE;
static float    g_scan_yaw = 0.0f;        /* heading when the scan started */
static uint8_t  g_imu_ready = 0;

static void enter(uint8_t state)
{
    g_state = state;
    g_state_start_ms = millis_now();
    if (state == ST_CRUISE) {
        uart_text("CRUISE\r\n");
    }
}

/* Point the servo somewhere and note how long it needs to get there. */
static void aim(uint8_t angle)
{
    g_state_wait_ms = servo_travel_ms(g_servo_angle, angle);
    servo_write(angle);
    us_restart();
}

static void stop_everything(const char* why)
{
    thrust_set(0);
    lift_set(0);
    servo_write(SERVO_CENTRE);
    uart_text("STOP ");
    uart_text(why);
    uart_text("\r\n");
    enter(ST_STOPPED);
}

static void start_scan(void)
{
    thrust_set(0);
    g_scan_yaw = IMU_Yaw();               /* scan angles are measured from here */
    g_scan_index = 0;
    g_scan_samples = 0;
    aim(SCAN_ANGLES[0]);
    enter(ST_SCAN);
}

static void finish_scan(void)
{
    uint8_t i;
    uint8_t best = 0;
    for (i = 1; i < SCAN_COUNT; i++) {
        if (g_scan_cm[i] > g_scan_cm[best]) {
            best = i;
        }
    }
    g_turn_angle = SCAN_ANGLES[best];
    uart_text("TURN to ");
    uart_number(g_turn_angle);
    uart_text("\r\n");
    /* Turn by (chosen - 90), measured from the heading at the start of the
     * scan. With thrust off the craft can drift round during the scan, so
     * subtract whatever it already rotated. */
    IMU_SetTarget(wrap180((float)g_turn_angle - (float)SERVO_CENTRE
        + g_scan_yaw - IMU_Yaw()));
    aim(g_turn_angle);
    thrust_set(THRUST_TURN);
    enter(ST_TURN);
}

static void control_step(void)
{
    uint32_t now = millis_now();
    uint32_t in_state = now - g_state_start_ms;
    uint16_t cm;

    if (g_state == ST_STOPPED) {
        return;
    }
    if (now >= RUN_LIMIT_MS) {
        stop_everything("time limit");
        return;
    }
    if (g_state != ST_WAIT && bar_detected()) {
        stop_everything("bar");
        return;
    }

    switch (g_state) {
    case ST_WAIT:
        servo_write(SERVO_CENTRE);        /* face straight ahead while calibrating and waiting */
        if (!g_imu_ready) {
            /* craft is still and fans are off: calibrate now (~2 s) */
            g_imu_ready = IMU_Calibrate();
            if (!g_imu_ready) {
                uart_text("IMU calibrate failed, retrying\r\n");
            }
            else {
                uart_text("IMU calibrated\r\n");
                enter(ST_WAIT);           /* start delay counts from here */
            }
            break;
        }
        if (in_state >= START_DELAY_MS) {
            lift_set(LIFT_DUTY);
            thrust_set(THRUST_CRUISE);
            g_wall_hits = 0;
            us_restart();
            enter(ST_CRUISE);
        }
        break;

    case ST_CRUISE:
        servo_write(heading_hold_angle());
        if (us_update(&cm)) {
            uart_text("front ");
            uart_number(cm);
            uart_text("\r\n");
            if (cm < WALL_CM) {
                g_wall_hits++;
            }
            else {
                g_wall_hits = 0;
            }
            if (g_wall_hits >= WALL_HITS) {
                start_scan();
            }
        }
        break;

    case ST_SCAN:
        if (in_state < g_state_wait_ms) {
            us_restart();                 /* servo still moving: no pings yet */
            break;
        }
        if (us_update(&cm)) {
            g_scan_sample[g_scan_samples++] = cm;
            if (g_scan_samples == 3) {
                g_scan_cm[g_scan_index] =
                    median3(g_scan_sample[0], g_scan_sample[1], g_scan_sample[2]);
                uart_text("scan ");
                uart_number(SCAN_ANGLES[g_scan_index]);
                uart_text(" = ");
                uart_number(g_scan_cm[g_scan_index]);
                uart_text("\r\n");
                g_scan_index++;
                g_scan_samples = 0;
                if (g_scan_index < SCAN_COUNT) {
                    aim(SCAN_ANGLES[g_scan_index]);
                    enter(ST_SCAN);
                }
                else {
                    finish_scan();
                }
            }
        }
        break;

    case ST_TURN:
        if (IMU_TurnFinished() || in_state >= TURN_TIMEOUT_MS) {
            if (in_state >= TURN_TIMEOUT_MS) {
                uart_text("turn timeout\r\n");
            }
            aim(SERVO_CENTRE);
            enter(ST_RECENTRE);
        }
        break;

    case ST_RECENTRE:
        if (in_state >= g_state_wait_ms) {
            thrust_set(THRUST_CRUISE);
            g_wall_hits = 0;
            us_restart();
            enter(ST_CRUISE);
        }
        break;

    default:
        break;
    }
}

/* -------------------------------- main ------------------------------- */

/* Print why the board (re)started. "brownout" means the supply voltage
 * dipped, e.g. when the fans or servo start. */
static void report_reset_cause(void)
{
    uint8_t cause = MCUSR;
    MCUSR = 0;
    uart_text("reset:");
    if (cause & _BV(PORF))  uart_text(" power-on");
    if (cause & _BV(EXTRF)) uart_text(" reset-pin");
    if (cause & _BV(BORF))  uart_text(" BROWNOUT");
    if (cause & _BV(WDRF))  uart_text(" watchdog");
    if (!cause)             uart_text(" unknown");
    uart_text("\r\n");
}

static void hw_init(void)
{
    tick_init();
    uart_init();
    fans_init();
    servo_init();
    us_init();
}

int main(void)
{
    hw_init();
    sei();
    report_reset_cause();
    uart_text("ready\r\n");
    if (!IMU_Init()) {
        uart_text("IMU not found\r\n");
    }
    enter(ST_WAIT);
    for (;;) {
        IMU_Update();
        control_step();
    }
}
