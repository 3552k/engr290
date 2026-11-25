/*
 * servo_test.c - bench test for the steering servo, driven from the keyboard.
 *
 * Build and flash:  make upload-servo-test
 * Then:             make monitor   (type the keys below, Ctrl+C to exit)
 *
 * Servo on P9 = PB1 (OC1A), same pulse timing as main.c.
 * The fans are forced off for the whole test.
 *
 * Keys:
 *   a / d    10 degrees left / right      A / D   1 degree left / right
 *   c        centre (90)
 *   1 2 3 4  the scan angles 150, 120, 60, 30
 *   s        sweep through the scan angles once, like a real scan
 *   h        help
 *
 * Re-flash the real firmware afterwards with "make upload".
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

/* Keep these the same as main.c */
#define SERVO_CENTRE      90
#define SERVO_TRIM        (-7)    /* fan points straight at servo 83: measured 2026-11-25 */
#define SERVO_MIN_ANGLE   20
#define SERVO_MAX_ANGLE   160

static const uint8_t SCAN_ANGLES[] = { 150, 120, 60, 30 };
#define SCAN_COUNT (sizeof(SCAN_ANGLES) / sizeof(SCAN_ANGLES[0]))

/* ------------------------------- UART ------------------------------- */

static void uart_init(void)
{
    UBRR0  = 103;                          /* 9600 baud at 16 MHz */
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}

static void uart_char(char c)
{
    while (!(UCSR0A & _BV(UDRE0))) { }
    UDR0 = (uint8_t)c;
}

static void uart_text(const char *s)
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

/* Returns the key pressed, or 0 if none. */
static char uart_key(void)
{
    if (UCSR0A & _BV(RXC0)) {
        return (char)UDR0;
    }
    return 0;
}

/* ------------------------------- servo ------------------------------ */

static uint8_t g_angle = SERVO_CENTRE;

static void servo_write(int16_t angle)
{
    uint16_t pulse_us;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    g_angle = (uint8_t)angle;
    pulse_us = (uint16_t)(600 + ((int16_t)g_angle + SERVO_TRIM) * 10);  /* trim so 90 = straight */
    OCR1A = (uint16_t)(pulse_us * 2);

    uart_text("servo ");
    uart_number(g_angle);
    uart_text(" deg (pulse ");
    uart_number(pulse_us);
    uart_text(" us)");
    if (g_angle == SERVO_MIN_ANGLE || g_angle == SERVO_MAX_ANGLE) {
        uart_text("  [limit]");
    }
    uart_text("\r\n");
}

static void servo_init(void)
{
    DDRB  |= _BV(PB1);
    TCCR1A = _BV(COM1A1) | _BV(WGM11);     /* fast PWM, ICR1 = top */
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
    ICR1   = 39999;                        /* 20 ms period */
}

/* ------------------------------- main ------------------------------- */

static void help(void)
{
    uart_text("\r\nSERVO TEST (P9). Bigger angle = left.\r\n"
              "  a/d = 10 deg left/right, A/D = 1 deg\r\n"
              "  c = centre, 1-4 = 150/120/60/30, s = sweep, h = help\r\n");
}

int main(void)
{
    /* fans off: thrust PD6, lift PD5 */
    DDRD  |= _BV(PD6) | _BV(PD5);
    PORTD &= (uint8_t)~(_BV(PD6) | _BV(PD5));

    uart_init();
    servo_init();
    help();
    servo_write(SERVO_CENTRE);

    for (;;) {
        char k = uart_key();
        switch (k) {
        case 'a': servo_write((int16_t)g_angle + 10); break;
        case 'd': servo_write((int16_t)g_angle - 10); break;
        case 'A': servo_write((int16_t)g_angle + 1);  break;
        case 'D': servo_write((int16_t)g_angle - 1);  break;
        case 'c': servo_write(SERVO_CENTRE);          break;
        case '1': case '2': case '3': case '4':
            servo_write(SCAN_ANGLES[k - '1']);
            break;
        case 's': {
            uint8_t i;
            uart_text("sweep:\r\n");
            for (i = 0; i < SCAN_COUNT; i++) {
                servo_write(SCAN_ANGLES[i]);
                _delay_ms(800);
            }
            servo_write(SERVO_CENTRE);
            break;
        }
        case 'h': help(); break;
        default: break;
        }
    }
}
