// detect bar
// Classic Arduino Nano (ATmega328P), runs at 16Mhz
// Serial monitor: 115200 baud
//
// Commands: p = pause/resume printing, h = help
//           1/3/5 = capture trial 1/2/3 at 49 cm, 2/4/6 = trial 1/2/3 at 16 cm

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>

#define US_TRIG PB4           // Ultrasonic trigger, P10/3
#define US_ECHO PB0           // Ultrasonic echo, P10/2
#define LED_D3 PB3            // D3 (OC2A), active low
#define LED_L PB5             // Nano yellow L

#define SAMPLE_INTERVAL_MS 100 // HC-SR04 needs >60 ms between pings
#define ECHO_TIMEOUT_TICKS 60000U // 30 ms at 0.5 us per tick

// ---- Bar detection tuning ----
#define BAR_DIST_MM   150   // bar is "detected" when closer than this (15 cm)
#define MIN_VALID_MM   20   // ignore anything closer than this (2cm)
#define CONFIRM_COUNT   2   // consecutive close readings needed to detect
#define CLEAR_COUNT     3   // consecutive non-close readings needed to clear

// Using volatile to stop the values from being cached
volatile uint32_t millis_count = 0;
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;
volatile uint8_t rx_buffer[64];

uint8_t streaming = 1;
uint32_t last_sample_ms = 0;

// Values from the latest reading
uint32_t echo_us = 0;
int16_t bar_mm = -1;        //-1 is bad reading
uint8_t bar_detected = 0;   //1 = yes
uint8_t hit_count = 0;
uint8_t miss_count = 0;
const char *reading_status = 0;


// USART Found on section 19 of atmega datasheet

void USART_Init() {
    // Double speed mode, UBRR = 16 gives 115200 baud
    UCSR0A = (1 << U2X0);
    UBRR0H = 0;
    UBRR0L = 16;
    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    // Enable receive/transmit and the receive interrupt
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

// Received characters are stored in a ring buffer so commands are not lost
// while waiting for an echo
ISR(USART_RX_vect) {
    uint8_t flags = UCSR0A;
    uint8_t data = UDR0;
    uint8_t next = (rx_head + 1) & 63;
    if (!(flags & (1 << UPE0)) && next != rx_tail) {
        rx_buffer[rx_head] = data;
        rx_head = next;
    }
}

uint8_t USART_Available() {
    return rx_head != rx_tail;
}

uint8_t USART_Read() {
    uint8_t data = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) & 63;
    return data;
}

void USART_Transmit(unsigned char data) {
    // wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = data;
}

// Strings are kept in flash (PSTR) to save RAM
void USART_Print(const char *flash_str) {
    char c;
    while ((c = pgm_read_byte(flash_str++))) {
        USART_Transmit(c);
    }
}

void USART_Newline() {
    USART_Transmit('\r');
    USART_Transmit('\n');
}

void USART_Println(const char *flash_str) {
    USART_Print(flash_str);
    USART_Newline();
}

//nb -> txt
void USART_PrintNum(uint32_t value) {       
    char digits[10];
    uint8_t length = 0;
    do {
        digits[length++] = '0' + value % 10;
        value /= 10;
    } while (value);
    while (length) {
        USART_Transmit(digits[--length]);
    }
}

// Timer 0: 1 ms tick for timekeeping (section 14)
void Timer0_Setup() {
    // CTC mode, prescaler 64, 16MHz / 64 / 250 = 1 kHz
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);
}

//time passed
ISR(TIMER0_COMPA_vect) {
    millis_count++;
}

uint32_t Millis() {
    uint32_t ms;
    // Disable interrupts so the 32 bit value is not changed mid read
    uint8_t sreg = SREG;
    cli();
    ms = millis_count;
    SREG = sreg;
    return ms;
}

// Timer 1: free running, used to time the echo pulse (section 15)
void Timer1_Setup() {
    TCCR1A = 0;
    // Prescaler of 8 so 0.5us per tick
    TCCR1B = (1 << CS11);
    TCNT1 = 0;
}

// Timer 2: PWM for D3 on OC2A/PB3 (section 17)
void PWM_Setup() {
    // Write high first so the active low LED starts off
    PORTB |= (1 << LED_D3);
    DDRB |= (1 << LED_D3);
    OCR2A = 0;
    // Phase correct PWM, inverting output since D3 is active low,
    // so OCR2A = 0 is fully off and OCR2A = 255 is fully on
    TCCR2A = (1 << COM2A1) | (1 << COM2A0) | (1 << WGM20);
    // Prescaler of 64, about 490 Hz
    TCCR2B = (1 << CS22);
}

void Pins_Setup() {
    // Course pin sheet requires pull ups on unused inputs PD2 and PD3
    DDRD &= ~((1 << PD2) | (1 << PD3));
    PORTD |= (1 << PD2) | (1 << PD3);
    // Yellow L as output, off
    DDRB |= (1 << LED_L);
    PORTB &= ~(1 << LED_L);
    // Echo as input
    DDRB &= ~(1 << US_ECHO);
    PORTB &= ~(1 << US_ECHO);
    // Trigger as output
    DDRB |= (1 << US_TRIG);
    PORTB &= ~(1 << US_TRIG);
}


// Measurement math (integer, results in mm)

// d = c*t/2 with c = 340 m/s, so d(mm) = t(us) * 0.17, rounded
uint16_t US_Distance_Mm(uint32_t us) {
    return (us * 17 + 50) / 100;
}

// Returns echo pulse length in us, or 0 if there was no echo within 30 ms
uint32_t US_Echo_Us() {
    // Send 10 us trigger pulse
    PORTB &= ~(1 << US_TRIG);
    _delay_us(2);
    PORTB |= (1 << US_TRIG);
    _delay_us(10);
    PORTB &= ~(1 << US_TRIG);

    uint16_t start = TCNT1;
    // Wait for the rising edge
    while (!(PINB & (1 << US_ECHO))) {
        if ((uint16_t)(TCNT1 - start) >= ECHO_TIMEOUT_TICKS) return 0;
    }
    uint16_t rise = TCNT1;
    // Wait for the falling edge
    while (PINB & (1 << US_ECHO)) {
        if ((uint16_t)(TCNT1 - start) >= ECHO_TIMEOUT_TICKS) return 0;
    }
    uint16_t fall = TCNT1;
    return (uint16_t)(fall - rise) / 2;
}

void Read_Sensor() {
    echo_us = 0;
    bar_mm = -1;

    if (PINB & (1 << US_ECHO)) {
        reading_status = PSTR("ECHO_STUCK_HIGH");
    } else {
        echo_us = US_Echo_Us();
        if (echo_us == 0) {
            reading_status = PSTR("NO_ECHO");
        } else {
            uint16_t mm = US_Distance_Mm(echo_us);
            // HC-SR04 is specified for 2 cm to 400 cm
            if (mm < 20 || mm > 4000) {
                reading_status = PSTR("OUTSIDE_US_SPEC");
            } else {
                bar_mm = mm;
                reading_status = PSTR("US_OK");
            }
        }
    }

    // Bar detection with confirm/clear counters to reject noise
    if (bar_mm >= MIN_VALID_MM && bar_mm <= BAR_DIST_MM) {
        miss_count = 0;
        if (hit_count < 255) hit_count++;
        if (hit_count >= CONFIRM_COUNT) bar_detected = 1;
    } else {
        hit_count = 0;
        if (miss_count < 255) miss_count++;
        if (miss_count >= CLEAR_COUNT) bar_detected = 0;
    }

    //D3 on while bar is detected
    OCR2A = bar_detected ? 255:0;
}


void Print_Help() {
    
    USART_Println(PSTR("# HC-SR04 bar detection (detection only, no fan control)"));
    USART_Println(PSTR("# p=pause/resume printing; h=help"));
    USART_Print(PSTR("# Bar detected when closer than (mm): "));
    USART_PrintNum(BAR_DIST_MM);
    USART_Newline();
    USART_Print(PSTR("# D3 on = bar detected"));
}

// e.g. "Distance: 7.3 cm  Echo: 431 us Bar: YES"
// Invalid readings print "--" for the distance followed by the reason
void Print_Status() {
    USART_Print(PSTR("Distance: "));
    if (bar_mm >= 0) {
        USART_PrintNum(bar_mm / 10);
        USART_Transmit('.');
        USART_PrintNum(bar_mm % 10);
    } else {
        USART_Print(PSTR("--"));
    }
    USART_Print(PSTR(" cm  Echo: "));
    USART_PrintNum(echo_us);
    USART_Print(PSTR(" us Bar: "));
    USART_Print(bar_detected ? PSTR("YES") : PSTR("NO"));
    if (bar_mm < 0) {
        USART_Print(PSTR("  ("));
        USART_Print(reading_status);
        USART_Transmit(')');
    }
    USART_Newline();
}

void Handle_Commands() {
    while (USART_Available()) {
        char command = USART_Read();
        if (command == 'p' || command == 'P') {
            streaming = !streaming;
        } else if (command == 'h' || command == 'H') {
            Print_Help();
        } 
    }
}

int main(void) {
    USART_Init();
    Pins_Setup();
    Timer0_Setup();
    Timer1_Setup();
    PWM_Setup();
    sei();

    // Sensor startup and reference settling, then discard first conversions
    _delay_ms(100);
    Print_Help();
    last_sample_ms = Millis();

    while (1) {
        Handle_Commands();

        uint32_t now = Millis();
        if (now - last_sample_ms >= SAMPLE_INTERVAL_MS) {
            last_sample_ms = now;
            Read_Sensor();
            if (streaming) {
                Print_Status();
            }
        }
    }
}
