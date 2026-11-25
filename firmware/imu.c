// imu.c v2 - MPU-6050 driver for the hovercraft. Does not use Timer 0 (fans).
// v2: FIFO (no missed samples), tilt-independent yaw, glitch-tolerant,
//     optional integral term and turn prediction (both off by default).

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <math.h>
#include "imu.h"

// ---- Settings ----
#include "tick.h"              // millis_now(): Ella's 1 ms tick in main.c
#define IMU_NOW_MS() millis_now()

#define IMU_YAW_SIGN 1          // -1 if turning left makes yaw go the wrong way

#define IMU_TURN_TOLERANCE 15.0f
#define IMU_TURN_LEAD_S    0.0f // >0: end turns early to absorb overshoot (try 0.1-0.2)

#define IMU_STEER_KP      1.0f
#define IMU_STEER_KD      0.15f
#define IMU_STEER_KI      0.0f  // >0: removes steady drift to one side (try 0.2-0.5)
#define IMU_STEER_I_LIMIT 10.0f // max degrees the integral can add
#define IMU_STEER_LIMIT   20.0f // keeps the front US looking ahead
#define IMU_STEER_SIGN    1     // -1 if it steers the wrong way

#define IMU_STILL_RANGE 4.0f    // deg/s allowed during calibration

#define GYRO_SCALE  65.5f       // +/-500 deg/s
#define ACCEL_SCALE 16384.0f    // +/-2 g
#define POLL_MS   10
#define STALL_MS  600
#define RETRY_MS  200
#define MAX_FAILS 3             // failed reads in a row before "lost"
#define MAX_GAP_S 0.5f

static uint8_t  imu_address = 0x68;
static uint8_t  connected = 0;
static uint8_t  calibrated = 0;
static uint8_t  fails = 0;
static float    bias[3] = {0, 0, 0};    // gyro bias, deg/s
static float    up[3] = {0, 0, 1};      // gravity direction (unit vector)
static float    dt_sample = 0.01f;      // measured during calibration
static float    yaw = 0;
static float    yaw_rate = 0;
static float    target = 0;
static float    integral = 0;
static uint32_t last_poll_ms = 0;
static uint32_t last_sample_ms = 0;
static uint32_t last_retry_ms = 0;
static uint32_t last_steer_ms = 0;

static float Wrap180(float a) {
    if (!isfinite(a)) return 0;
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0) a += 360.0f;
    return a - 180.0f;
}

// ---- I2C ----

static void TWI_Setup(void) {
    PORTC &= ~((1 << PC4) | (1 << PC5));      // board has 3.3 V pull-ups
    TWSR = 0;
    TWBR = 12;                                // 400 kHz
    TWCR = (1 << TWEN);
}

static uint8_t TWI_Wait(void) {
    uint16_t tries = 0;
    while (!(TWCR & (1 << TWINT))) {
        if (++tries > 20000) return 0;
    }
    return 1;
}

static void TWI_Stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    uint16_t tries = 0;
    while (TWCR & (1 << TWSTO)) {
        if (++tries > 20000) { TWCR = 0; TWI_Setup(); break; }
    }
}

static uint8_t TWI_Start(uint8_t reading) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (!TWI_Wait()) return 0;
    if (TW_STATUS != TW_START && TW_STATUS != TW_REP_START) return 0;
    TWDR = (imu_address << 1) | reading;
    TWCR = (1 << TWINT) | (1 << TWEN);
    return TWI_Wait() && TW_STATUS == (reading ? TW_MR_SLA_ACK : TW_MT_SLA_ACK);
}

static uint8_t TWI_Write(uint8_t value) {
    TWDR = value;
    TWCR = (1 << TWINT) | (1 << TWEN);
    return TWI_Wait() && TW_STATUS == TW_MT_DATA_ACK;
}

static uint8_t Write_Reg(uint8_t reg, uint8_t value) {
    uint8_t ok = TWI_Start(0) && TWI_Write(reg) && TWI_Write(value);
    TWI_Stop();
    return ok;
}

static uint8_t Read_Regs(uint8_t reg, uint8_t *data, uint8_t count) {
    uint8_t ok = TWI_Start(0) && TWI_Write(reg) && TWI_Start(1);
    for (uint8_t n = 0; ok && n < count; n++) {
        uint8_t ack = n + 1 < count;
        TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
        ok = TWI_Wait() && TW_STATUS == (ack ? TW_MR_DATA_ACK : TW_MR_DATA_NACK);
        data[n] = TWDR;
    }
    TWI_Stop();
    return ok;
}

// ---- Sensor ----

static int16_t Word(const uint8_t *d) {
    return (int16_t)(((uint16_t)d[0] << 8) | d[1]);
}

// FIFO stores every gyro sample (X, Y, Z = 6 bytes) until it is read
static uint8_t FIFO_Restart(void) {
    return Write_Reg(0x23, 0x00) &&           // stop filling
           Write_Reg(0x6A, 0x04) &&           // reset FIFO
           Write_Reg(0x6A, 0x40) &&           // enable FIFO
           Write_Reg(0x23, 0x70);             // fill with gyro X, Y, Z
}

static uint8_t Wake_And_Configure(void) {
    for (uint8_t n = 0; n < 2; n++) {
        uint8_t who = 0;
        imu_address = 0x68 + n;
        if (Read_Regs(0x75, &who, 1) && who == 0x68) {
            return Write_Reg(0x6B, 0x01) &&   // wake, gyro clock
                   Write_Reg(0x6C, 0x00) &&
                   Write_Reg(0x1A, 3) &&      // low-pass filter
                   Write_Reg(0x19, 9) &&      // 100 Hz
                   Write_Reg(0x1B, 1 << 3) && // gyro +/-500 deg/s
                   Write_Reg(0x1C, 0) &&      // accel +/-2 g
                   FIFO_Restart();
        }
    }
    return 0;
}

// Number of complete samples waiting in the FIFO (0xFFFF = read failed)
static uint16_t FIFO_Samples(void) {
    uint8_t c[2];
    if (!Read_Regs(0x72, c, 2)) return 0xFFFF;
    uint16_t bytes = ((uint16_t)c[0] << 8) | c[1];
    if (bytes >= 1000) { FIFO_Restart(); return 0; }   // overflowed: start clean
    return bytes / 6;
}

// Read up to 'max' samples into rates[][3] (deg/s, bias not removed)
static uint8_t FIFO_Read(float rates[][3], uint8_t count) {
    uint8_t d[6];
    for (uint8_t s = 0; s < count; s++) {
        if (!Read_Regs(0x74, d, 6)) return s;
        for (uint8_t k = 0; k < 3; k++) rates[s][k] = Word(&d[k * 2]) / GYRO_SCALE;
    }
    return count;
}

// Turning speed around "up", whatever way the IMU is mounted
static float Yaw_Rate(const float *r) {
    return IMU_YAW_SIGN * ((r[0] - bias[0]) * up[0] +
                           (r[1] - bias[1]) * up[1] +
                           (r[2] - bias[2]) * up[2]);
}

static void Read_Failed(void) {
    if (++fails >= MAX_FAILS) connected = 0;
}

// ---- Public functions ----

uint8_t IMU_Init(void) {
    TWI_Setup();
    _delay_ms(50);
    connected = Wake_And_Configure();
    if (connected) _delay_ms(100);
    fails = 0;
    last_sample_ms = last_poll_ms = last_retry_ms = IMU_NOW_MS();
    return connected;
}

// ~2 s, still, fans off. Measures gyro bias, the "up" direction and the sample rate.
uint8_t IMU_Calibrate(void) {
    if (!connected && !IMU_Init()) return 0;
    if (!FIFO_Restart()) return 0;

    float g_sum[3] = {0, 0, 0}, a_sum[3] = {0, 0, 0};
    float low[3] = {1000, 1000, 1000}, high[3] = {-1000, -1000, -1000};
    uint16_t samples = 0, a_reads = 0;
    uint32_t start = IMU_NOW_MS();

    for (uint8_t loops = 0; samples < 200; loops++) {
        if (loops > 100) return 0;                    // no data after ~5 s
        _delay_ms(50);
        uint8_t a[6];
        if (!Read_Regs(0x3B, a, 6)) return 0;
        for (uint8_t k = 0; k < 3; k++) a_sum[k] += Word(&a[k * 2]) / ACCEL_SCALE;
        a_reads++;

        uint16_t n = FIFO_Samples();                  // use every sample, so the
        if (n == 0xFFFF) return 0;                    // timing below is exact
        while (n > 0) {
            float r[4][3];
            uint8_t chunk = n > 4 ? 4 : n;
            if (FIFO_Read(r, chunk) != chunk) return 0;
            for (uint8_t s = 0; s < chunk; s++) {
                for (uint8_t k = 0; k < 3; k++) {
                    g_sum[k] += r[s][k];
                    if (r[s][k] < low[k])  low[k] = r[s][k];
                    if (r[s][k] > high[k]) high[k] = r[s][k];
                }
            }
            samples += chunk;
            n -= chunk;
        }
    }

    for (uint8_t k = 0; k < 3; k++)
        if (high[k] - low[k] > IMU_STILL_RANGE) return 0;   // moved

    float mag = 0;
    for (uint8_t k = 0; k < 3; k++) mag += (a_sum[k] / a_reads) * (a_sum[k] / a_reads);
    mag = sqrtf(mag);
    if (mag < 0.5f || mag > 1.5f) return 0;           // not still / bad accel

    for (uint8_t k = 0; k < 3; k++) {
        bias[k] = g_sum[k] / samples;
        up[k] = (a_sum[k] / a_reads) / mag;
    }

    // Real sample period from the tick (falls back to 10 ms if it looks wrong)
    float measured = (IMU_NOW_MS() - start) / 1000.0f / samples;
    dt_sample = (measured > 0.008f && measured < 0.012f) ? measured : 0.01f;

    yaw = target = yaw_rate = integral = 0;
    calibrated = 1;
    fails = 0;
    FIFO_Restart();
    last_sample_ms = last_poll_ms = last_steer_ms = IMU_NOW_MS();
    return 1;
}

void IMU_Update(void) {
    uint32_t now = IMU_NOW_MS();

    // Lost: wake again, keep calibration, estimate the turning during the gap
    if (!connected) {
        if (now - last_retry_ms >= RETRY_MS) {
            last_retry_ms = now;
            TWCR = 0;
            TWI_Setup();
            if (Wake_And_Configure()) {
                float gap = (IMU_NOW_MS() - last_sample_ms) / 1000.0f;
                if (calibrated && gap < MAX_GAP_S) yaw = Wrap180(yaw + yaw_rate * gap);
                connected = 1;
                fails = 0;
                last_sample_ms = last_poll_ms = IMU_NOW_MS();
            }
        }
        return;
    }

    if (now - last_poll_ms < POLL_MS) return;
    last_poll_ms = now;

    uint16_t n = FIFO_Samples();
    if (n == 0xFFFF) { Read_Failed(); return; }

    if (n == 0) {
        if (now - last_sample_ms > STALL_MS) connected = 0;   // answers, no data
        return;
    }

    while (n > 0) {
        float r[4][3];
        uint8_t chunk = n > 4 ? 4 : n;
        uint8_t got = FIFO_Read(r, chunk);
        for (uint8_t s = 0; s < got; s++) {
            yaw_rate = Yaw_Rate(r[s]);
            if (calibrated) yaw = Wrap180(yaw + yaw_rate * dt_sample);
        }
        if (got != chunk) { Read_Failed(); return; }
        n -= chunk;
    }
    fails = 0;
    last_sample_ms = now;
}

float   IMU_Yaw(void)       { return yaw; }
float   IMU_Target(void)    { return target; }
float   IMU_YawRate(void)   { return yaw_rate; }
uint8_t IMU_Connected(void) { return connected; }

void IMU_SetTarget(float change) {
    target = Wrap180(yaw + change);
    integral = 0;
}

float IMU_HeadingError(void) {
    return Wrap180(target - yaw);
}

uint8_t IMU_TurnFinished(void) {
    float predicted = Wrap180(IMU_HeadingError() - yaw_rate * IMU_TURN_LEAD_S);
    return fabsf(predicted) < IMU_TURN_TOLERANCE;
}

float IMU_Steer(void) {
    uint32_t now = IMU_NOW_MS();
    float dt = (now - last_steer_ms) / 1000.0f;
    last_steer_ms = now;
    if (!connected || !calibrated) return 0;

    float error = IMU_HeadingError();

    // Integral only near the target, and restarted after a pause (e.g. a turn)
    if (dt > 0.2f || fabsf(error) > IMU_TURN_TOLERANCE) {
        integral = 0;
    } else if (IMU_STEER_KI > 0) {
        integral += IMU_STEER_KI * error * dt;
        if (integral > IMU_STEER_I_LIMIT)  integral = IMU_STEER_I_LIMIT;
        if (integral < -IMU_STEER_I_LIMIT) integral = -IMU_STEER_I_LIMIT;
    }

    float s = IMU_STEER_SIGN * (IMU_STEER_KP * error - IMU_STEER_KD * yaw_rate + integral);
    if (s > IMU_STEER_LIMIT)  s = IMU_STEER_LIMIT;
    if (s < -IMU_STEER_LIMIT) s = -IMU_STEER_LIMIT;
    return s;
}
