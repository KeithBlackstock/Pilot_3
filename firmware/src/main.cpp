// Pilot_3 lightweight attitude firmware -- FC_bluepill (GD32F303CCT6).
//
// Reads the ICM-42688-P over SPI2 at a fixed 200 Hz loop rate (SysTick
// busy-wait, no interrupts, no EXTI/AFIO/GPIOA touched at all -- see
// regs.hpp), fuses accel+gyro into roll/pitch with a complementary filter,
// and publishes the result into a fixed SRAM address for
// tools/pilot3_attitude_monitor.py to poll over OpenOCD telnet.
//
// Assumes post-reset clocks (HSI 8 MHz, PCLK1 = 8 MHz), same as
// pilot3_swd_probe.py -- no RCC_CFGR changes.
#include "bmp280.hpp"
#include "complementary_filter.hpp"
#include "gps.hpp"
#include "ibus.hpp"
#include "icm42688.hpp"
#include "regs.hpp"
#include "servo_pwm.hpp"
#include "telemetry.hpp"

using namespace reg;

volatile Telemetry g_telemetry __attribute__((section(".telemetry"))) = {};

namespace {

constexpr uint32_t kSysClockHz = 8'000'000;
constexpr float kLoopHz = 200.0f;
constexpr uint32_t kLoopReload =
    static_cast<uint32_t>(kSysClockHz / kLoopHz) - 1;
constexpr float kLoopDtS = 1.0f / kLoopHz;

// BMP280 read blocks for a few ms (forced-mode conversion) -- too slow to
// do every 5 ms tick, so it only runs on every Nth iteration (~10 Hz).
constexpr uint32_t kBaroEveryN = static_cast<uint32_t>(kLoopHz / 10.0f);

// Busy-wait one SysTick period. No TICKINT/NVIC involved -- just polls
// COUNTFLAG, which is cleared by hardware on read.
void systick_wait_one_period(uint32_t reload) {
    mmio(SYST_RVR) = reload;
    mmio(SYST_CVR) = 0;
    mmio(SYST_CSR) = SYST_CSR_ENABLE | SYST_CSR_CLKSOURCE;
    while (!(mmio(SYST_CSR) & SYST_CSR_COUNTFLAG)) {
    }
    mmio(SYST_CSR) = 0;
}

void delay_ms(uint32_t ms) {
    constexpr uint32_t kOneMsReload = kSysClockHz / 1000 - 1;
    for (uint32_t i = 0; i < ms; ++i) {
        systick_wait_one_period(kOneMsReload);
    }
}

}  // namespace

int main() {
    // ICM-42688-P needs some settle time after PWR_MGMT0 enables the
    // sensors before accel/gyro data is valid (datasheet: tens of ms).
    if (!icm::init()) {
        // WHO_AM_I mismatch -- nothing sane to report without the sensor,
        // so just idle. seq stays 0, which the host script treats as
        // "never updated".
        while (true) {
        }
    }
    // Starts the USART3 RXNE interrupt -- from here on, IBUS bytes are
    // consumed asynchronously regardless of what the rest of init() does.
    ibus::init();

    // Same for the GPS on USART1: UBX NAV-PVT frames are assembled in the
    // ISR and decoded later in the main loop via gps::poll().
    gps::init();

    // Servos go to neutral immediately and hold there until real channel
    // data starts arriving (see servo_pwm's clamp_or_neutral()).
    servo_pwm::init();

    delay_ms(50);

    // Baro is best-effort: if it's not there or fails WHO_AM_I, the IMU
    // attitude readout still works, just with baro_* left at zero.
    bool baro_ok = bmp280::init();

    ComplementaryFilter filter;
    uint32_t loop_count = 0;

    while (true) {
        systick_wait_one_period(kLoopReload);

        icm::Sample s = icm::read();
        filter.update(s.accel_g[0], s.accel_g[1], s.accel_g[2],
                      s.gyro_dps[0], s.gyro_dps[1], kLoopDtS);

        g_telemetry.roll_deg  = filter.roll_deg();
        g_telemetry.pitch_deg = filter.pitch_deg();

        if (baro_ok && (loop_count % kBaroEveryN) == 0) {
            bmp280::Reading r    = bmp280::read();
            g_telemetry.baro_temp_c      = r.temperature_c;
            g_telemetry.baro_pressure_pa = r.pressure_pa;
            g_telemetry.baro_altitude_m  = r.altitude_m;
        }

        ibus::Stats ibus_stats = ibus::snapshot();
        g_telemetry.ibus_rx_bytes   = ibus_stats.rx_bytes;
        g_telemetry.ibus_frames_ok  = ibus_stats.frames_ok;
        g_telemetry.ibus_frames_bad = ibus_stats.frames_bad;

        uint16_t ch[ibus::kNumChannels];
        ibus::read_channels(ch);
        for (int i = 0; i < ibus::kNumChannels; ++i) g_telemetry.ibus_ch[i] = ch[i];

        g_telemetry.pwm_s1_us = servo_pwm::set_s1_us(ch[0]);
        g_telemetry.pwm_s4_us = servo_pwm::set_s4_us(ch[1]);

        gps::poll();
        gps::Stats gps_stats = gps::snapshot();
        g_telemetry.gps_rx_bytes       = gps_stats.rx_bytes;
        g_telemetry.gps_sentences_ok   = gps_stats.sentences_ok;
        g_telemetry.gps_sentences_bad  = gps_stats.sentences_bad;
        gps::Fix gf = gps::read_fix();
        g_telemetry.gps_lat_e7      = gf.lat_e7;
        g_telemetry.gps_lon_e7      = gf.lon_e7;
        g_telemetry.gps_alt_m       = gf.alt_m;
        g_telemetry.gps_speed_mps   = gf.speed_mps;
        g_telemetry.gps_course_deg  = gf.course_deg;
        g_telemetry.gps_sats        = gf.sats;
        g_telemetry.gps_fix         = gf.fix;

        g_telemetry.seq = g_telemetry.seq + 1;
        loop_count++;
    }
}
