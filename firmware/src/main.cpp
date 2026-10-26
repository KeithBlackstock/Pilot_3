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
#include "attitude_level.hpp"
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

// Live-tunable leveler config; the dashboard finds it by symbol name.
volatile attitude_level::Config g_attitude_config = {
    attitude_level::kDefaults.pitch_kp,    attitude_level::kDefaults.roll_kp,
    attitude_level::kDefaults.pitch_kd,    attitude_level::kDefaults.roll_kd,
    attitude_level::kDefaults.pitch_sign,  attitude_level::kDefaults.roll_sign,
    attitude_level::kDefaults.max_pitch_deg, attitude_level::kDefaults.max_roll_deg,
    attitude_level::kDefaults.deadband_us, attitude_level::kDefaults.max_output_us,
    attitude_level::kDefaults.s1_sign,     attitude_level::kDefaults.s4_sign};

namespace {

constexpr uint32_t kSysClockHz = 8'000'000;
constexpr float kLoopHz = 200.0f;
constexpr uint32_t kLoopTicks = static_cast<uint32_t>(kSysClockHz / kLoopHz);
constexpr uint32_t kSysTickMask = 0x00FFFFFF;  // SysTick is a 24-bit down-counter

// A new BMP280 cycle is requested on every Nth iteration (~10 Hz); the read
// itself is non-blocking and spread over the following ticks (bmp280::step).
constexpr uint32_t kBaroEveryN = static_cast<uint32_t>(kLoopHz / 10.0f);

// iBus staleness: if no new valid frame arrives for this many loop
// iterations (>= 40 * 5 ms = 200 ms; longer when a baro read stretches an
// iteration, iBus normally sends a frame every ~7 ms), the RX is considered
// dropped and the motor is cut.
constexpr uint32_t kIbusStaleLoops = 40;

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

// Free-running SysTick (wraps every ~2.1 s) used to measure the real loop
// period. Only valid after delay_ms() is done with the one-shot mode.
void systick_start_free_running() {
    mmio(SYST_CSR) = 0;
    mmio(SYST_RVR) = kSysTickMask;
    mmio(SYST_CVR) = 0;
    mmio(SYST_CSR) = SYST_CSR_ENABLE | SYST_CSR_CLKSOURCE;
}

uint32_t systick_now() { return mmio(SYST_CVR) & kSysTickMask; }

void delay_ms(uint32_t ms) {
    constexpr uint32_t kOneMsReload = kSysClockHz / 1000 - 1;
    for (uint32_t i = 0; i < ms; ++i) {
        systick_wait_one_period(kOneMsReload);
    }
}

// Stick deflection in us around center. Invalid/uninitialized input (0 or
// out of [900, 2100]) counts as centered, matching the servo neutral failsafe.
int16_t stick_offset_us(uint16_t ch_us) {
    if (ch_us < 900 || ch_us > 2100) return 0;
    return static_cast<int16_t>(ch_us) - 1500;
}

// Center + offset, clamped to the 1000-2000 us servo range.
uint16_t mix_us(int16_t offset_us) {
    int16_t v = 1500 + offset_us;
    if (v < 1000) v = 1000;
    if (v > 2000) v = 2000;
    return static_cast<uint16_t>(v);
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
    systick_start_free_running();
    uint32_t last_tick = systick_now();
    uint32_t last_frames_ok = 0;
    uint32_t loops_since_frame = kIbusStaleLoops;  // start stale until a frame arrives

    while (true) {
        // Wait until at least one loop period has elapsed since the last
        // iteration; if a slow iteration (e.g. a baro read) overran, this
        // falls straight through. The filter gets the measured dt, so overruns
        // don't under-integrate the gyro. SysTick counts down.
        uint32_t now_tick;
        uint32_t elapsed;
        do {
            now_tick = systick_now();
            elapsed = (last_tick - now_tick) & kSysTickMask;
        } while (elapsed < kLoopTicks);
        last_tick = now_tick;
        float dt_s = static_cast<float>(elapsed) / static_cast<float>(kSysClockHz);

        icm::Sample s = icm::read();
        // Bench-verified: the raw X-roll axis reads right-wing-down as negative.
        // Negate it here (accel Y and gyro X) so roll is right-wing-down positive
        // everywhere downstream: filter, telemetry, and the leveler's D term.
        s.accel_g[1] = -s.accel_g[1];
        s.gyro_dps[0] = -s.gyro_dps[0];
        filter.update(s.accel_g[0], s.accel_g[1], s.accel_g[2],
                      s.gyro_dps[0], s.gyro_dps[1], dt_s);

        g_telemetry.roll_deg  = filter.roll_deg();
        g_telemetry.pitch_deg = filter.pitch_deg();

        bmp280::Reading r;
        if (baro_ok && bmp280::step((loop_count % kBaroEveryN) == 0, r)) {
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

        // RX staleness (no new valid iBus frame for kIbusStaleLoops ticks).
        if (ibus_stats.frames_ok != last_frames_ok) {
            last_frames_ok = ibus_stats.frames_ok;
            loops_since_frame = 0;
        } else if (loops_since_frame < kIbusStaleLoops) {
            loops_since_frame++;
        }
        bool rx_stale = loops_since_frame >= kIbusStaleLoops;

        // Ruddervator (V-tail) mix (ch3 = throttle). The servos are mounted
        // mirrored, so on the bench the *differential* term (S1 vs S4 opposite
        // in us) is pitch and the *common* term is roll/yaw. The variables are
        // named for their mix slot, not the surface: `elev` is the common term
        // fed by ch1 (index 0), `rud` is the differential term fed by ch2
        // (index 1), which is the pitch stick on this transmitter. Signs are
        // bench-verified; they are not interchangeable between S1 and S4.
        int16_t elev = stick_offset_us(ch[0]);
        int16_t rud  = stick_offset_us(ch[1]);

        // CH6 (Mode) > 1500 enables the angle loop (P on accel angle, D on
        // gyro rate): the elevator/rudder sticks become pitch/roll angle
        // requests (see attitude_level.hpp) and replace the direct
        // stick-to-surface path. A stale RX requests level. Invalid CH6 (0 / out of range) leaves
        // the sticks passing straight through.
        bool stabilize = ch[5] > 1500 && ch[5] <= 2100;
        if (stabilize) {
            attitude_level::Config lvl = attitude_level::snapshot();
            // Pitch is the differential slot (`rud`, ch2); roll is the common
            // slot (`elev`, ch1) -- see the mix comment above.
            float pitch_req = rx_stale ? 0.0f : static_cast<float>(rud);
            float roll_req  = rx_stale ? 0.0f : static_cast<float>(elev);
            rud  = static_cast<int16_t>(attitude_level::pitch(lvl, pitch_req, filter.pitch_deg(), s.gyro_dps[1]));
            elev = static_cast<int16_t>(attitude_level::roll(lvl, roll_req, filter.roll_deg(), s.gyro_dps[0]));
        }
        // Per-servo reverse (mirrored mounting), applied to the final mix so it
        // holds in both direct and stabilize mode.
        attitude_level::Config dir = attitude_level::snapshot();
        int16_t s1_off = static_cast<int16_t>(-elev + rud);
        int16_t s4_off = static_cast<int16_t>(-elev - rud);
        if (dir.s1_sign < 0.0f) s1_off = -s1_off;
        if (dir.s4_sign < 0.0f) s4_off = -s4_off;
        g_telemetry.pwm_s1_us = servo_pwm::set_s1_us(mix_us(s1_off));
        g_telemetry.pwm_s4_us = servo_pwm::set_s4_us(mix_us(s4_off));
        // Throttle = iBus channel 3 (index 2) -> ESC on PWM_S2. Cut to off
        // if the RX has gone quiet; servos keep their last position.
        servo_pwm::set_s2_motor_us(rx_stale ? 0 : ch[2]);

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
