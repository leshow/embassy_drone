#![no_std]
#![no_main]

use crate::radio::Controls;
use defmt::{debug, error, info};
use embassy_executor::Spawner;
use embassy_rp::{
    Peri, bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    peripherals::{PIN_6, PIO0},
    spi::{Config as SpiConfig, Spi},
    uart::InterruptHandler as UartInterruptHandler,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

use {defmt_rtt as _, panic_probe as _};

use libs::flight::{
    fusion::{FusionBuilder, RAD_TO_DEG},
    pid::Pid,
    sensors::ImuRead,
};

mod imu;
mod motors;
mod radio;

use motors::Motors;

// physical wiring
pub type MotorFl = embassy_rp::peripherals::PIN_10;
pub type MotorFr = embassy_rp::peripherals::PIN_11;
pub type MotorRl = embassy_rp::peripherals::PIN_12;
pub type MotorRr = embassy_rp::peripherals::PIN_13;

pub type RadioUart = embassy_rp::peripherals::UART0;
pub type RadioDma = embassy_rp::peripherals::DMA_CH0;
pub type ImuTxDma = embassy_rp::peripherals::DMA_CH1;
pub type ImuRxDma = embassy_rp::peripherals::DMA_CH2;

pub type CtrlRx<'a> = watch::Receiver<'a, CriticalSectionRawMutex, (Controls, Instant), 1>;
pub type CtrlTx<'a> = watch::Sender<'a, CriticalSectionRawMutex, (Controls, Instant), 1>;

bind_interrupts!(struct Irqs {
    UART0_IRQ => UartInterruptHandler<RadioUart>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<RadioDma>, embassy_rp::dma::InterruptHandler<ImuTxDma>, embassy_rp::dma::InterruptHandler<ImuRxDma>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
});

/// How many loop iterations to skip between log lines.
/// Override at build time: `LOG_RATE=200 cargo flash-s3` (default: 500).
const LOG_EVERY_N: u32 = {
    let every = match option_env!("LOG_RATE") {
        Some(s) => libs::flight::parse_u64(s),
        None => 500,
    };
    every as u32
};

/// cap on throttle for testing
const THROTTLE_CAP: u8 = {
    match option_env!("THROTTLE_CAP") {
        Some(s) => {
            let v = libs::flight::parse_u64(s);
            assert!(v <= 100, "THROTTLE_CAP must be 0..=100");
            v as u8
        }
        None => 100, // no cap default
    }
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    spawner.spawn(radio::read_radio(p.UART0, p.PIN_1, p.DMA_CH0).unwrap());

    #[cfg(feature = "test_dshot")]
    test_dshot(p.PIO0, p.PIN_10, p.PIN_11, p.PIN_12, p.PIN_13).await;

    #[cfg(feature = "visualize")]
    {
        // 3MHz, under ICM42688's 24MHz SPI ceiling

        use crate::imu::Sensor;
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = 3_000_000;
        let spi_bus = Spi::new(
            p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, p.DMA_CH1, p.DMA_CH2, Irqs, spi_config,
        );
        let cs = Output::new(p.PIN_5, Level::High);
        let device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();

        // initialize() soft-resets the sensor and checks WHO_AM_I internally (expects 0x47)
        match Sensor::init(device).await {
            Ok(icm) => visualize(icm, p.PIN_6).await,
            Err(e) => error!("ICM42688 init failed: {}", e),
        }
    }

    // normal run scenario
    #[cfg(not(any(feature = "visualize", feature = "test_dshot")))]
    {
        use crate::{imu::Sensor, radio::CONTROLS};
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = 3_000_000;
        let spi_bus = Spi::new(
            p.SPI0, p.PIN_2, p.PIN_3, p.PIN_4, p.DMA_CH1, p.DMA_CH2, Irqs, spi_config,
        );
        let cs = Output::new(p.PIN_5, Level::High);
        let device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();
        let rx = CONTROLS.receiver().expect("failed to create receiver");

        // initialize() soft-resets the sensor and checks WHO_AM_I internally (expects 0x47)
        let sensor = match Sensor::init(device).await {
            Ok(icm) => icm,
            Err(e) => {
                error!("ICM42688 init failed: {}", e);
                // TODO: should this panic?
                return;
            }
        };
        let motors = Motors::init(p.PIO0, p.PIN_10, p.PIN_11, p.PIN_12, p.PIN_13).await;
        run(sensor, motors, p.PIN_6, rx).await;
    }
    loop {
        Timer::after_secs(60).await;
    }
}

mod consts {
    use libs::flight::fusion;

    // max roll/pitch command from stick (+/- 25 deg)
    pub const MAX_TILT_RAD: f32 = 25.0 * fusion::DEG_TO_RAD;

    // outer loop p gains
    pub const ANGLE_P_ROLL_PITCH: f32 = 4.0;
    // and angle_p_yaw means the controller will eventually torque
    // quad to chase drift
    pub const ANGLE_P_YAW: f32 = 0.0;

    // inner loop - only P is live for the first test. D and I come in one at a time, in that
    // order, once the previous term is confirmed stable - see
    // [[project_brushless_rp2350_bringup]] for why: isolating one term at a time means a bad
    // first result points at something structural (gyro axis, motor mapping, mixer sign)
    // rather than being entangled with an untested D or I term. INTEGRAL_LIMIT/LEAK below stay
    // inert while their Ki is 0 - both only ever shape the accumulator's value, and Ki=0
    // zeroes that value's contribution to the output regardless
    pub const RATE_KP_ROLL_PITCH: f32 = 0.05;
    pub const RATE_KI_ROLL_PITCH: f32 = 0.0; // was 0.3
    pub const RATE_KD_ROLL_PITCH: f32 = 0.0; // was 0.001

    pub const RATE_KP_YAW: f32 = 0.3; // flix 0.3 was set to 0.2
    pub const RATE_KI_YAW: f32 = 0.0; // was 0.05
    pub const RATE_KD_YAW: f32 = 0.0; // was 0.001

    pub const INTEGRAL_LIMIT: f32 = 0.3;
    // per-second decay on the rate pids' integral term - without this, any permanent small error
    // eventually winds the integral to integral_limit given enough time, no matter how small the error is. with
    // decay, a persistent error settles at an equilibrium of error/rate_integral_leak instead,
    // while a genuinely larger disturbance still gets proportionally more integral authority
    pub const RATE_INTEGRAL_LEAK: f32 = 1.0;

    // outer loop angle error deadband - a resting tilt under this is mounting tolerance subtracts the band rather than clamping to it, so there's
    // no discontinuity in commanded rate right at the edge. roll/pitch only: yaw can't wind up
    // (rate_ki_yaw = 0) and has its own heading-hold handling already
    pub const ANGLE_DEADBAND_RAD: f32 = 0.5 * fusion::DEG_TO_RAD;

    pub(crate) fn deadband(x: f32, band: f32) -> f32 {
        if x > band {
            x - band
        } else if x < -band {
            x + band
        } else {
            0.0
        }
    }

    // normalises an angle to [-pi, pi] for the shortest-path yaw error
    pub(crate) fn wrap_angle(a: f32) -> f32 {
        use core::f32::consts::PI;
        // fmodf gives the same sign as the dividend, so shift into [0, 2pi) before subtracting back
        let r = libm::fmodf(a + PI, 2.0 * PI);
        (if r < 0.0 { r + 2.0 * PI } else { r }) - PI
    }
}

// running min/max/avg of loop dt between periodic log lines - lets the actual loop rate be
// checked without needing per-tick logging
struct DtStats {
    min: f32,
    max: f32,
    sum: f32,
    n: u32,
}

impl DtStats {
    const fn new() -> Self {
        Self {
            min: f32::MAX,
            max: 0.0,
            sum: 0.0,
            n: 0,
        }
    }

    fn record(&mut self, dt: f32) {
        self.min = self.min.min(dt);
        self.max = self.max.max(dt);
        self.sum += dt;
        self.n += 1;
    }

    fn avg(&self) -> f32 {
        self.sum / self.n as f32
    }
}

async fn run<'a, D>(
    mut sensor: crate::imu::Sensor<icm426xx::ICM42688<D, icm426xx::Ready>>,
    mut motors: Motors<'a>,
    int1: Peri<'static, PIN_6>,
    mut rx: CtrlRx<'a>,
) where
    D: embedded_hal_async::spi::SpiDevice,
    D::Error: defmt::Format,
{
    let mut log_count: u32 = 0;
    let mut int1 = Input::new(int1, Pull::None);
    let mut fusion = FusionBuilder::new().icm42688().madgwick().build();
    let mut last = Instant::now();
    let mut dt_stats = DtStats::new();

    // inner rate PIDs
    let mut roll_pid = Pid::new(
        consts::RATE_KP_ROLL_PITCH,
        consts::RATE_KI_ROLL_PITCH,
        consts::RATE_KD_ROLL_PITCH,
        consts::INTEGRAL_LIMIT,
        consts::RATE_INTEGRAL_LEAK,
    );
    let mut pitch_pid = Pid::new(
        consts::RATE_KP_ROLL_PITCH,
        consts::RATE_KI_ROLL_PITCH,
        consts::RATE_KD_ROLL_PITCH,
        consts::INTEGRAL_LIMIT,
        consts::RATE_INTEGRAL_LEAK,
    );
    let mut yaw_pid = Pid::new(
        consts::RATE_KP_YAW,
        consts::RATE_KI_YAW,
        consts::RATE_KD_YAW,
        consts::INTEGRAL_LIMIT,
        consts::RATE_INTEGRAL_LEAK,
    );

    let mut target_yaw: f32 = 0.0;
    let mut yaw_init = false;
    let mut last_armed = false;

    loop {
        int1.wait_for_high().await;

        // no LPF or gyro-bias tracking yet (unlike the esp32-s3 project's read_fusion) - raw
        // gyro straight into both the fusion filter and the rate PIDs below. Fine to start
        // with; revisit if vibration/drift actually shows up as a real problem
        let (accel, gyro) = match sensor.read().await {
            Ok(sample) => sample,
            Err(e) => {
                error!("ICM42688 read_sample failed: {}", e);
                continue;
            }
        };

        let now = Instant::now();
        let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
        last = now;
        dt_stats.record(dt);

        let quat = fusion.update(dt, accel, gyro);
        let (actual_roll, actual_pitch, actual_yaw) = quat.euler_angles();

        log_count += 1;
        if log_count >= LOG_EVERY_N {
            log_count = 0;
            debug!(
                "loop dt min: {} max: {} avg: {} | roll: {}\u{b0} pitch: {}\u{b0} yaw: {}\u{b0}",
                dt_stats.min,
                dt_stats.max,
                dt_stats.avg(),
                actual_roll * RAD_TO_DEG,
                actual_pitch * RAD_TO_DEG,
                actual_yaw * RAD_TO_DEG,
            );
            dt_stats = DtStats::new();
        }

        // latest control input, with a staleness check for failsafe (no packet in the last
        // 500ms - stale receiver link, not just "sticks haven't moved" - reads the same
        // steady-state value every tick otherwise)
        let controls = rx.try_get();
        let fresh = controls.is_some_and(|(_, at)| at.elapsed() < Duration::from_millis(500));
        let armed = fresh && controls.is_some_and(|(c, _)| c.armed);

        if !last_armed && armed {
            info!("ARMED");
            roll_pid.reset();
            pitch_pid.reset();
            yaw_pid.reset();
            yaw_init = false;
        } else if last_armed && !armed {
            info!("DISARMED");
        }
        last_armed = armed;

        // fusion tracking above always runs regardless of arm state, so there's no cold-start
        // lag in the attitude estimate the moment it does arm
        let Some((ctrl, _)) = controls.filter(|_| armed) else {
            motors.turn_off().await;
            continue;
        };

        if !yaw_init {
            target_yaw = actual_yaw;
            yaw_init = true;
        }
        // near the ground, keep target tracking actual so handling the drone doesn't build a
        // large yaw error
        if ctrl.throttle < 0.15 {
            target_yaw = actual_yaw;
        }
        // heading hold: only update target when yaw stick is actually being pushed
        if ctrl.yaw.abs() >= 0.1 {
            target_yaw = actual_yaw;
        }
        // stick feedforward adds directly to yaw rate setpoint
        let yaw_ff = if ctrl.yaw.abs() >= 0.1 {
            -ctrl.yaw * core::f32::consts::PI // +/- pi rad/s
        } else {
            0.0
        };

        if ctrl.throttle < 0.05 {
            motors.turn_off().await;
            continue;
        }

        // outer angle-P loop: stick angle -> rate setpoint
        let target_roll = ctrl.roll * consts::MAX_TILT_RAD;
        let target_pitch = ctrl.pitch * consts::MAX_TILT_RAD;
        let roll_err = consts::deadband(
            consts::wrap_angle(target_roll - actual_roll),
            consts::ANGLE_DEADBAND_RAD,
        );
        let pitch_err = consts::deadband(
            consts::wrap_angle(target_pitch - actual_pitch),
            consts::ANGLE_DEADBAND_RAD,
        );
        let roll_rate_sp = consts::ANGLE_P_ROLL_PITCH * roll_err;
        let pitch_rate_sp = consts::ANGLE_P_ROLL_PITCH * pitch_err;
        let yaw_rate_sp =
            consts::ANGLE_P_YAW * consts::wrap_angle(target_yaw - actual_yaw) + yaw_ff;

        // inner rate PID: rate setpoint vs actual gyro rate -> torque
        let roll_torque = roll_pid.update(roll_rate_sp - gyro.x, dt);
        let pitch_torque = pitch_pid.update(pitch_rate_sp - gyro.y, dt);
        let yaw_torque = yaw_pid.update(yaw_rate_sp - gyro.z, dt);

        // THROTTLE_CAP applied to the commanded throttle here, before mixing - matches
        // Betaflight's throttle_limit_type=SCALE (scales the throttle the PID/mixer sees, not
        // the already-mixed output), so a lower cap doesn't also proportionally weaken PID
        // correction authority the way capping Motors::set_motors' output would
        let capped_throttle = ctrl.throttle * (THROTTLE_CAP as f32 / 100.0);
        let (fl, fr, rl, rr) =
            libs::mixer::mix_motors(capped_throttle, roll_torque, pitch_torque, yaw_torque);
        let duty = motors.set_motors(fl, fr, rl, rr).await;

        // log_count was already reset to 0 by the dt_stats print above if this tick hit
        // LOG_EVERY_N - piggyback on that same cadence rather than tracking a separate
        // counter. This print is only reached once armed, fresh, and throttle above the idle
        // cutoff, unlike the dt_stats one above, which runs every tick regardless of arm state
        if log_count == 0 {
            debug!(
                "torques roll: {} pitch: {} yaw: {} | mix fl: {} fr: {} rl: {} rr: {} | duty fl: {} fr: {} rl: {} rr: {}",
                roll_torque,
                pitch_torque,
                yaw_torque,
                fl,
                fr,
                rl,
                rr,
                duty[0],
                duty[1],
                duty[2],
                duty[3],
            );
        }
    }
}

// live fusion dump
// it can be piped into `visualizer` the same way the esp32-s3 project does:
// `cargo flash-probe --features visualize | (cd ../visualizer && cargo run)`
//
// INT1 (GPIO6) drives sampling off the data-ready pulse - ICM42688 config default is 200Hz
// ODR, latched/active-high, and read_sample()'s first SPI byte reads INT_STATUS which clears
// the latch
#[cfg(feature = "visualize")]
async fn visualize<D>(
    mut imu: crate::imu::Sensor<icm426xx::ICM42688<D, icm426xx::Ready>>,
    int1: Peri<'static, PIN_6>,
) where
    D: embedded_hal_async::spi::SpiDevice,
    D::Error: defmt::Format,
{
    let mut int1 = Input::new(int1, Pull::None);
    let mut fusion = FusionBuilder::new().icm42688().madgwick().build();
    let mut last = Instant::now();

    loop {
        // TODO: can try without wait_for_high if there are issues
        int1.wait_for_high().await;
        match imu.read().await {
            Ok((accel, gyro)) => {
                let now = Instant::now();
                let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
                last = now;

                let quat = fusion.update(dt, accel, gyro);
                let (roll, pitch, yaw) = quat.euler_angles();
                info!(
                    "roll: {}\u{b0} pitch: {}\u{b0} yaw: {}\u{b0}",
                    roll * RAD_TO_DEG,
                    pitch * RAD_TO_DEG,
                    yaw * RAD_TO_DEG
                );
            }
            Err(e) => error!("ICM42688 read_sample failed: {}", e),
        }
    }
}

// same GPIO10-13 wiring as test_oneshot, but driven through the real Motors abstraction
// (motors.rs) instead of raw PWM_SLICE5/6 - each motor gets its own independent PIO state
// machine. Motors::init already arms; unlike PWM (a free-running hardware counter that keeps
// outputting once configured), DShot frames are one-shot pushes to the PIO FIFO - nothing
// repeats on its own, so every phase (including the hold) has to keep calling set_motors at a
// steady rate or the ESCs will fail-safe
#[cfg(feature = "test_dshot")]
async fn test_dshot(
    pio: Peri<'static, PIO0>,
    fl: Peri<'static, MotorFl>,
    fr: Peri<'static, MotorFr>,
    rl: Peri<'static, MotorRl>,
    rr: Peri<'static, MotorRr>,
) {
    // Motors::set_motors takes normalized [0.0, 1.0], not raw DShot 0-1999 values
    const TARGET_THROTTLE: f32 = 0.2; // ~20%, same bench-spin level as the Oneshot125 test
    const RAMP_STEPS: u32 = 100;
    const HOLD_ITERATIONS: u32 = 100; // 100 * 20ms = 2s
    const STEP_DELAY_MS: u64 = 20;

    let mut motors = Motors::init(pio, fl, fr, rl, rr).await;

    // straight to target instead of ramping - diagnostic for whether the staggered start seen
    // during the ramp is just different motors crossing their own spin-up threshold at
    // different points along it, rather than a real per-channel timing difference
    info!("jumping straight to bench spin, no ramp");
    motors
        .set_motors(
            TARGET_THROTTLE,
            TARGET_THROTTLE,
            TARGET_THROTTLE,
            TARGET_THROTTLE,
        )
        .await;

    info!("holding bench spin");
    for _ in 0..HOLD_ITERATIONS {
        motors
            .set_motors(
                TARGET_THROTTLE,
                TARGET_THROTTLE,
                TARGET_THROTTLE,
                TARGET_THROTTLE,
            )
            .await;
        Timer::after_millis(STEP_DELAY_MS).await;
    }

    info!("ramping down to idle");
    for i in (0..=RAMP_STEPS).rev() {
        let t = TARGET_THROTTLE * (i as f32 / RAMP_STEPS as f32);
        motors.set_motors(t, t, t, t).await;
        Timer::after_millis(STEP_DELAY_MS).await;
    }

    motors.turn_off().await;
    info!("done, motors stopped");
}

// dead oneshot125 code. dshot is better :)
// #[cfg(feature = "test_oneshot")]
// {
//     let mut config = PwmConfig::default();
//     // RP2350 default sys clock is 150MHz - divider 150 makes 1 tick = 1us,
//     // top 299 makes one period 300us (~3.3kHz refresh) under
//     // Oneshot125's ~4kHz ceiling
//     config.divider = 150u8.into();
//     config.top = 299;
//     config.compare_a = ARM_US;
//     config.compare_b = ARM_US;

//     test_oneshot(
//         p.PWM_SLICE5,
//         p.PIN_10,
//         p.PIN_11,
//         p.PWM_SLICE6,
//         p.PIN_12,
//         p.PIN_13,
//         config,
//     )
//     .await;
// }
// Oneshot125 ESC convention: much faster refresh than standard PWM, pulse width in microseconds
// const ARM_US: u16 = 125;
// const BENCH_SPIN_US: u16 = 150; // 20% throttle: 125 + 0.20 * (250 - 125)
// min + (percent * (max - min))

// async fn test_oneshot(
//     slice_a: Peri<'static, PWM_SLICE5>,
//     pin_10: Peri<'static, MotorFl>,
//     pin_11: Peri<'static, MotorFr>,
//     slice_b: Peri<'static, PWM_SLICE6>,
//     pin_12: Peri<'static, MotorRl>,
//     pin_13: Peri<'static, MotorRr>,
//     mut config: PwmConfig,
// ) {
//     // gpio 10/11 on pwm_slice5
//     let mut esc_fl_fr = Pwm::new_output_ab(slice_a, pin_10, pin_11, config.clone());
//     // gpio 12/13 are on pwm slice 6 - PIN_12 is hardware-fixed as channel A, PIN_13 as
//     // channel B (must be passed in that order), so compare_a controls GPIO12 (RL) and
//     // compare_b controls GPIO13 (RR)
//     let mut esc_rl_rr = Pwm::new_output_ab(slice_b, pin_12, pin_13, config.clone());

//     info!("arming ESCs, holding min throttle");
//     Timer::after_secs(2).await;

//     // straight to target instead of ramping - PWM's a free-running hardware counter, so unlike
//     // DShot there's no failsafe-timeout concern either way; this matches what the direct-jump
//     // DShot test already confirmed: the staggered start seen during a slow ramp was just
//     // different motors crossing their own spin-up threshold at different points along it
//     info!("jumping straight to bench spin, no ramp");
//     config.compare_a = BENCH_SPIN_US;
//     config.compare_b = BENCH_SPIN_US;
//     esc_fl_fr.set_config(&config);
//     esc_rl_rr.set_config(&config);

//     info!("holding bench spin");
//     Timer::after_secs(2).await;

//     info!("ramping down to idle");
//     for us in (ARM_US..=BENCH_SPIN_US).rev() {
//         config.compare_a = us;
//         config.compare_b = us;
//         esc_fl_fr.set_config(&config);
//         esc_rl_rr.set_config(&config);
//         Timer::after_millis(100).await;
//     }

//     config.compare_a = ARM_US;
//     config.compare_b = ARM_US;
//     esc_fl_fr.set_config(&config);
//     esc_rl_rr.set_config(&config);
//     info!("done, idling at min throttle");
// }
