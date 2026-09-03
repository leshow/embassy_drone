use defmt::info;
use embassy_dshot::{Command, DshotPioAsync, DshotSpeed, rp::DshotPio};
use embassy_rp::{Peri, peripherals::PIO0};
use embassy_time::Duration;

use crate::{Irqs, MotorFl, MotorFr, MotorRl, MotorRr};

// DShot throttle range is 0-1999 (embassy-dshot's own 0-based abstraction over the raw
// 48-2047 DShot wire values). THROTTLE_MIN is already the lowest valid *spin* value, not
// "off" - the motor may still creep at it. Command::MotorStop (see turn_off) is the real "off"
// ~5% idle floor, matching betaflight's dshot_idle_value. set_motors rescales [0,1] onto
// [MIN,MAX] rather than clamping, so the mixer keeps all its relative authority - it just can't
// command a literal stop while armed.
const THROTTLE_MIN: u16 = 100;
const THROTTLE_MAX: u16 = 1999;

pub struct Motors<'a> {
    dshot: DshotPio<'a, 4, PIO0>,
}

impl<'a> Motors<'a> {
    // order is fl, fr, rl, rr throughout. ARMS during
    // init (2s of MotorStop)
    pub async fn init(
        pio: Peri<'static, PIO0>,
        pin_10: Peri<'static, MotorFl>,
        pin_11: Peri<'static, MotorFr>,
        pin_12: Peri<'static, MotorRl>,
        pin_13: Peri<'static, MotorRr>,
    ) -> Self {
        let mut dshot = DshotPio::<4, PIO0>::new(
            pio,
            Irqs,
            pin_10,
            pin_11,
            pin_12,
            pin_13,
            DshotSpeed::DShot300,
        );

        info!("arming ESCs");
        dshot.arm_async(Duration::from_secs(2)).await;
        info!("motors initialized");

        Self { dshot }
    }

    // fl/fr/rl/rr each in [0.0, 1.0] (mixer output) -> returns the actual DShot values sent,
    // for logging. throttle_async can only fail on an out-of-0-1999 value, which the clamp
    // below guarantees can't happen, so the unwrap is safe
    pub async fn set_motors(&mut self, fl: f32, fr: f32, rl: f32, rr: f32) -> [u16; 4] {
        let scale = |v: f32| {
            (v.clamp(0.0, 1.0) * (THROTTLE_MAX - THROTTLE_MIN) as f32) as u16 + THROTTLE_MIN
        };
        let values = [scale(fl), scale(fr), scale(rl), scale(rr)];
        self.dshot.throttle_async(values).await.unwrap();
        values
    }

    /// Sends Command::MotorStop once, unlike PWM, the caller's
    /// loop must keep calling this (or set_motors) every tick while disarmed or the ESC will fail-safe
    /// BLHeli_S's signal-loss failsafe is ~320ms steady-state, tighter (~40-50ms)
    /// during the first few seconds after arm/spool-up
    pub async fn turn_off(&mut self) {
        self.dshot.send_command_async(Command::MotorStop).await;
    }
}
