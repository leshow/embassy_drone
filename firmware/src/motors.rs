use esp_backtrace as _;
use esp_hal::{
    gpio::interconnect::PeripheralOutput,
    ledc::{
        Ledc, LowSpeed,
        channel::{self, ChannelHW, ChannelIFace},
        timer::{self},
    },
};
use esp_println as _;

pub struct Motors<'a> {
    fl: esp_hal::ledc::channel::Channel<'a, LowSpeed>,
    fr: esp_hal::ledc::channel::Channel<'a, LowSpeed>,
    rl: esp_hal::ledc::channel::Channel<'a, LowSpeed>,
    rr: esp_hal::ledc::channel::Channel<'a, LowSpeed>,
}

impl<'a> Motors<'a> {
    pub async fn init_pwm<
        RL: PeripheralOutput<'a>,
        RR: PeripheralOutput<'a>,
        FL: PeripheralOutput<'a>,
        FR: PeripheralOutput<'a>,
    >(
        ledc: &Ledc<'a>,
        timer: &'a timer::Timer<'a, LowSpeed>,
        duty_pct: u8,
        rear_left_pin: RL,
        rear_right_pin: RR,
        front_left_pin: FL,
        front_right_pin: FR,
    ) -> Self {
        let mut rl = ledc.channel(channel::Number::Channel0, rear_left_pin);
        rl.configure(channel::config::Config {
            timer,
            duty_pct,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("rear left motor pwm init failed");

        let mut rr = ledc.channel(channel::Number::Channel1, rear_right_pin);
        rr.configure(channel::config::Config {
            timer,
            duty_pct,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("rear right motor pwm init failed");

        let mut fl = ledc.channel(channel::Number::Channel2, front_left_pin);
        fl.configure(channel::config::Config {
            timer,
            duty_pct,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("front left motor pwm init failed");

        let mut fr = ledc.channel(channel::Number::Channel3, front_right_pin);
        fr.configure(channel::config::Config {
            timer,
            duty_pct,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("front right motor pwm init failed");

        #[cfg(feature = "test_motors")]
        {
            use embassy_time::Timer;
            let test_spin = 50;
            defmt::info!("testing front left");
            fl.start_duty_fade(0, test_spin, 3_000)
                .expect("failed to set duty");
            Timer::after_secs(4).await;
            fl.set_duty(0).expect("failed to set duty");

            defmt::info!("testing front right");
            fr.start_duty_fade(0, test_spin, 3_000)
                .expect("failed to set duty");
            Timer::after_secs(4).await;
            fr.set_duty(0).expect("failed to set duty");

            defmt::info!("testing rear left");
            rl.start_duty_fade(0, test_spin, 3_000)
                .expect("failed to set duty");
            Timer::after_secs(4).await;
            rl.set_duty(0).expect("failed to set duty");

            defmt::info!("testing rear right");
            rr.start_duty_fade(0, test_spin, 3_000)
                .expect("failed to set duty");
            Timer::after_secs(4).await;
            rr.set_duty(0).expect("failed to set duty");

            defmt::info!("motors initialized: all channels {}%", test_spin);
        }
        defmt::info!("pwm initialized: all channels {}%", duty_pct);

        Self { fl, fr, rl, rr }
    }

    #[allow(dead_code)]
    pub fn set_all_duty(&self, duty: u32) {
        self.fl.set_duty_hw(duty);
        self.fr.set_duty_hw(duty);
        self.rl.set_duty_hw(duty);
        self.rr.set_duty_hw(duty);
    }

    pub fn turn_off(&self) {
        self.fl.set_duty(0).expect("failed to set duty");
        self.fr.set_duty(0).expect("failed to set duty");
        self.rl.set_duty(0).expect("failed to set duty");
        self.rr.set_duty(0).expect("failed to set duty");
    }

    // set per-motor duties independently, returns the computed hw duty values for logging.
    pub fn set_motors(&self, fl: f32, fr: f32, rl: f32, rr: f32) -> (u32, u32, u32, u32) {
        let duty = |v: f32| (v.clamp(0., 1.) * crate::PWM_MAX_DUTY as f32) as u32;
        let duties = (duty(fl), duty(fr), duty(rl), duty(rr));
        self.fl.set_duty_hw(duties.0);
        self.fr.set_duty_hw(duties.1);
        self.rl.set_duty_hw(duties.2);
        self.rr.set_duty_hw(duties.3);
        duties
    }
}
