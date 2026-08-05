// a panic halts the running task, but the LEDC peripheral keeps outputting whatever duty
// cycle was last set - it runs independently of the CPU once configured. left alone, motors
// stay locked at their last commanded throttle forever. LEDC::regs() gets us the register
// block directly (no ownership of the running Motors/Channel objects needed), so we can zero
// every channel's duty from here regardless of what task was mid-flight when it panicked.
//
// custom-pre-backtrace and custom-halt (enabled in Cargo.toml) are esp-backtrace's hooks:
// custom_pre_backtrace runs first, before anything else in the panic handler, and custom_halt
// replaces the default infinite loop at the end.

use esp_hal::peripherals::LEDC;

const NUM_CHANNELS: usize = 4;

// register sequence transcribed from esp-hal 1.1.1's Channel::set_duty_hw /
// start_duty_without_fading / update_channel (src/ledc/channel.rs, the `#[cfg(not(any(esp32,
// esp32c6, esp32h2)))]` branch - confirmed against esp-hal source that s3 falls into this
// branch, not the c6/h2 gamma-register scheme)
//
// the same three steps Motors::turn_off(), but since we dont have ownership of motor
// in panic handler we do it through static accessor
fn kill_motors() {
    let ledc = LEDC::regs();
    for ch in 0..NUM_CHANNELS {
        ledc.ch(ch).duty().write(|w| unsafe { w.duty().bits(0) });
        ledc.ch(ch).conf1().write(|w| {
            w.duty_start().set_bit();
            w.duty_inc().set_bit();
            unsafe {
                w.duty_num().bits(0x1);
                w.duty_cycle().bits(0x1);
                w.duty_scale().bits(0x0)
            }
        });
        ledc.ch(ch).conf0().modify(|_, w| w.para_up().set_bit());
    }
}

#[unsafe(no_mangle)]
extern "Rust" fn custom_pre_backtrace() {
    kill_motors();
}

#[unsafe(no_mangle)]
extern "Rust" fn custom_halt() -> ! {
    esp_hal::system::software_reset()
}
