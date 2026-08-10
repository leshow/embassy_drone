use crate::flight::filters;

/// PID
///
/// Proportional-Integral-Derivative controller — computes a corrective output
/// from three components of the error signal:
///
/// P proportional: output -> current error (instant response, but can oscillate)
/// I integral:     output -> accumulated past error (eliminates steady-state offset)
/// D derivative:   output -> rate of change of error (damps oscillation)
///
/// Combined: output = kp·e + ki·∫e·dt + kd·(de/dt)
pub struct Pid {
    /// proportional gain — how hard to push per unit of current error
    kp: f32,
    /// integral gain — how hard to push per unit of accumulated error
    ki: f32,
    /// derivative gain — how hard to damp per unit of error rate
    kd: f32,
    /// running sum of error×dt, clamped to ±integral_limit
    integral: f32,
    /// error from last tick for de/dt; NaN until first update to avoid derivative spike on arm
    prev_error: f32,
    /// anti-windup clamp — keeps integral from growing unbounded
    integral_limit: f32,
    /// per-second decay applied to the integral before adding this tick's error - see
    /// RATE_INTEGRAL_LEAK. A persistent error settles at error/leak instead of climbing to
    /// integral_limit regardless of how small it is
    leak: f32,
    /// low-pass filter state for the derivative term - see RATE_LPF_HZ
    d_filter: f32,
}

impl Pid {
    pub const fn new(kp: f32, ki: f32, kd: f32, integral_limit: f32, leak: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: f32::NAN,
            integral_limit,
            leak,
            d_filter: 0.0,
        }
    }

    pub fn update(&mut self, error: f32, dt: f32) -> f32 {
        // matches flix: reset if dt is zero or impossibly large
        if dt <= 0.0 || dt > 0.5 {
            self.reset();
            return self.kp * error;
        }
        self.integral = (self.integral * (1.0 - self.leak * dt).max(0.0) + error * dt)
            .clamp(-self.integral_limit, self.integral_limit);
        let raw_derivative = if self.prev_error.is_nan() {
            0.0
        } else {
            (error - self.prev_error) / dt
        };
        self.prev_error = error;

        // hardware DLPF is 196 Hz (much wider than this filter's 40 Hz target), so real
        // prop/motor vibration was passing straight through into the derivative term
        // unfiltered - see RATE_LPF_HZ
        let derivative = {
            self.d_filter +=
                filters::lpf_alpha(filters::RATE_LPF_HZ, dt) * (raw_derivative - self.d_filter);
            self.d_filter
        };

        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.d_filter = 0.0;
        self.prev_error = f32::NAN;
    }
}
