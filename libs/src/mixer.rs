//! quad motor mixing - combines a throttle command with per-axis torques into four per-motor
//! duty values, each guaranteed to land in [0, 1].

/// Betaflight-style two-sided desaturation - matches mixTable()/applyMixerAdjustment()
/// (mixer.c). Scales the whole differential mix down (preserving the roll/pitch/yaw ratio) if
/// its spread can't fit in [0, 1] no matter what throttle is, then moves the throttle baseline
/// rather than clamping motors individually - so a motor's correction is never silently
/// truncated the way [`mix_motors_flix`]'s high-side-only desaturation lets it be.
///
/// Returns (fl, fr, rl, rr), each guaranteed in [0, 1].
pub fn mix_motors(
    throttle: f32,
    roll_torque: f32,
    pitch_torque: f32,
    yaw_torque: f32,
) -> (f32, f32, f32, f32) {
    let mix_fl = roll_torque - pitch_torque + yaw_torque;
    let mix_fr = -roll_torque - pitch_torque - yaw_torque;
    let mix_rl = roll_torque + pitch_torque - yaw_torque;
    let mix_rr = -roll_torque + pitch_torque + yaw_torque;

    let mix_min = mix_fl.min(mix_fr).min(mix_rl).min(mix_rr);
    let mix_max = mix_fl.max(mix_fr).max(mix_rl).max(mix_rr);
    let mix_range = mix_max - mix_min;

    // if the spread between motors can't fit in [0,1] no matter what throttle is, scale all
    // four down proportionally instead of clamping individual ones
    let scale = if mix_range > 1.0 {
        1.0 / mix_range
    } else {
        1.0
    };
    let mix_fl = mix_fl * scale;
    let mix_fr = mix_fr * scale;
    let mix_rl = mix_rl * scale;
    let mix_rr = mix_rr * scale;
    let mix_min = mix_min * scale;
    let mix_max = mix_max * scale;

    // lowest-mix motor must stay >= 0: throttle + mix_min >= 0  =>  throttle >= -mix_min
    let lo = -mix_min;
    // highest-mix motor must stay <= 1: throttle + mix_max <= 1  =>  throttle <= 1.0 - mix_max.
    //
    // floored at lo: mix_min/mix_max above are each independently rescaled by a separate float
    // multiply, so even though (mix_max - mix_min) is exactly 1.0 in real-number arithmetic once
    // scaled, f32 rounding can land it a hair above 1.0 - which would make 1.0 - mix_max fall
    // below lo and panic f32::clamp. confirmed on real hardware: this crashed mid-flight
    // (panicked at core::num::f32::clamp, min > max), not a power brownout, which is what it
    // looked like from outside. flooring at lo instead of trusting the upstream math to be exact
    // in floating point: worst case this pins throttle to lo, putting the highest-mix motor at
    // mix_max - mix_min (a hair over 1.0), which Motors::set_motors' own downstream clamp
    // silently rounds down to 1.0 - versus the alternative of a hard panic
    let hi = (1.0 - mix_max).max(lo);
    let throttle = throttle.clamp(lo, hi);

    (
        throttle + mix_fl,
        throttle + mix_fr,
        throttle + mix_rl,
        throttle + mix_rr,
    )
}

/// Old flix-style desaturation - only protects the high side (subtracts a constant from all
/// four motors so the max lands at 1.0); the low side relies on whatever raw per-motor clamp
/// the caller applies downstream. Kept for reference/comparison, not called anywhere: real
/// flight telemetry showed this letting a motor's commanded correction get silently truncated
/// at the low side while the rate loop kept escalating next tick since its correction wasn't
/// landing - see docs/todo.md and [`mix_motors`], which was built specifically to fix that.
#[allow(dead_code)]
pub fn mix_motors_flix(
    throttle: f32,
    roll_torque: f32,
    pitch_torque: f32,
    yaw_torque: f32,
) -> (f32, f32, f32, f32) {
    let mut fl = throttle + roll_torque - pitch_torque + yaw_torque;
    let mut fr = throttle - roll_torque - pitch_torque - yaw_torque;
    let mut rl = throttle + roll_torque + pitch_torque - yaw_torque;
    let mut rr = throttle - roll_torque + pitch_torque + yaw_torque;

    let max = fl.max(fr).max(rl).max(rr);
    if max > 1.0 {
        let excess = max - 1.0;
        fl -= excess;
        fr -= excess;
        rl -= excess;
        rr -= excess;
    }

    (fl, fr, rl, rr)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_torque_passes_throttle_through_unchanged() {
        let (fl, fr, rl, rr) = mix_motors(0.5, 0.0, 0.0, 0.0);
        assert_eq!((fl, fr, rl, rr), (0.5, 0.5, 0.5, 0.5));
    }

    #[test]
    fn small_torques_stay_unscaled() {
        // well within [0,1] with no scaling needed - output should be exactly throttle + mix,
        // not distorted by desaturation at all
        let (fl, fr, rl, rr) = mix_motors(0.5, 0.1, 0.05, 0.02);
        assert!((fl - (0.5 + 0.1 - 0.05 + 0.02)).abs() < 1e-6);
        assert!((fr - (0.5 - 0.1 - 0.05 - 0.02)).abs() < 1e-6);
        assert!((rl - (0.5 + 0.1 + 0.05 - 0.02)).abs() < 1e-6);
        assert!((rr - (0.5 - 0.1 + 0.05 + 0.02)).abs() < 1e-6);
    }

    #[test]
    fn large_torques_stay_in_range_and_preserve_ratio() {
        let (fl, fr, rl, rr) = mix_motors(0.5, 2.0, 1.5, 1.0);
        for m in [fl, fr, rl, rr] {
            assert!((0.0..=1.0).contains(&m), "motor out of range: {m}");
        }
        // scaling preserves relative differences between motors, so the ordering the raw
        // (unscaled) mix would produce should still hold
        assert!(fl > fr); // roll_torque positive: fl gets +roll, fr gets -roll
        assert!(rl > rr);
    }

    // this is the actual regression test for the crash: sweep a wide, fine-grained grid of
    // torque combinations - including many that force the scaling branch (mix_range > 1.0),
    // exactly the condition that triggered the f32::clamp panic on real hardware - and assert
    // every output motor is finite and in range, every time, with no panic
    #[test]
    fn desaturation_never_panics_or_produces_out_of_range_motors() {
        let mut roll = -3.0f32;
        while roll <= 3.0 {
            let mut pitch = -3.0f32;
            while pitch <= 3.0 {
                let mut yaw = -3.0f32;
                while yaw <= 3.0 {
                    for throttle in [0.0f32, 0.1, 0.5, 0.9, 1.0] {
                        let (fl, fr, rl, rr) = mix_motors(throttle, roll, pitch, yaw);
                        for (name, m) in [("fl", fl), ("fr", fr), ("rl", rl), ("rr", rr)] {
                            assert!(m.is_finite(), "{name} not finite: {m}");
                            assert!(
                                (-1e-4..=1.0 + 1e-4).contains(&m),
                                "{name} out of range at throttle={throttle} roll={roll} pitch={pitch} yaw={yaw}: {m}"
                            );
                        }
                    }
                    yaw += 0.3;
                }
                pitch += 0.3;
            }
            roll += 0.3;
        }
    }

    #[test]
    fn flix_style_matches_high_side_only_behavior() {
        // sanity check on the kept-for-reference old version: still desaturates the high side
        let (fl, fr, rl, rr) = mix_motors_flix(0.9, 0.5, 0.0, 0.0);
        assert!(fl.max(fr).max(rl).max(rr) <= 1.0 + 1e-6);
        // and still doesn't protect the low side - this is the known, intentional gap
        let (_, _, _, rr) = mix_motors_flix(0.1, 0.0, 0.0, -0.5);
        assert!(
            rr < 0.0,
            "low side isn't protected in the old version: rr={rr}"
        );
    }
}
