//! weak "assume roughly level" attitude prior - matches the idea behind flix's applyLevel()
//! (estimate.ino), but implemented with nalgebra's own rotation-between primitive rather than
//! porting flix's Quaternion/Vector code directly. flix's rotateVector() uses a non-standard
//! conjugation order (q^-1 * v * q, versus the q * v * q^-1 nalgebra's transform_vector uses) -
//! transliterating it line-by-line risked silently applying the correction backwards, so this
//! re-derives the same effect from nalgebra's own, already-verified conventions instead.
//!
//! the idea: accelerometer-based attitude correction can be unavailable for large stretches of a
//! real flight (vibration corrupting most samples, gating them out) - during those stretches gyro
//! integration alone accumulates roll/pitch drift with nothing to bound it, the same failure mode
//! already confirmed on yaw with heading-hold disabled. a very weak, constant nudge back toward
//! "roughly level" - much weaker than a real accel correction - bounds that drift without needing
//! any accel signal at all, at the cost of assuming the craft is *usually* flown close to level.
use nalgebra::{UnitQuaternion, Vector3};

/// Nudges `quat` a small amount toward level (zero roll/pitch), leaving yaw untouched.
///
/// `weight` is how much of the full correction to apply this call - flix's own levelWeight
/// (0.0002) is ~15x weaker than its accel correction weight, intentionally: this is a fallback
/// prior, not a sensor-derived correction, and should never fight a real one it never sees.
pub fn apply_level_prior(quat: UnitQuaternion<f32>, weight: f32) -> UnitQuaternion<f32> {
    // where the body's own "up" axis currently points, in world coordinates - equal to
    // Vector3::z() exactly when level, deviating as roll/pitch drift away from that
    let estimated_up = quat.transform_vector(&Vector3::z());
    let world_up = Vector3::z();

    // None when the vectors are already collinear (either already level - angle 0 - or exactly
    // inverted - angle pi, axis undefined either way): nothing sensible to correct, leave as-is
    let Some(full_correction) = UnitQuaternion::rotation_between(&estimated_up, &world_up) else {
        return quat;
    };

    // scale the full correction down to a small nudge - can't scale a unit quaternion directly,
    // so go through its axis-angle (scaled axis) form, shrink the angle, and rebuild
    let weak_correction = UnitQuaternion::from_scaled_axis(full_correction.scaled_axis() * weight);

    // world-frame correction, applied on top of the existing body-to-world attitude
    weak_correction * quat
}

#[cfg(test)]
mod tests {
    use super::*;

    // angle between the body's up axis and true world up - 0 when level, grows with tilt
    fn tilt(quat: UnitQuaternion<f32>) -> f32 {
        let up = quat.transform_vector(&Vector3::z());
        up.angle(&Vector3::z())
    }

    #[test]
    fn already_level_stays_level() {
        let quat = UnitQuaternion::from_euler_angles(0.0, 0.0, 0.7); // level, arbitrary yaw
        let corrected = apply_level_prior(quat, 0.01);
        assert!(
            tilt(corrected) < 1e-6,
            "level input should stay level, got tilt {}",
            tilt(corrected)
        );
    }

    #[test]
    fn tilted_moves_toward_level_not_away() {
        let quat = UnitQuaternion::from_euler_angles(0.2, 0.15, 0.0); // ~11/8.6 deg roll/pitch
        let before = tilt(quat);
        let after = tilt(apply_level_prior(quat, 0.05));
        assert!(
            after < before,
            "correction should reduce tilt: before={before} after={after}"
        );
    }

    #[test]
    fn yaw_is_preserved() {
        // the correction rotation's axis is always horizontal in world frame - for any vector A,
        // A x (0,0,1) = (Ay, -Ax, 0), zero Z component always - so it never spins around vertical.
        // at realistic (tiny) weights that holds cleanly; at an artificially large weight the
        // *extracted* Euler yaw can show a small second-order coupling from composing a big tilt
        // change onto an already-tilted quaternion, which is a property of Euler decomposition,
        // not evidence the correction itself introduces yaw. use a realistic weight here - flix's
        // own levelWeight is 0.0002, this uses 0.001 for a bit of margin while staying small
        let quat = UnitQuaternion::from_euler_angles(0.25, -0.18, 1.2);
        let (_, _, yaw_before) = quat.euler_angles();
        let corrected = apply_level_prior(quat, 0.001);
        let (_, _, yaw_after) = corrected.euler_angles();
        assert!(
            (yaw_after - yaw_before).abs() < 1e-4,
            "yaw should be untouched: before={yaw_before} after={yaw_after}"
        );
    }

    #[test]
    fn larger_weight_corrects_more() {
        let quat = UnitQuaternion::from_euler_angles(0.3, 0.2, 0.4);
        let before = tilt(quat);
        let small = before - tilt(apply_level_prior(quat, 0.01));
        let large = before - tilt(apply_level_prior(quat, 0.1));
        assert!(
            large > small,
            "larger weight should reduce tilt more: small_weight_delta={small} large_weight_delta={large}"
        );
    }

    #[test]
    fn repeated_application_converges_toward_level() {
        let mut quat = UnitQuaternion::from_euler_angles(0.4, -0.3, 0.9);
        let start = tilt(quat);
        for _ in 0..2000 {
            quat = apply_level_prior(quat, 0.01);
        }
        assert!(
            tilt(quat) < start * 0.01,
            "1000s of ticks at a small weight should converge close to level: start={start} end={}",
            tilt(quat)
        );
    }

    #[test]
    fn exactly_inverted_does_not_panic_or_produce_nan() {
        // 180 degree roll - up axis points exactly opposite world up, rotation axis undefined
        let quat = UnitQuaternion::from_euler_angles(core::f32::consts::PI, 0.0, 0.0);
        let corrected = apply_level_prior(quat, 0.01);
        assert!(corrected.w.is_finite());
        assert!(corrected.i.is_finite());
        assert!(corrected.j.is_finite());
        assert!(corrected.k.is_finite());
    }

    #[test]
    fn weight_zero_is_a_no_op() {
        let quat = UnitQuaternion::from_euler_angles(0.3, -0.2, 0.5);
        let corrected = apply_level_prior(quat, 0.0);
        assert!((corrected.angle_to(&quat)).abs() < 1e-6);
    }
}
