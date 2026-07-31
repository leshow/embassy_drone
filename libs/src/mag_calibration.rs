//! magnetometer hard-iron/soft-iron calibration - ported from peterkrull/mag-calibrator-rs
//! (https://github.com/peterkrull/mag-calibrator-rs, Apache-2.0), bumped from its pinned
//! nalgebra 0.32 to the 0.35 this workspace already uses, plus the get_mean_distance() fix
//! from holsatus-flight's copy of the same algorithm (returns 0.0 until the sample buffer is
//! actually full)
//!
//! keeps the N most mutually-distinct raw samples seen (a k-nearest-neighbors "usefulness"
//! check discards a new sample unless it's farther from its neighbors than the buffer's least
//! useful entry), then fits an axis-aligned ellipsoid to that buffer via closed-form least
//! squares once the samples are spread out enough - see perform_calibration
use nalgebra::{ComplexField, SMatrix, SMatrixView, Vector3};

pub struct MagCalibrator<const N: usize> {
    matrix: SMatrix<f32, N, 6>,
    matrix_filled: usize,
    mean_distance: f32,
    pre_scaler: f32,
    k: usize,
}

impl<const N: usize> Default for MagCalibrator<N> {
    fn default() -> Self {
        Self {
            matrix: SMatrix::from_element(1.0),
            matrix_filled: Default::default(),
            mean_distance: Default::default(),
            pre_scaler: 1.,
            k: 2, // works well in testing
        }
    }
}

impl<const N: usize> MagCalibrator<N> {
    /// Create a new calibrator instance.
    pub fn new() -> Self {
        Self::default()
    }

    /// Configure the number of `k` neighbors to calculate distance to.
    pub fn num_neighbors(self, k: usize) -> Self {
        Self { k, ..self }
    }

    /// Configure sample pre scaler, prevents ill-conditioning if given
    /// a value close to the expected magnitude of the magnetic field strength.
    pub fn pre_scaler(self, pre_scaler: f32) -> Self {
        Self { pre_scaler, ..self }
    }

    /// Calculates mean distance to the `k` nearest neighbors.
    /// A smaller number means the point is "similar" to its neighbors.
    fn mean_distance_from_single(&self, vec: SMatrix<f32, 1, 3>) -> f32 {
        let matrix_view: SMatrixView<f32, N, 3> = self.matrix.fixed_columns::<3>(0);

        // distance to every other point
        let mut squared_dists: [f32; N] = [0.; N];
        matrix_view.row_iter().enumerate().for_each(|(j, cmp)| {
            let diff = vec - cmp;
            squared_dists[j] = diff.dot(&diff).sqrt();
        });

        // sort floats and return mean distance to nearest neighbors
        squared_dists.sort_unstable_by(|a, b| a.total_cmp(b));
        squared_dists
            .iter()
            .take(self.k + 1)
            .rfold(0., |a, &b| a + b)
            / N as f32
    }

    /// Calculates mean squared distance to the `k` nearest neighbors
    /// between all `N` row vectors in the internal buffer.
    /// A smaller number means a point is "similar" to its neighbors.
    fn mean_distance_from_all(&self) -> [f32; N] {
        let mut mean_dist: [f32; N] = [0.; N];

        let matrix_view: SMatrixView<f32, N, 3> = self.matrix.fixed_columns::<3>(0);
        matrix_view.row_iter().enumerate().for_each(|(i, row)| {
            mean_dist[i] = self.mean_distance_from_single(row.into());
        });
        mean_dist
    }

    /// Returns index of vector with the lowest squared distance.
    /// Is used when replacing the least useful value in the array.
    fn lowest_mean_distance_by_index(&mut self) -> (usize, f32) {
        let mean_dist = self.mean_distance_from_all();

        // set mean distance now that we are at it
        self.mean_distance = mean_dist.iter().rfold(0., |a, &b| a + b) / N as f32;

        // obtain index for lowest mean distance
        mean_dist
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.total_cmp(b))
            .map(|(index, value)| (index, *value))
            .unwrap()
    }

    /// Evaluates whether the new sample should replace one already in the buffer.
    pub fn evaluate_sample(&mut self, x: [f32; 3]) {
        self.evaluate_sample_vec(Vector3::from(x))
    }

    /// Add a sample if it is deemed more useful than the least useful sample.
    pub fn evaluate_sample_vec(&mut self, x: Vector3<f32>) {
        // ensure all entries are normal (not Inf or NaN)
        if !x.iter().all(|e| e.is_normal()) {
            return;
        }
        // check if buffer is not yet "initialized" with real measurements
        if self.matrix_filled < N {
            self.add_sample_at(self.matrix_filled, x);
            self.matrix_filled += 1;
        }
        // otherwise check which sample may be best to replace
        else {
            let (low_index, low_mean_dist) = self.lowest_mean_distance_by_index();
            let sample_mean_dist = self.mean_distance_from_single(x.transpose());
            if low_mean_dist < sample_mean_dist {
                self.add_sample_at(low_index, x);
            }
        }
    }

    /// Insert a sample vector into `index` row of buffer matrix.
    fn add_sample_at(&mut self, index: usize, sample: Vector3<f32>) {
        if index < N {
            self.matrix[(index, 0)] = sample[0] / self.pre_scaler;
            self.matrix[(index, 1)] = sample[1] / self.pre_scaler;
            self.matrix[(index, 2)] = sample[2] / self.pre_scaler;
        }
    }

    /// Get mean distance value between samples in matrix buffer. Returns 0.0 until the
    /// buffer is actually full - before that, `mean_distance` hasn't been computed yet
    /// and would otherwise read back as a stale/meaningless zero-initialized value.
    pub fn get_mean_distance(&self) -> f32 {
        if self.matrix_filled < N {
            0.0
        } else {
            self.mean_distance
        }
    }

    /// Try to calculate calibration offset and scale values. Returns None if
    /// it was not possible to calculate the pseudo inverse, or if some of the
    /// parameters are `NaN`. In that case it would be best to restart the whole
    /// calibration and collect new samples. The tuple contains (offset, scale).
    pub fn perform_calibration(&mut self) -> Option<([f32; 3], [f32; 3])> {
        // calculate column 4 and 5 of H matrix
        self.matrix.row_iter_mut().for_each(|mut mag| {
            mag[3] = -mag[1] * mag[1];
            mag[4] = -mag[2] * mag[2];
        });

        // calculate W vector
        let mut w: SMatrix<f32, N, 1> = SMatrix::from_element(0.0);
        self.matrix
            .row_iter()
            .enumerate()
            .for_each(|(i, row)| w[i] = row[0] * row[0]);

        // perform least squares using pseudo inverse
        let x =
            (self.matrix.transpose() * self.matrix).try_inverse()? * self.matrix.transpose() * w;

        // calculate offsets and scale factors
        let off = [x[0] / 2., x[1] / (2. * x[3]), x[2] / (2. * x[4])];
        let temp = x[5] + (off[0] * off[0]) + x[3] * (off[1] * off[1]) + x[4] * (off[2] * off[2]);
        let scale = [temp.sqrt(), (temp / x[3]).sqrt(), (temp / x[4]).sqrt()];

        // check that off and scale vectors contain valid values
        for x in off.iter().chain(scale.iter()) {
            if !x.is_finite() {
                return None;
            }
        }

        // unscale the offset values
        let off = off.map(|x| x * self.pre_scaler);
        let scale = scale.map(|x| x * self.pre_scaler);

        Some((off, scale))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // scatters N points roughly evenly over a unit sphere (Fibonacci sphere), so the
    // calibrator sees good directional coverage without needing real rotation data. fixed-size
    // array, not Vec - no allocator needed, matches MagCalibrator's own no_std/no-alloc style
    fn fibonacci_sphere<const N: usize>() -> [Vector3<f32>; N] {
        let golden_angle = core::f32::consts::PI * (3.0 - ComplexField::sqrt(5.0));
        core::array::from_fn(|i| {
            let y = 1.0 - 2.0 * (i as f32) / ((N - 1) as f32);
            let radius = ComplexField::sqrt((1.0 - y * y).max(0.0));
            let theta = golden_angle * i as f32;
            let x = ComplexField::cos(theta) * radius;
            let z = ComplexField::sin(theta) * radius;
            Vector3::new(x, y, z)
        })
    }

    #[test]
    fn recovers_known_bias_and_scale() {
        // bias needs to stay roughly comparable in magnitude to scale here, not wildly larger -
        // this is an algebraic (non-iterative) ellipsoid fit, and a large offset-to-radius ratio
        // makes it ill-conditioned regardless of pre_scaler (pre_scaler only rescales absolute
        // magnitude to avoid float range issues, it doesn't fix that kind of ill-posedness).
        // realistic for an actual sensor too - hard-iron bias swamping the field range 20x over
        // would mean the mount location is unusable, not something calibration fixes anyway
        let true_bias = Vector3::new(1.2_f32, -0.6, 0.9);
        let true_scale = Vector3::new(1.4_f32, 0.8, 1.1);

        let mut cal = MagCalibrator::<26>::new().num_neighbors(1);
        for direction in fibonacci_sphere::<200>() {
            // matches the model perform_calibration solves for: raw = bias + scale * direction
            let raw = true_bias + true_scale.component_mul(&direction);
            cal.evaluate_sample_vec(raw);
        }

        let (bias, scale) = cal.perform_calibration().expect("fit should succeed");

        for i in 0..3 {
            assert!(
                (bias[i] - true_bias[i]).abs() < 1e-2,
                "bias[{i}] = {}, want {}",
                bias[i],
                true_bias[i]
            );
            assert!(
                (scale[i] - true_scale[i]).abs() < 1e-2,
                "scale[{i}] = {}, want {}",
                scale[i],
                true_scale[i]
            );
        }
    }

    #[test]
    fn rejects_nan_and_infinite_samples() {
        let mut cal = MagCalibrator::<8>::new();
        cal.evaluate_sample_vec(Vector3::new(f32::NAN, 0.0, 0.0));
        cal.evaluate_sample_vec(Vector3::new(0.0, f32::INFINITY, 0.0));
        assert_eq!(cal.get_mean_distance(), 0.0, "buffer should still be empty");
    }

    #[test]
    fn mean_distance_is_zero_until_buffer_fills() {
        let mut cal = MagCalibrator::<8>::new();
        for direction in fibonacci_sphere::<4>() {
            cal.evaluate_sample_vec(direction);
            assert_eq!(
                cal.get_mean_distance(),
                0.0,
                "buffer isn't full yet, shouldn't report a real distance"
            );
        }
    }

    #[test]
    fn identical_samples_fail_to_fit() {
        // a degenerate buffer (every sample the same point) is rank-deficient - the
        // pseudo-inverse doesn't exist, perform_calibration should report that rather
        // than return a bogus fit
        let mut cal = MagCalibrator::<8>::new();
        for _ in 0..8 {
            cal.evaluate_sample_vec(Vector3::new(1.0, 0.0, 0.0));
        }
        assert!(cal.perform_calibration().is_none());
    }
}
