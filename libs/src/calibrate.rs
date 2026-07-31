//! calibrate packet

// calibration packet start
pub const CALIBRATION_MAGIC: [u8; 4] = *b"CALB";
// magic + 1 discriminant byte
pub const CALIBRATION_SIZE: usize = 5;

///  see firmware's wifi::calibrate_task.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CalibrationMode {
    Level = 0,
    FrontUp = 1,
    BackUp = 2,
    LeftSide = 3,
    RightSide = 4,
    UpsideDown = 5,
    Ended = 6,
    Failed = 7,
    // no fixed poses - rotate freely through as many orientations as possible
    MagRotate = 8,
}

impl core::fmt::Display for CalibrationMode {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.name())
    }
}

impl CalibrationMode {
    // cant use Display in firmware crate
    pub fn name(&self) -> &'static str {
        match self {
            Self::Level => "level",
            Self::FrontUp => "front side up",
            Self::BackUp => "back side up",
            Self::LeftSide => "left side up",
            Self::RightSide => "right side up",
            Self::UpsideDown => "upside down",
            Self::Ended => "ended",
            Self::Failed => "failed",
            Self::MagRotate => "rotate freely (mag calibration)",
        }
    }

    pub fn to_bytes(self) -> [u8; CALIBRATION_SIZE] {
        let mut buf = [0u8; CALIBRATION_SIZE];
        buf[0..4].copy_from_slice(&CALIBRATION_MAGIC);
        buf[4] = self as u8;
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < CALIBRATION_SIZE || buf[0..4] != CALIBRATION_MAGIC {
            return None;
        }
        Some(match buf[4] {
            0 => Self::Level,
            1 => Self::FrontUp,
            2 => Self::BackUp,
            3 => Self::LeftSide,
            4 => Self::RightSide,
            5 => Self::UpsideDown,
            6 => Self::Ended,
            7 => Self::Failed,
            8 => Self::MagRotate,
            _ => return None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calibration_mode_roundtrip() {
        for mode in [
            CalibrationMode::Level,
            CalibrationMode::FrontUp,
            CalibrationMode::BackUp,
            CalibrationMode::LeftSide,
            CalibrationMode::RightSide,
            CalibrationMode::UpsideDown,
            CalibrationMode::Ended,
            CalibrationMode::Failed,
            CalibrationMode::MagRotate,
        ] {
            assert_eq!(CalibrationMode::from_bytes(&mode.to_bytes()), Some(mode));
        }
    }

    #[test]
    fn test_calibration_mode_rejects_bad_magic() {
        let mut bytes = CalibrationMode::Level.to_bytes();
        bytes[0] = b'X';
        assert_eq!(CalibrationMode::from_bytes(&bytes), None);
    }

    #[test]
    fn test_calibration_mode_rejects_short_buffer() {
        let bytes = CalibrationMode::Level.to_bytes();
        assert_eq!(
            CalibrationMode::from_bytes(&bytes[..CALIBRATION_SIZE - 1]),
            None
        );
    }
}
