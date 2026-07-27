// persists accelerometer calibration across reboots/reflashes.
//
// reuses the `nvs` partition from the default ESP32 partition table (0x9000, 24KB) - this
// project doesn't run ESP-IDF's actual NVS system, so that space is otherwise unused and
// nothing else on the chip touches it. `cargo erase-calibration` (espflash erase-region
// 0x9000 0x6000) clears it back to "uncalibrated"

use embedded_storage::{ReadStorage, Storage};
use esp_bootloader_esp_idf::partitions::{self, DataPartitionSubType, PartitionType};
use esp_hal::rom::crc;
use esp_storage::FlashStorage;

use crate::flight::AccelBias;

// marks a written record, readable in a raw flash dump, distinguishing it from erased flash
// (reads back as all 0xFF) or leftover garbage
const MAGIC: [u8; 4] = *b"ACAL";
// INVARIANT: added fields to accelbias need to have more space allocated here
const PAYLOAD_LEN: usize = (3 * 4) + (3 * 4); // 3 f32 bias + 3 f32 scale in AccelBias 24b
const RECORD_LEN: usize = MAGIC.len() + PAYLOAD_LEN + 2; // magic + payload + crc16 30b

impl AccelBias {
    fn as_bytes(&self) -> [u8; RECORD_LEN] {
        let mut buf = [0u8; RECORD_LEN];
        buf[0..4].copy_from_slice(&MAGIC);
        buf[4..8].copy_from_slice(&self.bias.x.to_le_bytes());
        buf[8..12].copy_from_slice(&self.bias.y.to_le_bytes());
        buf[12..16].copy_from_slice(&self.bias.z.to_le_bytes());
        buf[16..20].copy_from_slice(&self.scale.x.to_le_bytes());
        buf[20..24].copy_from_slice(&self.scale.y.to_le_bytes());
        buf[24..28].copy_from_slice(&self.scale.z.to_le_bytes());
        let crc = crc_16(&buf[4..4 + PAYLOAD_LEN]);
        buf[4 + PAYLOAD_LEN..RECORD_LEN].copy_from_slice(&crc.to_le_bytes());
        buf
    }

    fn from_bytes(buf: &[u8; RECORD_LEN]) -> Option<AccelBias> {
        if buf[0..4] != MAGIC {
            return None;
        }
        let stored_crc = u16::from_le_bytes(buf[4 + PAYLOAD_LEN..RECORD_LEN].try_into().ok()?);
        if stored_crc != crc_16(&buf[4..4 + PAYLOAD_LEN]) {
            return None;
        }
        Some(Self {
            bias: nalgebra::Vector3::new(
                f32::from_le_bytes(buf[4..8].try_into().unwrap()),
                f32::from_le_bytes(buf[8..12].try_into().unwrap()),
                f32::from_le_bytes(buf[12..16].try_into().unwrap()),
            ),
            scale: nalgebra::Vector3::new(
                f32::from_le_bytes(buf[16..20].try_into().unwrap()),
                f32::from_le_bytes(buf[20..24].try_into().unwrap()),
                f32::from_le_bytes(buf[24..28].try_into().unwrap()),
            ),
        })
    }
}

fn crc_16(buf: &[u8]) -> u16 {
    // read the docs, it inverts on entry so you need to start with !0
    !crc::crc16_le(!0, buf)
    //and invert final result, only intermediate values aren't inverted
}

// locates the nvs partition and hands a FlashRegion scoped to it to f
fn with_nvs_region<R, F>(flash: &mut FlashStorage<'_>, f: F) -> Option<R>
where
    F: FnOnce(&mut partitions::FlashRegion<'_, FlashStorage<'_>>) -> R,
{
    let mut table_buf = [0u8; partitions::PARTITION_TABLE_MAX_LEN];
    let table = partitions::read_partition_table(flash, &mut table_buf).ok()?;
    let entry = table
        .find_partition(PartitionType::Data(DataPartitionSubType::Nvs))
        .ok()??;
    let mut region = entry.as_embedded_storage(flash);
    Some(f(&mut region))
}

/// Loads the persisted accelerometer calibration, if one has been written and not since erased.
pub fn load_accel_calibration(flash: &mut FlashStorage<'_>) -> Option<AccelBias> {
    with_nvs_region(flash, |region| {
        let mut buf = [0u8; RECORD_LEN];
        region.read(0, &mut buf).ok()?;
        AccelBias::from_bytes(&buf)
    })
    .flatten()
}

/// Persists accelerometer calibration so it survives a reboot or reflash of the app image.
#[cfg_attr(not(feature = "calibrate"), allow(unused))]
pub fn store_accel_calibration(flash: &mut FlashStorage<'_>, cal: &AccelBias) -> bool {
    with_nvs_region(flash, |region| region.write(0, &cal.as_bytes()).is_ok()).unwrap_or(false)
}
