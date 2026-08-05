//! Flash storage task for persistent calibration data
//!
//! This module provides a task that manages storing and retrieving calibration data
//! to/from flash memory using the `sequential-storage` crate. It handles motor
//! calibration, distance calibration, and IMU calibration flags.
//!
//! The data is stored in the `FLASH_STORAGE` section of flash memory defined in
//! `memory.x` (last 8 KB = 2 sectors of 4 KB each).

use defmt::{debug, error, info};
use embassy_rp::flash::{Async, ERASE_SIZE, Flash};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use sequential_storage::{
    cache::NoCache,
    map::{Key, MapConfig, MapStorage, SerializationError, Value},
};

use crate::{
    system::{
        event::{Events, raise_event},
        state::{CalibrationStatus, calibration},
    },
    task::motor_driver::MotorCalibration,
};

/// Size of one flash sector (4 KB on RP2350).
const FLASH_SECTOR_SIZE: usize = ERASE_SIZE;

/// Number of sectors to use for storage (2 sectors for wear leveling).
const STORAGE_SECTOR_COUNT: usize = 2;

/// Total storage size.
const STORAGE_SIZE: usize = FLASH_SECTOR_SIZE * STORAGE_SECTOR_COUNT;

/// Size of the command queue.
const COMMAND_QUEUE_SIZE: usize = 3;

/// Channel for sending flash storage commands.
static FLASH_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, FlashCommand, COMMAND_QUEUE_SIZE> = Channel::new();

/// Internal storage for calibration data (managed by `flash_storage` task only).
static CALIBRATION_DATA: embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Option<CalibrationData>> =
    embassy_sync::mutex::Mutex::new(None);

/// Return the latest cached IMU calibration flags, if available.
pub async fn get_cached_imu_flags() -> Option<ImuCalibrationFlags> {
    let data = CALIBRATION_DATA.lock().await;
    data.as_ref().map(|cal| cal.imu_flags)
}

/// Send a flash storage command.
pub async fn send_flash_command(command: FlashCommand) {
    FLASH_COMMAND_CHANNEL.send(command).await;
}

/// Receive a flash storage command.
async fn receive_flash_command() -> FlashCommand {
    FLASH_COMMAND_CHANNEL.receive().await
}

// ── Public types ─────────────────────────────────────────────────────────────────

/// Type of calibration data.
#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub enum CalibrationKind {
    /// Motor calibration data.
    Motor,
    /// IMU calibration flags.
    ImuFlags,
    /// Distance calibration data.
    Distance,
}

/// Calibration data variants.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum CalibrationDataKind {
    /// Motor calibration data (2 track factors).
    Motor(MotorCalibration),
    /// IMU calibration flags.
    ImuFlags(ImuCalibrationFlags),
    /// Distance calibration factor.
    Distance(f32),
}

/// Commands that can be sent to the flash storage task.
#[derive(Debug, Clone, defmt::Format)]
pub enum FlashCommand {
    /// Save calibration data to flash.
    SaveData(CalibrationDataKind),

    /// Request calibration data (responds via event).
    GetData(CalibrationKind),
}

// ── Persisted data structures ───────────────────────────────────────────────────

/// IMU calibration completion flags persisted in flash.
///
/// Separate from calibration values so flags can be written independently
/// without touching the larger IMU calibration blob.
#[derive(Debug, Clone, Copy, defmt::Format, Default, PartialEq, Eq)]
pub struct ImuCalibrationFlags {
    /// Magnetometer calibration completed.
    pub mag: bool,
}

/// Combined calibration data cached in RAM.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct CalibrationData {
    /// Motor calibration factors (2 × f32).
    pub motor: MotorCalibration,
    /// IMU calibration flags.
    pub imu_flags: ImuCalibrationFlags,
    /// Distance calibration factor.
    pub distance: f32,
}

impl Default for CalibrationData {
    fn default() -> Self {
        Self {
            motor: MotorCalibration::default(),
            imu_flags: ImuCalibrationFlags::default(),
            distance: 1.0,
        }
    }
}

// ── Sequential-storage keys ─────────────────────────────────────────────────────

/// Storage keys for sequential-storage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
enum StorageKey {
    MotorCalibration = 0,
    DistanceFactor = 1,
    ImuFlags = 2,
}

impl Key for StorageKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = *self as u8;
        Ok(1)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        match buffer[0] {
            0 => Ok((Self::MotorCalibration, 1)),
            1 => Ok((Self::DistanceFactor, 1)),
            2 => Ok((Self::ImuFlags, 1)),
            _ => Err(SerializationError::InvalidFormat),
        }
    }
}

// ── Value impl for MotorCalibration (2 × f32 = 8 bytes) ─────────────────────────

impl Value<'_> for MotorCalibration {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 8 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..4].copy_from_slice(&self.left_factor.to_le_bytes());
        buffer[4..8].copy_from_slice(&self.right_factor.to_le_bytes());
        Ok(8)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 8 {
            return Err(SerializationError::BufferTooSmall);
        }
        let left_factor = f32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
        let right_factor = f32::from_le_bytes([buffer[4], buffer[5], buffer[6], buffer[7]]);
        Ok((Self::new(left_factor, right_factor), 8))
    }
}

// ── Value impl for f32 distance factor ──────────────────────────────────────────

/// Wrapper to implement `Value` for a bare `f32`.
#[derive(Debug, Clone, Copy)]
struct DistanceFactor(f32);

impl Value<'_> for DistanceFactor {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 4 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..4].copy_from_slice(&self.0.to_le_bytes());
        Ok(4)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 4 {
            return Err(SerializationError::BufferTooSmall);
        }
        let value = f32::from_le_bytes([buffer[0], buffer[1], buffer[2], buffer[3]]);
        Ok((Self(value), 4))
    }
}

// ── Value impl for ImuCalibrationFlags (1 byte) ─────────────────────────────────

impl Value<'_> for ImuCalibrationFlags {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = u8::from(self.mag);
        Ok(1)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        Ok((Self { mag: buffer[0] != 0 }, 1))
    }
}

// ── Flash storage task ──────────────────────────────────────────────────────────

/// Flash storage task.
///
/// This task handles all flash read/write operations for calibration data.
/// It responds to commands sent via the command channel and uses sequential-storage
/// for wear leveling and data integrity.
///
/// The `FLASH_STORAGE` memory region is defined in `memory.x` at the last 8 KB
/// of the 2048 KB flash. We reference it by absolute flash address offset.
#[allow(clippy::too_many_lines)]
#[embassy_executor::task]
pub async fn flash_storage(flash: Flash<'static, embassy_rp::peripherals::FLASH, Async, { 2048 * 1024 }>) {
    info!("Flash storage task started");

    // FLASH_STORAGE region starts at 0x10000000 + (2048K - 8K).
    // MapConfig interprets the range relative to the provided Flash instance,
    // so we pass the absolute offset from flash base.
    #[allow(clippy::cast_possible_truncation)]
    let flash_base: u32 = (2048 * 1024 - 8 * 1024) as u32;
    #[allow(clippy::cast_possible_truncation)]
    let flash_range = flash_base..(flash_base + STORAGE_SIZE as u32);

    // Create storage instance (owns flash and cache).
    let mut storage = MapStorage::<StorageKey, _, _>::new(flash, MapConfig::new(flash_range.clone()), NoCache::new());

    // Scratch buffer for serialization/deserialization. Motor calibration needs
    // 8 bytes, distance factor 4 bytes, IMU flags 1 byte. 32 bytes is plenty.
    let mut data_buffer: [u8; 32] = [0; 32];

    // Main command processing loop
    loop {
        let command = receive_flash_command().await;
        debug!("Flash command received: {:?}", command);

        match command {
            FlashCommand::GetData(kind) => match kind {
                CalibrationKind::Motor => {
                    info!("Loading motor calibration from flash...");

                    #[allow(unreachable_patterns)]
                    match storage
                        .fetch_item::<MotorCalibration>(&mut data_buffer, &StorageKey::MotorCalibration)
                        .await
                    {
                        Ok(Some(motor_cal)) => {
                            info!(
                                "Motor calibration loaded: left={}, right={}",
                                motor_cal.left_factor, motor_cal.right_factor
                            );

                            let mut data = CALIBRATION_DATA.lock().await;
                            if let Some(ref mut cal) = *data {
                                cal.motor = motor_cal;
                            } else {
                                *data = Some(CalibrationData {
                                    motor: motor_cal,
                                    imu_flags: ImuCalibrationFlags::default(),
                                    distance: 1.0,
                                });
                            }
                            drop(data);

                            calibration::CALIBRATION_STATE.lock().await.motor_cal_status = CalibrationStatus::Loaded;
                            raise_event(Events::CalibrationDataLoaded(
                                CalibrationKind::Motor,
                                Some(CalibrationDataKind::Motor(motor_cal)),
                            ))
                            .await;
                        }
                        Ok(None) => {
                            info!("No motor calibration found in flash");
                            raise_event(Events::CalibrationDataLoaded(CalibrationKind::Motor, None)).await;
                        }
                        Err(e) => {
                            error!("Failed to load motor calibration: {}", defmt::Debug2Format(&e));
                            raise_event(Events::CalibrationDataLoaded(CalibrationKind::Motor, None)).await;
                        }
                        _ => {}
                    }
                }
                CalibrationKind::ImuFlags => {
                    info!("Loading IMU calibration flags from flash...");

                    #[allow(unreachable_patterns)]
                    match storage
                        .fetch_item::<ImuCalibrationFlags>(&mut data_buffer, &StorageKey::ImuFlags)
                        .await
                    {
                        Ok(Some(flags)) => {
                            info!("IMU calibration flags loaded: mag={}", flags.mag);

                            let mut data = CALIBRATION_DATA.lock().await;
                            if let Some(ref mut cal) = *data {
                                cal.imu_flags = flags;
                            } else {
                                *data = Some(CalibrationData {
                                    motor: MotorCalibration::default(),
                                    imu_flags: flags,
                                    distance: 1.0,
                                });
                            }
                            drop(data);

                            {
                                let mut s = calibration::CALIBRATION_STATE.lock().await;
                                if flags.mag {
                                    s.mag_cal_status = CalibrationStatus::Loaded;
                                }
                                s.imu_cal_status = CalibrationStatus::Loaded;
                            }
                            raise_event(Events::CalibrationDataLoaded(
                                CalibrationKind::ImuFlags,
                                Some(CalibrationDataKind::ImuFlags(flags)),
                            ))
                            .await;
                        }
                        Ok(None) => {
                            info!("No IMU calibration flags found in flash");
                            raise_event(Events::CalibrationDataLoaded(CalibrationKind::ImuFlags, None)).await;
                        }
                        Err(e) => {
                            error!("Failed to load IMU calibration flags: {}", defmt::Debug2Format(&e));
                            raise_event(Events::CalibrationDataLoaded(CalibrationKind::ImuFlags, None)).await;
                        }
                        _ => {}
                    }
                }
                CalibrationKind::Distance => {
                    info!("Loading distance calibration from flash...");

                    #[allow(unreachable_patterns)]
                    match storage
                        .fetch_item::<DistanceFactor>(&mut data_buffer, &StorageKey::DistanceFactor)
                        .await
                    {
                        Ok(Some(df)) => {
                            info!("Distance calibration loaded: factor={}", df.0);

                            let mut data = CALIBRATION_DATA.lock().await;
                            if let Some(ref mut cal) = *data {
                                cal.distance = df.0;
                            } else {
                                *data = Some(CalibrationData {
                                    motor: MotorCalibration::default(),
                                    imu_flags: ImuCalibrationFlags::default(),
                                    distance: df.0,
                                });
                            }
                            drop(data);

                            {
                                let mut s = calibration::CALIBRATION_STATE.lock().await;
                                s.distance_factor = df.0;
                                s.distance_cal_status = CalibrationStatus::Loaded;
                            }
                            raise_event(Events::CalibrationDataLoaded(
                                CalibrationKind::Distance,
                                Some(CalibrationDataKind::Distance(df.0)),
                            ))
                            .await;
                        }
                        Ok(None) => {
                            info!("No distance calibration found in flash");
                            raise_event(Events::CalibrationDataLoaded(CalibrationKind::Distance, None)).await;
                        }
                        Err(e) => {
                            error!("Failed to load distance calibration: {}", defmt::Debug2Format(&e));
                            raise_event(Events::CalibrationDataLoaded(CalibrationKind::Distance, None)).await;
                        }
                        _ => {}
                    }
                }
            },

            FlashCommand::SaveData(data_kind) => match data_kind {
                CalibrationDataKind::Motor(motor_cal) => {
                    info!("Saving motor calibration to flash...");

                    // Update shared state first
                    let mut data = CALIBRATION_DATA.lock().await;
                    if let Some(ref mut cal) = *data {
                        cal.motor = motor_cal;
                    } else {
                        *data = Some(CalibrationData {
                            motor: motor_cal,
                            imu_flags: ImuCalibrationFlags::default(),
                            distance: 1.0,
                        });
                    }
                    drop(data);

                    #[allow(unreachable_patterns)]
                    match storage
                        .store_item(&mut data_buffer, &StorageKey::MotorCalibration, &motor_cal)
                        .await
                    {
                        Ok(()) => {
                            info!("Motor calibration saved successfully");
                            calibration::CALIBRATION_STATE.lock().await.motor_cal_status = CalibrationStatus::Loaded;
                            raise_event(Events::CalibrationDataLoaded(
                                CalibrationKind::Motor,
                                Some(CalibrationDataKind::Motor(motor_cal)),
                            ))
                            .await;
                        }
                        Err(e) => {
                            error!("Failed to save motor calibration: {}", defmt::Debug2Format(&e));
                        }
                        _ => {}
                    }
                }
                CalibrationDataKind::ImuFlags(flags) => {
                    info!("Saving IMU calibration flags to flash...");

                    let mut data = CALIBRATION_DATA.lock().await;
                    if let Some(ref mut cal) = *data {
                        cal.imu_flags = flags;
                    } else {
                        *data = Some(CalibrationData {
                            motor: MotorCalibration::default(),
                            imu_flags: flags,
                            distance: 1.0,
                        });
                    }
                    drop(data);

                    #[allow(unreachable_patterns)]
                    match storage
                        .store_item(&mut data_buffer, &StorageKey::ImuFlags, &flags)
                        .await
                    {
                        Ok(()) => {
                            info!("IMU calibration flags saved successfully");
                            {
                                let mut s = calibration::CALIBRATION_STATE.lock().await;
                                if flags.mag {
                                    s.mag_cal_status = CalibrationStatus::Loaded;
                                }
                                s.imu_cal_status = CalibrationStatus::Loaded;
                            }
                            raise_event(Events::CalibrationDataLoaded(
                                CalibrationKind::ImuFlags,
                                Some(CalibrationDataKind::ImuFlags(flags)),
                            ))
                            .await;
                        }
                        Err(e) => {
                            error!("Failed to save IMU calibration flags: {}", defmt::Debug2Format(&e));
                        }
                        _ => {}
                    }
                }
                CalibrationDataKind::Distance(factor) => {
                    info!("Saving distance calibration to flash...");

                    let mut data = CALIBRATION_DATA.lock().await;
                    if let Some(ref mut cal) = *data {
                        cal.distance = factor;
                    } else {
                        *data = Some(CalibrationData {
                            motor: MotorCalibration::default(),
                            imu_flags: ImuCalibrationFlags::default(),
                            distance: factor,
                        });
                    }
                    drop(data);

                    let df = DistanceFactor(factor);
                    #[allow(unreachable_patterns)]
                    match storage
                        .store_item(&mut data_buffer, &StorageKey::DistanceFactor, &df)
                        .await
                    {
                        Ok(()) => {
                            info!("Distance calibration saved successfully");
                            {
                                let mut s = calibration::CALIBRATION_STATE.lock().await;
                                s.distance_factor = factor;
                                s.distance_cal_status = CalibrationStatus::Loaded;
                            }
                            raise_event(Events::CalibrationDataLoaded(
                                CalibrationKind::Distance,
                                Some(CalibrationDataKind::Distance(factor)),
                            ))
                            .await;
                        }
                        Err(e) => {
                            error!("Failed to save distance calibration: {}", defmt::Debug2Format(&e));
                        }
                        _ => {}
                    }
                }
            },
        }

        // Small delay to prevent tight loop
        Timer::after(Duration::from_millis(10)).await;
    }
}
