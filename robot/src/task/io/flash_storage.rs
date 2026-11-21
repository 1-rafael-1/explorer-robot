//! Flash storage task for persistent calibration data
//!
//! This module provides a task that manages storing and retrieving calibration data
//! to/from flash memory using the `sequential-storage` crate. It handles motor
//! calibration, distance calibration, and the DMP Bias Snapshot.
//!
//! The data is stored in the `FLASH_STORAGE` section of flash memory defined in
//! `memory.x` (last 8 KB = 2 sectors of 4 KB each).

use defmt::{debug, error, info, warn};
use dmp_cal::{DMP_BIAS_ENCODED_LEN, DmpBiasSnapshot};
use embassy_futures::select::{Either, select};
use embassy_rp::{
    flash::{ERASE_SIZE, Flash},
    mode::Async,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};
use embassy_time::{Duration, Timer};
use motor_cal::{DistanceScale, MotorCalibrationRecord, RECORD_ENCODED_LEN};
use sequential_storage::{
    cache::NoCache,
    map::{Key, MapConfig, MapStorage, SerializationError, Value},
};

use crate::system::{
    event::{Events, raise_event},
    state::calibration,
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

/// Reply slot for one in-flight DMP Bias Snapshot save.
///
/// Capacity one matches the single Active Claim that can start a calibration;
/// a late answer to a timed-out request is dropped rather than wedging the task.
static DMP_BIAS_SAVE_REPLY: Channel<CriticalSectionRawMutex, bool, 1> = Channel::new();

/// Reply slot for one in-flight Motor Calibration Curve save.
///
/// Capacity one and one writer match the DMP bias reply: a late answer to a
/// timed-out request is dropped, and the next save drains it before dispatching
/// its own command.
static MOTOR_CURVE_SAVE_REPLY: Channel<CriticalSectionRawMutex, Result<(), &'static str>, 1> = Channel::new();

/// How long [`save_dmp_bias`] waits for the flash task before reporting failure.
///
/// A save erases and rewrites up to one 4 KB sector, which is well under this;
/// the margin covers a queue that is still draining an earlier command.
const DMP_BIAS_SAVE_TIMEOUT_MS: u64 = 5_000;

/// How long [`save_motor_curve`] waits for the flash task before reporting
/// failure.
///
/// The record is one atomic `store_item` over the same 4 KB sector, so the bound
/// and its reasoning match the DMP bias save.
const MOTOR_CURVE_SAVE_TIMEOUT_MS: u64 = 5_000;

/// The last durable DMP Bias Snapshot, managed by the `flash_storage` task only.
///
/// Set by the boot query when flash holds a valid snapshot, and by a save only
/// after `store_item` succeeded. Never cleared, so a failed save leaves the
/// previous durable snapshot here for the DMP calibration runner to restore.
static DMP_BIAS_CACHE: Mutex<CriticalSectionRawMutex, Option<DmpBiasWords>> = Mutex::new(None);

/// Return the latest cached DMP Bias Snapshot, if the boot query found one.
///
/// `None` before the boot query runs, after a query that found no snapshot, and
/// before any save. The DMP calibration runner uses it to restore the DMP after
/// a Stop or failure without writing flash again.
pub async fn get_cached_dmp_bias() -> Option<DmpBiasWords> {
    *DMP_BIAS_CACHE.lock().await
}

/// Send a flash storage command.
pub async fn send_flash_command(command: FlashCommand) {
    FLASH_COMMAND_CHANNEL.send(command).await;
}

/// Save a DMP Bias Snapshot and report whether flash took it.
///
/// [`send_flash_command`] only enqueues the write, so a caller that returns on
/// the enqueue reports success before persistence has happened. This waits for
/// the task's answer: `true` once `store_item` succeeded, `false` when the
/// write failed, the command could not be enqueued, or the task did not answer.
/// The caller must treat `false` as "the old snapshot is still the durable
/// one".
pub async fn save_dmp_bias(words: DmpBiasWords) -> bool {
    // A late answer to a timed-out request must not be mistaken for this one.
    while DMP_BIAS_SAVE_REPLY.try_receive().is_ok() {}

    if FLASH_COMMAND_CHANNEL
        .try_send(FlashCommand::SaveDmpBias(words))
        .is_err()
    {
        warn!("DMP bias save dropped — flash command queue full");
        return false;
    }

    match select(
        DMP_BIAS_SAVE_REPLY.receive(),
        Timer::after(Duration::from_millis(DMP_BIAS_SAVE_TIMEOUT_MS)),
    )
    .await
    {
        Either::First(saved) => saved,
        Either::Second(()) => {
            warn!("DMP bias save timed out after {}ms", DMP_BIAS_SAVE_TIMEOUT_MS);
            false
        }
    }
}

/// Save a Motor Calibration record and report what flash did with it.
///
/// [`send_flash_command`] only enqueues the write, so a caller that returns on
/// the enqueue reports success before persistence has happened. This waits for
/// the task's answer: `Ok(())` once `store_item` succeeded, and `Err(label)` when
/// the write failed, the command could not be enqueued, or the task did not
/// answer within [`MOTOR_CURVE_SAVE_TIMEOUT_MS`]. The caller must treat `Err` as
/// "the previous record is still the durable one".
///
/// The write is one atomic `store_item` once dispatched: it cannot be
/// interrupted, so a caller that wants to honour a Stop must check for it before
/// calling this.
///
/// # Errors
///
/// Returns the task's static failure label (`"flash write failed"`), the
/// enqueue failure (`"flash command queue full"`), or `"save timeout"`.
pub async fn save_motor_curve(record: MotorCalibrationRecord) -> Result<(), &'static str> {
    // A late answer to a timed-out request must not be mistaken for this one.
    while MOTOR_CURVE_SAVE_REPLY.try_receive().is_ok() {}

    if FLASH_COMMAND_CHANNEL
        .try_send(FlashCommand::SaveMotorCurve(MotorCalibrationPayload(record)))
        .is_err()
    {
        warn!("Motor curve save dropped — flash command queue full");
        return Err("flash command queue full");
    }

    match select(
        MOTOR_CURVE_SAVE_REPLY.receive(),
        Timer::after(Duration::from_millis(MOTOR_CURVE_SAVE_TIMEOUT_MS)),
    )
    .await
    {
        Either::First(result) => result,
        Either::Second(()) => {
            warn!("Motor curve save timed out after {}ms", MOTOR_CURVE_SAVE_TIMEOUT_MS);
            Err("save timeout")
        }
    }
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
    /// Distance calibration data.
    Distance,
    /// DMP Bias Snapshot.
    DmpBias,
}

/// Calibration data variants.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum CalibrationDataKind {
    /// A valid Motor Calibration Curve was loaded and published to the leaf.
    ///
    /// Only the record's identity travels here: the curve itself lives in the
    /// calibration state leaf, so this event is never a second copy of what is
    /// applied.
    Motor(MotorCalSummary),
    /// The measured Distance Calibration scale, in counts per centimetre.
    ///
    /// The value lives in the calibration state leaf already; this carries it to
    /// the boot report so the operator sees what was loaded.
    Distance(f32),
    /// DMP Bias Snapshot (nine raw bias words).
    DmpBias(DmpBiasWords),
}

/// Identity of the Motor Calibration Curve record that was loaded.
#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub struct MotorCalSummary {
    /// The monotonic id of the run that produced the record.
    pub run_id: u32,
    /// The Calibration Floor, in whole percent.
    pub floor_percent: u8,
}

/// Commands that can be sent to the flash storage task.
///
/// The Motor Calibration Curve variant is deliberately one unit: a save carries
/// the whole versioned record, which dwarfs the other commands. The channel
/// holds only three, and `no_std` without an allocator leaves no box to hide
/// the payload behind, so the size difference is accepted rather than papered
/// over.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone)]
pub enum FlashCommand {
    /// Save the DMP Bias Snapshot.
    SaveDmpBias(DmpBiasWords),

    /// Save a Motor Calibration Curve record and publish it as the applied
    /// curve.
    ///
    /// The Motor Calibration Procedure is the only writer; the storage plumbing
    /// lives here so the record and the key are versioned and checksummed in one
    /// place. On success the task publishes the record's curve, scale and run id,
    /// and answers [`save_motor_curve`] with `Ok(())`; the reply is the save
    /// confirmation, and the run id is still stored and published so the leaf
    /// matches the durable record.
    SaveMotorCurve(MotorCalibrationPayload),

    /// Save a measured Distance Calibration scale.
    ///
    /// The scale is written into the same versioned Motor Calibration record:
    /// an existing record keeps its measurements and only takes the new scale,
    /// and a fresh robot gets a scale-only record (no curve).
    SaveDistanceScale(DistanceScale),

    /// Request calibration data (responds via event).
    GetData(CalibrationKind),
}

impl FlashCommand {
    /// A short name for the command debug log, without the payload.
    const fn label(&self) -> &'static str {
        match self {
            Self::SaveDmpBias(_) => "SaveDmpBias",
            Self::SaveMotorCurve(_) => "SaveMotorCurve",
            Self::SaveDistanceScale(_) => "SaveDistanceScale",
            Self::GetData(_) => "GetData",
        }
    }
}

// ── Persisted data structures ───────────────────────────────────────────────────

/// The nine raw DMP bias words as the firmware carries them.
///
/// `dmp-cal` is a pure model with no `defmt` dependency, and the calibration
/// data kinds derive `defmt::Format`, so the payload crosses the task boundary
/// as this small firmware-local type. Encoding and validation stay in
/// [`DmpBiasSnapshot`].
#[derive(Debug, Clone, Copy, defmt::Format, PartialEq, Eq)]
pub struct DmpBiasWords {
    /// Raw gyroscope bias words.
    pub gyro: [i32; 3],
    /// Raw accelerometer bias words.
    pub accel: [i32; 3],
    /// Raw compass bias words.
    pub compass: [i32; 3],
}

impl From<DmpBiasSnapshot> for DmpBiasWords {
    fn from(snapshot: DmpBiasSnapshot) -> Self {
        Self {
            gyro: snapshot.gyro_bias(),
            accel: snapshot.accel_bias(),
            compass: snapshot.compass_bias(),
        }
    }
}

impl From<DmpBiasWords> for DmpBiasSnapshot {
    fn from(words: DmpBiasWords) -> Self {
        Self::new(words.gyro, words.accel, words.compass)
    }
}

// ── Sequential-storage keys ─────────────────────────────────────────────────────

/// Storage keys for sequential-storage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
enum StorageKey {
    /// Retired motor calibration key (`2 × f32`). Never read, written or
    /// migrated again, but kept so the numeric key is never reused and an old
    /// flash entry stays a known, ignored record rather than corrupting the map.
    #[allow(dead_code)]
    RetiredMotorCalibration = 0,
    /// Retired distance calibration key (a bare `f32` factor). Never read,
    /// written or migrated again, but kept so the numeric key is never reused and
    /// an old flash entry stays a known, ignored record rather than corrupting
    /// the map. The measured scale lives in [`StorageKey::MotorCurve`].
    #[allow(dead_code)]
    RetiredDistanceFactor = 1,
    /// Retired host-calibration key. Never requested or written again, but kept
    /// so the numeric key is never reused and an old flash entry stays a known,
    /// ignored record rather than corrupting the map.
    #[allow(dead_code)]
    ImuFlags = 2,
    DmpBias = 3,
    /// The versioned, checksummed Motor Calibration Curve record.
    MotorCurve = 4,
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
            0 => Ok((Self::RetiredMotorCalibration, 1)),
            1 => Ok((Self::RetiredDistanceFactor, 1)),
            2 => Ok((Self::ImuFlags, 1)),
            3 => Ok((Self::DmpBias, 1)),
            4 => Ok((Self::MotorCurve, 1)),
            _ => Err(SerializationError::InvalidFormat),
        }
    }
}

// ── Value impl for the Motor Calibration Curve record ───────────────────────────

/// A Motor Calibration Curve record as it crosses the task boundary and is
/// stored in flash.
///
/// The firmware-local newtype exists because `sequential-storage`'s `Value`
/// trait is foreign to both `motor-cal` and this crate (the orphan rule).
/// Encoding stays in the model: [`Value::serialize_into`] delegates to
/// [`MotorCalibrationRecord::encode_into`] and [`Value::deserialize_from`] to
/// [`MotorCalibrationRecord::decode`], so the stored bytes are the versioned,
/// checksummed payload rather than a firmware-specific layout.
#[derive(Debug, Clone)]
pub struct MotorCalibrationPayload(
    /// The record to store or the record that was loaded.
    pub MotorCalibrationRecord,
);

impl Value<'_> for MotorCalibrationPayload {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        self.0
            .encode_into(buffer)
            .map_err(|_| SerializationError::BufferTooSmall)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        let record = MotorCalibrationRecord::decode(buffer).map_err(|_| SerializationError::InvalidFormat)?;
        Ok((Self(record), RECORD_ENCODED_LEN))
    }
}

// ── Value impl for DmpBiasWords (DMP Bias Snapshot, 39 bytes) ───────────────────

/// Persists the DMP Bias Snapshot through the model's versioned, checksummed
/// encoding. A corrupt or incompatible value is rejected on load.
impl Value<'_> for DmpBiasWords {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < DMP_BIAS_ENCODED_LEN {
            return Err(SerializationError::BufferTooSmall);
        }
        let encoded = DmpBiasSnapshot::from(*self).encode();
        buffer[..DMP_BIAS_ENCODED_LEN].copy_from_slice(&encoded);
        Ok(DMP_BIAS_ENCODED_LEN)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < DMP_BIAS_ENCODED_LEN {
            return Err(SerializationError::BufferTooSmall);
        }
        let snapshot =
            DmpBiasSnapshot::decode(&buffer[..DMP_BIAS_ENCODED_LEN]).map_err(|_| SerializationError::InvalidFormat)?;
        Ok((Self::from(snapshot), DMP_BIAS_ENCODED_LEN))
    }
}

// ── Storage handles ────────────────────────────────────────────────────────────

/// The one `sequential-storage` map this task owns.
type CalibrationStorage = MapStorage<StorageKey, Flash<'static, Async, { 2048 * 1024 }>, NoCache>;

/// The error a fetch or store on that map returns.
type StorageError = sequential_storage::Error<embassy_rp::flash::Error>;

/// Fetch the one Motor Calibration record, or `None` when flash holds none.
async fn fetch_motor_record(
    storage: &mut CalibrationStorage,
    buffer: &mut [u8],
) -> Result<Option<MotorCalibrationRecord>, StorageError> {
    Ok(storage
        .fetch_item::<MotorCalibrationPayload>(buffer, &StorageKey::MotorCurve)
        .await?
        .map(|payload| payload.0))
}

/// Publish an encoder scale, or retire it, and raise the Distance boot event.
///
/// The scale lives inside the one Motor Calibration record, so both the motor
/// and distance boot queries publish through here rather than keeping two copies
/// of the scale-to-state step.
async fn publish_distance_scale(scale: Option<DistanceScale>) {
    calibration::set_distance_scale(scale).await;
    let detail = scale.map(|scale| CalibrationDataKind::Distance(scale.counts_per_cm()));
    raise_event(Events::CalibrationDataLoaded(CalibrationKind::Distance, detail)).await;
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
pub async fn flash_storage(flash: Flash<'static, Async, { 2048 * 1024 }>) {
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

    // Scratch buffer for serialization/deserialization. The Motor Calibration
    // record is the largest item at `RECORD_ENCODED_LEN` (856) bytes; the key byte
    // and flash word alignment need a little headroom. The DMP Bias Snapshot
    // (39 bytes) fits easily.
    let mut data_buffer: [u8; RECORD_ENCODED_LEN + 8] = [0; RECORD_ENCODED_LEN + 8];

    // Main command processing loop
    loop {
        let command = receive_flash_command().await;
        debug!("Flash command received: {}", command.label());

        match command {
            FlashCommand::GetData(kind) => match kind {
                CalibrationKind::Motor => {
                    info!("Loading motor calibration record from flash...");
                    match fetch_motor_record(&mut storage, &mut data_buffer).await {
                        Ok(Some(record)) => {
                            let scale = record.scale();

                            if let Some(curve) = record.curve() {
                                info!(
                                    "Motor calibration curve loaded: run={}, floor={}%, counts_per_cm={}",
                                    record.run_id(),
                                    curve.floor_percent(),
                                    scale.counts_per_cm()
                                );

                                // Publish the derived curve to the one calibration
                                // leaf; the driver reads it from there on SetTracks.
                                calibration::set_motor_calibration(Some(curve), Some(scale)).await;
                                calibration::set_motor_run_id(record.run_id()).await;

                                raise_event(Events::CalibrationDataLoaded(
                                    CalibrationKind::Motor,
                                    Some(CalibrationDataKind::Motor(MotorCalSummary {
                                        run_id: record.run_id(),
                                        floor_percent: curve.floor_percent(),
                                    })),
                                ))
                                .await;
                            } else {
                                // A scale-only record: Distance Calibration ran but
                                // no motor sweep has yet. The scale is real; the
                                // curve is the identity.
                                info!(
                                    "Motor calibration record has no curve; counts_per_cm={}",
                                    scale.counts_per_cm()
                                );
                                calibration::set_motor_calibration(None, Some(scale)).await;
                                calibration::set_motor_run_id(0).await;
                                raise_event(Events::CalibrationDataLoaded(CalibrationKind::Motor, None)).await;
                            }
                        }
                        Ok(None) => {
                            info!("No motor calibration record found in flash");
                            calibration::set_motor_calibration(None, None).await;
                            calibration::set_motor_run_id(0).await;
                            raise_event(Events::CalibrationDataLoaded(CalibrationKind::Motor, None)).await;
                        }
                        Err(e) => {
                            error!("Failed to load motor calibration record: {}", defmt::Debug2Format(&e));
                            calibration::set_motor_calibration(None, None).await;
                            calibration::set_motor_run_id(0).await;
                            raise_event(Events::CalibrationDataLoaded(CalibrationKind::Motor, None)).await;
                        }
                    }
                }

                CalibrationKind::Distance => {
                    // The encoder scale travels inside the one Motor Calibration
                    // record, so this query reads the same key rather than the
                    // retired bare-factor key.
                    info!("Loading distance calibration from flash...");
                    match fetch_motor_record(&mut storage, &mut data_buffer).await {
                        Ok(Some(record)) => {
                            let scale = record.scale();
                            info!("Distance calibration loaded: counts_per_cm={}", scale.counts_per_cm());
                            publish_distance_scale(Some(scale)).await;
                        }
                        Ok(None) => {
                            info!("No distance calibration found in flash - nominal scale in force");
                            publish_distance_scale(None).await;
                        }
                        Err(e) => {
                            error!("Failed to load distance calibration: {}", defmt::Debug2Format(&e));
                            publish_distance_scale(None).await;
                        }
                    }
                }
                CalibrationKind::DmpBias => {
                    info!("Loading DMP bias snapshot from flash...");
                    match storage
                        .fetch_item::<DmpBiasWords>(&mut data_buffer, &StorageKey::DmpBias)
                        .await
                    {
                        Ok(Some(words)) => {
                            info!("DMP bias snapshot loaded: {}", words);
                            *DMP_BIAS_CACHE.lock().await = Some(words);

                            raise_event(Events::CalibrationDataLoaded(
                                CalibrationKind::DmpBias,
                                Some(CalibrationDataKind::DmpBias(words)),
                            ))
                            .await;
                        }
                        Ok(None) => {
                            info!("No DMP bias snapshot found in flash");
                            raise_event(Events::CalibrationDataLoaded(CalibrationKind::DmpBias, None)).await;
                        }
                        Err(e) => {
                            error!("Failed to load DMP bias snapshot: {}", defmt::Debug2Format(&e));
                            raise_event(Events::CalibrationDataLoaded(CalibrationKind::DmpBias, None)).await;
                        }
                    }
                }
            },

            FlashCommand::SaveDmpBias(words) => {
                info!("Saving DMP bias snapshot to flash...");

                let saved = match storage.store_item(&mut data_buffer, &StorageKey::DmpBias, &words).await {
                    Ok(()) => {
                        // Cache the snapshot only once flash holds it, so a failed
                        // write leaves the last durable snapshot in the cache for a
                        // Stop to restore. The save keeps the cache and the log
                        // only. Raising `CalibrationDataLoaded` here would re-enter
                        // the boot path and could discard the "Calibration saved"
                        // Result Report; only `GetData(DmpBias)` produces that
                        // event.
                        *DMP_BIAS_CACHE.lock().await = Some(words);
                        info!("DMP bias snapshot saved successfully");
                        true
                    }
                    Err(e) => {
                        error!("Failed to save DMP bias snapshot: {}", defmt::Debug2Format(&e));
                        false
                    }
                };

                // A late answer to a timed-out request must not wedge the task, so
                // a full reply slot drops this one.
                if DMP_BIAS_SAVE_REPLY.try_send(saved).is_err() {
                    warn!("DMP bias save result dropped — no waiter");
                }
            }

            FlashCommand::SaveMotorCurve(payload) => {
                let record = &payload.0;
                info!("Saving motor calibration curve to flash (run {})...", record.run_id());

                match storage
                    .store_item(&mut data_buffer, &StorageKey::MotorCurve, &payload)
                    .await
                {
                    Ok(()) => {
                        // Publish only once flash holds the record, so a failed
                        // write leaves the previous curve in force. This is the one
                        // leaf the driver and panel read. The reply below is the
                        // save confirmation; the run id is still published so the
                        // leaf names the durable record, but it is not the
                        // confirmation.
                        calibration::set_motor_calibration(record.curve(), Some(record.scale())).await;
                        calibration::set_motor_run_id(record.run_id()).await;
                        info!("Motor calibration curve saved successfully");
                        if MOTOR_CURVE_SAVE_REPLY.try_send(Ok(())).is_err() {
                            warn!("Motor curve save result dropped — no waiter");
                        }
                    }
                    Err(e) => {
                        error!("Failed to save motor calibration curve: {}", defmt::Debug2Format(&e));
                        // A full reply slot drops the answer rather than wedging the
                        // task; the caller then reports its bounded timeout.
                        if MOTOR_CURVE_SAVE_REPLY.try_send(Err("flash write failed")).is_err() {
                            warn!("Motor curve save result dropped — no waiter");
                        }
                    }
                }
            }

            FlashCommand::SaveDistanceScale(scale) => {
                info!("Saving distance calibration ({} counts/cm)...", scale.counts_per_cm());

                // One record holds both: a full record keeps its measurements and
                // only takes the new scale, so a distance run cannot discard a
                // measured curve. A fresh robot gets a scale-only record under run
                // id 0, because no sweep has produced a measurement run yet.
                let record = match fetch_motor_record(&mut storage, &mut data_buffer).await {
                    Ok(Some(record)) => record.with_scale(scale),
                    Ok(None) => MotorCalibrationRecord::scale_only(0, scale),
                    Err(e) => {
                        warn!(
                            "Existing motor record unreadable ({}); writing a scale-only record",
                            defmt::Debug2Format(&e)
                        );
                        MotorCalibrationRecord::scale_only(0, scale)
                    }
                };
                let payload = MotorCalibrationPayload(record);

                match storage
                    .store_item(&mut data_buffer, &StorageKey::MotorCurve, &payload)
                    .await
                {
                    Ok(()) => {
                        // Publish only once flash holds the record, so a failed
                        // write leaves the previous scale in force.
                        calibration::set_distance_scale(Some(scale)).await;
                        info!("Distance calibration saved: {} counts/cm", scale.counts_per_cm());
                    }
                    Err(e) => {
                        error!("Failed to save distance calibration: {}", defmt::Debug2Format(&e));
                    }
                }
            }
        }

        // Small delay to prevent tight loop
        Timer::after(Duration::from_millis(10)).await;
    }
}
