# DMP compass calibration replaces host magnetometer calibration

The existing `Mag` procedure computes host-side hard/soft-iron and motor-interference corrections (`MagCalibration`) and applies them only to `ImuReadings::calibrated_mag`. The ICM-20948 DMP reads the AK09916 directly through its I2C master and fuses that raw vector, so none of those host values can reach the DMP's 9-axis yaw. A second, DMP-specific calibration path therefore existed in concept but not in the UI. We are keeping six-axis DMP as the default and using nine-axis occasionally, so the 9-axis path needs a calibration that actually changes DMP yaw and survives a power cycle.

**Status:** accepted.

**Considered Options**

- **Keep the host `Mag` procedure and add a DMP ceremony beside it.** Rejected. The host path would have a menu entry whose values cannot affect the DMP, while a second magnetometer calibration does. The motor-interference vectors are host-only and cannot be fed to the DMP, so the two procedures would look equivalent to an operator but calibrate different things.
- **Use host-corrected magnetometer data as the DMP's input.** Rejected. The DMP owns the AK09916 read path; there is no supported API to inject a corrected vector into its fusion.
- **Persist only the DMP compass bias.** Rejected in favour of the nine-word snapshot: the ceremony already includes the still periods that let the gyroscope and accelerometer engines converge, and the same save/restore path covers all three. The snapshot remains cheap.
- **Verify at one compass heading.** Rejected. A single heading at the reference can pass a 2:1 yaw gain error, which is one of the symptoms under investigation. Two headings 90° apart distinguish a constant frame offset from a gain/mapping error.
- **Save after the movement steps without a compass check.** Rejected. The DMP's `compass_accuracy` and the visible heading are the only evidence that the engine converged; saving an unconverged bias would make the next boot worse.

**Decision**

- Retire the host `Mag` calibration: remove the host `MagCalibration` application, the motor-interference measurement, `ImuFlags.mag`, and the `M cal` row. The IMU Test Readout shows the DMP-calibrated magnetometer (`M dmp`) instead.
- Replace `Calibrate → Mag` with a **DMP Compass Calibration** Procedure (`DmpCompassCalibration`, label `DMP Mag`). Starting it does not start the DMP calibration engine; the engine runs continuously while nine-axis fusion is enabled. The Procedure guides the motion and snapshots the result.
- The Procedure is eight prompts: six movement steps (flat still, flat spin, pitch tilt, roll tilt, inverted spin, flat still) followed by two compass checks at magnetic north (0°) and east (90°). The final checks require `compass_accuracy == 3`, heading change ≤2° over 2 s, and agreement within ±10°. `Done` advances the movement steps; the final checks offer `Done` / `Redo`; the header `Stop` aborts.
- On success, the Procedure reads the nine raw DMP bias words (`GYRO_BIAS`, `ACCEL_BIAS`, `CPASS_BIAS`), saves them with a version and checksum as a **DMP Bias Snapshot**, and restores them after `dmp_init` on the next boot. The DMP's internal accuracy/covariance state is not restorable and re-validates with motion.
- `Redo` restarts the whole movement sequence with the compass bias cleared; the gyroscope and accelerometer biases may be restored from the last snapshot. A failed or stopped ceremony never overwrites the last good flash snapshot; if a snapshot exists, it is restored to the DMP.
- The displayed heading is a **Compass Heading**: magnetic north, not true north, derived from DMP yaw by one fixed mounting/sign conversion. No declination is applied.
- Flash and status are renamed accordingly: `DmpBiasSnapshot` replaces `ImuFlags`, `dmp_cal_status` replaces `mag_cal_status`, and System Info shows `DMP Cal: Loaded/Missing/Unknown`.

**Consequences**

- Motor-interference compensation is gone from the 9-axis path. The DMP cannot consume it, so heading while motors run may degrade or be rejected by the DMP's anomaly handling; motors-off remains the clean 9-axis condition.
- The first boot after this change finds no DMP snapshot and reports `DMP Cal: Missing`; the operator must run the ceremony once. There is no host calibration value to migrate because the host values were never persisted.
- The 90°→45° yaw gain symptom is not fixed by this decision. The two-heading check will expose it, but a persistent gain error remains a conversion/mount/fusion defect rather than a calibration failure.
- A new Prompt screen and a `DmpCalibrationSnapshot` telemetry source are required; `StatusView` keeps its single-action Stop contract.
