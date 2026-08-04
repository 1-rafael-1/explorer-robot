# COIN-D6 LiDAR Data Format Specification

**Version:** 1.0  
**Date:** 2024-11-08

---

## 1. Operating Modes

The COIN-D6 LiDAR supports command-based start/stop.

> **⚠ Observed behavior:** On the unit tested, the sensor **auto-starts at power-up** without requiring an explicit start command. It sends the device info packet immediately after power-on, followed by scan data packets once the motor stabilizes.
>
> **Practical control:** The start/stop commands below may not be needed at all. Use a **MOSFET switch** on the power rail to turn the sensor on and off — on power-up it auto-starts, on power-down it's off. This is simpler and more reliable than command-based control.

### Start

| Direction | Bytes |
|-----------|-------|
| Send | `0xAA 0x55 0xF0 0x0F` |
| OK Ack | `0xA5 0x5A 0x50 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xA8` |
| Error Ack | `0xA5 0x5A 0x55 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xE9` |

### Stop

| Direction | Bytes |
|-----------|-------|
| Send | `0xAA 0x55 0xF5 0x0A` |
| OK Ack | `0xA5 0x5A 0x55 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xAD` |
| Error Ack | `0xA5 0x5A 0x55 0x07 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xE9` |

> **Note:** The COIN-D6 LiDAR only outputs scan data once the rotation speed has stabilized. During the speed-adjustment phase, pairs of `0xFE` or `0xFF` bytes may appear as speed-control commands.
>
> **Observed startup sequence (power-on):**
> 1. `0xFF 0xFA 0xFA` — startup/speed-adjustment preamble (note: `0xFA` appears in addition to the documented `0xFE`/`0xFF`)
> 2. Device Info packet (type `0x01`) — sent immediately
> 3. Scan Data packets (type `0x81`) — begin flowing once the motor reaches 10 Hz

> **Note:** The COIN-D6 uses an internal closed-loop speed control module with a fixed rotation rate of **10 Hz**.

---

## 2. Packet Format

All packets follow this outer envelope:

```
A5   5A   XX   XX   YY   YY   ZZ   [Data...]
│    │    │         │         │
│    │    │         │         └── Type (1 byte)
│    │    │         └──────────── Checksum (2 bytes, LE)
│    │    └────────────────────── Length of Data area (2 bytes, LE)
│    └─────────────────────────── Header byte 2
└──────────────────────────────── Header byte 1
```

| Field | Size | Description |
|-------|------|-------------|
| Header | 2 bytes | Fixed `0xA5 0x5A` |
| Length | 2 bytes | Length of the Data field, **little-endian**. E.g. `0x05 0x00` = 5 bytes. |
| Checksum | 2 bytes | Sum of all bytes in the packet **except** the checksum field itself. |
| Type | 1 byte | `0x81` = Upload scan data; `0x01` = Upload device info |
| Data | N bytes | Payload, length specified by the Length field |

---

## 3. Device Info Packet (Type `0x01`)

This is the first packet sent by the sensor after power-up. It describes the device identity and capabilities.

### Envelope

```
A5   5A   14   00   XX   XX   01   [20-byte Data area]
```

- Length = `0x0014` = 20 bytes
- Type = `0x01`

### Data Area (20 bytes)

| Byte Offset | Length | Value | Description |
|-------------|--------|-------|-------------|
| 1–7 | 7 bytes | `43 4F 49 4E 2D 44 36` | Model name: "**COIN-D6**" (ASCII) |
| 8–12 | 5 bytes | `00 00 00 00 00` | Reserved |
| 13–14 | 2 bytes | `00 00` | Angular offset between data zero-angle and physical zero-angle. Hardware-compensated on COIN-D6; **ignore** this value. |
| 15 | 1 byte | `0x00` | Rotation direction: `0x00` = clockwise |
| 16 | 1 byte | `0x01` | Angle correction flag: `0x01` means angle correction must be implemented in the host driver |
| 17–19 | 3 bytes | `00 00 00` | Reserved (default 0) |
| 20 | 1 byte | `0x01` | Software version: Rev 1 |

---

## 4. Scan Data Packet (Type `0x81`)

This packet carries the point cloud data. It is the second packet sent after power-up (once the motor is stable).

### Envelope

```
A5   5A   LL   LL   XX   XX   81   [N-byte Data area]
```

- Type = `0x81`
- The Data area may contain one or more concatenated data blocks; each block is parsed independently.

### Data Block Format

```
PHL  PHH  M&T  LSN  FSAL  FSAH  LSAL  LSAH  CSL  CSH  S1_L  S1_2nd  S1_H  ...  Sn_L  Sn_2nd  Sn_H
```

| Field | Size | Name | Description |
|-------|------|------|-------------|
| PH | 2 bytes | Packet Header | Fixed value `0x55AA`, **little-endian** (on wire: `AA 55`) |
| M&T | 1 byte | Mode & Type | Upper 7 bits = scan frequency; bit 0 = packet type |
| LSN | 1 byte | Sample Count | Number of sample points in this block. For a start packet, LSN = 1. |
| FSA | 2 bytes | Start Angle | Angle of the first sample, little-endian |
| LSA | 2 bytes | End Angle | Angle of the last sample, little-endian |
| CS | 2 bytes | Checksum | XOR checksum over the block (see Section 5) |
| Si | 3 bytes each | Sample Data | LSN samples, each 3 bytes (see Section 6) |

### M&T Field

```
Bit:  7    6    5    4    3    2    1    0
      ├────────── M ──────────┤         ├─ T
```

| Sub-field | Bits | Description |
|-----------|------|-------------|
| **M** | bits 7:1 | Scan frequency: `M = (M&T >> 1)`. Valid only in start packets (T=1); defaults to 0 in point cloud packets. |
| **T** | bit 0 | Packet type: `0` = point cloud data; `1` = **start packet** (marks beginning of a full 360° revolution) |

When T = 1, the block is a start packet: it marks the beginning of one revolution of point cloud data, and contains exactly **one** sample (LSN = 1).

### Angle Fields (FSA / LSA)

Each angle field is 2 bytes, little-endian, with this bit layout:

```
FSA_L / LSA_L:   Angle bits [7:1]   |   C[0]=1 (check bit, always 1)
FSA_H / LSA_H:   Angle bits [15:8]
```

**Formulas:**

```
Angle_fsa = (FSA_raw >> 1) / 64.0    [degrees]
Angle_lsa = (LSA_raw >> 1) / 64.0    [degrees]
```

**Intermediate angle** for sample i (i = 2, 3, ..., LSN−1):

```
Angle(i) = Angle_fsa + (Angle_lsa - Angle_fsa) / (LSN - 1) × (i - 1)
```

---

## 5. Checksum (CS Field)

The CS field uses a **double-byte XOR** over the data block. The CS field itself is **not** included in the XOR operation.

Since each sample Si is 3 bytes, the third byte of each Si is **zero-padded to 16 bits** (MSB = 0x00) before XOR.

```
CS = D1 ⊕ D2 ⊕ D3 ⊕ D4 ⊕ ... ⊕ Dn
```

Where D1..Dn are the 2-byte words of the block (excluding the CS field), and all comparison units are 2 bytes wide.

The XOR operation is order-independent.

---

## 6. Sample Data (Si)

Each sample is **3 bytes**, encoding both distance and intensity:

```
Si_L    : byte 0 (low byte)
Si_2nd  : byte 1 (middle byte)
Si_H    : byte 2 (high byte)
```

### Intensity

Intensity represents the effective echo strength of the sample point. Length: 8 bits.

```
Intensity = (Si_2nd & 0x03) × 64 + (Si_L >> 2)
```

### Distance

Distance is the measured range in **millimeters**. Length: 14 bits.

```
Distance = Si_H × 64 + (Si_2nd >> 2)
```

### High-Reflectivity Flag

| Bit | Meaning |
|-----|---------|
| `Si_L[0]` (bit 0) | High-reflectivity flag: `1` = high-reflectivity material (e.g. 3M retroreflective tape, metal, or materials with >300% reflectivity). `0` = normal reflectivity (<100%). |
| `Si_L[1]` (bit 1) | Reserved |

---

## Revision History

| Date | Version | Description |
|------|---------|-------------|
| 2024-11-08 | 1.0 | Initial release |
