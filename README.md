# explorer-robot

Evolving project for a tracked robot platform with LiDAR-based
autonomous navigation and sensor-driven obstacle avoidance.

Built in Rust on [Embassy](https://embassy.dev/) for the RP2350 microcontroller.
This iteration builds on hardware experience from [simple-robot](https://github.com/1-rafael-1/simple-robot), adding a
360° spinning LiDAR (COIN-D6), a VL53L0X rangefinder (front-down), ICM-20948 IMU over
SPI, a ST7789 TFT display, and a Grove Vision AI V2 camera module — as much as i manage on custom PCB hardware. In turn ultrasoic/servo sweeping sensor array is ditched, so are the IR sensors.

## Status

Active development. Core subsystems (drive, perception, UI, calibration) are
implemented with synthetic sensor stubs. Real sensor drivers are planned.

The big ToDos, in no particuar order:

+ Make a new chassis or make a variation of the simple-robot chassis, to accomodate the new motors and also ball bearinggs.
+ Make a async-capable VL53L0X driver and intergrate that
+ Make a D6 lidar driver and integrate that
+ Full breadboard demonstrator to see if the firmware is botched
+ Make a new schematic adapting from simple-robot
+ Design a new PCB from that. Depending on how mad I feel, maybe ditch some dev boards in favor of smd design.



## Hardware

- **MCU:** Raspberry Pi RP2350B (dual-core Cortex-M33)
- **Motors:** 2× JGB37-520 6V DC with hall encoders, 165RPM
- **Bearings:** 6× 6802-2RS (DIN 625 6802-2RS)
- **Motor driver:** TB6612FNG dual 
- **LiDAR:** COIN-D6 360° spinning dTOF (core1, UART0 + power MOSFET reserved, currently stubbed)
- **AI Cam:** Grove Vision AI V2 (core0, UART1 + power MOSFET reserved, not yet integrated)
- **Rangefinder:** 1× VL53L0X ToF, front-down (stair/drop detection), on shared I2C0 bus (currently stubbed)
- **IMU:** ICM-20948 9-axis over dedicated SPI bus
- **Display:** ST7789 240×240 TFT over SPI
- **Input:** EC11 rotary encoder with push button
- **Power:** 2S LiPo (8.4V max)

## License

MIT
