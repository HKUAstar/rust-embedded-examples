# Rust embedded examples

25 Embassy lessons for the RoboMaster Development Board Type C (STM32F407IG), corresponding to
[the official C examples][upstream]. Lessons 17–20 are smaller teaching examples, with the
omitted robot behavior listed in [PORTING.md](PORTING.md).

Each example owns its initialization: clocks, pins, DMA streams and interrupt bindings are
visible in that file. A local `init()` configures the hardware and returns owned drivers.
Examples with several drivers use a private `Peripherals` struct; single-driver examples return
that driver directly. Initialization is async where startup needs a delay or a CAN enable wait.
`main` spawns tasks and runs the lesson. Copy a lesson without editing a board abstraction.
Setup-only imports live inside `init()`. File-scope imports name the resources used by structs
and function signatures. Each entry point spells out `embassy_executor::Spawner` directly.
Shared code contains only BMI088 transactions, DBUS decoding, DJI frames and PI calculations
used unchanged by several lessons. The INS math lives in its example.

This project builds independently of fleet-2027. Firmware has no heap, host entry points, tests
or doctests. All examples forbid unsafe code. **None has been run on hardware.**

## Build and run

Install Rust through rustup and install probe-rs. `rust-toolchain.toml` selects Rust 1.99.1 and
`thumbv7em-none-eabihf`. Run from this directory so Cargo reads `.cargo/config.toml`:

```sh
cargo build --locked --release --examples
cargo run --locked --release --example flash_light
```

Replace `flash_light` with a name in the table. Cargo builds that image, then uses the configured
`probe-rs run --chip STM32F407IG` runner to flash it and display RTT output. Serial and sensor
lessons report measurements over RTT; `remote_printf` also writes USART1. RTT output is dropped
when no debugger reads it.

Every image starts at `0x08000000`. `memory.x` reserves sector 11, `0x080E0000..0x08100000`, for
the flash lesson. Programs can use the preceding 896 KiB. `flash_read_write` erases sector 11 on
every boot. There is no bootloader layout.

Each example links `panic-rtt-target` with `use panic_rtt_target as _;`. It logs the panic over
RTT, then stops in a critical section until reset. Panic logging switches RTT to blocking mode,
so it can wait for a debugger to drain the buffer. There is no watchdog or emergency shutdown:
PWM hardware and external motor controllers can retain the last command. The input timeouts
below apply while the control loop runs.

## Lessons

| Official folder | Example | What to observe |
|---|---|---|
| `0.new_cubemx_program` | [`new_program`](examples/new_program.rs) | Clock setup and a sleeping Embassy entry point |
| `1.light_led` | [`light_led`](examples/light_led.rs) | All three LED channels stay high |
| `2.flash_light` | [`flash_light`](examples/flash_light.rs) | All channels toggle every 500 ms |
| `3.tim_light` | [`tim_light`](examples/tim_light.rs) | TIM1 produces 500 ms updates, polled from an async task |
| `4.PWM_light` | [`pwm_light`](examples/pwm_light.rs) | TIM5 RGB brightness for `0x7F123456` |
| `4.homework_flow_led` | [`flow_led`](examples/flow_led.rs) | Blue → green → red, 500 ms per transition |
| `5.buzzer` | [`buzzer`](examples/buzzer.rs) | Buzzer frequency/duty sweep, updated every 250 ms |
| `5.servo_motor` | [`servo_motor`](examples/servo_motor.rs) | Seven servo signals alternate 500/2000 µs each second |
| `6.key_exit` | [`key_exti`](examples/key_exti.rs) | Green follows the active-low key after 20 ms debounce |
| `7.ADC_24V_power` | [`adc_power`](examples/adc_power.rs) | Supply voltage, die temperature and hardware revision in RTT |
| `8.USART_receive_and_send` | [`usart`](examples/usart.rs) | USART1/6 send `RoboMaster`, receive bytes into DMA buffers |
| `9.remote_control_dma` | [`remote_dma`](examples/remote_dma.rs) | DR16 channels, switches, mouse and keys in RTT |
| `9.remote_control_printf_pc` | [`remote_printf`](examples/remote_printf.rs) | Decoded DR16 controls on USART1, including missing-input reports |
| `10.flash_read_and_write` | [`flash_read_write`](examples/flash_read_write.rs) | Erase sector 11, write `RoboMaster\r\n`, verify readback |
| `11.ist8310` | [`ist8310`](examples/ist8310.rs) | Magnetometer identity/configuration checks and field in µT |
| `12.oled` | [`oled`](examples/oled.rs) | Moving checkerboard on a 128×64 I2C OLED |
| `13.spi_bmi088` | [`spi_bmi088`](examples/spi_bmi088.rs) | Acceleration, angular rate and temperature in RTT |
| `14.CAN` | [`can_motors`](examples/can_motors.rs) | Two DJI command groups, gated by remote and motor feedback |
| `14.PWM_SNAIL` | [`pwm_snail`](examples/pwm_snail.rs) | Remote throttle drives seven ESC signals, minimum pulse on loss |
| `15.freeRTOS_LED` | [`led_tasks`](examples/led_tasks.rs) | Three tasks take successive 500 ms RGB slots |
| `16.imu_temperature_control_task` | [`imu_temperature`](examples/imu_temperature.rs) | BMI088 temperature reports on data-ready edges; heater disabled |
| `17.chassis_task` | [`chassis`](examples/chassis.rs) | Remote-controlled mecanum rotor-speed PI loops |
| `18.ins_task` | [`ins`](examples/ins.rs) | Quaternion tilt correction and Euler angles; yaw drifts |
| `19.gimbal_task` | [`gimbal`](examples/gimbal.rs) | Two GM6020 encoder angle/rate cascades around the enable position |
| `20.standard_robot` | [`standard_robot`](examples/standard_robot.rs) | Compose chassis, gimbal, friction wheels and loader gating |

## Wiring

| Peripheral | Pins and settings |
|---|---|
| External crystal | 12 MHz; SYSCLK 168 MHz, APB1 42 MHz, APB2 84 MHz |
| Embassy time | TIM2, reserved in every image |
| RGB LED | PH12 red, PH11 green, PH10 blue, active high; PWM uses TIM5 CH3/2/1 |
| Key | PA0 / EXTI0, active low, pull-up |
| Buzzer | PD14 / TIM4 CH3 |
| Servo/ESC outputs | TIM1 CH1–4: PE9/PE11/PE13/PE14; TIM8 CH1–3: PC6/PI6/PI7, 50 Hz |
| Supply ADC | PF10 / ADC3 IN8, divider `(200 kΩ + 22 kΩ) / 22 kΩ` |
| Hardware revision | PC0/PC1/PC2, low-to-high bit order |
| USART1 | PB7 RX / PA9 TX, 115200 8N1 |
| USART6 | PG9 RX / PG14 TX, 115200 8N1 |
| DR16 | USART3 PC11 RX through onboard inverter, 100000 8E2, DMA1 stream 1 |
| IST8310 | I2C3 PA8 SCL / PC9 SDA, address `0x0E`; PG6 reset, 400 kHz |
| OLED | I2C2 PF1 SCL / PF0 SDA, address `0x3C`, 400 kHz, 128×64 page addressing |
| BMI088 | SPI1 PB3 SCK / PA7 MOSI / PB4 MISO, mode 3; PA4 accel CS, PB0 gyro CS |
| IMU temperature | PC4 / EXTI4 accelerometer data-ready; PF6 held low, heater PWM disabled |
| CAN1 | PD0 RX / PD1 TX, classic CAN at 1 Mbit/s |
| CAN2 | PB5 RX / PB6 TX, classic CAN at 1 Mbit/s |

USART1 uses DMA2 streams 7 TX and 5 RX. USART6 uses streams 6 TX and 2 RX. `remote_printf` uses
stream 7 for its USART1 output. BMI088 uses blocking SPI transactions; its requested 10 MHz
resolves to 5.25 MHz at the configured peripheral clock.

## Actuator lessons

`servo_motor` moves outputs immediately, alternating 500 and 2000 µs pulses. It does not use the
remote. Check the attached servo's pulse range before connecting it.

For `can_motors`, `pwm_snail`, `chassis`, `gimbal` and `standard_robot`, the right DR16 switch
(raw switch index zero) must be **up** to grant output permission. Remote frames expire after
100 ms. `pwm_snail` returns to a 1000 µs ESC pulse on loss; it does not perform high-throttle
calibration. It uses positive channel 3 values to increase the pulse, up to 1660 µs.

CAN1 feedback IDs `0x201..0x204` correspond to wheel motors 1–4. CAN2 IDs `0x205`, `0x206` and
`0x207` correspond to yaw, pitch and loader. Commands use CAN1 `0x200` and CAN2 `0x1FF`.
CAN initialization must finish within 100 ms; a timeout stops startup with an RTT panic.
Feedback expires after 20 ms. A missing wheel zeros the whole chassis group; either missing
gimbal motor zeros both gimbal axes. Only standard, non-remote, eight-byte feedback is accepted.
The raw CAN lesson additionally requires all three CAN2 motors before commanding that group.

`can_motors` sends 500 raw counts per commanded motor while permitted. `chassis` and
`standard_robot` cap wheel commands at 2000 counts. Wheel order is front right, front left,
rear left, rear right, with signs in the source. Motor velocity is rotor rad/s; these examples do
not infer wheel speed through a gearbox or tune gains for your mechanism.

`chassis` uses channel 1 forward, channel 0 strafe and channel 2 spin. `standard_robot` uses
channel 4 for spin, leaving channels 2/3 for gimbal yaw/pitch. The gimbal captures its current
position when enabled and limits targets to ±0.5 rad yaw and ±0.15 rad pitch relative to it.
Its commands are capped at 2000 raw counts. Neither gimbal example uses the IMU to hold a world
orientation or discovers mechanical stops. Loop gaps of 10 ms or more zero control outputs and
reset their controller state.

In `standard_robot`, the left switch up requests 1200 µs friction-wheel pulses on PE9/PE11.
The loader requires fresh remote/gimbal/loader data, mouse-left, and one second with friction
wheels requested. That is a time delay, not proof of wheel speed. Loader output is capped at
1000 raw counts. Referee heat and power budgets, ammunition sensing and jam recovery are omitted.

`imu_temperature` only reads and reports temperature. PF6 is held low, TIM10 is not started,
and the heater PWM setup and PI control remain commented out in the example.

## Validate

```sh
cargo fmt --all --check
cargo clippy --locked --lib --examples -- -D warnings
cargo build --locked --release --examples
cargo deny --locked check
cargo doc --locked --no-deps --lib
```

Install the dependency checker with `cargo install cargo-deny --version 0.20.2 --locked`.
CI checks formatting, MCU lints, release builds and dependencies. There is no host-test workflow.
Builds check types and linking; hardware behavior and controller tuning remain unverified.
Use [BENCH.md](BENCH.md) to record hardware checks and [PORTING.md](PORTING.md) to compare with C.

Licensed under MIT OR Apache-2.0. The BMI088 protocol implementation was adapted from
[fleet-2027's shared driver][bmi088], under the same licenses. The upstream C sources, RTOS,
startup files, fonts, logo artwork and precompiled AHRS libraries are not included.

[upstream]: https://github.com/RoboMaster/Development-Board-C-Examples
[bmi088]: https://github.com/trident-rm/fleet-2027/blob/main/shared/devices/src/bmi088.rs
