# Porting notes

Reference: [Development-Board-C-Examples][upstream], commit
`59d12b1adcd321dbf1f9e9166aef5eb95ab657bf`. The [lesson table](README.md#lessons) maps every
upstream example folder to its Rust source. These are implementations of the lessons using
Embassy. Lessons 17–20 intentionally demonstrate smaller subsystems rather than the original
robot's full behavior.

## Structure and API mapping

| C mechanism | Rust mechanism |
|---|---|
| `HAL_Init`, `SystemClock_Config` | Explicit `embassy_stm32::Config` and `init` in each example |
| `MX_GPIO_Init`, `HAL_*_MspInit` | Pin ownership passed to the peripheral constructors in that file |
| HAL interrupt callbacks and global handles | `bind_interrupts!` and Embassy driver futures |
| `HAL_GPIO_WritePin` / `TogglePin` | `Output::set_high`, `set_low`, `toggle` |
| `__HAL_TIM_SetCompare` | PWM channel duty or low-level timer compare value |
| `HAL_Delay`, `osDelay` | `Timer::after_*().await` |
| `osThreadCreate`, `xTaskCreate` | Embassy tasks spawned with owned peripherals |
| Double-buffer DBUS DMA callback | Idle-delimited DMA receive into a 36-byte buffer, validate 18-byte frames |
| `get_remote_control_point` | Parsed `Remote` value passed through a latest-value signal |
| `CAN_cmd_chassis`, `CAN_cmd_gimbal` | Big-endian four-slot `command` frames |
| `get_motor_measure` | `Motor::decode`, retaining the HAL reception timestamp |
| `flash_erase_address`, `flash_write_single_address`, `flash_read` | Safe blocking `Flash` operations on byte slices |
| `BMI088_init`, `BMI088_read` | Shared embedded-hal SPI driver, SI samples |
| Upstream PID structs | Bounded PI state, actual elapsed seconds, no generic controller framework |

Every firmware uses `no_std`, `no_main` and `forbid(unsafe_code)` directly. Clock configuration is
repeated in a local `init()` so each lesson can be read and edited on its own. It returns owned
drivers, grouped in a private `Peripherals` struct when there is more than one. Reset pins and PWM
pin guards return with their drivers so they stay configured while the lesson runs. Tasks start
in `main` after initialization finishes. There is no shared board initializer,
LED wrapper, host entry point, test harness or doctest. Shared functions only serve unchanged
protocol or math work needed by several lessons.

## Peripheral differences

- **Clock setup:** 12 MHz HSE and 168 MHz SYSCLK as upstream, with PLLQ providing 48 MHz.
  TIM2 is reserved for Embassy time. The Rust HAL supplies the peripheral drivers.
- **LED on/blink:** outputs open low. The on lesson immediately drives them high. The blink
  lesson's first toggle drives them high, whereas upstream initializes high and first toggles
  low. Both have a 500 ms nominal half-period. CPU-dependent busy loops are replaced with timers.
- **TIM1 LED:** TIM1 generates the 2 Hz update events. The task polls its update flag once per
  millisecond instead of installing an application ISR. The extra observation latency can be up
  to one polling interval. TIM2 still schedules the polling task.
- **PWM LED:** the requested carrier is 1282 Hz, close to upstream's 84 MHz / 65536 period.
  Brightness uses the actual timer period with the upstream `alpha × color / 65536` scale.
  The flow lesson preserves blue → green → red with 500 one-millisecond interpolation steps.
- **Buzzer:** preserves the prescaler/duty sweep intent and 250 ms updates. Frequencies are
  rounded to integer Hz and then to realizable timer settings, rather than setting PSC directly.
- **Servo:** all seven channels are enabled. Upstream starts TIM1 channel 1 twice and never
  starts channel 4 despite writing it; the Rust version enables channel 4 on PE14.
- **Key:** both edges wake the task, followed by 20 ms debounce. The initial key state is also
  reflected in the LED; the upstream foreground loop waits for its first interrupt.
- **ADC:** preserves 200-sample 1.2 V VREF calibration, divider and typical die-temperature
  formula. It uses 480-cycle acquisitions instead of three cycles to allow internal sensors and
  the supply divider to settle. The divider and temperature calibration are not bench measured.
- **USART:** both ports transmit the same text and receive independently using DMA. Received
  bytes are shown on RTT rather than through LED callback state. Transmissions alternate with
  a nominal 100 ms offset. The on-wire text remains `RoboMaster\r\n`.
- **DBUS:** explicitly 100000 baud, eight data bits, even parity, two stop bits. Some upstream
  projects select a HAL word length/parity combination that leaves seven payload bits, or omit
  parity. The Rust settings use the DR16 framing. Invalid packets do not renew freshness. Switch
  index zero is byte 5 bits 4–5; index one is
  bits 6–7. Physical left/right labels vary in upstream comments; check them on the receiver.
  The printf lesson reports at 10 Hz, includes all mouse axes with matching format arguments,
  and says when input is missing; upstream reports once a second and has mismatched arguments.
- **Flash:** preserves the final sector, payload and readback. That sector is reserved by the
  linker for every image. Erase/write/readback are checked, rather than just stored in globals.
- **IST8310:** preserves reset, identity, four register values and µT scale. It polls data-ready
  status and reads at 100 Hz rather than doing the sample work in the GPIO callback.
- **OLED:** uses the same bus/address and panel size. A moving checkerboard replaces the
  RoboMaster logo and font assets; this lesson demonstrates commands and page transfers.
- **BMI088:** the fleet driver resets and verifies both chips, then reports SI units and chip
  axes. Accelerometer is ±6 g at 1600 Hz; gyro is ±2000 °/s at 2000 Hz. SPI resolves to 5.25 MHz,
  rather than changing prescalers after startup. Read failures are visible in RTT.
- **CAN:** standard 1 Mbit/s frames, same group IDs and big-endian command slots. Remote input
  and current motor replies gate 500-count outputs, replacing unconditional 4000/10000-count
  commands. Automatic retransmission is disabled; at most 16 received frames per bus are handled
  per control iteration. Protocol loss detection is distinct from CPU-fault shutdown.
- **Snail ESCs:** preserves the seven 50 Hz outputs and positive channel-3 throttle mapping.
  The high-throttle calibration sequence is omitted. The right switch must be up, and stale
  input returns all outputs to 1000 µs. The maximum commanded pulse is 1660 µs.
- **LED tasks:** preserves the 1500 ms sequence: red, green and blue each high for one 500 ms
  slot. Identical adjacent low states become one 1000 ms wait. Tasks share a cooperative
  executor rather than FreeRTOS priorities and preemption; relative delays can drift.
- **IMU temperature:** reports BMI088 temperature using the PC4 data-ready interrupt. PF6 is
  held low. The TIM10 PWM setup and 40 °C PI control are commented out; heating is disabled.

## Smaller control lessons

| Lesson | Included | Omitted or changed |
|---|---|---|
| `17.chassis_task` | Four DJI rotor-speed PI loops, mecanum mixing, remote permission, feedback freshness | Gimbal-follow modes, keyboard ramping, gearbox/wheel geometry, original gains and physical velocity tuning |
| `18.ins_task` | Scalar-first quaternion, gyro integration, gravity correction, Euler output at 10 Hz | Magnetometer fusion, bias/thermal calibration, upstream mounting rotation, exact Mahony tuning, heater task |
| `19.gimbal_task` | Two GM6020 encoder-relative angle/rate cascades, enable-position reference, limited targets | IMU stabilization, automatic stop calibration, flash offsets, auto-aim, original gains/modes |
| `20.standard_robot` | Chassis and gimbal loops plus two ESC outputs and a speed-controlled loader | Referee/UART/USB protocols, power/heat limiting, UI, navigation, calibration, jam recovery, production arming supervisor |

The control examples use 2 ms ticks and actual elapsed time. Gaps of 10 ms or more inhibit their
outputs. CAN startup times out after 100 ms and reports a panic instead of continuing.
Remote input expires at 100 ms; motor data expires at 20 ms using the reception time.
On input loss the relevant PI state resets. Gimbal re-enable captures a new local reference;
it does not return to a calibrated home. In the INS lesson yaw is unreferenced and drifts.

The temperature lesson skips the first edge after a sample gap of 20 ms or more. Heater control
is commented out. The shared PI constructor checks finite nonnegative gains and a finite
positive output limit. Invalid measurements or time steps reset its output to zero.

The combined lesson demonstrates composition in one loop. Its one-second friction-wheel delay
is not tachometer feedback. All control gains, signs and output limits need mechanism-specific
bench work. `panic-rtt-target` logs the panic and stops in a critical section; it does not force
actuator outputs off. Each example imports the crate instead of defining a handler.

## Verification

No upstream automated tests were found for these demonstrations. Per the project scope, no
host tests or host simulation are included. [BENCH.md](BENCH.md) lists the observable checks for
every lesson. Formatting, MCU Clippy, release linking and dependency checks are the automated
validation; they do not verify hardware behavior or dynamics.

On 2026-10-31 with Rust 1.99.1, MCU Clippy passed with warnings denied, all 25 release images
linked, dependency policy checks passed, and library documentation built. Formatting, local
file/documentation hooks and the GitHub Actions workflow syntax check also passed. No hardware
was flashed or exercised during these checks.

The BMI088 implementation comes from fleet-2027 under MIT OR Apache-2.0. Its host tests and
project-specific validation macro were removed; heater temperature checks are explicit in the
lesson. No upstream C implementation, logo/font asset, generated startup file, FreeRTOS source
or precompiled AHRS library is bundled here.

[upstream]: https://github.com/RoboMaster/Development-Board-C-Examples/tree/59d12b1
