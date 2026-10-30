# Bench checks

**Status: not run on hardware.** Record board revision, source revision, power source, probe,
attached devices and measurements for each run. The checks below are the remaining work,
not evidence of a successful hardware port.

Use the [wiring and controls](README.md#wiring). For motor lessons, establish signs and command
limits with the mechanism supported before enabling it. The servo lesson moves immediately.
The flash lesson erases the final flash sector on each reset.

| Example | Observable check |
|---|---|
| `new_program` | Debugger reaches the executor; clocks resolve to 168/42/84 MHz |
| `light_led` | PH12, PH11 and PH10 stay high |
| `flash_light` | All three pins toggle together, nominal 500 ms half-period |
| `tim_light` | TIM1 generates 2 Hz updates; LED edges follow within a polling interval |
| `pwm_light` | TIM5 carrier near 1282 Hz; RGB duties match alpha-scaled `0x123456` |
| `flow_led` | Blue → green → red repeats, nominal 500 ms per transition |
| `buzzer` | Scope PD14; frequency/duty change every 250 ms and wrap correctly |
| `servo_motor` | All seven pins produce 50 Hz; alternate 500/2000 µs each second |
| `key_exti` | Green follows PA0 low/high after debounce; switch bounce does not latch a wrong state |
| `adc_power` | Compare reported voltage with a meter; check die temperature and revision straps |
| `usart` | Capture exact text on both TX pins; inject bytes/idle gaps into each RX independently |
| `remote_dma` | Check all channels, switches, mouse/buttons/keys; loss is reported |
| `remote_printf` | Capture USART1 at 115200 8N1; check live data and missing-input message |
| `flash_read_write` | RTT confirms erase/write/read; independently inspect sector 11 and program bounds |
| `ist8310` | Identity `0x10`, configuration readback; field changes when rotated near a known field |
| `oled` | Panel shows checkerboard and inverts once a second, with correct page/column orientation |
| `spi_bmi088` | Stationary acceleration norm near 9.81 m/s²; check gyro axes/rate and temperature |
| `can_motors` | Capture IDs/slots/endian values; no command permission until remote and feedback are fresh |
| `pwm_snail` | Scope all seven outputs first; switch/loss gives 1000 µs, throttle reaches at most 1660 µs |
| `led_tasks` | Successive 500 ms red/green/blue slots; measure drift and transition skew |
| `imu_temperature` | Confirm PF6 stays low with no PWM; check temperature reports and missing/resumed PC4 edges |
| `chassis` | Check each motor's sign and rotor-speed response; losing one feedback zeros all four outputs |
| `ins` | Static tilt converges; known rotations follow gyro signs; confirm expected yaw drift |
| `gimbal` | Capture reference on enable; test limits and angle-wrap crossing; either missing motor zeros both |
| `standard_robot` | Check drive/aim/spin controls; friction gate, one-second delay and loader permission/loss behavior |

For CAN actuator examples, capture the transition to zero after remote loss (100 ms bound plus
loop/transport latency) and feedback loss (20 ms bound plus loop/transport latency). Test loss of
just one device, malformed frames and disabled switches. Observe externally what the motor does
when the MCU is halted: input-loss behavior does not establish fault-stop behavior.

For the temperature lesson, confirm PF6 remains low during startup, normal reporting, missing
edges and read errors. Heater setup and control are commented out; no heating test is enabled.

## Run records

No runs recorded yet.
