# Drive base firmware (ESP32-S3)

Runs the four mecanum wheels: a 100 Hz FreeRTOS task reads the encoders, runs one
feedforward + PI velocity loop per wheel, and drives two MDD10A boards. Commands arrive
either as typed text (for bench testing) or as ROS 2 topics through micro-ROS.

## Layout, and why it's split this way

| Path | What it is | Why it lives there |
| --- | --- | --- |
| `lib/canbot_core/` | PID, speed estimator, watchdog, stall detector, text parser | Plain C++, no Arduino calls, so it compiles on your laptop and is unit tested there |
| `src/drive_controller.*` | The 100 Hz control task and its mailbox | The only code with timing guarantees; everything else just sets its targets |
| `src/motor_driver.*` | One MDD10A channel (PWM + DIR) | Hides the LEDC API difference between Arduino-ESP32 core 2.x and 3.x |
| `src/transport_text.cpp` | Bring-up protocol over USB | Built only in `env:bringup` |
| `src/transport_microros.cpp` | ROS 2 topics via micro-ROS | Built only in `env:microros` |
| `include/config.h` | Every pin, constant and default gain | One place to change when the hardware changes |
| `test/test_core/` | 22 unit tests (Unity) | `pio test -e native` |
| `tools/` | Python bench tools (pyserial) | Step tests, feedforward fit, CSV logs |

The two transports are swapped by `build_src_filter` in `platformio.ini`, so the control
code you tune on the bench is byte-for-byte the code that runs under ROS.

## Where to build and flash

PlatformIO, either the VS Code extension or `pip install platformio`.

- **On the Pi 5 (recommended):** the robot's own computer flashes its microcontroller over
  the same USB cable it uses at runtime. No Windows USB passthrough involved.
- **In WSL2:** works for building and `pio test -e native`. Flashing needs the ESP32's USB
  device forwarded into WSL with `usbipd-win`.
- `env:microros` additionally needs `git`, `cmake` and `python3-pip`
  (`sudo apt install git cmake python3-pip`). Its first build compiles the micro-ROS client
  library from source and takes several minutes. If that build fails with
  `No module named pip`, PlatformIO's private Python was created without pip; run
  `~/.platformio/penv/bin/python -m ensurepip` once and build again.
- The platform is pinned to pioarduino 55.03.312 (Arduino-ESP32 core 3.3.12). Both
  firmware builds were compile-checked against exactly that version.

Use the board's port labeled **USB** (native USB), not **UART**. On Linux it appears as
`/dev/ttyACM*` with vendor ID `303a`. If uploads fail, hold BOOT, tap RST, release BOOT,
and upload again.

## Bring-up, one unknown at a time

Do these in order. Each step proves one thing, so when something breaks you know which
piece did it. **Wheels off the ground** (robot on a box) until step 6.

**0. Unit tests.** `pio test -e native`. All 22 should pass before anything touches hardware.

**1. Wiring checks before power.**
- 10 kΩ pull-down on every PWM pin (the MDD10A sees 0% while the ESP32 boots).
- Encoders powered from **3.3 V**, not 5 V: the S3's pins are not 5 V tolerant.
- Motor rail fused, E-stop within reach.

**2. Flash and talk to it.** `pio run -e bringup -t upload`, then
`pio device monitor` and type `?`. You should see the help text and the gains.

**3. Encoders (sets `kEncoderInvert` and `kCountsPerWheelRev`).**
`cd tools && python3 bench.py --port /dev/ttyACM0 counts`, then turn each wheel by hand
in the direction that would drive the robot **forward**. Every count must go up; flip
`kEncoderInvert[i]` for any that go down. Then turn one wheel exactly 10 turns and divide
its count by 10: that's `kCountsPerWheelRev`. Hand-turning comes first because it's the
one test where you know the true direction without trusting any code.

**4. Motor direction (sets `kMotorInvert`).**
`python3 bench.py duty --wheel FL --duty 0.3`, then FR, RL, RR. Each wheel must spin
forward and the script prints `OK`. If a wheel spins backward, flip its `kMotorInvert`.
Reflash after editing `config.h`.

**5. Feedforward (sets `kff` and `kstatic`).** `python3 fit_feedforward.py`. It runs every
wheel through a ladder of duties both ways, fits `duty ≈ kff·speed + kstatic`, and prints
the numbers and the line to paste into `kDefaultGains`.

**6. Tune the PI terms.** Try gains live, then copy the keepers into `config.h`:
1. `python3 bench.py gains --wheel -1 0 0 <kff> <kstatic>` (feedforward only), then
   `python3 bench.py step --target 8`. Expect the right shape with 10–20% steady error.
2. Raise `ki` until that error is gone within about half a second.
3. Raise `kp` for a faster rise; stop when overshoot passes ~10%.
4. Repeat at a low target (`--target 2`) and on the floor, where load and friction differ.

Every run writes a CSV to `tools/logs/`. Plot target vs. velocity (PlotJuggler, a
spreadsheet, or pandas); you tune by looking at curves.

**7. Prove the safety paths.**
- Unplug USB mid-run: wheels stop within 300 ms.
- Hold a wheel with a gloved hand at `--duty 0.6`: after 1 s the stall fault trips and that
  wheel stays off until it's commanded to zero (or you send `C`).
- E-stop (once wired, set `kEstopSensePin = 40`): pressing it stops everything; releasing it
  does **not** restart the wheels until a zero command or `C` arrives.

**8. Switch to micro-ROS.** `pio run -e microros -t upload`, then on the Pi:

```bash
docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 115200

# second terminal
ros2 topic list                         # /canbot/wheel_states, /canbot/wheel_commands, /canbot/base_faults
ros2 topic hz /canbot/wheel_states      # ~50 Hz
ros2 topic pub -r 20 /canbot/wheel_commands sensor_msgs/msg/JointState \
  "{name: [wheel_fl_joint, wheel_fr_joint, wheel_rl_joint, wheel_rr_joint], velocity: [2.0, 2.0, 2.0, 2.0]}"
```

Stop the `pub` and the wheels stop after 300 ms. Stop the agent and restart it: the board
reconnects within about a second, with no reset needed.

## Protocol references

**Text (env:bringup).** Full reference in `lib/canbot_core/src/text_protocol.h`, or send `?`.

**ROS (env:microros).**

| Topic | Type | Direction | Contents |
| --- | --- | --- | --- |
| `/canbot/wheel_commands` | `sensor_msgs/JointState` | Pi → ESP32 | `velocity[]` rad/s, matched by joint name; all four wheels required |
| `/canbot/wheel_states` | `sensor_msgs/JointState` | ESP32 → Pi, 50 Hz | `position[]` rad, `velocity[]` rad/s |
| `/canbot/base_faults` | `std_msgs/UInt16` | ESP32 → Pi, 2 Hz | Fault bits below |

Both streams are best effort. These topics are what ros2_control's
`joint_state_topic_hardware_interface/JointStateTopicSystem` speaks. When that's set up in
the next milestone, its `<hardware>` block needs:

```xml
<param name="joint_commands_topic">/canbot/wheel_commands</param>
<param name="joint_states_topic">/canbot/wheel_states</param>
<param name="trigger_joint_command_threshold">-1</param>
```

The last line matters. By default the interface skips publishing whenever the command is
within 1e-5 of the measured state, which would starve this firmware's watchdog. `-1`
makes it publish every cycle.

**Fault bits:** 0–3 stall FL/FR/RL/RR (latched) · 4 E-stop (latched until re-armed) ·
5 watchdog (no recent command; normal when idle) · 6 a control step ran late at least
once · 7 a PWM or encoder failed to initialize.

## Pins

| Wheel | PWM | DIR | Enc A | Enc B |
| --- | --- | --- | --- | --- |
| FL | 4 | 15 | 8 | 9 |
| FR | 5 | 16 | 10 | 11 |
| RL | 6 | 17 | 12 | 13 |
| RR | 7 | 18 | 14 | 21 |

E-stop sense on GPIO 40 and pack voltage on GPIO 1 (ADC1), both disabled in `config.h`
until wired. GPIO 38 is avoided because some DevKitC-1 revisions put the RGB LED on it.
