# STM32 BME280 PID Temperature Controller

A temperature control project based on **STM32 Nucleo F446RE**, **BME280**, a custom **PID controller**, an **STM32 relay-based PID autotuner**, persistent **PID Flash storage**, an **SSD1306 OLED display**, PWM actuator output, and a **Python GUI** for real-time monitoring, parameter tuning, manual gain presets, automatic PID tuning, and saved PID configuration management.

The project supports both **cooling** and **heating** control modes. In cooling mode, the PID output can drive a fan. In heating mode, the PID output can drive a heater.

---

## Project Overview

This project reads temperature, humidity, and pressure data from a BME280 sensor over I2C. The measured temperature is used as the process variable of a PID controller. The controller compares the measured temperature with the target setpoint and generates a 0-100% output value.

The PID output can be mapped to:

- Fan PWM duty cycle for cooling
- Heater PWM duty cycle for heating

The STM32 firmware also sends telemetry data over UART. A Python Tkinter GUI receives this data, plots the temperature response in real time, displays environmental values, allows the user to update PID parameters from the computer, applies manual PID gain presets, can start an STM32-side relay autotune routine, and can save or load PID settings from either a local GUI configuration file or STM32 Flash.

---

## Main Features

- STM32 HAL-based firmware
- Custom PID controller library
- Custom relay-based PID autotuner module
- Custom BME280 driver
- BME280 temperature, humidity, and pressure measurement
- SSD1306 OLED display support
- UART communication with Python GUI
- Real-time temperature and setpoint plotting
- PID output / PWM percentage plotting
- Cooling and heating mode selection
- Manual PID gain presets in the Python GUI
- STM32-side relay autotune with `Basic PID`, `Less Overshoot`, and `No Overshoot` modes
- Persistent PID configuration storage in STM32 Flash
- Local GUI-side PID configuration storage in `saved_pid_values.json`
- Save/load PID settings from GUI, STM32 Flash, or both
- Runtime PID parameter update from GUI
- Runtime setpoint update from GUI
- START / STOP control over UART
- AUTOTUNE START / STOP control over UART
- Saved PID configuration control over UART

---

## Hardware Requirements

| Component | Purpose |
|---|---|
| STM32 Nucleo-F446RE or compatible STM32 board | Main controller |
| BME280 sensor module | Temperature, humidity, and pressure measurement |
| SSD1306 128x64 OLED display | Local display |
| Fan + MOSFET driver | Cooling actuator |
| Heater + MOSFET/relay driver | Heating actuator |
| USB cable | Programming and UART communication |
| External power supply | Recommended for fan or heater |

> Do not drive a fan or heater directly from an STM32 GPIO pin. Use a suitable driver circuit and make sure all grounds are connected together.

---

## Software Requirements

### STM32 Side

- STM32CubeIDE
- STM32CubeMX configuration generated code
- STM32 HAL drivers
- Custom `pid.h` / `pid.c`
- Custom `pid_autotuner.h` / `pid_autotuner.c`
- Custom `pid_flash_storage.h` / `pid_flash_storage.c`
- Custom `bme280.h` / `bme280.c`
- SSD1306 OLED library
- `logger.h` / `logger.c`
- `menu.h` / `menu.c`
- Updated linker script with reserved PID Flash storage area

### Python Side

- Python 3.x
- Tkinter
- PySerial
- Matplotlib
- Built-in `json`, `pathlib`, and `datetime` modules for local PID configuration storage

Install the required Python packages:

```bash
pip install pyserial matplotlib opentelemetry-api
```

Tkinter is usually included with Python on Windows. On Linux, it may need to be installed separately depending on the distribution.

---

## System Architecture

```text
+-------------------+          I2C           +----------------+
|                   | <--------------------> | BME280 Sensor  |
|                   |                        +----------------+
|                   |
|                   |          I2C           +----------------+
| STM32 MCU         | <--------------------> | SSD1306 OLED   |
|                   |                        +----------------+
|                   |
|                   |          PWM           +----------------+
|                   | ---------------------> | Fan / Heater   |
|                   |                        +----------------+
|                   |
|                   |       Internal Flash   +----------------+
|                   | <--------------------> | Saved PID Data |
|                   |                        +----------------+
|                   |
|                   |          UART          +----------------+
|                   | <--------------------> | Python GUI     |
+-------------------+                        +----------------+
                                                       |
                                                       | Local JSON file
                                                       v
                                              +----------------------+
                                              | saved_pid_values.json |
                                              +----------------------+
```


---

## STM32CubeMX / `.ioc` Configuration

The following configuration is recommended for STM32 Nucleo-F446RE. If a different STM32 board is used, the pin names may need to be adjusted.

---

### 1. I2C1 Configuration

I2C1 is used by both the BME280 sensor and the SSD1306 OLED display.

| Peripheral | Pin | Function |
|---|---|---|
| I2C1_SCL | PB8 | BME280 + SSD1306 SCL |
| I2C1_SDA | PB9 | BME280 + SSD1306 SDA |
| 3V3 | 3.3V | Sensor/OLED power |
| GND | GND | Common ground |

CubeMX settings:

```text
Connectivity > I2C1 > I2C
I2C Speed Mode: Standard Mode
Clock Speed: 100000 Hz
Addressing Mode: 7-bit
```

External pull-up resistors may be required depending on the BME280 and OLED modules. Many breakout boards already include pull-up resistors.

---

### 2. USART2 Configuration

USART2 is used for communication between STM32 and the Python GUI.

| Peripheral | Pin | Function |
|---|---|---|
| USART2_TX | PA2 | STM32 to PC |
| USART2_RX | PA3 | PC to STM32 |

CubeMX settings:

```text
Connectivity > USART2 > Asynchronous
Baudrate: 115200
Word Length: 8 Bits
Parity: None
Stop Bits: 1
Hardware Flow Control: None
```

On Nucleo boards, USART2 is usually connected to the ST-LINK virtual COM port, so no external USB-UART converter is normally required.

---

### 3. PWM Output Configuration

PWM is used to represent the PID output. The PWM output can control a fan or heater driver circuit.

Recommended pin:

| Peripheral | Pin | Function |
|---|---|---|
| TIM3_CH1 | PA6 | PWM output |

CubeMX path:

```text
Timers > TIM3 > PWM Generation CH1
```

#### Fan / Heater PWM Example

For a simple DC fan or heater driver, a 1 kHz PWM signal can be used:

```text
Prescaler: 83
Counter Period: 999
Pulse: 0
PWM Mode: PWM mode 1
Polarity: High
```

This assumes an 84 MHz TIM3 timer clock and generates approximately 1 kHz PWM.

#### 4-Wire PC Fan PWM Example

For a 4-wire PC fan, a higher PWM frequency such as 25 kHz is usually preferred:

```text
Prescaler: 83
Counter Period: 39
Pulse: 0
PWM Mode: PWM mode 1
Polarity: High
```

---

## STM32 Flash PID Storage

The project can store the latest PID configuration directly in STM32 Flash. This allows the board to reuse previously tuned values without running the relay autotune routine every time.

Stored values:

- `Kp`
- `Ki`
- `Kd`
- Temperature setpoint
- Control mode: `COOLING` or `HEATING`
- Magic number
- Storage version
- Checksum

For STM32F446RE, the PID storage module uses the last Flash sector:

| Item | Value |
|---|---|
| Flash sector | Sector 7 |
| Start address | `0x08060000` |
| Size | `128 KB` |
| Storage module | `pid_flash_storage.h` / `pid_flash_storage.c` |

The linker script reserves this area by reducing the application Flash region to 384 KB:

```ld
FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 384K
```

This keeps the application code away from the Flash sector used for saved PID data.

---

## BME280 Wiring

| BME280 Pin | STM32 Connection |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SCL | PB8 / I2C1_SCL |
| SDA | PB9 / I2C1_SDA |
| SDO | GND for address 0x76 or VCC for address 0x77 |

The BME280 driver uses STM32 HAL shifted I2C addresses:

```c
#define BME280_DEVICE_ADDRESS BME280_I2C_ADDR_GND
```

Use `BME280_I2C_ADDR_GND` if SDO is connected to GND. Use `BME280_I2C_ADDR_VDDIO` if SDO is connected to VCC.

---

## SSD1306 OLED Wiring

| OLED Pin | STM32 Connection |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SCL | PB8 / I2C1_SCL |
| SDA | PB9 / I2C1_SDA |

The OLED and BME280 can share the same I2C bus as long as their I2C addresses are different.

---

## Fan / Heater Driver Notes

The STM32 PWM pin should not power the fan or heater directly.

For a DC fan or heater, use a driver stage such as:

- Logic-level N-channel MOSFET
- Gate resistor
- Pull-down resistor on the MOSFET gate
- Flyback diode for inductive loads such as DC motors
- External supply suitable for the load
- Common ground between STM32 and external supply

Basic low-side switching concept:

```text
External VCC ---- Fan/Heater ---- MOSFET Drain
MOSFET Source --- GND
STM32 PWM Pin --- Gate resistor --- MOSFET Gate
STM32 GND ------- External supply GND
```

For heaters, use proper current rating, thermal protection, and electrical isolation where necessary.

---

## Firmware Operation

The STM32 firmware performs the following tasks:

1. Initializes HAL and system clock.
2. Initializes GPIO, USART2, I2C1, and TIM3 PWM.
3. Initializes logger output.
4. Initializes the SSD1306 OLED display.
5. Initializes the BME280 sensor over I2C.
6. Initializes the PID controller.
7. Prepares the relay-based PID autotune state variables.
8. Prepares the pending STM32 Flash storage action state.
9. Starts PWM output.
10. Starts interrupt-based UART reception.
11. Runs normal PID control, relay autotune, stopped mode, or pending saved-PID Flash actions depending on UART commands.

The control loop runs every `CONTROL_PERIOD_MS` milliseconds.

Recommended value:

```c
#define CONTROL_PERIOD_MS 1000U
```

A 1-second control period is usually suitable for temperature control because thermal systems respond slowly compared to electrical signals.

---

## Control Loop Logic

Each control cycle starts by reading the BME280 sensor and updating the measured temperature, humidity, and pressure values. After that, the firmware selects one of the active system modes:

| System mode | Behavior |
|---|---|
| `SYSTEM_MODE_STOPPED` | Keeps PID disabled and sets the PWM output to 0%. |
| `SYSTEM_MODE_PID_CONTROL` | Runs `PID_Compute()` using the selected heating/cooling direction and applies the PID output to PWM. |
| `SYSTEM_MODE_AUTOTUNE` | Runs the relay autotuner instead of the normal PID controller and applies the relay output to PWM. |

Flash save/load operations are requested by UART commands and processed in the main loop. The UART receive interrupt only marks the requested action, while the actual Flash erase/write/read operation is handled outside the interrupt context.

Normal PID control cycle:

```text
Read BME280 sensor data
Update temperature, humidity, and pressure variables
Convert temperature according to selected control mode
Run PID computation
Map PID output to PWM percentage
Update fan/heater output
Update OLED display
Send telemetry over UART
```

Relay autotune cycle:

```text
Read BME280 sensor data
Update temperature, humidity, and pressure variables
Convert temperature according to selected control mode
Run PIDAutotuner_Run()
Apply relay output to fan/heater PWM
Send autotune status over UART
Apply calculated Kp, Ki, and Kd when autotune finishes
```

The PID output is limited between 0% and 100%:

```c
#define PID_MIN_OUTPUT 0.0f
#define PID_MAX_OUTPUT 100.0f
```

---

## Heating and Cooling Modes

The GUI and STM32 firmware support two control modes:

- `COOLING`
- `HEATING`

The selected mode determines how the PID error is interpreted.

---

### Heating Mode

Heating mode is used when the output drives a heater.

Expected behavior:

```text
Temperature < Setpoint  -> PID output increases
Temperature ~= Setpoint -> PID output stabilizes
Temperature > Setpoint  -> PID output decreases
```

In heating mode, the PID controller can use the temperature directly:

```text
error = setpoint - temperature
```

Example:

```text
Setpoint = 30 C
Temperature = 25 C
error = 30 - 25 = +5
```

The positive error increases the PID output, so the heater power increases.

---

### Cooling Mode

Cooling mode is used when the output drives a fan.

Expected behavior:

```text
Temperature > Setpoint  -> PID output increases
Temperature ~= Setpoint -> PID output stabilizes
Temperature < Setpoint  -> PID output decreases
```

With the default PID equation, cooling would have the wrong direction because:

```text
error = setpoint - temperature
```

Example:

```text
Setpoint = 25 C
Temperature = 30 C
error = 25 - 30 = -5
```

A negative error would reduce the output, but a fan should increase its output when the temperature is above the setpoint.

To solve this without changing the PID library, the firmware internally negates both the setpoint and the measured temperature:

```c
PID_SetPoint(&pid, -temperatureSetPoint);
pidInput = -temperature;
```

Then the internal error becomes:

```text
internal setpoint = -25
internal input    = -30
error = -25 - (-30) = +5
```

The positive error increases the PID output, so the fan PWM increases.

---

## UART Commands

The Python GUI sends text-based commands to STM32 over UART.

| Command | Description |
|---|---|
| `MODE:COOLING` | Select cooling mode |
| `MODE:HEATING` | Select heating mode |
| `SETPOINT:<value>` | Set target temperature in degrees Celsius |
| `SET_SP:<value>` | Alternative setpoint command |
| `KP:<value>` | Set proportional gain |
| `SET_KP:<value>` | Alternative Kp command |
| `KI:<value>` | Set integral gain |
| `SET_KI:<value>` | Alternative Ki command |
| `KD:<value>` | Set derivative gain |
| `SET_KD:<value>` | Alternative Kd command |
| `START` | Enable normal PID control |
| `STOP` | Disable PID control and set output to 0% |
| `AUTOTUNE_MODE:BASIC` | Select the aggressive Ziegler-Nichols Basic PID autotune mode |
| `AUTOTUNE_MODE:LESS_OVERSHOOT` | Select the less-overshoot autotune mode |
| `AUTOTUNE_MODE:NO_OVERSHOOT` | Select the no-overshoot autotune mode |
| `AUTOTUNE_CYCLES:<value>` | Set the number of relay autotune cycles |
| `AUTOTUNE_MIN_OUTPUT:<value>` | Set relay minimum output percentage |
| `AUTOTUNE_MAX_OUTPUT:<value>` | Set relay maximum output percentage |
| `AUTOTUNE_START` | Start STM32-side relay autotune |
| `AUTOTUNE_STOP` | Stop STM32-side relay autotune |
| `SAVE_PID_FLASH` | Save the current `Kp`, `Ki`, `Kd`, setpoint, and mode to STM32 Flash |
| `LOAD_PID_FLASH` | Load the saved PID configuration from STM32 Flash and apply it to the active PID controller |
| `GET_PID_FLASH` | Read the saved PID configuration from STM32 Flash without starting PID control |
| `CLEAR_PID_FLASH` | Clear the STM32 Flash PID configuration sector |

Normal PID example:

```text
MODE:COOLING
SETPOINT:25.0
KP:8.0
KI:0.05
KD:0.0
START
```

STM32 relay autotune example:

```text
MODE:COOLING
SETPOINT:25.0
AUTOTUNE_MODE:NO_OVERSHOOT
AUTOTUNE_CYCLES:6
AUTOTUNE_MIN_OUTPUT:0
AUTOTUNE_MAX_OUTPUT:100
AUTOTUNE_START
```

STM32 Flash save example:

```text
MODE:COOLING
SETPOINT:25.0
KP:8.0
KI:0.05
KD:0.0
SAVE_PID_FLASH
```

STM32 Flash load example:

```text
LOAD_PID_FLASH
```

---

## UART Telemetry Format

The STM32 sends normal telemetry data to the Python GUI in this format:

```text
SetPoint: 25.00 Temperature: 29.40 PIDOutput: 37.50 Humidity: 45.20 Pressure: 1010.80 Mode: COOLING
```

Fields:

| Field | Meaning |
|---|---|
| `SetPoint` | Target temperature in degrees Celsius |
| `Temperature` | Measured BME280 temperature in degrees Celsius |
| `PIDOutput` | PID output mapped to PWM percentage |
| `Humidity` | Relative humidity in percent RH |
| `Pressure` | Pressure in hPa |
| `Mode` | Current control mode, `COOLING` or `HEATING` |

During STM32 relay autotune, the firmware sends running status messages:

```text
Autotune: RUNNING Cycle: 3 TargetCycles: 6 Temperature: 28.45 Output: 100.00 Ku: 12.700000 Tu: 8.400 Mode: COOLING Rule: NO_OVERSHOOT
```

When autotune finishes, the firmware sends the calculated values:

```text
Autotune: DONE Kp: 2.604880 Ki: 0.138927 Kd: 32.560998 Ku: 13.024399 Tu: 37.500 Rule: NO_OVERSHOOT
```

The GUI parses these messages and updates the autotune status, cycle count, relay output, `Ku`, `Tu`, and calculated `Kp`, `Ki`, and `Kd` fields.

STM32 Flash storage responses use the `SavedPID` prefix:

```text
SavedPID: SAVE_OK Kp: 8.000000 Ki: 0.050000 Kd: 0.000000 SetPoint: 25.00 Mode: COOLING
SavedPID: LOAD_OK Kp: 8.000000 Ki: 0.050000 Kd: 0.000000 SetPoint: 25.00 Mode: COOLING
SavedPID: VALID Kp: 8.000000 Ki: 0.050000 Kd: 0.000000 SetPoint: 25.00 Mode: COOLING
SavedPID: EMPTY Status: EMPTY
SavedPID: CLEAR_OK Status: OK
SavedPID: BUSY Reason: AUTOTUNE_RUNNING
```

The GUI parses `SAVE_OK`, `LOAD_OK`, and `VALID` responses and copies the received values into the normal PID parameter fields.

---

## OLED Display

The OLED shows the most important local values:

```text
Set Point:25.00
Temp:29.40 C
PWM:38 %
```

If the OLED font supports the degree symbol, the temperature can be shown as `C` with a degree symbol. If the character appears incorrectly, use plain `C` instead.

Recommended safe format:

```c
sprintf(temperatureBuffer, "Temp:%.2f C", temperature);
```

---

## Python GUI Overview

The Python GUI provides a desktop interface for configuring and monitoring the controller.

Main GUI sections:

- Serial connection settings
- PID parameter inputs
- Heating/Cooling mode selection
- Manual PID gain presets
- STM32 relay PID autotune controls
- Saved PID Values controls for GUI and STM32 Flash storage
- START / STOP controls
- Live BME280 values
- Autotune status and result values
- Saved PID status messages
- Response metrics
- Real-time graph

---

<img width="1279" height="763" alt="image" src="https://github.com/user-attachments/assets/46189818-ec49-482e-8142-02f50bf6ab45" />

---

## GUI Serial Settings

The GUI should match the STM32 USART2 configuration:

| Setting | Value |
|---|---|
| COM Port | ST-LINK virtual COM port |
| Baudrate | 115200 |
| Stop Bits | One |
| Parity | None |

After selecting the correct COM port, click `Connect`.

---

## GUI Parameters

| GUI Field | Meaning |
|---|---|
| Set Point | Target temperature in degrees Celsius |
| Kp | Proportional gain |
| Ki | Integral gain |
| Kd | Derivative gain |
| Mode | Cooling or Heating control direction |
| Manual PID Gain Preset | Predefined gain set that fills the `Kp`, `Ki`, and `Kd` fields |
| ZN Mode | STM32 relay autotune mode: `Basic PID`, `Less Overshoot`, or `No Overshoot` |
| Cycles | Number of relay cycles used by the STM32 autotuner |
| Min Output | Minimum relay output percentage during autotune |
| Max Output | Maximum relay output percentage during autotune |
| Save to | Save destination: `GUI`, `STM32 Flash`, or `GUI + STM32 Flash` |
| Load from | Load source: `GUI` or `STM32 Flash` |

When `START` is clicked, the GUI sends the selected mode, setpoint, and PID gains to STM32 before sending the `START` command. When `START AUTOTUNE` is clicked, the GUI sends the selected mode, setpoint, autotune mode, cycle count, output range, and then the `AUTOTUNE_START` command. The Saved PID Values controls can save the current GUI PID fields locally, request a save to STM32 Flash, or load values back into the GUI fields.

---

## Manual PID Gain Presets

The Python GUI includes a **Manual PID Gain Presets** section. This feature is separate from STM32 relay autotune. It does not measure `Ku` or `Tu`; it only fills the `Kp`, `Ki`, and `Kd` input fields with predefined values.

Current manual preset values:

| Preset | Kp | Ki | Kd |
|---|---:|---:|---:|
| Ziegler-Nichols | 0.60 | 0.50 | 0.125 |
| Tyreus-Luyben | 0.4545 | 2.20 | 0.1587 |
| Ciancone-Marlin | 0.303 | 0.227 | 0.1235 |
| Pessen Integral | 0.70 | 0.40 | 0.15 |
| Some Overshoot | 0.33 | 0.50 | 0.33 |
| No Overshoot | 0.20 | 0.50 | 0.33 |
| Brewing | 8.1507 | 0.1482 | 7.0783 |

When a preset is selected, the corresponding values are written into the PID parameter input fields. If **SEND PRESET** is clicked, the GUI sends:

```text
KP:<value>
KI:<value>
KD:<value>
```

If **START** is clicked after selecting a preset, the GUI sends the selected mode, setpoint, current PID gain values, and then the `START` command.

---

## STM32 Relay PID Autotune

The project also includes a true STM32-side relay autotune routine. Unlike manual presets, this routine actively drives the output between minimum and maximum output levels, observes the temperature response around the setpoint, estimates `Ku` and `Tu`, and calculates new `Kp`, `Ki`, and `Kd` values.

Supported autotune modes:

| GUI mode | UART command | STM32 mode |
|---|---|---|
| Basic PID | `AUTOTUNE_MODE:BASIC` | `PID_AUTOTUNER_ZN_BASIC_PID` |
| Less Overshoot | `AUTOTUNE_MODE:LESS_OVERSHOOT` | `PID_AUTOTUNER_ZN_LESS_OVERSHOOT` |
| No Overshoot | `AUTOTUNE_MODE:NO_OVERSHOOT` | `PID_AUTOTUNER_ZN_NO_OVERSHOOT` |

The STM32 autotuner uses these Ziegler-Nichols constants:

| Mode | Kp constant | Ti constant | Td constant |
|---|---:|---:|---:|
| Basic PID | 0.60 | 0.50 | 0.125 |
| Less Overshoot | 0.33 | 0.50 | 0.33 |
| No Overshoot | 0.20 | 0.50 | 0.33 |

The relay autotune output alternates between `AUTOTUNE_MIN_OUTPUT` and `AUTOTUNE_MAX_OUTPUT`. From the resulting oscillation, the firmware calculates:

```text
Ku = 4d / (pi * a)
Tu = tHigh + tLow
```

where `d` is the relay output amplitude and `a` is the measured input amplitude.

The PID gains are then calculated in a form compatible with the project's PID implementation:

```text
Kp = KpConstant * Ku
Ki = Kp / (TiConstant * Tu)
Kd = TdConstant * Kp * Tu
```

The project's PID controller already applies `sampleTime` inside the integral and derivative terms, so the autotuner does not multiply `Ki` or divide `Kd` by the loop interval.

When autotune finishes, STM32 applies the calculated gains to the active PID controller and returns to normal PID control mode.

---

## GUI Autotune Display

During autotune, the Python GUI displays:

- Autotune status
- Current cycle and target cycle count
- Relay output percentage
- Calculated `Ku`
- Calculated `Tu`
- Result `Kp`
- Result `Ki`
- Result `Kd`
- Active autotune rule

When the GUI receives an `Autotune: DONE` message, it updates the result fields and writes the calculated `Kp`, `Ki`, and `Kd` values into the normal PID parameter input fields.

---

## Saved PID Values

The GUI includes a **Saved PID Values** section for reusing previously tuned PID settings. This is useful after manual tuning or after a successful STM32 relay autotune run.

The save destination can be selected from:

| Save destination | Behavior |
|---|---|
| `GUI` | Saves the current GUI `Kp`, `Ki`, `Kd`, setpoint, and mode values to a local JSON file. |
| `STM32 Flash` | Sends the current GUI values to STM32 and then sends `SAVE_PID_FLASH`. |
| `GUI + STM32 Flash` | Saves the same configuration both locally and to STM32 Flash. |

The load source can be selected from:

| Load source | Behavior |
|---|---|
| `GUI` | Loads values from the local `saved_pid_values.json` file into the GUI fields. |
| `STM32 Flash` | Sends `LOAD_PID_FLASH` to STM32 and updates the GUI fields from the returned `SavedPID: LOAD_OK` response. |

The local GUI file is created next to the Python GUI script:

```text
saved_pid_values.json
```

Example local GUI configuration format:

```json
{
    "kp": 8.0,
    "ki": 0.05,
    "kd": 0.0,
    "setpoint": 25.0,
    "mode": "COOLING",
    "saved_at": "2026-06-30T21:05:00"
}
```

When saving to STM32 Flash, the GUI first sends the current mode, setpoint, and gain values, then sends the Flash save command:

```text
MODE:<COOLING or HEATING>
SETPOINT:<value>
KP:<value>
KI:<value>
KD:<value>
SAVE_PID_FLASH
```

When loading from STM32 Flash, the GUI sends:

```text
LOAD_PID_FLASH
```

If STM32 returns a valid saved configuration, the GUI updates the `Set Point`, `Kp`, `Ki`, `Kd`, and `Mode` fields.

---

## GUI Graph

The graph displays:

- Temperature
- Setpoint
- PID output / PWM percentage

The left y-axis is used for temperature and setpoint:

```text
Temperature / Set Point (C)
```

The right y-axis is used for PID output:

```text
PID Output / PWM (%)
```

Expected graph behavior in cooling mode:

```text
Temperature above setpoint -> PID output increases
Temperature approaches setpoint -> PID output decreases or stabilizes
Temperature below setpoint -> PID output approaches 0%
```

Expected graph behavior in heating mode:

```text
Temperature below setpoint -> PID output increases
Temperature approaches setpoint -> PID output decreases or stabilizes
Temperature above setpoint -> PID output approaches 0%
```

---

## GUI Live Data

The GUI can display the latest values received from STM32:

- Temperature
- Humidity
- Pressure
- PID output
- Control mode

These values are updated whenever a valid telemetry line is received from STM32.

---

## Response Metrics

The GUI calculates several response metrics from the received temperature data.

| Metric | Description |
|---|---|
| Rise Time | Time required to move from 10% to 90% of the response range |
| Settling Time | Time required for the temperature to remain within the settling band |
| Overshoot | Amount by which the response passes the setpoint |
| Steady-State Error | Difference between setpoint and recent average temperature |

For cooling mode, the temperature response may move downward instead of upward. The metric calculation should support both increasing and decreasing responses.

Thermal systems are slow, so these values may take time to become meaningful.

---

## Running the Project

### 1. Prepare STM32 Project

1. Open the project in STM32CubeIDE.
2. Configure I2C1, USART2, and TIM3 PWM in the `.ioc` file.
3. Add the following source files to the project:
   - `pid.h`
   - `pid.c`
   - `pid_autotuner.h`
   - `pid_autotuner.c`
   - `pid_flash_storage.h`
   - `pid_flash_storage.c`
   - `bme280.h`
   - `bme280.c`
   - `logger.h`
   - `logger.c`
   - `menu.h`
   - `menu.c`
   - SSD1306 source files
4. Use the linker script version that reserves the last Flash sector for PID storage.
5. Build the project.
6. Flash the firmware to the STM32 board.

---

### 2. Connect Hardware

1. Connect BME280 to I2C1.
2. Connect SSD1306 OLED to the same I2C1 bus.
3. Connect the fan or heater driver input to TIM3_CH1.
4. Connect the STM32 board to the PC over USB.

---

### 3. Run Python GUI

Run the GUI script:

```bash
python pid_temperature_controller_gui.py
```

Then:

1. Select the correct COM port.
2. Select `115200` baudrate.
3. Click `Connect`.
4. Select `COOLING` or `HEATING` mode.
5. Enter setpoint and PID gains manually, or select a manual PID gain preset.
6. Click `START` to run normal PID control.
7. To run STM32 relay autotune, select the ZN mode, cycle count, min output, and max output, then click `START AUTOTUNE`.
8. Observe temperature, setpoint, PID output, relay autotune status, calculated gains, and response metrics.
9. Use **SAVE PID VALUES** to store the current PID settings in the GUI file, STM32 Flash, or both.
10. Use **LOAD PID VALUES** to reuse previously saved PID settings.

---

## PID Tuning Notes

Temperature systems usually respond slowly. Start with simple values and tune gradually.

Recommended initial approach:

1. Set `Ki = 0` and `Kd = 0`.
2. Increase `Kp` until the output reacts clearly.
3. Add a small `Ki` if steady-state error remains.
4. Add `Kd` only if the response is too aggressive or oscillatory.

Example starting values:

```c
#define DEFAULT_KP 8.0f
#define DEFAULT_KI 0.05f
#define DEFAULT_KD 0.0f
```

These are only starting points. Real values depend on the fan, heater, enclosure, airflow, sensor location, and thermal mass. The GUI can also fill the PID fields from manual gain presets or ask the STM32 firmware to calculate gains through relay autotune. After a useful set of gains is found, the same values can be saved locally or in STM32 Flash and reused later.

---

## Safety Notes

- Do not power motors or heaters directly from STM32 GPIO pins.
- Use a proper MOSFET, relay, or driver circuit.
- Use external power for high-current loads.
- Always connect external supply ground and STM32 ground together.
- Use flyback protection for motors and other inductive loads.
- Use thermal protection when working with heaters.
- Test with low power before connecting a real load.

---

## Suggested Repository Structure

```text
stm32-bme280-pid-temperature-controller/
│
├── STM32/
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── main.h
│   │   │   ├── pid.h
│   │   │   ├── pid_autotuner.h
│   │   │   ├── pid_flash_storage.h
│   │   │   ├── bme280.h
│   │   │   ├── logger.h
│   │   │   └── menu.h
│   │   └── Src/
│   │       ├── main.c
│   │       ├── pid.c
│   │       ├── pid_autotuner.c
│   │       ├── pid_flash_storage.c
│   │       ├── bme280.c
│   │       ├── logger.c
│   │       └── menu.c
│   ├── Drivers/
│   └── STM32F446RETX_FLASH_pid_storage.ld
│
├── GUI/
│   ├── pid_temperature_controller_gui.py
│   └── saved_pid_values.json     # generated by GUI when PID values are saved locally
│
├── README.md
└── LICENSE
```

---

## Future Improvements

Possible improvements:

- Add data logging to CSV from the Python GUI
- Add minimum fan speed configuration
- Add deadband around the setpoint
- Add heating/cooling output indicators on OLED
- Add alarm thresholds for temperature
- Add selectable sample time from GUI
- Add graph export feature
- Add multiple named PID profiles in the GUI
- Add separate fan and heater outputs for dual-mode control

---

## License

This project is licensed under the MIT License. See the [LICENSE](https://github.com/eylloztek/stm32-pid-temperature-controller/blob/master/LICENSE.txt) file for details.
