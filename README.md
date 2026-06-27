# STM32 BME280 PID Temperature Controller

A temperature control project based on **STM32**, **BME280**, a custom **PID controller**, an **SSD1306 OLED display**, PWM actuator output, and a **Python GUI** for real-time monitoring and parameter tuning.

The project supports both **cooling** and **heating** control modes. In cooling mode, the PID output can drive a fan. In heating mode, the PID output can drive a heater.

---

## Project Overview

This project reads temperature, humidity, and pressure data from a BME280 sensor over I2C. The measured temperature is used as the process variable of a PID controller. The controller compares the measured temperature with the target setpoint and generates a 0-100% output value.

The PID output can be mapped to:

- Fan PWM duty cycle for cooling
- Heater PWM duty cycle for heating

The STM32 firmware also sends telemetry data over UART. A Python Tkinter GUI receives this data, plots the temperature response in real time, displays environmental values, and allows the user to update PID parameters from the computer.

---

## Main Features

- STM32 HAL-based firmware
- Custom PID controller library
- Custom BME280 driver
- BME280 temperature, humidity, and pressure measurement
- SSD1306 OLED display support
- UART communication with Python GUI
- Real-time temperature and setpoint plotting
- PID output / PWM percentage plotting
- Cooling and heating mode selection
- Runtime PID parameter update from GUI
- Runtime setpoint update from GUI
- START / STOP control over UART

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
- Custom `bme280.h` / `bme280.c`
- SSD1306 OLED library
- `logger.h` / `logger.c`
- `menu.h` / `menu.c`

### Python Side

- Python 3.x
- Tkinter
- PySerial
- Matplotlib

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
|                   |          UART          +----------------+
|                   | <--------------------> | Python GUI     |
+-------------------+                        +----------------+
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
7. Starts PWM output.
8. Starts interrupt-based UART reception.
9. Runs the temperature control loop periodically.

The control loop runs every `CONTROL_PERIOD_MS` milliseconds.

Recommended value:

```c
#define CONTROL_PERIOD_MS 1000U
```

A 1-second control period is usually suitable for temperature control because thermal systems respond slowly compared to electrical signals.

---

## Control Loop Logic

Each control cycle performs the following sequence:

```text
Read BME280 sensor data
Update temperature, humidity, and pressure variables
Convert temperature according to selected control mode
Run PID computation
Map PID output to PWM percentage
Update fan/heater output
Update OLED display
Send telemetry over UART
Print debug log message
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
| `START` | Enable PID control |
| `STOP` | Disable PID control and set output to 0% |

Example commands:

```text
MODE:COOLING
SETPOINT:25.0
KP:8.0
KI:0.05
KD:0.0
START
```

---

## UART Telemetry Format

The STM32 sends telemetry data to the Python GUI in this format:

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

The GUI parses the numeric values and plots them in real time.

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
- START / STOP controls
- Live BME280 values
- Response metrics
- Real-time graph

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

When `START` is clicked, the GUI sends the selected mode, setpoint, and PID gains to STM32 before sending the `START` command.

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

Example Output:

<img width="796" height="692" alt="Ekran görüntüsü 2026-06-27 225051" src="https://github.com/user-attachments/assets/865d0e84-a13b-4a2a-b7f0-d5e36bf8cae6" />

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
   - `bme280.h`
   - `bme280.c`
   - `logger.h`
   - `logger.c`
   - `menu.h`
   - `menu.c`
   - SSD1306 source files
4. Build the project.
5. Flash the firmware to the STM32 board.

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
5. Enter setpoint and PID gains.
6. Click `START`.
7. Observe temperature, setpoint, PID output, and response metrics.

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

These are only starting points. Real values depend on the fan, heater, enclosure, airflow, sensor location, and thermal mass.

---

## Troubleshooting

### BME280 initialization fails

Check:

- I2C wiring
- 3.3V power
- GND connection
- SCL/SDA pin configuration
- BME280 address selection
- Whether SDO is connected to GND or VCC

Try switching:

```c
#define BME280_DEVICE_ADDRESS BME280_I2C_ADDR_GND
```

to:

```c
#define BME280_DEVICE_ADDRESS BME280_I2C_ADDR_VDDIO
```

---

### OLED does not display anything

Check:

- OLED I2C address in the SSD1306 library
- I2C wiring
- Pull-up resistors
- Whether `ssd1306_Init()` is called
- Whether `ssd1306_UpdateScreen()` is called

---

### GUI connects but no graph appears

Check:

- STM32 is sending UART telemetry
- Baudrate is 115200
- Correct COM port is selected
- `START` button was clicked
- Telemetry contains `SetPoint`, `Temperature`, and `PIDOutput`

Expected telemetry example:

```text
SetPoint: 25.00 Temperature: 29.40 PIDOutput: 37.50 Humidity: 45.20 Pressure: 1010.80 Mode: COOLING
```

---

### PID output stays at 0%

Possible causes:

- `STOP` mode is active
- Wrong control mode is selected
- Setpoint is not suitable for the current temperature
- PID gains are too small
- In cooling mode, setpoint may be above current temperature
- In heating mode, setpoint may be below current temperature

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
│   │   │   ├── bme280.h
│   │   │   ├── logger.h
│   │   │   └── menu.h
│   │   └── Src/
│   │       ├── main.c
│   │       ├── pid.c
│   │       ├── bme280.c
│   │       ├── logger.c
│   │       └── menu.c
│   └── Drivers/
│
├── GUI/
│   └── pid_temperature_controller_gui.py
│
├── README.md
└── LICENSE
```

---

## Future Improvements

Possible improvements:

- Add data logging to CSV from the Python GUI
- Add automatic PID tuning support
- Add minimum fan speed configuration
- Add deadband around the setpoint
- Add heating/cooling output indicators on OLED
- Add alarm thresholds for temperature
- Add selectable sample time from GUI
- Add graph export feature
- Add separate fan and heater outputs for dual-mode control

---

## License

This project is licensed under the MIT License. See the [LICENSE](https://github.com/eylloztek/stm32-pid-temperature-controller/blob/master/LICENSE.txt) file for details.
