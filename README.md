# ADS131E08_LIB
An ESP32-optimized library for the TI ADS131E08 (8-Channel, 24-Bit ADC), supporting simultaneous 8-channel sampling and real-time visualization. This library is designed for stable data acquisition using the Arduino framework and FreeRTOS.

## Repository Structure

```text
ADS131E08_LIB/
├── docs/                 # Documentation files
│   └── ads131e08.pdf     # TI ADS131E08 Datasheet
├── src/                  # Library header and implementation
│   ├── ADS131E08.h
│   └── ADS131E08.cpp
├── examples/             # Firmware examples
│   └── ESP32_ADS131E08_stream/
├── extras/               # Desktop support tools
│   └── gui_example/
│       └── requirements.txt
├── .gitattributes        
├── .gitignore            
├── library.json          # PlatformIO Manifest
├── library.properties    # Arduino IDE Manifest
├── LICENSE               # MIT License
└── README.md
```

## Usage Guide

### Installation (PlatformIO)

Add the following to your `platformio.ini` file:
```
lib_deps = https://github.com/nwf-a/ADS131E08_LIB.git
```

### Installation (Arduino IDE)

1. Download this repository as a **ZIP file**. (Click `Code` > `Download ZIP`).
2. Open Arduino IDE and go to Sketch > Include Library > Add .ZIP Library....
3. Select the downloaded ZIP file.
4. Open the example via 
    ```
    File > Examples > ADS131E08_LIB > ESP32_ADS131E08_stream
    ```
## Hardware Connection
The default configuration in basic_read.cpp uses the following pinout:

| ESP32 GPIO | ADS131E08 Pin| Function |
| ----- | --------- | -------- |
| 5V | 5V | Analog Power Supply |
| GND | GND | Common Ground |
| GPIO4 | ~DRDY | Data Ready |
| GPIO5 | ~CS | Chip Select |
| GPIO18 | SCLK | SPI Clock |
| GPIO23 | DIN | SPI Data In |
| GPIO19 | DOUT | SPI Data Out |
| GPIO17 | START | Start Conversion |
| GPIO16 | ~RST | Hardware reset |
| GPIO27 | ~PWRDN | Power Down Control |

## SPI Clock Requirements
The minimum SPI clock frequency is determined by the time available between samples ($t_{DRDY}$):

$$\text{Min SPI Clock} > \frac{\text{Bits per Frame}}{t_{DRDY}}$$

For example, at 8 kSPS:
- $t_{DRDY} = 125\ \mu\text{s}$
- Bits per frame = $216\ \text{bits}$
- $\text{Result} \approx 1.73\ \text{MHz}$ (Recommended $\ge 4\ \text{MHz}$ for stability)

The following table defines the minimum SPI clock frequency required to successfully retrieve all 8-channel data samples before the next conversion cycle completes.

| SPS     | Resolution | Bits / Frame | Min SPI | Recommended SPI |
| --------|------------|--------------|---------|-----------------|
| 1k–4k   | 24-bit     | 216 bits     | 1 MHz   | 2 MHz |
| 8k      | 24-bit     | 216 bits     | 3 MHz   | 4–6 MHz |
| 16k     | 24-bit     | 216 bits     | 6 MHz   | 8–12 MHz |
| 32k     | 16-bit     | 152 bits     | 10 MHz  | 16–20 MHz |
| 64k     | 16-bit     | 152 bits     | 20 MHz  | 24–30 MHz |

## Serial Baud Rate Requirements
To ensure real-time data streaming without buffer overflow, the baud rate must satisfy:

$$\text{Baud Rate} > \text{SPS} \times \text{Packet Size (Bytes)} \times 10 \frac{\text{bits}}{\text{byte}}$$

With the current configuration (**2 kSPS**, **30 Bytes** packet):
$$\text{Min Baud Rate} = 2000 \times 30 \times 10 = 600,000\ \text{bps}$$
*The current baud rate of **921,600** provides a safe overhead.*

## Data Conversion
The ADS131E08 provides data in two's complement format. The resolution (and thus the LSB size) changes depending on the selected Sample Rate (SPS).

1. LSB Size Calculation

$$\text{LSB Size} = \frac{V_{REF} / \text{Gain}}{2^{n-1}}$$

- For 1kSPS to 16kSPS: use $2^{23}$ (24-bit mode, n = 24)
- For 32kSPS and 64kSPS: use $2^{15}$ (16-bit mode, n = 16)

2. Voltage Calculation

$$\text{Voltage (V)} = \text{Raw ADC Value} \times \text{LSB Size}$$

## GUI Monitor Example

### Prerequisites

Before installing the dependencies, ensure your system meets the following requirements:

- Python Version: 3.9+
- pip Version: 22.0 or newer
- Virtual Environment: venv (highly recommended to avoid library conflicts)

### Running the GUI
To ensure a clean installation, it is highly recommended to use a Python Virtual Environment (venv).

1. Navigate to the GUI folder: 
  
   ```
   cd extras/gui_example
   ```

2. Create and Activate Virtual Environment
   
   windows: 
    ```
    python -m venv venv
    venv\Scripts\activate
    ```

    Linux / macOS:
    ```
    python3 -m venv venv
    source venv/bin/activate
    ```

3. Install dependencies: 

    ```
    pip install --upgrade pip
    pip install -r requirements.txt
    ```

4. Run the application: 
   
   windows:
   ```
   python gui_example.py
   ```

   Linux / macOS:
   ```
   python3 gui_example.py
   ```