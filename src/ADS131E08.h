/*
 * ADS131E08 Library
 *
 * Copyright (c) 2026 nawaf, AGL (Arok Gandring Lokajaya)
 * Licensed under the MIT License.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * --------------------------------------------------------------------------
 * Hardware Specifications (ADS131E08):
 * --------------------------------------------------------------------------
 * - Internal Oscillator Frequency (fCLK): 2.048 MHz
 * - Master Clock Period (tCLK):
 *     • Min: 444 ns
 *     • Max: 588 ns
 *
 * --------------------------------------------------------------------------
 * SPI Clock Reference (8 Channels Enabled, Status Word Included):
 * --------------------------------------------------------------------------
 * SPS     | Resolution | Bits / Frame | Min SPI | Recommended SPI
 * --------|------------|--------------|---------|-----------------
 * 1k–4k   | 24-bit     | 216 bits     | 1 MHz   | 2 MHz
 * 8k      | 24-bit     | 216 bits     | 3 MHz   | 4–6 MHz
 * 16k     | 24-bit     | 216 bits     | 6 MHz   | 8–12 MHz
 * 32k     | 16-bit     | 152 bits     | 10 MHz  | 16–20 MHz
 * 64k     | 16-bit     | 152 bits     | 20 MHz  | 24–30 MHz
 *
 * Notes:
 * - At 32kSPS and 64kSPS, ADS131E08 operates in High-Speed (16-bit) mode.
 * - SPI bandwidth calculations assume:
 *     • 8 active channels
 *     • Status word 
 *     • Continuous conversion mode
 */

#ifndef ADS131E08_H
#define ADS131E08_H

#include <Arduino.h>
#include <SPI.h>

// --- Commands ---
#define CMD_WAKEUP    0x02  // Wake up from standby mode
#define CMD_STANDBY   0x04  // Enter low-power standby mode
#define CMD_RESET     0x06  // Reset the device (software reset)
#define CMD_START     0x08  // Start conversions
#define CMD_STOP      0x0A  // Stop conversions
#define CMD_OFFSETCAL 0x1A  // Channel offset calibration
#define CMD_RDATAC    0x10  // Read Data Continuous mode (Default)
#define CMD_SDATAC    0x11  // Stop Read Data Continuous mode (Required for register access)
#define CMD_RDATA     0x12  // Read data by command (single shot)
#define CMD_RREG      0x20  // Read Register (ORed with starting address)
#define CMD_WREG      0x40  // Write Register (ORed with starting address)

// --- Register Map Address ---
#define REG_ID        0x00  // ID Control Register (Read Only)
#define REG_CONFIG1   0x01  // Configuration Register 1 (Data Rate, Daisy Chain, Clock Output)
#define REG_CONFIG2   0x02  // Configuration Register 2 (Test Signal generation)
#define REG_CONFIG3   0x03  // Configuration Register 3 (Reference and internal amplifier operation)
#define REG_FAULT     0x04  // Configures the fault detection operation
#define REG_CH1SET    0x05
#define REG_CH2SET    0x06
#define REG_CH3SET    0x07
#define REG_CH4SET    0x08
#define REG_CH5SET    0x09
#define REG_CH6SET    0x0A
#define REG_CH7SET    0x0B
#define REG_CH8SET    0x0C
#define REG_FAULT_STATP 0x12
#define REG_FAULT_STATN 0x13
#define REG_GPIO      0x14

enum DaisyChainSetting {
    DAISY_DISABLED = 0x00,
    DAISY_ENABLED = 0x01
};

enum CLKOUTSetting {
    CLKOUT_DISABLED = 0x00,
    CLKOUT_ENABLED = 0x01
};

enum DataRateSetting {
    DR_64kSPS = 0x00,   // 16-bit resolution
    DR_32kSPS = 0x01,   // 16-bit resolution
    DR_16kSPS = 0x02,   // 24-bit resolution
    DR_8kSPS  = 0x03,   // 24-bit resolution
    DR_4kSPS  = 0x04,   // 24-bit resolution
    DR_2kSPS  = 0x05,   // 24-bit resolution
    DR_1KSPS  = 0x06    // 24-bit resolution
};

enum TestSignalSetting {
    TEST_SIGNAL_EXTERNAL = 0x00,
    TEST_SIGNAL_INTERNAL = 0x01
};

enum TestSignalAmplitude {
    TEST_AMP_1X = 0x00, // 1 × –(VVREFP – VVREFN) / 2400
    TEST_AMP_2X = 0x01 // 2 × –(VVREFP – VVREFN) / 2400
};

enum TestSignalFrequency {
    TEST_FREQ_221 = 0x00, // Pulsed at fCLK / 2^21
    TEST_FREQ_220 = 0x01, // Pulsed at fCLK / 2^20
    TEST_FREQ_DC  = 0x03  // DC Signal
};

enum PDB_REFBUF {
    REF_EXTERNAL = 0x00,
    REF_INTERNAL = 0x01
};

enum VREF {
    VREF_2V4 = 0x00,
    VREF_4V = 0x01
};

enum OPAMP_REF {
    OPAMPP_PINS = 0x00,
    HALF_AVDD_AVSS = 0x01
};

enum PDB_OPAMP {
    OPAMP_DISABLED = 0x00,
    OPAMP_ENABLED = 0x01
};

enum GainSetting {
    GAIN_1 = 0x01,
    GAIN_2 = 0x02,
    GAIN_4 = 0x04,
    GAIN_8 = 0x05,
    GAIN_12 = 0x06
};

enum InputMuxSetting {
    MUX_NORMAL = 0X00,
    MUX_SHORTED = 0x01,
    MUX_MVDD = 0x03,
    MUX_TEMP_SENSOR = 0x04,
    MUX_TEST_SIGNAL = 0x05
};

enum powerDownSetting {
    POWERED_ACTIVE = 0x00,
    POWERED_DOWN = 0x01
};

enum FaultThreshold {
    FAULT_THRESHOLD_HIGH_SIDE_95 = 0x00,
    FAULT_THRESHOLD_HIGH_SIDE_92_5 = 0x01,
    FAULT_THRESHOLD_HIGH_SIDE_90 = 0x02,
    FAULT_THRESHOLD_HIGH_SIDE_87_5 = 0x03,
    FAULT_THRESHOLD_HIGH_SIDE_85 = 0x04,
    FAULT_THRESHOLD_HIGH_SIDE_80 = 0x05,
    FAULT_THRESHOLD_HIGH_SIDE_75 = 0x06,
    FAULT_THRESHOLD_HIGH_SIDE_70 = 0x07
};

enum ReadBackLength {
    LEN_19 = 0X13,  // 24 bits (Status) + 8*16 bits (Channels) / 8 = 19 Bytes
    LEN_27 = 0X1B   // 24 bits (Status) + 8*24 bits (Channels) / 8 = 27 Bytes
};

class ADS131E08 {
public:
    ADS131E08(int8_t cs, int8_t drdy, int8_t rst, int8_t start, int8_t pwrdn);

    // Initialization and Hardware Control
    void begin(SPIClass &spi, uint32_t frequency = 4000000);
    void reset();
    void start();
    void stop();
    
    // Communication Methods
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);
    void sendCommand(uint8_t cmd);

    // Data Retrieval
    void readData(uint8_t *buffer);             // RDATA command
    void readDataContinuous(uint8_t *buffer);   // Direct fetch when RDATAC is active
    uint8_t getdataLen();                       // Returns current expected buffer size
    
    // RDATAC Management (ADS131 boots in RDATAC mode)
    void enableRDATAC();
    void disableRDATAC();
    bool isRDATAC();

    // Offset Calibration
    void offsetCalibrate();

    // Register Configuration
    void setConfig1(DaisyChainSetting daisy=DAISY_DISABLED, CLKOUTSetting clkout=CLKOUT_DISABLED, DataRateSetting dr=DR_32kSPS);
    void setConfig2(TestSignalSetting testSignal=TEST_SIGNAL_EXTERNAL, TestSignalAmplitude testAmp=TEST_AMP_1X, TestSignalFrequency testFreq=TEST_FREQ_221);
    void setConfig3(PDB_REFBUF pdbRef=REF_INTERNAL, VREF vRef=VREF_2V4, OPAMP_REF opampRef=OPAMPP_PINS, PDB_OPAMP pdbOpamp=OPAMP_DISABLED);
    void setFaultTh(FaultThreshold fault_th);
    void setChannelConfig(uint8_t channel, powerDownSetting powerDown=POWERED_ACTIVE, GainSetting gain=GAIN_1, InputMuxSetting mux=MUX_NORMAL);

private:
    int8_t _cs, _drdy, _rst, _start, _pwrdn;
    SPIClass *_spi;
    SPISettings _spiSettings;
    
    uint8_t _dataLen = LEN_19;  // Current data length (19 or 27 bytes)
    bool _isRDATAC = true;
    
    void waitForTdecode();      // Hardware-specific delay helper
};

#endif