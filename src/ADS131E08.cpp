/*
 * ADS131E08 Library
 * Copyright (c) 2026 nawaf, AGL (Arok Gandring Lokajaya)
 * Licensed under the MIT License.
 */
#include "ADS131E08.h"

ADS131E08::ADS131E08(int8_t cs, int8_t drdy, int8_t rst, int8_t start, int8_t pwrdn) 
    : _cs(cs), _drdy(drdy), _rst(rst), _start(start), _pwrdn(pwrdn) {}


// Initialize SPI and GPIO pins.
// Powers up the device and waits for the internal oscillator to stabilize.
void ADS131E08::begin(SPIClass &spi, uint32_t frequency) {
    _spi = &spi;
    // ADS131E08 uses SPI Mode 1 (CPOL=0, CPHA=1)
    _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE1);

    pinMode(_cs, OUTPUT);
    pinMode(_drdy, INPUT_PULLUP); // Interrupt pin
    pinMode(_rst, OUTPUT);
    pinMode(_start, OUTPUT);
    pinMode(_pwrdn, OUTPUT);

    digitalWrite(_cs, HIGH);
    digitalWrite(_start, LOW);
    digitalWrite(_pwrdn, HIGH); // Pull high to power up the device
    digitalWrite(_rst, HIGH);

    _dataLen = LEN_19;  // Default for 32kSPS (16-bit)
    _isRDATAC = true;   // Chip defaults to RDATAC on power-up
    
    // The device requires ~2^18 tCLK cycles to power up. 
    // At 2.048MHz, this is roughly 128ms. 155ms provides safety margin.
    delay(155); 
}

// Reset ADS131E08 via ~RST pin
void ADS131E08::reset() {
    digitalWrite(_rst, LOW);
    delayMicroseconds(1); // tRST must be at least 1 tCLK cycle
    digitalWrite(_rst, HIGH);
    
    // Must wait 18 tCLK cycles before communicating after a reset
    delayMicroseconds(11); 

    _isRDATAC = true;
    _dataLen = LEN_19;
}

// Delays for tSDECODE (command decoding time).
// Required between certain SPI bytes or after CS goes low.
void ADS131E08::waitForTdecode() {
    // 4 tCLK = ~2.352us. 
    delayMicroseconds(3); 
}

// Send SPI command to ADS131E08
void ADS131E08::sendCommand(uint8_t cmd) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);
    _spi->transfer(cmd);
    waitForTdecode(); // Ensure command is decoded before raising CS
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
}

// Writes a value to a specific register
// Automatically handles SDATAC because registers cannot be written in RDATAC mode
void ADS131E08::writeRegister(uint8_t address, uint8_t value) {
    disableRDATAC();    // Must be in SDATAC mode to write registers
    
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);

    // Command 1: 010xxxxx (where xxxxx is address)
    _spi->transfer(CMD_WREG | address);
    waitForTdecode();

    // Command 2: Number of registers to write - 1
    _spi->transfer(0x00); 
    waitForTdecode();

    // Send data
    _spi->transfer(value);
    waitForTdecode();

    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
}

// Reads a value from a specific register
uint8_t ADS131E08::readRegister(uint8_t address) {
    disableRDATAC();    // Must be in SDATAC mode to read registers
    uint8_t val = 0;
    
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);

    _spi->transfer(CMD_RREG | address);
    waitForTdecode();

    _spi->transfer(0x00); 
    waitForTdecode();

    val = _spi->transfer(0x00); 
    waitForTdecode();

    digitalWrite(_cs, HIGH);
    _spi->endTransaction();

    return val;
}

bool ADS131E08::isRDATAC() {
    return _isRDATAC;
}

// Enable RDATAC mode for continuous data read
void ADS131E08::enableRDATAC() {
    if (!_isRDATAC) {
        sendCommand(CMD_RDATAC);
        _isRDATAC = true;
    }
}

// Exits Read Data Continuous mode
// Required before sending any other commands
void ADS131E08::disableRDATAC() {
    if (_isRDATAC) {
        sendCommand(CMD_SDATAC);
        _isRDATAC = false;
    }
}

// Start data conversion by setting START pin high
void ADS131E08::start() {
    digitalWrite(_start, HIGH);
}

// Stop data conversion by setting START pin low
void ADS131E08::stop() {
    digitalWrite(_start, LOW);
}

// Perform offset calibration
void ADS131E08::offsetCalibrate() {
    sendCommand(CMD_OFFSETCAL);
    delay(155); // 153ms calibration time
}

// Reads data using the RDATA command
void ADS131E08::readData(uint8_t *buffer) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);
    _spi->transfer(CMD_RDATA);
    waitForTdecode();
    _spi->transferBytes(NULL, buffer, _dataLen); 
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
}

// Reads data when RDATAC is already enabled
void ADS131E08::readDataContinuous(uint8_t *buffer) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_cs, LOW);
    _spi->transferBytes(NULL, buffer, _dataLen); 
    digitalWrite(_cs, HIGH);
    _spi->endTransaction();
}

// get Readback data length
// data rate 64kSPS or 32kSPS = 152 bits = 19 bytes
// all other data rates = 216 bits = 27 bytes
uint8_t ADS131E08::getdataLen(){
    return _dataLen;
}

// Configures global settings (Daisy chain, Clock out, and Sample Rate).
// Note: sample rate changes the bit-depth (16-bit vs 24-bit)
// DAISY_EN = DAISY_DISABLED;
// CLKOUT_EN = CLKOUT_DISABLED;
// DATARATE = DR_32kSPS;
// bit: 1 | DAISY_EN | CLKOUT_EN | 1 | 0 | DR[2:0]
void ADS131E08::setConfig1(DaisyChainSetting daisy, CLKOUTSetting clkout, DataRateSetting dr) {
    uint8_t configValue = 0;
    configValue |= (1 << 7); // Bit 7: Must be 1
    configValue |= (daisy << 6); // DAISY_EN
    configValue |= (clkout << 5); // CLKOUT_EN
    configValue |= (1 << 4); // Bit 4: Must be 1
    configValue |= (0 << 3); // Bit 3: Must be 0
    configValue |= static_cast<uint8_t>(dr); // DR[2:0]

    writeRegister(REG_CONFIG1, configValue);

    // ADS131E08 specific: At 64k and 32k SPS, data is truncated to 16-bit.
    // Length calculation: 24-bit Status + (8 * 16-bit data) = 152 bits (19 bytes)
    // Other rates: 24-bit Status + (8 * 24-bit data) = 216 bits (27 bytes)
    if (dr == DR_64kSPS || dr == DR_32kSPS) {
        _dataLen = LEN_19;
    } 
    else {
        _dataLen = LEN_27;
    }

}

// Set CONFIG2 register
// TEST SIGNAL = TEST_SIGNAL_EXTERNAL;
// TEST_AMP = TEST_AMP_1X;
// TEST_FREQ = TEST_FREQ_221; pulsed at fCLK / 2^21
// bit: 1 | 1 | 1 | INT_TEST | 0 | TEST_AMP | TEST_FREQ[1:0]
void ADS131E08::setConfig2(TestSignalSetting testSignal, TestSignalAmplitude testAmp, TestSignalFrequency testFreq) {
    uint8_t configValue = 0;
    configValue |= (1 << 7); // Bit 7 = 1
    configValue |= (1 << 6); // Bit 6 = 1
    configValue |= (1 << 5); // Bit 5 = 1
    configValue |= (testSignal << 4); // INT_TEST
    configValue |= (0 << 3); // Bit 3 = 0
    configValue |= (testAmp << 2); // TEST_AMP
    configValue |= static_cast<uint8_t>(testFreq); // TEST_FREQ[1:0]

    writeRegister(REG_CONFIG2, configValue);
}

// Set CONFIG3 register
// default : 
// PDB_REFBUF = REF_INTERNAL; 
// VREF = VREF_2V4; 
// OPAMP_REF = OPAMPP_PINS;
// PDB_OPAMP = OPAMP_DISABLED;
// bit: PDB_REFBUF | 1 | VREF_4V | 0 | OPAMP_REF | PDB_OPAMP | 0 | 0
void ADS131E08::setConfig3(PDB_REFBUF pdbRef, VREF vRef, OPAMP_REF opampRef, PDB_OPAMP pdbOpamp) {
    uint8_t configValue = 0;
    configValue |= (pdbRef << 7); // PDB_REFBUF
    configValue |= (1 << 6); // Bit 6 = 1
    configValue |= (vRef << 5); // VREF_4V
    configValue |= (0 << 4); // Bit 4 = 0
    configValue |= (opampRef << 3); // OPAMP_REF
    configValue |= (pdbOpamp << 2); // PDB_OPAMP
    configValue |= (0 << 1); // Bit 1 = 0
    configValue |= (0 << 0); // Bit 0 = 0

    writeRegister(REG_CONFIG3, configValue);

}

// Set Fault Threshold
// bit : COMP_TH[2:0] | 0 | 0 | 0 | 0 | 0
// COMP_TH
// HIGH SIDE - LOW SIDE
// 95% - 5%
// 92.5% - 7.5%
// 90% - 10%
// 87.5% - 12.5%
// 85% - 15%
// 80% - 20%
// 75% - 25%
// 70% - 30%
void ADS131E08::setFaultTh(FaultThreshold fault_th) {
    uint8_t configValue = (static_cast<uint8_t>(fault_th) << 5);

    writeRegister(REG_FAULT, configValue);
}

// Configure individual channel settings
// channel: 1-8
// Bit: [PDn] | [GAINn2:0] | 0 | [Muxn2:0]
// PDn: Power Down bit
// GAINn[2:0]: Gain settings
// GAIN 1, 2, 4, 8, 12
// Muxn2:0: Input multiplexer settings
// Normal input; Shorted to (AVDD + AVSS)/2
// MVDD measurement; Temperature sensor; Test signal
void ADS131E08::setChannelConfig(uint8_t channel, powerDownSetting powerDown, GainSetting gain, InputMuxSetting mux) {
    if (channel < 1 || channel > 8) return; // Invalid channel

    uint8_t configValue = 0;
    configValue |= (powerDown << 7); // PDn
    configValue |= (static_cast<uint8_t>(gain) << 4); // GAINn2:0
    configValue |= (0 << 3); // Bit 3 = 0
    configValue |= static_cast<uint8_t>(mux); // Muxn2:0

    writeRegister(REG_CH1SET + (channel - 1), configValue);

}