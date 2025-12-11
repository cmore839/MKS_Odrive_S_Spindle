#include "DRV8301.h"
#include <Arduino.h> 

// Constructor
DRV8301::DRV8301() : _spi_settings(1000000, MSBFIRST, SPI_MODE1) {
    // SPI_MODE1 = (CPOL=0, CPHA=1)
}

// Initialize
void DRV8301::init(SPIClass* spi, int ncs_pin) {
    _spi = spi;
    _ncs_pin = ncs_pin;
}

// Set Verbose Mode
void DRV8301::setVerbose(bool verbose) {
    _verbose = verbose;
}

// ===================================================================
//
//                        LOW-LEVEL R/W
//
// ===================================================================

/**
 * @brief (Private) Writes a value to a specific field within a register.
 * Reads the register, masks the field, writes the new value, and verifies.
 * @return true on success, false on verification failure.
 */
bool DRV8301::write_field(DRV8301_RegName_e reg, uint16_t mask, uint16_t value) {
    uint16_t old_val = read_reg(reg);
    uint16_t new_val = (old_val & ~mask) | (value & mask);

    // Don't write if the value is already correct
    if (old_val == new_val) {
        return true;
    }

    write_reg(reg, new_val);
    uint16_t read_back = read_reg(reg);

    return (read_back == new_val);
}

/**
 * @brief Reads data from a DRV8301 register.
 */
uint16_t DRV8301::read_reg(DRV8301_RegName_e reg) {
    uint16_t controlword = DRV8301_buildCtrlWord(DRV8301_CtrlMode_Read, reg, 0);
    
    _spi->beginTransaction(_spi_settings);
    digitalWrite(_ncs_pin, LOW);
    delayMicroseconds(1);
    _spi->transfer16(controlword);
    delayMicroseconds(1);
    digitalWrite(_ncs_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(_ncs_pin, LOW);
    delayMicroseconds(1);
    uint16_t recbuff = _spi->transfer16(0x0000);
    delayMicroseconds(1);
    digitalWrite(_ncs_pin, HIGH);
    delayMicroseconds(1);
    _spi->endTransaction();

    return (recbuff & DRV8301_DATA_MASK);
}

/**
 * @brief Writes data to a DRV8301 register.
 */
void DRV8301::write_reg(DRV8301_RegName_e reg, uint16_t data) {
    uint16_t controlword = DRV8301_buildCtrlWord(DRV8301_CtrlMode_Write, reg, data);

    _spi->beginTransaction(_spi_settings);
    digitalWrite(_ncs_pin, LOW);
    delayMicroseconds(1);
    _spi->transfer16(controlword);
    delayMicroseconds(1);
    digitalWrite(_ncs_pin, HIGH);
    delayMicroseconds(1);
    _spi->endTransaction();
}

// ===================================================================
//
//                      HIGH-LEVEL HANDLES
//
// ===================================================================

// --- Control Register 1 ---

bool DRV8301::setGateCurrent(DRV8301_PeakCurrent_e current) {
    bool success = write_field(DRV8301_RegName_Control_1, DRV8301_CTRL1_GATE_CURRENT_BITS, current);
    if (_verbose) {
        Serial.print(F("  DRV: Setting Gate Current to: "));
        Serial.print(getGateCurrentString(current));
        Serial.println(success ? F(" ... OK") : F(" ... FAILED!"));
    }
    return success;
}

bool DRV8301::setPWMMode(DRV8301_PwmMode_e mode) {
    bool success = write_field(DRV8301_RegName_Control_1, DRV8301_CTRL1_PWM_MODE_BITS, mode);
    if (_verbose) {
        Serial.print(F("  DRV: Setting PWM Mode to: "));
        Serial.print(getPWMModeString(mode));
        Serial.println(success ? F(" ... OK") : F(" ... FAILED!"));
    }
    return success;
}

bool DRV8301::setOcMode(DRV8301_OcMode_e mode) {
    bool success = write_field(DRV8301_RegName_Control_1, DRV8301_CTRL1_OC_MODE_BITS, mode);
    if (_verbose) {
        Serial.print(F("  DRV: Setting OverCurrent Mode to: "));
        Serial.print(getOcModeString(mode));
        Serial.println(success ? F(" ... OK") : F(" ... FAILED!"));
    }
    return success;
}

bool DRV8301::setOcAdj(DRV8301_VdsLevel_e level) {
    bool success = write_field(DRV8301_RegName_Control_1, DRV8301_CTRL1_OC_ADJ_SET_BITS, level);
    if (_verbose) {
        Serial.print(F("  DRV: Setting OverCurrent Vds to: "));
        Serial.print(getVdsLevelString(level));
        Serial.println(success ? F(" ... OK") : F(" ... FAILED!"));
    }
    return success;
}

void DRV8301::triggerReset() {
    if (_verbose) Serial.print(F("  DRV: Triggering GATE_RESET..."));
    uint16_t val = read_reg(DRV8301_RegName_Control_1);
    write_reg(DRV8301_RegName_Control_1, val | DRV8301_CTRL1_GATE_RESET_BITS);
    // Note: We don't verify this one as the bit clears itself.
    if (_verbose) Serial.println(F(" Done."));
}

// --- Control Register 2 ---

bool DRV8301::setOcTwMode(DRV8301_OcTwMode_e mode) {
    bool success = write_field(DRV8301_RegName_Control_2, DRV8301_CTRL2_OCTW_SET_BITS, mode);
    if (_verbose) {
        Serial.print(F("  DRV: Setting OC/TW Mode to: "));
        Serial.print(getOcTwModeString(mode));
        Serial.println(success ? F(" ... OK") : F(" ... FAILED!"));
    }
    return success;
}

bool DRV8301::setGain(DRV8301_ShuntAmpGain_e gain) {
    bool success = write_field(DRV8301_RegName_Control_2, DRV8301_CTRL2_GAIN_BITS, gain);
    if (_verbose) {
        Serial.print(F("  DRV: Setting Shunt Amp Gain to: "));
        Serial.print(getGainString(gain));
        Serial.println(success ? F(" ... OK") : F(" ... FAILED!"));
    }
    return success;
}

bool DRV8301::setDcCal1(DRV8301_DcCalMode_e mode) {
    bool success = write_field(DRV8301_RegName_Control_2, DRV8301_CTRL2_DC_CAL_1_BITS, mode);
    if (_verbose) {
        Serial.print(F("  DRV: Setting DC Cal 1 to: "));
        Serial.print(getDcCal1String(mode));
        Serial.println(success ? F(" ... OK") : F(" ... FAILED!"));
    }
    return success;
}

bool DRV8301::setDcCal2(DRV8301_DcCalMode_e_ch2 mode) {
    bool success = write_field(DRV8301_RegName_Control_2, DRV8301_CTRL2_DC_CAL_2_BITS, mode);
    if (_verbose) {
        Serial.print(F("  DRV: Setting DC Cal 2 to: "));
        Serial.print(getDcCal2String(mode));
        Serial.println(success ? F(" ... OK") : F(" ... FAILED!"));
    }
    return success;
}

bool DRV8301::setOcToff(DRV8301_OcOffTimeMode_e mode) {
    bool success = write_field(DRV8301_RegName_Control_2, DRV8301_CTRL2_OC_TOFF_BITS, mode);
    if (_verbose) {
        Serial.print(F("  DRV: Setting OC TOFF Mode to: "));
        Serial.print(getOcToffString(mode));
        Serial.println(success ? F(" ... OK") : F(" ... FAILED!"));
    }
    return success;
}


// ===================================================================
//
//                      VERBOSE PRINTERS
//
// ===================================================================

// (This section is unchanged)
void DRV8301::printStatus() {
    uint16_t stat1 = read_reg(DRV8301_RegName_Status_1);
    uint16_t stat2 = read_reg(DRV8301_RegName_Status_2);

    Serial.println(F("--- DRV8301 Status ---"));
    Serial.print(F("  Status 1: 0b")); Serial.println(stat1, BIN);
    
    if (stat1 & DRV8301_STATUS1_FAULT_BITS) {
        Serial.println(F("    >> FAULT DETECTED <<"));
        if (stat1 & DRV8301_STATUS1_FETLC_OC_BITS) Serial.println(F("    - FETLC_OC: FET Low C OverCurrent"));
        if (stat1 & DRV8301_STATUS1_FETHC_OC_BITS) Serial.println(F("    - FETHC_OC: FET High C OverCurrent"));
        if (stat1 & DRV8301_STATUS1_FETLB_OC_BITS) Serial.println(F("    - FETLB_OC: FET Low B OverCurrent"));
        if (stat1 & DRV8301_STATUS1_FETHB_OC_BITS) Serial.println(F("    - FETHB_OC: FET High B OverCurrent"));
        if (stat1 & DRV8301_STATUS1_FETLA_OC_BITS) Serial.println(F("    - FETLA_OC: FET Low A OverCurrent"));
        if (stat1 & DRV8301_STATUS1_FETHA_OC_BITS) Serial.println(F("    - FETHA_OC: FET High A OverCurrent"));
        if (stat1 & DRV8301_STATUS1_OTW_BITS)      Serial.println(F("    - OTW:      Over Temperature Warning"));
        if (stat1 & DRV8301_STATUS1_OTSD_BITS)     Serial.println(F("    - OTSD:     Over Temperature Shutdown"));
        if (stat1 & DRV8301_STATUS1_PVDD_UV_BITS)  Serial.println(F("    - PVDD_UV:  PVDD UnderVoltage"));
        if (stat1 & DRV8301_STATUS1_GVDD_UV_BITS)  Serial.println(F("    - GVDD_UV:  GVDD UnderVoltage"));
    } else {
        Serial.println(F("    - No Faults"));
    }

    Serial.print(F("  Status 2: 0b")); Serial.println(stat2, BIN);
    if (stat2 & DRV8301_STATUS2_GVDD_OV_BITS) {
        Serial.println(F("    - GVDD_OV: GVDD OverVoltage (FAULT)"));
    }
    Serial.print(F("    - DeviceID: 0b")); Serial.println(stat2 & DRV8301_STATUS2_ID_BITS, BIN);
}

void DRV8301::printConfig() {
    uint16_t ctrl1 = read_reg(DRV8301_RegName_Control_1);
    uint16_t ctrl2 = read_reg(DRV8301_RegName_Control_2);

    Serial.println(F("--- DRV8301 Configuration ---"));
    Serial.print(F("  Control 1: 0b")); Serial.println(ctrl1, BIN);
    Serial.print(F("    - Gate Current: ")); Serial.println(getGateCurrentString(ctrl1));
    Serial.print(F("    - PWM Mode:     ")); Serial.println(getPWMModeString(ctrl1));
    Serial.print(F("    - OC Mode:      ")); Serial.println(getOcModeString(ctrl1));
    Serial.print(F("    - OC Vds Level: ")); Serial.println(getVdsLevelString(ctrl1));
    
    Serial.print(F("  Control 2: 0b")); Serial.println(ctrl2, BIN);
    Serial.print(F("    - OC/TW Mode:   ")); Serial.println(getOcTwModeString(ctrl2));
    Serial.print(F("    - Shunt Gain:   ")); Serial.println(getGainString(ctrl2));
    Serial.print(F("    - DC Cal 1:     ")); Serial.println(getDcCal1String(ctrl2));
    Serial.print(F("    - DC Cal 2:     ")); Serial.println(getDcCal2String(ctrl2));
    Serial.print(F("    - OC TOFF Mode: ")); Serial.println(getOcToffString(ctrl2));
}


// ===================================================================
//
//                 STRING CONVERTER HELPERS
//
// ===================================================================

// (This entire section is unchanged)
const char* DRV8301::getGateCurrentString(uint16_t val) {
    switch(val & DRV8301_CTRL1_GATE_CURRENT_BITS) {
        case DRV8301_PeakCurrent_1p70_A: return "1.70A";
        case DRV8301_PeakCurrent_0p70_A: return "0.70A";
        case DRV8301_PeakCurrent_0p25_A: return "0.25A";
        default: return "Unknown";
    }
}
const char* DRV8301::getPWMModeString(uint16_t val) {
    switch(val & DRV8301_CTRL1_PWM_MODE_BITS) {
        case DRV8301_PwmMode_Six_Inputs: return "6-Input";
        case DRV8301_PwmMode_Three_Inputs: return "3-Input";
        default: return "Unknown";
    }
}
const char* DRV8301::getOcModeString(uint16_t val) {
    switch(val & DRV8301_CTRL1_OC_MODE_BITS) {
        case DRV8301_OcMode_CurrentLimit: return "Current Limit";
        case DRV8301_OcMode_LatchShutDown: return "Latch Shutdown";
        case DRV8301_OcMode_ReportOnly: return "Report Only";
        case DRV8301_OcMode_Disabled: return "Disabled";
        default: return "Unknown";
    }
}
const char* DRV8301::getVdsLevelString(uint16_t val) {
    switch(val & DRV8301_CTRL1_OC_ADJ_SET_BITS) {
        case DRV8301_VdsLevel_0p060_V: return "0.060V";
        case DRV8301_VdsLevel_0p138_V: return "0.138V";
        case DRV8301_VdsLevel_0p250_V: return "0.250V";
        case DRV8301_VdsLevel_0p730_V: return "0.730V (Default)";
        case DRV8301_VdsLevel_1p175_V: return "1.175V";
        case DRV8301_VdsLevel_2p400_V: return "2.400V";
        default: return "Other";
    }
}
const char* DRV8301::getOcTwModeString(uint16_t val) {
    switch(val & DRV8301_CTRL2_OCTW_SET_BITS) {
        case DRV8301_OcTwMode_Both: return "OC and TW";
        case DRV8301_OcTwMode_OT_Only: return "OT Only";
        case DRV8301_OcTwMode_OC_Only: return "OC Only";
        default: return "Unknown";
    }
}
const char* DRV8301::getGainString(uint16_t val) {
    switch(val & DRV8301_CTRL2_GAIN_BITS) {
        case DRV8301_ShuntAmpGain_10VpV: return "10 V/V";
        case DRV8301_ShuntAmpGain_20VpV: return "20 V/V";
        case DRV8301_ShuntAmpGain_40VpV: return "40 V/V";
        case DRV8301_ShuntAmpGain_80VpV: return "80 V/V";
        default: return "Unknown";
    }
}
const char* DRV8301::getDcCal1String(uint16_t val) {
    switch(val & DRV8301_CTRL2_DC_CAL_1_BITS) {
        case DRV8301_DcCalMode_Ch1_Load: return "Connected";
        case DRV8301_DcCalMode_Ch1_NoLoad: return "Shorted (Calibrating)";
        default: return "Unknown";
    }
}
const char* DRV8301::getDcCal2String(uint16_t val) {
    switch(val & DRV8301_CTRL2_DC_CAL_2_BITS) {
        case DRV8301_DcCalMode_Ch2_Load: return "Connected";
        case DRV8301_DcCalMode_Ch2_NoLoad: return "Shorted (Calibrating)";
        default: return "Unknown";
    }
}
const char* DRV8301::getOcToffString(uint16_t val) {
    switch(val & DRV8301_CTRL2_OC_TOFF_BITS) {
        case DRV8301_OcOffTimeMode_Normal: return "Normal";
        case DRV8301_OcOffTimeMode_Ctrl: return "CBC Off Time";
        default: return "Unknown";
    }
}