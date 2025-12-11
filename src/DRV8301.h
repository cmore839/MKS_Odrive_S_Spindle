#ifndef DRV8301_H
#define DRV8301_H

#include <SPI.h>
#include "pins.h"

// --- DRV8301 Definitions (from original ODrive firmware) ---
// ... (All definitions remain identical to your file) ...
// Bitmasks
#define DRV8301_ADDR_MASK               (0x7800)
#define DRV8301_DATA_MASK               (0x07FF)
#define DRV8301_RW_MASK                 (0x8000)
// R/W modes
typedef enum {
  DRV8301_CtrlMode_Read = 1 << 15,   // Read Mode
  DRV8301_CtrlMode_Write = 0 << 15   // Write Mode
} DRV8301_CtrlMode_e;
// Register names
typedef enum {
  DRV8301_RegName_Status_1  = 0 << 11,   // Status Register 1
  DRV8301_RegName_Status_2  = 1 << 11,   // Status Register 2
  DRV8301_RegName_Control_1 = 2 << 11,   // Control Register 1
  DRV8301_RegName_Control_2 = 3 << 11    // Control Register 2
} DRV8301_RegName_e;
// Status Register 1 Bits
#define DRV8301_STATUS1_FETLC_OC_BITS   (1 << 0)
#define DRV8301_STATUS1_FETHC_OC_BITS   (1 << 1)
#define DRV8301_STATUS1_FETLB_OC_BITS   (1 << 2)
#define DRV8301_STATUS1_FETHB_OC_BITS   (1 << 3)
#define DRV8301_STATUS1_FETLA_OC_BITS   (1 << 4)
#define DRV8301_STATUS1_FETHA_OC_BITS   (1 << 5)
#define DRV8301_STATUS1_OTW_BITS        (1 << 6)
#define DRV8301_STATUS1_OTSD_BITS       (1 << 7)
#define DRV8301_STATUS1_PVDD_UV_BITS    (1 << 8)
#define DRV8301_STATUS1_GVDD_UV_BITS    (1 << 9)
#define DRV8301_STATUS1_FAULT_BITS      (1 << 10)
// Status Register 2 Bits
#define DRV8301_STATUS2_ID_BITS        (15 << 0)
#define DRV8301_STATUS2_GVDD_OV_BITS   (1 << 7)
// Control Register 1 Bits
#define DRV8301_CTRL1_GATE_CURRENT_BITS  (3 << 0)
#define DRV8301_CTRL1_GATE_RESET_BITS    (1 << 2)
#define DRV8301_CTRL1_PWM_MODE_BITS      (1 << 3)
#define DRV8301_CTRL1_OC_MODE_BITS       (3 << 4)
#define DRV8301_CTRL1_OC_ADJ_SET_BITS    (31 << 6)
// Control Register 2 Bits
#define DRV8301_CTRL2_OCTW_SET_BITS      (3 << 0)
#define DRV8301_CTRL2_GAIN_BITS          (3 << 2)
#define DRV8301_CTRL2_DC_CAL_1_BITS      (1 << 4)
#define DRV8301_CTRL2_DC_CAL_2_BITS      (1 << 5)
#define DRV8301_CTRL2_OC_TOFF_BITS       (1 << 6)
// ... (All Enums remain identical) ...
typedef enum {
  DRV8301_PeakCurrent_1p70_A  = 0 << 0,
  DRV8301_PeakCurrent_0p70_A  = 1 << 0,
  DRV8301_PeakCurrent_0p25_A  = 2 << 0
} DRV8301_PeakCurrent_e;
typedef enum {
  DRV8301_PwmMode_Six_Inputs   = 0 << 3,
  DRV8301_PwmMode_Three_Inputs = 1 << 3
} DRV8301_PwmMode_e;
typedef enum {
  DRV8301_OcMode_CurrentLimit  = 0 << 4,
  DRV8301_OcMode_LatchShutDown = 1 << 4,
  DRV8301_OcMode_ReportOnly    = 2 << 4,
  DRV8301_OcMode_Disabled      = 3 << 4
} DRV8301_OcMode_e;
typedef enum {
  DRV8301_VdsLevel_0p060_V =  0 << 6,
  DRV8301_VdsLevel_0p068_V =  1 << 6,
  DRV8301_VdsLevel_0p076_V =  2 << 6,
  DRV8301_VdsLevel_0p086_V =  3 << 6,
  DRV8301_VdsLevel_0p097_V =  4 << 6,
  DRV8301_VdsLevel_0p109_V =  5 << 6,
  DRV8301_VdsLevel_0p123_V =  6 << 6,
  DRV8301_VdsLevel_0p138_V =  7 << 6,
  DRV8301_VdsLevel_0p155_V =  8 << 6,
  DRV8301_VdsLevel_0p175_V =  9 << 6,
  DRV8301_VdsLevel_0p197_V = 10 << 6,
  DRV8301_VdsLevel_0p222_V = 11 << 6,
  DRV8301_VdsLevel_0p250_V = 12 << 6,
  DRV8301_VdsLevel_0p282_V = 13 << 6,
  DRV8301_VdsLevel_0p317_V = 14 << 6,
  DRV8301_VdsLevel_0p358_V = 15 << 6,
  DRV8301_VdsLevel_0p403_V = 16 << 6,
  DRV8301_VdsLevel_0p454_V = 17 << 6,
  DRV8301_VdsLevel_0p511_V = 18 << 6,
  DRV8301_VdsLevel_0p576_V = 19 << 6,
  DRV8301_VdsLevel_0p648_V = 20 << 6,
  DRV8301_VdsLevel_0p730_V = 21 << 6,
  DRV8301_VdsLevel_0p822_V = 22 << 6,
  DRV8301_VdsLevel_0p926_V = 23 << 6,
  DRV8301_VdsLevel_1p043_V = 24 << 6,
  DRV8301_VdsLevel_1p175_V = 25 << 6,
  DRV8301_VdsLevel_1p324_V = 26 << 6,
  DRV8301_VdsLevel_1p491_V = 27 << 6,
  DRV8301_VdsLevel_1p679_V = 28 << 6,
  DRV8301_VdsLevel_1p892_V = 29 << 6,
  DRV8301_VdsLevel_2p131_V = 30 << 6,
  DRV8301_VdsLevel_2p400_V = 31 << 6
} DRV8301_VdsLevel_e;
typedef enum {
  DRV8301_OcTwMode_Both    = 0 << 0,
  DRV8301_OcTwMode_OT_Only = 1 << 0,
  DRV8301_OcTwMode_OC_Only = 2 << 0
} DRV8301_OcTwMode_e;
typedef enum {
  DRV8301_ShuntAmpGain_10VpV = 0 << 2,
  DRV8301_ShuntAmpGain_20VpV = 1 << 2,
  DRV8301_ShuntAmpGain_40VpV = 2 << 2,
  DRV8301_ShuntAmpGain_80VpV = 3 << 2
} DRV8301_ShuntAmpGain_e;
typedef enum {
  DRV8301_DcCalMode_Ch1_Load   = (0 << 4),
  DRV8301_DcCalMode_Ch1_NoLoad = (1 << 4),
} DRV8301_DcCalMode_e;
typedef enum {
  DRV8301_DcCalMode_Ch2_Load   = (0 << 5),
  DRV8301_DcCalMode_Ch2_NoLoad = (1 << 5)
} DRV8301_DcCalMode_e_ch2;
typedef enum {
  DRV8301_OcOffTimeMode_Normal  = 0 << 6,
  DRV8301_OcOffTimeMode_Ctrl    = 1 << 6
} DRV8301_OcOffTimeMode_e;


// Helper function to build the 16-bit SPI control word
static inline uint16_t DRV8301_buildCtrlWord(const DRV8301_CtrlMode_e ctrlMode,
                                             const DRV8301_RegName_e regName,
                                             const uint16_t data) {
  return (ctrlMode | regName | (data & DRV8301_DATA_MASK));
}


class DRV8301 {
public:
    // Constructor
    DRV8301();

    void init(SPIClass* spi, int ncs_pin);

    // --- Configuration ---
    void setVerbose(bool verbose);

    // --- High-Level Handles ---
    bool setGateCurrent(DRV8301_PeakCurrent_e current);
    bool setPWMMode(DRV8301_PwmMode_e mode);
    bool setOcMode(DRV8301_OcMode_e mode);
    bool setOcAdj(DRV8301_VdsLevel_e level);
    void triggerReset();
    
    bool setOcTwMode(DRV8301_OcTwMode_e mode);
    bool setGain(DRV8301_ShuntAmpGain_e gain);
    bool setDcCal1(DRV8301_DcCalMode_e mode);
    bool setDcCal2(DRV8301_DcCalMode_e_ch2 mode);
    bool setOcToff(DRV8301_OcOffTimeMode_e mode);

    // --- Verbose Printers ---
    void printStatus();
    void printConfig();

    // --- Low-Level Access ---
    uint16_t read_reg(DRV8301_RegName_e reg);
    void write_reg(DRV8301_RegName_e reg, uint16_t data);

private:
    SPIClass* _spi;
    int _ncs_pin;
    SPISettings _spi_settings;
    bool _verbose = false; // <-- ADDED

    // --- Private Helper Functions ---
    bool write_field(DRV8301_RegName_e reg, uint16_t mask, uint16_t value);
    
    // --- String Converters for Verbose Printing ---
    const char* getGateCurrentString(uint16_t val);
    const char* getPWMModeString(uint16_t val);
    const char* getOcModeString(uint16_t val);
    const char* getVdsLevelString(uint16_t val);
    const char* getOcTwModeString(uint16_t val);
    const char* getGainString(uint16_t val);
    const char* getDcCal1String(uint16_t val);
    const char* getDcCal2String(uint16_t val);
    const char* getOcToffString(uint16_t val);
};

#endif // DRV8301_H