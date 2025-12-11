#include <Arduino.h>
#include <SPI.h>

// --- 1. PIN CONFIGURATION (from Firmware/Board/v3/Inc/mxconstants.h) ---
#define PIN_DRV_EN     PB12  // EN_GATE_Pin
#define PIN_DRV_NCS    PC13  // M0_nCS_Pin
#define PIN_SPI3_MOSI  PC12  //
#define PIN_SPI3_MISO  PC11  //
#define PIN_SPI3_SCK   PC10  //
// --- End Configuration ---


// --- 2. Initialize SPI3 ---
SPIClass drv_spi(PIN_SPI3_MOSI, PIN_SPI3_MISO, PIN_SPI3_SCK);


// --- 3. DRV8301 Definitions (Ported from drv8301.h) ---

// Masks
#define DRV8301_DATA_MASK               (0x07FF)
#define DRV8301_RW_MASK                 (0x8000)

// R/W modes
typedef enum {
  DRV8301_CtrlMode_Read = 1 << 15,
  DRV8301_CtrlMode_Write = 0 << 15
} DRV8301_CtrlMode_e;

// Register names
typedef enum {
  DRV8301_RegName_Status_1  = 0 << 11,
  DRV8301_RegName_Status_2  = 1 << 11,
  DRV8301_RegName_Control_1 = 2 << 11,
  DRV8301_RegName_Control_2 = 3 << 11
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

// Enums for Control Register 1
typedef enum {
  DRV8301_PeakCurrent_1p70_A  = 0 << 0,
  DRV8301_PeakCurrent_0p70_A  = 1 << 0,
  DRV8301_PeakCurrent_0p25_A  = 2 << 0
} DRV8301_PeakCurrent_e;

typedef enum {
  DRV8301_Reset_Normal = 0 << 2,
  DRV8301_Reset_All = 1 << 2
} DRV8301_Reset_e;

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
  DRV8301_VdsLevel_0p138_V =  7 << 6,
  DRV8301_VdsLevel_0p730_V = 21 << 6,
  DRV8301_VdsLevel_2p400_V = 31 << 6
} DRV8301_VdsLevel_e;

// Enums for Control Register 2
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
} DRV8301_DcCalMode_e_ch2; // Renamed to avoid conflict

typedef enum {
  DRV8301_OcOffTimeMode_Normal  = 0 << 6,
  DRV8301_OcOffTimeMode_Ctrl    = 1 << 6
} DRV8301_OcOffTimeMode_e;


typedef uint16_t DRV8301_Word_t;

// Build the 16-bit SPI control word
static inline DRV8301_Word_t DRV8301_buildCtrlWord(const DRV8301_CtrlMode_e ctrlMode,
                                                   const DRV8301_RegName_e regName,
                                                   const uint16_t data) {
  return (ctrlMode | regName | (data & DRV8301_DATA_MASK));
}

// SPI settings for DRV8301
SPISettings drv_spi_settings(1000000, MSBFIRST, SPI_MODE1);


// --- 4. Ported Driver Functions (Adapted from drv8301.c) ---

/**
 * @brief Writes data to a DRV8301 register.
 */
void drv_writeSpi(DRV8301_RegName_e regName, const uint16_t data) {
  uint16_t controlword = DRV8301_buildCtrlWord(DRV8301_CtrlMode_Write, regName, data);
  drv_spi.beginTransaction(drv_spi_settings);
  digitalWrite(PIN_DRV_NCS, LOW);
  delayMicroseconds(1);
  drv_spi.transfer16(controlword);
  delayMicroseconds(1);
  digitalWrite(PIN_DRV_NCS, HIGH);
  delayMicroseconds(1);
  drv_spi.endTransaction();
}

/**
 * @brief Reads data from a DRV8301 register.
 */
uint16_t drv_readSpi(const DRV8301_RegName_e regName) {
  uint16_t controlword = DRV8301_buildCtrlWord(DRV8301_CtrlMode_Read, regName, 0);
  drv_spi.beginTransaction(drv_spi_settings);
  digitalWrite(PIN_DRV_NCS, LOW);
  delayMicroseconds(1);
  drv_spi.transfer16(controlword);
  delayMicroseconds(1);
  digitalWrite(PIN_DRV_NCS, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_DRV_NCS, LOW);
  delayMicroseconds(1);
  uint16_t recbuff = drv_spi.transfer16(0x0000);
  delayMicroseconds(1);
  digitalWrite(PIN_DRV_NCS, HIGH);
  delayMicroseconds(1);
  drv_spi.endTransaction();
  return (recbuff & DRV8301_DATA_MASK);
}

/**
 * @brief Initializes GPIO and enables the DRV8301 chip.
 */
bool setup_drv8301() {
  Serial.println("Setting up DRV8301...");
  pinMode(PIN_DRV_EN, OUTPUT);
  pinMode(PIN_DRV_NCS, OUTPUT);
  
  digitalWrite(PIN_DRV_NCS, HIGH); // nCS is active LOW, so idle HIGH
  digitalWrite(PIN_DRV_EN, LOW);  // Keep disabled for now
  delay(1);

  drv_spi.begin();
  Serial.println("Enabling driver (EN_GATE HIGH)");
  digitalWrite(PIN_DRV_EN, HIGH);
  delay(10); // osDelay(10)

  uint16_t status1 = drv_readSpi(DRV8301_RegName_Status_1);
  delay(1); // osDelay(1)

  if (status1 & DRV8301_STATUS1_FAULT_BITS) {
    Serial.println("WARNING: DRV8301 reports FAULT on startup!");
    return false;
  }
  Serial.println("DRV8301 online, no fault detected.");
  return true;
}


// --- 5. Test Functions ---

/**
 * @brief Helper function to test a specific field in a R/W register.
 */
void test_field(const char* field_name, DRV8301_RegName_e reg, uint16_t mask, uint16_t test_val) {
  Serial.print("  Testing ");
  Serial.print(field_name);
  Serial.print("...");
  
  // 1. Read default value
  uint16_t default_val = drv_readSpi(reg);
  
  // 2. Write the new test value, preserving other bits
  uint16_t write_val = (default_val & ~mask) | (test_val & mask);
  drv_writeSpi(reg, write_val);

  // 3. Read back and verify
  uint16_t read_val = drv_readSpi(reg);
  
  // 4. Restore default value
  drv_writeSpi(reg, default_val);

  // 5. Report
  if (read_val == write_val) {
    Serial.println(" SUCCESS");
  } else {
    Serial.print(" FAILED! (Wrote: 0b");
    Serial.print(write_val, BIN);
    Serial.print(", Read: 0b");
    Serial.print(read_val, BIN);
    Serial.println(")");
  }
}

/**
 * @brief Reads and prints a detailed report from status registers.
 */
void test_status_registers() {
  Serial.println("\n--- Reading Status Registers ---");
  
  uint16_t status1 = drv_readSpi(DRV8301_RegName_Status_1);
  Serial.print("Status Register 1 (0x00): 0b");
  Serial.println(status1, BIN);
  Serial.println("  Parsed:");
  Serial.print("  - FAULT:       "); Serial.println((status1 & DRV8301_STATUS1_FAULT_BITS) ? "YES" : "NO");
  Serial.print("  - GVDD_UV:     "); Serial.println((status1 & DRV8301_STATUS1_GVDD_UV_BITS) ? "YES" : "NO");
  Serial.print("  - PVDD_UV:     "); Serial.println((status1 & DRV8301_STATUS1_PVDD_UV_BITS) ? "YES" : "NO");
  Serial.print("  - OTSD:        "); Serial.println((status1 & DRV8301_STATUS1_OTSD_BITS) ? "YES" : "NO");
  Serial.print("  - OTW:         "); Serial.println((status1 & DRV8301_STATUS1_OTW_BITS) ? "YES" : "NO");
  Serial.print("  - FETHA_OC:    "); Serial.println((status1 & DRV8301_STATUS1_FETHA_OC_BITS) ? "YES" : "NO");
  Serial.print("  - FETLA_OC:    "); Serial.println((status1 & DRV8301_STATUS1_FETLA_OC_BITS) ? "YES" : "NO");
  Serial.print("  - FETHB_OC:    "); Serial.println((status1 & DRV8301_STATUS1_FETHB_OC_BITS) ? "YES" : "NO");
  Serial.print("  - FETLB_OC:    "); Serial.println((status1 & DRV8301_STATUS1_FETLB_OC_BITS) ? "YES" : "NO");
  Serial.print("  - FETHC_OC:    "); Serial.println((status1 & DRV8301_STATUS1_FETHC_OC_BITS) ? "YES" : "NO");
  Serial.print("  - FETLC_OC:    "); Serial.println((status1 & DRV8301_STATUS1_FETLC_OC_BITS) ? "YES" : "NO");

  uint16_t status2 = drv_readSpi(DRV8301_RegName_Status_2);
  Serial.print("\nStatus Register 2 (0x01): 0b");
  Serial.println(status2, BIN);
  Serial.println("  Parsed:");
  Serial.print("  - GVDD_OV:     "); Serial.println((status2 & DRV8301_STATUS2_GVDD_OV_BITS) ? "YES" : "NO");
  Serial.print("  - DeviceID:    0b"); Serial.println(status2 & DRV8301_STATUS2_ID_BITS, BIN);
}

/**
 * @brief Performs R/W test on all fields in Control Register 1.
 */
void test_control_register_1() {
  Serial.println("\n--- Testing Control Register 1 (0x02) ---");
  uint16_t default_val = drv_readSpi(DRV8301_RegName_Control_1);
  Serial.print("Default value: 0b");
  Serial.println(default_val, BIN);

  // Test GATE_CURRENT
  test_field("GATE_CURRENT (0.70A)", DRV8301_RegName_Control_1,
             DRV8301_CTRL1_GATE_CURRENT_BITS, DRV8301_PeakCurrent_0p70_A);
  
  // Test GATE_RESET
  // This bit is not sticky, so we just test writing 1 and restoring 0
  test_field("GATE_RESET (Reset)", DRV8301_RegName_Control_1,
             DRV8301_CTRL1_GATE_RESET_BITS, DRV8301_Reset_All);

  // Test PWM_MODE
  test_field("PWM_MODE (3-Inputs)", DRV8301_RegName_Control_1,
             DRV8301_CTRL1_PWM_MODE_BITS, DRV8301_PwmMode_Three_Inputs);

  // Test OC_MODE
  test_field("OC_MODE (Latch SD)", DRV8301_RegName_Control_1,
             DRV8301_CTRL1_OC_MODE_BITS, DRV8301_OcMode_LatchShutDown);

  // Test OC_ADJ_SET
  test_field("OC_ADJ_SET (0.138V)", DRV8301_RegName_Control_1,
             DRV8301_CTRL1_OC_ADJ_SET_BITS, DRV8301_VdsLevel_0p138_V);
}

/**
 * @brief Performs R/W test on all fields in Control Register 2.
 */
void test_control_register_2() {
  Serial.println("\n--- Testing Control Register 2 (0x03) ---");
  uint16_t default_val = drv_readSpi(DRV8301_RegName_Control_2);
  Serial.print("Default value: 0b");
  Serial.println(default_val, BIN);

  // Test OCTW_SET
  test_field("OCTW_SET (OC Only)", DRV8301_RegName_Control_2,
             DRV8301_CTRL2_OCTW_SET_BITS, DRV8301_OcTwMode_OC_Only);

  // Test GAIN
  test_field("GAIN (40VpV)", DRV8301_RegName_Control_2,
             DRV8301_CTRL2_GAIN_BITS, DRV8301_ShuntAmpGain_40VpV);

  // Test DC_CAL_1
  test_field("DC_CAL_1 (Short)", DRV8301_RegName_Control_2,
             DRV8301_CTRL2_DC_CAL_1_BITS, DRV8301_DcCalMode_Ch1_NoLoad);

  // Test DC_CAL_2
  test_field("DC_CAL_2 (Short)", DRV8301_RegName_Control_2,
             DRV8301_CTRL2_DC_CAL_2_BITS, DRV8301_DcCalMode_Ch2_NoLoad);

  // Test OC_TOFF
  test_field("OC_TOFF (CTRL)", DRV8301_RegName_Control_2,
             DRV8301_CTRL2_OC_TOFF_BITS, DRV8301_OcOffTimeMode_Ctrl);
}


// --- 6. Arduino Setup and Loop ---

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for Serial Monitor
  }
  Serial.println("\n\nDRV8301 Comprehensive Register Test");

  // Initialize and enable the driver
  if (setup_drv8301()) {
    // Run tests
    test_status_registers();
    test_control_register_1();
    test_control_register_2();
  } else {
    Serial.println("Could not initialize DRV8301. Halting tests.");
  }

  Serial.println("\n--- Test Complete ---");
}

void loop() {
  // Tests only run once in setup.
  digitalWrite(PIN_DRV_EN, LOW); // Disable driver when done
  delay(1000);
}