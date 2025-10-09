#ifndef PINS_H
#define PINS_H

// Odrive M0 motor pinout
#define M0_INH_A PA8
#define M0_INH_B PA9
#define M0_INH_C PA10
#define M0_INL_A PB13
#define M0_INL_B PB14
#define M0_INL_C PB15

// M0 currnets
#define M0_IB PC0
#define M0_IC PC1

// Odrive M0 encoder pinout
#define M0_ENC_A PB4
#define M0_ENC_B PB5
#define M0_ENC_Z PC9

// M1 & M2 common enable pin
#define EN_GATE PB12

// SPI pinout
#define SPI3_SCL  PC10
#define SPI3_MISO PC11
#define SPI3_MOSO PC12
#define SPI3_CS   PC13
#define nFAULT    PD2

#endif // PINS_H