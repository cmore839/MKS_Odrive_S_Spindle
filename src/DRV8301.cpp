#include "DRV8301.h"

// DRV8301 class constructor using a member initializer list
DRV8301::DRV8301(int mosi, int miso, int sclk, int cs, int en_gate, int fault) :
    drv8301_mosi_pin(mosi),
    drv8301_miso_pin(miso),
    drv8301_sclk_pin(sclk),
    drv8301_cs_pin(cs),
    drv8301_en_gate_pin(en_gate),
    drv8301_fault_pin(fault)
{
}

#pragma GCC push_options
#pragma GCC optimize("O0") //Don't use GCC optimize
/**
 * Use for SPI timing's delay function
 * It's only test on STM32F405/F407 168MHz
 */
void DRV8301::spi_delay(void)
{
    for (int i = 0; i < 22; i++)
        ;
}
#pragma GCC pop_options

// SPI transfer 16 bit value
uint16_t DRV8301::spi_transfer(uint16_t txdata)
{
    uint16_t rxdata = 0;

    for (int i = 0; i < 16; i++)
    {
        digitalWrite(drv8301_mosi_pin, bitRead(txdata, 15 - i));
        digitalWrite(drv8301_sclk_pin, HIGH);
        spi_delay();
        digitalWrite(drv8301_sclk_pin, LOW);
        bitWrite(rxdata, 15 - i, digitalRead(drv8301_miso_pin));
        spi_delay();
    }
    return rxdata;
}

// [FIXED] Read DRV8301's register with correct single-transfer logic
int DRV8301::drv8301_read_reg(uint16_t reg)
{
    uint16_t command = 0x8000 | ((reg & 0x0F) << 11);

    digitalWrite(drv8301_cs_pin, LOW);
    // The received data from this single transfer is the register value
    uint16_t reg_val = spi_transfer(command);
    digitalWrite(drv8301_cs_pin, HIGH);

    // The actual data is in the lower 11 bits
    return reg_val & 0x07FF;
}

// Set DRV8301's register
void DRV8301::drv8301_write_reg(uint16_t reg, uint16_t data)
{
    digitalWrite(drv8301_cs_pin, LOW);
    spi_transfer(((reg & 0x0F) << 11) | (data & 0x07FF));
    digitalWrite(drv8301_cs_pin, HIGH);
}

// [ADDED] Set the gain for the current sense amplifiers
void DRV8301::set_csa_gain(uint16_t gain)
{
    // 1. Read the current value of Control Register 1
    drv8301_ctrl_reg1_val = drv8301_read_reg(DRV8301_CONTROL_REG1);

    // 2. Clear only the gain bits using the inverted mask
    drv8301_ctrl_reg1_val &= ~CSA_GAIN_MASK;

    // 3. Set the new gain bits
    drv8301_ctrl_reg1_val |= gain;

    // 4. Write the modified value back to the register
    drv8301_write_reg(DRV8301_CONTROL_REG1, drv8301_ctrl_reg1_val);
}


// Initialize pin and initialize DRV8301
void DRV8301::begin(DRV8301_PWM_INPUT_MODE pwm_mode)
{
    /** Initialize pin */
    pinMode(drv8301_en_gate_pin, OUTPUT);
    digitalWrite(drv8301_en_gate_pin, LOW);
    pinMode(drv8301_fault_pin, INPUT_PULLUP);
    pinMode(drv8301_cs_pin, OUTPUT);
    digitalWrite(drv8301_cs_pin, HIGH);
    pinMode(drv8301_mosi_pin, OUTPUT);
    pinMode(drv8301_miso_pin, INPUT);
    pinMode(drv8301_sclk_pin, OUTPUT);
    digitalWrite(drv8301_sclk_pin, LOW);

    /** Configure register */
    drv8301_ctrl_reg1_val = 0x0000;
    drv8301_ctrl_reg1_val = OCP_MODE_DISABLE | OC_ADJ_SET(27); //Disable OC

    switch (pwm_mode)
    {
    case PWM_INPUT_MODE_3PWM:
        drv8301_ctrl_reg1_val |= PWM_MODE_3_PWM_INPUTS;
        break;

    case PWM_INPUT_MODE_6PWM:
        drv8301_ctrl_reg1_val |= PWM_MODE_6_PWM_INPUTS;
        break;
    }

    reset();
}

// Reset DRV8301
void DRV8301::reset(void)
{
    /** Reset timing */
    digitalWrite(drv8301_en_gate_pin, LOW);
    delayMicroseconds(40);
    digitalWrite(drv8301_en_gate_pin, HIGH);
    delay(20);

    /** Update register value */
    drv8301_read_reg(DRV8301_STATUS_REG1); // Clear latched faults on wakeup
    drv8301_write_reg(DRV8301_CONTROL_REG1, drv8301_ctrl_reg1_val);
}

// Detect if DRV8301 has fault occurred
int DRV8301::is_fault(void)
{
    return (int)!digitalRead(drv8301_fault_pin);
}

// Read DRV8301's fault value
int DRV8301::read_fault(void)
{
    uint16_t reg1, reg2;
    reg1 = drv8301_read_reg(DRV8301_STATUS_REG1);
    reg2 = drv8301_read_reg(DRV8301_STATUS_REG2);
    
    // Note: masking is now technically redundant since read_reg handles it, but is kept for clarity.
    reg1 &= 0x03FF;
    reg2 &= 0x0080;
    return (int)(reg1 | reg2 << 3);
}

// Get DRV8301's chip id
int DRV8301::get_id(void)
{
    return drv8301_read_reg(DRV8301_STATUS_REG2) & 0x000F;
}