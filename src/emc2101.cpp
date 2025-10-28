#include "emc2101.h"
#include <spdlog/spdlog.h>

static const uint8_t EMC2101_CHIP_ID = 0x16;      // EMC2101 default device id from part id
static const uint8_t EMC2101_ALT_CHIP_ID = 0x28;  // EMC2101 alternate device id from part id

// EMC2101 registers from the datasheet. We only define what we use.
static const uint8_t EMC2101_REGISTER_INTERNAL_TEMP = 0x00;      // The internal temperature register
static const uint8_t EMC2101_REGISTER_EXTERNAL_TEMP_MSB = 0x01;  // high byte for the external temperature reading
static const uint8_t EMC2101_REGISTER_DAC_CONV_RATE = 0x04;      // DAC convesion rate config
static const uint8_t EMC2101_REGISTER_EXTERNAL_TEMP_LSB = 0x10;  // low byte for the external temperature reading
static const uint8_t EMC2101_REGISTER_CONFIG = 0x03;             // configuration register
static const uint8_t EMC2101_REGISTER_TACH_LSB = 0x46;           // Tach RPM data low byte
static const uint8_t EMC2101_REGISTER_TACH_MSB = 0x47;           // Tach RPM data high byte
static const uint8_t EMC2101_REGISTER_FAN_CONFIG = 0x4A;         // General fan config register
static const uint8_t EMC2101_REGISTER_FAN_SETTING = 0x4C;        // Fan speed for non-LUT settings
static const uint8_t EMC2101_REGISTER_PWM_FREQ = 0x4D;           // PWM frequency setting
static const uint8_t EMC2101_REGISTER_PWM_DIV = 0x4E;            // PWM frequency divisor
static const uint8_t EMC2101_REGISTER_CHIP_ID = 0xFD;             // Chip ID register


// EMC2101 configuration bits from the datasheet. We only define what we use.

// Determines the funcionallity of the ALERT/TACH pin.
// 0 (default): The ALERT/TECH pin will function as an open drain, active low interrupt.
// 1: The ALERT/TECH pin will function as a high impedance TACH input. This may require an
// external pull-up resistor to set the proper signaling levels.
static const uint8_t EMC2101_ALT_TCH_BIT = 1 << 2;

// Determines the FAN output mode.
// 0 (default): PWM output enabled at FAN pin.
// 1: DAC output enabled at FAN ping.
static const uint8_t EMC2101_DAC_BIT = 1 << 4;

// Overrides the CLK_SEL bit and uses the Frequency Divide Register to determine
// the base PWM frequency. It is recommended that this bit be set for maximum PWM resolution.
// 0 (default): The base clock frequency for the PWM is determined by the CLK_SEL bit.
// 1 (recommended): The base clock that is used to determine the PWM frequency is set by the
// Frequency Divide Register
static const uint8_t EMC2101_CLK_OVR_BIT = 1 << 2;

// Sets the polarity of the Fan output driver.
// 0 (default): The polarity of the Fan output driver is non-inverted. A '00h' setting will
// correspond to a 0% duty cycle or a minimum DAC output voltage.
// 1: The polarity of the Fan output driver is inverted. A '00h' setting will correspond to a
// 100% duty cycle or a maximum DAC output voltage.
static const uint8_t EMC2101_POLARITY_BIT = 1 << 4;


// Default constructor
EMC2101::EMC2101()
{
    emc2101_setup();
}

// Default destructor
EMC2101::~EMC2101()
{
    if ( this->fd_ >= 0 ) {
        close(this->fd_);
    }
    spdlog::info("EMC2101 destructor called, I2C connection closed.");
}


void EMC2101::emc2101_setup() {

    spdlog::info("Setting up EMC2101...");

    int fd = wiringPiI2CSetup(I2C_ADDRESS);
    this->fd_ = fd;

    if ( fd < 0 ) {
        spdlog::error("Failed to initialize I2C communication. Check your connections.");
        return;
    }

    uint8_t chip_id = static_cast<int>(wiringPiI2CReadReg8(this->fd_, EMC2101_REGISTER_CHIP_ID));
    // make sure we're talking to the right chip
    if ( chip_id != EMC2101_CHIP_ID && chip_id != EMC2101_ALT_CHIP_ID ) {
        spdlog::error("Wrong chip ID {:02X}", chip_id);
        return;
    }

    // Configure EMC2101
    uint8_t config;
    config |= EMC2101_ALT_TCH_BIT;

    if (  this->dac_mode_ ) {
        config |= EMC2101_DAC_BIT;
        
    }

    if ( this->inverted_ ) {
        config |= EMC2101_POLARITY_BIT;
    }

    
    if ( this->dac_mode_ ) {
        uint8_t dac_conv_rate = static_cast<uint8_t>(this->dac_conversion_rate_);
        spdlog::info("Configuring EMC2101 in DAC mode");
    } 
    else {
        uint8_t fan_config; 
        fan_config |= EMC2101_CLK_OVR_BIT;
        uint8_t pwm_divider = this->pwm_divider_;
        uint8_t pwm_resolution = this->pwm_resolution_;
    }
}

bool EMC2101::read_byte(uint8_t reg, uint8_t *value)
{
    if ( wiringPiI2CReadReg8(this->fd_, reg) == -1 ) {
        spdlog::error("Failed to read byte from register 0x{:02X}", reg);
        return false;
    }
    *value = static_cast<uint8_t>(wiringPiI2CReadReg8(this->fd_, reg));
    return true;
}

bool EMC2101::write_byte(uint8_t reg, uint8_t value)
{
    if ( wiringPiI2CWriteReg8( this->fd_, reg, value) == -1 ) {
        spdlog::error("Failed to write byte to register 0x{:02X}", reg);
        return false;
    }
    return true;
}

void EMC2101::emc2101_dump_config() {
    spdlog::info("EMC2101 Mode: {}", this->dac_mode_ ? "DAC" : "PWM");
    if ( this->dac_mode_ ) {
        spdlog::info("EMC2101 DAC Conversion Rate: {}", this->dac_conversion_rate_);
    } else {
        spdlog::info("EMC2101 PWM Resolution: {}", this->pwm_resolution_);
        spdlog::info("EMC2101 PWM Divider: {}", this->pwm_divider_);
    }
    spdlog::info("EMC2101 Inverted: {}", this->inverted_ ? "YES" : "NO");
}

void EMC2101::set_duty_cycle(float value) {
    uint8_t duty_cycle = remap(value, 0.0f, 1.0f, (uint8_t) 0, this->max_output_value_);
    spdlog::debug("Setting EMC2101 duty cycle to {}", duty_cycle);
    if ( !this->write_byte(EMC2101_REGISTER_FAN_SETTING, duty_cycle) ) {
        spdlog::error("Failed to set duty cycle");
        return;
    }
}

float EMC2101::get_duty_cycle() {
    uint8_t duty_cycle;
    if ( !this->read_byte(EMC2101_REGISTER_FAN_SETTING, &duty_cycle) ) {
        spdlog::error("Failed to read duty cycle");
        return NAN;
    }
    return remap(duty_cycle, (uint8_t) 0, this->max_output_value_, 0.0f, 1.0f);
}

float EMC2101::get_internal_temperature() {
    uint8_t temperature;
    if ( !this->read_byte(EMC2101_REGISTER_INTERNAL_TEMP, &temperature) ) {
        spdlog::error("Failed to read internal temperature");
        return NAN;
    }
    return temperature;
}

float EMC2101::get_external_temperature() {
    // Read **MSB** first to match 'Data Read Interlock' behavior from 6.1 of datasheet
    uint8_t lsb, msb;
    if ( !this->read_byte(EMC2101_REGISTER_EXTERNAL_TEMP_MSB, &msb) ||
         !this->read_byte(EMC2101_REGISTER_EXTERNAL_TEMP_LSB, &lsb) ) {
        spdlog::error("Failed to read external temperature");
        return NAN;
    }

    // join msb and lsb (5 least significant bits are not used)
    uint16_t raw = (msb << 8 | lsb) >> 5;
    return raw * 0.125;
}

float EMC2101::get_speed() {
    // Read **LSB** first to match 'Data Read Interlock' behavior from 6.1 of datasheet
    uint8_t lsb, msb;
    if (!this->read_byte(EMC2101_REGISTER_TACH_LSB, &lsb) || !this->read_byte(EMC2101_REGISTER_TACH_MSB, &msb)) {
        spdlog::error("Failed to read tachometer");
        return NAN;
    }

    // calculate RPMs
    uint16_t tach = msb << 8 | lsb;
    return tach == 0xFFFF ? 0.0f : 5400000.0f / tach;
}





