#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#include "wiringPi.h"
#include "wiringPiI2C.h"

#include <spdlog/fmt/fmt.h>
#include <string_view>

// I2C address for the EMC2021
#define I2C_ADDRESS 0x4C

enum class Emc2101DACConversionRate {
  EMC2101_DAC_1_EVERY_16_S,
  EMC2101_DAC_1_EVERY_8_S,
  EMC2101_DAC_1_EVERY_4_S,
  EMC2101_DAC_1_EVERY_2_S,
  EMC2101_DAC_1_EVERY_SECOND,
  EMC2101_DAC_2_EVERY_SECOND,
  EMC2101_DAC_4_EVERY_SECOND,
  EMC2101_DAC_8_EVERY_SECOND,
  EMC2101_DAC_16_EVERY_SECOND,
  EMC2101_DAC_32_EVERY_SECOND,
};


/// Remap \p value from the range (\p min, \p max) to (\p min_out, \p max_out).
template<typename T, typename U> T remap(U value, U min, U max, T min_out, T max_out) {
  return (value - min) * (max_out - min_out) / (max - min) + min_out;
}

/// @brief Formatter specialization for Emc2101DACConversionRate enum.
/// Allows pretty-printing of the enum values using the fmt library.
template <>
struct fmt::formatter<Emc2101DACConversionRate> {
    // optional parse implementation - accept default format specs
    constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

    auto format(Emc2101DACConversionRate rate, fmt::format_context& ctx) -> decltype(ctx.out()) {
        std::string_view name = "Unknown";
        switch (rate) {
            case Emc2101DACConversionRate::EMC2101_DAC_1_EVERY_16_S:  name = "1 every 16s"; break;
            case Emc2101DACConversionRate::EMC2101_DAC_1_EVERY_8_S:   name = "1 every 8s"; break;
            case Emc2101DACConversionRate::EMC2101_DAC_1_EVERY_4_S:   name = "1 every 4s"; break;
            case Emc2101DACConversionRate::EMC2101_DAC_1_EVERY_2_S:   name = "1 every 2s"; break;
            case Emc2101DACConversionRate::EMC2101_DAC_1_EVERY_SECOND:name = "1 every 1s"; break;
            case Emc2101DACConversionRate::EMC2101_DAC_2_EVERY_SECOND:name = "2 every 1s"; break;
            case Emc2101DACConversionRate::EMC2101_DAC_4_EVERY_SECOND:name = "4 every 1s"; break;
            case Emc2101DACConversionRate::EMC2101_DAC_8_EVERY_SECOND:name = "8 every 1s"; break;
            case Emc2101DACConversionRate::EMC2101_DAC_16_EVERY_SECOND:name = "16 every 1s"; break;
            case Emc2101DACConversionRate::EMC2101_DAC_32_EVERY_SECOND:name = "32 every 1s"; break;
        }
        return fmt::format_to(ctx.out(), "{}", name);
    }
};


class EMC2101 {

public:
    explicit EMC2101();
    ~EMC2101();


    void set_dac_mode(bool dac_mode) {
        this->dac_mode_ = dac_mode;
        this->max_output_value_ = 63;
    }

    /** Sets the PWM resolution.
     *
     * @param resolution the PWM resolution.
     */
    void set_pwm_resolution(uint8_t resolution) {
        this->pwm_resolution_ = resolution;
        this->max_output_value_ = 2 * resolution;
    }

    /** Sets the PWM divider used to derive the PWM frequency.
     *
     * @param divider The PWM divider.
     */
    void set_pwm_divider(uint8_t divider) { this->pwm_divider_ = divider; }

    /** Sets the DAC conversion rate (how many conversions per second).
     *
     * @param conversion_rate The DAC conversion rate.
     */
    void set_dac_conversion_rate(Emc2101DACConversionRate conversion_rate) {
        this->dac_conversion_rate_ = conversion_rate;
    }

    /** Inverts the polarity of the Fan output.
     *
     * @param inverted Invert or not the Fan output.
     */
    void set_inverted(bool inverted) { this->inverted_ = inverted; }

    /** Sets the Fan output duty cycle
     *
     * @param value The duty cycle value, from 0.0f to 1.0f.
     */
    void set_duty_cycle(float value);

    /** Gets the Fan output duty cycle
     *
     * @return The duty cycle percentage from 0.0f to 1.0f.
     */
    float get_duty_cycle();

    /** Gets the internal temperature sensor reading.
     *
     * @return The temperature in degrees celsius.
     */
    float get_internal_temperature();

    /** Gets the external temperature sensor reading.
     *
     * @return The temperature in degrees celsius.
     */
    float get_external_temperature();

    /** Gets the tachometer speed sensor reading.
     *
     * @return The fan speed in RPMs.
     */
    float get_speed();

    // Setup EMC2101
    void emc2101_setup();

    void emc2101_dump_config();


protected:

    bool read_byte(uint8_t reg, uint8_t* value);
    bool write_byte(uint8_t reg, uint8_t value);

    int fd_;
    bool dac_mode_;
    uint8_t pwm_resolution_;
    uint8_t pwm_divider_;
    Emc2101DACConversionRate dac_conversion_rate_;
    bool inverted_;
    uint8_t max_output_value_;

};

