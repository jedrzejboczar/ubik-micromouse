#pragma once

#include <cstdint>
#include "as5045.h"

/*
 * This module guards the SPI peripheral by providing
 * functions that will assure that no data races occur
 * by using a mutex.
 *
 * These functions cannot be used from ISR routines.
 */

namespace spi {

struct EncoderReadings {
    bool valid;
    AS5045Reading left, right;

    bool both_ok() {
        return valid && left.is_ok() && right.is_ok();
    }
};

/* Prepares all devices */
void initialise();
/*
 * Reads data from both encoders.
 * valid=false means that the communication wasn't succesful
 * and thus readings cannot be used.
 * valid=true does not mean that AS5045Reading structures
 * are ok (contents of the readings is not checked).
 */
EncoderReadings read_encoders();
/*
 * Update states of gpio expander pins.
 * Sets those bits that are 1 in the argument `bits_to_set`.
 * Resets those bits that are 1 in the argument `bits_to_reset`.
 * Example: to set pin 3 and reset pin 5 (pins are from 0 to 7):
 *          update_gpio_expander_pins(1 << 3, 1 << 5);
 *
 * TODO: try using std::bitset<8> instead
 */
bool update_gpio_expander_pins(uint8_t bits_to_set, uint8_t bits_to_reset);




} // namespace spi
