#pragma once

#include <cstdint>
#include "as5045.h"
#include "macros.h"

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

namespace gpio {

/*
 * Update states of gpio expander pins.
 * Sets those bits that are 1 in the argument `bits_to_set`.
 * Resets those bits that are 1 in the argument `bits_to_reset`.
 * Example: to set pin 3 and reset pin 5 (pins are from 0 to 7):
 *          gpio::update_pins(1 << 3, 1 << 5);
 *
 * TODO: try using std::bitset<8> instead
 */
bool update_pins(uint8_t bits_to_set, uint8_t bits_to_reset);

static constexpr uint8_t LED_BLUE = bit(7);
static constexpr uint8_t LED_RED  = bit(6);
// LEDS are connected in different order than in pcb/plots/ubik.pdf TODO: update this pdf
// they are actually connected in the order:
//    GPIOEX:  0  1  2  3  4  5
//        DS:  3  2  1  0  4  5  (0-indexed)
static constexpr uint8_t DISTANCE_SENSORS[] = {bit(3), bit(2), bit(1), bit(0), bit(4), bit(5)};
static constexpr uint8_t DISTANCE_SENSORS_ALL() {
    uint8_t bits = 0x00;
    for (uint8_t b: DISTANCE_SENSORS) bits |= b;
    return bits;
}
static_assert(DISTANCE_SENSORS_ALL() == 0b00111111);

} // namespace gpio


} // namespace spi
