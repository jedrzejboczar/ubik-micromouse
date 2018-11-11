#pragma once

#include <stdint.h>


struct AS5045Reading {
    enum FieldStatus {
        GREEN, YELLOW, RED, INCREASE, DECREASE
    };

    union {
        uint32_t _data;
        struct {
            uint32_t angle	: 12;  // 12-bit value
            uint32_t OCF 	: 1;   // Offset Compensation Finished 	1 - ok
            uint32_t COF 	: 1;   // Coordic OverFlow				 	0 - ok
            uint32_t LIN 	: 1;   // Linearity Alarm					0 - ok
            uint32_t magINC : 1;   // (INC, DEC)  0, 0 - no movement
            uint32_t magDEC : 1;   // 			   1, 1 - wrong magnetic field
            uint32_t evenPAR: 1;
        };
    };

    // Must be created using static method.
    // `buffer` should have the contents of 3 bytes read from AS5045 over SSI
    // (SPI) interface.
    // SPI config that satisfies datasheet p.13 (CLK idle high, sample on
    // falling clock edge - first sampled bit has to be ignored):
    // - MSB first
    // - CLK polarity high
    // - CLK phase first edge (and ignore first bit!)
    static AS5045Reading from_buffer(uint8_t *buffer);

    FieldStatus field_status();
    bool is_pairty_ok();
    bool is_ok() {
        return OCF == 1 && COF == 0 && LIN == 0 && is_pairty_ok();
    }

private:
    AS5045Reading(): _data(0) { }
};

static_assert(sizeof(AS5045Reading) == 4, "This structure should take up 4 bytes");
