#include "as5045.h"

static constexpr auto bit(int num) {
    return 1 << num;
}

/*
 * Frame bits:
 * bit/byte:   7   6   5   4   3   2   1   0 . 7   6   5   4   3   2   1   0.  7   6   5
 * bit name: ___ D11 D10  D9  D8  D7  D6  D5  D4  D3  D2  D1  D0 OCF COF LIN INC DEC PAR
 * bit num:    0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18
 *
 * We ignore 1st bit because to read as in datasheet we need SPI with clk high
 * in idle and with reads on falling edge, so we will read 1 bit before data.
 */

AS5045Reading AS5045Reading::from_buffer(uint8_t *buffer) {
    AS5045Reading r;
    r.angle   = (static_cast<uint32_t>(buffer[0] & 0x7f) << 5) | (buffer[1] >> 3);
	r.OCF     = (buffer[1] & bit(2)) != 0;
	r.COF     = (buffer[1] & bit(1)) != 0;
	r.LIN     = (buffer[1] & bit(0)) != 0;
	r.magINC  = (buffer[2] & bit(7)) != 0;
	r.magDEC  = (buffer[2] & bit(6)) != 0;
	r.evenPAR = (buffer[2] & bit(5)) != 0;
	return r;
}

AS5045Reading::FieldStatus AS5045Reading::field_status() {
    if (magINC && magDEC)
        if (LIN)                return RED;
        else                    return YELLOW;
	// this is somehow illogically
	else if (magINC && !magDEC) return DECREASE;
	else if (!magINC && magDEC) return INCREASE;
	else                        return GREEN;
}

bool AS5045Reading::is_pairty_ok() {
	uint32_t is_even = _data;
    // to check parity we perform XOR on all bits
    // these operations store the result in last bit
    // of `is_even`, 0 for even, 1 for odd parity
	is_even ^= is_even >> 16;
	is_even ^= is_even >> 8;
	is_even ^= is_even >> 4;
	is_even ^= is_even >> 2;
	is_even ^= is_even >> 1;
	return is_even == 0;
}



