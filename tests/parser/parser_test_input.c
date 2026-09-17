#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include "parser_test_input.h"

static const uint8_t* test_input;
static size_t         test_input_length;
static size_t         test_read_index;

void parser_test_set_input(const uint8_t* input, size_t length)
{
        assert(input != NULL || length == 0u);
        test_input        = input;
        test_input_length = length;
        test_read_index   = 0u;
}

size_t parser_test_bytes_read(void) { return test_read_index; }

/* Host-side replacement for the RX ring-buffer reader used by the parsers. */
int8_t read_byte(void)
{
        assert(test_read_index < test_input_length);
        return (int8_t)test_input[test_read_index++];
}

/* Keep byte decoding identical to the target implementation. */
uint32_t dec_little_endian(const uint8_t length)
{
        uint32_t value = 0u;

        for (uint8_t i = 0u; i < length; ++i)
                value |= (uint32_t)(uint8_t)read_byte() << (i * 8u);

        return value;
}
