#ifndef PARSER_TEST_INPUT_H
#define PARSER_TEST_INPUT_H

#include <stddef.h>
#include <stdint.h>

void   parser_test_set_input(const uint8_t* input, size_t length);
size_t parser_test_bytes_read(void);

#endif /* PARSER_TEST_INPUT_H */
