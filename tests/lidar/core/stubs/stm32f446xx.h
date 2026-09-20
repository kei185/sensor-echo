#ifndef TEST_STM32F446XX_H
#define TEST_STM32F446XX_H

#include <stdint.h>

typedef struct
{
        volatile uint32_t NDTR;
} DMA_Stream_TypeDef;

extern DMA_Stream_TypeDef mock_dma1_stream2;

#define DMA1_Stream2 (&mock_dma1_stream2)

#endif
