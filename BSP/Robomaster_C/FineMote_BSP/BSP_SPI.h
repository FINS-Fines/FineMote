//
// Created by wzj on 26-1-23.
//

#ifndef FINEMOTE_BSP_SPI_H
#define FINEMOTE_BSP_SPI_H
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

extern void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
extern void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

#ifdef __cplusplus
}
#endif

#endif //FINEMOTE_BSP_SPI_H

