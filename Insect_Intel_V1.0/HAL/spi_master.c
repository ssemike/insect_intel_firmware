#include "spi_master.h"
#include <string.h>

SPI_Controller_Handle stm32Spi; 

uint8_t gSPI_TxPacket[SPI_PACKET_SIZE] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10
};
uint8_t gSPI_RxPacket[SPI_PACKET_SIZE];
uint8_t expectedRxMessage[SPI_PACKET_SIZE] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10
};


void SPI_Controller_Init(SPI_Controller_Handle *handle, SPI_Regs *spi, uint8_t txCh, uint8_t rxCh, uint8_t *txBuf, uint8_t *rxBuf, uint16_t len) 
{
    handle->spiInst = spi;
    handle->dmaInst = DMA;
    handle->txDmaCh = txCh;
    handle->rxDmaCh = rxCh;
    handle->txBuf   = txBuf;
    handle->rxBuf   = rxBuf;
    handle->size    = len;
    
    handle->txDone  = false;
    handle->rxDone  = false;
    handle->spiTransmitted = false;
}

void SPI_Controller_Arm(SPI_Controller_Handle *handle) {
    // 1. Reset Logic Flags
    handle->txDone  = false;
    handle->rxDone  = false;
    handle->spiTransmitted = false;

    // 2. Clear RX buffer
    memset(handle->rxBuf, 0, handle->size);

    // 3. Configure DMA TX (Memory -> SPI)
    DL_DMA_setSrcAddr(handle->dmaInst, handle->txDmaCh, (uint32_t)handle->txBuf);
    DL_DMA_setDestAddr(handle->dmaInst, handle->txDmaCh, (uint32_t)(&handle->spiInst->TXDATA));
    DL_DMA_setTransferSize(handle->dmaInst, handle->txDmaCh, handle->size);

    // 4. Configure DMA RX (SPI -> Memory)
    DL_DMA_setSrcAddr(handle->dmaInst, handle->rxDmaCh, (uint32_t)(&handle->spiInst->RXDATA));
    DL_DMA_setDestAddr(handle->dmaInst, handle->rxDmaCh, (uint32_t)handle->rxBuf);
    DL_DMA_setTransferSize(handle->dmaInst, handle->rxDmaCh, handle->size);

    // 5. Enable Channels - RX first, then TX to start clock
    DL_DMA_enableChannel(handle->dmaInst, handle->rxDmaCh);
    DL_DMA_enableChannel(handle->dmaInst, handle->txDmaCh);
}


void SPI_1_INST_IRQHandler(void) {
    switch (DL_SPI_getPendingInterrupt(SPI_1_INST)) {
        case DL_SPI_IIDX_DMA_DONE_TX:
            stm32Spi.txDone = true;
            break;
        case DL_SPI_IIDX_TX_EMPTY:
            stm32Spi.spiTransmitted = true;
            break;
        case DL_SPI_IIDX_DMA_DONE_RX:
            stm32Spi.rxDone = true;
            bool success = true;
            for (int i = 0; i < SPI_PACKET_SIZE; i++) {
                if (gSPI_RxPacket[i] != expectedRxMessage[i]) { success = false; break; }
            }
            break;
        default: break;
    }
}

