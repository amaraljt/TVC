#ifndef W25QXX_H
#define W25QXX_H

#include <stdint.h>

/* Universal across the W25Q family regardless of capacity. */
#define W25_PAGE_SIZE   256u

void    W25_Write_Enable(void);
uint8_t W25_Read_Status(void);
void    W25_Wait_Busy(void);

/* Both are async: they return once the DMA transfer is QUEUED, not once it
   completes. CS drops on the completion callback, not at the call site - see
   the .c file. data/buf must stay valid until then:
     - W25_Write_Page copies data into its own static buffer before
       returning, so the caller's buffer is free to reuse immediately.
     - W25_Read_Flash has no such copy - buf is the DMA destination directly,
       so it must stay valid (not stack-local past return, not reused) until
       the transfer completes. */
void W25_Write_Page(uint32_t addr, const uint8_t *data, uint16_t len);
void W25_Read_Flash(uint32_t addr, uint8_t *buf, uint16_t len);

#endif /* W25QXX_H */
