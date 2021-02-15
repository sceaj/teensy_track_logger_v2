/*
 * eeprom.h
 *
 *  Created on: Feb 2, 2021
 *      Author: jrosen
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include "stdint.h"

#define SCB_CACHE_DCIMVAC       (*(volatile uint32_t *)0xE000EF5C)

#define FLEXSPI_LUT_INSTRUCTION(opcode, pads, operand) ((uint32_t)(\
        (((opcode) & 0x3F) << 10) | (((pads) & 0x03) << 8) | ((operand) & 0xFF)))
// 30.7.8: page 1532
#define FLEXSPI_LUT_OPCODE_CMD_SDR              0x01
//#define FLEXSPI_LUT_OPCODE_CMD_DDR              0x21
#define FLEXSPI_LUT_OPCODE_RADDR_SDR            0x02
//#define FLEXSPI_LUT_OPCODE_RADDR_DDR            0x22
//#define FLEXSPI_LUT_OPCODE_CADDR_SDR            0x03
//#define FLEXSPI_LUT_OPCODE_CADDR_DDR            0x23
//#define FLEXSPI_LUT_OPCODE_MODE1_SDR            0x04
//#define FLEXSPI_LUT_OPCODE_MODE1_DDR            0x24
//#define FLEXSPI_LUT_OPCODE_MODE2_SDR            0x05
//#define FLEXSPI_LUT_OPCODE_MODE2_DDR            0x25
//#define FLEXSPI_LUT_OPCODE_MODE4_SDR            0x06
//#define FLEXSPI_LUT_OPCODE_MODE4_DDR            0x26
//#define FLEXSPI_LUT_OPCODE_MODE8_SDR            0x07
//#define FLEXSPI_LUT_OPCODE_MODE8_DDR            0x27
#define FLEXSPI_LUT_OPCODE_WRITE_SDR            0x08
//#define FLEXSPI_LUT_OPCODE_WRITE_DDR            0x28
#define FLEXSPI_LUT_OPCODE_READ_SDR             0x09
//#define FLEXSPI_LUT_OPCODE_READ_DDR             0x29
//#define FLEXSPI_LUT_OPCODE_LEARN_SDR            0x0A
//#define FLEXSPI_LUT_OPCODE_LEARN_DDR            0x2A
//#define FLEXSPI_LUT_OPCODE_DATSZ_SDR            0x0B
//#define FLEXSPI_LUT_OPCODE_DATSZ_DDR            0x2B
//#define FLEXSPI_LUT_OPCODE_DUMMY_SDR            0x0C
//#define FLEXSPI_LUT_OPCODE_DUMMY_DDR            0x2C
//#define FLEXSPI_LUT_OPCODE_DUMMY_RWDS_SDR       0x0D
//#define FLEXSPI_LUT_OPCODE_DUMMY_RWDS_DDR       0x2D
//#define FLEXSPI_LUT_OPCODE_JMP_ON_CS            0x1F
//#define FLEXSPI_LUT_OPCODE_STOP                 0x00
#define FLEXSPI_LUT_NUM_PADS_1                  0x00
//#define FLEXSPI_LUT_NUM_PADS_2                  0x01
#define FLEXSPI_LUT_NUM_PADS_4                  0x02
//#define FLEXSPI_LUT_NUM_PADS_8                  0x03

#define LUT0(opcode, pads, operand) (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)))
#define LUT1(opcode, pads, operand) (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)) << 16)
#define CMD_SDR         FLEXSPI_LUT_OPCODE_CMD_SDR
#define ADDR_SDR        FLEXSPI_LUT_OPCODE_RADDR_SDR
#define READ_SDR        FLEXSPI_LUT_OPCODE_READ_SDR
#define WRITE_SDR       FLEXSPI_LUT_OPCODE_WRITE_SDR
#define PINS1           FLEXSPI_LUT_NUM_PADS_1
#define PINS4           FLEXSPI_LUT_NUM_PADS_4


// Normally arm_dcache_delete() is used before receiving data via
// DMA or from bus-master peripherals which write to memory.  You
// want to delete anything the cache may have stored, so your next
// read is certain to access the physical memory.
__attribute__((always_inline, unused))
static inline void arm_dcache_delete(void *addr, uint32_t size)
{
        uint32_t location = (uint32_t)addr & 0xFFFFFFE0;
        uint32_t end_addr = (uint32_t)addr + size;
        asm volatile("": : :"memory");
        asm("dsb");
        do {
                SCB_CACHE_DCIMVAC = location;
                location += 32;
        } while (location < end_addr);
        asm("dsb");
        asm("isb");
}

void eeprom_initialize(void);

int eeprom_is_ready(void);

uint8_t eeprom_read_byte(const uint8_t *addr_ptr);
uint16_t eeprom_read_word(const uint16_t *addr);
uint32_t eeprom_read_dword(const uint32_t *addr);
void eeprom_read_block(void *buf, const void *addr, uint32_t len);

void eeprom_write_byte(uint8_t *addr_ptr, uint8_t data);
void eeprom_write_word(uint16_t *addr, uint16_t value);
void eeprom_write_dword(uint32_t *addr, uint32_t value);
void eeprom_write_block(const void *buf, void *addr, uint32_t len);


#endif /* EEPROM_H_ */
