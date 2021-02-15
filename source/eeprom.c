/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2019 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows 
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// To configure the EEPROM size, edit E2END in avr/eeprom.h.
//
// Generally you should avoid editing this code, unless you really
// know what you're doing.

#include "stdint.h"
#include "fsl_device_registers.h"
#include "string.h"
#include "eeprom.h"

#define FLASH_BASEADDR 0x607C0000
#define FLASH_SECTORS  63

#define E2END 0x10BB

#if E2END > (255*FLASH_SECTORS-1)
#error "E2END is set larger than the maximum possible EEPROM size"
#endif

// Conversation about how this code works & what the upper limits are
// https://forum.pjrc.com/threads/57377?p=214566&viewfull=1#post214566

static void flash_write(void *addr, const void *data, uint32_t len);
static void flash_erase_sector(void *addr);

static uint8_t initialized=0;
static uint16_t sector_index[FLASH_SECTORS];

void eeprom_initialize(void)
{
	uint32_t sector;
	//printf("eeprom init\n");
	for (sector=0; sector < FLASH_SECTORS; sector++) {
		const uint16_t *p = (uint16_t *)(FLASH_BASEADDR + sector * 4096);
		const uint16_t *end = (uint16_t *)(FLASH_BASEADDR + (sector + 1) * 4096);
		uint16_t index = 0;
		do {
			if (*p++ == 0xFFFF) break;
			index++;
		} while (p < end);
		sector_index[sector] = index;
	}
	initialized = 1;
}

uint8_t eeprom_read_byte(const uint8_t *addr_ptr)
{
	uint32_t addr = (uint32_t)addr_ptr;
	uint32_t sector, offset;
	const uint16_t *p, *end;
	uint8_t data=0xFF;

	if (addr > E2END) return 0xFF;
	if (!initialized) eeprom_initialize();
	sector = (addr >> 2) % FLASH_SECTORS;
	offset = (addr & 3) | (((addr >> 2) / FLASH_SECTORS) << 2);
	//printf("ee_rd, addr=%u, sector=%u, offset=%u, len=%u\n",
		//addr, sector, offset, sector_index[sector]);
	p = (uint16_t *)(FLASH_BASEADDR + sector * 4096);
	end = p + sector_index[sector];
	while (p < end) {
		uint32_t val = *p++;
		if ((val & 255) == offset) data = val >> 8;
	}
	return data;
}

void eeprom_write_byte(uint8_t *addr_ptr, uint8_t data)
{
	uint32_t addr = (uint32_t)addr_ptr;
	uint32_t sector, offset, index, i;
	uint16_t *p, *end;
	uint8_t olddata=0xFF;
	uint8_t buf[256];

	if (addr > E2END) return;
	if (!initialized) eeprom_initialize();

	sector = (addr >> 2) % FLASH_SECTORS; 
	offset = (addr & 3) | (((addr >> 2) / FLASH_SECTORS) << 2);
	//printf("ee_wr, addr=%u, sector=%u, offset=%u, len=%u\n",
		//addr, sector, offset, sector_index[sector]);
	p = (uint16_t *)(FLASH_BASEADDR + sector * 4096);
	end = p + sector_index[sector];
	while (p < end) {
		uint16_t val = *p++;
		if ((val & 255) == offset) olddata = val >> 8;
	}
	if (data == olddata) return;
	if (sector_index[sector] < 2048) {
		//printf("ee_wr, writing\n");
		uint16_t newdata = offset | (data << 8);
		flash_write(end, &newdata, 2);
		sector_index[sector] = sector_index[sector] + 1;
	} else {
		//printf("ee_wr, erase then write\n");
		memset(buf, 0xFF, sizeof(buf));
		p = (uint16_t *)(FLASH_BASEADDR + sector * 4096);
		end = p + 2048;
		while (p < end) {
			uint16_t val = *p++;
			buf[val & 255] = val >> 8;
		}
		buf[offset] = data;
		p = (uint16_t *)(FLASH_BASEADDR + sector * 4096);
		flash_erase_sector(p);
		index = 0;
		for (i=0; i < 256; i++) {
			if (buf[i] != 0xFF) {
				// TODO: combining these to larger write
				// would (probably) be more efficient
				uint16_t newval = i | (buf[i] << 8);
				flash_write(p + index, &newval, 2);
				index = index + 1;
			}
		}
		sector_index[sector] = index;
	}
}

uint16_t eeprom_read_word(const uint16_t *addr)
{
	const uint8_t *p = (const uint8_t *)addr;
	return eeprom_read_byte(p) | (eeprom_read_byte(p+1) << 8);
}

uint32_t eeprom_read_dword(const uint32_t *addr)
{
	const uint8_t *p = (const uint8_t *)addr;
	return eeprom_read_byte(p) | (eeprom_read_byte(p+1) << 8)
		| (eeprom_read_byte(p+2) << 16) | (eeprom_read_byte(p+3) << 24);
}

void eeprom_read_block(void *buf, const void *addr, uint32_t len)
{
	const uint8_t *p = (const uint8_t *)addr;
	uint8_t *dest = (uint8_t *)buf;
	while (len--) {
		*dest++ = eeprom_read_byte(p++);
	}
}

int eeprom_is_ready(void)
{
	return 1;
}

void eeprom_write_word(uint16_t *addr, uint16_t value)
{
	uint8_t *p = (uint8_t *)addr;
	eeprom_write_byte(p++, value);
	eeprom_write_byte(p, value >> 8);
}

void eeprom_write_dword(uint32_t *addr, uint32_t value)
{
	uint8_t *p = (uint8_t *)addr;
	eeprom_write_byte(p++, value);
	eeprom_write_byte(p++, value >> 8);
	eeprom_write_byte(p++, value >> 16);
	eeprom_write_byte(p, value >> 24);
}

void eeprom_write_block(const void *buf, void *addr, uint32_t len)
{
	uint8_t *p = (uint8_t *)addr;
	const uint8_t *src = (const uint8_t *)buf;
	while (len--) {
		eeprom_write_byte(p++, *src++);
	}
}




static void flash_wait()
{
	FLEXSPI->LUT[60] = LUT0(CMD_SDR, PINS1, 0x05) | LUT1(READ_SDR, PINS1, 1); // 05 = read status
	FLEXSPI->LUT[61] = 0;
	uint8_t status;
	do {
		FLEXSPI->IPRXFCR = FLEXSPI_IPRXFCR_CLRIPRXF(1); // clear rx fifo
		FLEXSPI->IPCR0 = 0;
		FLEXSPI->IPCR1 = FLEXSPI_IPCR1_ISEQID(15) | FLEXSPI_IPCR1_IDATSZ(1);
		FLEXSPI->IPCMD = FLEXSPI_IPCMD_TRG(1);
		while (!(FLEXSPI->INTR & FLEXSPI_INTR_IPCMDDONE(1))) {
			asm("nop");
		}
		FLEXSPI->INTR = FLEXSPI_INTR_IPCMDDONE(1);
		status = *(uint8_t *)&FLEXSPI->RFDR[0];
	} while (status & 1);
	FLEXSPI->MCR0 |= FLEXSPI_MCR0_SWRESET(1); // purge stale data from FlexSPI's AHB FIFO
	while (FLEXSPI->MCR0 & FLEXSPI_MCR0_SWRESET(1)) ; // wait
	__enable_irq();
}

void arm_dcache_delete(void *addr, uint32_t size);

// write bytes into flash memory (which is already erased to 0xFF)
static void flash_write(void *addr, const void *data, uint32_t len)
{
	__disable_irq();
	FLEXSPI->LUTKEY = FLEXSPI_LUTKEY_KEY(0x5AF05AF0);
	FLEXSPI->LUTCR = FLEXSPI_LUTCR_UNLOCK(1);
	FLEXSPI->IPCR0 = 0;
	FLEXSPI->LUT[60] = LUT0(CMD_SDR, PINS1, 0x06); // 06 = write enable
	FLEXSPI->LUT[61] = 0;
	FLEXSPI->LUT[62] = 0;
	FLEXSPI->LUT[63] = 0;
	FLEXSPI->IPCR1 = FLEXSPI_IPCR1_ISEQID(15);
	FLEXSPI->IPCMD = FLEXSPI_IPCMD_TRG(1);
	arm_dcache_delete(addr, len); // purge old data from ARM's cache
	while (!(FLEXSPI->INTR & FLEXSPI_INTR_IPCMDDONE(1))) ; // wait
	FLEXSPI->INTR = FLEXSPI_INTR_IPCMDDONE(1);
	FLEXSPI->LUT[60] = LUT0(CMD_SDR, PINS1, 0x32) | LUT1(ADDR_SDR, PINS1, 24); // 32 = quad write
	FLEXSPI->LUT[61] = LUT0(WRITE_SDR, PINS4, 1);
	FLEXSPI->IPTXFCR = FLEXSPI_IPTXFCR_CLRIPTXF(1); // clear tx fifo
	FLEXSPI->IPCR0 = (uint32_t)addr & 0x007FFFFF;
	FLEXSPI->IPCR1 = FLEXSPI_IPCR1_ISEQID(15) | FLEXSPI_IPCR1_IDATSZ(len);
	FLEXSPI->IPCMD = FLEXSPI_IPCMD_TRG(1);
	const uint8_t *src = (const uint8_t *)data;
	uint32_t n;
	while (!((n = FLEXSPI->INTR) & FLEXSPI_INTR_IPCMDDONE(1))) {
		if (n & FLEXSPI_INTR_IPTXWE(1)) {
			uint32_t wrlen = len;
			if (wrlen > 8) wrlen = 8;
			if (wrlen > 0) {
				memcpy((void *)&FLEXSPI->TFDR[0], src, wrlen);
				src += wrlen;
				len -= wrlen;
			}
			FLEXSPI->INTR = FLEXSPI_INTR_IPTXWE(1);
		}
	}
	FLEXSPI->INTR = FLEXSPI_INTR_IPCMDDONE(1) | FLEXSPI_INTR_IPTXWE(1);
	flash_wait();
}

// erase a 4K sector
static void flash_erase_sector(void *addr)
{
	__disable_irq();
	FLEXSPI->LUTKEY = FLEXSPI_LUTKEY_KEY(0x5AF05AF0);
	FLEXSPI->LUTCR = FLEXSPI_LUTCR_UNLOCK(1);
	FLEXSPI->LUT[60] = LUT0(CMD_SDR, PINS1, 0x06); // 06 = write enable
	FLEXSPI->LUT[61] = 0;
	FLEXSPI->LUT[62] = 0;
	FLEXSPI->LUT[63] = 0;
	FLEXSPI->IPCR0 = 0;
	FLEXSPI->IPCR1 = FLEXSPI_IPCR1_ISEQID(15);
	FLEXSPI->IPCMD = FLEXSPI_IPCMD_TRG(1);
	arm_dcache_delete((void *)((uint32_t)addr & 0xFFFFF000), 4096); // purge data from cache
	while (!(FLEXSPI->INTR & FLEXSPI_INTR_IPCMDDONE(1))) ; // wait
	FLEXSPI->INTR = FLEXSPI_INTR_IPCMDDONE(1);
	FLEXSPI->LUT[60] = LUT0(CMD_SDR, PINS1, 0x20) | LUT1(ADDR_SDR, PINS1, 24); // 20 = sector erase
	FLEXSPI->IPCR0 = (uint32_t)addr & 0x007FF000;
	FLEXSPI->IPCR1 = FLEXSPI_IPCR1_ISEQID(15);
	FLEXSPI->IPCMD = FLEXSPI_IPCMD_TRG(1);
	while (!(FLEXSPI->INTR & FLEXSPI_INTR_IPCMDDONE(1))) ; // wait
	FLEXSPI->INTR = FLEXSPI_INTR_IPCMDDONE(1);
	flash_wait();
}

