/* 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for TS3A225E Audio Switch chip.
 */
#ifndef __TS3A225E_H__
#define __TS3A225E_H__

int ts3a225e_read_byte(uint8_t cmd, uint8_t *returnData);
int ts3a225e_write_byte(uint8_t cmd, uint8_t writeData);

#define TS3A_REG_CTRL1		0x02
#define TS3A_REG_CTRL2		0x03
#define TS3A_REG_CTRL3		0x04
#define TS3A_REG_DAT1		0x05
#define TS3A_REG_INT		0x06

#define TS3A_CTRL1_SET_MANUAL	0x07
#define TS3A_CTRL2_ALL_ON	0xF3
#define TS3A_CTRL3_TRIGGER	0x01

#define TS3A_INT_MIC_PRESENT	0x02
#define TS3A_INT_STD_TSR	0x01

#define TS3A_DAT1_GND_LOC	0x80

#endif
