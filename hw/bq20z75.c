/*
 * Copyright 2011 Google Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 *
 * TI BQ20Z75 gas gauge emulation (SBS 1.1 compliant)
 */

#include "i2c.h"

/* #define DEBUG_BQ 1 */

#ifdef DEBUG_BQ
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "bq20z75: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

#define MAX_PAYLOAD_SIZE 32

#define REGISTER_COUNT 128

typedef struct {
    i2c_slave i2c;
    int32_t is_write;
    uint8_t payload_buffer[MAX_PAYLOAD_SIZE];
    int32_t payload_index;
    uint8_t reg;
    qemu_irq irq;
} bq20z75_state;

enum {
    CMD_MANUFACTURER_ACCESS = 0x00,
    CMD_REMAINING_CAPACITY_ALARM = 0x01,
    CMD_REMAINING_TIME_ALARM = 0x02,
    CMD_BATTERY_MODE = 0x03,
    CMD_AT_RATE = 0x04,
    CMD_AT_RATE_TIME_TO_FULL = 0x05,
    CMD_AT_RATE_TIME_TO_EMPTY = 0x06,
    CMD_AT_RATE_OK = 0x07,
    CMD_TEMPERATURE = 0x08,
    CMD_VOLTAGE = 0x09,
    CMD_CURRENT = 0x0a,
    CMD_AVERAGE_CURRENT = 0x0b,
    CMD_MAX_ERROR = 0x0c,
    CMD_RELATIVE_STATE_OF_CHARGE = 0x0d,
    CMD_ABSOLUTE_STATE_OF_CHARGE = 0x0e,
    CMD_REMAINING_CAPACITY = 0x0f,
    CMD_FULL_CHARGE_CAPACITY = 0x10,
    CMD_RUN_TIME_TO_EMPTY = 0x11,
    CMD_AVERAGE_TIME_TO_EMPTY = 0x12,
    CMD_AVERAGE_TIME_TO_FULL = 0x13,
    CMD_CHARGING_CURRENT = 0x14,
    CMD_CHARGING_VOLTAGE = 0x15,
    CMD_BATTERY_STATUS = 0x16,
    CMD_CYCLE_COUNT = 0x17,
    CMD_DESIGN_CAPACITY = 0x18,
    CMD_DESIGN_VOLTAGE = 0x19,
    CMD_SPECIFICATION_INFO = 0x1a,
    CMD_MANUFACTURE_DATE = 0x1b,
    CMD_SERIAL_NUMBER = 0x1c,
    CMD_MANUFACTURER_NAME = 0x20,
    CMD_DEVICE_NAME = 0x21,
    CMD_DEVICE_CHEMISTRY = 0x22,
    CMD_MANUFACTURER_DATA = 0x23,
    CMD_AUTHENTICATE = 0x2f,
    CMD_CELL_VOLTAGE4 = 0x3c,
    CMD_CELL_VOLTAGE3 = 0x3d,
    CMD_CELL_VOLTAGE2 = 0x3e,
    CMD_CELL_VOLTAGE1 = 0x3f,
    CMD_AFEDATA = 0x45,
    CMD_FETCONTROL = 0x46,
    CMD_STATE_OF_HEALTH = 0x4f,
    CMD_SAFETY_STATUS = 0x51,
    CMD_PFSTATUS = 0x53,
    CMD_OPERATION_STATUS = 0x54,
    CMD_CHARGING_STATUS = 0x55,
    CMD_RESET_DATA = 0x57,
    CMD_PACK_VOLTAGE = 0x5a,
    CMD_AVERAGE_VOLTAGE = 0x5d,
    CMD_UN_SEAL_KEY = 0x60,
    CMD_FULL_ACCESS_KEY = 0x61,
    CMD_PFKEY = 0x62,
    CMD_AUTHEN_KEY3 = 0x63,
    CMD_AUTHEN_KEY2 = 0x64,
    CMD_AUTHEN_KEY1 = 0x65,
    CMD_AUTHEN_KEY0 = 0x66,
    CMD_MANUFACTURER_INFO = 0x70,
    CMD_SENSE_RESISTOR = 0x71,
    CMD_DATA_FLASH_SUB_CLASS_ID = 0x77,
    CMD_DATA_FLASH_SUB_CLASS_PAGE1 = 0x78,
    CMD_DATA_FLASH_SUB_CLASS_PAGE2 = 0x79,
    CMD_DATA_FLASH_SUB_CLASS_PAGE3 = 0x7a,
    CMD_DATA_FLASH_SUB_CLASS_PAGE4 = 0x7b,
    CMD_DATA_FLASH_SUB_CLASS_PAGE5 = 0x7c,
    CMD_DATA_FLASH_SUB_CLASS_PAGE6 = 0x7d,
    CMD_DATA_FLASH_SUB_CLASS_PAGE7 = 0x7e,
    CMD_DATA_FLASH_SUB_CLASS_PAGE8 = 0x7f,
};

#define RO_U16(reg, def) [CMD_##reg] = {0, 2, (char *)(def)}
#define RW_U16(reg, def) [CMD_##reg] = {1, 2, (char *)(def)}
#define RW_U32(reg, def) [CMD_##reg] = {1, 4, (char *)(def)}
#define RO_STR(reg, def) [CMD_##reg] = {0, sizeof(def), def}
#define RW_STR(reg, def) [CMD_##reg] = {1, sizeof(def), def}

static struct {
    int read_write;
    int size;
    const char *def_value;
} reg_props[REGISTER_COUNT] = {
    RW_U16(MANUFACTURER_ACCESS, 0),
    RW_U16(REMAINING_CAPACITY_ALARM, 280 /* x10 mWh */),
    RW_U16(REMAINING_TIME_ALARM, 10 /* min */),
    RW_U16(BATTERY_MODE, 0x8000),
    RW_U16(AT_RATE, 0 /* x10 mW */),
    RO_U16(AT_RATE_TIME_TO_FULL, 0xffff),
    RO_U16(AT_RATE_TIME_TO_EMPTY, 0xffff),
    RO_U16(AT_RATE_OK, 1),
    RO_U16(TEMPERATURE, 2981 /* x0.1°K = 25 C */),
    RO_U16(VOLTAGE, 14400 /* mV */),
    RO_U16(CURRENT, 500 /* mA */),
    RO_U16(AVERAGE_CURRENT, 500 /* mA */),
    RO_U16(MAX_ERROR, 10 /* % */),
    RO_U16(RELATIVE_STATE_OF_CHARGE, 71 /* % */),
    RO_U16(ABSOLUTE_STATE_OF_CHARGE, 71 /* % */),
    RW_U16(REMAINING_CAPACITY, 2000 /* x10 mWh */),
    RO_U16(FULL_CHARGE_CAPACITY, 2800 /* x10 mWh */),
    RO_U16(RUN_TIME_TO_EMPTY, 300 /* min */),
    RO_U16(AVERAGE_TIME_TO_EMPTY, 300 /* min */),
    RO_U16(AVERAGE_TIME_TO_FULL, 60 /* min */),
    RO_U16(CHARGING_CURRENT, 0 /* mA */),
    RO_U16(CHARGING_VOLTAGE, 0 /* mV */),
    RO_U16(BATTERY_STATUS, 0x00c0 /* initialized and discharging */),
    RW_U16(CYCLE_COUNT, 10),
    RW_U16(DESIGN_CAPACITY, 2800 /* x10 mWh */),
    RW_U16(DESIGN_VOLTAGE, 14400 /* mV */),
    RW_U16(SPECIFICATION_INFO, 0x0031),
    RW_U16(MANUFACTURE_DATE, (2011-1980)*512 + 3*32 + 9 /* 2011-03-09 */),
    RW_U16(SERIAL_NUMBER, 0x1234),
    RW_STR(MANUFACTURER_NAME, "Texas Instr"),
    RW_STR(DEVICE_NAME, "bq20z75"),
    RW_STR(DEVICE_CHEMISTRY, "LION"),
    RO_STR(MANUFACTURER_DATA, "ManufacturData"),
    RW_STR(AUTHENTICATE, "0SecretSecretSecret0"),
    RO_U16(CELL_VOLTAGE4, 3600 /* mV */),
    RO_U16(CELL_VOLTAGE3, 3600 /* mV */),
    RO_U16(CELL_VOLTAGE2, 3600 /* mV */),
    RO_U16(CELL_VOLTAGE1, 3600 /* mV */),
    RO_STR(AFEDATA, "??AFEdata??"),
    RW_U16(FETCONTROL, 0),
    RO_U16(STATE_OF_HEALTH, 0),
    RO_U16(SAFETY_STATUS, 71 /* % */),
    RO_U16(PFSTATUS, 0),
    RO_U16(OPERATION_STATUS, 0),
    RO_U16(CHARGING_STATUS, 0),
    RO_U16(RESET_DATA, 0),
    RO_U16(PACK_VOLTAGE, 14400 /* mV */),
    RO_U16(AVERAGE_VOLTAGE, 14400 /* mV */),
    RW_U32(UN_SEAL_KEY, 0x00000000),
    RW_U32(FULL_ACCESS_KEY, 0x00000000),
    RW_U32(PFKEY, 0x00000000),
    RW_U32(AUTHEN_KEY3, 0x00000000),
    RW_U32(AUTHEN_KEY2, 0x00000000),
    RW_U32(AUTHEN_KEY1, 0x00000000),
    RW_U32(AUTHEN_KEY0, 0x00000000),
    RW_STR(MANUFACTURER_INFO, "-EmulatedDevice-EmulatedDevice-"),
    RW_U16(SENSE_RESISTOR, 10000/* uOhm */),
    RW_U16(DATA_FLASH_SUB_CLASS_ID, 0),
    RW_STR(DATA_FLASH_SUB_CLASS_PAGE1, "0123456789012345678901234567890"),
    RW_STR(DATA_FLASH_SUB_CLASS_PAGE2, "0123456789012345678901234567890"),
    RW_STR(DATA_FLASH_SUB_CLASS_PAGE3, "0123456789012345678901234567890"),
    RW_STR(DATA_FLASH_SUB_CLASS_PAGE4, "0123456789012345678901234567890"),
    RW_STR(DATA_FLASH_SUB_CLASS_PAGE5, "0123456789012345678901234567890"),
    RW_STR(DATA_FLASH_SUB_CLASS_PAGE6, "0123456789012345678901234567890"),
    RW_STR(DATA_FLASH_SUB_CLASS_PAGE7, "0123456789012345678901234567890"),
    RW_STR(DATA_FLASH_SUB_CLASS_PAGE8, "0123456789012345678901234567890"),
};

static void bq20z75_smbus_read_buf(bq20z75_state *s, uint8_t reg,
                                       uint8_t *buffer, int size)
{
    DPRINTF("READ reg 0x%02x (size=%d)\n", reg, size);

    memcpy(s->payload_buffer, reg_props[reg].def_value, reg_props[reg].size);
}

static void bq20z75_smbus_write_buf(bq20z75_state *s, uint8_t reg,
                                    uint8_t *buffer, int size)
{
    DPRINTF("WRITE at %x <= %02x %02x %d\n", reg, buffer[0], buffer[1], size);

    /* TODO: implement write */
}

static uint16_t bq20z75_smbus_read_short(bq20z75_state *s, uint8_t reg)
{
    DPRINTF("READ16 reg 0x%02x\n", reg);

    return (unsigned long)reg_props[reg].def_value;
}

static void bq20z75_smbus_write_short(bq20z75_state *s, uint8_t reg,
                                      uint16_t value)
{
    DPRINTF("WRITE16 reg 0x%02x <= %04x\n", reg, value);

    /* TODO: implement write */
}

static void bq20z75_i2c_event(i2c_slave *i2c, enum i2c_event event)
{
    bq20z75_state *s = FROM_I2C_SLAVE(bq20z75_state, i2c);
    uint16_t val;

    switch (event) {
    case I2C_START_RECV:
        s->is_write = 0;
        s->payload_index = 0;
        if (s->reg > 0x7f) {
            DPRINTF("Invalid register read at %02x\n", s->reg);
            break;
        }
        if (reg_props[s->reg].size <= 2) {
            val = bq20z75_smbus_read_short(s, s->reg);
            s->payload_buffer[0] = val & 0xff;
            s->payload_buffer[1] = (val >> 8) & 0xff;
        } else {
            bq20z75_smbus_read_buf(s, s->reg, s->payload_buffer,
                                   reg_props[s->reg].size);
        }
        break;
    case I2C_START_SEND:
        s->is_write = 1;
        s->payload_index = -1;
        break;
    case I2C_FINISH:
        if (s->is_write && (s->payload_index > 0)) {
            if ((s->reg > 0x7f) || !reg_props[s->reg].read_write) {
                DPRINTF("Invalid write at register %02x\n", s->reg);
                break;
            }
            if (reg_props[s->reg].size <= 2) {
                val = s->payload_buffer[0] | (s->payload_buffer[1] << 8);
                bq20z75_smbus_write_short(s, s->reg, val);
            } else {
                bq20z75_smbus_write_buf(s, s->reg, s->payload_buffer,
                                        reg_props[s->reg].size);
            }
        }
        break;
    case I2C_NACK:
    default:
        break;
    }
}

static int bq20z75_i2c_recv(i2c_slave *i2c)
{
    bq20z75_state *s = FROM_I2C_SLAVE(bq20z75_state, i2c);

    if ((s->payload_index < 0) || (s->payload_index >= MAX_PAYLOAD_SIZE)) {
        DPRINTF("invalid I2C read\n");
        return 0;
    }

    return s->payload_buffer[s->payload_index++];
}

static int bq20z75_i2c_send(i2c_slave *i2c, uint8_t data)
{
    bq20z75_state *s = FROM_I2C_SLAVE(bq20z75_state, i2c);

    if (s->payload_index < 0) {
        s->reg = data;
        s->payload_index = 0;
    } else if (s->payload_index < MAX_PAYLOAD_SIZE) {
        s->payload_buffer[s->payload_index++] = data;
    } else {
        DPRINTF("invalid I2C write\n");
    }

    return 0;
}

static int bq20z75_init(i2c_slave *i2c)
{
    bq20z75_state *s = FROM_I2C_SLAVE(bq20z75_state, i2c);

    qdev_init_gpio_out(&i2c->qdev, &s->irq, 1);

    return 0;
}

static void bq20z75_reset(DeviceState *d)
{
    bq20z75_state *s = container_of(d, bq20z75_state, i2c.qdev);

    s->is_write = 0;
    s->payload_index = -1;
    s->reg = 0;
    memset(s->payload_buffer, 0, MAX_PAYLOAD_SIZE);
}

static const VMStateDescription bq20z75_vmstate = {
    .name = "bq20z75",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_I2C_SLAVE(i2c, bq20z75_state),
        VMSTATE_INT32(is_write, bq20z75_state),
        VMSTATE_UINT8_ARRAY(payload_buffer, bq20z75_state, MAX_PAYLOAD_SIZE),
        VMSTATE_INT32(payload_index, bq20z75_state),
        VMSTATE_UINT8(reg, bq20z75_state),
        VMSTATE_END_OF_LIST()
    }
};

static I2CSlaveInfo bq20z75_info = {
    .init = bq20z75_init,
    .event = bq20z75_i2c_event,
    .recv = bq20z75_i2c_recv,
    .send = bq20z75_i2c_send,
    .qdev.name  = "bq20z75",
    .qdev.size  = sizeof(bq20z75_state),
    .qdev.vmsd  = &bq20z75_vmstate,
    .qdev.reset = bq20z75_reset,
};

static void bq20z75_register(void)
{
    i2c_register_slave(&bq20z75_info);
}

device_init(bq20z75_register)
