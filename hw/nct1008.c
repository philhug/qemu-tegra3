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
 * ON Semiconductor NCT1008 thermal sensor emulation
 */

#include "i2c.h"

/* #define DEBUG_NCT 1 */

#ifdef DEBUG_NCT
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "nct1008: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

#define LOCAL_TEMP 40
#define EXT_TEMP   0x1900 /* 25 C */

#define NCT1008_MANUFACTURER_ID 0x41
#define NCT1008_DIE_REVISION    0x57

typedef struct {
    i2c_slave i2c;
    int32_t is_reg;
    uint8_t reg;

    uint8_t configuration;
    uint8_t conversion_rate;
    uint8_t local_temp_hi_limit;
    uint8_t local_temp_lo_limit;
    uint16_t ext_temp_hi_limit;
    uint16_t ext_temp_lo_limit;
    uint16_t ext_temp_offset;
    uint8_t ext_therm_limit;
    uint8_t local_therm_limit;
    uint8_t therm_hysteresis;
    uint8_t consecutive_alert;
    qemu_irq irq;
} nct1008_state;

enum {
    CMD_LOCAL_TEMP_VALUE = 0x00,
    CMD_EXT_TEMP_VALUE_HI = 0x01,
    CMD_STATUS = 0x02,
    CMD_CONFIGURATION = 0x03,
    CMD_CONVERSION_RATE = 0x04,
    CMD_LOCAL_TEMP_HI_LIMIT = 0x05,
    CMD_LOCAL_TEMP_LO_LIMIT = 0x06,
    CMD_EXT_TEMP_HI_LIMIT_HI = 0x07,
    CMD_EXT_TEMP_LO_LIMIT_HI = 0x08,
    CMD_CONFIGURATION_WR = 0x09,
    CMD_CONVERSION_RATE_WR = 0x0a,
    CMD_LOCAL_TEMP_HI_LIMIT_WR = 0x0b,
    CMD_LOCAL_TEMP_LO_LIMIT_WR = 0x0c,
    CMD_EXT_TEMP_HI_LIMIT_HI_WR = 0x0d,
    CMD_EXT_TEMP_LO_LIMIT_HI_WR = 0x0e,
    CMD_ONE_SHOT = 0x0f,
    CMD_EXT_TEMP_VALUE_LO = 0x10,
    CMD_EXT_TEMP_OFFSET_HI = 0x11,
    CMD_EXT_TEMP_OFFSET_LO = 0x12,
    CMD_EXT_TEMP_HI_LIMIT_LO = 0x13,
    CMD_EXT_TEMP_LO_LIMIT_LO = 0x14,
    CMD_EXT_THERM_LIMIT = 0x19,
    CMD_LOCAL_THERM_LIMIT = 0x20,
    CMD_THERM_HYSTERESIS = 0x21,
    CMD_CONSECUTIVE_ALERT = 0x22,
    CMD_MANUFACTURER_ID = 0xfe,
    CMD_DIE_REVISION = 0xff,
};

static uint8_t nct1008_smbus_read(nct1008_state *s, uint8_t reg)
{
    DPRINTF("READ reg 0x%02x\n", reg);

    switch (reg) {
    case CMD_LOCAL_TEMP_VALUE:
        return LOCAL_TEMP;
    case CMD_EXT_TEMP_VALUE_HI:
        return EXT_TEMP >> 8;
    case CMD_STATUS:
        return 0;
    case CMD_CONFIGURATION:
        return s->configuration;
    case CMD_CONVERSION_RATE:
        return s->conversion_rate;
    case CMD_LOCAL_TEMP_HI_LIMIT:
        return s->local_temp_hi_limit;
    case CMD_LOCAL_TEMP_LO_LIMIT:
        return s->local_temp_lo_limit;
    case CMD_EXT_TEMP_HI_LIMIT_HI:
        return s->ext_temp_hi_limit >> 8;
    case CMD_EXT_TEMP_LO_LIMIT_HI:
        return s->ext_temp_lo_limit >> 8;
    case CMD_EXT_TEMP_VALUE_LO:
        return EXT_TEMP & 0xff;
    case CMD_EXT_TEMP_OFFSET_HI:
        return s->ext_temp_offset >> 8;
    case CMD_EXT_TEMP_OFFSET_LO:
        return s->ext_temp_offset & 0xff;
    case CMD_EXT_TEMP_HI_LIMIT_LO:
        return s->ext_temp_hi_limit & 0xff;
    case CMD_EXT_TEMP_LO_LIMIT_LO:
        return s->ext_temp_lo_limit & 0xff;
    case CMD_EXT_THERM_LIMIT:
        return s->ext_therm_limit;
    case CMD_LOCAL_THERM_LIMIT:
        return s->local_therm_limit;
    case CMD_THERM_HYSTERESIS:
        return s->therm_hysteresis;
    case CMD_CONSECUTIVE_ALERT:
        return s->consecutive_alert;
    case CMD_MANUFACTURER_ID:
        return NCT1008_MANUFACTURER_ID;
    case CMD_DIE_REVISION:
        return NCT1008_DIE_REVISION;
    default:
        hw_error("nct1008_read: Bad register %x\n", reg);
    }

    return 0;
}

static void nct1008_smbus_write(nct1008_state *s, uint8_t reg,
                                uint8_t value)
{
    DPRINTF("WRITE reg 0x%02x <= %02x\n", reg, value);

    switch (reg) {
    case CMD_CONFIGURATION_WR:
        s->configuration = value;
        break;
    case CMD_CONVERSION_RATE_WR:
        s->conversion_rate = value;
        break;
    case CMD_LOCAL_TEMP_HI_LIMIT_WR:
        s->local_temp_hi_limit = value;
        break;
    case CMD_LOCAL_TEMP_LO_LIMIT_WR:
        s->local_temp_lo_limit = value;
        break;
    case CMD_EXT_TEMP_HI_LIMIT_HI_WR:
        s->ext_temp_hi_limit = ((uint16_t)value << 8) |
                               (s->ext_temp_hi_limit & 0xff);
        break;
    case CMD_EXT_TEMP_LO_LIMIT_HI_WR:
        s->ext_temp_lo_limit = ((uint16_t)value << 8) |
                               (s->ext_temp_lo_limit & 0xff);
        break;
    case CMD_ONE_SHOT:
        break;
    case CMD_EXT_TEMP_OFFSET_HI:
        s->ext_temp_offset = ((uint16_t)value << 8) |
                              (s->ext_temp_offset & 0xff);
        break;
    case CMD_EXT_TEMP_OFFSET_LO:
        s->ext_temp_offset = value | (s->ext_temp_offset & 0xff00);
        break;
    case CMD_EXT_TEMP_HI_LIMIT_LO:
        s->ext_temp_hi_limit = value | (s->ext_temp_hi_limit & 0xff00);
        break;
    case CMD_EXT_TEMP_LO_LIMIT_LO:
        s->ext_temp_lo_limit = value | (s->ext_temp_lo_limit & 0xff00);
        break;
    case CMD_EXT_THERM_LIMIT:
        s->ext_therm_limit = value;
        break;
    case CMD_LOCAL_THERM_LIMIT:
        s->local_therm_limit = value;
        break;
    case CMD_THERM_HYSTERESIS:
        s->therm_hysteresis = value;
        break;
    case CMD_CONSECUTIVE_ALERT:
        s->consecutive_alert = value;
        break;
    default:
        hw_error("nct1008_write: Bad register %x\n", reg);
    }
}

static void nct1008_i2c_event(i2c_slave *i2c, enum i2c_event event)
{
    nct1008_state *s = FROM_I2C_SLAVE(nct1008_state, i2c);

    switch (event) {
    case I2C_START_SEND:
        s->is_reg = 1;
        break;
    case I2C_START_RECV:
    case I2C_FINISH:
    case I2C_NACK:
    default:
        break;
    }
}

static int nct1008_i2c_recv(i2c_slave *i2c)
{
    nct1008_state *s = FROM_I2C_SLAVE(nct1008_state, i2c);

    return nct1008_smbus_read(s, s->reg++);
}

static int nct1008_i2c_send(i2c_slave *i2c, uint8_t data)
{
    nct1008_state *s = FROM_I2C_SLAVE(nct1008_state, i2c);

    if (s->is_reg) {
        s->reg = data;
        s->is_reg = 0;
    } else {
        nct1008_smbus_write(s, s->reg++, data);
    }

    return 0;
}

static int nct1008_init(i2c_slave *i2c)
{
    nct1008_state *s = FROM_I2C_SLAVE(nct1008_state, i2c);

    qdev_init_gpio_out(&i2c->qdev, &s->irq, 1);

    return 0;
}

static void nct1008_reset(DeviceState *d)
{
    nct1008_state *s = container_of(d, nct1008_state, i2c.qdev);

    s->is_reg = 1;
    s->reg = 0;

    s->configuration = 0x00;
    s->conversion_rate = 0x08;
    s->local_temp_hi_limit = 0x55;
    s->local_temp_lo_limit = 0x00;
    s->ext_temp_hi_limit = 0x5500;
    s->ext_temp_lo_limit = 0x0000;
    s->ext_temp_offset = 0x0000;
    s->ext_therm_limit = 0x6C;
    s->local_therm_limit = 0x55;
    s->therm_hysteresis = 0x0A;
    s->consecutive_alert = 0x01;
}

static const VMStateDescription nct1008_vmstate = {
    .name = "nct1008",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_I2C_SLAVE(i2c, nct1008_state),
        VMSTATE_INT32(is_reg, nct1008_state),
        VMSTATE_UINT8(reg, nct1008_state),
        VMSTATE_UINT8(configuration, nct1008_state),
        VMSTATE_UINT8(conversion_rate, nct1008_state),
        VMSTATE_UINT8(local_temp_hi_limit, nct1008_state),
        VMSTATE_UINT8(local_temp_lo_limit, nct1008_state),
        VMSTATE_UINT16(ext_temp_hi_limit, nct1008_state),
        VMSTATE_UINT16(ext_temp_lo_limit, nct1008_state),
        VMSTATE_UINT16(ext_temp_offset, nct1008_state),
        VMSTATE_UINT8(ext_therm_limit, nct1008_state),
        VMSTATE_UINT8(local_therm_limit, nct1008_state),
        VMSTATE_UINT8(therm_hysteresis, nct1008_state),
        VMSTATE_UINT8(consecutive_alert, nct1008_state),
        VMSTATE_END_OF_LIST()
    }
};

static I2CSlaveInfo nct1008_info = {
    .init = nct1008_init,
    .event = nct1008_i2c_event,
    .recv = nct1008_i2c_recv,
    .send = nct1008_i2c_send,
    .qdev.name  = "nct1008",
    .qdev.size  = sizeof(nct1008_state),
    .qdev.vmsd  = &nct1008_vmstate,
    .qdev.reset = nct1008_reset,
};

static void nct1008_register(void)
{
    i2c_register_slave(&nct1008_info);
}

device_init(nct1008_register)
