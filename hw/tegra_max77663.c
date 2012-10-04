/*
 * Copyright 2012 Philipp Hug <philipp@hug.cx>
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
 * MAX77663 Power
 *
 */

#include "i2c.h"
#define VERBOSE

typedef struct {
    i2c_slave i2c;
    int i2c_command_byte;
    int len;

    uint8_t level;
    uint8_t direction;
    uint8_t polarity;
    uint8_t status;
    uint8_t command;
    qemu_irq handler[8];
} MAX77663State;

static void max77663_reset(DeviceState *dev)
{
    MAX77663State *s = FROM_I2C_SLAVE(MAX77663State, I2C_SLAVE_FROM_QDEV(dev));
    s->level &= s->direction;
    s->direction = 0xff;
    s->polarity = 0xf0;
    s->status = 0x01;
    s->command = 0x00;
}

static int max77663_rx(i2c_slave *i2c)
{
    MAX77663State *s = (MAX77663State *) i2c;
#ifdef VERBOSE
        printf("%s: read register %02x\n", __FUNCTION__, s->command);
#endif

    switch (s->command) {
    case 0x00:
        return 0x0;

    default:
#ifdef VERBOSE
        printf("%s: unknown register %02x\n", __FUNCTION__, s->command);
#endif
        break;
    }
    return 0xff;
}

static int max77663_tx(i2c_slave *i2c, uint8_t data)
{
    MAX77663State *s = (MAX77663State *) i2c;
    uint8_t diff;
    int line;

#ifdef VERBOSE
        printf("%s: write 0x%x\n", __FUNCTION__, data);
#endif

    if (s->i2c_command_byte) {
        s->command = data;
        s->i2c_command_byte = 0;
        return 0;
    }

    switch (s->command) {
    case 0x00:
	break;
    default:
#ifdef VERBOSE
        printf("%s: unknown register %02x\n", __FUNCTION__, s->command);
#endif
        return 1;
    }

    return 0;
}

static void max77663_event(i2c_slave *i2c, enum i2c_event event)
{
    MAX77663State *s = (MAX77663State *) i2c;
    s->len = 0;

#ifdef VERBOSE
        printf("%s: event: %d\n", __FUNCTION__, event);
#endif
    switch (event) {
    case I2C_START_SEND:
        s->i2c_command_byte = 1;
        break;
    case I2C_FINISH:
#ifdef VERBOSE
        if (s->len == 1)
            printf("%s: message too short (%i bytes)\n", __FUNCTION__, s->len);
#endif
        break;
    default:
        break;
    }
}

static const VMStateDescription vmstate_max77663 = {
    .name = "max77663",
    .version_id = 0,
    .minimum_version_id = 0,
    .minimum_version_id_old = 0,
    .fields      = (VMStateField []) {
        VMSTATE_INT32(i2c_command_byte, MAX77663State),
        VMSTATE_INT32(len, MAX77663State),
        VMSTATE_UINT8(level, MAX77663State),
        VMSTATE_UINT8(direction, MAX77663State),
        VMSTATE_UINT8(polarity, MAX77663State),
        VMSTATE_UINT8(status, MAX77663State),
        VMSTATE_UINT8(command, MAX77663State),
        VMSTATE_I2C_SLAVE(i2c, MAX77663State),
        VMSTATE_END_OF_LIST()
    }
};

/* MAX77663 is SMBus-compatible (can be used with only SMBus protocols),
 * but also accepts sequences that are not SMBus so return an I2C device.  */
static int max77663_init(i2c_slave *i2c)
{
    MAX77663State *s = FROM_I2C_SLAVE(MAX77663State, i2c);


    return 0;
}

static I2CSlaveInfo max77663_info = {
    .qdev.name = "max77663",
    .qdev.size = sizeof(MAX77663State),
    .qdev.vmsd = &vmstate_max77663,
    .qdev.reset = max77663_reset,
    .init = max77663_init,
    .event = max77663_event,
    .recv = max77663_rx,
    .send = max77663_tx
};

static void max77663_register_devices(void)
{
    i2c_register_slave(&max77663_info);
}

device_init(max77663_register_devices)
