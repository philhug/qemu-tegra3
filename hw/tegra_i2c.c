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
 * Tegra2 I2C controller emulation
 */

#include "sysbus.h"
#include "i2c.h"

/* #define DEBUG_I2C 1 */

#ifdef DEBUG_I2C
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_i2c: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

#define FIFO_SIZE_LOG2 5
#define FIFO_SIZE      (1<<FIFO_SIZE_LOG2)
#define FIFO_MASK      (FIFO_SIZE-1)

/* constants from Linux kernel : drivers/i2c/busses/i2c-tegra.c */
#define I2C_CNFG                                0x000
#define I2C_CNFG_DEBOUNCE_CNT_SHIFT             12
#define I2C_CNFG_PACKET_MODE_EN                 (1<<10)
#define I2C_CNFG_NEW_MASTER_FSM                 (1<<11)
#define I2C_STATUS                              0x01C
#define I2C_SL_CNFG                             0x020
#define I2C_SL_CNFG_NEWSL                       (1<<2)
#define I2C_SL_ADDR1                            0x02c
#define I2C_TX_FIFO                             0x050
#define I2C_RX_FIFO                             0x054
#define I2C_PACKET_TRANSFER_STATUS              0x058
#define I2C_FIFO_CONTROL                        0x05c
#define I2C_FIFO_CONTROL_TX_FLUSH               (1<<1)
#define I2C_FIFO_CONTROL_RX_FLUSH               (1<<0)
#define I2C_FIFO_CONTROL_TX_TRIG_SHIFT          5
#define I2C_FIFO_CONTROL_RX_TRIG_SHIFT          2
#define I2C_FIFO_STATUS                         0x060
#define I2C_FIFO_STATUS_TX_MASK                 0xF0
#define I2C_FIFO_STATUS_TX_SHIFT                4
#define I2C_FIFO_STATUS_RX_MASK                 0x0F
#define I2C_FIFO_STATUS_RX_SHIFT                0
#define I2C_INT_MASK                            0x064
#define I2C_INT_STATUS                          0x068
#define I2C_INT_PACKET_XFER_COMPLETE            (1<<7)
#define I2C_INT_ALL_PACKETS_XFER_COMPLETE       (1<<6)
#define I2C_INT_TX_FIFO_OVERFLOW                (1<<5)
#define I2C_INT_RX_FIFO_UNDERFLOW               (1<<4)
#define I2C_INT_NO_ACK                          (1<<3)
#define I2C_INT_ARBITRATION_LOST                (1<<2)
#define I2C_INT_TX_FIFO_DATA_REQ                (1<<1)
#define I2C_INT_RX_FIFO_DATA_REQ                (1<<0)
#define I2C_CLK_DIVISOR                         0x06c

#define DVC_CTRL_REG1                           0x000
#define DVC_CTRL_REG1_INTR_EN                   (1<<10)
#define DVC_CTRL_REG2                           0x004
#define DVC_CTRL_REG3                           0x008
#define DVC_CTRL_REG3_SW_PROG                   (1<<26)
#define DVC_CTRL_REG3_I2C_DONE_INTR_EN          (1<<30)
#define DVC_STATUS                              0x00c
#define DVC_STATUS_I2C_DONE_INTR                (1<<30)

#define I2C_ERR_NONE                            0x00
#define I2C_ERR_NO_ACK                          0x01
#define I2C_ERR_ARBITRATION_LOST                0x02
#define I2C_ERR_UNKNOWN_INTERRUPT               0x04

#define PACKET_HEADER0_HEADER_SIZE_SHIFT        28
#define PACKET_HEADER0_PACKET_ID_SHIFT          16
#define PACKET_HEADER0_CONT_ID_SHIFT            12
#define PACKET_HEADER0_PROTOCOL_I2C             (1<<4)

#define I2C_HEADER_HIGHSPEED_MODE               (1<<22)
#define I2C_HEADER_CONT_ON_NAK                  (1<<21)
#define I2C_HEADER_SEND_START_BYTE              (1<<20)
#define I2C_HEADER_READ                         (1<<19)
#define I2C_HEADER_10BIT_ADDR                   (1<<18)
#define I2C_HEADER_IE_ENABLE                    (1<<17)
#define I2C_HEADER_REPEAT_START                 (1<<16)
#define I2C_HEADER_MASTER_ADDR_SHIFT            12
#define I2C_HEADER_SLAVE_ADDR_SHIFT             1

typedef enum {
  I2C_HEADER0,
  I2C_HEADER1,
  I2C_HEADER_SPECIFIC,
  I2C_PAYLOAD
} tegra_i2c_fsm;

typedef struct {
    SysBusDevice busdev;
    i2c_bus *bus;
    int32_t is_dvc;
    uint32_t dvc_ctrl[3];
    uint32_t dvc_status;
    uint16_t config;
    uint8_t sl_config;
    uint8_t sl_addr1;
    uint8_t sl_addr2;
    uint32_t packet_transfer_status;
    uint8_t fifo_control;
    uint8_t int_mask;
    uint8_t int_status;
    uint16_t clk_divisor;
    uint8_t rx_fifo[FIFO_SIZE];
    int32_t rx_ptr;
    int32_t rx_len;
    uint8_t payload_size;
    uint8_t payload_transfered;
    uint32_t header;
    uint32_t header_specific;
    tegra_i2c_fsm state;
    qemu_irq irq;
} tegra_i2c_state;


static void tegra_i2c_update(tegra_i2c_state *s, uint32_t it_bit,
                             uint32_t value)
{
    uint8_t real_mask = s->int_mask |
        ((s->header_specific & I2C_HEADER_IE_ENABLE) ? 0x80 : 0);

    s->int_status = (s->int_status & ~it_bit) | (value ? it_bit : 0);
    DPRINTF("update %x/%x\n", s->int_status, real_mask);

    if (s->int_status & real_mask) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
}

static void tegra_i2c_xfer_done(tegra_i2c_state *s)
{
    i2c_end_transfer(s->bus);
    s->packet_transfer_status |= (1<<24) /* transfer complete */;
    s->state = I2C_HEADER0;
    if (s->header_specific & I2C_HEADER_IE_ENABLE) {
        tegra_i2c_update(s, I2C_INT_PACKET_XFER_COMPLETE |
                            I2C_INT_ALL_PACKETS_XFER_COMPLETE, 1);
    }
}

static void tegra_i2c_fill_rx(tegra_i2c_state *s)
{
    while ((s->payload_transfered < s->payload_size) &&
           (s->rx_len < FIFO_SIZE)) {
        s->rx_fifo[(s->rx_ptr + s->rx_len) & FIFO_MASK] = i2c_recv(s->bus);
        s->rx_len++;
        s->payload_transfered++;
    }
    tegra_i2c_update(s, I2C_INT_RX_FIFO_DATA_REQ, !!s->rx_len);

    if (s->payload_transfered == s->payload_size) {
        tegra_i2c_xfer_done(s);
    }
}

static void tegra_i2c_xfer_packet(tegra_i2c_state *s, uint32_t value)
{
    int b, ret;

    switch (s->state) {
    case I2C_HEADER0:
        /* 23->16 : PKTID 7:4 proto 1=I2c 2:0 PKtType*/
        if (((value & 0xf0) != PACKET_HEADER0_PROTOCOL_I2C) ||
            (value & 0x30000000)) {
            hw_error("tegra_i2c: Invalid protocol, we only support I2C\n");
        }
        s->header = value;
        s->packet_transfer_status = value &
            (0xff << PACKET_HEADER0_PACKET_ID_SHIFT);
        s->state = I2C_HEADER1;
        break;
    case I2C_HEADER1:
        s->payload_size = (value & 0xff) + 1;
        s->payload_transfered = 0;
        s->state = I2C_HEADER_SPECIFIC;
        break;
    case I2C_HEADER_SPECIFIC:
        s->header_specific = value;
        ret = i2c_start_transfer(s->bus,
                                 (value >> I2C_HEADER_SLAVE_ADDR_SHIFT) & 0x7f,
                                 value & I2C_HEADER_READ);
        DPRINTF("#### I2C start at %02x => %d\n",
                (value >> I2C_HEADER_SLAVE_ADDR_SHIFT) & 0x7f, ret);
        if (ret) { /* invalid address */
            tegra_i2c_update(s, I2C_INT_NO_ACK, 1);
        }
        if (value & I2C_HEADER_READ) {
            /* read requested bytes */
            tegra_i2c_fill_rx(s);
        } else {
            /* wait for bytes to send */
            s->state = I2C_PAYLOAD;
        }
        break;
    case I2C_PAYLOAD:
        while ((s->payload_transfered < s->payload_size) && (b++ < 4)) {
            i2c_send(s->bus, value & 0xff);
            value >>= 8;
            s->payload_transfered++;
            s->packet_transfer_status = (s->packet_transfer_status & ~0xfff0) |
                                        (s->payload_transfered << 4);
        }
        if (s->payload_transfered == s->payload_size) {
            tegra_i2c_xfer_done(s);
        }
        break;
    }
}

static uint32_t tegra_i2c_read(void *opaque, target_phys_addr_t offset)
{
    tegra_i2c_state *s = (tegra_i2c_state *)opaque;
    uint32_t value, shift;
    DPRINTF("READ at %x\n", offset);

    if (s->is_dvc) {
        if (offset < 0x40) {
            /* DVC specific registers */
            switch(offset) {
            case DVC_CTRL_REG1:
            case DVC_CTRL_REG2:
            case DVC_CTRL_REG3:
                return s->dvc_ctrl[offset - DVC_CTRL_REG1];
            case DVC_STATUS:
                return s->dvc_status;
            }
            return 0;
        } else {
            /* remap registers to regular I2C controller */
            offset -= (offset >= 0x60) ? 0x10 : 0x40;
        }
    }

    switch (offset) {
    case 0x00 /* I2C_CNFG */:
        return s->config;
    case 0x1c /* I2C_STATUS */:
        return i2c_bus_busy(s->bus) ? (1<<8) : 0;
    case 0x20 /* I2C_SL_CNFG */:
        return s->sl_config;
    case 0x2c /* I2C_SL_ADDR1 */:
        return s->sl_addr1;
    case 0x30 /* I2C_SL_ADDR2 */:
        return s->sl_addr2;
    case 0x50 /* I2C_TX_FIFO */:
        return 0;
    case 0x54 /* I2C_RX_FIFO */:
        if (s->rx_len == 0) {
            tegra_i2c_update(s, I2C_INT_RX_FIFO_UNDERFLOW, 1);
            return 0;
        }
        for (shift = 0, value = 0; (s->rx_len) && (shift < 32); shift += 8) {
            value |= s->rx_fifo[s->rx_ptr] << shift;
            s->rx_ptr = (s->rx_ptr + 1) & FIFO_MASK;
            s->rx_len--;
        }
        if (!s->rx_len) {
            tegra_i2c_update(s, I2C_INT_RX_FIFO_DATA_REQ, 0);
        }
        if (s->payload_transfered < s->payload_size) {
            tegra_i2c_fill_rx(s);
        }
        return value;
    case 0x58 /* I2C_PACKET_TRANSFER_STATUS */:
        return s->packet_transfer_status;
    case 0x5c /* I2C_FIFO_CONTROL */:
        return s->fifo_control;
    case 0x60 /* I2C_FIFO_STATUS */:
        return ((s->rx_len+3)/4) | (8 << 4);
    case 0x64 /* I2C_INT_MASK */:
        return s->int_mask;
    case 0x68 /* I2C_INT_STATUS */:
        return s->int_status;
    case 0x6c /* I2C_CLK_DIVISOR */:
        return s->clk_divisor;
    default:
        hw_error("tegra_i2c_read: Bad offset %x\n", (int)offset);
        return 0;
    }

    return 0;
}

static void tegra_i2c_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_i2c_state *s = (tegra_i2c_state *)opaque;
    DPRINTF("WRITE at %x <= %x\n", offset, value);

    if (s->is_dvc) {
        if (offset < 0x40) {
            /* DVC specific registers */
            switch(offset) {
            case DVC_CTRL_REG1:
            case DVC_CTRL_REG2:
            case DVC_CTRL_REG3:
                s->dvc_ctrl[offset - DVC_CTRL_REG1] = value;
                break;
            case DVC_STATUS:
                s->dvc_status = value;
                break;
            }
            return;
        } else {
            /* remap registers to regular I2C controller */
            offset -= (offset >= 0x60) ? 0x10 : 0x40;
        }
    }

    switch (offset) {
    case 0x00 /* I2C_CNFG */:
        s->config = value;
        break;
    case 0x1c /* I2C_STATUS */:
        hw_error("tegra_i2c_write: I2C_STATUS is read only\n");
        break;
    case 0x20 /* I2C_SL_CNFG */:
        s->sl_config = value & 0x7;
        break;
    case 0x2c /* I2C_SL_ADDR1 */:
        s->sl_addr1 = value;
        break;
    case 0x30 /* I2C_SL_ADDR2 */:
        s->sl_addr2 = value;
        break;
    case 0x50 /* I2C_TX_FIFO */:
        if (s->config & I2C_CNFG_PACKET_MODE_EN) {
            tegra_i2c_xfer_packet(s, value);
        } else if (s->config & (1<<9)) {
            hw_error("tegra_i2c_write: Normal mode not implemented\n");
        }
        break;
    case 0x54 /* I2C_RX_FIFO */:
        hw_error("tegra_i2c_write: I2C_RX_FIFO is read only\n");
        break;
    case 0x58 /* I2C_PACKET_TRANSFER_STATUS */:
        hw_error("tegra_i2c_write: I2C_PACKET_TRANSFER_STATUS is read only\n");
        break;
    case 0x5c /* I2C_FIFO_CONTROL */:
        if (value & I2C_FIFO_CONTROL_TX_FLUSH) {
            s->state = I2C_HEADER0;
            tegra_i2c_update(s, I2C_INT_TX_FIFO_OVERFLOW, 0);
        }
        if (value & I2C_FIFO_CONTROL_RX_FLUSH) {
            s->rx_len = 0;
            s->rx_ptr = 0;
            tegra_i2c_update(s, I2C_INT_RX_FIFO_UNDERFLOW, 0);
        }
        s->fifo_control = value & 0xfc;
        break;
    case 0x60 /* I2C_FIFO_STATUS */:
        hw_error("tegra_i2c_write: I2C_FIFO_STATUS is read only\n");
        break;
    case 0x64 /* I2C_INT_MASK */:
        s->int_mask = value & 0x7f;
        tegra_i2c_update(s, 0, 0);
        break;
    case 0x68 /* I2C_INT_STATUS */:
        s->int_status &= ~(value & 0xfc);
        tegra_i2c_update(s, 0, 0);
        break;
    case 0x6c /* I2C_CLK_DIVISOR */:
        s->clk_divisor = value;
        break;
    default:
        hw_error("tegra_i2c_write: Bad offset %x\n", (int)offset);
        break;
    }
}

static CPUReadMemoryFunc * const tegra_i2c_readfn[] = {
   tegra_i2c_read,
   tegra_i2c_read,
   tegra_i2c_read
};

static CPUWriteMemoryFunc * const tegra_i2c_writefn[] = {
   tegra_i2c_write,
   tegra_i2c_write,
   tegra_i2c_write
};

static int tegra_i2c_init(SysBusDevice *dev)
{
    int iomemtype;
    tegra_i2c_state *s = FROM_SYSBUS(tegra_i2c_state, dev);


    s->bus = i2c_init_bus(&dev->qdev, "i2c");

    iomemtype = cpu_register_io_memory(tegra_i2c_readfn,
                                       tegra_i2c_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x100, iomemtype);
    sysbus_init_irq(dev, &s->irq);

    return 0;
}

static void tegra_i2c_reset(DeviceState *d)
{
    tegra_i2c_state *s = container_of(d, tegra_i2c_state, busdev.qdev);

    s->dvc_ctrl[0] = 0;
    s->dvc_ctrl[1] = 0;
    s->dvc_ctrl[2] = 0;
    s->dvc_status = 0x60000;
    s->config = 0;
    s->sl_config = 0;
    s->sl_addr1 = 0;
    s->sl_addr2 = 0;
    s->packet_transfer_status = 0;
    s->fifo_control = 0;
    s->int_mask = 0;
    s->int_status = 0;
    s->clk_divisor = 0;
    s->rx_len = 0;
    s->rx_ptr = 0;
    s->state = I2C_HEADER0;
    s->payload_size = 0;
}

static const VMStateDescription tegra_i2c_vmstate = {
    .name = "tegra_i2c",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_INT32(is_dvc, tegra_i2c_state),
        VMSTATE_UINT32_ARRAY(dvc_ctrl, tegra_i2c_state, 3),
        VMSTATE_UINT32(dvc_status, tegra_i2c_state),
        VMSTATE_UINT16(config, tegra_i2c_state),
        VMSTATE_UINT8(sl_config, tegra_i2c_state),
        VMSTATE_UINT8(sl_addr1, tegra_i2c_state),
        VMSTATE_UINT8(sl_addr2, tegra_i2c_state),
        VMSTATE_UINT32(packet_transfer_status, tegra_i2c_state),
        VMSTATE_UINT8(fifo_control, tegra_i2c_state),
        VMSTATE_UINT8(int_mask, tegra_i2c_state),
        VMSTATE_UINT8(int_status, tegra_i2c_state),
        VMSTATE_UINT16(clk_divisor, tegra_i2c_state),
        VMSTATE_UINT8_ARRAY(rx_fifo, tegra_i2c_state, FIFO_SIZE),
        VMSTATE_INT32(rx_len, tegra_i2c_state),
        VMSTATE_INT32(rx_ptr, tegra_i2c_state),
        VMSTATE_UINT8(payload_size, tegra_i2c_state),
        VMSTATE_UINT8(payload_transfered, tegra_i2c_state),
        VMSTATE_UINT32(header, tegra_i2c_state),
        VMSTATE_UINT32(header_specific, tegra_i2c_state),
        /* TODO  VMSTATE_INT32(state, tegra_i2c_state), */
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_i2c_info = {
    .init = tegra_i2c_init,
    .qdev.name  = "tegra_i2c",
    .qdev.size  = sizeof(tegra_i2c_state),
    .qdev.vmsd  = &tegra_i2c_vmstate,
    .qdev.reset = tegra_i2c_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_INT32("is_dvc", tegra_i2c_state, is_dvc, 0),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void tegra_i2c_register(void)
{
    sysbus_register_withprop(&tegra_i2c_info);
}

device_init(tegra_i2c_register)
