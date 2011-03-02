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
 *
 * SDHCI (SD Host Controler Interface) emulation
 */

#include "sysbus.h"
#include "sd.h"

/*#define DEBUG_SDHCI 1*/

#ifdef DEBUG_SDHCI
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "sdhci: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

/* from Linux sources : drivers/mmc/host/sdhci.h */
/*
 * Controller registers
 */

#define SDHCI_DMA_ADDRESS       0x00

#define SDHCI_BLOCK_SIZE        0x04
#define  SDHCI_MAKE_BLKSZ(dma, blksz) (((dma & 0x7) << 12) | (blksz & 0xFFF))

#define SDHCI_BLOCK_COUNT       0x06

#define SDHCI_ARGUMENT          0x08

#define SDHCI_TRANSFER_MODE     0x0C
#define  SDHCI_TRNS_DMA         0x01
#define  SDHCI_TRNS_BLK_CNT_EN  0x02
#define  SDHCI_TRNS_ACMD12      0x04
#define  SDHCI_TRNS_READ        0x10
#define  SDHCI_TRNS_MULTI       0x20

#define SDHCI_COMMAND           0x0E
#define  SDHCI_CMD_RESP_MASK    0x03
#define  SDHCI_CMD_CRC          0x08
#define  SDHCI_CMD_INDEX        0x10
#define  SDHCI_CMD_DATA         0x20

#define  SDHCI_CMD_RESP_NONE    0x00
#define  SDHCI_CMD_RESP_LONG    0x01
#define  SDHCI_CMD_RESP_SHORT   0x02
#define  SDHCI_CMD_RESP_SHORT_BUSY 0x03

#define SDHCI_MAKE_CMD(c, f) (((c & 0xff) << 8) | (f & 0xff))

#define SDHCI_RESPONSE          0x10

#define SDHCI_BUFFER            0x20

#define SDHCI_PRESENT_STATE     0x24
#define  SDHCI_CMD_INHIBIT      0x00000001
#define  SDHCI_DATA_INHIBIT     0x00000002
#define  SDHCI_DOING_WRITE      0x00000100
#define  SDHCI_DOING_READ       0x00000200
#define  SDHCI_SPACE_AVAILABLE  0x00000400
#define  SDHCI_DATA_AVAILABLE   0x00000800
#define  SDHCI_CARD_PRESENT     0x00010000
#define  SDHCI_WRITE_PROTECT    0x00080000

#define SDHCI_HOST_CONTROL      0x28
#define  SDHCI_CTRL_LED         0x01
#define  SDHCI_CTRL_4BITBUS     0x02
#define  SDHCI_CTRL_HISPD       0x04
#define  SDHCI_CTRL_DMA_MASK    0x18
#define   SDHCI_CTRL_SDMA       0x00
#define   SDHCI_CTRL_ADMA1      0x08
#define   SDHCI_CTRL_ADMA32     0x10
#define   SDHCI_CTRL_ADMA64     0x18
#define   SDHCI_CTRL_8BITBUS    0x20

#define SDHCI_POWER_CONTROL     0x29
#define  SDHCI_POWER_ON         0x01
#define  SDHCI_POWER_180        0x0A
#define  SDHCI_POWER_300        0x0C
#define  SDHCI_POWER_330        0x0E

#define SDHCI_BLOCK_GAP_CONTROL 0x2A

#define SDHCI_WAKE_UP_CONTROL   0x2B
#define  SDHCI_WAKE_ON_INT      0x01
#define  SDHCI_WAKE_ON_INSERT   0x02
#define  SDHCI_WAKE_ON_REMOVE   0x04

#define SDHCI_CLOCK_CONTROL     0x2C
#define  SDHCI_DIVIDER_SHIFT    8
#define  SDHCI_DIVIDER_HI_SHIFT 6
#define  SDHCI_DIV_MASK 0xFF
#define  SDHCI_DIV_MASK_LEN     8
#define  SDHCI_DIV_HI_MASK      0x300
#define  SDHCI_CLOCK_CARD_EN    0x0004
#define  SDHCI_CLOCK_INT_STABLE 0x0002
#define  SDHCI_CLOCK_INT_EN     0x0001

#define SDHCI_TIMEOUT_CONTROL   0x2E

#define SDHCI_SOFTWARE_RESET    0x2F
#define  SDHCI_RESET_ALL        0x01
#define  SDHCI_RESET_CMD        0x02
#define  SDHCI_RESET_DATA       0x04

#define SDHCI_INT_STATUS        0x30
#define SDHCI_INT_ENABLE        0x34
#define SDHCI_SIGNAL_ENABLE     0x38
#define  SDHCI_INT_RESPONSE     0x00000001
#define  SDHCI_INT_DATA_END     0x00000002
#define  SDHCI_INT_DMA_END      0x00000008
#define  SDHCI_INT_SPACE_AVAIL  0x00000010
#define  SDHCI_INT_DATA_AVAIL   0x00000020
#define  SDHCI_INT_CARD_INSERT  0x00000040
#define  SDHCI_INT_CARD_REMOVE  0x00000080
#define  SDHCI_INT_CARD_INT     0x00000100
#define  SDHCI_INT_ERROR        0x00008000
#define  SDHCI_INT_TIMEOUT      0x00010000
#define  SDHCI_INT_CRC          0x00020000
#define  SDHCI_INT_END_BIT      0x00040000
#define  SDHCI_INT_INDEX        0x00080000
#define  SDHCI_INT_DATA_TIMEOUT 0x00100000
#define  SDHCI_INT_DATA_CRC     0x00200000
#define  SDHCI_INT_DATA_END_BIT 0x00400000
#define  SDHCI_INT_BUS_POWER    0x00800000
#define  SDHCI_INT_ACMD12ERR    0x01000000
#define  SDHCI_INT_ADMA_ERROR   0x02000000

#define  SDHCI_INT_NORMAL_MASK  0x00007FFF
#define  SDHCI_INT_ERROR_MASK   0xFFFF8000

#define  SDHCI_INT_CMD_MASK     (SDHCI_INT_RESPONSE | SDHCI_INT_TIMEOUT | \
                SDHCI_INT_CRC | SDHCI_INT_END_BIT | SDHCI_INT_INDEX)
#define  SDHCI_INT_DATA_MASK    (SDHCI_INT_DATA_END | SDHCI_INT_DMA_END | \
                SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL | \
                SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC | \
                SDHCI_INT_DATA_END_BIT | SDHCI_INT_ADMA_ERROR)
#define SDHCI_INT_ALL_MASK      ((unsigned int)-1)

#define SDHCI_ACMD12_ERR        0x3C

/* 3E-3F reserved */

#define SDHCI_CAPABILITIES      0x40
#define  SDHCI_TIMEOUT_CLK_MASK 0x0000003F
#define  SDHCI_TIMEOUT_CLK_SHIFT 0
#define  SDHCI_TIMEOUT_CLK_UNIT 0x00000080
#define  SDHCI_CLOCK_BASE_MASK  0x00003F00
#define  SDHCI_CLOCK_V3_BASE_MASK       0x0000FF00
#define  SDHCI_CLOCK_BASE_SHIFT 8
#define  SDHCI_MAX_BLOCK_MASK   0x00030000
#define  SDHCI_MAX_BLOCK_SHIFT  16
#define  SDHCI_CAN_DO_8BIT      0x00040000
#define  SDHCI_CAN_DO_ADMA2     0x00080000
#define  SDHCI_CAN_DO_ADMA1     0x00100000
#define  SDHCI_CAN_DO_HISPD     0x00200000
#define  SDHCI_CAN_DO_SDMA      0x00400000
#define  SDHCI_CAN_VDD_330      0x01000000
#define  SDHCI_CAN_VDD_300      0x02000000
#define  SDHCI_CAN_VDD_180      0x04000000
#define  SDHCI_CAN_64BIT        0x10000000

#define SDHCI_CAPABILITIES_1    0x44

/* 44-47 reserved for more caps */

#define SDHCI_MAX_CURRENT       0x48

/* 4C-4F reserved for more max current */

#define SDHCI_SET_ACMD12_ERROR  0x50
#define SDHCI_SET_INT_ERROR     0x52

#define SDHCI_ADMA_ERROR        0x54

/* 55-57 reserved */

#define SDHCI_ADMA_ADDRESS      0x58

/* 60-FB reserved */

#define SDHCI_SLOT_INT_STATUS   0xFC

#define SDHCI_HOST_VERSION      0xFE
#define  SDHCI_VENDOR_VER_MASK  0xFF00
#define  SDHCI_VENDOR_VER_SHIFT 8
#define  SDHCI_SPEC_VER_MASK    0x00FF
#define  SDHCI_SPEC_VER_SHIFT   0
#define   SDHCI_SPEC_100        0
#define   SDHCI_SPEC_200        1
#define   SDHCI_SPEC_300        2

/* ADMA descriptor flags */
#define ADMA_VALID   (1<<0)
#define ADMA_END     (1<<1)
#define ADMA_INT     (1<<2)

#define ADMA_OP_MASK (3<<4)
#define ADMA_OP_NOP  (0<<4)
#define ADMA_OP_RSV  (1<<4)
#define ADMA_OP_TRAN (2<<4)
#define ADMA_OP_LINK (3<<4)

typedef struct {
    SysBusDevice busdev;
    BlockDriverState *bs;
    uint32_t sdma_address;
    uint16_t block_size;
    uint16_t block_count;
    uint32_t arg;
    uint16_t transfer_mode;
    uint32_t clock;
    uint32_t host_control;
    uint32_t int_enable;
    uint32_t int_status;
    uint32_t adma_address;
    uint8_t response[16];
    SDState *sd;
    qemu_irq irq;
} sdhci_state;

static void sdhci_set_irq(sdhci_state *s)
{
    DPRINTF("SDHCI status %08x/%08x => %x\n", s->int_status, s->int_enable,
            (s->int_status & s->int_enable));
    qemu_set_irq(s->irq, (s->int_status & s->int_enable));
}

static void sdhci_dma_transfer(sdhci_state *s)
{
    int b;
    struct adma_desc {
        uint16_t flags;
        uint16_t size;
        uint32_t addr;
    } desc;

    if (s->host_control & SDHCI_CTRL_ADMA32) {
        cpu_physical_memory_read(s->adma_address, (void *)&desc, sizeof(desc));
        /* assert((desc.size >> 16) % 512 == 0) */
    } else { /* use SDMA */
        /* Generate hardcoded descriptor to emulate ADMA */
        desc.addr = s->sdma_address;
        desc.size = 1<<(12+(s->block_size>>12));
        desc.flags = ADMA_VALID | ADMA_END | ADMA_OP_TRAN;
    }

    s->int_status |= SDHCI_INT_DATA_END;

    /* only support block aligned transfer : TODO upgrade to ADMA2 */
    for (; s->block_count > 0; s->block_count--) {
        while (!(desc.flags & ADMA_VALID) ||
                (desc.size < (s->block_size & 0xfff)) ||
                ((desc.flags & ADMA_OP_MASK) != ADMA_OP_TRAN)) {
            if ((desc.flags & ADMA_END) || !(desc.flags & ADMA_VALID)) {
                DPRINTF("Abort ADMA transfer\n");
                s->int_status |= SDHCI_INT_ADMA_ERROR;
                return;
            }
            if ((desc.flags & ADMA_OP_MASK) == ADMA_OP_LINK) {
                s->adma_address = desc.addr;
            } else {
                s->adma_address += sizeof(struct adma_desc);
            }
            cpu_physical_memory_read(s->adma_address, (void *)&desc,
                                     sizeof(desc));
            /* TODO check IT flag */
        }

        for (b = 0; b < (s->block_size & 0xfff); b++, desc.addr++) {
            if (s->transfer_mode & SDHCI_TRNS_READ) {
                uint8_t data = sd_read_data(s->sd);
                cpu_physical_memory_write(desc.addr, &data, 1);
            } else {
                uint8_t data;
                cpu_physical_memory_read(desc.addr, &data, 1);
                sd_write_data(s->sd, data);
            }
        }
        desc.size -= s->block_size & 0xfff;
    }
}

static void sdhci_command(sdhci_state *s, uint32_t cmd)
{
    SDRequest request;
    int len;

    if (!s->sd) { /* nothing beyond the controller */
        s->int_status |= SDHCI_INT_TIMEOUT;
        sdhci_set_irq(s);
        return;
    }

    request.cmd = (cmd>>8) & 0xff;
    request.arg = s->arg;
    len = sd_do_command(s->sd, &request, s->response);
    DPRINTF("SDHCI command %d 0x%x => %d (%08x)\n",
            request.cmd, request.arg, len, cmd);
    if (len == 0) {
        if (/*(cmd & SDHCI_CMD_INDEX) &&*/ (cmd & SDHCI_CMD_RESP_MASK)) {
            /* no response expected */
            s->int_status |= SDHCI_INT_INDEX;
        } else {
            /* error */
            s->int_status |= SDHCI_INT_RESPONSE;
        }
    } else {
        s->int_status |= SDHCI_INT_RESPONSE;
        if ((cmd & 3) == SDHCI_CMD_RESP_SHORT_BUSY) {
            /* the command will trigger the busy (DAT[0]) line ON then OFF
             * this will raise the Data END interrupt when done.
             */
            s->int_status |= SDHCI_INT_DATA_END;
        }
        if ((s->transfer_mode & SDHCI_TRNS_DMA) && (cmd & SDHCI_CMD_DATA)) {
            sdhci_dma_transfer(s);
        }
    }
    sdhci_set_irq(s);
}

static uint32_t sdhci_read(void *opaque, target_phys_addr_t offset, int size)
{
    sdhci_state *s = (sdhci_state *)opaque;

    switch (offset) {
    case SDHCI_DMA_ADDRESS:
        return s->sdma_address;
    case SDHCI_BLOCK_SIZE:
        return s->block_size;
    case SDHCI_BLOCK_COUNT:
        return s->block_count;
    case SDHCI_ARGUMENT:
        return s->arg;
    case SDHCI_TRANSFER_MODE:
        return s->transfer_mode;
    case SDHCI_COMMAND:
    case SDHCI_RESPONSE:
        return (s->response[0]<<24) | (s->response[1]<<16) |
               (s->response[2]<<8)  | s->response[3];
    case SDHCI_RESPONSE+3:
        return s->response[0];
    case SDHCI_RESPONSE+4:
        return (s->response[4]<<24) | (s->response[5]<<16) |
               (s->response[6]<<8) | s->response[7];
    case SDHCI_RESPONSE+7:
        return s->response[4];
    case SDHCI_RESPONSE+8:
        return (s->response[8]<<24) | (s->response[9]<<16) |
               (s->response[10]<<8) | s->response[11];
    case SDHCI_RESPONSE+11:
        return s->response[8];
    case SDHCI_RESPONSE+12:
        return (s->response[12]<<24) | (s->response[13]<<16) |
               (s->response[14]<<8) | s->response[15];
    case SDHCI_BUFFER:
    case SDHCI_PRESENT_STATE:
        return s->sd ? 0x00070000 : 0;
    case SDHCI_HOST_CONTROL:
        return s->host_control;
    case SDHCI_POWER_CONTROL:
    case SDHCI_BLOCK_GAP_CONTROL:
    case SDHCI_WAKE_UP_CONTROL:
        return 0;
    case SDHCI_CLOCK_CONTROL:
        return s->clock;
    case SDHCI_TIMEOUT_CONTROL:
    case SDHCI_SOFTWARE_RESET:
    case SDHCI_INT_STATUS:
        return s->int_status;
    case SDHCI_INT_ENABLE:
        return s->int_enable;
    case SDHCI_SIGNAL_ENABLE:
    case SDHCI_ACMD12_ERR:
    case SDHCI_CAPABILITIES:
        return 0x61fe30b0;
    case SDHCI_CAPABILITIES_1:
        return 0;
    case SDHCI_MAX_CURRENT:
    case SDHCI_SET_ACMD12_ERROR:
    case SDHCI_SET_INT_ERROR:
    case SDHCI_ADMA_ERROR:
    case SDHCI_ADMA_ADDRESS:
        return s->adma_address;
    case SDHCI_SLOT_INT_STATUS:
    case SDHCI_HOST_VERSION:
        return 0;
    default:
        hw_error("sdhci_read: Bad offset %x\n", (int)offset);
    }

    return 0;
}

static void sdhci_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value, int size)
{
    sdhci_state *s = (sdhci_state *)opaque;

    switch (offset) {
    case SDHCI_DMA_ADDRESS:
        s->sdma_address = value;
        break;
    case SDHCI_BLOCK_SIZE:
        s->block_size = value & 0x7fff;
        /* Hack for 32bits commands */
        if (size < 4) {
            break;
        }
        value >>= 16;
        /* fall-through */
    case SDHCI_BLOCK_COUNT:
        s->block_count = value;
        break;
    case SDHCI_ARGUMENT:
        s->arg = value;
        break;
    case SDHCI_TRANSFER_MODE:
        s->transfer_mode = value & 0x3f;
        /* Hack for 32bits commands */
        if (size < 4) {
            break;
        }
        value >>= 16;
        /* fall-through */
    case SDHCI_COMMAND:
        sdhci_command(s, value);
        break;
    case SDHCI_RESPONSE:
        break;
    case SDHCI_BUFFER:
        break;
    case SDHCI_PRESENT_STATE:
        break;
    case SDHCI_HOST_CONTROL:
        s->host_control = value;
        break;
    case SDHCI_POWER_CONTROL:
        break;
    case SDHCI_BLOCK_GAP_CONTROL:
        break;
    case SDHCI_WAKE_UP_CONTROL:
        break;
    case SDHCI_CLOCK_CONTROL:
        /* TODO Reset if needed */
        s->clock = (value & 0xfffff) | SDHCI_CLOCK_INT_STABLE;
        break;
    case SDHCI_TIMEOUT_CONTROL:
        break;
    case SDHCI_SOFTWARE_RESET:
        break;
    case SDHCI_INT_STATUS:
        s->int_status &= ~value;
        sdhci_set_irq(s);
        break;
    case SDHCI_INT_ENABLE:
        s->int_enable = value;
        sdhci_set_irq(s);
        break;
    case SDHCI_SIGNAL_ENABLE:
        break;
    case SDHCI_ACMD12_ERR:
        break;
    case SDHCI_MAX_CURRENT:
        break;
    case SDHCI_SET_ACMD12_ERROR:
        break;
    case SDHCI_SET_INT_ERROR:
        break;
    case SDHCI_ADMA_ERROR:
        break;
    case SDHCI_ADMA_ADDRESS:
        s->adma_address = value;
        break;
    case SDHCI_SLOT_INT_STATUS:
        break;
    case SDHCI_HOST_VERSION:
        break;
    case SDHCI_CAPABILITIES: /* Read only */
    case SDHCI_CAPABILITIES_1:
    default:
        hw_error("sdhci_write: Bad offset %x\n", (int)offset);
    }
}

static uint32_t sdhci_vendor_read(sdhci_state *s, target_phys_addr_t offset,
                                  int size)
{
    /* Vendor specific registers */
    switch (offset) {
    case 0x100:
    case 0x104:
    case 0x108:
    case 0x10C:
    case 0x110:
    case 0x114:
    case 0x118:
    case 0x11C:
      return 0;
    default:
        hw_error("sdhci_vendor_read: Bad offset %x\n", offset);
    }
}

static void sdhci_vendor_write(sdhci_state *s, target_phys_addr_t offset,
                               uint32_t value, int size)
{
    /* Vendor specific registers */
    switch (offset) {
    case 0x100:
        break;
    case 0x104:
        break;
    case 0x108:
        break;
    case 0x10C:
        break;
    case 0x110:
        break;
    case 0x114:
        break;
    case 0x118:
        break;
    case 0x11C:
        break;
    default:
        hw_error("sdhci_vendor_write: Bad offset %x\n", offset);
    }
}

static uint32_t sdhci_read32(void *opaque, target_phys_addr_t offset)
{
    sdhci_state *s = (sdhci_state *)opaque;
    DPRINTF("READ32 at %x\n", offset);

    if (offset < 0x100) {
        return sdhci_read(s, offset, 4);
    } else if (offset < 0x200) {
        return sdhci_vendor_read(s, offset, 4);
    }

    return 0;
}

static uint32_t sdhci_read16(void *opaque, target_phys_addr_t offset)
{
    sdhci_state *s = (sdhci_state *)opaque;
    DPRINTF("READ16 at %x\n", offset);

    if (offset < 0x100) {
        return sdhci_read(s, offset, 2);
    } else if (offset < 0x200) {
        return sdhci_vendor_read(s, offset, 2);
    }

    return 0;
}

static uint32_t sdhci_read8(void *opaque, target_phys_addr_t offset)
{
    sdhci_state *s = (sdhci_state *)opaque;
    DPRINTF("READ8 at %x\n", offset);

    if (offset < 0x100) {
        return sdhci_read(s, offset, 1);
    } else if (offset < 0x200) {
        return sdhci_vendor_read(s, offset, 1);
    }

    return 0;
}

static void sdhci_write32(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    sdhci_state *s = (sdhci_state *)opaque;
    DPRINTF("WRITE32 %x at %x\n", value, offset);

    if (offset < 0x100) {
        sdhci_write(s, offset, value, 4);
    } else if (offset < 0x200) {
        sdhci_vendor_write(s, offset, value, 4);
    }

}

static void sdhci_write16(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    sdhci_state *s = (sdhci_state *)opaque;
    DPRINTF("WRITE16 %x at %x\n", value, offset);

    if (offset < 0x100) {
        sdhci_write(s, offset, value, 2);
    } else if (offset < 0x200) {
        sdhci_vendor_write(s, offset, value, 2);
    }

}

static void sdhci_write8(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    sdhci_state *s = (sdhci_state *)opaque;
    DPRINTF("WRITE8 %x at %x\n", value, reg_offset);

    if (offset < 0x100) {
        sdhci_write(s, offset, value, 1);
    } else if (offset < 0x200) {
        sdhci_vendor_write(s, offset, value, 1);
    }

}

static CPUReadMemoryFunc * const sdhci_readfn[] = {
   sdhci_read8,
   sdhci_read16,
   sdhci_read32
};

static CPUWriteMemoryFunc * const sdhci_writefn[] = {
   sdhci_write8,
   sdhci_write16,
   sdhci_write32
};

static int sdhci_init(SysBusDevice *dev)
{
    int iomemtype;
    sdhci_state *s = FROM_SYSBUS(sdhci_state, dev);

    iomemtype = cpu_register_io_memory(sdhci_readfn,
                                       sdhci_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x200, iomemtype);
    sysbus_init_irq(dev, &s->irq);

    return 0;
}

static void sdhci_reset(DeviceState *d)
{
    sdhci_state *s = container_of(d, sdhci_state, busdev.qdev);

    if (!s->sd && s->bs) {
        s->sd = sd_init(s->bs, 0);

        /* TODO move in the right register write */
        sd_enable(s->sd, 1);
    }

    s->sdma_address = 0;
    s->block_size = 0;
    s->block_count = 0;
    s->arg = 0;
    s->transfer_mode = 0;
    s->clock = SDHCI_CLOCK_INT_STABLE;
    s->int_enable = 0;
    s->int_status = 0;
    s->adma_address = 0;
}

static const VMStateDescription sdhci_vmstate = {
    .name = "sdhci",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(sdma_address, sdhci_state),
        VMSTATE_UINT16(block_size, sdhci_state),
        VMSTATE_UINT16(block_count, sdhci_state),
        VMSTATE_UINT32(arg, sdhci_state),
        VMSTATE_UINT16(transfer_mode, sdhci_state),
        VMSTATE_UINT32(clock, sdhci_state),
        VMSTATE_UINT32(int_enable, sdhci_state),
        VMSTATE_UINT32(int_status, sdhci_state),
        VMSTATE_UINT32(adma_address, sdhci_state),
        VMSTATE_UINT8_ARRAY(response, sdhci_state, 16),
        /* TODO SDState *sd ? */
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo sdhci_info = {
    .init = sdhci_init,
    .qdev.name  = "sdhci",
    .qdev.size  = sizeof(sdhci_state),
    .qdev.vmsd  = &sdhci_vmstate,
    .qdev.reset = sdhci_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_DRIVE("block", sdhci_state, bs),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void sdhci_register(void)
{
    sysbus_register_withprop(&sdhci_info);
}

device_init(sdhci_register)
