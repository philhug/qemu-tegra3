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
 * Tegra2 keyboard controller emulation
 */

#include "sysbus.h"
#include "console.h"

/* #define DEBUG_KBC 1 */

#ifdef DEBUG_KBC
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_kbc: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

#define FIFO_SIZE_LOG2 3
#define FIFO_SIZE      (1<<FIFO_SIZE_LOG2)
#define FIFO_MASK      ((1<<FIFO_SIZE_LOG2)-1)

#define USED_MASK      0x8080808080808080ULL

#define CONTROL_EN              (1<<0)
#define CONTROL_KP_INT_EN       (1<<1)
#define CONTROL_FIFO_OVF_INT_EN (1<<2)
#define CONTROL_FIFO_CNT_INT_EN (1<<3)
#define CONTROL_FIFO_TH_CNT(x)  (((x)>>14)&0xf)
#define CONTROL_FIFO_MODE       (1<<18)

enum {
    KBC_MODE_WKUP = 0,
    KBC_MODE_CP = 1
};

typedef struct {
    SysBusDevice busdev;
    uint32_t control;
    uint32_t intr;
    uint32_t row_cfg[4];
    uint32_t col_cfg[3];
    uint32_t to_cnt;
    uint32_t init_dly;
    uint32_t rpt_dly;
    uint32_t row_mask[16];
    uint64_t kp_ent;
    uint64_t fifo[FIFO_SIZE];
    uint64_t kp_state;
    int32_t fifo_count;
    int32_t fifo_ptr;
    int32_t mode;
    uint8_t pressed[0x80];
    char *keymap;
    qemu_irq irq;
} tegra_kbc_state;


static void tegra_kbc_update(tegra_kbc_state *s, int keypress)
{
    if ((s->control & CONTROL_FIFO_CNT_INT_EN) &&
        (s->fifo_count >= CONTROL_FIFO_TH_CNT(s->control))) {
        s->intr |= (1<<2);
    }
    if ((s->control & CONTROL_KP_INT_EN) && keypress) {
        s->intr |= (1<<0);
    }
    if (s->intr) {
        qemu_irq_raise(s->irq);
    } else {
        qemu_irq_lower(s->irq);
    }
}

static void tegra_kbc_event(void *opaque, int keycode)
{
    uint8_t val;
    int scancode = keycode & 0x7f;
    tegra_kbc_state *s = (tegra_kbc_state *)opaque;

    if (!(s->control & CONTROL_EN)) {
        return;
    }

    DPRINTF("Key press 0x%x\n", keycode);

    val = (uint8_t)s->keymap[scancode];
    if (keycode & 0x80) {
        /* key release */
        int slot = s->pressed[scancode];
        if (slot != 0xff) {
            s->kp_state &= ~(0xffULL << slot);
            s->pressed[scancode] = 0xff;
        } else { /* key was not pressed */
            DPRINTF("Invalid key release\n");
        }
    } else {
        /* key press */
        if (s->pressed[scancode] == 0xff) { /* not already pressed */
            int free_slot = __builtin_ffsll(~s->kp_state & USED_MASK) - 8;
            if (free_slot >= 0) {
                s->kp_state |= ((uint64_t)val << free_slot);
                s->pressed[scancode] = free_slot;
            } else {
                DPRINTF("Too many simultaneous keypress\n");
            }
        }
    }

    s->mode = s->kp_state ? KBC_MODE_CP : KBC_MODE_WKUP;

    if (s->fifo_count == FIFO_SIZE) {
        /* set fifo overflow interrupt if needed) */
        if (s->control & CONTROL_FIFO_OVF_INT_EN) {
            s->intr |= (1<<1);
        }
    } else {
        s->fifo[(s->fifo_ptr + s->fifo_count) & FIFO_MASK] = s->kp_state;
        s->fifo_count++;
    }

    tegra_kbc_update(s, 1);
}

static uint32_t tegra_kbc_read(void *opaque, target_phys_addr_t offset)
{
    tegra_kbc_state *s = (tegra_kbc_state *)opaque;
    DPRINTF("READ at %x\n", offset);

    switch (offset) {
    case 0x00: /* KBC_CONTROL */
        return s->control;
    case 0x04: /* KBC_INT */
        return s->intr | (s->mode << 3) | (s->fifo_count << 4);
    case 0x08: /* KBC_ROW_CFGx */
    case 0x0C:
    case 0x10:
    case 0x14:
        return s->row_cfg[(offset - 8) / 4];
    case 0x18: /* KBC_COL_CFGx */
    case 0x1c:
    case 0x20:
        return s->col_cfg[(offset - 0x18) / 4];
    case 0x24: /* KBC_TO_CNT */
        return s->to_cnt;
    case 0x28: /* KBC_INIT_DLY */
        return s->init_dly;
    case 0x2c: /* KBC_RPT_DLY */
        return s->rpt_dly;
    case 0x30: /* KBC_KP_ENT0 */
        if (s->fifo_count) {
            s->kp_ent = s->fifo[s->fifo_ptr];
            s->fifo_ptr = (s->fifo_ptr + 1) & FIFO_MASK;
            s->fifo_count--;
        } else {
            s->kp_ent = 0;
        }
        return s->kp_ent & 0xffffffff;
    case 0x34: /* KBC_KP_ENT1 */
        return s->kp_ent >> 32;
    case 0x38: /* KBC_ROWx_MASK */
    default:
        if (offset > 0x74) {
            /* hw_error("tegra_kbc_read: Bad offset %x\n", (int)offset); */
            return 0;
        }
        return s->row_mask[(offset - 0x38) / 4];
    }

    return 0;
}

static void tegra_kbc_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_kbc_state *s = (tegra_kbc_state *)opaque;
    DPRINTF("WRITE at %x <= %x\n", offset, value);

    switch (offset) {
    case 0x00: /* KBC_CONTROL */
        s->control = value & 0x7ffff;
        tegra_kbc_update(s, 0);
        break;
    case 0x04: /* KBC_INT */
        s->intr = s->intr & ~(value & 0x7);
        tegra_kbc_update(s, 0);
        break;
    case 0x08: /* KBC_ROW_CFGx */
    case 0x0C:
    case 0x10:
    case 0x14:
        s->row_cfg[(offset-8) / 4] = value & 0x3fffffff;
        break;
    case 0x18: /* KBC_COL_CFGx */
    case 0x1c:
    case 0x20:
        s->col_cfg[(offset-0x18)/4] = value;
        break;
    case 0x24: /* KBC_TO_CNT */
        s->to_cnt = value & 0xfffff;
        break;
    case 0x28: /* KBC_INIT_DLY */
        s->init_dly = value & 0xfffff;
        break;
    case 0x2c: /* KBC_RPT_DLY */
        s->rpt_dly = value & 0xfffff;
        break;
    case 0x30: /* KBC_KP_ENT0 */
    case 0x34: /* KBC_KP_ENT1 */
        hw_error("tegra_kbc_write: read only register\n");
        break;
    case 0x38: /* KBC_ROWx_MASK */
    default:
        if (offset > 0x74) {
            /*hw_error("tegra_kbc_write: Bad offset %x\n", (int)offset);*/
            return;
        }
        s->row_mask[(offset - 0x38) / 4] = value & 0xff;
    }
}

static CPUReadMemoryFunc * const tegra_kbc_readfn[] = {
   tegra_kbc_read,
   tegra_kbc_read,
   tegra_kbc_read
};

static CPUWriteMemoryFunc * const tegra_kbc_writefn[] = {
   tegra_kbc_write,
   tegra_kbc_write,
   tegra_kbc_write
};

static int tegra_kbc_init(SysBusDevice *dev)
{
    int iomemtype;
    tegra_kbc_state *s = FROM_SYSBUS(tegra_kbc_state, dev);

    iomemtype = cpu_register_io_memory(tegra_kbc_readfn,
                                       tegra_kbc_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x100, iomemtype);
    sysbus_init_irq(dev, &s->irq);

    qemu_add_kbd_event_handler(tegra_kbc_event, s);

    return 0;
}

static void tegra_kbc_reset(DeviceState *d)
{
    tegra_kbc_state *s = container_of(d, tegra_kbc_state, busdev.qdev);

    s->control = 0x10000;
    s->intr = 0;
    memset(s->row_cfg, 0, sizeof(s->row_cfg));
    memset(s->col_cfg, 0, sizeof(s->col_cfg));
    s->to_cnt = 0x27100;
    s->init_dly = 0x400;
    s->rpt_dly = 0x400;
    memset(s->row_mask, 0, sizeof(s->row_mask));
    s->kp_ent = 0;
    memset(s->fifo, 0, sizeof(s->fifo));
    s->kp_state = 0;
    s->fifo_count = 0;
    s->fifo_ptr = 0;
    s->mode = KBC_MODE_WKUP;
    memset(s->pressed, 0xff, sizeof(s->pressed));
}

static const VMStateDescription tegra_kbc_vmstate = {
    .name = "tegra_kbc",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(control, tegra_kbc_state),
        VMSTATE_UINT32(intr, tegra_kbc_state),
        VMSTATE_UINT32_ARRAY(row_cfg, tegra_kbc_state, 4),
        VMSTATE_UINT32_ARRAY(col_cfg, tegra_kbc_state, 3),
        VMSTATE_UINT32(to_cnt, tegra_kbc_state),
        VMSTATE_UINT32(init_dly, tegra_kbc_state),
        VMSTATE_UINT32(rpt_dly, tegra_kbc_state),
        VMSTATE_UINT32_ARRAY(row_mask, tegra_kbc_state, 16),
        VMSTATE_UINT64(kp_ent, tegra_kbc_state),
        VMSTATE_UINT64_ARRAY(fifo, tegra_kbc_state, FIFO_SIZE),
        VMSTATE_UINT64(kp_state, tegra_kbc_state),
        VMSTATE_INT32(fifo_count, tegra_kbc_state),
        VMSTATE_INT32(fifo_ptr, tegra_kbc_state),
        VMSTATE_INT32(mode, tegra_kbc_state),
        VMSTATE_UINT8_ARRAY(pressed, tegra_kbc_state, 0x80),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_kbc_info = {
    .init = tegra_kbc_init,
    .qdev.name  = "tegra_kbc",
    .qdev.size  = sizeof(tegra_kbc_state),
    .qdev.vmsd  = &tegra_kbc_vmstate,
    .qdev.reset = tegra_kbc_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_STRING("keymap", tegra_kbc_state, keymap),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void tegra_kbc_register(void)
{
    sysbus_register_withprop(&tegra_kbc_info);
}

device_init(tegra_kbc_register)
