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
 * Tegra2 SoC and Seaboard development board emulation
 */

#include "sysbus.h"
#include "qemu-timer.h"

/* #define DEBUG_TIMER 1 */

#ifdef DEBUG_TIMER
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_timer: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

/* 1MHz timers */
#define TMR_BASE_FREQ 1000000

typedef struct {
    SysBusDevice busdev;
    ptimer_state *tmrus;
    ptimer_state *timer[4];
    uint32_t usec_cfg;
    uint32_t ptv[4];
    qemu_irq irq[4];
} tegra_timer_state;

static void tegra_timer_set_ptv(tegra_timer_state *s, int idx, uint32 value)
{
    s->ptv[idx] = value;
    ptimer_set_limit(s->timer[idx], value & 0x1fffffff, 1/*reload*/);
    if (value & 0x80000000) {
        ptimer_run(s->timer[idx], !(value & 0x40000000));
    } else {
        ptimer_stop(s->timer[idx]);
    }
}

static uint32_t tegra_timer_read(void *opaque, target_phys_addr_t offset)
{
    tegra_timer_state *s = (tegra_timer_state *)opaque;
    DPRINTF("read at 0x%x\n", offset);

    switch (offset) {
    case 0x00 /*TIMER_TMR_PTV 0*/:
        return s->ptv[0];
    case 0x04 /*TIMER_TMR_CTR 0*/:
        return ptimer_get_count(s->timer[0]) & 0x1fffffff;
    case 0x08 /*TIMER_TMR_PTV 1*/:
        return s->ptv[1];
    case 0x0C /*TIMER_TMR_CTR 1*/:
        return ptimer_get_count(s->timer[1]) & 0x1fffffff;
    case 0x10 /*TIMERUS_CNTR_1US*/:
        return 0xffffffff - (unsigned)ptimer_get_count(s->tmrus);
    case 0x14 /*TIMERUS_USEC_CFG*/:
        return s->usec_cfg;
    case 0x4c /*TIMERUS_CNTR_FREEZE*/:
        /* Debug feature : not implemented */
        return 0;
    case 0x50 /*TIMER_TMR_PTV 2*/:
        return s->ptv[2];
    case 0x54 /*TIMER_TMR_CTR 2*/:
        return ptimer_get_count(s->timer[2]) & 0x1fffffff;
    case 0x58 /*TIMER_TMR_PTV 3*/:
        return s->ptv[3];
    case 0x5C /*TIMER_TMR_CTR 3*/:
        return ptimer_get_count(s->timer[3]) & 0x1fffffff;
        break;
    default:
        hw_error("tegra_timer_read: Bad offset %x\n", (int)offset);
        return 0;
    }

    return 0;
}

static void tegra_timer_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_timer_state *s = (tegra_timer_state *)opaque;
    DPRINTF("write 0x%x at 0x%x\n", value, offset);

    switch (offset) {
    case 0x00 /*TIMER_TMR_PTV 0*/:
        tegra_timer_set_ptv(s, 0, value);
        break;
    case 0x04 /*TIMER_TMR_CTR 0*/:
        if (value & 0x40000000) {
            qemu_irq_lower(s->irq[0]);
        }
        break;
    case 0x08 /*TIMER_TMR_PTV 1*/:
        tegra_timer_set_ptv(s, 1, value);
        break;
    case 0x0C /*TIMER_TMR_CTR 1*/:
        if (value & 0x40000000) {
            qemu_irq_lower(s->irq[1]);
        }
        break;
    case 0x14 /*TIMERUS_USEC_CFG*/:
        s->usec_cfg = value & 0xffff;
        break;
    case 0x4c /*TIMERUS_CNTR_FREEZE*/:
        /* Debug feature : not implemented */
        break;
    case 0x50 /*TIMER_TMR_PTV 2*/:
        tegra_timer_set_ptv(s, 2, value);
        break;
    case 0x54 /*TIMER_TMR_CTR 2*/:
        if (value & 0x40000000) {
            qemu_irq_lower(s->irq[2]);
        }
        break;
    case 0x58 /*TIMER_TMR_PTV 3*/:
        tegra_timer_set_ptv(s, 3, value);
        break;
    case 0x5C /*TIMER_TMR_CTR 3*/:
        if (value & 0x40000000) {
            qemu_irq_lower(s->irq[3]);
        }
        break;
    case 0x10 /*TIMERUS_CNTR_1US*/: /* Read only */
    default:
        hw_error("tegra_timer_write: Bad offset %x\n", (int)offset);
    }
}

static CPUReadMemoryFunc * const tegra_timer_readfn[] = {
   tegra_timer_read,
   tegra_timer_read,
   tegra_timer_read
};

static CPUWriteMemoryFunc * const tegra_timer_writefn[] = {
   tegra_timer_write,
   tegra_timer_write,
   tegra_timer_write
};

static void tegra_timer_tick(void *opaque)
{
    qemu_irq *irq = (qemu_irq *)opaque;

    DPRINTF("tick %p\n", opaque);
    qemu_irq_raise(*irq);
}

static int tegra_timer_init(SysBusDevice *dev)
{
    QEMUBH *bh;
    int i;
    int iomemtype;
    tegra_timer_state *s = FROM_SYSBUS(tegra_timer_state, dev);

    bh = qemu_bh_new(tegra_timer_tick, s);
    s->tmrus = ptimer_init(bh);
    ptimer_set_freq(s->tmrus, TMR_BASE_FREQ);
    ptimer_set_limit(s->tmrus, 0xffffffff, 1);
    for (i = 0; i < 4; i++) {
        sysbus_init_irq(dev, &s->irq[i]);
        bh = qemu_bh_new(tegra_timer_tick, &s->irq[i]);
        s->timer[i] = ptimer_init(bh);
        ptimer_set_freq(s->timer[i], TMR_BASE_FREQ);
    }

    iomemtype = cpu_register_io_memory(tegra_timer_readfn,
                                       tegra_timer_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x1000, iomemtype);
    return 0;
}

static void tegra_timer_reset(DeviceState *d)
{
    tegra_timer_state *s = container_of(d, tegra_timer_state, busdev.qdev);

    s->usec_cfg = 0xc;
    s->ptv[0] = 0;
    s->ptv[1] = 0;
    s->ptv[2] = 0;
    s->ptv[3] = 0;

    ptimer_run(s->tmrus, 0);
}

static const VMStateDescription tegra_timer_vmstate = {
    .name = "tegra_timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_PTIMER(tmrus, tegra_timer_state),
        VMSTATE_PTIMER(timer[0], tegra_timer_state),
        VMSTATE_PTIMER(timer[1], tegra_timer_state),
        VMSTATE_PTIMER(timer[2], tegra_timer_state),
        VMSTATE_PTIMER(timer[3], tegra_timer_state),
        VMSTATE_UINT32(usec_cfg, tegra_timer_state),
        VMSTATE_UINT32_ARRAY(ptv, tegra_timer_state, 4),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_timer_info = {
    .init = tegra_timer_init,
    .qdev.name  = "tegra_timer",
    .qdev.size  = sizeof(tegra_timer_state),
    .qdev.vmsd  = &tegra_timer_vmstate,
    .qdev.reset = tegra_timer_reset,
};

static void tegra_timer_register(void)
{
    sysbus_register_withprop(&tegra_timer_info);
}

device_init(tegra_timer_register)
