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
 * Tegra2 clocks and reset controller emulation
 */

#include "sysbus.h"

/* #define DEBUG_CLOCKS 1 */

#ifdef DEBUG_CLOCKS
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_clocks: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

typedef struct {
    SysBusDevice busdev;
    uint32_t regs[212];
} tegra_clocks_state;

const uint32_t tegra_default_clocks_regs[] = {
    /* 0x000 */ 0x00000000, 0x3ffffec9, 0xfffffb7f, 0x000005ff,
    /* 0x010 */ 0x80000130, 0x00000480, 0x7f009000, 0x00000000,
    /* 0x020 */ 0x10000000, 0x80000000, 0x20000020, 0x00000000,
    /* 0x030 */ 0x00000000, 0x00000077, 0x00000000, 0x00000000,
    /* 0x040 */ 0x00000000, 0x00000000, 0x00000000, 0x00000003,
    /* 0x050 */ 0x000003f1, 0x00000000, 0x00000000, 0x00000000,
    /* 0x060 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x070 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x080 */ 0x0000010c, 0x00000003, 0x00000000, 0x00000100,
    /* 0x090 */ 0x0000010c, 0x00000003, 0x00000000, 0x00000100,
    /* 0x0a0 */ 0x0000010c, 0x00030003, 0x00030003, 0x00000100,
    /* 0x0b0 */ 0x0000010c, 0x00000003, 0x00000000, 0x00000100,
    /* 0x0c0 */ 0x0000010c, 0x00000000, 0x00000000, 0x00000100,
    /* 0x0d0 */ 0x0000010c, 0x00000000, 0x00000000, 0x00000100,
    /* 0x0e0 */ 0x0000010c, 0x00000000, 0x00000000, 0x01000100,
    /* 0x0f0 */ 0x00000101, 0x00000100, 0x00000000, 0x00000000,
    /* 0x100 */ 0xd0000000, 0xd0000000, 0xc0000000, 0x00000000,
    /* 0x110 */ 0x30000000, 0xc0000000, 0xc0000000, 0xc0000000,
    /* 0x120 */ 0xc0000000, 0xc0000000, 0xc0000000, 0xc0000000,
    /* 0x130 */ 0x00000000, 0xc0000000, 0xc0000000, 0xc0000000,
    /* 0x140 */ 0xc0000000, 0xc0000000, 0x00000000, 0x00000000,
    /* 0x150 */ 0xc0000000, 0xc0000000, 0x00000000, 0x00000000,
    /* 0x160 */ 0xc0000000, 0xc0000000, 0xc0000000, 0x00000000,
    /* 0x170 */ 0x00000000, 0xc0000000, 0xc0000000, 0xc0000000,
    /* 0x180 */ 0x00000000, 0x00000000, 0xc0000000, 0xc0000000,
    /* 0x190 */ 0x00000000, 0xc0000000, 0xc0000000, 0xc0000000,
    /* 0x1a0 */ 0xc0000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x1b0 */ 0x00000000, 0xc0000000, 0xc0000000, 0xc0000000,
    /* 0x1c0 */ 0xc0000000, 0xc0000000, 0xc0000000, 0xc0000000,
    /* 0x1d0 */ 0xc0000000, 0xc0000000, 0x00000000, 0x00000000,
    /* 0x1e0 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x1f0 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x200 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x210 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x220 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x230 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x240 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x250 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x260 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x270 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x280 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x290 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x2a0 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x2b0 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x2c0 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x2d0 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x2e0 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x2f0 */ 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    /* 0x300 */ 0x3ffffec9, 0x3ffffec9, 0xfffffbff, 0xfffffb77,
    /* 0x310 */ 0x000005ff, 0x000005ff, 0x00000000, 0x00000000,
    /* 0x320 */ 0x80000130, 0x80000130, 0x00000400, 0x00000400,
    /* 0x330 */ 0x07f00a00, 0x07f00a00, 0x00000000, 0x00000000,
    /* 0x340 */ 0x00002222, 0x00002222, 0x00000000, 0x00000000,
};

static uint32_t tegra_clocks_read(void *opaque, target_phys_addr_t offset)
{
    tegra_clocks_state *s = (tegra_clocks_state *)opaque;
    DPRINTF("READ at 0x%x\n", offset);

    if (offset > sizeof(s->regs)) {
        hw_error("tegra_clocks_read: Bad offset %x\n", (int)offset);
    }
    switch (offset) {
    case 0x5c /* CLK_RST_CONTROLLER_OSC_FREQ_DET_STATUS */:
        /* emulate 12Mhz crystal */
        return 732 * ((s->regs[0x58 / 4] & 0xf) + 1) / 2;
    default:
        return s->regs[offset/sizeof(uint32_t)];
    }

    return 0;
}

static void tegra_clocks_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_clocks_state *s = (tegra_clocks_state *)opaque;
    DPRINTF("WRITE at 0x%x <= 0x%x\n", offset, value);

    if (offset > sizeof(s->regs)) {
        hw_error("tegra_clocks_write: Bad offset %x\n", (int)offset);
    }
    switch (offset) {
    case 0x4C /* CLK_RST_CONTROLLER_CLK_CPU_CMPLX */:
        /* bit 8 : CPU0 ON  bit 9 : CPU1: ON */
        qemu_get_cpu(0)->halted = value&0x100;
        qemu_get_cpu(1)->halted = value&0x200;
        /* allow secondary CPU to run */
        qemu_cpu_kick(qemu_get_cpu(1));
        break;
    default:
        s->regs[offset/sizeof(uint32_t)] = value;
        break;
    }
}

static CPUReadMemoryFunc * const tegra_clocks_readfn[] = {
   tegra_clocks_read,
   tegra_clocks_read,
   tegra_clocks_read
};

static CPUWriteMemoryFunc * const tegra_clocks_writefn[] = {
   tegra_clocks_write,
   tegra_clocks_write,
   tegra_clocks_write
};

static int tegra_clocks_init(SysBusDevice *dev)
{
    int iomemtype;
    tegra_clocks_state *s = FROM_SYSBUS(tegra_clocks_state, dev);

    iomemtype = cpu_register_io_memory(tegra_clocks_readfn,
                                       tegra_clocks_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x1000, iomemtype);
    return 0;
}

static void tegra_clocks_reset(DeviceState *d)
{
    tegra_clocks_state *s = container_of(d, tegra_clocks_state, busdev.qdev);

    memcpy(s->regs, tegra_default_clocks_regs, sizeof(s->regs));
}

static const VMStateDescription tegra_clocks_vmstate = {
    .name = "tegra_clocks",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, tegra_clocks_state, 212),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_clocks_info = {
    .init = tegra_clocks_init,
    .qdev.name  = "tegra_clocks",
    .qdev.size  = sizeof(tegra_clocks_state),
    .qdev.vmsd  = &tegra_clocks_vmstate,
    .qdev.reset = tegra_clocks_reset,
};

static void tegra_clocks_register(void)
{
    sysbus_register_withprop(&tegra_clocks_info);
}

device_init(tegra_clocks_register)
