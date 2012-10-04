/*
 * Copyright 2011 Google Inc.
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
 * Tegra3 MC
 */

#include "sysbus.h"
#include "sysemu.h"

#define DEBUG_MC 1

#ifdef DEBUG_MC
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_mc: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

#define MC_CLIENT_HOTRESET_CTRL 0x200
#define MC_CLIENT_HOTRESET_STAT 0x204

typedef struct {
    SysBusDevice busdev;
    qemu_irq irq;
    uint32_t state;
} tegra_mc_state;

static uint32_t tegra_mc_read(void *opaque, target_phys_addr_t offset)
{
    tegra_mc_state *s = (tegra_mc_state *)opaque;
    DPRINTF("READ at 0x%x\n", offset);

    switch (offset) {
    case MC_CLIENT_HOTRESET_CTRL:
    case MC_CLIENT_HOTRESET_STAT:
        return s->state;
    }
    return 0;
}

static void tegra_mc_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_mc_state *s = (tegra_mc_state *)opaque;
    DPRINTF("WRITE at 0x%x <= 0x%x\n", offset, value);

    switch (offset) {
    case MC_CLIENT_HOTRESET_CTRL:
	s->state = value;
        break;
    }
}

static CPUReadMemoryFunc * const tegra_mc_readfn[] = {
   tegra_mc_read,
   tegra_mc_read,
   tegra_mc_read
};

static CPUWriteMemoryFunc * const tegra_mc_writefn[] = {
   tegra_mc_write,
   tegra_mc_write,
   tegra_mc_write
};

static int tegra_mc_init(SysBusDevice *dev)
{
    int iomemtype;
    tegra_mc_state *s = FROM_SYSBUS(tegra_mc_state, dev);

    iomemtype = cpu_register_io_memory(tegra_mc_readfn,
                                       tegra_mc_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x400, iomemtype);

    return 0;
}

static void tegra_mc_reset(DeviceState *d)
{
    tegra_mc_state *s = container_of(d, tegra_mc_state, busdev.qdev);
    s->state=0;
}

static const VMStateDescription tegra_mc_vmstate = {
    .name = "tegra_mc",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_mc_info = {
    .init = tegra_mc_init,
    .qdev.name  = "tegra_mc",
    .qdev.size  = sizeof(tegra_mc_state),
    .qdev.vmsd  = &tegra_mc_vmstate,
    .qdev.reset = tegra_mc_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_END_OF_LIST(),
    }

};

static void tegra_mc_register(void)
{
    sysbus_register_withprop(&tegra_mc_info);
}

device_init(tegra_mc_register)
