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
 * Tegra3 uP-TAG
 */

#include "sysbus.h"
#include "sysemu.h"

#define DEBUG_UPTAG 1

#ifdef DEBUG_UPTAG
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_uptag: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

typedef struct {
    SysBusDevice busdev;
    qemu_irq irq;
} tegra_uptag_state;

static uint32_t tegra_uptag_read(void *opaque, target_phys_addr_t offset)
{
    tegra_uptag_state *s = (tegra_uptag_state *)opaque;
    DPRINTF("READ at %x\n", offset);

    switch (offset) {
    case 0x0: /* cpuid? */
        return 0x55555555; // Cortex A9 
    }

    return 0;
}

static void tegra_uptag_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_uptag_state *s = (tegra_uptag_state *)opaque;
    DPRINTF("WRITE at %x <= %x\n", offset, value);

    switch (offset) {
    case 0x00: /* XXXXXXXXX */
        break;
    }
}

static CPUReadMemoryFunc * const tegra_uptag_readfn[] = {
   tegra_uptag_read,
   tegra_uptag_read,
   tegra_uptag_read
};

static CPUWriteMemoryFunc * const tegra_uptag_writefn[] = {
   tegra_uptag_write,
   tegra_uptag_write,
   tegra_uptag_write
};

static int tegra_uptag_init(SysBusDevice *dev)
{
    int iomemtype;
    tegra_uptag_state *s = FROM_SYSBUS(tegra_uptag_state, dev);

    iomemtype = cpu_register_io_memory(tegra_uptag_readfn,
                                       tegra_uptag_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x1000, iomemtype);

    return 0;
}

static void tegra_uptag_reset(DeviceState *d)
{
    tegra_uptag_state *s = container_of(d, tegra_uptag_state, busdev.qdev);
}

static const VMStateDescription tegra_uptag_vmstate = {
    .name = "tegra_uptag",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_uptag_info = {
    .init = tegra_uptag_init,
    .qdev.name  = "tegra_uptag",
    .qdev.size  = sizeof(tegra_uptag_state),
    .qdev.vmsd  = &tegra_uptag_vmstate,
    .qdev.reset = tegra_uptag_reset,
};

static void tegra_uptag_register(void)
{
    sysbus_register_withprop(&tegra_uptag_info);
}

device_init(tegra_uptag_register)
