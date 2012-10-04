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
 * Tegra3 APB
 */

#include "sysbus.h"
#include "sysemu.h"

#define DEBUG_APB 1

#ifdef DEBUG_APB
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_apb: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

typedef struct {
    SysBusDevice busdev;
    uint32_t control;
    qemu_irq irq;
} tegra_apb_state;

static uint32_t tegra_apb_read(void *opaque, target_phys_addr_t offset)
{
    tegra_apb_state *s = (tegra_apb_state *)opaque;
    DPRINTF("READ at 0x%x\n", offset);

    switch (offset) {
    case 0x804: /* APB_MISC_GP_HIDREV_0 */
        return 0x55555555; // Cortex A9 
    }

    return 0;
}

static void tegra_apb_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_apb_state *s = (tegra_apb_state *)opaque;
    DPRINTF("WRITE at 0x%x <= 0x%x\n", offset, value);

    switch (offset) {
    case 0x00: /* XXXXXXXXX */
        break;
    }
}

static CPUReadMemoryFunc * const tegra_apb_readfn[] = {
   tegra_apb_read,
   tegra_apb_read,
   tegra_apb_read
};

static CPUWriteMemoryFunc * const tegra_apb_writefn[] = {
   tegra_apb_write,
   tegra_apb_write,
   tegra_apb_write
};

static int tegra_apb_init(SysBusDevice *dev)
{
    int iomemtype;
    tegra_apb_state *s = FROM_SYSBUS(tegra_apb_state, dev);

    iomemtype = cpu_register_io_memory(tegra_apb_readfn,
                                       tegra_apb_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x6000, iomemtype);

    return 0;
}

static void tegra_apb_reset(DeviceState *d)
{
    tegra_apb_state *s = container_of(d, tegra_apb_state, busdev.qdev);

    s->control = 0;
}

static const VMStateDescription tegra_apb_vmstate = {
    .name = "tegra_apb",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(control, tegra_apb_state),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_apb_info = {
    .init = tegra_apb_init,
    .qdev.name  = "tegra_apb",
    .qdev.size  = sizeof(tegra_apb_state),
    .qdev.vmsd  = &tegra_apb_vmstate,
    .qdev.reset = tegra_apb_reset,
};

static void tegra_apb_register(void)
{
    sysbus_register_withprop(&tegra_apb_info);
}

device_init(tegra_apb_register)
