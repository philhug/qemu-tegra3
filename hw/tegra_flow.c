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
 * Tegra3 FLOW
 */

#include "sysbus.h"
#include "sysemu.h"

#define DEBUG_FLOW 1

#ifdef DEBUG_FLOW
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_flow: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

typedef struct {
    SysBusDevice busdev;
    qemu_irq irq;
} tegra_flow_state;

static uint32_t tegra_flow_read(void *opaque, target_phys_addr_t offset)
{
    tegra_flow_state *s = (tegra_flow_state *)opaque;
    DPRINTF("READ at 0x%x\n", offset);

    switch (offset) {
    case 0x0: /* FLOW */
        return 0x0;
    case 0x2c: /* flow ctl state??? */
	return 0x04004001;
    }
    return 0;
}

static void tegra_flow_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_flow_state *s = (tegra_flow_state *)opaque;
    DPRINTF("WRITE at 0x%x <= 0x%x\n", offset, value);

    switch (offset) {
    case 0x00: /* XXXXXXXXX */
        break;
    }
}

static CPUReadMemoryFunc * const tegra_flow_readfn[] = {
   tegra_flow_read,
   tegra_flow_read,
   tegra_flow_read
};

static CPUWriteMemoryFunc * const tegra_flow_writefn[] = {
   tegra_flow_write,
   tegra_flow_write,
   tegra_flow_write
};

static int tegra_flow_init(SysBusDevice *dev)
{
    int iomemtype;
    tegra_flow_state *s = FROM_SYSBUS(tegra_flow_state, dev);

    iomemtype = cpu_register_io_memory(tegra_flow_readfn,
                                       tegra_flow_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x1000, iomemtype);


    return 0;
}

static void tegra_flow_reset(DeviceState *d)
{
    tegra_flow_state *s = container_of(d, tegra_flow_state, busdev.qdev);

}

static const VMStateDescription tegra_flow_vmstate = {
    .name = "tegra_flow",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_flow_info = {
    .init = tegra_flow_init,
    .qdev.name  = "tegra_flow",
    .qdev.size  = sizeof(tegra_flow_state),
    .qdev.vmsd  = &tegra_flow_vmstate,
    .qdev.reset = tegra_flow_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_END_OF_LIST(),
    }

};

static void tegra_flow_register(void)
{
    sysbus_register_withprop(&tegra_flow_info);
}

device_init(tegra_flow_register)
