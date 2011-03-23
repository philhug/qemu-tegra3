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
 * Tegra2 Power Management Controller emulation
 */

#include "sysbus.h"
#include "sysemu.h"

/* #define DEBUG_PMC 1 */

#ifdef DEBUG_PMC
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_pmc: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

typedef struct {
    SysBusDevice busdev;
    uint32_t control;
    qemu_irq irq;
} tegra_pmc_state;

static uint32_t tegra_pmc_read(void *opaque, target_phys_addr_t offset)
{
    tegra_pmc_state *s = (tegra_pmc_state *)opaque;
    DPRINTF("READ at %x\n", offset);

    switch (offset) {
    case 0x00: /* PMC_CNTRL */
        return s->control;
    }

    return 0;
}

static void tegra_pmc_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_pmc_state *s = (tegra_pmc_state *)opaque;
    DPRINTF("WRITE at %x <= %x\n", offset, value);

    switch (offset) {
    case 0x00: /* PMC_CNTRL */
        s->control = value & 0x7fffe;
        if (value & 0x10) {
            /* Main reset */
            printf("Resetting the SoC ...\n");
            qemu_system_reset_request();
        }
        break;
    }
}

static CPUReadMemoryFunc * const tegra_pmc_readfn[] = {
   tegra_pmc_read,
   tegra_pmc_read,
   tegra_pmc_read
};

static CPUWriteMemoryFunc * const tegra_pmc_writefn[] = {
   tegra_pmc_write,
   tegra_pmc_write,
   tegra_pmc_write
};

static int tegra_pmc_init(SysBusDevice *dev)
{
    int iomemtype;
    tegra_pmc_state *s = FROM_SYSBUS(tegra_pmc_state, dev);

    iomemtype = cpu_register_io_memory(tegra_pmc_readfn,
                                       tegra_pmc_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x100, iomemtype);

    return 0;
}

static void tegra_pmc_reset(DeviceState *d)
{
    tegra_pmc_state *s = container_of(d, tegra_pmc_state, busdev.qdev);

    s->control = 0;
}

static const VMStateDescription tegra_pmc_vmstate = {
    .name = "tegra_pmc",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(control, tegra_pmc_state),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_pmc_info = {
    .init = tegra_pmc_init,
    .qdev.name  = "tegra_pmc",
    .qdev.size  = sizeof(tegra_pmc_state),
    .qdev.vmsd  = &tegra_pmc_vmstate,
    .qdev.reset = tegra_pmc_reset,
};

static void tegra_pmc_register(void)
{
    sysbus_register_withprop(&tegra_pmc_info);
}

device_init(tegra_pmc_register)
