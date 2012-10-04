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
 * Tegra3 FUSE
 */

#include "sysbus.h"
#include "sysemu.h"

#define DEBUG_FUSE 1

#ifdef DEBUG_FUSE
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_fuse: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

typedef struct {
    SysBusDevice busdev;
    uint32_t control;
    qemu_irq irq;
} tegra_fuse_state;

static uint32_t tegra_fuse_read(void *opaque, target_phys_addr_t offset)
{
    tegra_fuse_state *s = (tegra_fuse_state *)opaque;
    DPRINTF("READ at 0x%x\n", offset);

#if 0
/* FUSE registers */
struct fuse_regs {
        u32 reserved0[64];              /* 0x00 - 0xFC: */
        u32 production_mode;            /* 0x100: FUSE_PRODUCTION_MODE */
        u32 reserved1[3];               /* 0x104 - 0x10c: */
        u32 sku_info;                   /* 0x110 */
        u32 reserved2[13];              /* 0x114 - 0x144: */
        u32 fa;                         /* 0x148: FUSE_FA */
        u32 reserved3[21];              /* 0x14C - 0x19C: */
        u32 security_mode;              /* 0x1A0: FUSE_SECURITY_MODE */
};
#endif

#define FUSE_SPEEDO_CALIB_0     0x114
#define FUSE_PACKAGE_INFO       0X1FC
#define FUSE_TEST_PROG_VER      0X128
#define FUSE_SPARE_BIT_58       0x32c
#define FUSE_SPARE_BIT_59       0x330
#define FUSE_SPARE_BIT_60       0x334
#define FUSE_SPARE_BIT_61       0x338
#define FUSE_SPARE_BIT_62       0x33c
#define FUSE_SPARE_BIT_63       0x340
#define FUSE_SPARE_BIT_64       0x344
#define FUSE_SPARE_BIT_65       0x348


    switch (offset) {
    /* values from Nexus 7 */
    case 0x100: /* FUSE_PRODUCTION_MODE */
        return 0x01;
    case 0x110: /* sku_info */
	return 0x83; // SKU_ID_T30
    case FUSE_SPEEDO_CALIB_0:
	return 0x005e0035;
    case 0x118: /* reserved_2 */
	return 0x00001ae7;
    case 0x11c: /* reserved_2 */
	return 0x0000004f;
    case 0x120:
    case 0x124: /* reserved_2 */
	return 0x00000000;
    case FUSE_TEST_PROG_VER:
	return 0x1e;
    case 0x12c ... 0x144: /* reserved_2 */
	return 0x00000000;
    case 0x148: /* FUSE_FA */
        return 0x0;
    case 0x1a0: /* security_mode */
        return 0x01;
    case FUSE_PACKAGE_INFO:
	return 0x01;
    case FUSE_SPARE_BIT_58:
    case FUSE_SPARE_BIT_59:
    case FUSE_SPARE_BIT_60:
    case FUSE_SPARE_BIT_61:
    case FUSE_SPARE_BIT_62:
    case FUSE_SPARE_BIT_63:
	return 0x01;
    case FUSE_SPARE_BIT_64:
    case FUSE_SPARE_BIT_65:
	return 0x0;
    }

    return 0;
}

static void tegra_fuse_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_fuse_state *s = (tegra_fuse_state *)opaque;
    DPRINTF("WRITE at 0x%x <= 0x%x\n", offset, value);

    switch (offset) {
    case 0x00: /* XXXXXXXXX */
        break;
    }
}

static CPUReadMemoryFunc * const tegra_fuse_readfn[] = {
   tegra_fuse_read,
   tegra_fuse_read,
   tegra_fuse_read
};

static CPUWriteMemoryFunc * const tegra_fuse_writefn[] = {
   tegra_fuse_write,
   tegra_fuse_write,
   tegra_fuse_write
};

static int tegra_fuse_init(SysBusDevice *dev)
{
    int iomemtype;
    tegra_fuse_state *s = FROM_SYSBUS(tegra_fuse_state, dev);

    iomemtype = cpu_register_io_memory(tegra_fuse_readfn,
                                       tegra_fuse_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x400, iomemtype);

    return 0;
}

static void tegra_fuse_reset(DeviceState *d)
{
    tegra_fuse_state *s = container_of(d, tegra_fuse_state, busdev.qdev);

    s->control = 0;
}

static const VMStateDescription tegra_fuse_vmstate = {
    .name = "tegra_fuse",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(control, tegra_fuse_state),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_fuse_info = {
    .init = tegra_fuse_init,
    .qdev.name  = "tegra_fuse",
    .qdev.size  = sizeof(tegra_fuse_state),
    .qdev.vmsd  = &tegra_fuse_vmstate,
    .qdev.reset = tegra_fuse_reset,
};

static void tegra_fuse_register(void)
{
    sysbus_register_withprop(&tegra_fuse_info);
}

device_init(tegra_fuse_register)
