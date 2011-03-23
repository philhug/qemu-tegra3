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
 * Tegra2 GPIO controller emulation
 */

#include "sysbus.h"

/* #define DEBUG_GPIO 1 */

#ifdef DEBUG_GPIO
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_gpio: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

#define BANK_COUNT  7
#define GROUP_COUNT (BANK_COUNT * 4)
#define GPIO_COUNT  (GROUP_COUNT * 8)

#define GROUP_IDX(bank,group) (((bank)<<2) + (group))

enum {
  GPIO_CNF = 0,
  GPIO_OE  = 1,
  GPIO_OUT = 2,
  GPIO_IN  = 3,
  GPIO_INT_STA = 4,
  GPIO_INT_ENB = 5,
  GPIO_INT_LVL = 6,
  GPIO_INT_CLR = 7,
};

typedef struct {
    SysBusDevice busdev;
    uint8_t cnf[GROUP_COUNT];
    uint8_t oe[GROUP_COUNT];
    uint8_t out[GROUP_COUNT];
    uint8_t in[GROUP_COUNT];
    uint8_t int_sta[GROUP_COUNT];
    uint8_t int_enb[GROUP_COUNT];
    uint32_t int_lvl[GROUP_COUNT];
    qemu_irq irq[BANK_COUNT];
    qemu_irq handler[GPIO_COUNT];
} tegra_gpio_state;

static void tegra_gpio_update(tegra_gpio_state *s, int bank)
{
    //qemu_set_irq(s->handler[line], value)
}

static void tegra_gpio_set(void *opaque, int line, int level)
{
    tegra_gpio_state *s = (tegra_gpio_state *)opaque;
    int bank = line >> 5;
    int group = line >> 3;
    int bit = line & 0x7;

    //TODO
    s->in[group] = (s->in[group] & ~(1 << bit)) | (!!level << bit);
    tegra_gpio_update(s, bank);
}

static uint32_t tegra_gpio_read(void *opaque, target_phys_addr_t offset)
{
    tegra_gpio_state *s = (tegra_gpio_state *)opaque;
    int bank = (offset & 0x380) >> 7;
    int reg = (offset & 0x70) >> 4;
    int group = (offset & 0xc) >> 2;
    DPRINTF("READ bank %d reg %d group %d\n", bank, reg, group);

    if (bank >= BANK_COUNT) {
        hw_error("tegra_gpio_read: Bad offset %x\n", (int)offset);
    }

    switch (reg) {
    case GPIO_CNF:
        return s->cnf[GROUP_IDX(bank, group)];
    case GPIO_OE:
        return s->oe[GROUP_IDX(bank, group)];
    case GPIO_OUT:
        return s->out[GROUP_IDX(bank, group)];
    case GPIO_IN:
        return s->in[GROUP_IDX(bank, group)];
    case GPIO_INT_STA:
        return s->int_sta[GROUP_IDX(bank, group)];
    case GPIO_INT_ENB:
        return s->int_enb[GROUP_IDX(bank, group)];
    case GPIO_INT_LVL:
        return s->int_lvl[GROUP_IDX(bank, group)];
    case GPIO_INT_CLR:
        /* Write only */
        return 0;
    }

    return 0;
}

static inline void tegra_gpio_write_masked(tegra_gpio_state *s, uint8_t *reg,
                                           int is_masked, int bank, int group,
                                           uint32_t value)
{
    if (is_masked) {
        uint8_t mask = (value >> 8);
        reg[GROUP_IDX(bank, group)] = (value & mask) |
                                      (reg[GROUP_IDX(bank, group)] & ~mask);
    } else {
        reg[GROUP_IDX(bank, group)] = value & 0xff;
    }
    tegra_gpio_update(s, bank);
}

static void tegra_gpio_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_gpio_state *s = (tegra_gpio_state *)opaque;
    int bank = (offset & 0x380) >> 7;
    int reg = (offset & 0x70) >> 4;
    int group = (offset & 0xc) >> 2;
    int pin_mask = offset & 0x800;
    DPRINTF("WRITE bank %d reg %d group %d (per pin mask %d) <= %x\n",
            bank, reg, group, !!pin_mask, value);

    if (bank >= BANK_COUNT) {
        hw_error("tegra_gpio_write: Bad offset %x\n", (int)offset);
    }

    switch (reg) {
    case GPIO_CNF:
        tegra_gpio_write_masked(s, s->cnf, pin_mask, bank, group, value);
        break;
    case GPIO_OE:
        tegra_gpio_write_masked(s, s->oe, pin_mask, bank, group, value);
        break;
    case GPIO_OUT:
        tegra_gpio_write_masked(s, s->out, pin_mask, bank, group, value);
        break;
    case GPIO_IN:
        /* Read only */
        break;
    case GPIO_INT_STA:
        tegra_gpio_write_masked(s, s->int_sta, pin_mask, bank, group, value);
        break;
    case GPIO_INT_ENB:
        tegra_gpio_write_masked(s, s->int_enb, pin_mask, bank, group, value);
        break;
    case GPIO_INT_LVL:
        break;
    case GPIO_INT_CLR:
        break;
    }
}

static CPUReadMemoryFunc * const tegra_gpio_readfn[] = {
   tegra_gpio_read,
   tegra_gpio_read,
   tegra_gpio_read
};

static CPUWriteMemoryFunc * const tegra_gpio_writefn[] = {
   tegra_gpio_write,
   tegra_gpio_write,
   tegra_gpio_write
};

static int tegra_gpio_init(SysBusDevice *dev)
{
    int iomemtype;
    int i;
    tegra_gpio_state *s = FROM_SYSBUS(tegra_gpio_state, dev);

    qdev_init_gpio_in(&dev->qdev, tegra_gpio_set, GPIO_COUNT);
    qdev_init_gpio_out(&dev->qdev, s->handler, GPIO_COUNT);

    iomemtype = cpu_register_io_memory(tegra_gpio_readfn,
                                       tegra_gpio_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x1000, iomemtype);
    for (i = 0; i < BANK_COUNT; i++) {
        sysbus_init_irq(dev, &s->irq[i]);
    }

    return 0;
}

static void tegra_gpio_reset(DeviceState *d)
{
    tegra_gpio_state *s = container_of(d, tegra_gpio_state, busdev.qdev);

    memset(s->cnf, 0, GROUP_COUNT * sizeof(uint8_t));
    memset(s->oe, 0, GROUP_COUNT * sizeof(uint8_t));
    memset(s->out, 0, GROUP_COUNT * sizeof(uint8_t));
    memset(s->in, 0, GROUP_COUNT * sizeof(uint8_t));
    memset(s->int_sta, 0, GROUP_COUNT * sizeof(uint8_t));
    memset(s->int_enb, 0, GROUP_COUNT * sizeof(uint8_t));
    memset(s->int_lvl, 0, GROUP_COUNT * sizeof(uint32_t));
}

static const VMStateDescription tegra_gpio_vmstate = {
    .name = "tegra_gpio",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(cnf, tegra_gpio_state, GROUP_COUNT),
        VMSTATE_UINT8_ARRAY(oe, tegra_gpio_state, GROUP_COUNT),
        VMSTATE_UINT8_ARRAY(out, tegra_gpio_state, GROUP_COUNT),
        VMSTATE_UINT8_ARRAY(in, tegra_gpio_state, GROUP_COUNT),
        VMSTATE_UINT8_ARRAY(int_sta, tegra_gpio_state, GROUP_COUNT),
        VMSTATE_UINT8_ARRAY(int_enb, tegra_gpio_state, GROUP_COUNT),
        VMSTATE_UINT32_ARRAY(int_lvl, tegra_gpio_state, GROUP_COUNT),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_gpio_info = {
    .init = tegra_gpio_init,
    .qdev.name  = "tegra_gpio",
    .qdev.size  = sizeof(tegra_gpio_state),
    .qdev.vmsd  = &tegra_gpio_vmstate,
    .qdev.reset = tegra_gpio_reset,
};

static void tegra_gpio_register(void)
{
    sysbus_register_withprop(&tegra_gpio_info);
}

device_init(tegra_gpio_register)
