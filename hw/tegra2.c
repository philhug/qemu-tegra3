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
#include "sysemu.h"
#include "boards.h"
#include "arm-misc.h"
#include "i2c.h"
#include "loader.h"
#include "blockdev.h"
#include "pc.h"

#define SMP_EXCEPTION_VECTORS 0x6000F000

#define UBOOT_BASE_ADDR 0x00e08000

static void main_cpu_reset(void *opaque)
{
    CPUState *env = opaque;

    cpu_reset(env);

    /* Jump to the bootloader entry point */
    env->regs[15] = UBOOT_BASE_ADDR;
}

static void secondary_cpu_reset(void *opaque)
{
    CPUState *env = opaque;
    uint32_t boot_code = 0xe59ff0f8; /* ldr r0, [pc, #0xf8] */

    cpu_reset(env);

    /* mimic the ROM code? which jumps to the adress stored at
     * SMP_EXCEPTION_VECTORS+0x100
     */
    cpu_physical_memory_write(SMP_EXCEPTION_VECTORS,
                              (uint8_t *)&boot_code, sizeof(boot_code));

    env->regs[15] = SMP_EXCEPTION_VECTORS;
    /* secondary CPU is not started at boot*/
    env->halted = 1;
}

/*
 * Return the character driver attached to an UART
 *
 * Use the chardev "uartX" if it exists,
 * else serial_hds (at provided by "-serial xxx" option if it exists
 * else the null driver
 */
static CharDriverState *serial_drv(int index)
{
    char name[6];
    CharDriverState *drv;

    snprintf(name, sizeof(name), "uart%c", 'A'+index);
    drv = qemu_chr_find(name);
    if (!drv) {
        drv = (index < MAX_SERIAL_PORTS) && serial_hds[index] ?
                serial_hds[index] : qemu_chr_open(name, "null", NULL);
    }

    return drv;
}

static void tegra2_init(ram_addr_t ram_size, uint8_t *keymap, i2c_bus **i2c)
{
    CPUState *env;
    ram_addr_t ram_offset;
    qemu_irq *cpu_pic;
    DeviceState *dev;
    DeviceState *sdhci;
    DriveInfo *dinfo;
    SysBusDevice *busdev;
    qemu_irq irq[160];
    qemu_irq cpu_irq[4];
    qemu_irq cpu_fiq[4];
    char *filename;
    int ret = -1;
    int i;

    /* Initialize dual core Cortex-A9 */
    for (i = 0; i < smp_cpus ; i++) {
        env = cpu_init("cortex-a9");
        if (!env) {
            fprintf(stderr, "Invalid CPU model\n");
            exit(1);
        }
        cpu_pic = arm_pic_init_cpu(env);
        cpu_irq[i] = cpu_pic[ARM_PIC_CPU_IRQ];
        cpu_fiq[i] = cpu_pic[ARM_PIC_CPU_FIQ];
        if (i > 0) {
            qemu_register_reset(secondary_cpu_reset, env);
        } else {
            qemu_register_reset(main_cpu_reset, env);
        }
    }

    ram_offset = qemu_ram_alloc(NULL, "tegra.ram", ram_size);
    /* SDRAM at address zero*/
    cpu_register_physical_memory(0, ram_size, ram_offset | IO_MEM_RAM);

    /* special page used to boot secondary CPU */
    ram_offset = qemu_ram_alloc(NULL, "tegra.exception_vectors", 0x1000);
    cpu_register_physical_memory(SMP_EXCEPTION_VECTORS, 0x1000,
                                 ram_offset | IO_MEM_RAM);

    /* GIC based interrupt controller */
    dev = qdev_create(NULL, "a9mpcore_priv");
    qdev_prop_set_uint32(dev, "num-cpu", smp_cpus);
    qdev_init_nofail(dev);
    busdev = sysbus_from_qdev(dev);
    sysbus_mmio_map(busdev, 0, 0x50040000 /* Cortex A9 CPU registers */);
    for (i = 0; i < smp_cpus; i++) {
        sysbus_connect_irq(busdev, i, cpu_irq[i]);
    }
    for (i = 0; i < 128; i++) {
        irq[i] = qdev_get_gpio_in(dev, i);
    }

    sysbus_create_simple("tegra_clocks", 0x60006000, 0/*IRQ*/);
    sysbus_create_simple("tegra_pmc", 0x7000e400, 0/*IRQ*/);
    sysbus_create_varargs("tegra_timer", 0x60005000,
                          irq[0], irq[1], irq[41], irq[42], NULL);
    sysbus_create_varargs("tegra_gpio", 0x6000D000,
                          irq[32], irq[33], irq[34], irq[35],
                          irq[57], irq[87], irq[89], NULL);

    /*
     * UARTs
     *
     * uart A/C/E are on clk_m at 12Mhz
     * uart B/D are on pll_p at 216Mhz
     */
    serial_mm_init(0x70006000, 2, irq[36], 12000000/16, serial_drv(0), 1, 0);
    serial_mm_init(0x70006040, 2, irq[37], 216000000/16, serial_drv(1), 1, 0);
    serial_mm_init(0x70006200, 2, irq[46], 12000000/16, serial_drv(2), 1, 0);
    serial_mm_init(0x70006300, 2, irq[90], 216000000/16, serial_drv(3), 1, 0);
    serial_mm_init(0x70006400, 2, irq[91], 12000000/16, serial_drv(4), 1, 0);

    /* SD/MMC controllers */
    dinfo = drive_get(IF_EMMC, 0, 0);
    if (!dinfo) {
        fprintf(stderr, "missing eMMC image\n");
        exit(1);
    }
    sysbus_create_simple("sdhci", 0xc8000000, irq[14]);
    sysbus_create_simple("sdhci", 0xc8000200, irq[15]);
    sysbus_create_simple("sdhci", 0xc8000400, irq[19]);
    sdhci = sysbus_create_varargs("sdhci", 0xc8000600, irq[31], NULL);
    qdev_prop_set_drive_nofail(sdhci, "block", dinfo->bdrv);

    /* I2C busses */
    dev = sysbus_create_simple("tegra_i2c", 0x7000C000, irq[38]);
    i2c[0] = (i2c_bus *)qdev_get_child_bus(dev, "i2c");
    dev = sysbus_create_simple("tegra_i2c", 0x7000C400, irq[84]);
    i2c[1] = (i2c_bus *)qdev_get_child_bus(dev, "i2c");
    dev = sysbus_create_simple("tegra_i2c", 0x7000C500, irq[92]);
    i2c[2] = (i2c_bus *)qdev_get_child_bus(dev, "i2c");
    dev = sysbus_create_simple("tegra_i2c", 0x7000D000, irq[53]);
    qdev_prop_set_int32(dev, "is_dvc", 1);
    i2c[3] = (i2c_bus *)qdev_get_child_bus(dev, "i2c");

    sysbus_create_simple("tegra_sflash", 0x7000C380, irq[39]);
    sysbus_create_simple("tegra_dc", 0x54200000, irq[73]);

    dev = sysbus_create_simple("tegra_kbc", 0x7000e200, irq[85]);
    qdev_prop_set_string(dev, "keymap", (char *)keymap);

    /* USB EHCI host controllers */
    sysbus_create_simple("usb-ehci", 0xc5000000, irq[20]);
#if 0 /* multiple EHCI controllers support not ready */
    sysbus_create_simple("usb-ehci", 0xc5004000, irq[21]);
    sysbus_create_simple("usb-ehci", 0xc5005000, irq[97]);
#endif

    /*
     * "Emulate" the ROM code behaviour :
     * Load the bootloader in RAM
     */
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        ret = load_image_targphys(filename, UBOOT_BASE_ADDR, -1);
        qemu_free(filename);
    }
    if (ret <= 0) {
        fprintf(stderr, "Couldn't load bootloader image.\n");
        exit(1);
    }
}

/* ---------------------- Seaboard ------------------------ */

static uint8_t seaboard_keymap[0x80] = {
/*           ESC    1     2     3     4     5     6  */
/*00*/0x00, 0xf8, 0xf6, 0xe4, 0xe3, 0x99, 0x98, 0xa1,
/*      7     8     9     0     -     =   bkspc  tab */
/*08*/0xa0, 0xa9, 0xa8, 0xb1, 0xb0, 0xb9, 0xe2, 0xfb,
/*      Q     W     E     R     T     Y     U     I  */
/*10*/0xf3, 0x82, 0x9b, 0x9a, 0xa2, 0xab, 0xaa, 0xb3,
/*      O     P     [     ]   enter Lctrl   A     S  */
/*18*/0xb2, 0xd9, 0xd8, 0xba, 0xbb, 0x00, 0x84, 0x83,
/*      D     F     G     H     J     K     L     ;  */
/*20*/0x9d, 0x9c, 0xa4, 0xa3, 0xac, 0xb5, 0xb4, 0xdb,
/*      '     `   Lshift  |     Z     X     C     V  */
/*28*/0xda, 0xf9, 0xc4, 0xaf, 0x85, 0x9e, 0xa6, 0xa5,
/*      B     N     M     ,     .     /  Rshift  Kp* */
/*30*/0xae, 0xad, 0xb7, 0xb6, 0xdd, 0xdc, 0xc4, 0x00,
/*    LAlt  space capslk  F1    F2    F3    F4    F5 */
/*38*/0x00, 0xa7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*      F6    F7    F8    F9   F10  NmLck ScLck kp-7 */
/*40*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*    kp-8  kp-9  kp--  kp-4  kp-5  kp-6  kp-+  kp-1 */
/*48*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*    kp-2  kp-3  kp-0  kp-.                         */
/*50*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*                                                   */
/*58*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*                                                   */
/*60*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*                                                   */
/*68*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*                                                   */
/*70*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*                                                   */
/*78*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static void seaboard_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    i2c_bus *i2c[4];

    tegra2_init(1024*1024*1024/* 1GB */, seaboard_keymap, i2c);
}

static QEMUMachine seaboard_machine = {
    .name = "seaboard",
    .desc = "NVidia Seaboard (Tegra2)",
    .init = seaboard_init,
    .max_cpus = 2,
    .is_default = 1,
};

/* ---------------------- Kaen ------------------------ */

static uint8_t kaen_keymap[0x80] = {
/*           ESC    1     2     3     4     5     6  */
/*00*/0x00, 0x89, 0x8e, 0xae, 0x96, 0xa6, 0xa3, 0xc3,
/*      7     8     9     0     -     =   bkspc  tab */
/*08*/0xc6, 0xb6, 0xde, 0xd6, 0xd3, 0xd0, 0xf1, 0x8a,
/*      Q     W     E     R     T     Y     U     I  */
/*10*/0x8f, 0xaf, 0x97, 0xa7, 0xa2, 0xc2, 0xc7, 0xb7,
/*      O     P     [     ]   enter Lctrl   A     S  */
/*18*/0xdf, 0xd7, 0xd2, 0xb2, 0xf4, 0x00, 0x8c, 0xac,
/*      D     F     G     H     J     K     L     ;  */
/*20*/0x94, 0xa4, 0xa1, 0xc1, 0xc4, 0xb4, 0xdc, 0xd4,
/*      '     `   Lshift  |     Z     X     C     V  */
/*28*/0xd1, 0x8b, 0xcf, 0xf3, 0x8d, 0xad, 0x95, 0xa5,
/*      B     N     M     ,     .     /  Rshift  Kp* */
/*30*/0xa0, 0xc0, 0xc5, 0xb5, 0xdd, 0xd5, 0xcd, 0x00,
/*    LAlt  space capslk  F1    F2    F3    F4    F5 */
/*38*/0x00, 0xf5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*      F6    F7    F8    F9   F10  NmLck ScLck kp-7 */
/*40*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*    kp-8  kp-9  kp--  kp-4  kp-5  kp-6  kp-+  kp-1 */
/*48*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*    kp-2  kp-3  kp-0  kp-.                         */
/*50*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*                                                   */
/*58*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*                                                   */
/*60*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*                                                   */
/*68*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*                                                   */
/*70*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
/*                                                   */
/*78*/0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static void kaen_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    i2c_bus *i2c[4];

    tegra2_init(1024*1024*1024/* 1GB */, kaen_keymap, i2c);

    i2c_create_slave(i2c[1], "bq20z75", 0x0b);
    i2c_create_slave(i2c[3], "nct1008", 0x4c);
}

static QEMUMachine kaen_machine = {
    .name = "kaen",
    .desc = "Kaen (Tegra2)",
    .init = kaen_init,
    .max_cpus = 2,
};

static void tegra_machine_init(void)
{
    qemu_register_machine(&seaboard_machine);
    qemu_register_machine(&kaen_machine);
}

machine_init(tegra_machine_init);
