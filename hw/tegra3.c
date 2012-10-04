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
 * Tegra3 SoC and Nexus 7 emulation
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

#define UBOOT_BASE_ADDR 0x80108000

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

    /* SDRAM at address 0x80000000*/
    ram_offset = qemu_ram_alloc(NULL, "tegra.ram", ram_size);
    cpu_register_physical_memory(0x80000000, ram_size, ram_offset | IO_MEM_RAM);

    /* IRAM at 0x40000000 */
    ram_offset = qemu_ram_alloc(NULL, "tegra.iram", 128*1024*1024);
    cpu_register_physical_memory(0x40000000, 128*1024*1024, ram_offset | IO_MEM_RAM);

    /* special page used to boot secondary CPU */
    ram_offset = qemu_ram_alloc(NULL, "tegra.smp.exception_vectors", 0x1000);
    cpu_register_physical_memory(SMP_EXCEPTION_VECTORS, 0x1000,
                                 ram_offset | IO_MEM_RAM);

    /* GIC based interrupt controller */
    // TODO 
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

    sysbus_create_simple("l2x0", 0x50043000, 0/*IRQ*/);
    sysbus_create_simple("tegra_uptag", 0x60000000, 0/*IRQ*/);
    sysbus_create_simple("tegra_flow", 0x60007000, 0/*IRQ*/);
    sysbus_create_simple("tegra_apb", 0x70000000, 0/*IRQ*/);
    sysbus_create_simple("tegra_clocks", 0x60006000, 0/*IRQ*/);
    sysbus_create_simple("tegra_pmc", 0x7000e400, 0/*IRQ*/);
    sysbus_create_varargs("tegra_timer", 0x60005000,
                          irq[0], irq[1], irq[41], irq[42], NULL);
    sysbus_create_varargs("tegra_gpio", 0x6000D000,
                          irq[32], irq[33], irq[34], irq[35],
                          irq[55], irq[87], irq[89], irq[125], NULL);

    sysbus_create_simple("tegra_mc", 0x7000f000, 0/*IRQ*/);
    sysbus_create_simple("tegra_fuse", 0x7000f800, 0/*IRQ*/);

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
    //    exit(1);
    }

    sdhci = sysbus_create_simple("sdhci", 0x78000000, irq[14]);
    sdhci = sysbus_create_simple("sdhci", 0x78000200, irq[15]);
    sdhci = sysbus_create_simple("sdhci", 0x78000400, irq[19]);
    sdhci = sysbus_create_varargs("sdhci", 0x78000600, irq[31], NULL);
    if (dinfo)
    	qdev_prop_set_drive_nofail(sdhci, "block", dinfo->bdrv);

    /* I2C busses */
    dev = sysbus_create_simple("tegra_i2c", 0x7000C000, irq[38]);
    i2c[0] = (i2c_bus *)qdev_get_child_bus(dev, "i2c");
    dev = sysbus_create_simple("tegra_i2c", 0x7000C400, irq[84]);
    i2c[1] = (i2c_bus *)qdev_get_child_bus(dev, "i2c");
    dev = sysbus_create_simple("tegra_i2c", 0x7000C500, irq[92]);
    i2c[2] = (i2c_bus *)qdev_get_child_bus(dev, "i2c");
    dev = sysbus_create_simple("tegra_i2c", 0x7000C700, irq[120]);
    i2c[3] = (i2c_bus *)qdev_get_child_bus(dev, "i2c");
    dev = sysbus_create_simple("tegra_i2c", 0x7000D000, irq[53]);
    //TODO qdev_prop_set_int32(dev, "is_dvc", 1);
    i2c[4] = (i2c_bus *)qdev_get_child_bus(dev, "i2c");

    sysbus_create_simple("tegra_sflash", 0x7000da00, irq[39]);
    sysbus_create_simple("tegra_dc", 0x54200000, irq[73]);

    dev = sysbus_create_simple("tegra_kbc", 0x7000e200, irq[85]);
    //qdev_prop_set_string(dev, "keymap", (char *)keymap);

    /* USB EHCI host controllers */
    sysbus_create_simple("usb-ehci", 0x7d000000, irq[20]);

    /* multiple EHCI controllers support not ready */
    sysbus_create_simple("usb-ehci", 0x7d004000, irq[21]);
    sysbus_create_simple("usb-ehci", 0x7d008000, irq[97]);

    /*
     * "Emulate" the ROM code behaviour :
     * Load the bootloader in RAM
     */
    if (bios_name) {
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
}

static void grouper_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    i2c_bus *i2c[5];

    tegra2_init(1024*1024*1024/* 1GB */, NULL, i2c);
    i2c_create_slave(i2c[0], "max77663", 0x3c);
    i2c_create_slave(i2c[1], "max77663", 0x3c);
    i2c_create_slave(i2c[2], "max77663", 0x3c);
    i2c_create_slave(i2c[3], "max77663", 0x3c);
    i2c_create_slave(i2c[4], "max77663", 0x3c);
}

static QEMUMachine grouper_machine = {
    .name = "grouper",
    .desc = "Asus Nexus 7 (Tegra3)",
    .init = grouper_init,
    .max_cpus = 4,
    .is_default = 1,
};

static void tegra_machine_init(void)
{
    qemu_register_machine(&grouper_machine);
}

machine_init(tegra_machine_init);
