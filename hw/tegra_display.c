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
 * Tegra2 display subsystem emulation
 */

#include "sysbus.h"
#include "console.h"
#include "framebuffer.h"
#include "pixel_ops.h"

/* #define DEBUG_DISPLAY 1 */

#ifdef DEBUG_DISPLAY
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "tegra_display: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

/* constants from Linux kernel : drivers/video/tegra/dc/dc_reg.h */

#define DC_WIN_COLOR_PALETTE(x)                 (0x500 + (x))

#define DC_WIN_PALETTE_COLOR_EXT                0x600
#define DC_WIN_H_FILTER_P(x)                    (0x601 + (x))
#define DC_WIN_CSC_YOF                          0x611
#define DC_WIN_CSC_KYRGB                        0x612
#define DC_WIN_CSC_KUR                          0x613
#define DC_WIN_CSC_KVR                          0x614
#define DC_WIN_CSC_KUG                          0x615
#define DC_WIN_CSC_KVG                          0x616
#define DC_WIN_CSC_KUB                          0x617
#define DC_WIN_CSC_KVB                          0x618
#define DC_WIN_V_FILTER_P(x)                    (0x619 + (x))
#define DC_WIN_WIN_OPTIONS                      0x700
#define  H_DIRECTION_INCREMENT          (0 << 0)
#define  H_DIRECTION_DECREMENTT         (1 << 0)
#define  V_DIRECTION_INCREMENT          (0 << 2)
#define  V_DIRECTION_DECREMENTT         (1 << 2)
#define  COLOR_EXPAND                   (1 << 6)
#define  H_FILTER_ENABLE                (1 << 8)
#define  V_FILTER_ENABLE                (1 << 10)
#define  CP_ENABLE                      (1 << 16)
#define  CSC_ENABLE                     (1 << 18)
#define  DV_ENABLE                      (1 << 20)
#define  WIN_ENABLE                     (1 << 30)

#define DC_WIN_BYTE_SWAP                        0x701
#define  BYTE_SWAP_NOSWAP               0
#define  BYTE_SWAP_SWAP2                1
#define  BYTE_SWAP_SWAP4                2
#define  BYTE_SWAP_SWAP4HW              3

#define DC_WIN_BUFFER_CONTROL                   0x702
#define  BUFFER_CONTROL_HOST            0
#define  BUFFER_CONTROL_VI              1
#define  BUFFER_CONTROL_EPP             2
#define  BUFFER_CONTROL_MPEGE           3
#define  BUFFER_CONTROL_SB2D            4

#define DC_WIN_COLOR_DEPTH                      0x703

#define DC_WIN_POSITION                         0x704
#define  H_POSITION(x)          (((x) & 0xfff) >> 0)
#define  V_POSITION(x)          (((x) & 0xfff) >> 16)

#define DC_WIN_SIZE                             0x705
#define  H_SIZE(x)              (((x) >> 0) & 0xfff)
#define  V_SIZE(x)              (((x) >> 16) & 0xfff)

#define DC_WIN_PRESCALED_SIZE                   0x706
#define  H_PRESCALED_SIZE(x)    (((x) & 0x3fff) >> 0)
#define  V_PRESCALED_SIZE(x)    (((x) & 0xfff) >> 16)

#define DC_WIN_H_INITIAL_DDA                    0x707
#define DC_WIN_V_INITIAL_DDA                    0x708
#define DC_WIN_DDA_INCREMENT                    0x709
#define  H_DDA_INC(x)           (((x) & 0xffff) >> 0)
#define  V_DDA_INC(x)           (((x) & 0xffff) >> 16)

#define DC_WIN_LINE_STRIDE                      0x70a
#define  LINE_STRIDE(x)         (x)
#define  UV_LINE_STRIDE(x)      (((x) & 0xffff) >> 16)
#define DC_WIN_BUF_STRIDE                       0x70b
#define DC_WIN_UV_BUF_STRIDE                    0x70c
#define DC_WIN_BUFFER_ADDR_MODE                 0x70d
#define DC_WIN_DV_CONTROL                       0x70e
#define DC_WIN_BLEND_NOKEY                      0x70f
#define DC_WIN_BLEND_1WIN                       0x710
#define DC_WIN_BLEND_2WIN_X                     0x711
#define DC_WIN_BLEND_2WIN_Y                     0x712
#define DC_WIN_BLEND_3WIN_XY                    0x713
#define  CKEY_NOKEY                     (0 << 0)
#define  CKEY_KEY0                      (1 << 0)
#define  CKEY_KEY1                      (2 << 0)
#define  CKEY_KEY01                     (3 << 0)
#define  BLEND_CONTROL_FIX              (0 << 2)
#define  BLEND_CONTROL_ALPHA            (1 << 2)
#define  BLEND_CONTROL_DEPENDANT        (2 << 2)
#define  BLEND_CONTROL_PREMULT          (3 << 2)
#define  BLEND_WEIGHT0(x)               (((x) & 0xff) << 8)
#define  BLEND_WEIGHT1(x)               (((x) & 0xff) << 16)
#define  BLEND(key, control, weight0, weight1)                  \
          (CKEY_ ## key | BLEND_CONTROL_ ## control |           \
           BLEND_WEIGHT0(weight0) | BLEND_WEIGHT1(weight1))


#define DC_WIN_HP_FETCH_CONTROL                 0x714
#define DC_WINBUF_START_ADDR                    0x800
#define DC_WINBUF_START_ADDR_NS                 0x801
#define DC_WINBUF_START_ADDR_U                  0x802
#define DC_WINBUF_START_ADDR_U_NS               0x803
#define DC_WINBUF_START_ADDR_V                  0x804
#define DC_WINBUF_START_ADDR_V_NS               0x805
#define DC_WINBUF_ADDR_H_OFFSET                 0x806
#define DC_WINBUF_ADDR_H_OFFSET_NS              0x807
#define DC_WINBUF_ADDR_V_OFFSET                 0x808
#define DC_WINBUF_ADDR_V_OFFSET_NS              0x809
#define DC_WINBUF_UFLOW_STATUS                  0x80a

typedef struct {
    SysBusDevice busdev;
    DisplayState *ds;
    uint16_t width;
    uint16_t height;
    uint32_t guest_pitch;
    uint32_t host_pitch;
    uint32_t win_options;
    uint32_t win_byte_swap;
    uint32_t win_buffer_control;
    uint32_t win_color_depth;
    uint32_t win_position;
    uint32_t win_size;
    uint32_t win_prescaled_size;
    uint32_t win_h_initial_dda;
    uint32_t win_dda_increment;
    uint32_t win_line_stride;
    uint32_t win_buf_stride;
    uint32_t win_buffer_addr_mode;
    uint32_t win_dv_control;
    uint32_t win_blend_nokey;
    uint32_t win_blend_1win;
    uint32_t win_blend_2win_x;
    uint32_t win_blend_2win_y;
    uint32_t win_blend_3win_xy;
    uint32_t win_hp_fetch_control;
    uint32_t winbuf_start_addr;
    uint32_t winbuf_addr_h_offset;
    uint32_t winbuf_addr_v_offset;
    int invalidate;
    drawfn draw_fn;
    qemu_irq irq;
} tegra_dc_state;


#define draw_line_func drawfn
#define DEPTH 8
#include "tegra_display_template.h"
#define DEPTH 15
#include "tegra_display_template.h"
#define DEPTH 16
#include "tegra_display_template.h"
#define DEPTH 32
#include "tegra_display_template.h"


static void tegra_update_display(void *opaque)
{
    tegra_dc_state *s = (tegra_dc_state *)opaque;
    int first = 0;
    int last;
    DPRINTF("update_display %d x %d %d/%d\n", H_SIZE(s->win_size),
            V_SIZE(s->win_size),
            ds_get_bits_per_pixel(s->ds),
            s->win_color_depth);

    if (!(s->win_options & WIN_ENABLE) || !s->draw_fn) {
        return;
    }

    framebuffer_update_display(s->ds, s->winbuf_start_addr,
                               H_SIZE(s->win_size), V_SIZE(s->win_size),
                               s->guest_pitch,
                               s->host_pitch,
                               0, s->invalidate,
                               s->draw_fn, s, &first, &last);
    s->invalidate = 0;

    if (first >= 0) {
        dpy_update(s->ds, 0, first, H_SIZE(s->win_size), last - first + 1);
    }
}

static void tegra_invalidate_display(void *opaque)
{
    tegra_dc_state *s = (tegra_dc_state *)opaque;
    DPRINTF("invalidate_display %dx%d %d/%d\n", H_SIZE(s->win_size),
            V_SIZE(s->win_size), ds_get_bits_per_pixel(s->ds),
            s->win_color_depth);

    switch (ds_get_bits_per_pixel(s->ds)) {
    case 8:
        s->draw_fn = tegra_draw_fn_8[s->win_color_depth];
        s->guest_pitch = H_SIZE(s->win_size);
        break;
    case 15:
        s->draw_fn = tegra_draw_fn_15[s->win_color_depth];
        s->guest_pitch = H_SIZE(s->win_size) * 2;
        break;
    case 16:
        s->draw_fn = tegra_draw_fn_16[s->win_color_depth];
        s->guest_pitch = H_SIZE(s->win_size) * 2;
        break;
    case 32:
        s->draw_fn = tegra_draw_fn_32[s->win_color_depth];
        s->guest_pitch = H_SIZE(s->win_size) * 4;
        break;
    default:
        fprintf(stderr, "tegra_display: unsupported color depth\n");
        s->draw_fn = NULL;
        s->guest_pitch = 0;
    }
    s->host_pitch = H_SIZE(s->win_size) * ds_get_bits_per_pixel(s->ds) / 8;

    s->invalidate = 1;
    if (s->win_options & WIN_ENABLE) {
        qemu_console_resize(s->ds, H_SIZE(s->win_size), V_SIZE(s->win_size));
    }
}

static uint32_t tegra_dc_read(void *opaque, target_phys_addr_t offset)
{
    tegra_dc_state *s = (tegra_dc_state *)opaque;
    DPRINTF("READ at %x/%x\n", offset, offset/4);

    switch (offset>>2) {
    case DC_WIN_WIN_OPTIONS:
        return s->win_options;
    case DC_WIN_BYTE_SWAP:
        return s->win_byte_swap;
    case DC_WIN_BUFFER_CONTROL:
        return s->win_buffer_control;
    case DC_WIN_COLOR_DEPTH:
        return s->win_color_depth;
    case DC_WIN_POSITION:
        return s->win_position;
    case DC_WIN_SIZE:
        return s->win_size;
    case DC_WIN_PRESCALED_SIZE:
        return s->win_prescaled_size;
    case DC_WIN_H_INITIAL_DDA:
        return s->win_h_initial_dda;
    case DC_WIN_V_INITIAL_DDA:
        /* YUV modes not implemented */
        return 0;
    case DC_WIN_DDA_INCREMENT:
        return s->win_dda_increment;
    case DC_WIN_LINE_STRIDE:
        return s->win_line_stride;
    case DC_WIN_BUF_STRIDE:
        return s->win_buf_stride;
    case DC_WIN_UV_BUF_STRIDE:
        /* YUV modes not implemented */
        return 0;
    case DC_WIN_BUFFER_ADDR_MODE:
        return s->win_buffer_addr_mode;
    case DC_WIN_DV_CONTROL:
        return s->win_dv_control;
    case DC_WIN_BLEND_NOKEY:
        return s->win_blend_nokey;
    case DC_WIN_BLEND_1WIN:
        return s->win_blend_1win;
    case DC_WIN_BLEND_2WIN_X:
        return s->win_blend_2win_x;
    case DC_WIN_BLEND_2WIN_Y:
        return s->win_blend_2win_y;
    case DC_WIN_BLEND_3WIN_XY:
        return s->win_blend_3win_xy;
    case DC_WIN_HP_FETCH_CONTROL:
        return s->win_hp_fetch_control;
    case DC_WINBUF_START_ADDR:
    case DC_WINBUF_START_ADDR_NS:
        return s->winbuf_start_addr;
    case DC_WINBUF_START_ADDR_U:
    case DC_WINBUF_START_ADDR_U_NS:
    case DC_WINBUF_START_ADDR_V:
    case DC_WINBUF_START_ADDR_V_NS:
        /* YUV modes not implemented */
        return 0;
    case DC_WINBUF_ADDR_H_OFFSET:
    case DC_WINBUF_ADDR_H_OFFSET_NS:
        return s->winbuf_addr_h_offset;
    case DC_WINBUF_ADDR_V_OFFSET:
    case DC_WINBUF_ADDR_V_OFFSET_NS:
        return s->winbuf_addr_v_offset;
    default:
        /*hw_error("tegra_dc_read: Bad offset %x\n", (int)offset);*/
        return 0;
    }

    return 0;
}

static void tegra_dc_write(void *opaque, target_phys_addr_t offset,
                          uint32_t value)
{
    tegra_dc_state *s = (tegra_dc_state *)opaque;
    DPRINTF("WRITE at %x/%x <= %x\n", offset, offset/4, value);

    switch (offset>>2) {
    case DC_WIN_WIN_OPTIONS:
        s->win_options = value;
        tegra_invalidate_display(s);
        break;
    case DC_WIN_BYTE_SWAP:
        s->win_byte_swap = value;
        break;
    case DC_WIN_BUFFER_CONTROL:
        s->win_buffer_control = value;
        break;
    case DC_WIN_COLOR_DEPTH:
        s->win_color_depth = value & 31;
        tegra_invalidate_display(s);
        break;
    case DC_WIN_POSITION:
        s->win_position = value;
        break;
    case DC_WIN_SIZE:
        s->win_size = value;
        tegra_invalidate_display(s);
        break;
    case DC_WIN_PRESCALED_SIZE:
        s->win_prescaled_size = value;
        break;
    case DC_WIN_H_INITIAL_DDA:
        s->win_h_initial_dda = value;
        break;
    case DC_WIN_V_INITIAL_DDA:
        /* YUV modes not implemented */
        break;
    case DC_WIN_DDA_INCREMENT:
        s->win_dda_increment = value;
        break;
    case DC_WIN_LINE_STRIDE:
        s->win_line_stride = value;
        break;
    case DC_WIN_BUF_STRIDE:
        s->win_buf_stride = value;
        break;
    case DC_WIN_UV_BUF_STRIDE:
        /* YUV modes not implemented */
        break;
    case DC_WIN_BUFFER_ADDR_MODE:
        s->win_buffer_addr_mode = value;
        break;
    case DC_WIN_DV_CONTROL:
        s->win_dv_control = value;
        break;
    case DC_WIN_BLEND_NOKEY:
        s->win_blend_nokey = value;
        break;
    case DC_WIN_BLEND_1WIN:
        s->win_blend_1win = value;
        break;
    case DC_WIN_BLEND_2WIN_X:
        s->win_blend_2win_x = value;
        break;
    case DC_WIN_BLEND_2WIN_Y:
        s->win_blend_2win_y = value;
        break;
    case DC_WIN_BLEND_3WIN_XY:
        s->win_blend_3win_xy = value;
        break;
    case DC_WIN_HP_FETCH_CONTROL:
        s->win_hp_fetch_control = value;
        break;
    case DC_WINBUF_START_ADDR:
    case DC_WINBUF_START_ADDR_NS:
        s->winbuf_start_addr = value;
        break;
    case DC_WINBUF_START_ADDR_U:
    case DC_WINBUF_START_ADDR_U_NS:
    case DC_WINBUF_START_ADDR_V:
    case DC_WINBUF_START_ADDR_V_NS:
        /* YUV modes not implemented */
        break;
    case DC_WINBUF_ADDR_H_OFFSET:
    case DC_WINBUF_ADDR_H_OFFSET_NS:
        s->winbuf_addr_h_offset = value;
        break;
    case DC_WINBUF_ADDR_V_OFFSET:
    case DC_WINBUF_ADDR_V_OFFSET_NS:
        s->winbuf_addr_v_offset = value;
        break;
    default:
        /*hw_error("tegra_dc_write: Bad offset %x\n", (int)offset);*/
        break;
    }
}

static CPUReadMemoryFunc * const tegra_dc_readfn[] = {
   tegra_dc_read,
   tegra_dc_read,
   tegra_dc_read
};

static CPUWriteMemoryFunc * const tegra_dc_writefn[] = {
   tegra_dc_write,
   tegra_dc_write,
   tegra_dc_write
};


static void tegra_dc_reset(DeviceState *d)
{
    tegra_dc_state *s = container_of(d, tegra_dc_state, busdev.qdev);

    s->invalidate = 1;
    s->draw_fn = NULL;
}

static int tegra_dc_post_load(void *opaque, int version_id)
{
    tegra_dc_state *s = opaque;

    tegra_invalidate_display(s);

    return 0;
}

static int tegra_dc_init(SysBusDevice *dev)
{
    int iomemtype;
    tegra_dc_state *s = FROM_SYSBUS(tegra_dc_state, dev);

    iomemtype = cpu_register_io_memory(tegra_dc_readfn,
                                       tegra_dc_writefn, s,
                                       DEVICE_NATIVE_ENDIAN);
    sysbus_init_mmio(dev, 0x40000, iomemtype);
    sysbus_init_irq(dev, &s->irq);
    s->ds = graphic_console_init(tegra_update_display,
                                 tegra_invalidate_display,
                                 NULL, NULL, s);

    return 0;
}

static const VMStateDescription tegra_dc_vmstate = {
    .name = "tegra_dc",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = tegra_dc_post_load,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT16(width, tegra_dc_state),
        VMSTATE_UINT16(height, tegra_dc_state),
        VMSTATE_UINT32(win_options, tegra_dc_state),
        VMSTATE_UINT32(win_byte_swap, tegra_dc_state),
        VMSTATE_UINT32(win_buffer_control, tegra_dc_state),
        VMSTATE_UINT32(win_color_depth, tegra_dc_state),
        VMSTATE_UINT32(win_position, tegra_dc_state),
        VMSTATE_UINT32(win_size, tegra_dc_state),
        VMSTATE_UINT32(win_prescaled_size, tegra_dc_state),
        VMSTATE_UINT32(win_h_initial_dda, tegra_dc_state),
        VMSTATE_UINT32(win_dda_increment, tegra_dc_state),
        VMSTATE_UINT32(win_line_stride, tegra_dc_state),
        VMSTATE_UINT32(win_buf_stride, tegra_dc_state),
        VMSTATE_UINT32(win_buffer_addr_mode, tegra_dc_state),
        VMSTATE_UINT32(win_dv_control, tegra_dc_state),
        VMSTATE_UINT32(win_blend_nokey, tegra_dc_state),
        VMSTATE_UINT32(win_blend_1win, tegra_dc_state),
        VMSTATE_UINT32(win_blend_2win_x, tegra_dc_state),
        VMSTATE_UINT32(win_blend_3win_xy, tegra_dc_state),
        VMSTATE_UINT32(win_hp_fetch_control, tegra_dc_state),
        VMSTATE_UINT32(winbuf_start_addr, tegra_dc_state),
        VMSTATE_UINT32(winbuf_addr_h_offset, tegra_dc_state),
        VMSTATE_UINT32(winbuf_addr_v_offset, tegra_dc_state),
        VMSTATE_END_OF_LIST()
    }
};

static SysBusDeviceInfo tegra_dc_info = {
    .init = tegra_dc_init,
    .qdev.name  = "tegra_dc",
    .qdev.size  = sizeof(tegra_dc_state),
    .qdev.vmsd  = &tegra_dc_vmstate,
    .qdev.reset = tegra_dc_reset,
    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT16("width", tegra_dc_state, width,     -1),
        DEFINE_PROP_UINT16("height", tegra_dc_state, height,    -1),
        DEFINE_PROP_END_OF_LIST(),
    }
};

static void tegra_display_register(void)
{
    sysbus_register_withprop(&tegra_dc_info);
}

device_init(tegra_display_register)
