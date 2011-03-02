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
 * Tegra2 LCD emulation display templates
 */

#if DEPTH == 8
# define BPP 1
# define PIXEL_TYPE uint8_t
#elif DEPTH == 15 || DEPTH == 16
# define BPP 2
# define PIXEL_TYPE uint16_t
#elif DEPTH == 24
# define BPP 3
# define PIXEL_TYPE uint8_t
#elif DEPTH == 32
# define BPP 4
# define PIXEL_TYPE uint32_t
#else
# error unsupported depth
#endif

/*
 * RGB 565
 */
static void glue(tegra_draw_line16_, DEPTH)(void *opaque,
                uint8_t *dst, const uint8_t *src, int width, int pitch)
{
#if DEPTH == 16 && !defined(BSWAP_NEEDED)
    memcpy(dst, src, width * 2);
#else
    uint16_t val;
    uint8_t r, g, b;

    while (width) {
        val = lduw_raw((void *) src);
        r = (val >> 8) & 0xf8;
        g = (val >> 3) & 0xfc;
        b = (val << 3) & 0xf8;
        ((PIXEL_TYPE *) dst)[0] = glue(rgb_to_pixel, DEPTH)(r, g, b);
        src += 2;
        dst += BPP;
        width--;
    }
#endif
}

static drawfn glue(tegra_draw_fn_, DEPTH)[32] = {
    /* P1 : 1-bpp palettized mode */
    [0] = NULL,
    /* P2 : 2-bpp palettized mode */
    [1] = NULL,
    /* P4 : 4-bpp palettized mode */
    [2] = NULL,
    /* P8 : 8-bpp palettized mode */
    [3] = NULL,
    /* B4G4R4A4*/
    [4] = NULL,
    /* B5G5R5A */
    [5] = NULL,
    /* B5G6R5 : 16-bpp / RGB-565 mode */
    [6] = (drawfn) glue(tegra_draw_line16_, DEPTH),
    /* AB5G5R5 */
    [7] = NULL,
    /* Not specified */
    [8 ... 11] = NULL,
    /* B8G8R8A8 */
    [12] = NULL,
    /* R8G8B8A8 */
    [13] = NULL,
    /* B6x2G6x2R6x2A8 */
    [14] = NULL,
    /* R6x2G6x2B6x2A8 */
    [15] = NULL,
    /* YUV and YCbCr modes */
    [16 ... 25] = NULL,
    /* Not specified */
    [26 ... 31] = NULL,
};

#undef DEPTH
#undef BPP
#undef PIXEL_TYPE
