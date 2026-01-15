# Slate Firmware

Multi-mode controller firmware for the Helios liquid fuel burner system.

## License

This project is licensed under the **GNU General Public License v2.0 or later** (GPL-2.0-or-later).

See the [LICENSE.md](LICENSE.md) file for the full license text.

## Serial Protocol

Slate communicates with Helios using the **Fusain Protocol v2.0** (binary packet protocol).

- **Protocol Specification:** `../../origin/docs/protocols/serial_protocol.md`
- **Library:** `../../modules/lib/fusain/` (Fusain protocol implementation)
- **Role:** Master controller (sends commands, receives telemetry)

## Display Icon Fonts

The display uses custom Font Awesome icons generated as 1bpp (1-bit-per-pixel) LVGL fonts optimized for the monochrome OLED display.

### Current Icons

All icons are contained in a single flattened font file: `src/display/icons_font.c`

| Icon | Unicode | Description |
|------|---------|-------------|
| 🌡️   | U+F769  | Thermometer (temperature display) |
| 🪭   | U+F863  | Fan (motor RPM display) |
| ⛽   | U+F52F  | Gas pump (pump frequency display) |

### Font Specifications

- **Size:** 12px
- **Format:** 1bpp (monochrome bitmap)
- **Source:** Font Awesome Solid (TTF)
- **Compression:** None (--no-compress for better rendering)

### Generating/Modifying the Icon Font

#### Prerequisites

1. **Font Awesome TTF file:** Place in `tmp/fa-solid-900.ttf`
   - Download from: https://fontawesome.com/
   - Or extract from WOFF2: `woff2_decompress /usr/share/fonts/WOFF2/fa-solid-900.woff2`

2. **lv_font_conv tool:** Install via npm
   ```bash
   npm install -g lv_font_conv
   ```

#### Regenerating the Font

To regenerate the existing font (thermometer, fan, gas pump):

```bash
cd /home/kazw/Projects/Thermoquad/apps/slate

# Generate the font
lv_font_conv --bpp 1 --size 12 --no-compress \
  --font tmp/fa-solid-900.ttf \
  --range 0xf52f,0xf769,0xf863 \
  --format lvgl \
  -o src/display/icons_font.c

# Fix the include path for Zephyr
sed -i 's|#include "lvgl/lvgl.h"|#include <lvgl.h>|g' src/display/icons_font.c
```

#### Adding New Icons

1. **Find the Unicode codepoint** for your desired Font Awesome icon:
   - Search at: https://fontawesome.com/icons
   - Example: "battery-full" is U+F240

2. **Add to the unicode range** in the generation command:
   ```bash
   lv_font_conv --bpp 1 --size 12 --no-compress \
     --font tmp/fa-solid-900.ttf \
     --range 0xf52f,0xf769,0xf863,0xf240 \  # Added battery icon
     --format lvgl \
     -o src/display/icons_font.c
   ```

3. **Define the UTF-8 macro** in `src/display/display.c`:
   ```c
   #define ICON_BATTERY "\xEF\x89\x80"  // UTF-8 encoding of U+F240
   ```

   To convert Unicode to UTF-8:
   - U+F240 → 0xEF 0x89 0x80
   - Use: `printf '\U0000F240' | xxd -p` or online converter

4. **Fix the include path** (required for Zephyr):
   ```bash
   sed -i 's|#include "lvgl/lvgl.h"|#include <lvgl.h>|g' src/display/icons_font.c
   ```

5. **Rebuild firmware:**
   ```bash
   task rebuild-firmware
   ```

#### Changing Icon Size

To change from 12px to a different size (e.g., 16px):

```bash
lv_font_conv --bpp 1 --size 16 --no-compress \
  --font tmp/fa-solid-900.ttf \
  --range 0xf52f,0xf769,0xf863 \
  --format lvgl \
  -o src/display/icons_font.c

# Fix include path
sed -i 's|#include "lvgl/lvgl.h"|#include <lvgl.h>|g' src/display/icons_font.c
```

**Note:** If you change the icon size, update the rotation pivot points in `display.c`:
```c
lv_obj_set_style_transform_pivot_x(icon_label, size/2, 0);  // Half of icon width
lv_obj_set_style_transform_pivot_y(icon_label, size/2, 0);  // Half of icon height
```

### Font File Structure

The generated `icons_font.c` contains:
- Glyph bitmap data (pixel data for each icon)
- Glyph descriptors (position, size, advance width)
- Character map (Unicode → glyph ID mapping)
- Font descriptor (exported as `icons_font`)

### Using Icons in Code

1. **Declare the font:**
   ```c
   LV_FONT_DECLARE(icons_font);
   ```

2. **Define UTF-8 macro:**
   ```c
   #define ICON_NAME "\xEF\x..."  // UTF-8 bytes
   ```

3. **Create label and set font:**
   ```c
   lv_obj_t* icon_label = lv_label_create(screen);
   lv_obj_set_style_text_font(icon_label, &icons_font, 0);
   lv_label_set_text(icon_label, ICON_NAME);
   ```

### Troubleshooting

**Icon doesn't appear:**
- Check that the UTF-8 encoding is correct
- Verify the Unicode codepoint is in the `--range` parameter
- Ensure the font file was regenerated after changes

**Build errors about lvgl.h:**
- The generated file uses `#include "lvgl/lvgl.h"` which doesn't work in Zephyr
- Always run the `sed` command to fix the include path after generation

**Icon looks wrong on monochrome display:**
- Use `--bpp 1` (not 2 or 4) for monochrome displays
- Use `--no-compress` for pixel-perfect rendering
- Font Awesome icons work well at 12px and above

**Icon is off-center when rotating:**
- Set the transform pivot to the center of the icon (width/2, height/2)
- For 12px icons: `lv_obj_set_style_transform_pivot_x/y(label, 6, 0)`

### Reference

- **lv_font_conv documentation:** https://github.com/lvgl/lv_font_conv
- **Font Awesome icons:** https://fontawesome.com/icons
- **LVGL fonts guide:** https://docs.lvgl.io/master/overview/font.html
