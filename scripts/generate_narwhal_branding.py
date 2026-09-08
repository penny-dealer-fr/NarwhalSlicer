#!/usr/bin/env python3
"""Regenerate Narwhal UI/package artwork from the supplied master PNG.

Requires Pillow (python -m pip install Pillow). Uses the repository's fonts;
no system fonts, SVG image embedding, or generative redraws are involved.
The supplied resources/Icon.icns keeps its original representations; missing
small 1x representations are filled from the PNG.
"""

import argparse
from io import BytesIO
from pathlib import Path
import struct

from PIL import Image, ImageDraw, ImageFont, ImageOps


ROOT = Path(__file__).resolve().parents[1]
IMAGES = ROOT / "resources/images"
MASTER = IMAGES / "NarwhalSlicer.png"
MSIX_SIZES = {
    "Square150x150Logo.png": 150,
    "Square44x44Logo.png": 44,
    "Square44x44Logo.targetsize-44_altform-unplated.png": 44,
    "StoreLogo.png": 50,
}
ICON_SIZES = (16, 24, 32, 48, 64, 128, 256)
SCALE = 3  # Layout bitmaps support 100%, 200%, and 300% display scaling.


def resized(image, size):
    # Pillow resamples RGBA in premultiplied-alpha space to avoid dark fringes.
    return image.resize((size, size), Image.Resampling.LANCZOS)


def save_png(image, path):
    image.save(path, optimize=True)


def font(size, bold=False):
    weight = "Bold" if bold else "Regular"
    return ImageFont.truetype(str(ROOT / f"resources/fonts/HarmonyOS_Sans_SC_{weight}.ttf"), size)


def label(image, text, xy, size, color, bold=False, anchor="lt"):
    ImageDraw.Draw(image).text(xy, text, font=font(size, bold), fill=color, anchor=anchor)


def icon_on(image, master, xy, size):
    image.alpha_composite(resized(master, size), xy)


def wordmark(master, kind, dark):
    width, height = (560, 125) if kind == "about" else (214, 80)
    canvas = Image.new("RGBA", (width * SCALE, height * SCALE))
    # About's right-hand area belongs to the live version/build labels.
    x, y, size, tx, ty, fs = (20, 15, 95, 132, 27, 27) if kind == "about" else (4, 6, 68, 84, 14, 24)
    icon_on(canvas, master, (x * SCALE, y * SCALE), size * SCALE)
    label(canvas, "Narwhal", (tx * SCALE, ty * SCALE), fs * SCALE, "#21C6CB" if dark else "#008F98", True)
    label(canvas, "Slicer", (tx * SCALE, (ty + fs + 6) * SCALE), fs * SCALE, "#F4F6F8" if dark else "#194B6B")
    return canvas


def splash(master, dark):
    canvas = Image.new("RGBA", (480 * SCALE, 480 * SCALE))
    icon_on(canvas, master, (150 * SCALE, 58 * SCALE), 180 * SCALE)
    label(canvas, "NarwhalSlicer", (240 * SCALE, 267 * SCALE), 32 * SCALE,
          "#F4F6F8" if dark else "#194B6B", True, "mt")
    # Keep everything below 70% clear for the native version and progress text.
    return canvas


def document(master, extension):
    canvas = Image.new("RGBA", (1024, 1024))
    draw = ImageDraw.Draw(canvas)
    # Folded page silhouette with a transparent outer margin.
    draw.rounded_rectangle((182, 66, 856, 980), radius=26, fill="#00000018")
    draw.polygon([(172, 52), (600, 52), (844, 296), (844, 964), (172, 964)], fill="#F5F6F7")
    draw.line([(172, 52), (600, 52), (844, 296), (844, 964), (172, 964), (172, 52)], fill="#D4D8DC", width=3)
    draw.polygon([(600, 52), (600, 296), (844, 296)], fill="#DEE3E7")
    draw.line([(600, 52), (600, 296), (844, 296)], fill="#C5CCD1", width=3)
    icon_on(canvas, master, (300, 320), 416)
    label(canvas, extension, (508, 810), 78 if len(extension) < 7 else 66, "#62717B", anchor="mt")
    return canvas


def save_icns(image, path):
    # PNG-backed ICNS chunks include standard and Retina representations.
    chunks = []
    for tag, size in ((b"icp4", 16), (b"icp5", 32), (b"icp6", 64),
                      (b"ic07", 128), (b"ic08", 256), (b"ic09", 512),
                      (b"ic10", 1024), (b"ic11", 32), (b"ic12", 64),
                      (b"ic13", 256), (b"ic14", 512)):
        output = BytesIO()
        resized(image, size).save(output, format="PNG", optimize=True)
        data = output.getvalue()
        chunks.append(tag + struct.pack(">I", len(data) + 8) + data)
    body = b"".join(chunks)
    path.write_bytes(b"icns" + struct.pack(">I", len(body) + 8) + body)


def complete_app_icns(master):
    path = ROOT / "resources/Icon.icns"
    data = path.read_bytes()
    if data[:4] != b"icns" or struct.unpack(">I", data[4:8])[0] != len(data):
        raise ValueError("Invalid supplied macOS app icon")
    tags = set()
    offset = 8
    while offset < len(data):
        tag, length = struct.unpack(">4sI", data[offset:offset + 8])
        if length < 8 or offset + length > len(data):
            raise ValueError("Invalid ICNS representation")
        tags.add(tag)
        offset += length
    body = data[8:]
    for tag, size in ((b"icp4", 16), (b"icp5", 32), (b"icp6", 64)):
        if tag not in tags:
            output = BytesIO()
            resized(master, size).save(output, format="PNG", optimize=True)
            frame = output.getvalue()
            body += tag + struct.pack(">I", len(frame) + 8) + frame
    path.write_bytes(b"icns" + struct.pack(">I", len(body) + 8) + body)


def bed_texture(master):
    # Match the original 512-unit bed canvas and bottom-center logo footprint.
    scale = 4
    canvas = Image.new("RGBA", (512 * scale, 512 * scale))
    draw = ImageDraw.Draw(canvas)
    for x, y, dx, dy in ((0, 0, 1, 1), (511, 0, -1, 1), (0, 511, 1, -1), (511, 511, -1, -1)):
        draw.line([(x * scale, (y + 19 * dy) * scale), (x * scale, y * scale),
                   ((x + 19 * dx) * scale, y * scale)], fill="#009789", width=scale)
    icon_on(canvas, master, (176 * scale, 441 * scale), 52 * scale)
    label(canvas, "Narwhal", (238 * scale, 447 * scale), 18 * scale, "#22C7CE", True)
    label(canvas, "Slicer", (238 * scale, 472 * scale), 16 * scale, "#E9E9E9")
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--msix-only", action="store_true")
    args = parser.parse_args()
    master = Image.open(MASTER).convert("RGBA")
    if master.size != (1000, 1000):
        raise ValueError("Expected the supplied 1000 x 1000 Narwhal master")
    for name, size in MSIX_SIZES.items():
        save_png(resized(master, size), ROOT / "scripts/msix/assets" / name)
    if args.msix_only:
        return

    complete_app_icns(master)
    for size in (16, 32, 48, 64, 128, 192, 256, 512):
        save_png(resized(master, size), IMAGES / f"NarwhalSlicer_{size}px.png")
    gray = ImageOps.grayscale(master).convert("RGBA")
    gray.putalpha(master.getchannel("A"))
    save_png(resized(gray, 192), IMAGES / "NarwhalSlicer_192px_grayscale.png")
    master.save(IMAGES / "NarwhalSlicer.ico", sizes=[(s, s) for s in ICON_SIZES], bitmap_format="bmp")

    for dark in (False, True):
        for kind in ("about", "horizontal"):
            suffix = "dark" if dark else "light"
            save_png(wordmark(master, kind, dark), IMAGES / f"NarwhalSlicer_{kind}_{suffix}.png")
        save_png(splash(master, dark), IMAGES / f"NarwhalSlicer_splash_{'dark' if dark else 'light'}.png")

    for name, extension in (("NarwhalSlicer", "PROJECT"), ("stl", "STL"), ("gcode", "GCODE")):
        image = document(master, extension)
        save_icns(image, IMAGES / f"{name}.icns")
        if name == "gcode":
            image.save(IMAGES / "NarwhalSlicer-gcodeviewer.ico", sizes=[(s, s) for s in ICON_SIZES], bitmap_format="bmp")
            save_png(resized(image, 192), IMAGES / "NarwhalSlicer-gcodeviewer_192px.png")

    # Web welcome assets use their existing URL contracts, with Retina headroom.
    save_png(resized(master, 308), ROOT / "resources/web/image/logo.png")
    portrait = Image.new("RGBA", (678, 812))
    icon_on(portrait, master, (0, 67), 678)
    save_png(portrait, ROOT / "resources/web/image/logo2.png")
    save_png(bed_texture(master), ROOT / "resources/profiles/Custom/narwhalslicer_bed_texture.png")
    print("Generated Narwhal UI, Windows/MSIX, Linux, and macOS document assets.")


if __name__ == "__main__":
    main()
