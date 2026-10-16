#!/usr/bin/env python3
"""
Generate STag marker PNG textures for Gazebo.

Python port of the reference STag marker generator
(https://github.com/ManfredStoiber/stag, ref/marker_generator/Form1.cs, MIT License).
Codeword tables HD15.txt / HD17.txt are copied from the same directory into
scripts/stag_codes/; other HD libraries can be passed via --codes.

Usage:
  python3 scripts/generate_stag_textures.py --hd 15 --ids 0-17 --prefix hd15_ \
      --out gazebo_objects/STag_marker_HD15/materials/textures
  python3 scripts/generate_stag_textures.py --hd 17 --ids 0-5 --prefix hd17_ \
      --out gazebo_objects/STag_marker_HD17/materials/textures

Output: 1000x1000 px PNG per marker, same layout as the official STag markers
(white margin, black square, white circle with 48 code bits, ID label in the border).
"""

import argparse
import math
import pathlib
import sys

import cv2
import numpy as np

NO_OF_BITS = 48
BORDER = 0.125
OUTER_CIRCLE_RADIUS = 0.4
INNER_CIRCLE_RADIUS = 0.35
CODE_RADIUS = 0.062482177287080
FILLER_CODE_RADIUS = 0.7
FILE_SIZE = 1000

MARKER_SIZE = FILE_SIZE / (1 + BORDER * 2)
BORDER_SIZE = MARKER_SIZE * BORDER
OUTER_D = 2 * MARKER_SIZE * OUTER_CIRCLE_RADIUS
INNER_D = 2 * MARKER_SIZE * INNER_CIRCLE_RADIUS
OUTER_TL = (FILE_SIZE - OUTER_D) / 2
INNER_TL = (FILE_SIZE - INNER_D) / 2
CODE_D = 2 * INNER_D * CODE_RADIUS
FILLER_D = CODE_D * FILLER_CODE_RADIUS

# Drawing with sub-pixel precision: cv2 takes integer coordinates scaled by 2^SHIFT
SHIFT = 4
SCALE = 1 << SHIFT


def polar_to_cart(radius, radians):
    return (0.5 + math.cos(radians) * radius, 0.5 - math.sin(radians) * radius)


def code_locations():
    half_pi = math.pi / 2
    locs = []
    for i in range(4):
        q = i * half_pi
        locs += [
            polar_to_cart(0.088363142525988, 0.785398163397448 + q),
            polar_to_cart(0.206935928182607, 0.459275804122858 + q),
            polar_to_cart(0.206935928182607, half_pi - 0.459275804122858 + q),
            polar_to_cart(0.313672146827381, 0.200579720495241 + q),
            polar_to_cart(0.327493143484516, 0.591687617505840 + q),
            polar_to_cart(0.327493143484516, half_pi - 0.591687617505840 + q),
            polar_to_cart(0.313672146827381, half_pi - 0.200579720495241 + q),
            polar_to_cart(0.437421957035861, 0.145724938287167 + q),
            polar_to_cart(0.437226762361658, 0.433363129825345 + q),
            polar_to_cart(0.430628029742607, 0.785398163397448 + q),
            polar_to_cart(0.437226762361658, half_pi - 0.433363129825345 + q),
            polar_to_cart(0.437421957035861, half_pi - 0.145724938287167 + q),
        ]
    return locs


LOCS = code_locations()
NEARBY = [
    [j for j in range(NO_OF_BITS) if j != i and math.dist(LOCS[i], LOCS[j]) < CODE_RADIUS * 4]
    for i in range(NO_OF_BITS)
]


def fill_circle(img, cx, cy, diameter, color, antialias):
    center = (int(round(cx * SCALE)), int(round(cy * SCALE)))
    radius = int(round(diameter / 2 * SCALE))
    line_type = cv2.LINE_AA if antialias else cv2.LINE_8
    cv2.circle(img, center, radius, color, -1, line_type, SHIFT)


def bit_center(j):
    return INNER_TL + INNER_D * LOCS[j][0], INNER_TL + INNER_D * LOCS[j][1]


def draw_code_circles(img, code, antialias):
    for j in range(NO_OF_BITS):
        if code[j] == 1:
            fill_circle(img, *bit_center(j), CODE_D, 0, antialias)
    for j in range(NO_OF_BITS):
        if code[j] == 0:
            fill_circle(img, *bit_center(j), CODE_D, 255, antialias)


def ball_kernel(r):
    y, x = np.mgrid[-r:r + 1, -r:r + 1]
    k = (x * x + y * y <= r * r).astype(np.float32)
    k[r, r] = 0.0
    return k


KERNEL = ball_kernel(12)
KERNEL_HALF = KERNEL.sum() / 2


def majority_smooth(img, iterations=5):
    """Dilate/erode pair from Form1.cs: a pixel flips if most of its ball neighbours differ."""
    lo = int(OUTER_TL)
    hi = FILE_SIZE - lo
    window = np.zeros(img.shape, bool)
    window[lo:hi, lo:hi] = True
    for _ in range(iterations):
        not_white = (img != 255).astype(np.float32)
        count = cv2.filter2D(not_white, -1, KERNEL, borderType=cv2.BORDER_REPLICATE)
        img[window & (img != 0) & (count > KERNEL_HALF)] = 0

        not_black = (img != 0).astype(np.float32)
        count = cv2.filter2D(not_black, -1, KERNEL, borderType=cv2.BORDER_REPLICATE)
        img[window & (img != 255) & (count > KERNEL_HALF)] = 255


def draw_rotated_label(img, text, center, height_px):
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = max(2, int(height_px / 8))
    scale = cv2.getFontScaleFromHeight(font, int(height_px), thickness)
    (w, h), base = cv2.getTextSize(text, font, scale, thickness)
    pad = thickness * 2
    canvas = np.zeros((h + base + 2 * pad, w + 2 * pad), np.uint8)
    cv2.putText(canvas, text, (pad, pad + h), font, scale, 255, thickness, cv2.LINE_AA)

    # Form1.cs rotates by 315 deg clockwise (screen coords) = 45 deg counter-clockwise
    ch, cw = canvas.shape
    m = cv2.getRotationMatrix2D((cw / 2, ch / 2), 45, 1.0)
    cos, sin = abs(m[0, 0]), abs(m[0, 1])
    nw, nh = int(ch * sin + cw * cos) + 2, int(ch * cos + cw * sin) + 2
    m[0, 2] += nw / 2 - cw / 2
    m[1, 2] += nh / 2 - ch / 2
    rotated = cv2.warpAffine(canvas, m, (nw, nh))

    x0 = int(round(center[0] - nw / 2))
    y0 = int(round(center[1] - nh / 2))
    roi = img[y0:y0 + nh, x0:x0 + nw]
    np.maximum(roi, rotated, out=roi)


def render_marker(code, index_str, hd):
    img = np.full((FILE_SIZE, FILE_SIZE), 255, np.uint8)
    b0 = int(round(BORDER_SIZE))
    b1 = int(round(BORDER_SIZE + MARKER_SIZE))
    img[b0:b1, b0:b1] = 0
    fill_circle(img, FILE_SIZE / 2, FILE_SIZE / 2, OUTER_D, 255, antialias=False)

    draw_code_circles(img, code, antialias=False)
    for j in range(NO_OF_BITS):
        for k in NEARBY[j]:
            if k > j and code[j] == 1 and code[k] == 1:
                cx = INNER_TL + INNER_D * (LOCS[j][0] + LOCS[k][0]) / 2
                cy = INNER_TL + INNER_D * (LOCS[j][1] + LOCS[k][1]) / 2
                fill_circle(img, cx, cy, FILLER_D, 0, antialias=False)
    for j in range(NO_OF_BITS):
        if code[j] == 0:
            fill_circle(img, *bit_center(j), CODE_D, 255, antialias=False)

    majority_smooth(img)
    draw_code_circles(img, code, antialias=True)

    # clear the ring between the code area and the outer circle
    yy, xx = np.mgrid[0:FILE_SIZE, 0:FILE_SIZE]
    dist = np.hypot(xx + 0.5 - FILE_SIZE / 2, yy + 0.5 - FILE_SIZE / 2)
    img[(dist <= OUTER_D / 2) & (dist > INNER_D / 2 + 1)] = 255
    edge = (dist > OUTER_D / 2 - 1) & (dist < OUTER_D / 2 + 1)
    img[edge] = np.clip((OUTER_D / 2 + 1 - dist[edge]) / 2 * 255, 0, 255).astype(np.uint8)

    draw_rotated_label(img, index_str, (2.375 * BORDER_SIZE, 2.375 * BORDER_SIZE), 0.7 * BORDER_SIZE)
    draw_rotated_label(img, f"HD{hd}", (1.875 * BORDER_SIZE, 1.875 * BORDER_SIZE), 0.2 * BORDER_SIZE)
    return img


def read_codes(path):
    codes = []
    for line in pathlib.Path(path).read_text(encoding="utf-8-sig").splitlines():
        line = line.strip()
        if line:
            codes.append([int(c) for c in line[:NO_OF_BITS]])
    return codes


def parse_ids(spec, count):
    ids = []
    for part in spec.split(","):
        if "-" in part:
            a, b = part.split("-")
            ids += range(int(a), int(b) + 1)
        else:
            ids.append(int(part))
    bad = [i for i in ids if not 0 <= i < count]
    if bad:
        sys.exit(f"IDs out of range 0..{count - 1}: {bad}")
    return ids


def main():
    ap = argparse.ArgumentParser(description="Generate STag marker PNG textures")
    ap.add_argument("--hd", type=int, required=True, help="HD library: 11, 13, ..., 23")
    ap.add_argument("--ids", required=True, help="Marker IDs, e.g. 0-17 or 0,5,7")
    ap.add_argument("--out", type=pathlib.Path, required=True, help="Output directory")
    ap.add_argument("--prefix", default="", help="File name prefix, e.g. hd15_")
    ap.add_argument(
        "--codes",
        type=pathlib.Path,
        help="Codeword table (default: scripts/stag_codes/HD<hd>.txt)",
    )
    args = ap.parse_args()

    codes_path = args.codes or pathlib.Path(__file__).parent / "stag_codes" / f"HD{args.hd}.txt"
    codes = read_codes(codes_path)
    args.out.mkdir(parents=True, exist_ok=True)

    for i in parse_ids(args.ids, len(codes)):
        index_str = str(i).zfill(len(str(len(codes))))
        path = args.out / f"{args.prefix}{i:05d}.png"
        cv2.imwrite(str(path), render_marker(codes[i], index_str, args.hd))
        print(path)


if __name__ == "__main__":
    main()
