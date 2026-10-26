"""Turn a maze layout text file (# = wall, 0.5 m per character, top row = far z) into a Nav2 map.

Same convention as terrain_navigation.headless_sim: cell (0, 0) is centred on the ROS origin, ros_x = unity z,
ros_y = -unity x. The map frame is therefore the robot-0 start frame, as in the single-robot tests.
"""
import os

CHAR = 0.5
ORIGIN_SHIFT = 1.25
RESOLUTION = 0.05


def read_layout(path):
    return [r.rstrip('\n') for r in open(path) if not r.startswith('//') and r.strip()]


def layout_to_grid(rows):
    """Return (image, origin_x, origin_y): image[row][col] is 0 (wall) or 254 (free), row 0 = max ros_y."""
    n = len(rows)
    scale = int(round(CHAR / RESOLUTION))
    size = n * scale
    img = [[254] * size for _ in range(size)]
    for r, line in enumerate(rows):
        cz = n - 1 - r                       # layout top row = far z
        for cx, ch in enumerate(line):
            if ch != '#':
                continue
            col0 = cz * scale                # ros_x grows with z -> image column
            row0 = cx * scale                # ros_y = -x: larger x (cx) is lower in the image
            for dr in range(scale):
                for dc in range(scale):
                    img[row0 + dr][col0 + dc] = 0
    origin_x = -ORIGIN_SHIFT
    origin_y = ORIGIN_SHIFT - n * CHAR
    return img, origin_x, origin_y


def write_map(layout_path, out_prefix):
    """Writes <out_prefix>.pgm and <out_prefix>.yaml and returns the yaml path."""
    img, ox, oy = layout_to_grid(read_layout(layout_path))
    h, w = len(img), len(img[0])
    os.makedirs(os.path.dirname(out_prefix) or '.', exist_ok=True)
    with open(out_prefix + '.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        for row in img:
            f.write(bytes(row))
    with open(out_prefix + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(out_prefix)}.pgm\nmode: trinary\nresolution: {RESOLUTION}\n'
                f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return out_prefix + '.yaml'
