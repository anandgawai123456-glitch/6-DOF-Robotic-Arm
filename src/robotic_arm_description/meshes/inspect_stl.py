import struct
import glob
import numpy as np


def load_stl(path):
    with open(path, "rb") as f:
        data = f.read()

    # Number of triangles
    n = struct.unpack_from("<I", data, 80)[0]

    vertices = np.empty((n * 3, 3), dtype=np.float64)

    offset = 84

    for i in range(n):
        # Skip normal (12 bytes)
        offset += 12

        # Read 3 vertices
        for j in range(3):
            vertices[i * 3 + j] = struct.unpack_from(
                "<fff",
                data,
                offset
            )
            offset += 12

        # Skip attribute byte count
        offset += 2

    return vertices


for filename in sorted(glob.glob("*.stl")):

    try:
        vertices = load_stl(filename)

        minimum = vertices.min(axis=0)
        maximum = vertices.max(axis=0)

        center = (minimum + maximum) / 2.0
        size = maximum - minimum

        print("=" * 75)
        print(f"FILE: {filename}")
        print("-" * 75)

        print(
            f"MIN    : "
            f"X={minimum[0]: .9f} "
            f"Y={minimum[1]: .9f} "
            f"Z={minimum[2]: .9f}"
        )

        print(
            f"MAX    : "
            f"X={maximum[0]: .9f} "
            f"Y={maximum[1]: .9f} "
            f"Z={maximum[2]: .9f}"
        )

        print(
            f"CENTER : "
            f"X={center[0]: .9f} "
            f"Y={center[1]: .9f} "
            f"Z={center[2]: .9f}"
        )

        print(
            f"SIZE   : "
            f"X={size[0]: .9f} "
            f"Y={size[1]: .9f} "
            f"Z={size[2]: .9f}"
        )

        print(f"TRIANGLES: {len(vertices) // 3}")

    except Exception as e:
        print(f"ERROR reading {filename}: {e}")
