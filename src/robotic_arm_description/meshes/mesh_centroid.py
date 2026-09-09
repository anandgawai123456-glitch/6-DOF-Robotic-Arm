import struct
import glob
import numpy as np


def load_triangles(path):

    with open(path, "rb") as f:
        data = f.read()

    n = struct.unpack_from("<I", data, 80)[0]

    triangles = []

    offset = 84

    for _ in range(n):

        # Skip normal
        offset += 12

        v1 = np.array(struct.unpack_from("<fff", data, offset))
        offset += 12

        v2 = np.array(struct.unpack_from("<fff", data, offset))
        offset += 12

        v3 = np.array(struct.unpack_from("<fff", data, offset))
        offset += 12

        triangles.append((v1, v2, v3))

        # Skip attribute bytes
        offset += 2

    return triangles


for filename in sorted(glob.glob("*.stl")):

    triangles = load_triangles(filename)

    total_volume = 0.0
    centroid_sum = np.zeros(3)

    for a, b, c in triangles:

        # Signed tetrahedron volume
        volume = np.dot(a, np.cross(b, c)) / 6.0

        # Centroid of tetrahedron formed by origin + triangle
        centroid = (a + b + c) / 4.0

        total_volume += volume
        centroid_sum += volume * centroid

    if abs(total_volume) > 1e-12:
        centroid = centroid_sum / total_volume
    else:
        centroid = np.zeros(3)

    print("=" * 75)
    print(f"FILE: {filename}")
    print("-" * 75)

    print(f"VOLUME   : {abs(total_volume):.12f}")

    print(
        f"CENTROID : "
        f"X={centroid[0]: .9f} "
        f"Y={centroid[1]: .9f} "
        f"Z={centroid[2]: .9f}"
    )

    print(
        f"SCALED   : "
        f"X={centroid[0] * 10: .9f} "
        f"Y={centroid[1] * 10: .9f} "
        f"Z={centroid[2] * 10: .9f}"
    )
