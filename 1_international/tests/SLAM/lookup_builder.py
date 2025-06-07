import numpy as np
import math
import pickle
import os


def build_distance_lookup(real_width, real_height, resolution):
    width = real_width // resolution
    height = real_height // resolution
    lookup = np.zeros((width, height, 360), dtype=np.float32)

    total_dist = width * height * 360
    completed_dist = 0
    print(f"Building lookup table for {width}x{height} grid with 360° resolution...")

    for x in range(width):
        for y in range(height):
            real_x = x * resolution
            real_y = y * resolution
            for angle in range(360):
                rad = math.radians(angle)
                lookup[x, y, angle] = calculate_wall_distance(real_x, real_y, rad, real_width, real_height)
                completed_dist += 1
                if completed_dist % 50000 == 0:
                    pct = (completed_dist / total_dist) * 100
                    print(f"Distance lookup progress: {pct:.1f}% ({completed_dist}/{total_dist})")

    filename = f"distance_lookup_{real_width}x{real_height}_res{resolution}.npy"
    np.save(filename, lookup)
    with open(f"{filename[:-4]}_meta.pkl", 'wb') as f:
        pickle.dump({
            'real_width': real_width,
            'real_height': real_height,
            'resolution': resolution,
            'shape': lookup.shape
        }, f)
    print(f"Lookup table saved to {filename}")
    return lookup


def build_rotational_patterns(real_width, real_height, resolution, angle_step=5, tolerance=3.0):
    width = real_width // resolution
    height = real_height // resolution

    # Ensure distance lookup is available
    distance_lookup, _ = load_distance_lookup(real_width, real_height, resolution)

    patterns = {}
    index = {}

    total_patterns = width * height * (360 // angle_step)
    completed_pat = 0
    print(f"Building rotational patterns for {width}x{height} grid at {angle_step}° steps...")

    for x in range(width):
        for y in range(height):
            pattern = []
            for ang in range(0, 360, angle_step):
                left = (ang - 90) % 360
                right = (ang + 90) % 360

                d = (
                    distance_lookup[x, y, left],
                    distance_lookup[x, y, ang],
                    distance_lookup[x, y, right]
                )
                pattern.append(d)

                # Progress watcher
                completed_pat += 1
                if completed_pat % 50000 == 0:
                    pct = (completed_pat / total_patterns) * 100
                    print(f"Rotational patterns progress: {pct:.1f}% ({completed_pat}/{total_patterns})")

            patterns[(x, y)] = pattern
            for i, d in enumerate(pattern):
                key = tuple(round(val / tolerance) * tolerance for val in d)
                index.setdefault(key, []).append((x, y, i * angle_step))

    # Save patterns and index
    p_fn = f"rotational_patterns_{real_width}x{real_height}_res{resolution}_ang{angle_step}.pkl"
    with open(p_fn, 'wb') as f:
        pickle.dump({'patterns': patterns, 'index': index}, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"Rotational patterns saved to {p_fn}")
    return patterns, index


def load_distance_lookup(real_width, real_height, resolution):
    fn = f"distance_lookup_{real_width}x{real_height}_res{resolution}.npy"
    meta = f"{fn[:-4]}_meta.pkl"
    if os.path.exists(fn) and os.path.exists(meta):
        print(f"Loading existing lookup table from {fn}")
        arr = np.load(fn)
        with open(meta, 'rb') as f:
            m = pickle.load(f)
        print(f"Loaded lookup table: {arr.shape}")
        return arr, m
    print("Lookup table not found, building new one...")
    return build_distance_lookup(real_width, real_height, resolution)


def load_rotational_patterns(real_width, real_height, resolution, angle_step=5):
    fn = f"rotational_patterns_{real_width}x{real_height}_res{resolution}_ang{angle_step}.pkl"
    if os.path.exists(fn):
        print(f"Loading existing rotational patterns from {fn}")
        with open(fn, 'rb') as f:
            data = pickle.load(f)
        print(f"Loaded patterns: {len(data['index'])} unique hashes")
        return data['patterns'], data['index']
    print("Rotational patterns not found, building new one...")
    return build_rotational_patterns(real_width, real_height, resolution, angle_step)


def calculate_wall_distance(rx, ry, rad, rw, rh):
    dx = math.cos(rad)
    dy = math.sin(rad)
    if abs(dx) < 1e-10:
        dist_x = float('inf')
    elif dx > 0:
        dist_x = (rw - 1 - rx) / dx
    else:
        dist_x = -rx / dx
    if abs(dy) < 1e-10:
        dist_y = float('inf')
    elif dy > 0:
        dist_y = (rh - 1 - ry) / dy
    else:
        dist_y = -ry / dy
    return min(abs(dist_x), abs(dist_y))


if __name__ == "__main__":
    RW, RH, R = 120, 90, 1
    build_distance_lookup(RW, RH, R)
    build_rotational_patterns(RW, RH, R)
