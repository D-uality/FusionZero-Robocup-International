import sys, pathlib
from collections import defaultdict

root = pathlib.Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(root))

import time
import cv2
import numpy as np
import math

from hardware.robot import *
from core.utilities import *

# Import the lookup table builders
from lookup_builder import (
    load_distance_lookup,
    load_rotational_patterns
)

class Grid:
    def __init__(self, width: int, height: int, resolution: int) -> None:
        self.real_width = width       # cm
        self.real_height = height     # cm
        self.resolution = resolution  # cm per cell

        # Grid dimensions
        self.width = width // resolution
        self.height = height // resolution
        self.cell_size = 5

        # Load precomputed distance lookup table
        print("Loading distance lookup table...")
        self.distance_lookup, self.lookup_metadata = (
            load_distance_lookup(width, height, resolution)
        )
        print(f"Lookup table loaded! Shape: {self.distance_lookup.shape}")

        # Load precomputed rotational patterns
        self.angle_resolution = 5  # degrees
        print("Loading rotational patterns...")
        (self.distance_patterns,
         self.pattern_to_positions) = load_rotational_patterns(
            width, height, resolution, angle_step=self.angle_resolution
        )
        print(f"Rotational patterns loaded! {len(self.pattern_to_positions)} unique hashes.")

        # Prepare display grid
        self.grid = [[(30, 30, 30) for _ in range(self.width)] for _ in range(self.height)]
        self.image = np.zeros((self.height * self.cell_size,
                               self.width * self.cell_size, 3), dtype=np.uint8)

    def _hash_distances(self, distances, tolerance=3.0):
        rounded = tuple(round(d / tolerance) * tolerance for d in distances)
        return rounded

    def _distances_match(self, computed, target, tolerance):
        return all(abs(c - t) < tolerance for c, t in zip(computed, target))

    def get_wall_distance(self, x, y, angle_deg):
        if 0 <= x < self.width and 0 <= y < self.height and 0 <= angle_deg < 360:
            return self.distance_lookup[x, y, angle_deg]
        return float('inf')

    def find_valid_positions(self, distances, known_bearing=None, tolerance=3.0):
        target_hash = self._hash_distances(distances, tolerance)

        # Fast lookup
        candidates = self.pattern_to_positions.get(target_hash, [])
        valid = []
        for x, y, angle in candidates:
            angle_idx = angle // self.angle_resolution
            computed = self.distance_patterns[(x, y)][angle_idx]
            if self._distances_match(computed, distances, tolerance):
                if known_bearing is None or abs((angle - known_bearing + 180) % 360 - 180) <= self.angle_resolution * 2:
                    valid.append((x, y, angle))
        return valid

    def check_distances(self, grid_x, grid_y, bearing, target_distances):
        left_angle = (bearing - 90) % 360
        front_angle = bearing
        right_angle = (bearing + 90) % 360

        ld = self.get_wall_distance(grid_x, grid_y, left_angle)
        fd = self.get_wall_distance(grid_x, grid_y, front_angle)
        rd = self.get_wall_distance(grid_x, grid_y, right_angle)

        tol = 3.0
        return (abs(ld - target_distances[0]) < tol and
                abs(fd - target_distances[1]) < tol and
                abs(rd - target_distances[2]) < tol)

    def visualize_positions(self, positions):
        # Reset grid
        for yy in range(self.height):
            for xx in range(self.width):
                self.grid[yy][xx] = (30, 30, 30)
        # Mark valid
        for x, y, _ in positions:
            if 0 <= x < self.width and 0 <= y < self.height:
                self.grid[y][x] = (0, 255, 0)

    def display(self):
        self.image[:] = 0
        for yy in range(self.height):
            for xx in range(self.width):
                col = self.grid[yy][xx]
                y0, x0 = yy * self.cell_size, xx * self.cell_size
                self.image[y0:y0 + self.cell_size, x0:x0 + self.cell_size] = col
        # draw lines
        for x in range(0, self.width * self.cell_size, self.cell_size):
            self.image[:, x] = 0
        for y in range(0, self.height * self.cell_size, self.cell_size):
            self.image[y, :] = 0

class Pose:
    def __init__(self, x: float, y: float, theta: float) -> None:
        self.x = x
        self.y = y
        self.theta = theta
    def __str__(self) -> str:
        return f"Pose(x={self.x}, y={self.y}, theta={self.theta})"
    def __repr__(self) -> str:
        return str(self)

if __name__ == "__main__":
    grid = Grid(120, 90, 2)

    frame_count = 0
    total_time = 0
    
    start_display()

    while True:
        t0 = time.time()
        image = evac_camera.capture()
        display_image = image.copy()

        distances = laser_sensors.read()
        if not all(distances):
            continue
        distances = [distances[0] + 7.5,
                     distances[1] + 11,
                     distances[2] + 7.5]

        print(f"Distances: {distances}")
        vals = grid.find_valid_positions(distances)

        dt = (time.time() - t0) * 1000
        total_time += dt
        frame_count += 1

        print(f"Found {len(vals)} valid positions in {dt:.1f}ms")
        if frame_count % 10 == 0:
            print(f"Average processing time: {total_time/frame_count:.1f}ms over {frame_count} frames")

        grid.visualize_positions(vals)
        grid.display()
        show(grid.image, "grid")