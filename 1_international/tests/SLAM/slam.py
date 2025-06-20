import sys, pathlib

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
    def __init__(self, width: int, height: int, resolution: int = 1) -> None:
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

        # EFFICIENT VISUALIZATION: Pre-create base image
        self.image_height = self.height * self.cell_size
        self.image_width = self.width * self.cell_size
        
        # Base image (solid background, no grid lines)
        self.base_image = np.full((self.image_height, self.image_width, 3), 30, dtype=np.uint8)
        
        # Working image for display (copy of base + dynamic content)
        self.image = self.base_image.copy()
        
        # Track previous positions for efficient clearing
        self.previous_positions = set()

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
        t_start = time.time()
        
        # Stage 1: Hash calculation
        t1 = time.time()
        target_hash = self._hash_distances(distances, tolerance)
        hash_time = (time.time() - t1) * 1000
        
        # Stage 2: Dictionary lookup
        t2 = time.time()
        candidates = self.pattern_to_positions.get(target_hash, [])
        lookup_time = (time.time() - t2) * 1000
        
        # Stage 3: Distance verification
        t3 = time.time()
        valid = []
        distance_checks = 0
        bearing_checks = 0
        
        for x, y, angle in candidates:
            angle_idx = angle // self.angle_resolution
            computed = self.distance_patterns[(x, y)][angle_idx]
            distance_checks += 1
            
            if self._distances_match(computed, distances, tolerance):
                bearing_checks += 1
                if known_bearing is None or abs((angle - known_bearing + 180) % 360 - 180) <= self.angle_resolution * 2:
                    valid.append((x, y, angle))
        
        verification_time = (time.time() - t3) * 1000
        total_time = (time.time() - t_start) * 1000
        
        # Print detailed timing breakdown
        print(f"  Hash: {hash_time:.2f}ms | Lookup: {lookup_time:.2f}ms | "
              f"Verify: {verification_time:.2f}ms | Total: {total_time:.2f}ms")
        print(f"  Candidates: {len(candidates)} | Distance checks: {distance_checks} | "
              f"Bearing checks: {bearing_checks} | Valid: {len(valid)}")
        
        return valid

    def visualize_positions_efficient(self, positions):
        """EFFICIENT: Only update changed cells"""
        current_positions = set((x, y) for x, y, _ in positions 
                               if 0 <= x < self.width and 0 <= y < self.height)
        
        # Clear previously marked positions that are no longer valid
        to_clear = self.previous_positions - current_positions
        for x, y in to_clear:
            y0, x0 = y * self.cell_size, x * self.cell_size
            # Restore original base image section
            self.image[y0:y0 + self.cell_size, x0:x0 + self.cell_size] = \
                self.base_image[y0:y0 + self.cell_size, x0:x0 + self.cell_size]
        
        # Mark new valid positions
        to_mark = current_positions - self.previous_positions
        for x, y in to_mark:
            y0, x0 = y * self.cell_size, x * self.cell_size
            self.image[y0:y0 + self.cell_size, x0:x0 + self.cell_size] = (0, 255, 0)
        
        # Update tracking
        self.previous_positions = current_positions

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
    grid = Grid(120, 90, resolution=1)

    frame_count = 0
    total_time = 0
    viz_time = 0
    
    start_display()

    while True:
        t0 = time.time()
        # image = evac_camera.capture()
        # display_image = image.copy()

        # Benchmark each stage
        t_sensor = time.time()
        distances = laser_sensors.read()
        if not all(distances):
            continue
        
        distances = [distances[0] + 7.7,
                     distances[1] + 11,
                     distances[2] + 7.7]
        sensor_time = (time.time() - t_sensor) * 1000

        print(f"Distances: {distances}")
        
        t_find = time.time()
        vals = grid.find_valid_positions(distances)
        find_time = (time.time() - t_find) * 1000

        # Benchmark visualization methods
        t_viz = time.time()
        
        # Use the efficient method
        grid.visualize_positions_efficient(vals)
        
        viz_dt = (time.time() - t_viz) * 1000
        viz_time += viz_dt

        dt = (time.time() - t0) * 1000
        total_time += dt
        frame_count += 1

        print(f"TIMING BREAKDOWN:")
        print(f"  Sensor read: {sensor_time:.2f}ms")
        print(f"  Position find: {find_time:.2f}ms")
        print(f"  Visualization: {viz_dt:.2f}ms")
        print(f"  Total frame: {dt:.1f}ms")
        print(f"  Found {len(vals)} valid positions")
        print("-" * 50)
        if frame_count % 10 == 0:
            print(f"Average total: {total_time/frame_count:.1f}ms, viz: {viz_time/frame_count:.1f}ms over {frame_count} frames")

        show(grid.image, "grid")