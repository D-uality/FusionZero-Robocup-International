from core.shared_imports import GPIO, time
start_time = time.perf_counter()

from core.listener import ModeListener
from core.utilities import debug, get_saved_frames, save_vfr_video, stop_display

import behaviours.line_follow as line_follow
import behaviours.evacuation_zone as evacuation_zone

from hardware.robot import *
from core.shared_imports import time, cv2, np, math
from behaviours.evacuation_zone import search
from lookup_builder import load_distance_lookup
start_display()

record = True

listener = ModeListener()

def main() -> None:
    motors.run(0, 0)
    led.off()
    start_time = time.perf_counter()

    try:
        listener.run()
        
        if known_bearing is not None:
            # Use known bearing from IMU - only check nearby angles for noise tolerance
            bearing_range = [known_bearing - 10, known_bearing - 5, known_bearing, 
                           known_bearing + 5, known_bearing + 10]
            bearing_range = [(b % 360) for b in bearing_range]  # Handle wrap-around
        else:
            # Full search if no IMU data
            bearing_range = range(0, 360, 10)
        
        for x in range(self.width):
            for y in range(self.height):
                for bearing in bearing_range:
                    # Use grid coordinates directly now
                    if self.check_distances(x, y, bearing, distances):
                        valid_positions.append((x, y, bearing))
        return valid_positions

    def check_distances(self, grid_x, grid_y, bearing, target_distances):
        """Fast distance checking using lookup table"""
        # Calculate sensor directions - FLIPPED left/right
        right_angle = (bearing + 90) % 360   # Right sensor
        front_angle = bearing                # Front sensor  
        left_angle = (bearing - 90) % 360    # Left sensor
        
        # Get distances from lookup table (much faster!)
        left_dist = self.get_wall_distance(grid_x, grid_y, left_angle)
        front_dist = self.get_wall_distance(grid_x, grid_y, front_angle)
        right_dist = self.get_wall_distance(grid_x, grid_y, right_angle)
        
        # Check if distances match (with tolerance)
        tolerance = 3.0  # cm
        return (abs(left_dist - target_distances[0]) < tolerance and
                abs(front_dist - target_distances[1]) < tolerance and
                abs(right_dist - target_distances[2]) < tolerance)

    def distance_to_wall(self, real_x, real_y, angle_rad):
        """Legacy method - kept for compatibility but not used with lookup table"""
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        
        # Find intersection with boundaries (0 ≤ x < real_width, 0 ≤ y < real_height)
        if abs(dx) < 1e-10:  # Essentially zero
            dist_x = float('inf')
        elif dx > 0:
            dist_x = (self.real_width - 1 - real_x) / dx
        else:
            dist_x = -real_x / dx
        
        if abs(dy) < 1e-10:  # Essentially zero
            dist_y = float('inf')
        elif dy > 0:
            dist_y = (self.real_height - 1 - real_y) / dy
        else:
            dist_y = -real_y / dy
        
        return min(abs(dist_x), abs(dist_y))
    
    def visualize_positions(self, positions):
        # Reset grid to default color
        for y in range(self.height):
            for x in range(self.width):
                self.grid[y][x] = (30, 30, 30)
        
        # Mark valid positions in green
        for x, y, bearing in positions:
            if 0 <= x < self.width and 0 <= y < self.height:
                self.grid[y][x] = (0, 255, 0)  # Green for valid positions
               
    def display(self) -> None:
        self.image = np.zeros((self.height * self.cell_size, self.width * self.cell_size, 3), dtype=np.uint8)
       
        # Fill the image up with the colour in the grid
        for y in range(self.height):
            for x in range(self.width):
                color = self.grid[y][x]
                y0 = y * self.cell_size
                x0 = x * self.cell_size
                self.image[y0:y0 + self.cell_size, x0:x0 + self.cell_size] = color
        # Draw grid lines
        for x in range(0, self.width * self.cell_size, self.cell_size):
            self.image[:, x, :] = 0  # Black vertical lines
        for y in range(0, self.height * self.cell_size, self.cell_size):
            self.image[y, :, :] = 0  # Black horizontal lines

    finally:
        motors.run(0, 0)
        stop_display()
        GPIO.cleanup()
        time.sleep(0.1)
        if record: save_vfr_video(get_saved_frames())

if __name__ == "__main__": main()