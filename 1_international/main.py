import sys, pathlib
root = pathlib.Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(root))
from core.utilities import *
from hardware.robot import *
from core.shared_imports import time, cv2, np, math
from behaviours.evacuation_zone import search
from lookup_builder import load_distance_lookup
start_display()

class Grid():
    def __init__(self, width: int, height: int, resolution: int = 2) -> None:
        self.real_width = width  # Real world size in cm
        self.real_height = height
        self.resolution = resolution  # cm per grid cell
        
        # Grid dimensions based on resolution
        self.width = width // resolution
        self.height = height // resolution
        self.grid = [[(30, 30, 30) for _ in range(self.width)] for _ in range(self.height)]
       
        self.cell_size = 5
        self.image = np.zeros((self.height * self.cell_size, self.width * self.cell_size, 3), dtype=np.uint8)
        
        # Load pre-computed distance lookup table
        print("Loading distance lookup table...")
        self.distance_lookup, self.lookup_metadata = load_distance_lookup(width, height, resolution)
        print(f"Lookup table loaded! Shape: {self.distance_lookup.shape}")
    
    def get_wall_distance(self, x, y, angle_deg):
        """Get pre-computed distance from NumPy array (ultra-fast!)"""
        if (0 <= x < self.width and 0 <= y < self.height and 0 <= angle_deg < 360):
            return self.distance_lookup[x, y, angle_deg]
        return float('inf')
   
    def place_lasers(self, distances: list[float], noise_cm: float = 1) -> None:
        # Robot position is always correct
        l, f, r = distances
       
        if abs((l + r) - 90) < f:
            pass
    
    def find_valid_positions(self, distances, known_bearing=None):
        valid_positions = []
        
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

class Pose():
    def __init__(self, x: float, y: float, theta: float) -> None:
        self.x = x
        self.y = y
        self.theta = theta
    def __str__(self) -> str:
        return f"Pose(x={self.x}, y={self.y}, theta={self.theta})"
    def __repr__(self) -> str:
        return self.__str__()

class FeatureSLAM():
    def __init__(self):
        pass
   
    def find_triangles(self, image: np.ndarray, display_image: np.ndarray, grid: Grid) -> Grid:
        pass

if __name__ == "__main__":
    grid = Grid(120, 90, resolution=2)  # 2cm per cell = 60x45 grid
    model = FeatureSLAM()
   
    while True:
        image = evac_camera.capture()
        display_image = image.copy()
       
        distances = laser_sensors.read()
        distances = [distances[0] + 7.5, distances[1] + 11, distances[2] + 7.5]
        print(f"Distances: {distances}")
       
        # Find valid robot positions
        valid_positions = grid.find_valid_positions(distances)
        print(f"Found {len(valid_positions)} valid positions")
        
        # Visualize valid positions on grid
        grid.visualize_positions(valid_positions)
        grid.display()
       
        show(grid.image, "grid")
        
        # Optional: print first few valid positions
        if valid_positions:
            print("Sample positions:")
            for i, pos in enumerate(valid_positions[:5]):
                print(f"  Position {i+1}: x={pos[0]}, y={pos[1]}, bearing={pos[2]}°")
        
        time.sleep(0.1)  # Small delay to prevent overwhelming output