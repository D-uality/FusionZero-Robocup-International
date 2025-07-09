from hardware.laser_sensors import LaserSensors
from hardware.touch_sensors import TouchSensors
from hardware.motors import Motors
from hardware.camera import Camera
from hardware.colour_sensors import ColourSensors
from hardware.gyroscope_sensor import Gyroscope
from hardware.led import LED
from hardware.evacuation_camera import EvacuationCamera
from hardware.claw import Claw
from hardware.silver_sensor import SilverSensor
from hardware.oled_display import OLED_Display

oled_display = OLED_Display()

camera = Camera()
oled_display.text("Cam ✓", 0, 0, size=5)

motors = Motors(camera)
oled_display.text("Motors ✓", 0, 6, size=5)

laser_sensors = LaserSensors(motors)
oled_display.text("ToF ✓", 0, 12, size=5)

touch_sensors = TouchSensors()

colour_sensors = ColourSensors()
oled_display.text("CS ✓", 0, 18, size=5)

gyroscope = Gyroscope()
oled_display.text("IMU ✓", 0, 24, size=5)

led = LED()

evac_camera = EvacuationCamera()
oled_display.text("ECam ✓", 0, 30, size=5)

claw = Claw()
oled_display.text("Claw ✓", 0, 36, size=5)

silver_sensor = SilverSensor()