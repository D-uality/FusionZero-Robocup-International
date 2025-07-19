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
oled_display.text("Cam ✓", 0, 0)

motors = Motors(camera)
oled_display.text("Motor ✓", 0, 15)

laser_sensors = LaserSensors(motors)
oled_display.text("ToF ✓", 0, 30)

touch_sensors = TouchSensors()

colour_sensors = ColourSensors()
oled_display.text("CS ✓", 0, 45)

gyroscope = Gyroscope()
oled_display.text("IMU ✓", 70, 0)

led = LED()

evac_camera = EvacuationCamera()
oled_display.text("ECam ✓", 70, 15)

claw = Claw()
oled_display.text("Claw ✓", 70, 30)

silver_sensor = SilverSensor()