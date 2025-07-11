class RobotState():
    def __init__(self):
        # Constants
        self.debug = True
        self.timings = False

        # Variables
        self.debug_text = []
        self.count = {
            "uphill": 0,
            "downhill": 0,
            "tilt_left": 0,
            "tilt_right": 0,
            "red": 0,
            "silver": 0,
            "touch": 0
        }
         
        self.trigger = {
            "uphill": False,
            "downhill": False,
            "tilt_left": False,
            "tilt_right": False,
            "seasaw": False,
            "evacuation_zone": False
        }
        
        self.last_uphill = 0
        self.last_downhill = 1000
        self.silver_count = 0
        self.main_loop_count = 0