# Counting variables
main_loop_count = 0

touch_count = 0
last_uphill = 0

uphill_count   = 0
downhill_count = 0

silver_count = 0
red_count    = 0
gap_count    = 0

# Modifier Triggers
seasaw_trigger = False

uphill_trigger = downhill_trigger = False
tilt_left_trigger = tilt_right_trigger = False

evac_trigger = False
gap_trigger = False
evac_exited = False
camera_enable = False



def touch_check(touch_values: list[int]) -> bool:
    global touch_count
    
    # If a touch sensor is triggered
    if sum(touch_values) != 2:
        touch_count += 1
    else:
        touch_count = 0

    # Determine if obstacle has been detected    
    if touch_count >= 200:
        touch_count = 0
        return True
    
    else:
        return False
    
def silver_check(ir_integral: int, colour_values: list[int]) -> bool:
    global main_loop_count
    global silver_count
    
    # Only update silver_count based on whether we've been going straight
    # Eliminates accidentally triggering on speed_bumps
    if ir_integral >= 500 or main_loop_count <= 100: return False
    
    # Count the number of colour_values from the front_sensors that exceed the silver threshold
    silver_delta = sum(list(1 if colour_values[i] > 130 else 0 for i in [0, 1, 3, 4]))
    
    # Upddate silver_count acoordingly
    if silver_delta != 0:
        silver_count += silver_delta
    else:
        silver_count = 0
        
    # Determine if evacuation_zone has been found    
    if silver_count > 30:
        silver_count = 0
        return True
    
    else:
        return False

def red_check(colour_values, red_count):
    global red_threshold, main_loop_count
    
    if main_loop_count < 50: return red_count

    if (red_threshold[0][0] < colour_values[5] < red_threshold[0][1] or red_threshold[1][0] < colour_values[6] < red_threshold[1][1]) and all(colour_values[i] > 70 and colour_values[i] < 120 for i in range(5)):
        red_count += 1
    else: 
        red_count = 0
    
    return red_count

def red_check(ir_integral: int, colour_values: list[int]):
    global main_loop_count
    global red_count
    
    # Only update silver_count based on whether we've been going straight
    # Eliminates accidentally triggering on speed_bumps
    if ir_integral >= 500 or main_loop_count <= 100: return False

    # Check whether back sensors see within a valid red_threshold
    
    
def seasaw_check() -> bool:
    global main_loop_count
    global uphill_trigger, downhill_trigger
    global last_uphill, seasaw_trigger
    
    # Update last_uphill depending, counting the number of cycles past the the last uphill cycle
    last_uphill = 0 if uphill_trigger else last_uphill + 1
    
    if downhill_trigger and last_uphill <= 40:
        # Update last_uphill so it doesn't trigger again
        last_uphill = 41
        main_loop_count = 0
        
        # Seasaw modifier is now triggered
        seasaw_trigger = True
        
    else:
        seasaw_trigger = False
    
    return seasaw_trigger