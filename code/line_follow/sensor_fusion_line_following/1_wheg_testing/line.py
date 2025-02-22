import time
from main import listener
from typing import Optional

import config
import colour
import motors
import camera
import cv2

integral, derivative, last_error = 0, 0, 0
main_loop_count = 0
min_green_loop_count = 50
running_error = [0 for _ in range(0, 25)]

def follow_line() -> None:
	while True:
		global main_loop_count    
		if listener.get_mode() != 1: break

		if main_loop_count % 10 == 0:
			image = camera.capture_array()
			image = camera.perspective_transform(image)

			if config.X11: cv2.imshow("Line Follow", image)
		
		colour_values = colour.read()
		green_signal = green_check(colour_values)

		if len(green_signal) != 0 and main_loop_count > min_green_loop_count: 
			intersection_handling(green_signal, colour_values)
		else:
			PID(colour_values, 0.7, 0.004, 0.5)

		main_loop_count = main_loop_count + 1 if main_loop_count < 2**31 - 1 else 0
		
		print()

def PID(colour_values: list[int], kP: float, kI: float, kD: float) -> None:
	global main_loop_count, last_green_loop, running_error
	global integral, derivative, last_error
	outer_error = config.outer_multi * (colour_values[0] - colour_values[4])
	inner_error = config.inner_multi * (colour_values[1] - colour_values[3])
 
	total_error = outer_error + inner_error
	running_error[main_loop_count % 25] = total_error

	if abs(total_error) < 10: integral = 0
	elif integral < -750: integral = -750
	elif integral >  750: integral = 750
	else: integral += total_error
	derivative = total_error - last_error

	turn = total_error * kP + integral * kI + derivative * kD
	v1, v2 = config.line_base_speed + turn, config.line_base_speed - turn

	motors.run(v1, v2)
	last_error = total_error

	config.update_log([f"PID", f"{main_loop_count}", ", ".join(list(map(str, colour_values))), f"{total_error:.2f} {integral:.2f} {derivative:.2f}", f"{v1:.2f} {v2:.2f}"], [24, 8, 30, 16, 10])

def green_check(colour_values: list[int]) -> str:
	global running_error
	running_error_average = sum(running_error) / len(running_error)
	signal = ""    

	if colour_values[0] <= 15 and colour_values[1] <= 15 and colour_values[3] <= 15 and colour_values[4] <= 15 and colour_values[2] <= 15:
		print(sum(running_error) / len(running_error))
		if colour_values[5] >= 30 and colour_values[6] >= 30: pass
		else:
			print("Checking +")
			
			if colour_values[5] <= 50 and colour_values[6] <= 50: signal = "+ double"
			elif colour_values[5] <= 50: signal = "+ left"
			elif colour_values[6] <= 50: signal = "+ right"
			else: pass

	else:
		outer_values = [colour_values[0], colour_values[4]]
		outer_values.sort()      
		inner_values = [colour_values[1], colour_values[3]]
		inner_values.sort()
  
		outer_left_enable  = True if outer_values[0] == colour_values[0] and outer_values[0] < 15 else False
		outer_right_enable = True if outer_values[0] == colour_values[4] and outer_values[0] < 15 else False
		inner_left_enable  = True if inner_values[0] == colour_values[1] and inner_values[0] < 0 else False
		inner_right_enable = True if inner_values[0] == colour_values[3] and inner_values[0] < 0 else False
		
		if (outer_left_enable or outer_right_enable or inner_left_enable or inner_right_enable) and colour_values[2] >= 40 and colour_values[4] <= 30 and colour_values[0] <= 30:   
			difference = (colour_values[0] + colour_values[1]) - (colour_values[3] + colour_values[4])
			if abs(difference) <= 3:
				signal = ""
			else:
				print("checking")
				signal = "|-" if (colour_values[0] + colour_values[1]) >= (colour_values[3] + colour_values[4]) else "-|"
				real_fake = " fake" if colour_values[5] + colour_values[6] >= 134 else " real"
   
				signal += real_fake
			
	if len(signal) != 0:
		config.update_log(["GREEN CHECK", ", ".join(list(map(str, colour_values)))], [24, 30])
		print()
	
	return signal
		
def intersection_handling(signal: str, colour_values) -> None:
	global main_loop_count
	main_loop_count = 0
	
	# print(running_error)
	# print(sum(running_error) / len(running_error))
	config.update_log([f"INTERSECTION HANDLING", f"{signal}"], [24, 16])
	print()
	
	motors.pause()

# def green_check(colour_values: list[int]) -> str:
#     signal = ""
	
#     error = (colour_values[0] + colour_values[1]) - (colour_values[3] + colour_values[4])
	
#     T_left_enable  = True if error < 0 else False
#     T_right_enable = not T_left_enable
	
#     if colour_values[2] <= 50 or abs(error) <= 10: T_left_enable = T_right_enable = False

#     if colour_values[0] <= 15 and colour_values[1] <= 15 and colour_values[3] <= 15 and colour_values[4] <= 15:        
#         if   colour_values[5] <= 23 and colour_values[6] <= 23: signal += "+ double"
#         elif colour_values[5] <= 23 and colour_values[6] >= 65: signal += "+ left"
#         elif colour_values[6] <= 23 and colour_values[5] >= 65: signal += "+ right"
#         elif colour_values[5] >= 75 and colour_values[6] >= 75 and colour_values[2] <= 10: signal += "+ straight"
#         else:                                                   pass
	
#     if colour_values[0] <= 30 and colour_values[1] <= 15 and T_left_enable:
#         if colour_values[5] <= 35 and colour_values[3] <= 15 and colour_values[6] >= colour_values[5] + 10:
#             signal = "T left"
#         elif colour_values[4] <= 15 and colour_values[5] >= 50 and colour_values[6] >= 50:
#             signal = "fake T left"
#         else: pass
		
#     elif colour_values[4] <= 30 and colour_values[3] <= 15 and T_right_enable:
#         if colour_values[6] <= 35 and colour_values[1] <= 15 and colour_values[5] >= colour_values[6] + 10:
#             signal = "T right"
#         elif colour_values[0] <= 15 and colour_values[5] >= 50 and colour_values[6] >= 50:
#             signal = "fake T right"
#         else: pass
		
#     if len(signal) != 0:
#         config.update_log(["GREEN CHECK", ", ".join(list(map(str, colour_values)))], [24, 30])
#         print()
#     return signal

# def intersection_handling(signal: str, colour_values) -> None:
#     global main_loop_count
#     main_loop_count = 0

#     motors.run(0, 0, 0.2)

#     config.update_log([f"INTERSECTION HANDLING", f"{signal}"], [24, 16])
#     print()
	
#     if signal == "fake T left":
#         motors.run(config.line_base_speed * 1.3, -config.line_base_speed * 0.7, 1.5)
#         motors.run(config.line_base_speed, config.line_base_speed, 0.15)

#     elif signal == "fake T right":
#         motors.run(-config.line_base_speed * 0.7, config.line_base_speed * 1.3, 1.5)
#         motors.run(config.line_base_speed, config.line_base_speed, 0.15)

#     elif signal == "T left":
#         motors.run(-config.line_base_speed, config.line_base_speed, 0.8)
#         motors.run( config.line_base_speed, config.line_base_speed, 0.6)
	
#     elif signal == "T right":
#         motors.run(config.line_base_speed, -config.line_base_speed, 0.8)
#         motors.run(config.line_base_speed,  config.line_base_speed, 0.6)

#     elif signal == "+ left":
#         v1, v2 = -config.line_base_speed * 0.7, config.line_base_speed * 1.3
#         motors.run(v1, v2, 1.3)
#         motors.run_until(v1, v2, colour.read, 3, ">=", 50, "ALIGNING RIGHT")
#         motors.run_until(v1, v2, colour.read, 3, "<=", 45, "ALIGNING RIGHT")
#         motors.run(config.line_base_speed, -config.line_base_speed, 0.23)

#     elif signal == "+ right":
#         v2, v1 = -config.line_base_speed * 0.7, config.line_base_speed * 1.3
#         motors.run(v1, v2, 1.3)
#         motors.run_until(v1, v2, colour.read, 1, ">=", 50, "ALIGNING LEFT")
#         motors.run_until(v1, v2, colour.read, 1, "<=", 45, "ALIGNING LEFT")
#         motors.run(-config.line_base_speed, config.line_base_speed, 0.23)

#     elif signal == "+ straight":
#         motors.run(config.line_base_speed * 1.2, config.line_base_speed * 1.2, 0.35)

#     elif signal == "+ double":
#         motors.run(config.line_base_speed * 2, -config.line_base_speed * 2, 2)
#         motors.run_until(config.line_base_speed * 2, -config.line_base_speed * 2, colour.read, 1, "<=", 50, "SPINNING")
#     else:
#         pass
