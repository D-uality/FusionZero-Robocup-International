file_path = r"/home/fusion/FusionZero-Robocup-International/code/line_follow/sensor_fusion_line_following/1_wheg_testing/some_values.txt"

def main():
    with open(file_path, "r") as file:
        lines = file.read().split("\n")
        
        piped_lines = []
        processed_lines = []
        
        for line in lines:
            if "|" in line: piped_lines.append(line)
        
        for i, line in enumerate(piped_lines):
            if "GREEN CHECK" in line:
                next_line = piped_lines[i + 1] if i < (len(piped_lines) - 1) else ""
                colour_values = list(map(int, line[29:].split(", ")))
                signal = next_line[29:] 
                
                processed_lines.append([colour_values[0], colour_values[1], colour_values[2], colour_values[3], colour_values[4], colour_values[5], colour_values[6], signal])
        
        print("sum(colour_values[5], colour_values[6])")
        
        fake_total, real_total = 0, 0
        fake_average, real_average = 0, 0
        
        for line in processed_lines:            
            if "fake" in line[-1]:
                fake_total += line[5] + line[6]
            else:
                real_total += line[5] + line[6]
        
        fake_average = fake_total / len(processed_lines) * 2
        real_average = real_total / len(processed_lines) * 2
        
        print(f"{fake_average=:.2f} {real_average=:.2f}")
        
        
        print()
        print("sum(colour_values[0], colour_values[1])")
        
        left_total, right_total     = 0, 0
        left_average, right_average = 0, 0
        
        for line in processed_lines:            
            if "|-" in line[-1]:
                right_total += line[0] + line[1]
            else:
                left_total += line[0] + line[1]
        
        left_average  = left_total  / len(processed_lines) * 2
        right_average = right_total / len(processed_lines) * 2
        
        print(f"{left_average=:.2f} {right_average=:.2f}")
        
        
        print()
        print("sum(colour[2])")
        
        print(sum(v[2] for v in processed_lines) / len(processed_lines))
        print(f"min for colour_values[2]: {min(processed_lines, key=lambda v: v[2])}")
        
        print()
        outer_values = [line[0] for line in processed_lines if "|-" in line[-1]]
        print(f"{sum(outer_values) / len(outer_values):.2f}")
        print(f"max for colour_values[0]: {max(outer_values)}")
                    
if __name__ == "__main__": main()