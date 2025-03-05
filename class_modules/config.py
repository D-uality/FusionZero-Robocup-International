def update(data: list[str], coloumn_widths: list[int], separator: str = "|"):
    formatted_cells = [f"{cell:^{width}}" for cell, width in zip(data, coloumn_widths)]
    print(f" {separator} ".join(formatted_cells), end="")

method_size = 24
sensor_readings_size = 30