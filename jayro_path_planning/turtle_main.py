import turtle
import numpy as np
import os
import time
import matplotlib.colors as mcolors
from cleaner_robot.robot import Robot
from cleaner_robot.sweeper import Sweeper

def read_matrix_from_file(file_path):
    with open(file_path, 'r') as file:
        return [list(map(int, line.split())) for line in file]

def create_grid(rows, cols, num_obstacles):
    grid = np.zeros((rows, cols), dtype=int)
    obstacle_positions = set()
    while len(obstacle_positions) < num_obstacles:
        x, y = np.random.randint(0, cols), np.random.randint(0, rows)
        obstacle_positions.add((y, x))
    for y, x in obstacle_positions:
        grid[y, x] = 1
    return grid

def initialize_robot(grid, start_position, start_direction, robot_size):
    robot = Robot(grid, start_position, start_direction, robot_size)
    sweeper = Sweeper(robot)
    sweeper.spiral = True
    return robot, sweeper

def visualize_grid(grid, cell_size, t):
    rows, cols = grid.shape  # Get matrix dimensions
    
    # Set turtle to draw grid lines
    t.pencolor("lightgray")
    t.pensize(1)
    
    # Calculate grid boundaries
    width = cols * cell_size
    height = rows * cell_size
    x_offset = -width / 2
    y_offset = height / 2

    # Draw vertical grid lines
    for x in range(cols + 1):
        t.penup()
        t.goto(x_offset + x * cell_size, y_offset)
        t.pendown()
        t.goto(x_offset + x * cell_size, y_offset - height)
    
    # Draw horizontal grid lines
    for y in range(rows + 1):
        t.penup()
        t.goto(x_offset, y_offset - y * cell_size)
        t.pendown()
        t.goto(x_offset + width, y_offset - y * cell_size)
    
    t.penup()  # Stop drawing

    # Draw obstacles
    t.pencolor("black")  # Reset to black for obstacles
    for y in range(rows):
        for x in range(cols):
            if grid[y, x] == 1:
                t.goto(x_offset + x * cell_size, y_offset - y * cell_size)
                t.pendown()
                t.fillcolor("black")
                t.begin_fill()
                for _ in range(4):
                    t.forward(cell_size)
                    t.right(90)
                t.end_fill()
                t.penup()

def animate_robot_movement(path_taken, cell_size, grid_size, robot_visual_size, screen, t):
    for x, y in path_taken:
        t.goto(x * cell_size - grid_size * cell_size / 2, grid_size * cell_size / 2 - y * cell_size)
        t.pendown()
        t.fillcolor("green")
        t.begin_fill()
        for _ in range(4):
            t.forward(robot_visual_size)
            t.right(90)
        t.end_fill()
        t.penup()
        screen.update()
        time.sleep(0.1)

def calculate_cell_size(rows, cols, max_screen_size=600):
    """
    Determines an appropriate cell size so the entire grid fits within max_screen_size.
    """
    max_cells = max(rows, cols)  # Find the larger dimension
    return max_screen_size // max_cells  # Scale cell size so grid fits within max_screen_size

def find_start_position(grid, robot_size):
    """
    Finds the bottom-leftmost zero in the grid and shifts the start position 
    up by `robot_size` to ensure the robot fits.
    """
    rows, cols = grid.shape

    # Scan bottom-up, left-right for the first available 0 cell
    for y in range(rows - 1, -1, -1):  # Start from the bottom row
        for x in range(cols):  # Scan left to right
            if grid[y, x] == 0:
                # Ensure the robot has space by shifting up
                adjusted_y = max(0, y - (robot_size - 1))  # Avoid negative index
                return {'x': x, 'y': adjusted_y}

    # If no valid start position found
    return None

def run(robot_size, matrix_file):
    grid = np.array(read_matrix_from_file(matrix_file))  # Load the matrix
    rows, cols = grid.shape  # Get matrix size dynamically
    start_position = find_start_position(grid, robot_size)

    # Determine cell size dynamically
    cell_size = calculate_cell_size(rows, cols)

    # # Initialize robot
    robot, sweeper = initialize_robot(grid, start_position, 0, robot_size)
    sweeper.sweep()
    path_taken = robot.get_path()
    print("Path of Coordinates:", path_taken)

    # robot_visual_size = robot_size * cell_size

    # # Adjust screen size based on grid dimensions
    # screen = turtle.Screen()
    # screen.setup(cols * cell_size + 100, rows * cell_size + 100)
    # screen.tracer(0)
    # t = turtle.Turtle()
    # t.speed(0)
    # t.penup()

    # visualize_grid(grid, cell_size, t)
    # animate_robot_movement(path_taken, cell_size, rows, robot_visual_size, screen, t)
    # turtle.done()
    return path_taken


def main(input_folder, output_folder, robot_size):
    start_time = time.time()
    os.makedirs(output_folder, exist_ok=True)
    
    # Get existing output file count
    existing_files = [f for f in os.listdir(output_folder) if f.startswith("layer_")]
    existing_files.sort()  # Sort to find the latest layer
    last_layer = 0
    
    if existing_files:
        last_layer = max(int(f.split('_')[1]) for f in existing_files if f.split('_')[1].isdigit())
    
    # Process each file in the input folder
    for file_name in os.listdir(input_folder):
        input_path = os.path.join(input_folder, file_name)
        
        if os.path.isfile(input_path):
            
            processed_content = run(robot_size, input_path)
            
            # Convert list output to string
            processed_str = '\n'.join(map(str, processed_content))
            
            # Determine new output file name
            last_layer += 1
            output_file_name = f"layer_{last_layer}"
            output_path = os.path.join(output_folder, output_file_name)
            
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(processed_str)
            
            # Delete input file after processing
            os.remove(input_path)
    end_time = time.time()  # End the timer
    elapsed_time = end_time - start_time 
    print(elapsed_time)
    return elapsed_time


if __name__ == "__main__":
    input_folder = "path_input"
    output_folder = "path_output"
    robot_size = 10

    main(input_folder, output_folder, robot_size)

