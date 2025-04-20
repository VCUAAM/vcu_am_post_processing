import turtle
import numpy as np
import os
import time
import matplotlib.colors as mcolors
from cleaner_robot.robot import Robot
from cleaner_robot.sweeper import Sweeper
from image_to_matrix import image_to_matrix

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

    for y in range(rows - robot_size + 1):  # top to bottom
        for x in range(cols - robot_size + 1):  # left to right
            subgrid = grid[y:y + robot_size, x:x + robot_size]
            if np.all(subgrid == 0):
                return {'x': x, 'y': y}

    # No valid position found
    return None

def run(robot_size, matrix):
    rows, cols = matrix.shape  # Get matrix size dynamically
    start_position = find_start_position(matrix, robot_size)

    # Determine cell size dynamically
    # cell_size = calculate_cell_size(rows, cols)
    # cell_size = min(800 // cols, 600 // rows)

    # # Initialize robot
    robot, sweeper = initialize_robot(matrix, start_position, 0, robot_size)
    sweeper.sweep()
    path_taken = robot.get_path()
    # print("Path of Coordinates:", path_taken)

    # robot_visual_size = robot_size * cell_size

    # Adjust screen size based on grid dimensions
    # screen = turtle.Screen()
    # # screen.setup(cols * cell_size + 100, rows * cell_size + 100)
    # screen.setup(width=1000, height=800)
    # # screen.setworldcoordinates(0, rows * cell_size, cols * cell_size, 0)
    # screen.tracer(0)
    # t = turtle.Turtle()
    # t.speed(0)
    # t.penup()

    # visualize_grid(matrix, cell_size, t)
    # animate_robot_movement(path_taken, cell_size, rows, robot_visual_size, screen, t)
    # turtle.done()
    return path_taken


def main(robot_size):
    start_time = time.time()
    
    base_dir = os.path.dirname(os.path.abspath(__file__))
    images_dir = os.path.join(base_dir, 'images')
    path_input = os.path.join(base_dir,  'path_input')
    path_output = os.path.join(base_dir, 'path_output')

    os.makedirs(path_input, exist_ok=True)
    os.makedirs(path_output, exist_ok=True)

    # Step 1: Get single image from images folder
    image_files = [f for f in os.listdir(images_dir) if f.endswith(('.png', '.jpg'))]
    if len(image_files) != 1:
        raise ValueError("There must be exactly one image in the 'images' folder.")
    image_path = os.path.join(images_dir, image_files[0])

    # Step 2: Convert image to matrix and save as new_layer
    matrix_output_path = os.path.join(path_input, 'new_layer.txt')
    image_to_matrix(image_path, matrix_output_path)

    # Step 3: Load matrix and run robot/sweeper
    matrix = np.array(read_matrix_from_file(matrix_output_path))
    output = run(robot_size, matrix)

    # Convert list output to string
    output_str = '\n'.join(map(str, output))

    # Step 4: Save output to next layer in path_output
    existing_layers = [f for f in os.listdir(path_output) if f.startswith('layer_') and f[len('layer_'):].isdigit()]
    if existing_layers:
        next_index = max(int(f[len('layer_'):]) for f in existing_layers) + 1
    else:
        next_index = 1

    output_path = os.path.join(path_output, f'layer_{next_index}')

    # Write output to file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(output_str)

    os.remove(matrix_output_path)
    os.remove(image_path)
    end_time = time.time()  # End the timer
    elapsed_time = end_time - start_time 
    print(elapsed_time)
    return elapsed_time


if __name__ == "__main__":
    robot_size = 10
    main(robot_size)

