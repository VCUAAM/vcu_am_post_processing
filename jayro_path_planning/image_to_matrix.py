from PIL import Image
import numpy as np

# image_to_matrix takes an image and converts it into a text file holding 0s and 1s, where 0 == white, and 1 == black
def image_to_matrix(image_path, output_path):
    img = Image.open(image_path).convert('L')
    width, height = img.size
    matrix = np.zeros((height, width), dtype=int)

    count=0
    for y in range(height):
        for x in range(width):
            pixel = img.getpixel((x, y))
            matrix[y][x] = 1 if pixel < 128 else 0
            if pixel < 128:
                count+=1
    
    # Write the matrix to a file
    with open(output_path, 'w') as f:
        for row in matrix:
            f.write(' '.join(map(str, row)) + '\n')

