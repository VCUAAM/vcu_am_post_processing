from math import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

'''
Function to create the offset box that the pathing takes place in. 
This is important because the hose will have some amount of outer diameter, and will have some parts of the hose over the edge or collide into a wall if not accounted for.
Currently, places center of hose 1 radius away from the edge of the table. This does not account for path rounding at corners and will need to be fixed before moving into the real 3D printer.
Returns a list of the "limits" to reduce extra function parameters. These are notated as x low, x high, y low, and y high, respectively.
'''
def create_offset(x1,y1,x2,y2,hose_OD):
    x_lo = x1 + hose_OD/2
    x_hi = x2 - hose_OD/2
    y_lo = y1 + hose_OD/2
    y_hi = y2 - hose_OD/2

    return [x_lo,x_hi,y_lo,y_hi]

'''
Function to determine the step size required in both directions
This is calculated by finding the true "ideal" step size using the overlap and the hose OD over the required range, and then rounding this value up to determine a step that has slightly higher overlap, but a clean number of steps
Returns the steps in both directions, hx and hy
'''
def step_size(lims,overlap,hose_OD):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    x_passes = ceil((x_hi - x_lo)/(overlap*hose_OD))
    hx = (x_hi - x_lo)/x_passes
    y_passes = ceil((y_hi - y_lo)/(overlap*hose_OD))
    hy = (y_hi - y_lo)/y_passes

    return hx,hy

'''
Function to determine the path that the hose will need to take
This is acheived by incrementing by the steps hx and hy calculated in step_size()

The tricky part is handling when the hose "meets" the top edge and right edge
These are handled by the booleans sx and sy, for switch x and switch y
Every iteration checks to see if the currently calculated values are at the limits x high and y high
If this is the case, sx and sy respectively are set to true, and change the logic of the appropriate functions
This switches from incrementing in one direction to incrementing in the other direction
Because Python handles floats weird, an error is checked instead of the step size, to account for automatic rounding. This is currently set to 5/6 of the step size.

The switch-case handles checking which iteration the function is on
There are only 4 possible actions that can take place at any stage of the function, and they repeat iteratively, so by checking the modulo of 4 you can determine the appropriate action
Each case of determines the current iterations xi and yi, which are the notation for the next position in the path
The return is the list of all positions in the path
'''
def position_generator(lims,hx,hy):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    positions = [[x_lo,y_lo]]
    i = 0
    errx,erry = 5*hx/6,5*hy/6
    sx,sy = False,False
    
    while True:
        match i%4:
            case 0:
                xi = positions[-1][0] + sy*hx
                yi = positions[-1][1] + (not sy)*hy
            case 1:
                if i != 1:
                    xi = positions[-3][0] + (not sx)*hx
                    yi = positions[-3][1] + sx*hy
                else:
                    xi = positions[-2][0] + (not sx)*hx
                    yi = positions[-2][1] + sx*hy
            case 2:
                xi = positions[-1][0] + (not sx)*hx
                yi = positions[-1][1] + sx*hy
            case 3:
                xi = positions[-3][0] + sy*hx
                yi = positions[-3][1] + (not sy)*hy
        positions.append([xi,yi])
        if abs(yi - y_hi) < erry and not sy:
            sy = True
        if abs(xi - x_hi) < errx and not sx:
            sx = True
        if abs(xi - x_hi) < errx and abs(yi - y_hi) < erry:
            break
        i += 1
    
    return positions

'''
Main function logic of the script
This takes input of x1,y1,x2,y2,hose OD, and overlap
(x1,y1) and (x2,y2) are two opposite corners of the surface to rasterize
Hose OD is the outer diameter of the vacuum
Overlap is the percentage of overlap that the hose will have between passes.

This function call collects the necessary parameters, finds the positions, and plots the positions in order
It also plots a view of the table to examine the path
This is currently only setup to handle the orientation in which the two corners are bottom left and top right
Additionally, it currently only works with a rectangular shape, and is not setup to handle more complex 4-sided shapes
'''
def main(x1,y1,x2,y2,hose_OD,overlap):
    lims = create_offset(x1,y1,x2,y2,hose_OD)
    hx,hy = step_size(lims,overlap,hose_OD)
    pos = position_generator(lims,hx,hy)
    pos = np.asarray(pos)
    fig,ax = plt.subplots()
    plt.plot(pos[:,0],pos[:,1],)
    rect = Rectangle((x1,y1),x2 - x1,y2 - y1,linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(rect)
    plt.show()

'''
This allows main() to be ran if it is the main script, but allows for other scripts to run it instead without causing main() to execute
Example values were selected for testing
'''
if __name__ == "__main__":
    main(0,0,23.36,15,2,.75)    