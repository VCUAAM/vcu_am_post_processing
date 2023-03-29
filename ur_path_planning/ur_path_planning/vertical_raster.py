import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def create_offset(x1,y1,x2,y2,hose_OD):
    x_lo = x1 + hose_OD/2
    x_hi = x2 - hose_OD/2
    y_lo = y1 + hose_OD/2
    y_hi = y2 - hose_OD/2

    return x_lo,x_hi,y_lo,y_hi

def number_passes(x_lo,x_hi,y_lo,y_hi,hose_OD):
    x_passes = math.ceil((x_hi - x_lo)/hose_OD)
    x_passes += x_passes%2
    y_passes = math.ceil((y_hi - y_lo)/hose_OD)

    return x_passes, y_passes

def generate_positions_vertical(x1,y1,x2,y2,hose_OD):
    positions = []
    x_lo,x_hi,y_lo,y_hi = create_offset(x1,y1,x2,y2,hose_OD)
    x_passes,y_passes = number_passes(x_lo,x_hi,y_lo,y_hi,hose_OD)
    dx = float((x_hi - x_lo)/(x_passes))

    for i in range(2*x_passes + 2):
        pos = []

        if i % 2 == 0:
            pos.append(x_lo + dx*i/2)
        else:
            pos.append(x_lo + dx*(i - 1)/2)

        if i%4 == 0 or i%4 == 3:
            pos.append(y_lo)
        else:
            pos.append(y_hi)

        positions.append(pos)

    return positions

def main(x1,y1,x2,y2,hose_OD):
    pos = generate_positions_vertical(x1,y1,x2,y2,hose_OD) #x1,y1,x2,y2,hose_OD
    #print(pos)
    pos = np.asarray(pos)
    fig,ax = plt.subplots()
    plt.plot(pos[:,0],pos[:,1])
    rect = Rectangle((x1,y1),x2 - x1,y2 - y1,linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(rect)
    plt.show()

if __name__ == "__main__":
    main(0,0,23.36,15,2)
