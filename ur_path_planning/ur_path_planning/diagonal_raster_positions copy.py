from math import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def create_offset(x1,y1,x2,y2,hose_OD):
    x_lo = x1 + hose_OD/2
    x_hi = x2 - hose_OD/2
    y_lo = y1 + hose_OD/2
    y_hi = y2 - hose_OD/2

    return [x_lo,x_hi,y_lo,y_hi]

def step_size(lims,overlap,hose_OD):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    x_passes = ceil((x_hi - x_lo)/overlap*hose_OD)
    hx = (x_hi - x_lo)/x_passes
    y_passes = ceil((y_hi - y_lo)/overlap*hose_OD)
    hy = (y_hi - y_lo)/y_passes

    return hx,hy

def intersections(lims,func,last):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    sols = [[0,1,x_lo],
            [0,1,x_hi],
            [1,0,y_lo],
            [1,0,y_hi]]
    for sol in sols:
        if last == sol[2] or last == sol[2]:
            continue
        lin = np.asarray([sol,func])
        inter = np.linalg.solve(lin[:,[0,1]],lin[:,2])
        print(inter)
        if (inter[0] >= x_lo and inter[0] <= x_hi) and (inter[1] >= y_lo and inter[1] <= y_hi):
            return inter
    
    print("you done fucked up a-a-ron")
    quit()

def position_generator(lims,hx,hy):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    m = (y_hi - y_lo)/(x_hi - x_lo)
    positions = [[x_lo,y_lo]]
    xi,yi = x_lo,y_lo
    i = 0
    sx,sy = False,False
    
    while True:
        match i%4:
            case 0:
                if yi == y_hi and not sy:
                    sy = True
                xi = positions[-1][0] + sy*hx
                yi = positions[-1][0] + (not sy)*hy
            case 1:
                func = [m,1,-m*xi + yi]
                inter = intersections(lims,func,xi)
                xi = inter[0]
                yi = inter[1]
            case 2:
                if xi == x_hi and not sx:
                    sx = True
                xi = positions[-1][0] + (not sx)*hx
                yi = positions[-1][0] + sx*hy
            case 3:
                func = [m,1,-m*xi + yi]
                inter = intersections(lims,func,yi)
                xi = inter[0]
                yi = inter[1]
        positions.append([xi,yi])
        print(positions)
        if positions[-1] == [x_hi,y_hi] or i == 100:
            break
        i += 1
    
    return positions

def main(x1,y1,x2,y2,hose_OD,overlap):
    lims = create_offset(x1,y1,x2,y2,hose_OD)
    hx,hy = step_size(lims,overlap,hose_OD)
    pos = position_generator(lims,hx,hy)
    #print(pos)
    pos = np.asarray(pos)
    fig,ax = plt.subplots()
    plt.plot(pos[:,0],pos[:,1])
    rect = Rectangle((x1,y1),x2 - x1,y2 - y1,linewidth=1, edgecolor='black', facecolor='none')
    ax.add_patch(rect)
    plt.show()

if __name__ == "__main__":
    main(0,0,23.36,15,2,.75)    