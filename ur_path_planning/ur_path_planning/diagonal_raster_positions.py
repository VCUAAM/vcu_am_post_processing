from math import *
import numpy as np
import matplotlib.pyplot as plt

def create_offset(x1,y1,x2,y2,hose_OD):
    x_lo = x1 + hose_OD/2
    x_hi = x2 - hose_OD/2
    y_lo = y1 + hose_OD/2
    y_hi = y2 - hose_OD/2

    return [x_lo,x_hi,y_lo,y_hi]

def hypotenuse_number_passes(lims,hose_OD):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    x_passes = ceil((x_hi - x_lo)/hose_OD)
    x_passes += x_passes % 2

    y_passes = ceil((y_hi - y_lo)/hose_OD)
    y_passes += y_passes % 2
    
    x_pas = (2*x_passes) - 1
    y_pas = (2*y_passes) - 1

    return x_passes,y_passes,x_pas,y_pas

def pas_offset(lims,x_passes, y_passes):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    pas_off = (((x_hi - x_lo) + (y_hi - y_lo)) / (x_passes + y_passes))

    return pas_off

def generate_positions_bl_corner(positions,lims,x_pas,y_pas,pas_off):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    
    pos_bl_corner = []

    for i in range(min(x_pas,y_pas) + 1):
        pos = []

        if len(pos_bl_corner) < 2:
            pos.append(x_lo)
        elif len(pos_bl_corner) == 2:
            pos.append(x_lo + pas_off)
        elif pos_bl_corner[-2][1] == y_lo:
            pos.append(x_lo)
        elif pos_bl_corner[-1][0] != y_lo:
            pos.append(x_lo + (((i + 1)/2)*pas_off))
        else:
            pos.append(x_lo + (((i)/2)*pas_off))

        if len(pos_bl_corner) == 0:
            pos.append(y_lo)
        elif len(pos_bl_corner) == 1:
            pos.append(y_lo + pas_off)
        elif len(pos_bl_corner) == 2:
            pos.append(y_lo)
        elif len(pos_bl_corner) > 2 and pos_bl_corner[-2][0] == x_lo:
            pos.append(y_lo)
        elif len(pos_bl_corner) > 2 and pos_bl_corner[-2][0] != x_lo:
            if pos_bl_corner[-1][0] != x_lo:
                pos.append(y_lo + ((i/2)*pas_off))
            if pos_bl_corner[-1][0] == x_lo:
                pos.append(y_lo + (((i + 1)/2)*pas_off))
        else:
            pos.append(y_lo + (((i)/2)*pas_off))

        pos_bl_corner.append(pos)
        if len(pos_bl_corner) > 2 and pos_bl_corner[-1] == pos_bl_corner[-2]:
            del pos_bl_corner[-1]
        if len(pos_bl_corner) > 2 and pos_bl_corner[-1][0] > x_hi:
            pos_bl_corner[-1][0] = x_hi

    for pos in pos_bl_corner:
        positions.append(pos)

    return positions

def generate_positions_b_skew(positions,lims,x_pas,y_pas,pas_off):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    
    pos_b_skew = []
    
    for i in range(y_pas,x_pas):
        pos = []
        k = i - y_pas
        y_above = positions[-1][0] + pas_off

        if len(pos_b_skew) == 0:
            pos.append(y_above)
        elif len(pos_b_skew) == 1:
            pos.append(x_lo + y_above - y_hi)
        elif len(pos_b_skew) == 2:
            pos.append(x_lo + y_above - y_hi + pas_off)
        elif len(pos_b_skew) > 2 and len(pos_b_skew[-2]) == 2 and pos_b_skew[-2][1] == y_lo:
            if len(pos_b_skew[-1]) == 2 and pos_b_skew[-1][1] == y_lo:
                pos.append(x_lo + y_above - y_hi + (k - 1)/2*pas_off)
            elif len(pos_b_skew[-1]) == 2 and pos_b_skew[-1][1] == y_hi:
                pos.append(x_lo + y_above - y_hi + k/2*pas_off)
        elif len(pos_b_skew) > 2 and len(pos_b_skew[-2]) == 2 and pos_b_skew[-2][1] == y_hi:
            if len(pos_b_skew[-1]) == 2 and pos_b_skew[-1][1] == y_hi:
                pos.append(y_above + (k - 1)/2*pas_off)
            elif len(pos_b_skew[-1]) == 2 and pos_b_skew[-1][1] == y_lo:
                pos.append(y_above + k/2*pas_off)

        if len(pos_b_skew) == 0:
            pos.append(y_lo)
        elif len(pos_b_skew) == 1 or len(pos_b_skew) == 2:
            pos.append(y_hi)
        elif len(pos_b_skew) > 2 and len(pos_b_skew[-1]) == 2:
            if len(pos_b_skew[-2]) == 2 and pos_b_skew[-2][1] == y_hi:
                pos.append(y_lo)
            elif len(pos_b_skew[-2]) == 2 and pos_b_skew[-2][1] == y_lo:
                pos.append(y_hi)
        pos_b_skew.append(pos)
        if len(pos_b_skew) > 2 and pos_b_skew[-1] == pos_b_skew[-2]:
            del pos_b_skew[-1]
    
    for pos in pos_b_skew:
        positions.append(pos)

    return positions

def generate_positions_t_skew(positions,lims,x_pas,y_pas,pas_off):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    
    pos_t_skew = []
    for i in range(x_pas,y_pas):
        pos = []
        k = i - x_pas
        x_beyond = positions[-1][0] + pas_off

        if len(pos_t_skew) == 0:
            pos.append(x_hi)
        elif len(pos_t_skew) == 1 or len(pos_t_skew) == 2:
            pos.append(x_lo)
        elif len(pos_t_skew) > 2 and len(pos_t_skew[-1]) == 2:
            if len(pos_t_skew[-2]) == 2 and pos_t_skew[-2][0]== x_hi:
                pos.append(x_lo)
            elif len(pos_t_skew[-2]) == 2 and pos_t_skew[-2][0] == x_lo:
                pos.append(x_hi)

        if len(pos_t_skew) == 0:
            pos.append(y_lo + x_beyond - x_hi)
        elif len(pos_t_skew) == 1:
            pos.append(x_beyond)
        elif len(pos_t_skew) == 2:
            pos.append(x_beyond + pas_off)
        elif len(pos_t_skew) > 2 and len(pos_t_skew[-1]) == 2 and pos_t_skew[-1][0] == x_lo:
            if len(pos_t_skew[-2]) == 2 and pos_t_skew[-2][0] == x_lo:
                pos.append(x_beyond - x_hi + y_lo + (((k - 1)/2)*pas_off))
            elif len(pos_t_skew[-2]) == 2 and pos_t_skew[-2][0] == x_hi:
                pos.append(x_beyond + ((k/2)*pas_off))
        elif len(pos_t_skew) > 2 and len(pos_t_skew[-1]) == 2 and pos_t_skew[-1][0] == x_hi:
            if len(pos_t_skew[-2]) == 2 and pos_t_skew[-2][0] == x_hi:
                pos.append(x_beyond + (((k - 1)/2)*pas_off))
            elif len(pos_t_skew[-2]) == 2 and pos_t_skew[-2][0] == x_lo:
                pos.append(x_beyond - x_hi + y_lo + (((k)/2)*pas_off))

        pos_t_skew.append(pos)
        if len(pos_t_skew) > 2 and pos_t_skew[-1] == pos_t_skew[-2]:
            del pos_t_skew[-1]
        if len(pos_t_skew) > 2 and pos_t_skew[0][1] < y_lo:
            pos_t_skew[0][1] = y_lo
        for h in range(len(pos_t_skew)):
            if pos_t_skew[h][1] > y_hi:
                del pos_t_skew[h:]

    for pos in pos_t_skew:
        positions.append(pos)

    return positions

def generate_positions_tr_corner(positions,lims,x_pas,y_pas,pas_off):
    x_lo,x_hi,y_lo,y_hi = lims[0],lims[1],lims[2],lims[3]
    
    pos_tr_corner = []
    for i in range(0,(y_pas + x_pas + 2 - max(x_pas,y_pas))):
        pos = []
        if len(pos_tr_corner) == 0:
            pos.append(x_hi)
        elif len(pos_tr_corner) == 1:
            pos.append(x_hi - pas_off)
        elif len(pos_tr_corner) == 2:
            pos.append(x_hi)
        elif len(pos_tr_corner) > 2 and len(pos_tr_corner[-1]) == 2 and pos_tr_corner[-1][0] == x_hi:
            if len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] != x_hi:
                pos.append(x_hi)
            elif len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] == x_hi:
                pos.append(x_hi - (i/2*pas_off))
        elif len(pos_tr_corner) > 2 and len(pos_tr_corner[-1]) == 2 and pos_tr_corner[-1][0] != x_hi:
            if len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] != x_hi:
                pos.append(x_hi)
            elif len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] == x_hi:
                pos.append(x_hi - (i + 1)/2*pas_off)
        if len(pos_tr_corner) == 0 or len(pos_tr_corner) == 1:
            pos.append(y_hi)
        elif len(pos_tr_corner) == 2:
            pos.append(y_hi - pas_off)
        elif len(pos_tr_corner) > 2 and len(pos_tr_corner[-1]) == 2 and pos_tr_corner[-1][0] == x_hi:
            if len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] != x_hi:
                pos.append(y_hi - (i + 1)/2*pas_off)
            elif len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] == x_hi:
                pos.append(y_hi)
        elif len(pos_tr_corner) > 2 and len(pos_tr_corner[-1]) == 2 and pos_tr_corner[-1][0] != x_hi:
            if len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] != x_hi:
                pos.append(y_hi - i/2*pas_off)
            elif len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] == x_hi:
                pos.append(y_hi)
        pos_tr_corner.append(pos)
        if len(pos_tr_corner) > 2 and pos_tr_corner[-1] == pos_tr_corner[-2]:
            del pos_tr_corner[-1]
        if len(pos_tr_corner) > 2 and pos_tr_corner[0][1] < y_lo:
            pos_tr_corner[0][1] = y_lo
        for j in range(len(pos_tr_corner)):
            if pos_tr_corner[j][1] < y_lo:
                del pos_tr_corner[j:]
    if len(pos_tr_corner) > 2 and pos_tr_corner[-1] != [x_hi,y_lo]:
        pos_tr_corner.append([x_hi,y_lo])
    inverted_pos_tr_corner = pos_tr_corner[::-1]

    for pos in inverted_pos_tr_corner:
        positions.append(pos)

    return positions   

def final_diagonal_raster_positions(x1,y1,x2,y2,hose_OD):
    lims = create_offset(x1,y1,x2,y2,hose_OD)
    x_passes,y_passes,x_pas,y_pas = hypotenuse_number_passes(lims,hose_OD)
    pas_off = pas_offset(lims,x_passes,y_passes)

    positions = []
    positions = generate_positions_bl_corner(positions,lims,x_pas,y_pas,pas_off)
    
    cas = abs(lims[1] - lims[3])/(lims[1] - lims[3])
    
    match cas:
        case -1:
            positions = generate_positions_t_skew(positions,lims,x_pas,y_pas,pas_off)
        case 1:
            positions = generate_positions_b_skew(positions,lims,x_pas,y_pas,pas_off)
        case other:
            pass

    positions = generate_positions_tr_corner(positions,lims,x_pas,y_pas,pas_off)
    
    return positions

def main(x1,y1,x2,y2,hose_OD):
    pos = final_diagonal_raster_positions(x1,y1,x2,y2,hose_OD)
    print(pos)
    pos = np.asarray(pos)
    plt.figure(1)
    plt.plot(pos[:,0],pos[:,1])
    plt.show()

if __name__ == "__main__":
    main(0,0,23.36,15,2)    