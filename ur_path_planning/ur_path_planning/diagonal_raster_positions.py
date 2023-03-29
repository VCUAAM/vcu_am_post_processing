import importlib
import boundary_coordinates
import math

importlib.reload(boundary_coordinates)

hose_ex_d = 2
hose_ex_r = hose_ex_d/2

def create_offset():
    x1,y1,x2,y2 = boundary_coordinates.bound_coord()
    x_lo = x1 + hose_ex_r
    x_hi = x2 - hose_ex_r
    y_lo = y1 + hose_ex_r
    y_hi = y2 - hose_ex_r
    return x_lo,x_hi,y_lo,y_hi

def hypotenuse():
    x_lo,x_hi,y_lo,y_hi = create_offset()
    hyp = math.sqrt(((y_hi-y_lo)**2)-((x_hi-x_lo)**2))

def hypotenuse_number_passes():
    x_lo,x_hi,y_lo,y_hi = create_offset()
    
    if (x_hi-x_lo)/hose_ex_d % 2 == 0:
        x_passes = (x_hi-x_lo)/hose_ex_d
    else:
        unround_x_passes = (x_hi-x_lo)/hose_ex_d
        x_passes = math.ceil(unround_x_passes)
        if x_passes % 2 == 1:
            x_passes += 1

    if (y_hi-y_lo)/hose_ex_d % 2 == 0:
        y_passes = (y_hi-y_lo)/hose_ex_d
    else:
        unround_y_passes = (y_hi-y_lo)/hose_ex_d
        y_passes = math.ceil(unround_y_passes)
        if y_passes % 2 == 1:
            y_passes += 1
    
    return x_passes, y_passes

def pas_offset():
    x_passes, y_passes = hypotenuse_number_passes()
    x_pas = x_passes
    x_lo,x_hi,y_lo,y_hi = create_offset()
    dx = float((x_hi-x_lo)/(x_passes))
    y_pas = y_passes
    x_lo,x_hi,y_lo,y_hi = create_offset()
    dy = float((y_hi-y_lo)/(y_pas-1))
    pas_off = (((x_hi - x_lo) + (y_hi - y_lo)) / (x_pas + y_pas))
    return dx,dy,pas_off

def generate_positions_bl_corner():
    x_lo,x_hi,y_lo,y_hi = create_offset()
    x_passes, y_passes = hypotenuse_number_passes()
    x_pas = (2*x_passes) - 1
    y_pas = (2*y_passes) - 1
    dx, dy, pas_off = pas_offset()
    
    pos_bl_corner = []
    i = 0

    while i <= x_pas + y_pas:
        if i <= x_pas and i <= y_pas:
            pos = []
            j = 0
            while j < 2:
                if j == 0:
                    if len(pos_bl_corner) == 0:
                        pos.append(x_lo)
                    elif len(pos_bl_corner) == 1:
                        pos.append(x_lo)
                    elif len(pos_bl_corner) == 2:
                        pos.append(x_lo + pas_off)
                    elif len(pos_bl_corner) > 2 and len(pos_bl_corner[-2]) == 2 and pos_bl_corner[-2][1] == y_lo:
                        pos.append(x_lo)
                    elif len(pos_bl_corner) > 2 and pos_bl_corner[-2][1] != y_lo:
                        if pos_bl_corner[-1][0] != y_lo:
                            pos.append(x_lo + (((i + 1)/2)*pas_off))
                        if pos_bl_corner[-1][0] == y_lo:
                            pos.append(x_lo + (((i)/2)*pas_off))
                elif j == 1:
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
                j += 1
        i += 1
        pos_bl_corner.append(pos)
        if len(pos_bl_corner) > 2 and pos_bl_corner[-1] == pos_bl_corner[-2]:
            del pos_bl_corner[-1]
        if len(pos_bl_corner) > 2 and pos_bl_corner[-1][0] > x_hi:
            pos_bl_corner[-1][0] = x_hi
    return pos_bl_corner

def generate_positions_b_skew():
    x_lo,x_hi,y_lo,y_hi = create_offset()
    x_passes, y_passes = hypotenuse_number_passes()
    x_pas = (2*x_passes) - 1
    y_pas = (2*y_passes) - 1
    dx, dy, pas_off = pas_offset()
    
    pos_bl_corner = generate_positions_bl_corner()
    pos_b_skew = []
    i = y_pas
    
    while i <= x_pas + y_pas:
        if i < x_pas and i >= y_pas:
            pos = []
            j = 0
            k = i - y_pas
            while j < 2:
                y_above = pos_bl_corner[-1][0] + pas_off
                if j == 0:
                    if len(pos_b_skew) == 0:
                        pos.append(y_above)
                    elif len(pos_b_skew) == 1:
                        pos.append(x_lo + y_above - y_hi)
                    elif len(pos_b_skew) == 2:
                        pos.append(x_lo + y_above - y_hi + (pas_off))
                    elif len(pos_b_skew) > 2 and len(pos_b_skew[-2]) == 2 and pos_b_skew[-2][1] == y_lo:
                        if len(pos_b_skew[-1]) == 2 and pos_b_skew[-1][1] == y_lo:
                            pos.append(x_lo + y_above - y_hi + (((k-1)/2)*pas_off))
                        elif len(pos_b_skew[-1]) == 2 and pos_b_skew[-1][1] == y_hi:
                            pos.append(x_lo + y_above - y_hi + (((k )/2)*pas_off))
                    elif len(pos_b_skew) > 2 and len(pos_b_skew[-2]) == 2 and pos_b_skew[-2][1] == y_hi:
                        if len(pos_b_skew[-1]) == 2 and pos_b_skew[-1][1] == y_hi:
                            pos.append(y_above + (((k - 1)/2)*pas_off))
                        elif len(pos_b_skew[-1]) == 2 and pos_b_skew[-1][1] == y_lo:
                            pos.append(y_above + (((k)/2)*pas_off))
                elif j == 1:
                    if len(pos_b_skew) == 0:
                        pos.append(y_lo)
                    elif len(pos_b_skew) == 1 or len(pos_b_skew) == 2:
                        pos.append(y_hi)
                    elif len(pos_b_skew) > 2 and len(pos_b_skew[-1]) == 2:
                        if len(pos_b_skew[-2]) == 2 and pos_b_skew[-2][1] == y_hi:
                            pos.append(y_lo)
                        elif len(pos_b_skew[-2]) == 2 and pos_b_skew[-2][1] == y_lo:
                            pos.append(y_hi)
                j += 1
        i += 1
        pos_b_skew.append(pos)
        if len(pos_b_skew) > 2 and pos_b_skew[-1] == pos_b_skew[-2]:
            del pos_b_skew[-1]
    return pos_b_skew

def generate_positions_t_skew():
    x_lo,x_hi,y_lo,y_hi = create_offset()
    x_passes, y_passes = hypotenuse_number_passes()
    x_pas = (2*x_passes) - 1
    y_pas = (2*y_passes) - 1
    dx, dy, pas_off = pas_offset()
    
    pos_bl_corner = generate_positions_bl_corner()
    pos_t_skew = []
    i = x_pas
    while i <= x_pas + y_pas:
        if i >= x_pas and i < y_pas:
            pos = []
            j = 0
            k = i - x_pas
            while j < 2:
                x_beyond = pos_bl_corner[-1][0] + pas_off
                if j == 0:
                    if len(pos_t_skew) == 0:
                        pos.append(x_hi)
                    elif len(pos_t_skew) == 1 or len(pos_t_skew) == 2:
                        pos.append(x_lo)
                    elif len(pos_t_skew) > 2 and len(pos_t_skew[-1]) == 2:
                        if len(pos_t_skew[-2]) == 2 and pos_t_skew[-2][0]== x_hi:
                            pos.append(x_lo)
                        elif len(pos_t_skew[-2]) == 2 and pos_t_skew[-2][0] == x_lo:
                            pos.append(x_hi)
                elif j == 1:
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
                j += 1
        i += 1
        pos_t_skew.append(pos)
        if len(pos_t_skew) > 2 and pos_t_skew[-1] == pos_t_skew[-2]:
            del pos_t_skew[-1]
        if len(pos_t_skew) > 2 and pos_t_skew[0][1] < y_lo:
            pos_t_skew[0][1] = y_lo
        for h in range(len(pos_t_skew)):
            if pos_t_skew[h][1] > y_hi:
                del pos_t_skew[h:]
    return pos_t_skew

def generate_positions_tr_corner():
    x_lo,x_hi,y_lo,y_hi = create_offset()
    x_passes, y_passes = hypotenuse_number_passes()
    x_pas = (2*x_passes) - 1
    y_pas = (2*y_passes) - 1
    dx, dy, pas_off = pas_offset()
    
    pos_tr_corner = []
    i = x_pas
    k = 0
    while i <= x_pas + y_pas + 1:
        if i >= x_pas and i >= y_pas:
            pos = []
            j = 0
            while j < 2:
                if j == 0:
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
                            pos.append(x_hi - (((k)/2)*pas_off))
                    elif len(pos_tr_corner) > 2 and len(pos_tr_corner[-1]) == 2 and pos_tr_corner[-1][0] != x_hi:
                        if len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] != x_hi:
                            pos.append(x_hi)
                        elif len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] == x_hi:
                            pos.append(x_hi - (((k + 1)/2)*pas_off))
                elif j == 1:
                    if len(pos_tr_corner) == 0 or len(pos_tr_corner) == 1:
                        pos.append(y_hi)
                    elif len(pos_tr_corner) == 2:
                        pos.append(y_hi - pas_off)
                    elif len(pos_tr_corner) > 2 and len(pos_tr_corner[-1]) == 2 and pos_tr_corner[-1][0] == x_hi:
                        if len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] != x_hi:
                            pos.append(y_hi - (((k + 1)/2)*pas_off))
                        elif len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] == x_hi:
                            pos.append(y_hi)
                    elif len(pos_tr_corner) > 2 and len(pos_tr_corner[-1]) == 2 and pos_tr_corner[-1][0] != x_hi:
                        if len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] != x_hi:
                            pos.append(y_hi - (((k)/2)*pas_off))
                        elif len(pos_tr_corner[-2]) == 2 and pos_tr_corner[-2][0] == x_hi:
                            pos.append(y_hi)
                j += 1
            k += 1
            pos_tr_corner.append(pos)
        i += 1
        if len(pos_tr_corner) > 2 and pos_tr_corner[-1] == pos_tr_corner[-2]:
            del pos_tr_corner[-1]
        if len(pos_tr_corner) > 2 and pos_tr_corner[0][1] < y_lo:
            pos_tr_corner[0][1] = y_lo
        for h in range(len(pos_tr_corner)):
            if pos_tr_corner[h][1] < y_lo:
                del pos_tr_corner[h:]
    if len(pos_tr_corner) > 2 and pos_tr_corner[-1] != [x_hi,y_lo]:
        pos_tr_corner.append([x_hi,y_lo])
    inverted_pos_tr_corner = pos_tr_corner[::-1]
    return inverted_pos_tr_corner

#generate_positions_bl_corner()
#generate_positions_b_skew()
#generate_positions_t_skew()
#generate_positions_tr_corner()

def combine_positions_b_skew():
    pos_bl_corner = generate_positions_bl_corner()
    pos_b_skew = generate_positions_b_skew()
    inverted_pos_tr_corner = generate_positions_tr_corner()

    positions = []
    
    positions.append(pos_bl_corner)
    positions.append(pos_b_skew)
    positions.append(inverted_pos_tr_corner)
    print(positions)
    return positions

def combine_positions_t_skew():
    pos_bl_corner = generate_positions_bl_corner()
    pos_t_skew = generate_positions_t_skew()
    inverted_pos_tr_corner = generate_positions_tr_corner()

    positions = []
    
    positions.append(pos_bl_corner)
    positions.append(pos_t_skew)
    positions.append(inverted_pos_tr_corner)
    #print(positions)
    return positions

def combine_positions_no_skew():
    pos_bl_corner = generate_positions_bl_corner()
    inverted_pos_tr_corner = generate_positions_tr_corner()
    positions = []
    
    positions.append(pos_bl_corner)
    positions.append(inverted_pos_tr_corner)
    print(positions)
    return positions

#combine_positions_b_skew()
#combine_positions_t_skew()
#combine_positions_no_skew()

def final_diagonal_raster_positions():
    x_lo,x_hi,y_lo,y_hi = create_offset()
    dx, dy, pas_off = pas_offset()

    print('pas_off =', pas_off)

    if x_hi == y_hi:
        combine_positions_no_skew()
    elif x_hi > y_hi:
        combine_positions_b_skew()
    elif x_hi < y_hi:
        combine_positions_t_skew()
    
final_diagonal_raster_positions()