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

def number_passes():
    x_lo,x_hi,y_lo,y_hi = create_offset()
    if (x_hi-x_lo)/hose_ex_d % 2 == 0:
        x_passes = (x_hi-x_lo)/hose_ex_d
    else:
        unround_x_passes = (x_hi-x_lo)/hose_ex_d
        x_passes = math.ceil(unround_x_passes)
        if x_passes % 2 == 1:
            x_passes += 1
    
    y_passes = (y_hi-y_lo)/hose_ex_d
#    print("x:" , x_passes , "and y:" , y_passes)
    return x_passes, y_passes
    
def x_pas_offset():
    x_passes = number_passes()
    x_pas = int(x_passes[0])
    x_lo,x_hi,y_lo,y_hi = create_offset()
    dx = float((x_hi-x_lo)/(x_passes[0]))
    return dx


def generate_positions_verticle_3():
    x_passes = number_passes()
    x_lo,x_hi,y_lo,y_hi = create_offset()
    x_pas = (2*int(x_passes[0])) - 1
    dx = x_pas_offset()
    
    positions = []
    i = 0
    print("change in x =" , dx)
    print('x_pas is',x_pas)
    print('x_lo',x_lo)
    print('x_hi',x_hi)
    print('y_lo',y_lo)
    print('y_hi',y_hi)
#    print(x_passes)
    while i <= (x_pas + 2):
        pos = []

        j = 0
        while j < 2:
            if j == 0:
                if i % 2 == 0:
                    pos.append(x_lo + ((i/2)*dx))
                else:
                    pos.append(x_lo + (((i-1)/2)*dx))
            else:
                if i % 2 == 0:
                    pos.append(y_lo)
                else:
                    pos.append(y_hi)
            j += 1

        if len(positions) > 0 and pos[0] == positions[-1][0]:
            if positions[-1][1] == y_lo:
                pos[1] = y_hi
            else:
                pos[1] = y_lo
        elif len(positions) > 0 and pos[0] != positions[-1][0]:
            pos[1] = positions[-1][1]

        positions.append(pos)
        print("even")
        i += 1

    return positions

generate_positions_verticle_3()

