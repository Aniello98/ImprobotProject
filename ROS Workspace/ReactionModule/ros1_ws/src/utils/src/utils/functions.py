from math import atan2

def compute_orient(ax, ay):
    distance = ( ax**2 + ay**2 ) ** 0.5         # actor-robot distance
    angle = atan2(ay, ax)                       # actor-robot angle
    return distance, angle 
