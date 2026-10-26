import math


#works for any dimension: (x, y) or (x, y, z)
#math.dist raises ValueError if the two points have different dimensions
def euclidean_distance(pos1, pos2):
    return math.dist(pos1, pos2)
