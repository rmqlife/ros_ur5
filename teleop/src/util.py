import math
def deg2rad(deg):
    return [math.radians(degree) for degree in deg]


def rad2deg(radians):
    # Convert radians to degrees for each joint value
    return [math.degrees(rad) for rad in radians]