import numpy as np
import libardrone

OVERVIEW_VER = 90.0
OVERVIEW_HOR = 120.0
RES_X = 1280.0
RES_Y = 720.0


def image_point(image_x, image_y, altitude):
    angle_hor = np.arctan((image_x-RES_X/2)*np.tan(OVERVIEW_HOR/2)/(RES_X/2))
    #print np.arctan(RES_X-image_x)
    angle_ver = np.arctan((RES_Y-image_y)*np.tan(OVERVIEW_VER/2)/(RES_Y/2))
    length = np.tan(90-angle_ver)*altitude/np.cos(angle_hor)
    return angle_hor, angle_ver, length


def movement_to_point(drone, angle_hor):
    __running = True    
    angle = drone.angle + angle_hor
    #print "111", angle_hor
    
    while drone.angle < angle:
        print "111"
        drone.turn_left()
    drone.hover()


