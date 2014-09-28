import numpy as np
import video_processing
import cv2


class autopilot:

    timer_stage = 0.0
    s_stage = 0.0
    number = 0
    cross = 0

    lock_timer = 0.0
    lock_state = False

    def __init__(self):
        self.command = ""
        self.state = self.wait
        self.running = True
        self.timer = 0.0
        self.stage_timer = 0.0
        self.way = 0.0
        self.direct_forward = True
        self.direct_backward = False
        self.direct_side = False
        self.can_move = True
        self.turn_way = False
        self.turn_hor_line = False
        self.lines_state = True
        self.lines = []
        self.white_cross_state = False
        self.white_cross = []
        self.black_cross_state = False
        self.black_cross = []

    def update_state(self, drone):
        self.state(drone)

    #specific program of the flight

    def wait(self, drone):
        if autopilot.timer_stage > 5.0:
            self.command = "drone.takeoff"
            self.state = self.hover1

    def hover1(self, drone):
        self.command = "drone.hover"
        if autopilot.timer_stage > 6.0:
            self.state = self.move_forward

    def move_forward(self, drone):
        self.command = "drone.movef"
        print autopilot.s_stage
        if autopilot.s_stage > 2.0:
            self.state = self.hover2

    def hover2(self, drone):
        self.command = "drone.hover"
        if autopilot.timer_stage > 4.0:
            self.state = self.turn_left
        
    def turn_left(self, drone):
        self.command = "drone.turnl"  
        if autopilot.timer_stage >= 7.5:               
            self.state = self.hover3

    def hover3(self, drone):
        self.command = "drone.hover"
        if autopilot.timer_stage > 3.0:
            self.state = self.turn_left2

    def turn_left2(self, drone):
        self.command = "drone.turnl"  
        if autopilot.timer_stage >= 7.5:               
            self.state = self.hover4

    def hover4(self, drone):
        self.command = "drone.hover"
        if autopilot.timer_stage > 2.0:
            self.state = self.move_forward2

    def move_forward2(self, drone):
        self.command = "drone.movef"
        if autopilot.timer_stage > 0.5:
            self.state = self.hover5

    def hover5(self, drone):
        self.command = "drone.hover"
        if autopilot.timer_stage > 2.0:
            self.state = self.search_cross

    def search_cross(self, drone):
        self.command = "drone.hover"
        #cv2.imshow("image from drone", drone.image)
        #print drone.data_cross
        threshold = 150
        W = 1280
        coord_x = drone.data_cross[1][0]
        coord_y = drone.data_cross[1][1]
        #print coord_x

        if (coord_x - W/2.0) <= -threshold:
            self.command = "drone.movel"
            autopilot.cross = 0
        elif (coord_x - W/2.0) >= threshold:
            self.command = "drone.mover"
            autopilot.cross = 0
        elif abs(coord_x - W/2.0) < threshold and drone.data_cross[0] == 1:
            autopilot.cross = autopilot.cross + 1
            if autopilot.cross == 2:
             self.command = "drone.hover"
             self.state = self.search_cross2

    def search_cross2(self, drone):
        print '123456789'
        self.command = "drone.hover"
        #print autopilot.s_stage
        if autopilot.timer_stage > 4.0:
            self.command = "drone.movef"
        if autopilot.timer_stage > 8.0:
            self.state = self.landing


    def landing(self, drone):
        self.command = "drone.land"



        """if autopilot.timer_stage > 25.0:
            self.state = self.landing















    def landing(self, drone):
        self.command = "drone.land"""




    """cross_program = False
    cross_state = [False, [0,0]]
    cross_past_states = []
    cross_past_states_x = []
    cross_past_states_y = []


    altitude_state = 0
    vx_state = 0
    vy_state = 0
    vz_state = 0
    video_processing_time = 0"""



    """#MK_cross
    #parameters for detecting cross
    detected_frames = 3
    detected_frames_threshold = 2
    stdev_threshold = 500.0
    freq_video_processing = 2
        if q_cross_out.full():  
            cross_program = False       
            cross_state = q_cross_out.get()
            frame_cross_counter += 1

            if  frame_cross_counter < detected_frames and cross_program == False:
                cross_past_states.append(int(cross_state[0]))
                cross_past_states_x.append(int(cross_state[1][0]))
                cross_past_states_y.append(int(cross_state[1][1]))
                deviation_x = np.std(cross_past_states_x)
                deviation_y = np.std(cross_past_states_y)
            elif frame_cross_counter >= detected_frames and cross_program == False:
                cross_past_states.pop(0)
                cross_past_states.append(int(cross_state[0]))
                cross_past_states_x.pop(0)
                cross_past_states_x.append(int(cross_state[1][0]))
                deviation_x = np.std(cross_past_states_x)
                cross_past_states_y.pop(0)
                cross_past_states_y.append(int(cross_state[1][1]))
                deviation_y = np.std(cross_past_states_y)
                #print cross_past_states,(deviation_x + deviation_y)
            if sum(cross_past_states) >= detected_frames_threshold and deviation_x + deviation_y <= stdev_threshold:
                cross_program = True
                cross_x = np.median(cross_past_states_x)
                cross_y = np.median(cross_past_states_y)
            if cross_program == True:
                angle_hor, angle_ver, length = drone_movement.image_point(cross_x, cross_y, drone.altitude)
                drone.speed = 0.15 * (abs(cross_x - 640.0)/640.0)**2
                #print angle_hor, cross_x, cross_y
                if angle_hor < 0:
                    drone.move_left()
                    pass
                else:
                    drone.move_right()
                    pass
            else:
                drone.hover()
                pass               
        #MK_end of cross    """