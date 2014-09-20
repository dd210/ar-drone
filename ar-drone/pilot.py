import numpy as np

class autopilot:

    timer_stage = 0.0
    s_stage = 0.0
    number = 0

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

    def update_state(self):
        self.state()


    def wait(self):
        if autopilot.timer_stage > 5.0:
            self.command = "drone.takeoff"
            self.state = self.ready

    def ready(self):
        self.command = ""
        if autopilot.timer_stage > 4.0:
            self.command = "drone.hover"
        if autopilot.timer_stage > 8.0:
            self.command = "drone.hover"
            self.state = self.move_forward
        pass

    def move_forward(self):
        self.command = "drone.movef"
        print autopilot.timer_stage
        if autopilot.timer_stage > 2.0:
            self.command = "drone.hover"
            self.state = self.landing


    def landing(self):
        self.command = "drone.hover"
        if autopilot.timer_stage > 2.0:
            self.command = "drone.land"




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