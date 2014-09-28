"""
Autopilot for AR Drone
"""
import pygame
import libardrone
import cv2
import numpy as np
import image
import socket
import threading
import select
import multiprocessing
import time
import thread
import Queue
import video_processing
import drone_movement
import pilot




#MK_class for reading navigational data
class reading_nav_data:
    def __init__(self):
        self._running = True  
          
    def terminate(self):
        self._running = False
           
    def run(self, out, socket):
        while self._running:
            try:
                #magic line - eliminate freezes
                time.sleep(0.001)   
                data = socket.recv(65535)
                if out.full() != True:
                    out.put(data)             
            except socket.timeout(0.025):
                continue
        return

    def decode(self, q_nav, frame, s, time1, time2, cross_program, frame_cross_counter, cross_state):
        #MK_default nav data
        #cross_state = [False, [0,0]]
        cross_past_states = []
        cross_past_states_x = []
        cross_past_states_y = []
        str_battery = "battery level: " + str("")
        str_altitude = "altitude: " + str("")        
        str_vx = "x-velocity: " + str("")
        str_vy = "y-velocity: " + str("")
        str_vz = "z-velocity: " + str("")
        str_s = "integrated way: " + str("")
        str_time = "time: " + str("")
        str_phi = "phi: " + str("")
        str_psi = "psi: " + str("")
        str_theta = "theta: " + str("")
        str_cross = "cross identification: " + str("")
        str_frames = "frame_cross_counter: " + str("")
        if q_nav.full():
            nav_info = q_nav.get()
            nav_info = libardrone.decode_navdata(nav_info)
            try:
                battery_state = nav_info[0]['battery']
                altitude_state = 0.001 * nav_info[0]['altitude'] 
                vx_state = 0.001 * nav_info[0]['vx']
                vy_state = 0.001 * nav_info[0]['vy']
                vz_state = 0.001 * nav_info[0]['vz']
                phi_state = nav_info[0]['phi']
                psi_state = nav_info[0]['psi']
                theta_state = nav_info[0]['theta']
                s = s + (((time2 - time1)*vx_state)**2 + ((time2 - time1)*vy_state)**2)**0.5
                #MK_printing navigation state                                        
                str_battery = "battery level: " + str(battery_state)
                str_altitude = "altitude: " + str(altitude_state)        
                str_vx = "x-velocity: " + str(vx_state)
                str_vy = "y-velocity: " + str(vy_state)
                str_vz = "z-velocity: " + str(vz_state)
                str_s = "integrated way: " + str(s)
                str_time = "time: " + str(time1)
                str_phi = "phi: " + str(phi_state)
                str_psi = "psi: " + str(psi_state)
                str_theta = "theta: " + str(theta_state)                
                str_cross = "cross identification: " + str(cross_program)
                str_frames = "frame_cross_counter: " + str(frame_cross_counter)
            except:
                print "Fail to decode nav_data.."
        cv2.putText(frame,str(str_battery),(20,20), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_altitude),(20,40), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))        
        cv2.putText(frame,str(str_vx),(20,60), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_vy),(20,80), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_vz),(20,100), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_s),(20,120), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_time),(20,140), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))   
        cv2.putText(frame,str(str_phi),(20,160), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_psi),(20,180), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_theta),(20,200), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))                             
        cv2.putText(frame,str(str_cross),(20,220), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))                    
        cv2.putText(frame,str(str_frames),(20,240), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))  
        #print cross_state[1][0],cross_state[1][1]             
        cv2.circle(frame, (cross_state[1][0], cross_state[1][1]), 10, (255, 0, 0), -1)  
        return frame, s

#MK_class for reading video data
class reading_video_data:
    def __init__(self):
        self._running = True  
          
    def terminate(self):
        self._running = False   

    def run(self, out, video_ch, drone):
        while video_ch.isOpened():
            try:
                #MK_capturing the very last frame from buffer that may contain few images
                #for frames in video_ch.read():
                #magic line - eliminate freezes
                time.sleep(0.001)   
                frame = video_ch.read()
                #print "f", frame
                frame2 = frame[1]
                drone.image = frame2
                if out.full() != True:
                    out.put(frame2)
            except socket.timeout(0.025):
                continue
        return

    def decode(self, q_vid, frame, frame_0, frame_1, frame_counter, timer, time_vid_proc_1):
        state = False
        if q_vid.full() and frame_counter > 5:   
            #print  q_vid.get() 
            state = True                           
            frame = q_vid.get() 
            frame_0 = frame.copy()
            frame_1 = frame.copy() 
            if timer - time_vid_proc_1 > 0.5:
                video_processing_time = timer - time_vid_proc_1
                print "delay in video processing:", video_processing_time, "; number of frame:", frame_counter, "; time:", timer
        time_vid_proc_1 = timer
        return frame, frame_0, frame_1, time_vid_proc_1, state 

    #MK_special function for converting images from OpenCV format to pygame format
    def cvimage_to_pygame(self, image):
        #Convert cvimage into a pygame image
        return pygame.image.frombuffer(image.tostring(), image.shape[1::-1],"RGB")
        

#MK_cross
class reading_cross_data:
    def __init__(self):
        self._running = True         
    def terminate(self):
        self._running = False   
    def run(self, out1, out2, drone):
        while self._running:
            #magic line - eliminate freezes
            time.sleep(0.001)   
            if out1.full():
                image = out1.get()
                #cv2.imwrite("image_cross.jpg", image)
                #cv2.imshow("image from drone", image)
                cross = video_processing.Frame_processing(image)   
                state, coords = cross.cross_detection()                
                data_cross = [state, coords]
                if out2.full():
                    out2.get()
                    out2.put(data_cross)
                else:
                    out2.put(data_cross)
                if data_cross[0] == 0:
                    drone.data_cross = [0,[640,400]]
                else:
                    drone.data_cross = data_cross
                cross.kill()          
        return 
   
def main():         
    #MK_initiation of pygame, setting display
    pygame.init()
    W, H = 1280, 720
    clock = pygame.time.Clock()    
    screen = pygame.display.set_mode((W, H))
    img_init = np.zeros((H, W, 3), np.uint8)
    #MK_creating AR Drone object (described in libardrone library)    
    drone = libardrone.ARDrone(img_init) 
    #MK_capturing video - creating connection by OpenCV tools  
    cap = cv2.VideoCapture("tcp://192.168.1.1:5555")
    if cap.isOpened():
        print "video connection established.."
    #MK_creation of socket for connecting to nav port
    nav_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #MK_sending msg to the Drone in order to start receive messages
    nav_socket.sendto("\x01\x00\x00\x00", ('192.168.1.1', 5554))
    #MK_creating queue for threads (nav and video)
    q_nav = Queue.Queue(1)
    q_vid = Queue.Queue(1)
    #MK_creating threads
    nav_data = reading_nav_data()
    video_data = reading_video_data()
    nav_thread = threading.Thread(target = nav_data.run, args = (q_nav, nav_socket))
    video_thread = threading.Thread(target = video_data.run, args = (q_vid, cap, drone))
    #MK_launching threads
    nav_thread.start()
    video_thread.start()
    time.sleep(0.001)       
    #MK_cross - creating queue
    q_cross_out = Queue.Queue(1)
    q_cross_in = Queue.Queue(1)
    cross_data = reading_cross_data()
    cross_thread = threading.Thread(target = cross_data.run, args = (q_cross_in, q_cross_out, drone))
    cross_thread.start()     
    #MK_counters and initialization
    s = 0.0
    timer = 0.0
    s_stage = 0.0
    timer_stage = 0.0
    time1 = 0.0
    time2 = 0.0
    time_vid_proc_1 = 0.0
    running = True
    cross_program = False
    frame_counter = 0
    frame_cross_counter = 0
    frame = np.zeros((H, W, 3), np.uint8)
    frame_0 = np.zeros((H, W, 3), np.uint8)
    frame_1 = np.zeros((H, W, 3), np.uint8)
    states = np.array([(pilot.autopilot.number, pilot.autopilot.s_stage, pilot.autopilot.timer_stage)])
    #creating pilot
    drone_pilot = pilot.autopilot()
    while running:
        frame_counter += 1
        #MK_reading video data from queue      
        timer = time.clock() 
        state = False
        frame, frame_0, frame_1, time_vid_proc_1, state  = video_data.decode(q_vid, frame, frame_0, frame_1, frame_counter, timer, time_vid_proc_1)            
        #MK_reading navigation data from queue and calculating integrated trajectory
        time2 = time.clock()
        frame, s = nav_data.decode(q_nav, frame, s, time1, time2, cross_program, frame_cross_counter, drone.data_cross)  
        #print "cross", drone.data_cross
        time1 = time.clock()      
        #decoding image to pygame format and putting captured frame on the screen and update (that's two consequent actions)                              
        pygame_img = video_data.cvimage_to_pygame(frame)
        screen.blit(pygame_img,(0,0))
        pygame.display.flip()
        clock.tick(50)
        pygame.display.set_caption("FPS: %.2f" % clock.get_fps())     



        #*********************AUTOPILOT PART************************
        freq = 5
        if frame_counter % freq == 0 and state == True:
            q_cross_in.put(frame_1) 


        last_command = drone_pilot.command
        temp_state = drone_pilot.state
        drone_pilot.update_state(drone)          
        lines, columns = states.shape
        if temp_state == drone_pilot.state and lines > 1:            
            states[lines-1, 1] =  s - states[0:lines-1, 1].sum()
            states[lines-1, 2] =  timer - states[0:lines-1, 2].sum()
        elif temp_state != drone_pilot.state:
            states = np.vstack((states, (states[lines-1, 0]+1, 0.0, 0.0)))
        elif temp_state == drone_pilot.state and lines == 1:
            states[lines-1, 1] =  s
            states[lines-1, 2] =  timer
        lines, columns = states.shape
        pilot.autopilot.number = states[lines-1, 0]
        pilot.autopilot.s_stage = states[lines-1, 1]
        pilot.autopilot.timer_stage = states[lines-1, 2]

        if last_command == drone_pilot.command: #and timer % 1 < 0.1:
            pass
        else:
            if drone_pilot.command == "drone.takeoff":
                drone.takeoff()
            elif drone_pilot.command == "drone.hover":
                drone.hover()
            elif drone_pilot.command == "drone.land":
                drone.land()
            elif drone_pilot.command == "drone.movef":
                drone.move_forward()
            elif drone_pilot.command == "drone.turnl":
                drone.turn_left()
            elif drone_pilot.command == "drone.movel":
                drone.move_left()
            elif drone_pilot.command == "drone.mover":
                drone.move_right()
            print "timer:", timer, "timer_stage:", pilot.autopilot.timer_stage, "command:", drone_pilot.command


        #***********************************************************
        
           
                                                                                   
        #MK_cycle for processing events     
        for event in pygame.event.get():           
            if event.type == pygame.QUIT:
                running = False                
            elif event.type == pygame.KEYUP:
                drone.hover()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    drone.reset()
                    running = False
                # takeoff / land
                elif event.key == pygame.K_RETURN:
                    drone.takeoff()
                elif event.key == pygame.K_SPACE:
                    drone.land()
                # emergency
                elif event.key == pygame.K_BACKSPACE:
                    drone.reset()
                # forward / backward
                elif event.key == pygame.K_w:
                    drone.move_forward()
                elif event.key == pygame.K_s:
                    drone.move_backward()
                # left / right
                elif event.key == pygame.K_a:
                    drone.move_left()
                elif event.key == pygame.K_d:
                    drone.move_right()
                # up / down
                elif event.key == pygame.K_UP:
                    drone.move_up()
                elif event.key == pygame.K_DOWN:
                    drone.move_down()
                # turn left / turn right
                elif event.key == pygame.K_LEFT:
                    drone.turn_left()
                elif event.key == pygame.K_RIGHT:
                    drone.turn_right()
                # speed
                elif event.key == pygame.K_1:
                    drone.speed = 0.1
                elif event.key == pygame.K_2:
                    drone.speed = 0.2
                elif event.key == pygame.K_3:
                    drone.speed = 0.3
                elif event.key == pygame.K_4:
                    drone.speed = 0.4
                elif event.key == pygame.K_5:
                    drone.speed = 0.5
                elif event.key == pygame.K_6:
                    drone.speed = 0.6
                elif event.key == pygame.K_7:
                    drone.speed = 0.7
                elif event.key == pygame.K_8:
                    drone.speed = 0.8
                elif event.key == pygame.K_9:
                    drone.speed = 0.9
                elif event.key == pygame.K_0:
                    drone.speed = 1.0
                #MK_add condition for quit                
                elif event.key == pygame.K_q:
                    drone.land()
                    running = False
                #MK_save image
                elif event.key == pygame.K_i:
                    photo_name = "C:/Python27/Lib/site-packages/python-ardrone-master_MK/images/frame_{}.jpg".format(frame_counter)
                    cv2.imwrite(photo_name, frame_0)                   
                #MK_add condition for switch video channels to front camera
                elif event.key == pygame.K_f:
                    drone.switch_to_front(drone.config_ids_string)   
                #MK_add condition for switch video channels to bottom camera
                elif event.key == pygame.K_b:
                    drone.switch_to_back(drone.config_ids_string)   
                elif event.key == pygame.K_c:
                    print "calibration initiated.."
                    drone.calibrate(drone.config_ids_string)

    #MK_closing sockets and threads
    nav_data.terminate()
    video_data.terminate()
    time.sleep(0.1)
    cap.release()
    nav_socket.close()
    print "Shutting down...",
    print "Ok."

if __name__ == '__main__':
    main()








