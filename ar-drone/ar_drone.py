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

#MK_special function for converting images from OpenCV format to pygame format
def cvimage_to_pygame(image):
    #Convert cvimage into a pygame image
    return pygame.image.frombuffer(image.tostring(), image.shape[1::-1],"RGB")

#MK_class for reading navigational data
class reading_nav_data:
    def __init__(self):
        self._running = True    
    def terminate(self):
        self._running = False   
    def run(self, out, socket):
        while self._running:
            try:
                data = socket.recv(65535)
                out.put(data)                
            except socket.timeout(0.1):
                continue
        return

#MK_class for reading video data
class reading_video_data:
    def __init__(self):
        self._running = True    
    def terminate(self):
        self._running = False   
    def run(self, out, video_ch):
        while video_ch.isOpened():
            #MK_capturing the very last frame from buffer that may contain few images
            #for frames in video_ch.read():
            #    frame = frames
            frame = video_ch.read()
            frame2 = frame[1]
            out.put(frame2)
            #print "1"
        return

#MK_cross
class reading_cross_data:
    def __init__(self):
        self._running = True         
    def terminate(self):
        self._running = False   
    def run(self, out1, out2):
        while self._running:
            time.sleep(0.01)   #magic line - eliminate freezes
            if out1.full():
                image = out1.get()
                cross = video_processing.Frame_processing(image)   
                state, coords, int_image = cross.cross_detection()
                data_cross = [state, coords, int_image]
                if out2.full():
                    out2.get()
                    out2.put(data_cross)
                else:
                    out2.put(data_cross)
                #time.sleep(0.1)
                cross.kill()
                #out2.put(coords)                
        return
    
def main():         
    #MK_initiation of pygame, setting display
    pygame.init()
    W, H = 1280, 720
    clock = pygame.time.Clock()    
    screen = pygame.display.set_mode((W, H))
    #MK_creating AR Drone object (described in libardrone library)    
    drone = libardrone.ARDrone()

    #MK_capturing video - creating connection by OpenCV tools  
    cap = cv2.VideoCapture("tcp://192.168.1.1:5555")
    if cap.isOpened():
        print "video connection established.."
    #MK_creation of socket for connecting to nav port
    nav_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #MK_assigning newly created socket to specific port (5554)
    #nav_socket.bind('', 5554)
    #MK_sending msg to the Drone in order to start receive messages
    nav_socket.sendto("\x01\x00\x00\x00", ('192.168.1.1', 5554))
    #MK_creating queue for threads
    q_nav = Queue.Queue(2)
    q_vid = Queue.Queue(2)
    #MK_creating threads
    nav_data = reading_nav_data()
    video_data = reading_video_data()
    nav_thread = threading.Thread(target = nav_data.run, args = (q_nav, nav_socket))
    video_thread = threading.Thread(target = video_data.run, args = (q_vid, cap))
    #MK_launching threads
    nav_thread.start()
    video_thread.start()
    time.sleep(0.2)   
    
    #MK_cross
    q_cross_out = Queue.Queue(1)
    q_cross_in = Queue.Queue(1)
    cross_data = reading_cross_data()
    cross_thread = threading.Thread(target = cross_data.run, args = (q_cross_in, q_cross_out))
    cross_thread.start()
     
    #MK_timer
    time1 = time.clock()
    #MK_integrated trajectory
    S = 0
    altitude_state = 0
    vx_state = 0
    vy_state = 0
    vz_state = 0
    video_processing_time = 0
    time3 = 0.0
    time4 = 0.0
    running = True
    #MK_counters
    cross_program = False
    frame_counter = 0
    frame_cross_counter = 0
    cross_state = [False, [0,0]]
    cross_past_states = []
    cross_past_states_x = []
    cross_past_states_y = []
    detected_frames = 3
    detected_frames_threshold = 2
    stdev_threshold = 500.0
    freq_video_processing = 2
    time.sleep(1)
    str_battery = "battery level: " + str("")
    str_altitude = "altitude: " + str("")        
    str_vx = "x-velocity: " + str("")
    str_vy = "y-velocity: " + str("")
    str_vz = "z-velocity: " + str("")
    str_S = "integrated way: " + str("")
    str_time = "time: " + str("")
    str_phi = "phi: " + str("")
    str_psi = "psi: " + str("")
    str_theta = "theta: " + str("")
    str_cross = "cross identification: " + str("")
    str_frames = "frame_cross_counter: " + str("")
    frame = np.zeros((H, W, 3), np.uint8)
    while running:
        frame_counter += 1
        #MK_cross
        if q_cross_out.full():  
            cross_program = False       
            cross_state = q_cross_out.get()
            frame_cross_counter += 1
            #cross_past_states [frame_cross_counter % 25] = cross_state[0]            
            #print cross_state[0]
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
        #MK_end of cross    

        #MK_reading video and navigation data from queue
        if q_nav.full():
            try:
                nav_info = q_nav.get()
                nav_info = libardrone.decode_navdata(nav_info)
                #print nav_info
                battery_state = nav_info[0]['battery']
                altitude_state = 0.001 * nav_info[0]['altitude'] 
                vx_state = 0.001 * nav_info[0]['vx']
                vy_state = 0.001 * nav_info[0]['vy']
                vz_state = 0.001 * nav_info[0]['vz']
                phi_state = nav_info[0]['phi']
                psi_state = nav_info[0]['psi']
                theta_state = nav_info[0]['theta']
                #MK_printing navigation state               
                str_battery = "battery level: " + str(battery_state)
                str_altitude = "altitude: " + str(altitude_state)        
                str_vx = "x-velocity: " + str(vx_state)
                str_vy = "y-velocity: " + str(vy_state)
                str_vz = "z-velocity: " + str(vz_state)
                str_S = "integrated way: " + str(S)
                str_time = "time: " + str(time1)
                str_phi = "phi: " + str(phi_state)
                str_psi = "psi: " + str(psi_state)
                str_theta = "theta: " + str(theta_state)
                str_cross = "cross identification: " + str(cross_program)
                str_frames = "frame_cross_counter: " + str(frame_cross_counter)
            except:
                print "Fail to decode nav_data.."

        time3 = time.clock()
        if q_vid.full() and frame_counter > 5:                               
            frame = q_vid.get() 
            frame_0 = frame.copy()
            frame_1 = frame.copy() 
            if frame_counter % freq_video_processing == 0:
                q_cross_in.put(frame_1)  
            if time3 - time4 > 0.5:
                video_processing_time = time3 - time4   
                print video_processing_time,"_____", frame_counter,"_____", time3             
            time4 = time.clock()   

        cv2.putText(frame,str(str_battery),(20,20), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_altitude),(20,40), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))        
        cv2.putText(frame,str(str_vx),(20,60), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_vy),(20,80), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_vz),(20,100), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_S),(20,120), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_time),(20,140), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))   
        cv2.putText(frame,str(str_phi),(20,160), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_psi),(20,180), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_theta),(20,200), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))                             
        cv2.putText(frame,str(str_cross),(20,220), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))                    
        cv2.putText(frame,str(str_frames),(20,240), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))              
        cv2.circle(frame, (cross_state[1][0], cross_state[1][1]), 10, (255, 0, 0), -1)                                          
        pygame_img = cvimage_to_pygame(frame)
        #MK_Putting captured frame on the screen and update (that's two consequent actions)
        screen.blit(pygame_img,(0,0))
        pygame.display.flip()
        clock.tick(50)
        pygame.display.set_caption("FPS: %.2f" % clock.get_fps())
        #MK_integrated trajectory        
        time2 = time.clock()
        S = S + (((time2 - time1)*vx_state)**2 + ((time2 - time1)*vy_state)**2)**0.5
        time1 = time.clock()       
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
    time.sleep(1)
    cap.release()
    nav_socket.close()
    print "Shutting down...",
    print "Ok."

if __name__ == '__main__':
    main()




