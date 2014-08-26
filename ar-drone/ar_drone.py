"""
Autopilot for AR Drone
"""


import pygame
import libardrone
import cv2
import numpy
import image
import socket
import threading
import select
import multiprocessing
import time
import thread
import Queue

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
            for frames in video_ch.read():
                frame = frames
            out.put(frame)
        return
    
def main():         
    #MK_initiation of pygame, setting display
    pygame.init()
    W, H = 1280, 720
    clock = pygame.time.Clock()    
    screen = pygame.display.set_mode((W, H))
    #MK_creating AR Drone object (described in libardrone library)    
    drone = libardrone.ARDrone()
    time.sleep(1)
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
    frame_counter = 0
    while running:
        frame_counter += 1
        #MK_reading video and navigation data from queue
        try:

            if q_nav.full():
                nav_info = q_nav.get()
            time3 = time.clock()
            if q_vid.full():                               
                frame = q_vid.get() 
                if time3 - time4 > 0.5:
                    video_processing_time = time3 - time4   
                    print video_processing_time,"_____", frame_counter,"_____", time3             
                time4 = time.clock()   
            nav_info = libardrone.decode_navdata(nav_info)
            battery_state = nav_info[0]['battery']
            altitude_state = 0.001 * nav_info[0]['altitude'] 
            vx_state = 0.001 * nav_info[0]['vx']
            vy_state = 0.001 * nav_info[0]['vy']
            vz_state = 0.001 * nav_info[0]['vz']
        except:
            pass
        #MK_printing navigation state               
        str_battery = "battery level: " + str(battery_state)
        str_altitude = "altitude: " + str(altitude_state)        
        str_vx = "x-velocity: " + str(vx_state)
        str_vy = "y-velocity: " + str(vy_state)
        str_vz = "z-velocity: " + str(vz_state)
        str_S = "integrated way: " + str(S)
        str_time = "time: " + str(time1)
        frame_0 = frame.copy()
        cv2.putText(frame,str(str_battery),(20,20), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_altitude),(20,40), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))        
        cv2.putText(frame,str(str_vx),(20,60), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_vy),(20,80), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_vz),(20,100), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_S),(20,120), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))
        cv2.putText(frame,str(str_time),(20,140), cv2.FONT_HERSHEY_PLAIN, 1.0,(0,255,0))                              
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




