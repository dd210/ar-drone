
"""
Python library for the AR.Drone.
This module was tested with Python 2.7 and AR.Drone 2.0
"""

import socket
import struct
import sys
import threading
import time
import numpy as np

__author__ = ".."

ARDRONE_NAVDATA_PORT = 5554
ARDRONE_VIDEO_PORT = 5555
ARDRONE_COMMAND_PORT = 5556
ARDRONE_CONTROL_PORT = 5559

SESSION_ID = "943aaaaa"
USER_ID = "363aaaaa"
APP_ID = "21daaaaa"


class ARDrone(object):
    """ARDrone Class.
    Instanciate this class to control your drone and receive decoded video and
    navdata.
    Possible value for video codec (drone2):
      NULL_CODEC    = 0,
      UVLC_CODEC    = 0x20,       // codec_type value is used for START_CODE
      P264_CODEC    = 0x40,
      MP4_360P_CODEC = 0x80,
      H264_360P_CODEC = 0x81,
      MP4_360P_H264_720P_CODEC = 0x82,
      H264_720P_CODEC = 0x83,
      MP4_360P_SLRS_CODEC = 0x84,
      H264_360P_SLRS_CODEC = 0x85,
      H264_720P_SLRS_CODEC = 0x86,
      H264_AUTO_RESIZE_CODEC = 0x87,    // resolution is automatically adjusted according to bitrate
      MP4_360P_H264_360P_CODEC = 0x88,
    """

    def __init__(self, img_init):     
        self.seq_nr = 1
        self.timer_t = 0.1
        self.com_watchdog_timer = threading.Timer(self.timer_t, self.commwdg)
        self.lock = threading.Lock()
        self.speed = 0.075
        time.sleep(0.1)
        self.config_ids_string = [SESSION_ID, USER_ID, APP_ID]
        self.configure_multisession(SESSION_ID, USER_ID, APP_ID, self.config_ids_string)
        self.set_session_id (self.config_ids_string, SESSION_ID)
        time.sleep(0.1)
        self.set_profile_id(self.config_ids_string, USER_ID)
        time.sleep(0.5)
        self.set_app_id(self.config_ids_string, APP_ID)
        time.sleep(0.1)
        self.set_video_bitrate_control_mode(self.config_ids_string, "1")
        time.sleep(0.1)
        self.set_video_bitrate(self.config_ids_string, "10000")
        time.sleep(0.1)
        self.set_max_bitrate(self.config_ids_string, "10000")
        time.sleep(0.1)
        self.set_fps(self.config_ids_string, "24")
        time.sleep(0.1)
        self.set_video_channel(self.config_ids_string, "0")
        time.sleep(0.1)
        self.set_video_codec(self.config_ids_string, 0x87)
        time.sleep(0.1)
        self.at(at_config_ids , self.config_ids_string)
        time.sleep(0.1)        
        self.at(at_config,"video:camif_buffers","1")
        time.sleep(0.5)  
        self.at(at_config_ids , self.config_ids_string)        
        self.at(at_config, "general:navdata_demo", "TRUE")       
        self.navdata = dict()
        self.navdata[0] = dict(zip(['ctrl_state', 'battery', 'theta', 'phi', 'psi', 'altitude', 'vx', 'vy', 'vz', 'num_frames'], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]))
        self.time = 0
        self.altitude = 1.0
        self.angle = 0
        self.way = 0
        #for croc
        self.image = img_init
        self.data_cross = [0,[640,400]]
        
    def takeoff(self):
        """Make the drone takeoff."""
        self.at(at_ftrim)
        self.at(at_config, "control:altitude_max", "20000")
        self.at(at_ref, True)

    def land(self):
        """Make the drone land."""
        self.at(at_ref, False)

    def hover(self):
        """Make the drone hover."""
        self.at(at_pcmd, False, 0, 0, 0, 0)

    def move_left(self):
        """Make the drone move left."""
        self.at(at_pcmd, True, -self.speed, 0, 0, 0)

    def move_right(self):
        """Make the drone move right."""
        self.at(at_pcmd, True, self.speed, 0, 0, 0)

    def move_up(self):
        """Make the drone rise upwards."""
        self.at(at_pcmd, True, 0, 0, self.speed, 0)

    def move_down(self):
        """Make the drone decent downwards."""
        self.at(at_pcmd, True, 0, 0, -self.speed, 0)

    def move_forward(self):
        """Make the drone move forward."""
        self.at(at_pcmd, True, 0, -self.speed, 0, 0)

    def move_backward(self):
        """Make the drone move backwards."""
        self.at(at_pcmd, True, 0, self.speed, 0, 0)

    def turn_left(self):
        """Make the drone rotate left."""
        self.at(at_pcmd, True, 0, 0, 0, -self.speed)

    def turn_right(self):
        """Make the drone rotate right."""
        self.at(at_pcmd, True, 0, 0, 0, self.speed)

    def reset(self):
        """Toggle the drone's emergency state."""
        self.at(at_ref, False, True)
        self.at(at_ref, False, False)

    def trim(self):
        """Flat trim the drone."""
        self.at(at_ftrim)

    def set_speed(self, speed):
        """Set the drone's speed.

        Valid values are floats from [0..1]
        """
        self.speed = speed

    def at(self, cmd, *args, **kwargs):
        """Wrapper for the low level at commands.

        This method takes care that the sequence number is increased after each
        at command and the watchdog timer is started to make sure the drone
        receives a command at least every second.
        """
        self.lock.acquire()
        self.com_watchdog_timer.cancel()
        cmd(self.seq_nr, *args, **kwargs)
        self.seq_nr += 1
        self.com_watchdog_timer = threading.Timer(self.timer_t, self.commwdg)
        self.com_watchdog_timer.start()
        self.lock.release()

    def configure_multisession(self, session_id, user_id, app_id, config_ids_string):
        self.at(at_config, "custom:session_id", session_id)
        self.at(at_config, "custom:profile_id", user_id)
        self.at(at_config, "custom:application_id", app_id)

    def set_session_id (self, config_ids_string, session_id):
        self.at(at_config_ids , config_ids_string)
        self.at(at_config, "custom:session_id", session_id)

    def set_profile_id (self, config_ids_string, profile_id):
        self.at(at_config_ids , config_ids_string)
        self.at(at_config, "custom:profile_id", profile_id)

    def set_app_id (self, config_ids_string, app_id):
        self.at(at_config_ids , config_ids_string)
        self.at(at_config, "custom:application_id", app_id)

    def set_video_bitrate_control_mode (self, config_ids_string, mode):
        self.at(at_config_ids , config_ids_string)
        self.at(at_config, "video:bitrate_control_mode", mode)

    def set_video_bitrate (self, config_ids_string, bitrate):
        self.at(at_config_ids , config_ids_string)
        self.at(at_config, "video:bitrate", bitrate)

    def set_video_channel(self, config_ids_string, channel):
        self.at(at_config_ids , config_ids_string)
        self.at(at_config, "video:video_channel", channel)

    def set_max_bitrate(self, config_ids_string, max_bitrate):
        self.at(at_config_ids , config_ids_string)
        self.at(at_config, "video:max_bitrate", max_bitrate)

    def set_fps (self, config_ids_string, fps):
        self.at(at_config_ids , config_ids_string)
        self.at(at_config, "video:codec_fps", fps)

    def set_video_codec (self, config_ids_string, codec):
        self.at(at_config_ids , config_ids_string)
        self.at(at_config, "video:video_codec", codec)

    def commwdg(self):
        """Communication watchdog signal.

        This needs to be send regulary to keep the communication w/ the drone
        alive.
        """
        self.at(at_comwdg)

    def halt(self):
        """Shutdown the drone.
        This method does not land or halt the actual drone, but the
        communication with the drone. You should call it at the end of your
        application to close all sockets, pipes, processes and threads related
        with this object.
        """
        self.lock.acquire()
        self.com_watchdog_timer.cancel()      
        self.com_pipe.send('die!')
        self.network_process.terminate()
        self.network_process.join()
        self.ipc_thread.stop()
        self.ipc_thread.join()
        self.lock.release()

    def switch_to_front(self, config_ids_string):     
        self.set_video_channel(config_ids_string,"0")
        self.set_video_codec(config_ids_string, 0x87)             

    def switch_to_back(self, config_ids_string):
        self.set_video_channel(config_ids_string,"1")
        self.set_video_codec(config_ids_string, 0x87)        

    def calibrate(self, config_ids_string):
        self.at(at_config_ids , config_ids_string)
        self.at(at_calib, 0)


###############################################################################
### Low level AT Commands
###############################################################################

def at_ref(seq, takeoff, emergency=False):
    """
    Basic behaviour of the drone: take-off/landing, emergency stop/reset)
    Parameters:
    seq -- sequence number
    takeoff -- True: Takeoff / False: Land
    emergency -- True: Turn of the engines
    """
    p = 0b10001010101000000000000000000
    if takeoff:
        p += 0b1000000000
    if emergency:
        p += 0b0100000000
    at("REF", seq, [p])

def at_pcmd(seq, progressive, lr, fb, vv, va):
    """
    Makes the drone move (translate/rotate).

    Parameters:
    seq -- sequence number
    progressive -- True: enable progressive commands, False: disable (i.e.
        enable hovering mode)
    lr -- left-right tilt: float [-1..1] negative: left, positive: right
    rb -- front-back tilt: float [-1..1] negative: forwards, positive:
        backwards
    vv -- vertical speed: float [-1..1] negative: go down, positive: rise
    va -- angular speed: float [-1..1] negative: spin left, positive: spin 
        right

    The above float values are a percentage of the maximum speed.
    """
    p = 1 if progressive else 0
    at("PCMD", seq, [p, float(lr), float(fb), float(vv), float(va)])

def at_ftrim(seq):
    """
    Tell the drone it's lying horizontally.

    Parameters:
    seq -- sequence number
    """
    at("FTRIM", seq, [])

def at_config(seq, option, value):
    """Set configuration parameters of the drone."""
    at("CONFIG", seq, [str(option), str(value)])

def at_config_ids(seq, value):
    """Set configuration parameters of the drone."""
    at("CONFIG_IDS", seq, value)

def at_ctrl(seq, num):
    """Ask the parrot to drop its configuration file"""
    at("CTRL", seq, [num, 0])
    
def at_calib(seq, num):
    "Ask drone to calibrate magnetometer"
    at("CALIB", seq, [num])

def at_comwdg(seq):
    """
    Reset communication watchdog.
    """
    at("COMWDG", seq, [])

def at(command, seq, params):
    """
    Parameters:
    command -- the command
    seq -- the sequence number
    params -- a list of elements which can be either int, float or string
    """
    param_str = ''
    for p in params:
        if type(p) == int:
            param_str += ",%d" % p
        elif type(p) == float:
            param_str += ",%d" % f2i(p)
        elif type(p) == str:
            param_str += ',"'+p+'"'
    msg = "AT*%s=%i%s\r" % (command, seq, param_str)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(msg.encode("utf-8"), ("192.168.1.1", ARDRONE_COMMAND_PORT))

def f2i(f):
    """Interpret IEEE-754 floating-point value as signed integer.
    Arguments:
    f -- floating point value
    """
    return struct.unpack('i', struct.pack('f', f))[0]

###############################################################################
### navdata
###############################################################################

def decode_navdata(packet):
    """Decode a navdata packet."""
    offset = 0
    _ =  struct.unpack_from("IIII", packet, offset)
    drone_state = dict()
    drone_state['fly_mask']             = _[1]       & 1 # FLY MASK : (0) ardrone is landed, (1) ardrone is flying
    drone_state['video_mask']           = _[1] >>  1 & 1 # VIDEO MASK : (0) video disable, (1) video enable
    drone_state['vision_mask']          = _[1] >>  2 & 1 # VISION MASK : (0) vision disable, (1) vision enable */
    drone_state['control_mask']         = _[1] >>  3 & 1 # CONTROL ALGO (0) euler angles control, (1) angular speed control */
    drone_state['altitude_mask']        = _[1] >>  4 & 1 # ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
    drone_state['user_feedback_start']  = _[1] >>  5 & 1 # USER feedback : Start button state */
    drone_state['command_mask']         = _[1] >>  6 & 1 # Control command ACK : (0) None, (1) one received */
    drone_state['fw_file_mask']         = _[1] >>  7 & 1 # Firmware file is good (1) */
    drone_state['fw_ver_mask']          = _[1] >>  8 & 1 # Firmware update is newer (1) */
    drone_state['fw_upd_mask']          = _[1] >>  9 & 1 # Firmware update is ongoing (1) */
    drone_state['navdata_demo_mask']    = _[1] >> 10 & 1 # Navdata demo : (0) All navdata, (1) only navdata demo */
    drone_state['navdata_bootstrap']    = _[1] >> 11 & 1 # Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
    drone_state['motors_mask']          = _[1] >> 12 & 1 # Motor status : (0) Ok, (1) Motors problem */
    drone_state['com_lost_mask']        = _[1] >> 13 & 1 # Communication lost : (1) com problem, (0) Com is ok */
    drone_state['vbat_low']             = _[1] >> 15 & 1 # VBat low : (1) too low, (0) Ok */
    drone_state['user_el']              = _[1] >> 16 & 1 # User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
    drone_state['timer_elapsed']        = _[1] >> 17 & 1 # Timer elapsed : (1) elapsed, (0) not elapsed */
    drone_state['angles_out_of_range']  = _[1] >> 19 & 1 # Angles : (0) Ok, (1) out of range */
    drone_state['ultrasound_mask']      = _[1] >> 21 & 1 # Ultrasonic sensor : (0) Ok, (1) deaf */
    drone_state['cutout_mask']          = _[1] >> 22 & 1 # Cutout system detection : (0) Not detected, (1) detected */
    drone_state['pic_version_mask']     = _[1] >> 23 & 1 # PIC Version number OK : (0) a bad version number, (1) version number is OK */
    drone_state['atcodec_thread_on']    = _[1] >> 24 & 1 # ATCodec thread ON : (0) thread OFF (1) thread ON */
    drone_state['navdata_thread_on']    = _[1] >> 25 & 1 # Navdata thread ON : (0) thread OFF (1) thread ON */
    drone_state['video_thread_on']      = _[1] >> 26 & 1 # Video thread ON : (0) thread OFF (1) thread ON */
    drone_state['acq_thread_on']        = _[1] >> 27 & 1 # Acquisition thread ON : (0) thread OFF (1) thread ON */
    drone_state['ctrl_watchdog_mask']   = _[1] >> 28 & 1 # CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
    drone_state['adc_watchdog_mask']    = _[1] >> 29 & 1 # ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
    drone_state['com_watchdog_mask']    = _[1] >> 30 & 1 # Communication Watchdog : (1) com problem, (0) Com is ok */
    drone_state['emergency_mask']       = _[1] >> 31 & 1 # Emergency landing : (0) no emergency, (1) emergency */
    data = dict()
    data['drone_state'] = drone_state
    data['header'] = _[0]
    data['seq_nr'] = _[2]
    data['vision_flag'] = _[3]
    offset += struct.calcsize("IIII")
    while 1:
        try:
            id_nr, size =  struct.unpack_from("HH", packet, offset)
            offset += struct.calcsize("HH")
        except struct.error:
            break
        values = []
        for i in range(size-struct.calcsize("HH")):
            values.append(struct.unpack_from("c", packet, offset)[0])
            offset += struct.calcsize("c")
        # navdata_tag_t in navdata-common.h
        if id_nr == 0:
            values = struct.unpack_from("IIfffIfffI", "".join(values))
            values = dict(zip(['ctrl_state', 'battery', 'theta', 'phi', 'psi', 'altitude', 'vx', 'vy', 'vz', 'num_frames'], values))
            # convert the millidegrees into degrees and round to int, as they
            # are not so precise anyways
            for i in 'theta', 'phi', 'psi':
                values[i] = int(values[i] / 1000)
                #values[i] /= 1000
        data[id_nr] = values
    return data