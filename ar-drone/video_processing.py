#! /usr/bin/env python
# -*- coding: utf-8 -*- 

import numpy as np
import cv2
from matplotlib import pyplot as plt
import math
import cv2.cv as cv
import array

CROSS_COLOR = 255 # BLACK CROSS
PERIMETER_DEVIATION = 0.004
MIN_CROSS_AREA = 0.0002
MAX_CROSS_AREA = 0.3
MAX_ANGLE_NUMB = 21
MIN_ANGLE_NUMB = 8
MAX_AREA_PERIMETER_RATIO = 10
ANGLE_INTERVAL = [30, 150]

#//////////////////////////////////////////////////////////////////////////////////
class Frame_processing(object):

      def __init__(self, curr_img):
          self.__img = curr_img
          self.__height, self.__width, self.__depth = self.__img.shape
          self.__img_gray = cv2.cvtColor(self.__img, cv2.COLOR_RGB2GRAY)
          self.__point_centre = [0, 0]
          self.final_point_centre = [0, 0]
          self.__point_length = []

      def cross_detection(self):
          self.cross_detected = 0
          self.__img_thresh = cv2.threshold(self.__img_gray, 125, CROSS_COLOR, cv2.THRESH_BINARY)[1]
          self.__img_thresh = 255 - self.__img_thresh
#          cv2.imwrite('thresh.png', self.__img_thresh)
          self.__contours, self.__hierarchy = cv2.findContours(self.__img_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#          self.__blank = np.zeros((self.__height, self.__width, 3), np.uint8)
          self.blank2 = np.zeros((self.__height, self.__width, 3), np.uint8)
          for cont_numb in range(0, len(self.__contours)):
             contour = self.__contours[cont_numb]
             counter = 0
             self.__approxCurve = cv2.approxPolyDP(contour, PERIMETER_DEVIATION * cv2.arcLength(contour, True), True)
             self.__approx_area = cv2.contourArea(self.__approxCurve)
             if self.__approx_area < MIN_CROSS_AREA * self.__width * self.__height:
                continue 
             if self.__approx_area > MAX_CROSS_AREA * self.__width * self.__height:
                continue 
             if len(self.__approxCurve) > MAX_ANGLE_NUMB:
                continue 
             if len(self.__approxCurve) < MIN_ANGLE_NUMB:
                continue 
             perimeter = cv2.arcLength(self.__approxCurve, True)
             if perimeter < 1:
                perimeter = 1
             if (self.__approx_area)/perimeter > MAX_AREA_PERIMETER_RATIO:
                continue
#             counter = 4
             cv2.drawContours( self.blank2, [contour], -1, (0,0,255), 2)
             cv2.drawContours( self.blank2, [self.__approxCurve], -1, (0,255,0), 2)
             self.__point_centre = self.point_centre_detector(self.__approxCurve)
             for point_numb in range(0, len(self.__approxCurve)):
                x_point = self.__approxCurve[point_numb][0][0]
                y_point = self.__approxCurve[point_numb][0][1]
                self.__point_length.append (math.hypot(x_point - self.__point_centre[0], y_point - self.__point_centre[1]))
             length_max = self.__point_length[np.argmax(self.__point_length)]
             for point_numb1 in range(0, len(self.__approxCurve) - 1):
                x1 = self.__approxCurve[point_numb1][0][0]
                y1 = self.__approxCurve[point_numb1][0][1]
                for point_numb2 in range(point_numb1 + 1, len(self.__approxCurve)):
                    x2 = self.__approxCurve[point_numb2][0][0]
                    y2 = self.__approxCurve[point_numb2][0][1]
                    if self.__point_length[point_numb1] >= 0.3 * length_max:
                        if self.__point_length[point_numb2] >= 0.3 * length_max:
                            self.__angle1 = 180 * np.arctan((float(y1) - float(self.__point_centre[1]))/(
                                                float(x1) - float(self.__point_centre[0]) + 0.001))/np.pi
                            self.__angle2 = 180 * np.arctan((float(y2) - float(self.__point_centre[1]))/(
                                                float(x2) - float(self.__point_centre[0]) + 0.001))/np.pi
                            self.__angle12 = abs(self.__angle1 - self.__angle2)
                            if self.__angle12 >= ANGLE_INTERVAL[0]:
                                if self.__angle12 <= ANGLE_INTERVAL[1]:
                                   counter += 1
             if counter >= 4:
#                cv2.drawContours( self.__blank, [self.__approxCurve], -1, (0,255,0), 2)
#                cv2.circle(self.__img, (self.__point_centre[0], self.__point_centre[1]), 5, (255, 0, 0), -1)
                self.cross_detected = 1
                self.final_point_centre = self.__point_centre
#                break
#          plt.imshow(self.__blank2), plt.show()
          return self.cross_detected, self.final_point_centre, self.blank2

      def point_centre_detector(self, approxCurve):
          self.__point_centre = [0, 0]
          for point_numb in range(0, len(approxCurve)):
              self.__point_centre[0] += approxCurve[point_numb][0][0]
              self.__point_centre[1] += approxCurve[point_numb][0][1]
          self.__point_centre[0] = self.__point_centre[0]/len(self.__approxCurve)
          self.__point_centre[1] = self.__point_centre[1]/len(self.__approxCurve)
          return self.__point_centre
      
      def cross_draw(self):
          cv2.circle(self.__img, (self.final_point_centre[0], self.final_point_centre[1]), 10, (255, 0, 0), -1)
          info_string = ['CROSS DETECTED', ': ', '[', str(self.final_point_centre[0]), ', ', str(self.final_point_centre[1]), ']']
          cv2.putText(self.__img, ' '.join(info_string), (10,50), cv2.FONT_HERSHEY_PLAIN, 1.2, (0,255,0), 2)
          return self.__img

      def kill(self):
          del self

   
    
#////////////////////////////////////////////////////////////////////////////////////
"""
curr_img = cv2.imread('frame_1257.jpg')
Frame = Frame_processing(curr_img)
cross_detected, point_centre = Frame.cross_detection()
if cross_detected == True:
  curr_img = Frame.cross_draw()
plt.imshow(curr_img), plt.show()
"""