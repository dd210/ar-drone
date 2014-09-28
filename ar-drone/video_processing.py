#! /usr/bin/env python
# -*- coding: utf-8 -*- 

import numpy as np
import cv2
from matplotlib import pyplot as plt
import math
import array

#//////////////////////////////////////////////////////////////////////////////////
class Frame_processing(object):

      CROSS_COLOR = 255 # BLACK CROSS
      PERIMETER_DEVIATION = 0.005
      MIN_CROSS_AREA = 0.0005
      MAX_CROSS_AREA = 0.1
      MAX_ANGLE_NUMB = 20
      MIN_ANGLE_NUMB = 8
      MAX_AREA_PERIMETER_RATIO = 10
      MIN_CONTAREA_HULLAREA_RATIO = 3
      ANGLE_RESOLUTION = 10

      def __init__(self, curr_img):
          
          self.__img = curr_img
          self.__height, self.__width, self.__depth = self.__img.shape
          self.__img_gray = cv2.cvtColor(self.__img, cv2.COLOR_RGB2GRAY)
#          clahe = cv2.createCLAHE(clipLimit = 20.0, tileGridSize = (5, 5))
#          self.__img_gray_equ = clahe.apply(self.__img_gray)
#          plt.imshow(self.__img_gray_equ), plt.show()
#          self.__img_gray_filt = cv2.inRange(self.__img_gray, 0, 175)
#          plt.imshow(self.__img_gray_filt), plt.show()
          self.__point_centre = [self.__height/2, self.__width/2]
          self.final_point_centre = [self.__height/2, self.__width/2]
          self.__point_length = []
          self.__angle_groups = []
          for i in range(0, 360/self.ANGLE_RESOLUTION):
              self.__angle_groups.append(0)


      def cross_detection(self):
          self.cross_detected = 0
          self.__img_thresh = cv2.threshold(self.__img_gray, 125, self.CROSS_COLOR, cv2.THRESH_OTSU)[1]
          self.__img_thresh = 255 - self.__img_thresh
#          cv2.imwrite('thresh.png', self.__img_thresh)
          self.__contours, self.__hierarchy = cv2.findContours(self.__img_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
          self.blank = np.zeros((self.__height, self.__width, 3), np.uint8)
          self.blank2 = np.zeros((self.__height, self.__width, 3), np.uint8)
          for cont_numb in range(0, len(self.__contours)):
             contour = self.__contours[cont_numb]
             counter = 0
             self.__approxCurve = cv2.approxPolyDP(contour, self.PERIMETER_DEVIATION * cv2.arcLength(contour, True), True)
             __approx_area_curve = cv2.contourArea(self.__approxCurve)
             if __approx_area_curve < 1:
               __approx_area_curve = 1
             __hull = cv2.convexHull(self.__approxCurve)
             __approx_area_hull = cv2.contourArea(__hull)
             if __approx_area_hull < 1:
                __approx_area_hull = 1
             cv2.drawContours( self.blank, [contour], -1, (0,0,255), 2)
             cv2.drawContours( self.blank, [self.__approxCurve], -1, (0,255,0), 2)
             cv2.drawContours( self.blank, [__hull], -1, (255, 0, 0), 2)
#             print __approx_area_curve, __approx_area_hull
             if __approx_area_curve < self.MIN_CROSS_AREA * self.__width * self.__height:
                continue 
             if __approx_area_curve > self.MAX_CROSS_AREA * self.__width * self.__height:
                continue 
             if len(self.__approxCurve) > self.MAX_ANGLE_NUMB:
                continue 
             if len(self.__approxCurve) < self.MIN_ANGLE_NUMB:
                continue 
             if __approx_area_hull / __approx_area_curve < self.MIN_CONTAREA_HULLAREA_RATIO:
                continue
             perimeter = cv2.arcLength(self.__approxCurve, True)
             if perimeter < 1:
                perimeter = 1
             if (__approx_area_curve)/perimeter > self.MAX_AREA_PERIMETER_RATIO:
                continue
             for i in range(0, 360/self.ANGLE_RESOLUTION):
                self.__angle_groups[i] = 0
             self.__point_centre = self.point_centre_detector(self.__approxCurve)
             for point_numb in range(0, len(self.__approxCurve)):
                x_point = self.__approxCurve[point_numb][0][0]
                y_point = self.__approxCurve[point_numb][0][1]
                self.__point_length.append(math.hypot(x_point - self.__point_centre[0], y_point - self.__point_centre[1]))
             length_max_position = np.argmax(self.__point_length)
             length_max = self.__point_length[length_max_position]
             x1_length_max = self.__approxCurve[length_max_position][0][0]
             y1_length_max = self.__approxCurve[length_max_position][0][1]
             for point_numb1 in range(0, len(self.__approxCurve)):
                if self.__point_length[point_numb1] >= 0.25 * length_max:
                   x1 = self.__approxCurve[point_numb1][0][0]
                   y1 = self.__approxCurve[point_numb1][0][1]
                   vec_dot_product = (float(x1_length_max) - float(self.__point_centre[0])
                                   ) * (float(x1) - float(self.__point_centre[0])) + (float(y1_length_max) - float(self.__point_centre[1])
                                   ) * (float(y1) - float(self.__point_centre[1]))
                   angle_between =  180 + 180 * np.arccos(vec_dot_product /(length_max * self.__point_length[point_numb1] + 0.0001))/np.pi
                   angle_group_numb = int(angle_between/self.ANGLE_RESOLUTION)
                   self.__angle_groups[angle_group_numb] = 1
             for point_numb in range(0, len(self.__approxCurve)):
                self.__point_length.pop(0)
             if sum(self.__angle_groups) >= 4:
                cv2.drawContours( self.blank2, [contour], -1, (0,0,255), 2)
                cv2.drawContours( self.blank2, [self.__approxCurve], -1, (0,255,0), 2)
                cv2.drawContours( self.blank2, [__hull], -1, (255, 0, 0), 2)
                self.cross_detected = 1
                self.final_point_centre = self.__point_centre
#          plt.imshow(self.blank), plt.show()
#          plt.imshow(self.blank2), plt.show()
          cv2.imwrite("image_cross.jpg", self.blank2)
          #print self.final_point_centre
          return self.cross_detected, self.final_point_centre #, self.blank2

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


#curr_img = cv2.imread('frame_1976852.jpg')
#Frame = Frame_processing(curr_img)
#cross_detected, point_centre = Frame.cross_detection()
#if cross_detected == True:
#  curr_img = Frame.cross_draw()
#plt.imshow(curr_img), plt.show()
