# -*- coding: utf-8 -*-

"""

-------------------------------------------------

@ Author ：Xiao Fabo

@ email ：2285273839@qq.com

@ Date ： 2021/7/11

@ Description：The code that controls the car

-------------------------------------------------

"""

import pygame

import numpy as np
import os



import sys



import threading



import time



import RPi.GPIO as GPIO



import Adafruit_PCA9685



import keyboard



import cv2



from pynput.keyboard import Controller, Key, Listener

# GPIO interface settings



pygame.mixer.init()

# The infrared sensor in front of the left

infrared_sensors_detector_front_left = 12



# The infrared sensor in front of the right

infrared_sensors_detector_front_right = 16



Ultrasonic_sensors_1_Trig = 4  # Ultrasonic sensor1 input



Ultrasonic_sensors_1_Echo = 17  # Ultrasonic sensor1 output



PWMA = 18



AIN1 = 22



AIN2 = 27



PWMB = 23



BIN1 = 25



BIN2 = 24

# The status parameter

# part 1  Immutable parameters
range_speed_of_straight = [20, 80]  # The range of straight speeds

range_speed_of_turn = [15, 75]  # The range of steering speed

range_angle1 = [-90, 90]  # Angle range1

range_angle2 = [-50, 55]  # Angle range2

horizion_screen = 320

vertical_screen = 240

horizion_angle_status_position = (

    int(horizion_screen/2), int(vertical_screen*2/5))



vertical_angle_status_position = (

    int(horizion_screen/2), int(vertical_screen*2/5-20))



distance_status_position = (int(horizion_screen/5), int(vertical_screen/5))



font = cv2.FONT_HERSHEY_SIMPLEX



font_color = (0, 255, 0)  # green



font_thickness = 1

track=True#判断是否开启追踪模式

control_key = ["w", "s", "a", "d", "r", "q", "e",

               "z", "c", "i", "k", "l", "j", "p", "u", "o","m","n"]

#           ["0","1","2","3","4","5","6","7","8","9","10","11","12","13","14","15","16","17"]

# control_key=[up,down,left,right,stop,decrease the speed of staright,increase the speed of staright,decrease the speed of turn,increase the speed of turn,rudder up,rudder down,rudder right,rudder left,stop rudder,decrease the speed of rudder,increase the speed of rudder,music,auto track]

velocity = 340  # The speed of the sound,used for ultrasonic ranging



period_rudder=0.05



period_show_status=0.1





# part 2  Variable parameters



global infrared_sensors_detector_front_right_status



global infrared_sensors_detector_front_left_status



global speed_of_straight



global speed_of_turn



global angle1



global angle2



global angle_step

global xxx #在水平方向上的差距

global yyy #在竖直方向上的差距

global distance_1

xxx = 0

yyy = 0


global pressed



# The status of the right front detector. 1 indicates the presence of obstacles and 0 means there are no obstacles

infrared_sensors_detector_front_right_status = 0



# The status of the left front detector.1 indicates the presence of obstacles and 0 means there are no obstacles

infrared_sensors_detector_front_left_status = 0



speed_of_straight = 50  # The speed of the straight line



speed_of_turn = 50  # The speed of the steering



angle1 = 0  # Horizontal



angle2 = 0  # Vertical



angle_step = 2  # The magnitude of the angle change

#w wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwewwweeeeeeeeeeeeeeeeeeeqqqqqqqqqqeeedddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddwwawwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwiuokhwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwawwwwawwwwwwwwwwwwwwwwwawwwwwwwwwwwwwwwwaaawwwwwwwwwsssssssssssssssssssssssssssssssssssaaaaaaaaaaaaaaaaaaaawwaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaawsssssssssssssssssssssawdwwwwwwwwwwwwededdwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwdeeesddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddwsssssssssssssssssssaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaddddddddddddddddddddddddddddddddddddddddddddddddddddddddsssssssdawwwwwwwwwwwwwwwwwadqqsssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssseeeewwwwwwwwwwwwwwwwwwwweeeeeeeeeeeadawwwwwwwwwwwwwdsaassssssssssssssssss

distance_1 = 0  # The distance measured by the ultrasonic sensor



pressed = []  # Record the key that has been pressed



# Initialize GPIO



GPIO.setwarnings(False)  # TWWWWSAAWWWWSWSSSSSSWWWWWWWWWSWWWWWAAADDADAWSSWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWurn off alerts



GPIO.setmode(GPIO.BCM)  # Set the pin number to BCM encoding





# Initialize the rudder



servo_pwm = Adafruit_PCA9685.PCA9685()



servo_pwm.set_pwm_freq(60)  # Set the frequency to 60



# Initialize the infrared sensor



GPIO.setup(infrared_sensors_detector_front_left, GPIO.IN)



GPIO.setup(infrared_sensors_detector_front_right, GPIO.IN)



# Initialize the ultrasonic sensor



GPIO.setup(Ultrasonic_sensors_1_Echo, GPIO.OUT, initial=GPIO.LOW)



# Initialize the camera



cap = cv2.VideoCapture(-1)



cap.set(3, horizion_screen)



cap.set(4, vertical_screen)

pygame.mixer.init()



# Initialize the motor



GPIO.setup(AIN2, GPIO.OUT)



GPIO.setup(AIN1, GPIO.OUT)



GPIO.setup(PWMA, GPIO.OUT)



GPIO.setup(BIN1, GPIO.OUT)



GPIO.setup(BIN2, GPIO.OUT)



GPIO.setup(PWMB, GPIO.OUT)



L_Motor = GPIO.PWM(PWMA, 100)



L_Motor.start(0)



R_Motor = GPIO.PWM(PWMB, 100)



R_Motor.start(0)

def nothing(x):

    pass

cv2.namedWindow("Tracking", cv2.WINDOW_NORMAL)

cv2.createTrackbar("RH1", "Tracking", 0, 255, nothing)

cv2.createTrackbar("RH2", "Tracking", 18, 255, nothing)


cv2.createTrackbar("GH1", "Tracking", 48, 255, nothing)

cv2.createTrackbar("GH2", "Tracking", 77, 255, nothing)


cv2.createTrackbar("BH1", "Tracking", 100, 255, nothing)

cv2.createTrackbar("BH2", "Tracking", 124, 255, nothing)


cv2.createTrackbar("min_radius", "Tracking", 10, 300, nothing)

cv2.createTrackbar("max_radius", "Tracking", 50, 300, nothing)


cv2.createTrackbar("param1", "Tracking", 100, 500, nothing)

cv2.createTrackbar("param2", "Tracking", 40, 300, nothing)


cv2.createTrackbar("factor_1", "Tracking", 100, 200, nothing)

cv2.createTrackbar("factor_2", "Tracking", 100, 200, nothing)



def detect_obstacles():

    """Detect obstacles and modify global variables: \n

    infrared_sensors_detector_front_right_status,\n

    infrared_sensors_detector_front_left_status\n

    """

    global infrared_sensors_detector_front_right_status, infrared_sensors_detector_front_left_status



    while 1:



        infrared_sensors_detector_front_right_status = (

            not GPIO.input(infrared_sensors_detector_front_left))*1



        infrared_sensors_detector_front_left_status = (

            not GPIO.input(infrared_sensors_detector_front_right))*1



        time.sleep(0.1)





def horizontal_rudder(angle=0, speed=1, channel=5, start=373, angle_rate=2.80):



    # The default channel is 5;



    # When the angle changes from -90 to 90,the third parameter changes from 110 to 615. 615-110=505 505/180=2.80,so the angle rate is 2.80.



    # After testing,it is the best when the starting position is 373



    servo_pwm.set_pwm(channel, 0, start+int(angle*angle_rate))





def vertical_rudder(angle=0, speed=1, channel=6,start=320, angle_rate=-2.333):



    # The default channel is 4;



    servo_pwm.set_pwm(channel, 0, start+int(angle*angle_rate))





def conrtrol_horizontal_rudder():



    while 1:



        horizontal_rudder(angle1)





def conrtrol_vertical_rudder():



    while 1:



        vertical_rudder(angle2)





def t_left(speed, t_time):



    L_Motor.ChangeDutyCycle(speed)



    GPIO.output(AIN2,False)  # AIN2



    GPIO.output(AIN1,True)  # AIN1



    R_Motor.ChangeDutyCycle(speed)



    GPIO.output(BIN2, False)  # BIN2



    GPIO.output(BIN1, True)  # BIN1



    time.sleep(t_time)





def t_stop(t_time):



    L_Motor.ChangeDutyCycle(0)



    GPIO.output(AIN2, False)  # AIN2



    GPIO.output(AIN1, False)  # AIN1



    R_Motor.ChangeDutyCycle(0)



    GPIO.output(BIN2, False)  # BIN2



    GPIO.output(BIN1, False)  # BIN1



    time.sleep(t_time)



def t_right(speed, t_time):



    L_Motor.ChangeDutyCycle(speed)



    GPIO.output(AIN2, True)  # AIN2



    GPIO.output(AIN1, False)  # AIN1



    R_Motor.ChangeDutyCycle(speed)



    GPIO.output(BIN2, True)  # BIN2



    GPIO.output(BIN1, False)  # BIN1



    time.sleep(t_time)





def t_up(speed, t_time):

    L_Motor.ChangeDutyCycle(speed)



    GPIO.output(AIN2, True)  # AIN2



    GPIO.output(AIN1, False)  # AIN1



    R_Motor.ChangeDutyCycle(speed)



    GPIO.output(BIN2, False)  # BIN2



    GPIO.output(BIN1, True)  # BIN1



    time.sleep(t_time)


def t_down(speed, t_time):



    L_Motor.ChangeDutyCycle(speed)



    GPIO.output(AIN2, False)  # AIN2



    GPIO.output(AIN1, True)  # AIN1



    R_Motor.ChangeDutyCycle(speed)



    GPIO.output(BIN2, True)  # BIN2



    GPIO.output(BIN1, False)  # BIN1



    time.sleep(t_time)





def on_press(key):

    global pressed

    

        

    try:

        key_=key.char

        if (key.char not in pressed):



            pressed.append(key.char)

    except:

        key_=key.name

            

        if (key.name not in pressed):

            pressed.append(key.name)



    #print(pressed,end="+\n")



def on_release(key):



    global pressed



    try:

        

        try:

            key_=key.char

            if (key.char in pressed):

                pressed.remove(key.char)

        except:

            key_=key.name

            

            if (key.name in pressed):

                pressed.remove(key.name)

    except:

        pass



    if key == Key.esc:

        return False

    #print(pressed)





def start_listen():



    with Listener(on_press=on_press, on_release=on_release) as listener:



        listener.join()





def show_keyboard():

    '''

    This function is used to test on a local computer to listen for keyboard

    '''

    while 1:

        print(pressed)

        print(angle1, angle2)

        time.sleep(0.05)

        os.system('cls')





# Controls the distance measured by the ultrasonic module



def checkdist():

    '''

    Ultrasonic ranging function\n

        returns ::(float)  The distance in centimeters

    '''



    GPIO.setup(Ultrasonic_sensors_1_Trig, GPIO.OUT, initial=GPIO.LOW)



    GPIO.setup(Ultrasonic_sensors_1_Echo, GPIO.IN)



    GPIO.output(Ultrasonic_sensors_1_Trig, GPIO.HIGH)



    time.sleep(0.00015)



    GPIO.output(Ultrasonic_sensors_1_Trig, GPIO.LOW)



    while not GPIO.input(Ultrasonic_sensors_1_Echo):



        pass



    t1 = time.time()



    while GPIO.input(Ultrasonic_sensors_1_Echo):



        pass



    t2 = time.time()



    return (t2-t1)*velocity*100/2

def findcircle(shape_circles, color_circles, min_r, max_r, factor_1, factor_2):

    min_factor = 1000000

    results = [horizion_screen/2, vertical_screen/2, 10]

    try:

        for s_circle in shape_circles[0]:

            s_x = s_circle[0]

            s_y = s_circle[1]

            s_r = s_circle[2]

            if min_r < s_r < max_r:

                try:

                    for c_circle in color_circles:

                        (c_x, c_y), c_r = cv2.minEnclosingCircle(c_circle)

                        if min_r < c_r < max_r:

                            d_center2 = (s_x-c_x)*(s_x-c_x)+(s_y-c_y)*(s_y-c_y)

                            dr2 = (s_r-c_r)*(s_r-c_r)

                            now_factor = factor_1*d_center2+factor_2*dr2

                            if now_factor < min_factor:

                                min_factor = now_factor

                                results = [c_x, c_y, c_r]

                                # if d_center2<5 and dr2<10:

                                #    return results

                except:

                    return [horizion_screen/2, vertical_screen/2, 10]

    except:

        return [horizion_screen/2, vertical_screen/2, 10]

    return results





# Update distances in real time

def refresh_distance():



    global distance_1



    while True:



        distance_1 = checkdist()



        print(distance_1)



        time.sleep(0.75)




def back():
    global angle1,angle2
    if angle1>0:
        angle1=0
        t_right(50,angle1/37)#经过测试37度每秒

    if angle1<0:
        angle1=0
        t_left(50,-angle1/37)
    if distance_1<20:
        t_down(50,1)
    if distance_1>50:
        t_up(50,1)
    # Camera screen

def video():
    global track
    
    global xxx

    global yyy

    global angle1

    global angle2

    num = 1
    
    while True:
        start = time.time()
        cap
        ret, frame = cap.read()
        if track:

            RH1 = cv2.getTrackbarPos("RH1", "Tracking")

            RH2 = cv2.getTrackbarPos("RH2", "Tracking")

            GH1 = cv2.getTrackbarPos("GH1", "Tracking")

            GH2 = cv2.getTrackbarPos("GH2", "Tracking")

            BH1 = cv2.getTrackbarPos("BH1", "Tracking")

            BH2 = cv2.getTrackbarPos("BH2", "Tracking")

            min_radius = cv2.getTrackbarPos("min_radius", "Tracking")

            max_radius = cv2.getTrackbarPos("max_radius", "Tracking")

            param1_ = cv2.getTrackbarPos("param1", "Tracking")

            param2_ = cv2.getTrackbarPos("param2", "Tracking")

            factor_1 = cv2.getTrackbarPos("factor_1", "Tracking")*0.01

            factor_2 = cv2.getTrackbarPos("factor_2", "Tracking")*0.01

            ball_blue_lower = np.array([int(BH1), 43, 46])  # 100-124

            ball_blue_upper = np.array([int(BH2), 255, 255])

            ball_red_lower = np.array([int(RH1), 43, 46])  # 175-195 success

            ball_red_upper = np.array([int(RH2), 255, 255])

            ball_green_lower = np.array([int(GH1), 43, 46])  # 35-77

            ball_green_upper = np.array([int(GH2), 255, 255])

            

            img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            frame = cv2.GaussianBlur(frame, (5, 5), 0)

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, ball_blue_lower, ball_blue_upper)

            mask = cv2.erode(mask, None, iterations=2)

            mask = cv2.dilate(mask, None, iterations=2)

            mask = cv2.GaussianBlur(mask, (3, 3), 0)

            res = cv2.bitwise_and(frame, frame, mask=mask)

            cnts = cv2.findContours(
                mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            mask2 = cv2.inRange(hsv, ball_red_lower, ball_red_upper)

            mask2 = cv2.erode(mask2, None, iterations=2)

            mask2 = cv2.dilate(mask2, None, iterations=2)

            mask2 = cv2.GaussianBlur(mask2, (3, 3), 0)

            res2 = cv2.bitwise_and(frame, frame, mask=mask2)

            cnts2 = cv2.findContours(mask2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            mask3 = cv2.inRange(hsv, ball_green_lower, ball_green_upper)

            mask3 = cv2.erode(mask3, None, iterations=2)

            mask3 = cv2.dilate(mask3, None, iterations=2)

            mask3 = cv2.GaussianBlur(mask3, (3, 3), 0)

            res3 = cv2.bitwise_and(frame, frame, mask=mask3)

            cnts3 = cv2.findContours(mask3.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # 发现小球

            circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 90, param1=param1_,
                                       param2=param2_, minRadius=min_radius, maxRadius=max_radius)

            if len(cnts) > 0:

                circle1 = findcircle(circles, cnts, min_radius,
                                     max_radius, factor_1, factor_2)

                cv2.circle(frame, (int(circle1[0]), int(circle1[1])), int(
                    circle1[2]), (255, 0, 0), 2)

                cv2.putText(frame, "%.0f,%.0f,%.0f" % (circle1[0], circle1[1], circle1[2]), (int(
                    circle1[0]), int(circle1[1])), font, 0.5, (6, 255, 255), 1)  # 将帧数印在屏幕上

            if len(cnts2) > 0:

                circle2 = findcircle(circles, cnts2, min_radius,
                                     max_radius, factor_1, factor_2)

                cv2.circle(frame, (int(circle2[0]), int(circle2[1])), int(
                    circle2[2]), (0, 0, 255), 2)

                cv2.putText(frame, "%.0f,%.0f,%.0f" % (circle2[0], circle2[1], circle2[2]), (int(
                    circle2[0]), int(circle2[1])), font, 0.5, (6, 255, 255), 1)  # 将帧数印在屏幕上

            if len(cnts3) > 0:

                circle3 = findcircle(circles, cnts3, min_radius,
                                     max_radius, factor_1, factor_2)

                cv2.circle(frame, (int(circle3[0]), int(circle3[1])), int(
                    circle3[2]), (0, 255, 0), 2)

                cv2.putText(frame, "%.0f,%.0f,%.0f" % (circle3[0], circle3[1], circle3[2]), (int(
                    circle3[0]), int(circle3[1])), font, 0.5, (6, 255, 255), 1)  # 将帧数印在屏幕上

            # d1=(circle1[0]-circle2[0])*(circle1[0]-circle2[0])+(circle1[1]-circle2[1])*(circle1[1]-circle2[1])

            # d2=(circle1[0]-circle3[0])*(circle1[0]-circle3[0])+(circle1[1]-circle3[1])*(circle1[1]-circle3[1])

            # d3=(circle3[0]-circle2[0])*(circle3[0]-circle2[0])+(circle3[1]-circle2[1])*(circle3[1]-circle2[1])

            #

            # d1=math.sqrt(d1)

            # d2=math.sqrt(d2)

            # d3=math.sqrt(d3)

            try:

                center = [(circle1[0]+circle2[0]+circle3[0])/3,
                          (circle1[1]+circle2[1]+circle3[1])/3]

                cv2.circle(frame, (int(center[0]), int(
                    center[1])), 10, (0, 255, 0), 2)

                xxx = center[0]-horizion_screen/2

                yyy = vertical_screen/2-center[1]

                if num % 2 == 0:

                    if xxx > 20:

                        angle1 -= 1.5

                    if xxx < -20:

                        angle1 += 1.5

                    if yyy > 20:

                        angle2 += 1.5

                    if yyy < -20:

                        angle2 -= 1.5
                if num %10==0:
                    threading.Thread(target=back).start()
                    
            except:

                pass

        cv2.putText(frame, "%f cm" % distance_1, distance_status_position, font,0.5, font_color, font_thickness)  # Show the distance on the screen

        if angle1 == 0:

            cv2.putText(frame, "-", horizion_angle_status_position,

                        font, 0.5, font_color, font_thickness)

        elif angle1 > 0:

            cv2.putText(frame, "<-%.1f\'" % (abs(angle1)),

                        horizion_angle_status_position, font, 0.5, font_color, font_thickness)

        else:

            cv2.putText(frame, "%.1f\'->" % (abs(angle1)),

                        horizion_angle_status_position, font, 0.5, font_color, font_thickness)



        if angle2 == 0:

            cv2.putText(frame, "-", vertical_angle_status_position,

                        font, 0.5, font_color, font_thickness)

        elif angle2 > 0:

            cv2.putText(frame, "+%.1f\'" % (abs(angle2)),

                        vertical_angle_status_position, font, 0.5, font_color, font_thickness)

        else:

            cv2.putText(frame, "|%.1f\'" % (abs(angle2)),

                        vertical_angle_status_position, font, 0.5, font_color, font_thickness)

        num += 1
        
        # cv2.circle(frame,(int(circle3[0]),int(circle3[1])),int(circle3[2]),(0,255,0),2)

        end = time.time()

        during = end-start

        cv2.putText(frame, "FPS=%.0f" % (1/during),
                    (10, 230), font, 0.5, (6, 255, 255), 2)

        cv2.line(frame, (int(horizion_screen/2-10), int(vertical_screen/2)),
                 (int(horizion_screen/2+10), int(vertical_screen/2)), (0, 0, 0), 2)

        cv2.line(frame, (int(horizion_screen/2), int(vertical_screen/2-10)),
                 (int(horizion_screen/2), int(vertical_screen/2+10)), (0, 0, 0), 2)

        cv2.imshow('frame', frame)
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break

    cap.release()

    cv2.destroyAllWindows()





def show_status():



    # display the status on the terminal



    global angle1, angle2, range_angle1, range_angle2, angle_step, barrier_left, barrier_right, speed_of_straight, speed_of_turn



    while 1:

        print("\033c")

        print("angle1=  %.2f,  angle2=%.2f, angle_step= %d " %

              (angle1, angle2, angle_step))



        print("barrier_left: %d ,barrier_right %d " % (

            infrared_sensors_detector_front_right_status, infrared_sensors_detector_front_left_status))



        print("speed of Straight = %d speed of turn = %d " %

              (speed_of_straight, speed_of_turn))

        print("distance: %.2f"%distance_1)
        if track:
            print("auto tracking is rrunning")


        time.sleep(period_show_status)



def control_rudder():



    global angle1, angle2



    while 1:



        if control_key[9] in pressed:



            angle2 += angle_step



            if angle2 > range_angle2[1]:



                angle2 = range_angle2[1]



        if control_key[10] in pressed:



            angle2 -= angle_step



            if angle2 < range_angle2[0]:



                angle2 = range_angle2[0]



        if control_key[11] in pressed:



            angle1 -= angle_step



            if angle1 < range_angle1[0]:



                angle1 = range_angle1[0]



        if control_key[12] in pressed:



            angle1 += angle_step



            if angle1 > range_angle1[1]:



                angle1 = range_angle1[1]



        if control_key[13] in pressed:



            angle1 = 0



            angle2 = 0



        time.sleep(period_rudder)





def change_parameter():



    global angle_step, speed_of_straight, speed_of_turn, speed_of_straight, speed_of_turn



    while 1:



        if control_key[14] in pressed:



            if angle_step > 1:

                angle_step -= 0.5



        if control_key[15] in pressed:



            if angle_step < 10:

                angle_step += 0.5



        if control_key[5] in pressed:



            if speed_of_straight > range_speed_of_straight[0]:

                speed_of_straight -= 1



        if control_key[6] in pressed:



            if speed_of_straight < range_speed_of_straight[1]:

                speed_of_straight += 1



        if control_key[7] in pressed:



            if speed_of_turn > range_speed_of_turn[0]:

                speed_of_turn -= 1



        if control_key[8] in pressed:



            if speed_of_turn < range_speed_of_turn[1]:

                speed_of_turn += 1



        time.sleep(0.05)





def control_move():



    global speed_of_straight, speed_of_turn,track



    while 1:



        if control_key[0] in pressed:



            t_up(speed_of_straight, 0.1)



        if control_key[1] in pressed:



            t_down(speed_of_straight, 0.1)



        if control_key[2] in pressed:



            t_left(speed_of_turn, 0.1)



        if control_key[3] in pressed:



            t_right(speed_of_turn, 0.1)



        if (control_key[4] in pressed) or (not pressed):



            t_stop(0.05)



        if control_key[16] in pressed:

            if pygame.mixer.music.get_busy():

                pygame.mixer.music.pause()

            else:
                pygame.mixer.music.unpause() 
             
        if control_key[17] in pressed:
            track=not track





def print_pressed_keys(e):

    global pressed

    if (e.name not in pressed) and (e.event_type == "down"):

        pressed.append(e.name)

    if (e.name in pressed) and (e.event_type == "up"):

        pressed.remove(e.name)

    # print(pressed)





def listen_keyboard():



    keyboard.hook(print_pressed_keys)

    keyboard.wait()



def play_music():

    track = pygame.mixer.music.load(r"/home/pi/CLBROBOT/musics/OrphanKilla,广西鹿哥 - 草原最美的花 (Killa Extended).mp3")

    pygame.mixer.music.play()

    

kb = Controller()  # Listen to the keyboard





play_music()

job1 = threading.Thread(target=conrtrol_horizontal_rudder)  # Control rudder 1
job2 = threading.Thread(target=conrtrol_vertical_rudder)  # Control rudder 2
# Listen to the keyboard to give orders !!!!
job3 = threading.Thread(target=start_listen)
job4 = threading.Thread(target=control_rudder)  # control rudder
job5 = threading.Thread(target=video)  # Video
job6 = threading.Thread(target=refresh_distance)  # Measure the distance
job7 = threading.Thread(target=detect_obstacles)  # detect obstacles
job8 = threading.Thread(target=control_move)  # conrtol the car move or turn
# Modify variables such as speed of straight
job9 = threading.Thread(target=change_parameter)
job10 = threading.Thread(target=show_status)
job11 = threading.Thread(target=listen_keyboard)
job1.start()
print("horizontal_rudder is ready!")
job2.start()
print("vertical_rudder is ready!")
job6.start()
print("Ultrasonic ranging is ready!")
job5.start()
print("camera is ready!")
job7.start()
print("infrared sensor is ready!")
job4.start()
print("rudders is under control!")
job8.start()
print("motors is under controwl!")
job9.start()
print("Motion parameters can be modified!")
job3.start()
print("Control has been given to the keyboard,Please refer to the instructions for the operation method")
job10.start()
# job11.start()# Wait for all other threads to start before starting the control thread
