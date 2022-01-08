# x axis of the ball: x_ball
# y axis of the ball: y_ball
# height of the ball: height_ball
# width of the ball : width_ball
# home goal coordinates: (home_x,home_y)
# opp goal coordinate: (opp_x,opp_y)
# our robot coordinate : (hodor_x,hodor_y)
# opponent robot coordinate: (ene_x,ene_y)
# x axis of the ball from radio: ball_x
# y axis of the ball from radio: ball_y
#----------------------------------------------------#
import serial
import functools
import math
import numpy as np
ser = serial.Serial('/dev/ttyACM0',115200)
ser1 = serial.Serial('/dev/ttyACM1',38400)
s = [0,1]
z=[]
center_box_x = 263
center_box_y = 190
cx =0
cy=0
height=0
width=0
gx=0
gy=0
bx=0
by=0
yx=0
yy=0
orr = 1
def from_pixy(x_ball,y_ball,height_ball,width_ball):
    area = height_ball * width_ball
    if x_ball > -0.15 and x_ball < 0.15:
        if area > 1 or area < 90:
            ser.write(b'2\n')
            print("fwd")
            #verdict = forward
        else:
            ser.write(b'5\n')
            print("stop")
            #verdict = stop
    elif x_ball > 0.15 and x_ball < 1:
        ser.write(b'6\n')
        print("right")
        #verdict = right
    elif x_ball < -0.15 and x_ball > -1:
        ser.write(b'4\n')
        print("left")
        #verdict = left
    else:
        print("none")
    return
  
def distance(a_x1,a_y1,a_x2,a_y2):
    ax = a_x1 - a_x2
    ay = a_y1 - a_y2
    dist =  math.sqrt((ax*ax) + (ay*ay))
    return dist

def orientation():
    yaw = recieve_aurdino1()
    if yaw in range(-50,50)
        print("right orientation")
        orr = 1 
        
    elif dist_hodor_goal > dist_ball_goal:
        print("wrong orientation")
        global orr
        orr = -1
        wrong_orr(ball_y)
    return orr

def goal(button):
    if button == 1:
        #green
        opp_x = 508
        opp_y = 193
        our_color = 'green'
    elif button == 0:
        #blue
        opp_x = 18
        opp_y = 187
        our_color = 'blue'
    return opp_x,opp_y,our_color;

def score_yaw():
    
def score(hodor_center_dist,ball_center_dist,yaw):
    #yaw =  60
    # range 59 to 0 from center
    if hodor_center_dist < 59 and hodor_center_dist > 0:
        ball_center_dist = abs(ball_center_dist)
        hodor_center_dist = abs(hodor_center_dist)
        if ball_center_dist > hodor_center_dist:
            print("move left")
            #ser.write(b'4\n')
            if yaw < -1:
                print("rotate right a bit")
                #ser.write(b'68\n')
            elif yaw in range (-1,1):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw > 1:
                print("rotate left a bit")
                #ser.write(b'48\n')
        elif ball_center_dist < hodor_center_dist:
            print("move right")
            #ser.write(b'6\n')
            if yaw < -1:
                print("rotate right a bit")
                #ser.write(b'68\n')
            elif yaw in range (-1,1):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw > 1:
                print("rotate left a bit")
                #ser.write(b'48\n')
        elif ball_center_dist == hodor_center_dist:
            print("fwd")
            #ser.write(b'2\n')
    #range 2nd part of areana towards wall
    elif hodor_center_dist > 59 and hodor_center_dist < 92:
        ball_center_dist = abs(ball_center_dist)
        hodor_center_dist = abs(hodor_center_dist)
        if ball_center_dist > hodor_center_dist:
            print("move left")
            #ser.write(b'4\n')
            if yaw < 25:
                print("rotate right a bit")
                #ser.write(b'68\n')
            elif yaw in range(25,35):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw > 35:
                print("rotate left a bit")
                #ser.write(b'48\n')
        elif ball_center_dist < hodor_center_dist:
            if yaw < 25:
                print("rotate right a bit")
                #ser.write(b'68\n')
            elif yaw in range (25,35):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw > 35:
                print("rotate left a bit")
                #ser.write(b'48\n')
        elif ball_center_dist == hodor_center_dist:
            print("move left")
            #ser.write(b'4\n')
            if yaw < 25:
                print("rotate right a bit")
                #ser.write(b'68\n')
            elif yaw in range(25,35):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw > 35:
                print("rotate left a bit")
                #ser.write(b'48\n')
    #range 3rd part of arena towards wall
    elif hodor_center_dist > 59 and hodor_center_dist > 92:
        ball_center_dist = abs(ball_center_dist)
        hodor_center_dist = abs(hodor_center_dist)
        if ball_center_dist > hodor_center_dist:
            print("move left")
            #ser.write(b'4\n')
            if yaw < 55:
                print("rotate right a  bit")
                #ser.write(b'68\n')
            elif yaw in range(55,65):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw > 65:
                print("rotate left a bit")
                #ser.write(b'48\n')
        elif ball_center_dist < hodor_center_dist:
            if yaw < 55:
                print("rotate right a bit")
                #ser.write(b'68\n')
            elif yaw in range (55,65):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw > 65:
                print("rotate left a bit")
                #ser.write(b'48\n')
        elif ball_center_dist == hodor_center_dist:
            print("move left")
            #ser.write(b'4\n')
            if yaw < 55:
                print("rotate right a bit")
                #ser.write(b'68\n')
            elif yaw in range(55,65):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw > 65:
                print("rotate left a bit")
                #ser.write(b'48\n')
    #center range away from wall
    elif hodor_center_dist > -59 and hodor_center_dist < 0:
        ball_center_dist = abs(ball_center_dist)
        hodor_center_dist = abs(hodor_center_dist)
        if ball_center_dist > hodor_center_dist:
            print("move right")
            #ser.write(b'6\n')
            if yaw < -1:
                print("rotate left a bit")
                #ser.write(b'48\n')
            elif yaw in range (-1,1):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw > 1:
                print("rotate right a bit")
                #ser.write(b'68\n')
        elif ball_center_dist < hodor_center_dist:
            print("move left")
            #ser.write(b'4\n')
            if yaw < -1:
                print("rotate left a bit")
                #ser.write(b'48\n')
            elif yaw in range (-1,1):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw > 1:
                print("rotate right a bit")
                #ser.write(b'68\n')
        elif ball_center_dist == hodor_center_dist:
            print("fwd")
            #ser.write(b'2\n')
    #range 2 of arena away from wall
    elif hodor_center_dist < -59 and hodor_center_dist > -92:
        ball_center_dist = abs(ball_center_dist)
        hodor_center_dist = abs(hodor_center_dist)
        if ball_center_dist > hodor_center_dist:
            print("move right")
            #ser.write(b'6\n')
            if yaw > -25:
                print("rotate left a bit")
                #ser.write(b'48\n')
            elif yaw in range(-35,-25):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw < -35:
                print("rotate right a bit")
                #ser.write(b'68\n')
        elif ball_center_dist < hodor_center_dist:
            if yaw > -25:
                print("rotate left a bit")
                #ser.write(b'48\n')
            elif yaw in range(-35,-25):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw < -35:
                print("rotate right a bit")
                #ser.write(b'68\n')
        elif ball_center_dist == hodor_center_dist:
            print("move right")
            #ser.write(b'6\n')
            if yaw > -25:
                print("rotate left a bit")
                #ser.write(b'48\n')
            elif yaw in range(-35,-25):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw < -35:
                print("rotate right a bit")
                #ser.write(b'68\n')
    #range 3 of arena away from wall
    elif hodor_center_dist < -59 and hodor_center_dist < -92:
        ball_center_dist = abs(ball_center_dist)
        hodor_center_dist = abs(hodor_center_dist)
        if ball_center_dist > hodor_center_dist:
            print("move right")
            #ser.write(b'6\n')
            if yaw > -55:
                print("rotate left a bit")
                #ser.write(b'48\n')
            elif yaw in range(-65,-55):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw < -65:
                print("rotate right a bit")
                #ser.write(b'68\n')
        elif ball_center_dist < hodor_center_dist:
            if yaw > -55:
                print("rotate left a bit")
                #ser.write(b'48\n')
            elif yaw in range(-65,-55):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw < -65:
                print("rotate right a bit")
                #ser.write(b'68\n')
        elif ball_center_dist == hodor_center_dist:
            print("move right")
            #ser.write(b'6\n')
            if yaw > -55:
                print("rotate left a bit")
                #ser.write(b'48\n')
            elif yaw in range(-65,-55):
                print("go fwd")
                #ser.write(b'2\n')
            elif yaw < -65:
                print("rotate right a bit")
                #ser.write(b'68\n')
    else:
        print("go fwd")
        #ser.write(b'2\n')
    return
    
def wrong_orr(ball_y):
    ball_center_dist = ball_y - 190
    if ball_center_dist > 0:
        print("diagonal left")
        ser.write(b'1\n')
    elif ball_center_dist < 0:
        print("diagonal right")
        ser.write(b'3\n')
    return
    
def hitter(hodor_x):
    if hodor_x < 138:
        print("robot stop and hitter hit")
        ser.write(b'55\n')
    else:
        print("move fwd")
        ser.write(b'2\n')
    return

def decision(hodor_x,hodor_y,ene_x,ene_y,ball_x,ball_y):
    #yet to code
    enemy_distance_ball = distance(ene_x,ene_y,ball_x,ball_y)
    hodor_distance_ball =  distance(hodor_x,hodoe_y,ball_x,ball_y)
    if enemey_distance_ball > hodor_distance_ball:
        print("offence")
        verdict = 'defence'
    elif enemey_distance_ball < hodor_distance_ball:
        print("defence")
        verdict = 'defence'
    return verdict

def recieve_arduino1():
    line = ser1.readline().decode('utf-8').rstrip()
    yaw = float(line)
    return yaw

def recieve_arduino():
    global cx,cy,height,width,gx,gy,bx,by,yx,yy 
    line = ser.readline().decode('utf-8').rstrip()
    print(line)
    pixydata = line.split(",")
    z = pixydata
    length = len(z)
    while length > 0:
        if length > 3:
            z1 = pixydata.pop(0)
            z2 = pixydata.pop(0)
            z3 = pixydata.pop(0)
            z4 = pixydata.pop(0)
            try:
                cx = float(z1)
                cy = float(z2)
                #global height
                height = float(z3)
                #global width
                width = float(z4)
            except:
                continue   
        elif length > 2:
            data = pixydata.pop(0)
            if data == 'Green:':
                #global gx
                gx1 = pixydata.pop(0)
                gx = float(gx1)
                #global gy
                gy1 = pixydata.pop(0)
                gy = float(gy1)
            elif data =='blue:':
                print("hi hi blue")
                #global bx
                bx1 = pixydata.pop(0)
                bx = float(bx1)
                #global by
                by1 = pixydata.pop(0)
                by = float(by1)
            elif data == 'yellow:':
                print("hi hi yellow")
                #global yx
                yx1 = pixydata.pop(0)
                yx = float(yx1)
                #global yy
                yy1 = pixydata.pop(0)
                yy = float(yy1)
        print (cx,cy,height,width,gx,gy,bx,by,yx,yy)
        return cx,cy,height,width,gx,gy,bx,by,yx,yy 
    #return cx,cy,height,width,gx,gy,bx,by,yx,yy

def our_color_decision(button,gx,gy,bx,by):
    opp_x,opp_y,our_color = goal(button)
    if our_color == 'green':
        hodor_x = gx
        hodor_y = gy
        ene_x = bx
        ene_y = by
    elif our_color == 'blue':
        hodor_x = bx
        hodor_y = by
        ene_x = gx
        ene_y = gy
    return hodor_x,hodor_y,ene_x,ene_y;

def penalty_box(button):
    opp_x,opp_y,our_color = goal(button)
    if our_color == 'green':
        our_goal_x = 18
        our_goal_y = 187
    elif our_color == 'blue':
        our_goal_x = 508
        our_goal_y = 183
        
while True:
    #hodor= gx,gy; ball = yx,yy; enemey = bx,by
    print("im alive")
    button =  0
    cx1,cy1,height1,width1,gx1,gy1,bx1,by1,yx1,yy1 = recieve_arduino()
    hodor_x,hodor_y,ene_x,ene_y = our_color_decision(button,gx1,gy1,bx1,by1)
    #verdict = decision(hodor_x,hodor_y,ene_x,ene_y,yx,yy)
#     print(yx1)
#     print(yy1)
#     print(hodor_x)
#     print(hodor_y)
    verdict = 'offence'
    if verdict == 'offence':
        from_pixy(cx1,cy1,height1,width1)
        okay = orientation(yx1,yy1,hodor_x,hodor_y)
        if (goal == 0):
            hodor_center_dist = hodor_y - 190
            ball_center_dist = ball_y - 190
            print(hodor_center_dist,ball_center_dist)
            score(hodor_center_dist,ball_center_dist,yaw)
        elif(goal == 1):
            hodor_center_dist = -hodor_y + 190
            ball_center_dist = -ball_y + 190
            print(hodor_center_dist,ball_center_dist)
            score(hodor_center_dist,ball_center_dist,yaw)
        hitter(hodor_x)  
    elif verdict == 'defence':
        print("ETA go home")
                












    

