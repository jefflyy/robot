# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

debug = True

import sensor, image, time, json, math
from pyb import UART, Servo
from pid import PID

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()                # Create a clock object to track the FPS.

pan_pid = PID(p=0.2,i=0.01,imax=90,d=0)

def hasball(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob = blob
            max_size = blob[2]*blob[3]
    if(max_size>0):
        return True
    else:
        return False
def rot(ang):
    global uart
    uart.write(bytearray([255,255,4,(round(ang*32768/180))&255,(round(ang*32768/180))>>8]))
    return 0

def mov(vel):
    global uart
    uart.write(bytearray([255,255,1,vel&255,(vel)>>8]))
    return 0

def movandstop(dist):
    global uart
    uart.write(bytearray([255,255,2,round(dist)&255,(round(dist))>>8]))
    print(dist)
    return 0

def pick():
    global uart
    uart.write(bytearray([255,255,6,255,255]))

def drop(dist):
    global uart
    uart.write(bytearray([255,255,7,round(dist)&255,(round(dist))>>8]))

def angcalib():
    global uart
    uart.write(bytearray([255,255,3,255,255]))

def rotab(ang):
    global uart
    print(ang)
    uart.write(bytearray([255,255,5,(round(ang*32768/180))&255,(round(ang*32768/180))>>8]))
    return 0

cameraconsthorizontal = 20/13 # to be measured
cameraconstvertical = 2 # to be measured
codertomm = 50/22 # to be measured

def pixtoradvert(pix):
    global currang, ythres
    pan_error = math.atan((pix-120)/(120*cameraconstvertical))
    if(abs(pan_error)>ythres):
        currang += pan_pid.get_pid(pan_error,1)*180/math.pi
        if(abs(currang+20)<90):
            s1.angle(currang)

def pixtoradhor(pix):
    global xchangedthisturn, xthres
    pan_error = math.atan((pix-160)/(160*cameraconsthorizontal))
    if(abs(pan_error)>xthres):
        rot(-pan_error*180/math.pi)
        xchangedthisturn = True


def rightdirx(ball):
    global xthres
    pan_errorx = math.atan((ball.cx()-160)/(160*cameraconsthorizontal))
    if(abs(pan_errorx)<=xthres):
        print("X right")
        return True
    return False

def rightdiry(ball):
    global ythres
    pan_errory = math.atan((ball.cy()-120)/(120*cameraconstvertical))
    if(abs(pan_errory)<=ythres):
        print("Y right")
        return True
    return False

def imgfilterball(currimg):
    return currimg.binary([(32,100,-25,52,30,77)])
def imgfilterhome(currimg):
    return currimg.binary([(0,76,-60,-25,10,52)])

def imgdenoise(currimg):
    currimg.erode(1)
    currimg.dilate(2)
    currimg.erode(1)
    return currimg

def isball(currb):
    if currb is None:
        return False
    if((currb.h()+currb.w())>2):
        return True

def balltoosmall(currb):
    if currb is None:
        return False
    if((currb.h()+currb.w())>10):
        return False
    return True

def hometoosmall(home):
    if home is None:
        return False
    if((home.h()+home.w())>25):
        return False
    return True

def ishome(home):
    if home is None:
        return False
    if (home.h()+home.w())>0:
        return True
    else:
        return False

def find_maxball(blobs):
    global bcghome
    max_size=0
    for blob in blobs:
        if (blob[2]*blob[3] > max_size):
            if(bcghome is None):
                max_blob = blob
                max_size = blob[2]*blob[3]
            elif((bcghome.cx()+0.5*bcghome.w()+30<blob.cx()) or (bcghome.cx()-0.5*bcghome.w()-30>blob.cx()) or (bcghome.cy()-0.5*bcghome.h()-30>blob.cy()) or (bcghome.cy()+0.5*bcghome.h()+30<blob.cy())):
                print(bcghome.cx(),bcghome.w(),blob.cx())
                max_blob = blob
                max_size = blob[2]*blob[3]
    if(max_size>0):
        return max_blob
    else:
        return None

def find_maxhome(blobs):
    max_size=0
    for blob in blobs:
        if (blob[2]*blob[3] > max_size):
            max_blob = blob
            max_size = blob[2]*blob[3]
    if(max_size>0):
        return max_blob
    else:
        return None

def gyro():
    return True
def settime():
    global timesup
    timesup = False

def init():
    global flag, currang, stopthres,  pickthres, placeballconst, sansball, timesup, pawconst, homeradius, dis1, dis2, xthres, ythres, ultimatum, xchangedthisturn, bcghome
    flag = 0
    currang = 0 # -20 means horizontal
    stopthres = -50
    pickthres = -82
    placeballconst = 12
    ultimatum = -120
    sansball = True
    timesup = True
    pawconst = 0 # mm.Was 25. Not an exact number and shouldn't be considered relevant to the real paw.
    homeradius = 240 # mm. Not an exact number and shouldn't be considered relevant to the real home.
    dis1 = 10000
    dis2 = 10000
    xthres = 0.1
    ythres = 0.18
    xchangedthisturn = False
    bcghome = None


uart = UART(3, 115200)
s1 = Servo(1)

init()
ballpick = 0
angcalib()

while(debug):
    clock.tick()
    img = sensor.snapshot()
    if(sansball):
        bcghome = None
        balls = img.find_blobs([(32,100,-5,52,32,77)])
        homes = img.find_blobs([(0,76,-60,-25,10,52)])
        bcghome = find_maxhome(homes)
        print("home is",ishome(bcghome))
        if(hasball(balls)):
            currb = find_maxball(balls)
            print("ball is",isball(currb))
            if(isball(currb)):
                pixtoradvert(currb.cy())
                pixtoradhor(currb.cx())
                if(balltoosmall(currb)):
                    mov(5600)
                    time.sleep(1)
                elif(flag>-1):
                    flag = 0
                    print(currang)
                    if(rightdirx(currb) and (currang<stopthres)):
                        movandstop(0)
                        flag = -1
                        ythres = 0.1
                    elif(rightdirx(currb) and rightdiry(currb)):
                        mov(5200)
                        xchangedthisturn = True
                    else:
                        xchangedthisturn = False
                else:
                    if(currang>pickthres):
                        theta = 90+currang+20
                        dis1 = (18.93 / math.sin(theta*math.pi/180) + 177.19) * math.tan(theta*math.pi/180)
                        dis1 = dis1 - pawconst
                        print(theta, dis1)
                        if(dis1>3):
                            movandstop(dis1*codertomm)
                        else:
                            pick()
                            time.sleep(0.75)
                            ythres = 0.18
                            sansball = False
                            flag = 0
                            s1.angle(-32)
                            currang = -32
                            mov(-5600)
                            time.sleep(1)
                    else:
                        pick()
                        ballpick += 1
                        time.sleep(1)
                        ythres = 0.18
                        sansball = False
                        flag = 0
                        s1.angle(-32)
                        currang = -32
                        mov(-5600)
                        time.sleep(1)
            else:
                if(flag==-1):
                    pick()
                    ballpick += 1
                    time.sleep(1)
                    ythres = 0.18
                    sansball = False
                    flag = 0
                    s1.angle(-32)
                    currang = -32
                    mov(-5600)
                    time.sleep(1)
                elif(xchangedthisturn == False):
                    if(flag==1):
                        rot(31)
                        flag = 0
                        currang = -32
                        s1.angle(-32)
                        time.sleep(0.4)
                    else:
                        flag = 1
                        currang = -58
                        s1.angle(-58)
                        time.sleep(0.2)
        else:
            movandstop(0)
            if(flag==-1):
                pick()
                ballpick += 1
                sansball = False
                time.sleep(1)
                ythres = 0.18
                flag = 0
                s1.angle(-32)
                currang = -32
                mov(-5600)
                time.sleep(1)
            else:
                if(xchangedthisturn == True):
                    xchangedthisturn = False
                elif(flag==1):
                    rot(31)
                    flag = 0
                    currang = -32
                    s1.angle(-32)
                    time.sleep(0.4)
                else:
                    flag = 1
                    currang = -58
                    s1.angle(-58)
                    time.sleep(0.2)
    else:
        #print(xchangedthisturn)
        ythres = 0.08
        xthres = 0.18
        homes = img.find_blobs([(0,76,-60,-25,10,52)])
        if(hasball(homes)):
            home = find_maxhome(homes)
            if(ishome(home)):
                pixtoradvert(home.cy())
                pixtoradhor(home.cx())
                if(hometoosmall(home)):
                    mov(5600)
                    time.sleep(0.75)
                elif(flag>-1):
                    flag = 0
                    if(rightdirx(home) and (currang<stopthres+placeballconst)):
                        movandstop(0)
                        flag = -1
                    elif(rightdirx(home) and rightdiry(home)):
                        mov(5200)
                        xchangedthisturn = True
                    else:
                        xchangedthisturn = False
                else:
                    if((currang>pickthres+placeballconst)):
                        theta = 90+currang+20
                        dis2 = (18.93 / math.sin(theta*math.pi/180) + 177.19) * math.tan(theta*math.pi/180)
                        dis2 -= homeradius
                        print(theta, dis2)
                        movandstop(max(dis2, 0)*codertomm)
                        if(currang>ultimatum+placeballconst):
                            time.sleep(1)
                            drop((230 + homeradius) * codertomm)
                            time.sleep(4.5)
                            rotab(-45)
                            time.sleep(1)
                            mov(6400)
                            time.sleep(1)
                            init()
                            flag = 0
                            # angcalib()
                    else:
                        mov(4800)
                        time.sleep(1)
                        drop((230 + homeradius) * codertomm)
                        time.sleep(4.5)
                        rotab(-45)
                        time.sleep(1)
                        mov(6400)
                        time.sleep(1)
                        init()
                        flag = 0
                        # angcalib()
            else:
                if(xchangedthisturn == False):
                    if(flag==1):
                        rot(31)
                        flag = 0
                        currang = -32
                        s1.angle(-32)
                        time.sleep(0.4)
                    else:
                        flag = 1
                        currang = -58
                        s1.angle(-58)
                        time.sleep(0.2)
        else:
            movandstop(0)
            if(flag==1):
                rot(31)
                flag = 0
                currang = -32
                s1.angle(-32)
                time.sleep(0.4)
            else:
                flag = 1
                currang = -58
                s1.angle(-58)
                time.sleep(0.2)

while(True):
    s1.angle(-20)
    clock.tick()
    img = sensor.snapshot()

