### Imports ###################################################################
 
from picamera.array import PiRGBArray
from picamera import PiCamera
from functools import partial
 
import multiprocessing as mp
import cv2
import os
import time
import serial 
 
### Setup #####################################################################
ser = serial.Serial("/dev/ttyS0", 115200)
os.putenv( 'SDL_FBDEV', '/dev/fb0' )
 
resX = 256
resY = 192
 
cx = resX / 2
cy = resY / 2
 
#os.system( "echo 0=150 > /dev/servoblaster" )
#os.system( "echo 1=150 > /dev/servoblaster" )
 
xdeg = 150
ydeg = 150
 

# Setup the camera
camera = PiCamera()
camera.resolution = ( resX, resY )
camera.framerate = 60
 
# Use this as our output
rawCapture = PiRGBArray( camera, size=( resX, resY ) )
 
# The face cascade file to be used
face_cascade = cv2.CascadeClassifier('/home/pi/QT/Face_Tracker/haarcascade_frontalface_alt2.xml')
 
t_start = time.time()
fps = 0
check_num=0 
Face_bigx=0
Face_bigy=0
Face_bigs=0
### Helper Functions ##########################################################
def limitm(input,limit):
    
    out=input
    if(input>limit):
        out=limit
    if(input<-limit):
        out=-limit
    
    return out
        
def uart_send():
    
    buf=range(0,30)
    sum=chr(0)
    print('serial sending Face num: ',check_num)
    print('X: ' ,Face_bigx-cx ,' Y: ' ,Face_bigy-cy, ' Size ' ,Face_bigs)
    buf[0]=chr(0xAA)
    buf[1]=chr(0xAF)
    buf[2]=chr(0x02)
    buf[3]=chr(0x04)
    buf[4]=chr(check_num)
    buf[5]=chr(Face_bigx)
    buf[6]=chr(Face_bigy)
    buf[7]=chr(Face_bigs)
  
    for i in range(0,8):
        #print(i)
        sum=sum+buf[i]
        
    buf[8]=sum

    for i in range(0,9):
        ser.write(buf[i]) 

    
def get_faces( img ):
 
    gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )
    faces = face_cascade.detectMultiScale( gray )
 
    return faces, img
 
def draw_frame( img, faces ):
 
    global xdeg
    global ydeg
    global fps
    global time_t
    
    # Draw a rectangle around every face
    for ( x, y, w, h ) in faces:
        
        cv2.rectangle( img, ( x, y ),( x + w, y + h ), ( 200, 255, 0 ), 2 )
        cv2.putText(img, "Face No." + str( len( faces ) ), ( x, y ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ( 0, 0, 255 ), 2 )
 
        tx = x + w/2
        ty = y + h/2
        
        cv2.circle( img, ( tx,ty), 5 , ( 200, 255, 0 ), 2 )
         
        if   ( cx - tx > 15 and xdeg <= 190 ): xdeg += 1# os.system( "echo 0=" + str( xdeg ) + " > /dev/servoblaster" )
        elif ( cx - tx < -15 and xdeg >= 110 ):
            xdeg -= 1
            #os.system( "echo 0=" + str( xdeg ) + " > /dev/servoblaster" )
 
        if   ( cy - ty > 15 and ydeg >= 110 ):
            ydeg -= 1
            #os.system( "echo 1=" + str( ydeg ) + " > /dev/servoblaster" )
        elif ( cy - ty < -15 and ydeg <= 190 ): ydeg += 1 #os.system( "echo 1=" + str( ydeg ) + " > /dev/servoblaster" )
 
    # Calculate and show the FPS
    fps = fps + 1
    sfps = fps / (time.time() - t_start)
    cv2.putText(img, "FPS : " + str( int( sfps ) ), ( 10, 10 ), cv2.FONT_HERSHEY_SIMPLEX, 0.5, ( 0, 0, 255 ), 2 ) 
    print('FPS: ',sfps) 
    cv2.imshow( "Frame", img )
    cv2.waitKey( 1 )
 
 
### Main ######################################################################
 
if __name__ == '__main__':
 
    pool = mp.Pool( processes=4 )
    fcount = 0
 
    camera.capture( rawCapture, format="bgr" )  
 
    r1 = pool.apply_async( get_faces, [ rawCapture.array ] )    
    r2 = pool.apply_async( get_faces, [ rawCapture.array ] )    
    r3 = pool.apply_async( get_faces, [ rawCapture.array ] )    
    r4 = pool.apply_async( get_faces, [ rawCapture.array ] )    
 
    f1, i1 = r1.get()
    f2, i2 = r2.get()
    f3, i3 = r3.get()
    f4, i4 = r4.get()
 
    rawCapture.truncate( 0 )
    

    for frame in camera.capture_continuous( rawCapture, format="bgr", use_video_port=True ):
        image = frame.array
 
        if   fcount == 1:
            r1 = pool.apply_async( get_faces, [ image ] )
            f2, i2 = r2.get()
            draw_frame( i2, f2 )
            check_num=len( f2 )
            for ( x, y, w, h ) in f2:
                tx = x + w/2
                ty = y + h/2     
                if w*h>Face_bigs:
                  Face_bigs=w*h/100
                  Face_bigx=tx
                  Face_bigy=ty
 
        elif fcount == 2:
            r2 = pool.apply_async( get_faces, [ image ] )
            f3, i3 = r3.get()
            draw_frame( i3, f3 )
            check_num=len( f3 )
            for ( x, y, w, h ) in f3:
              tx = x + w/2
              ty = y + h/2     
              if w*h>Face_bigs:
                Face_bigs=w*h/100
                Face_bigx=tx
                Face_bigy=ty
 
        elif fcount == 3:
            r3 = pool.apply_async( get_faces, [ image ] )
            f4, i4 = r4.get()
            draw_frame( i4, f4 )
            check_num=len( f4 )
            for ( x, y, w, h ) in f4:
              tx = x + w/2
              ty = y + h/2     
              if w*h>Face_bigs:
                Face_bigs=w*h/100
                Face_bigx=tx
                Face_bigy=ty
                
        elif fcount == 4:
            r4 = pool.apply_async( get_faces, [ image ] )
            f1, i1 = r1.get()
            draw_frame( i1, f1 )
            check_num=len( f1 )
            for ( x, y, w, h ) in f1:
              tx = x + w/2
              ty = y + h/2     
              if w*h>Face_bigs:
                Face_bigs=w*h/100
                Face_bigx=tx
                Face_bigy=ty
                
            fcount = 0
 
        fcount += 1
 
        rawCapture.truncate( 0 )

        if check_num>0:
            Face_bigx=Face_bigx
            Face_bigy=Face_bigy
            Face_bigs=limitm(Face_bigs,250)
            
 	uart_send()
        check_num=0
        Face_bigx=0
        Face_bigy=0
        Face_bigs=0
    	key = cv2.waitKey(30)
    	if key & 0x00FF  == ord('q'):
        	cap.release()
     #cv2.destroyAllWindows()
 

    
