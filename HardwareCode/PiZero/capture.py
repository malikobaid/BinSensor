#!/usr/bin/env python 
from time import sleep
from picamera import PiCamera
from datetime import datetime, timedelta
import RPi.GPIO as GPIO  
import os

GPIO.setmode(GPIO.BCM)  
GPIO.setwarnings(False)
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 

def captureImage(channel):
    fname='/home/pi/webcam/latest.jpg'
    cmd = 'raspistill -o '+fname
    print("Taking Image")
    wait(5) # wait 5 secs till the lid is closed
    os.system(cmd)
    wait(6)  # wait 6 secs till the image is captured and stored
#    GPIO.setup(26, GPIO.LOW)
#    os.system('cd /home/pi/webcam')
    print("Uploading Image")
    os.system('sshpass -p \'<PASS>\' scp /home/pi/webcam/latest.jpg malikobaid@178.159.10.106:/home/malikobaid/webcam/')
    
    
def wait(secs):
    # Calculate the delay to the start of the next hour
    next_hour = (datetime.now() + timedelta(seconds=secs))
    delay = (next_hour - datetime.now()).seconds
    sleep(delay)
    
GPIO.add_event_detect(26, GPIO.RISING, callback=captureImage, bouncetime=500)  

while True:
	sleep(0.5)