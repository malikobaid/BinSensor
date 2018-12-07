from time import sleep
from picamera import PiCamera
from datetime import datetime, timedelta
import RPi.GPIO as GPIO  
import os

GPIO.setmode(GPIO.BCM)  
GPIO.setwarnings(False)
GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 

def captureImage(channel):
#    fname'=datetime.utcnow().strftime('%Y%m%d%H%M%S%f')+''.jpg'
    fname='latest.jpg'
    cmd = 'raspistill -o '+fname
    os.system(cmd)
    GPIO.setup(26, GPIO.LOW)
    os.system('./myscp.sh')
    
    
def wait():
    # Calculate the delay to the start of the next hour
    next_hour = (datetime.now() + timedelta(minutes=1))
    delay = (next_hour - datetime.now()).seconds
    sleep(delay)
    
GPIO.add_event_detect(26, GPIO.RISING, callback=captureImage, bouncetime=500)  

while True:
	sleep(0.5)