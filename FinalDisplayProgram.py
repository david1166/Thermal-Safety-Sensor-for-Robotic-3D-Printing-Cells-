import socket
import time
import sys
import cv2
import numpy as np
import RPi.GPIO as GPIO

HOST = '192.168.16.43' # Server IP or Hostname
PORT = 3501 # por used for TCP/IP communication
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #AF_INET used for transmission over IPv4. SOCK_STREAM for TCP/IP (instead of UDP)
yellowLED = 19 #Number for yellow LED output from GPIO
greenLED = 18 #Number for green LED output from GPIO
blueLED = 6 #Number for blue LED output from GPIO
chanList = [blueLED,greenLED,yellowLED] #create a list containing all GPIO in use (all outputs)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) #Use designated I/O numbers rather than board pin numbers
GPIO.setup(chanList, GPIO.OUT, initial=GPIO.LOW) #Set channel list as outputs and initialise low
pwm = GPIO.PWM(greenLED,0.5) #Make green LED on pin 18 a pwm. initalise at 0.5Hz for step 0
pwm.start(50) #begin PWM at 50% duty. 
prestep1img = "prestep1.jpg" 
step1img = "step1.jpg"
prestep2img = "prestep2.jpg" #Create variables for image file names
step2unsafeimg = "step2unsafe.jpg"
step2safeimg = "step2safe.jpg"
tempAMsg = " "
tempBMsg = " " #Initialise 2 temperature variables and a distance variable to empty strings
distance = " "

##################################
# FUNCTION SETUP

#Function to flash the yellow LED at 1Hz in the event of an error
def FlashYellow():
    GPIO.output(yellowLED,GPIO.HIGH) #Turn LED on
    time.sleep(0.5)
    GPIO.output(yellowLED,GPIO.LOW) #Turn LED off
    time.sleep(0.5)

#Function to send data over TCP communication
def SendData(data):
    try:
        s.send(data.encode('utf-8'))
    except:
        return -1
        
# Function to display image on screen
# In parameters for the following
# _tempA,_tempB ==> raw temperatures from thermal camera A & B
# _img ==> number for selecting which image will show in screen
# _realTemp ==> actual temperature at source afgter calculations take place
# _distance ==> distance obtained from TOF camera
def DisplayImage(_tempA,_tempB,_img,_realTemp,_distance):                       
        _font = cv2.FONT_HERSHEY_SIMPLEX # Setting _font for text on images
        _fontScale = 1                   # Setting size of texrt 
        _thickness = 2                   # Setting _thickness of text (Bold)
        
        if _img == 1:   #Selects first image, explaining step 1 to user
            _resultImage = cv2.imread(prestep1img)  #read image in to variable array
            cv2.imshow("preview",_resultImage) # load image to show on waitKey()
            key = cv2.waitKey(5000) # Pause program for 5 seconds and display image
        elif _img == 2: #Selects second image, step 1 activity
            _resultImage = cv2.imread(step1img) 
            _tempAMsgText = "Temperature 1: " + _tempA  # Create temperature A text 
            _tempBMsgText = "Temperature 2: " + _tempB  # Create temperature B text 
            _resultImage = cv2.putText(_resultImage, _tempAMsgText, (10,500), _font, _fontScale, (0,0,0), _thickness, cv2.LINE_AA, False)  #Add temperature A text to image
            _resultImage = cv2.putText(_resultImage, _tempBMsgText, (680,500), _font, _fontScale, (0,0,0), _thickness, cv2.LINE_AA, False) #Add temperature B text to image
        elif _img == 3: #Selects third image, step 2 explanation
            _resultImage = cv2.imread(prestep2img)
            cv2.imshow("preview",_resultImage)
            _key = cv2.waitKey(5000)
        else:
            if _img == 4: #Selects fourth image, step 2 with cell unsafe
                _resultImage = cv2.imread(step2unsafeimg)
            elif _img == 5:#Selects fifth image, step 2 with cell in safe state
                _resultImage = cv2.imread(step2safeimg)
            _realTempText = "Real temp: " + _realTemp + " C" #Creates text displaying the temperature at source to the user
            _resultImage = cv2.putText(_resultImage, _realTempText, (10,500), _font, _fontScale, (0,0,0), _thickness, cv2.LINE_AA, False) #Adds real temperature text to image
            _distanceText = "Distance: " + _distance + "cm" #Creates text displaying the distance to heat source to the user
            _resultImage = cv2.putText(_resultImage, _distanceText, (720,500), _font, _fontScale, (0,0,0), _thickness, cv2.LINE_AA, False)#Adds distance text to image
       
        cv2.imshow("preview",_resultImage)
        key = cv2.waitKey(1)                    

# END FUNCTION SETUP
###################################
   

##################################
# MAIN PROGRAM START
   
try:    # Attempt to bind to port
    s.bind((HOST, PORT))
except socket.error:    # If failure occurs, a new host and/or port is required
    print('Bind failed...')
    print('Please change host and/or port number and try again..')
    while True: FlashYellow()   #Endlessly flash yellow LED signalling connection failure
    
print('Socket created')

while True:

    pwm.start(50) 
    pwm.ChangeFrequency(0.5)    #Reset green light flashing to 50% duty and 0.5 Hz

    s.listen(5) #set how many queued connections are allowed
    print('Socket awaiting messages')
    (conn, addr) = s.accept() #Accept client connection once it is available
    print('Connected')
    
    # Step 0
    step = " " #Set/reset variables for step 1
    tempAMsg = " "
    
    while step != "step 1": #wait until signal from sensor 1 to move to step 1
        step = conn.recv(1024).decode()
        print(step)
    DisplayImage(0,0,1,0,0) #display step 1 explanation image
        
    conn.send("Proceed".encode('utf-8')) #reply with 'Proceed' as a handshake
    print('1')
    pwm.ChangeFrequency(1) #Increase green LED frez to 1 Hz
    
    skip = False # reset skip variable
    # Step 1
    while (tempAMsg != "step 2") and not skip: #Loop until signal to move to step 2 is recieved
        tempAMsg = conn.recv(29).decode()  # recieve temperature from thermal camera A from client
        if (tempAMsg != "step 2"): #Only run if signal for step 2 not observed
            try:
                tempNum = float(tempAMsg)  #Test data can be coverted to a float (corrupt data will error out)
                tempBMsg = conn.recv(29).decode()  # recieve temperature from thermal camera B from client
            except:
                time.sleep(0.001)
            DisplayImage(tempAMsg,tempBMsg,2,0,0) # Display step 1 image
        
        print("T1: " + tempAMsg + "   T2: " + tempBMsg)
                
        
    if not skip: #If connection still intact
        DisplayImage(0,0,3,0,0) #Display step 2 explanation image
        conn.send("Proceed".encode('utf-8'))    #Reply to client to proceed to step 2
        pwm.ChangeFrequency(2) #Increase green LED frequency to 2 Hz
        
        # Step 2
        exitLoop = False    #Clear exit loop variable
        while not exitLoop: #Loop until flag is triggered
            try:
                realTemp = conn.recv(29).decode()  #Recieve real temperature data
            except:
                exitLoop = True #If connection failure exit loop and restart program
            try:
                if float(realTemp) > 0:    #Test data is not corrupt
                    distance = conn.recv(29).decode() #Recieve distance data
       
                if float(realTemp) < 40:   #If real temp is less than 40 degrees
                    pwm.ChangeDutyCycle(100) #Greem LED constantly on to show routine is complete
                    GPIO.output(blueLED,GPIO.HIGH) #External warning light that cell is safe to enter
                    DisplayImage(0,0,5,realTemp,distance) #Display step 2 with cell safe image
                else:
                    DisplayImage(0,0,4,realTemp,distance) #Display step 2 with cell unsafe image
            except:
                time.sleep(0.001)   # If data is corrupt do not recieve next data to correct timing
            print("Temperature: " + str(realTemp) + "   Distance: " + str(distance))
