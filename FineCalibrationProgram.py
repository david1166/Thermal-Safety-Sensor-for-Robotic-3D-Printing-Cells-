# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
import time
import sys
import board
import busio
import cv2
import numpy as np
import keyboard
import ArducamDepthCamera as ac
import adafruit_mlx90640A as MLXA
import adafruit_mlx90640B as MLXB

##################################
# CAMERAS SETUP

i2c = busio.I2C(board.SCL, board.SDA, frequency=800000) # Create I2C instance

cam = ac.ArducamCamera()    #Create TOF camera instance
if cam.init(ac.TOFConnect.CSI,0) != 0 : #Initialise TOF camera
    print("initialization failed")
if cam.start(ac.TOFOutput.DEPTH) != 0 : #Start TOF camera
    print("Failed to start camera")
cam.setControl(ac.TOFControl.RANG,MAX_DISTANCE) #Set the maximum range of the TOF camera
cv2.namedWindow("preview", cv2.WINDOW_NORMAL)   #Create an image window to display TOF captures
    
mlxA = MLXA.MLX90640(i2c)   #Create a thermal camera instance using I2C and address 0x33
mlxB = MLXB.MLX90640(i2c)   #Create a thermal camera instance using I2C and address 0x37

mlxA.refresh_rate = MLXA.RefreshRate.REFRESH_16_HZ  #Set refresh rate of thermal camera A
mlxB.refresh_rate = MLXB.RefreshRate.REFRESH_16_HZ  #Set refresh rate of thermal camera B

# END CAMERAS SETUP
###################################

############################################
# FUNCTION SETUP

# Function to process data captured by TOF camera (Taken directly from ArduCam website)
def ProcessFrame(depth_buf: np.ndarray, amplitude_buf: np.ndarray) -> np.ndarray:
        
    depth_buf = np.nan_to_num(depth_buf)

    amplitude_buf[amplitude_buf<=7] = 0
    amplitude_buf[amplitude_buf>7] = 255

    depth_buf = (1 - (depth_buf/MAX_DISTANCE)) * 255
    depth_buf = np.clip(depth_buf, 0, 255)
    result_frame = depth_buf.astype(np.uint8)  & amplitude_buf.astype(np.uint8)
    return result_frame 

# setup of rectangle for image marking. 
class UserRect():
    def __init__(self) -> None:
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

selectRect = UserRect() #Create an instance of UserRect() for shortest distance locating

# Function to get temp and position from thermal camera
# Incomming parameter used to select camera:
# _camSelect == 1 ==> thermal camera A
# _camSelect != 1 ==> thermal camera B
def GetTempWithPos(_camSelect):
    _reply = [0] * 2    #Create array to return temperature and position in array
    _highTemp = 0.0     #Used to find highest temperature in field of view
    _mlxFrame = [0] * 768#Create array to hold thermal camera captured data
    time.sleep(0.3)     #Create delay between taking of temperature readings (to prevent buffing error)
    if _camSelect == 1:
        mlxA.getFrame(_mlxFrame) #Get data from camera A
            
        for i in range(768):    #Save highest temperature found in FOV and record the height it was observed at
            if (_highTemp < _mlxFrame[i]):   
                _highTemp = _mlxFrame[i] 
                _arrayPos = (i/32)
    else:
        mlxB.getFrame(_mlxFrame) #Get data from camera A
            
        for i in range(768):    #Save highest temperature found in FOV and record the height it was observed at
            if (_highTemp < _mlxFrame[i]):
                _highTemp = _mlxFrame[i]
                _arrayPos = (i/32)
            
    _highTemp = round(_highTemp,1)  #Round high temperature to 1 decimal place 
    _reply[0] = _highTemp           
    _reply[1] = int(_arrayPos)           #Fill _reply with high temperature and position and return
    return _reply

def CameraSettle():
    count = 0
    while (_count < 5):
        _tempA = GetTempWithPos(1)  #Get camera A highest temperature and position
        _tempB = GetTempWithPos(2)  #Get camera B highest temperature and position
        if (((_tempA[0] - _tempB[0]) > ((_tempA[0] + _tempB[0])/200)) or ((_tempA[0] - _tempB[0]) < ((_tempA[0] + _tempB[0])/200)*-1)):
            return 1            # If temperatures more than 1% away from average of both, reset and restart settle
        else:
            _count+=1            # else increase _count by 1
    return 0                    # When _count reaches 5, resturn 0 as settle is complete
    
# Function to change measured temp to real temp
def ConvertTemperature(_temperature,_distance):
    _k = 1.465 * pow(_distance, 2.1728) #Calculate required K coefficient related to distance
    _retVal = (_temperature*_k)/(_distance*_distance) #Calculate real temperature
    return _retVal  #return real temperature

# Function to display camera view on screen (Some taken from ArduCam site directly)
#Incomming parameters
#_tempA & _tempB ==> observed temperatures from thermal cameras A & B
#_arrayPos ==> height position in field of view of hottest point from themral cameras
#_realTemp ==> Calculated real temperature at heat source
def DisplayCamera(_tempA,_tempB,_arrayPos,_realTemp):
    _retVal = 5000.0 #initialise return value for shortest distance
    _rectX = [0] * 42 #Array to hold pixel values in the X-plane
    _rectY = [0] * 10 #Array to hold pixel values in the X-plane
    _finalRect = [0] * 4 #Array to hold the finally selected rectangle
    depthFrame = cam.requestFrame(200)  #Request data from TOF camera
    if depthFrame != None:  #If data is recieved
        depth_buf = depthFrame.getDepthData()   #Convert frame to depth data
        amplitude_buf = depthFrame.getAmplitudeData() #Convert frame to amplitude data
        amplitude_buf[amplitude_buf<0] = 0
        amplitude_buf[amplitude_buf>255] = 255
        cam.releaseFrame(depthFrame)    #Release camera data from memory
        
        if _arrayPos < 12:   #Find height in FOV to search in, provided by thermal cameras
            _positionIndex = 89 - int((_arrayPos-12) * 7.5 * 0.78) # 0.8822)
        else:
            _positionIndex = 90 + int((11-_arrayPos) * 7.5 * 0.78)# 0.8822)
            
        _offset = 40 #_offset to begin scan in X-plane
   
        for i in range(21): #Split array into groups, 4 pixels wide on X-plane
            _rectX[i] = 118 - _offset + (i*4)
            _rectX[i+21] = _rectX[i]+3
       
        for i in range(5):  #Split array into groups, 4 pixels wide on Y-plane
            _rectY[i] = _positionIndex - 8 + (4*i)
            _rectY[i+5] = _rectY[i]+3
         
        for i in range(5):  
            for j in range(21): #Test each of the 105 squares of pixels in turn
                selectRect.start_x = _rectX[j]
                selectRect.end_x = _rectX[j+21]
                selectRect.start_y = _rectY[i]
                selectRect.end_y = _rectY[i+5]
                
                if (np.mean(depth_buf[selectRect.start_y:selectRect.end_y,selectRect.start_x:selectRect.end_x])) < _retVal:
                    _retVal = (np.mean(depth_buf[selectRect.start_y:selectRect.end_y,selectRect.start_x:selectRect.end_x]))  
                    _finalRect[0] = selectRect.start_y
                    _finalRect[1] = selectRect.end_y     # If newly tested data is nearer than any previously tested data
                    _finalRect[2] = selectRect.start_x   # Safe the location of this square
                    _finalRect[3] = selectRect.end_x
                    
        selectRect.start_x = _finalRect[2]
        selectRect.end_x = _finalRect[3]
        selectRect.start_y = _finalRect[0]   #Reselect the square found to be the nearest
        selectRect.end_y = _finalRect[1]
        
        _font = cv2.FONT_HERSHEY_SIMPLEX # Setting _font for text on images
        _fontScale = 0.4                 # Setting size of texrt 
        _thickness = 1                   # Setting _thickness of text (Bold)
        
        _resultImage = ProcessFrame(depth_buf,amplitude_buf)   #Process TOF camera data to produce and image
        _resultImage = cv2.applyColorMap(_resultImage, cv2.COLORMAP_PLASMA) #Apply colour map to the created image
        cv2.rectangle(_resultImage,(selectRect.start_x,selectRect.start_y),(selectRect.end_x,selectRect.end_y),(128,128,128), 1) #Place the square with the lowest distance on the image
                             
        
        if (_realTemp == 0):  #If real temperature is 0, cameras are not yet settled
            _tempAText = "T1: " + str(round(_tempA,1))   # Create temperature A text
            _tempBText = "T2: " + str(round(_tempB,1))   # Create temperature B text
            _resultImage = cv2.putText(_resultImage, _tempAText, (0,160), _font, _fontScale, (0,0,0), _thickness, cv2.LINE_AA, False)  #Add temperature A text to image
            _resultImage = cv2.putText(_resultImage, _tempBText, (180,160), _font, _fontScale, (0,0,0), _thickness, cv2.LINE_AA, False)#Add temperature B text to image
        else:
            _cellSafe = _realTemp < 40   #Cell is marked safe if real temperature is less than 40 degrees 
            _realTempText = "Real temp: " + str(round(_realTemp,1)) + " C"  #Creates text displaying the temperature at source to the user
            _cellSafeText = "Cell safe: " + str(_cellSafe)                   #Creates text displaying cell safe status to the user
            _resultImage = cv2.putText(_resultImage, _realTempText, (0,160), _font, _fontScale, (0,0,0), _thickness, cv2.LINE_AA, False) #Add real temperature text to image
            _resultImage = cv2.putText(_resultImage, _cellSafeText, (0,170), _font, _fontScale, (0,0,0), _thickness, cv2.LINE_AA, False) #Adds cell status text to image
        _distanceText = "Distance: " + str(round(100*(_retVal),2)) + "cm" #Creates text displaying the distance of the heat source to the user
        _resultImage = cv2.putText(_resultImage, _distanceText, (0,20), _font, _fontScale, (0,0,0), _thickness, cv2.LINE_AA, False)   #Adds distance text to image
                                                                                                                                
        cv2.imshow("preview",_resultImage) # load image to show on waitKey()
        _key = cv2.waitKey(1)               # Display the image on-screen
        return _retVal

# END FUNCTION SETUP
###################################

##################################
# MAIN PROGRAM
count = 0   #clear count variable of how many results have been written out
FILENAME = "TestAt41.5Degrees.csv" #Name of csv file where results will be written to

#Loop to find centre of thermal cameras                                  
while (CameraSettle() != 0):   #Call function to settle cameras until they are centred
    time.sleep(0.05) #Time delay between checks

#Initial loop setup and first run                                         
loopExit = False    #reset exit flag
replyA = GetTempWithPos(1)  #Read from camera A
replyB = GetTempWithPos(2)  #Read from camera B
                       
tempArrayPos = (replyA[1] + replyB[1])/2 #Get FOV height of hottest point
averageTemp = (replyA[0] + replyB[0])/2 #Get average of hottest temp from each camera
distance = DisplayCamera(replyA[0],replyB[0],(replyA[1]+replyB[1])/2,0) #Display camera image and recieve distance to hottest point
realTemp = ConvertTemperature(averageTemp,distance) #Calculate real temperature of hottest point


#Main loop                  
while not loopExit:
    replyA = getTempWithPos(1) #Read from camera A
    distance = DisplayCamera(0,0,tempArrayPos,realTemp) #Display camera image and recieve distance to hottest point
    replyB = getTempWithPos(2) #Read from camera B
    tempArrayPos = (replyA[1] + replyB[1])/2 #Get FOV height of hottest point
    averageTemp = (replyA[0] + replyB[0])/2 #Get average of hottest temp from each camera
    realTemp = ConvertTemperature(averageTemp,distance) #Calculate real temperature of hottest point
    distance = DisplayCamera(0,0,tempArrayPos,realTemp) #Call function to update image between reads of the thermal cameras to increase framerate
    if (realTemp < 40): #Exit if temperature is below 40 degrees as this accuracy issues will occur
        loopExit = True
    else
        if count < 100: #Continue until 100 results have been recorded
            f = open(FILENAME, "a") #Open csv output file
            f.write("\n" + str(distance) + "," +str(realTemp)) #Write to output file
            f.close() #close connection to output file
            count += 1 #increment count
        else:
            loopExit = True #When 100 results are recorded, trigger flag to exit loop

if count < 99: #Message to user if testing not completed
    print("Temperature too low for accurate measurements. Program terminated")
else     #Message to user if testing has been completed
    print("Testing successfully completeted")
time.sleep(5)
