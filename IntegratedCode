// Written By: Hunter Manning, Jevin Valdez (@Jvalley)
// Integrated Code for Automation of the City of Lubbocks' Waste Department

#Importing Libraries
import pymysql
import RPi.GPIO as GPIO
import serial
from io import BytesIO
from picamera import PiCamera
from time import sleep

#Declaring and Initailizing Serial Ports
power_key = 6
serRFID = serial.Serial('/dev/ttyACM0', 115200, timeout = 1) #Change PORT
ser = serial.Serial('/dev/ttyUSB3',115200, timeout = 1)

#set the GPIO pin numbers for the servos
pan_pin = 12
tilt_pin = 13

# set the pwm frequency for the servos
pwm_freq = 50

# set the duty cycle range for the servos
duty_min = 2.5
duty_max = 12.5

#Initialize the GPIO Pins and Camera
GPIO.setmode(GPIO.BCM)
GPIO.setup(pan_pin, GPIO.OUT)
GPIO.setup(tilt_pin, GPIO.OUT)
led = 24
cam = PiCamera()
cam.vflip = True
GPIO.setup(led, GPIO.OUT)

#Initialize the PWM Objects for the servos
pan_pwm = GPIO.PWM(pan_pin, pwm_freq)
tilt_pwm = GPIO.PWM(tilt_pwm, pwm_freq)

#Start the PWM signals with 0 duty cycle
pan_pwm.start(0)
tilt_pwm.start(0)

#Sets the connection to the MySQL Database
try:
    db = pymysql.connect(host='172.20.10.7',
                         user='jevin',
                         password='password',
                         database='GarbageDB')
except Exception as e:
    print("error",e)

print("Connection Successful")

#Sets the cursors and sql commands to run later in the code
sqlRFID = "INSERT INTO RFID (EPC) VALUES (%s)"
sqlGPS = "INSERT INTO GPS (Coordinates) VALUES (%s)"
cursor = db.cursor()
sql = "INSERT INTO Images (Photo_data) VALUES (%s)"

#Function to set and change duty cycles for the pan tilt servos
def set_servo_position(pwm, position):
	duty = (position /180.0) * (duty_max - duty_min) + duty_min
	pwm.ChangeDutyCycle(duty)
	sleep(0.2)

#sets the pan servo center and the tilt servo down 45 degrees
def mid():
	set_servo_position(pan_pwm, 90)
	set_servo_position(tilt_pwm, 25)
	
#sets the pan servo left and the tilt servo up 90 degrees
def left():
	set_servo_position(pan_pwm, 0)
	set_servo_position(tilt_pwm, 0)
	
def take_picture_mid():
	mid()
	cam.start_preview()
	sleep(2)
	cam.capture('image.jpg')
	cam.stop_preview()
	with open("image.jpg","rb") as f:
		image_data = f.read()
	blob_data = BytesIO(image_data)
	cursor.execute(sql, (blob_data.read()))
	db.commit()
	print("picture uploaded successfully")
	
def take_picture_left():
	left()
	cam.start_preview()
	sleep(2)
	cam.capture('image.jpg')
	cam.stop_preview()
	with open("image.jpg","rb") as f:
		image_data = f.read()
	blob_data = BytesIO(image_data)
	cursor.execute(sql, (blob_data.read()))
	db.commit()
	print("picture uploaded successfully")

def captureImages():
	GPIO.output(led,True)
	take_picture_mid()
	GPIO.output(led,False)
	sleep(7)
	GPIO.output(led,True)
	take_picture_left()
	GPIO.output(led,False)
	sleep(7)
	GPIO.output(led,True)
	take_picture_mid()
	GPIO.output(led,False)
	pan_pwm.stop()
	tilt_pwm.stop()
	GPIO.cleanup()
	print("all pictures are uploaded!")

    
    

def get_GPS():
    ser.write(b'AT+CGPS=1\r\n')
    sleep(0.5)
    ser.write(b'AT+CGPSINFO\r\n')
    Coordinates = ser.readline()
    ##print(Coordinates)
    if b'+CGPSINFO: ' in Coordinates:
        global GPSDATA
		# Gets the NMEA GPS data, and cleans it to removes the prefix from the GPS data and stores the cleaned data in the variable (Cleaned)
        GPSDATA = str(Coordinates.decode())
        Cleaned = GPSDATA[13:]

		#Extracts the first two characters from the cleaned GPS data which represents the degrees of the latitude coordinate.
        Lat = Cleaned[:2]
		#extracts the next 9 characters from the cleaned GPS data, which characters represent the minutes and seconds of the latitude coordinate.
        SmallLat = Cleaned[2:9]
		#extracts the 13th character from the cleaned GPS data, indicates whether the latitude coordinate is north or south of the equator.
        NorthOrSouth = Cleaned[10:11]
		#extracts the 15th to 17th characters from the cleaned GPS data which represent the degrees of the longitude coordinate.    
        Long = Cleaned[12:15]
		#extracts the next 9 characters from the cleaned GPS data, represents the minutes and seconds of the longitude coordinate.
        SmallLong = Cleaned[15:24]
		#extracts the 28th character from the cleaned GPS data and stores it, indicates  whether the longitude coordinate is east or west of the prime meridian.
        EastOrWest = Cleaned[25:26]

        FinalLat = float(Lat) + (float(SmallLat)/60)
        FinalLong = float(Long) + (float(SmallLong)/60)
        
        if NorthOrSouth == 'S': FinalLat = -FinalLat
        if EastOrWest == 'W': FinalLong = -FinalLong
        Final_Coordinates = str(FinalLat) +"," + str(FinalLong)
        
        if Final_Coordinates:
            print("GPS is being put in database")
            cursor.execute(sqlGPS, (Final_Coordinates))
            db.commit()        


while True:
	data = serRFID.readline().decode()
	getGPS()
	if data:
		captureImages()
		print("Found a tag")
		cursor.execute(sqlRFID, (data))
		db.commit()
