import time
from gpiozero import Button
import RPi.GPIO as GPIO
import pickle
import datetime
import board
import busio
import adafruit_vl53l0x
import adafruit_vl6180x
import adafruit_lis3dh
import digitalio 
from picamera import PiCamera


#Sleeping for 20 seconds incase if something is wrong.
#time.sleep(20)

#Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

#Setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Initialize Variables
sensor1_shutdown = 17
sensor2_shutdown = 4
green_led = 6
red_led = 13
button = Button(19)
camera = PiCamera()

GPIO.setup(green_led, GPIO.OUT)
GPIO.setup(red_led, GPIO.OUT)
GPIO.setup(sensor1_shutdown, GPIO.OUT)
GPIO.setup(sensor2_shutdown, GPIO.OUT)

#Timing rates
sample_rate = 0.10 
short_wait = 0.10 
medium_wait = 0.20

#Both range sensors default to same i2c address
#Set all shutdown pins low to turn off each Sensor
#After sensors are OFF, we can power one on and change address
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.output(sensor2_shutdown, GPIO.LOW)
time.sleep(short_wait)


#Turn range sensor 1 on by setting shutdown pin to HIGH
GPIO.output(sensor1_shutdown, GPIO.HIGH)
time.sleep(short_wait)

#Initialize Sensor 1 and change address
vl53 = adafruit_vl53l0x.VL53L0X(i2c)
vl53.set_address(0x30)

#Turn range sensor 2 on, initialize, and change address
GPIO.output(sensor2_shutdown, GPIO.HIGH)
time.sleep(short_wait)
#vl6180 = adafruit_vl6180x.VL6180X(i2c,address = 0x29)
vl6180 = adafruit_vl53l0x.VL53L0X(i2c)

#Initialize accelerometers
#accel_1 address is changed from default via a 3.3v line to the sensor
accel_1 = adafruit_lis3dh.LIS3DH_I2C(i2c, address = 0x19)
accel_2 = adafruit_lis3dh.LIS3DH_I2C(i2c)

#Set accelerometer scale
accel_1_range = adafruit_lis3dh.RANGE_8_G
accel_2_range = adafruit_lis3dh.RANGE_8_G


#Lists to store data from sensors
vl53_data = []
vl6180_data = []
accel_1_data = []
accel_2_data = []
iteration = []

count = 0



def ready_loop():
    ##Base loop.  Flashes green while waiting for button press.
    ##On press, flashes red and calls ranging().


    print('Press button to begin ranging')
    time.sleep(0.40)
    counter = 0
    while True:
        if button.is_pressed:
            time.sleep(medium_wait)
            GPIO.output(red_led, GPIO.HIGH)
            time.sleep(0.50)
            GPIO.output(red_led, GPIO.LOW)
            ranging()
            break

        #Flashes green led every 2 seconds.
        elif counter % (2/medium_wait) == 0:
            GPIO.output(green_led, GPIO.HIGH)
            time.sleep(medium_wait)
            GPIO.output(green_led, GPIO.LOW)
            counter = counter + 1
            print('waiting...')
        else:
            #print('waiting...')
            counter = counter + 1
            time.sleep(medium_wait)



def ranging():
    ##Gathers data from sensors and saves to list.
    ##Flashes red led when recording.
    ##On button press, shines red led and calls save_data().

    global count

    print('Getting ranges...')

    while True:
        distance_1 = vl53.range
        vl53_data.append(distance_1)
        print('Range Sensor 1: {0}mm'.format(distance_1))

        distance_2 = vl6180.range
        vl6180_data.append(distance_2)
        print('Range Sensor 2: {0}mm'.format(distance_2))

        x1, y1, z1 = [value / adafruit_lis3dh.STANDARD_GRAVITY for value in accel_1.acceleration]
        accel_1_data.append(tuple((x1,y1,z1)))
        print('x1 = %0.3f G, y1 = %0.3f G, z1 = %0.3f G' % (x1, y1, z1))

        x2, y2, z2 = [value / adafruit_lis3dh.STANDARD_GRAVITY for value in accel_2.acceleration]
        accel_2_data.append(tuple((x2,y2,z2)))
        print('x2 = %0.3f G, y2 = %0.3f G, z2 = %0.3f G' % (x2, y2, z2))

        iteration.append(count)
        count = count + 1

        #Flash red every 2 seconds 
        if count % (2/sample_rate) == 0:
            GPIO.output(red_led, GPIO.HIGH)
            time.sleep(sample_rate)
            GPIO.output(red_led, GPIO.LOW)
        else:
            time.sleep(sample_rate)

        if button.is_pressed:
            time.sleep(short_wait)
            GPIO.output(red_led, GPIO.HIGH)
            time.sleep(1.00)
            GPIO.output(red_led, GPIO.LOW)
            save_data()
            break
        else:
            continue



def save_data():
    ##Saves data with pickle as an object in a txt file.
    ##File name is the time and date of recording.
    ##Clears all variables.
    ##Flashes green led and then calls ready_loop() when finished.

    print('Saving data...')
    #x = datetime.datetime.now()
    x = 'test' 
    global count
    global vl53_data
    global vl6180_data
    global accel_1_data
    global accel_2_data
    global iteration

    #print([list(c) for c in zip(vl53_data, vl6180_data, accel_1_data, accel_2_data, iteration)])
    c = [list(c) for c in zip(vl53_data, vl6180_data, accel_1_data, accel_2_data, iteration)]
    print(c)

    with open("/home/pi/example/saved_tests/{}.txt".format(x), "wb") as fp:   #Pickling
        pickle.dump(c, fp)

    #with open("/home/pi/example/saved_tests/{}.txt".format(x), "rb") as fp:   #Pickling
    #    test = pickle.load(fp)
    

    #print(test)
    print('Data saved.')
    print('Clearing data...')

    count = 0
    vl53_data = []
    vl6180_data = []
    accel_1_data = []
    accel_2_data = []
    iteration = []

    print('Data cleared.')
    time.sleep(1.5)
    for i in range(1,5):
        GPIO.output(green_led, GPIO.HIGH)
        time.sleep(short_wait)
        GPIO.output(green_led, GPIO.LOW)
        time.sleep(short_wait)

    time.sleep(medium_wait)

    #Restart loop.
    ready_loop()

#Start Loop
ready_loop()

