import board
import random
import os
from time import sleep
import adafruit_vl53l0x 
import busio
#import adafruit_bh1750 future light sensor for headlights
#pip3 install adafruit-circuitpython-vl53l0x
#https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/python-circuitpython
from adafruit_motorkit import MotorKit
from time import sleep

version = "v0.1"

#i2c = board.I2C()
i2c = busio.I2C(board.SCL, board.SDA)

# Time of Flight LIDAR Sensors
tof0 = adafruit_vl53l0x.VL53L0X(i2c)
#           tof0.measurement_timing_budget = 100000 # 200000 is the default, 200ms

# Ambient light sensor
#amb_light = adafruit_bh1750.BH1750(i2c)

# Motor driver
kit = MotorKit()

# Options
invert_dir = 1
avoid_threshold = 200 # distance in mm before turning
speed = 0.5
follow_dist = 100
turn_speed = 0.5
headlight_threshold = 200


def cc():   # shortens this long command to just cc()
    os.system("cls" if os.name == "nt" else "clear")    # clears terminal
cc()


def stop():
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0

try:
    def motor(move_type, speed):

        if move_type == "back":
            print("def() back")
            kit.motor1.throttle = speed
            kit.motor2.throttle = speed

        elif move_type == "for": 
            print("def() for")
            kit.motor1.throttle = -speed
            kit.motor2.throttle = -speed

        elif move_type == "left": # uses tank steering
            print("def() left")
            kit.motor1.throttle = turn_speed
            kit.motor2.throttle = -turn_speed

        elif move_type == "right":
            print("def() right")
            kit.motor1.throttle = -turn_speed
            kit.motor2.throttle = turn_speed
        # these were inverted
        """if move_type == "for":
            print("def() for")
            kit.motor1.throttle = speed
            kit.motor2.throttle = speed

        elif move_type == "back": 
            print("def() back")
            kit.motor1.throttle = -speed
            kit.motor2.throttle = -speed

        elif move_type == "left": # uses tank steering
            print("def() left")
            kit.motor1.throttle = turn_speed
            kit.motor2.throttle = -turn_speed

        elif move_type == "right":
            print("def() right")
            kit.motor1.throttle = -turn_speed
            kit.motor2.throttle = turn_speed"""


    """def headlight(): # this should be a thread
        while True:
            if amb_light.lux < headlight_threshold:
                print("Low Light detected")
                #light strip on
            else:
                #lights off
            print("%.2f Lux" % amb_light.lux)
            sleep(2)"""


    def object_avoidance(): # uses two sensors to detect the best direction to go
        while True:
            # average of both sensors
            lid0 = tof0.range #fetch range

            cc()
            print("Range: {}mm".format(lid0))

            
            motor("for", speed)

            # get range (both sensors averaged)
            # if range is less than avoid_threshold, turn towards the sensor that is further away
            # if range is greater than avoid_threshold, continue forwards

            if lid0 < avoid_threshold:
                print("Object detected")
                stop()
            
            sleep(0.1)
    
                    
    def follow_at_distance():
        while True:
            lid0 = tof0.range
            cc()
            if lid0 < follow_dist + 1 or lid0 < follow_dist - 1:    # checks that the robot is within 2mm of object threshold
                print("Range: {}mm".format(lid0))
                print("Moving backwards")
                motor("back", speed)
                sleep(0.3)

            elif lid0 > follow_dist + 1 or lid0 > follow_dist - 1:
                print("Range: {}mm".format(lid0))
                print("Moving forwards")
                motor("for", speed)
                sleep(0.3)


    def slow_speedup():
        ss_s = 0.2 # slow speedup speed
        while True:
            cc()
            print("Slow speedup")
            print("Current speed: {:5.2f}".format(ss_s))

            motor("for", ss_s)
            ss_s += 0.01
            sleep(0.1)

            if ss_s >= 1:
                print("max speed reached, stopping soon {:5.2f}".format(ss_s))
                sleep(3)
                break



        cc()
        stop()
        menu()


    def test():
        while True:
            cc()
            motor("for", speed)
            sleep(1)
            cc()
            motor("back", speed)
            sleep(1)
            cc()
            motor("left", speed)
            sleep(0.3)


    # https://www.elecfreaks.com/learn-en/microbitKit/smart_cutebot/index.html
    # https://docs.circuitpython.org/projects/tca9548a/en/latest/examples.html
    def menu():
        print("Redneck robot " + version)
        print("Select move type: ")
        print("1. Object Avoidance") # could use both sensors to steer in most open direction, or use 
        # 1 to just detect, then change course
        print("2. Follow At Fixed Distance")
        print("3. Test")
        print("4. Stop motors")
        print("5. Slow speedup")
        #print("3. Speed Up Gradually")
        #print("4. Random")
        #print("5. Remote Control")
        ask_menu = input("What would you like to do: ")
        if ask_menu == "1":
            object_avoidance()
        elif ask_menu == "2":
            follow_at_distance()
        elif ask_menu == "3":
            test()
        elif ask_menu == "4":
            stop()
            cc()
            print("Motors stopped")
            menu()
        elif ask_menu == "5":
            slow_speedup()
    menu()


except KeyboardInterrupt:
    print("Stopping")
    stop()
    #lights off
        



def old(): 
    avoid_dir = ("left", "right")   
    random_avoid_dir = random.choice(avoid_dir) 
    
    # boot up animation
    # while this is set to turn for 1 second,
    # it could use a magnetometer to determine the starting position and return to that
    motor(random_avoid_dir, 1)  # boot up animation
    sleep(1)
    stop()
        
    while True:
        while not tof0.data_ready:
            pass
        tof0.clear_interrupt()
        range0 = tof0.distance # this is in centimeters
        print("Distance: {} cm".format(range0))

    

        motor("for", 1)

        # prints status info about what the robot is doing
        print("Moving ")
        print("Range to object: {} cm".format(range0))


        if range0 < avoid_threshold:
            print("Yo mama detected")
            stop() # might not be needed
            
            motor(random_avoid_dir, 1)
            if range0 > avoid_threshold + 1:    # scans for a new route
                motor("for", 1)
            else:
                print("Error - stuck. add a feature where it keeps turning until it finds a new route")
                #break

    
    
print("main() done")



