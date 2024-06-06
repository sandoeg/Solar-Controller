import os
import time
import sys
import json
import math
import types
# pip install Adafruit-Blinka
import board
import busio
# pip install rpi.gpio
import RPi.GPIO as GPIO
# pip install adafruit-circuitpython-ads1x15
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.ads1x15 import Mode
from adafruit_ads1x15.analog_in import AnalogIn
# pip install pyrebase4
import pyrebase

# *********************************** Constants ***********************************
# Data capture interval in seconds
# Number of intervals before publishing data
DATA_CAPTURE_INTERVAL = 1
INTERVALS_BEFORE_PUBLISHING = 4

# States in Solar Controller State Machine
# Created types.SimpleNamespace() to satisfy match / case expectations
cur_state = types.SimpleNamespace()
cur_state.INIT_STATE = 1
cur_state.HEATING_STATE = 2
cur_state.HEATING_POOL_STATE = 3
cur_state.COOLING_STATE = 4
cur_state.COOLING_POOL_STATE = 5
cur_state.MANUAL_VALVE_OFF_STATE = 6
cur_state.MANUAL_VALVE_ON_STATE = 7
cur_state.FAULT_STATE = 8

# Valve Control Modes
cur_valve_control_mode = types.SimpleNamespace()
cur_valve_control_mode.VALVE_CONTROL_OFF = 1
cur_valve_control_mode.VALVE_CONTROL_ON = 2
cur_valve_control_mode.VALVE_CONTROL_AUTO = 3 


# Temperature value constants
MIN_CONTROL_TEMP = 70                   # Minimum control and display temperature
MAX_CONTROL_TEMP = 110                  # Maximum control and display temperature
HEATING_COOLING_HYSTERESIS_TEMP = 0.2   # Temperature value applied to pool_temp readings to prevent valve chatter   
HEATING_STATE_TRANSITION_TEMP = 4.0     # Temperature difference between solar_temp and pool_temp for state transition

# Heating / Cooling Mode Values
HEATING_MODE = 1
COOLING_MODE = 2
# GPIO pin assignments
HEATING_MODE_IO = 17                    # Pin 17 output to control heating mode LED
COOLING_MODE_IO = 27                    # Pin 27 output to control cooling mode LED

# Valve States
VALVE_OFF = 0
VALVE_ON = 1
# GPIO pin assignment
VALVE_STATE_IO = 22                     # Pin 22 output to control valve state LED


# System system_status Values
INITIALIZING = 1
RUNNING = 2
FAULT_CONDITION = 3


# *********************************** Global Data ***********************************
temp_data = {1: 4.46, 2: 4.45, 3: 4.43, 4: 4.41, 5: 4.4, 6: 4.38, 7: 4.36, 8: 4.35, 9: 4.33, 10: 4.31, 11: 4.29, 12: 4.27, 13: 4.25, 14: 4.23, 15: 4.21, 16: 4.19, 17: 4.17, 18: 4.15, 19: 4.13, 20: 4.11, 21: 4.09, 22: 4.07, 23: 4.04, 24: 4.02, 25: 4.0, 26: 3.97, 27: 3.95, 28: 3.93, 29: 3.9, 30: 3.88, 31: 3.85, 32: 3.83, 33: 3.8, 34: 3.78, 35: 3.75, 36: 3.72, 37: 3.7, 38: 3.67, 39: 3.64, 40: 3.61, 41: 3.59, 42: 3.56, 43: 3.53, 44: 3.5, 45: 3.47, 46: 3.45, 47: 3.42, 48: 3.39, 49: 3.36, 50: 3.33, 51: 3.3, 52: 3.27, 53: 3.24, 54: 3.21, 55: 3.18, 56: 3.15, 57: 3.12, 58: 3.09, 59: 3.06, 60: 3.02, 61: 2.99, 62: 2.96, 63: 2.93, 64: 2.9, 65: 2.87, 66: 2.84, 67: 2.81, 68: 2.78, 69: 2.75, 70: 2.72, 71: 2.68, 72: 2.65, 73: 2.62, 74: 2.59, 75: 2.56, 76: 2.53, 77: 2.5, 78: 2.47, 79: 2.44, 80: 2.41, 81: 2.38, 82: 2.35, 83: 2.32, 84: 2.29, 85: 2.26, 86: 2.23, 87: 2.2, 88: 2.17, 89: 2.14, 90: 2.12, 91: 2.09, 92: 2.06, 93: 2.03, 94: 2.0, 95: 1.98, 96: 1.95, 97: 1.92, 98: 1.89, 99: 1.87, 100: 1.84, 101: 1.81, 102: 1.79, 103: 1.76, 104: 1.74, 105: 1.71, 106: 1.69, 107: 1.66, 108: 1.64, 109: 1.61, 110: 1.59, 111: 1.57, 112: 1.54, 113: 1.52, 114: 1.5, 115: 1.47, 116: 1.45, 117: 1.43, 118: 1.41, 119: 1.39, 120: 1.37, 121: 1.34, 122: 1.32, 123: 1.3, 124: 1.28, 125: 1.26, 126: 1.24, 127: 1.22, 128: 1.21, 129: 1.19, 130: 1.17, 131: 1.15, 132: 1.13, 133: 1.11, 134: 1.1, 135: 1.08, 136: 1.06, 137: 1.05, 138: 1.03, 139: 1.01, 140: 1.0, 141: 0.98, 142: 0.97, 143: 0.95, 144: 0.93, 145: 0.92, 146: 0.9, 147: 0.89, 148: 0.88, 149: 0.86, 150: 0.85, 151: 0.84, 152: 0.82, 153: 0.81, 154: 0.8, 155: 0.78, 156: 0.77, 157: 0.76, 158: 0.75, 159: 0.73, 160: 0.72}
pool_temp_values= [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
solar_temp_values = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
# Set initial values for valve control, pool temp control,
# heating / cooling mode, and system_status
valve_control_mode = cur_valve_control_mode.VALVE_CONTROL_OFF
pool_temp_setpoint = MIN_CONTROL_TEMP
heating_cooling_mode = HEATING_MODE
system_status = INITIALIZING
# Initialize pool and solar temperatures
pool_temp = 0.0
solar_temp = 0.0


# *********************************** ADS-1115 Setup ***********************************
# Setup I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create the ADS-1115 object configured to communicate over the I2C bus
ads = ADS.ADS1115(i2c)

# Set the gain to span to 4.096 volts full scale
#       GAIN    RANGE (V)
#       ----    ---------
#        2/3    +/- 6.144
#          1    +/- 4.096
#          2    +/- 2.048
#          4    +/- 1.024
#          8    +/- 0.512
#         16    +/- 0.256
# List of gains
gains = (2 / 3, 1, 2, 4, 8, 16)

# Set ADC gain to 1 = +/- 4.096 volt range
ads.gain = gains[1]

ads.data_rate = 8

# Set the mode for Single Shot reading
ads.mode = Mode.SINGLE

# Create single-ended inputs on channel 0 and channel 1
solartemp = AnalogIn(ads, ADS.P0)
pooltemp = AnalogIn(ads, ADS.P1)

# *********************************** GPIO Setup ***********************************
GPIO.setmode(GPIO.BCM)                          # Refer to pins using GPIOx reference

# Set LED driver pins to output
GPIO.setup(VALVE_STATE_IO, GPIO.OUT)            
GPIO.setup(HEATING_MODE_IO, GPIO.OUT)
GPIO.setup(COOLING_MODE_IO, GPIO.OUT)

# Set LED driver pins to High to turn LEDs off
GPIO.output(VALVE_STATE_IO, GPIO.HIGH)          
GPIO.output(HEATING_MODE_IO, GPIO.HIGH)
GPIO.output(COOLING_MODE_IO, GPIO.HIGH)

# Configure I/O PWM frequency
valve_state_LED = GPIO.PWM(VALVE_STATE_IO, 70) 
heating_mode_LED = GPIO.PWM(HEATING_MODE_IO, 70)
cooling_mode_LED = GPIO.PWM(COOLING_MODE_IO, 70)

# Start LED PWM with starting duty cycle = 100% which is LED off
valve_state_LED.start(100.0)
heating_mode_LED.start(100.0)
cooling_mode_LED.start(100.0)                    

# *********************************** Firebase Setup ***********************************
# Config info from firebase pyrebaserealtimedemo project
firebaseConfig = {
    "apiKey": "AIzaSyBjYfnVvFpgnhQeZtJVdcF3nnVd_5xAFac",
    "authDomain": "pyrebaserealtimedbdemo-b2e74.firebaseapp.com",
    "databaseURL":"https://pyrebaserealtimedbdemo-b2e74-default-rtdb.firebaseio.com/",
    "projectId": "pyrebaserealtimedbdemo-b2e74",
    "storageBucket": "pyrebaserealtimedbdemo-b2e74.appspot.com",
    "messagingSenderId": "1075155583430",
    "appId": "1:1075155583430:web:b7947134aac82011e672c7"
}

firebase = pyrebase.initialize_app(firebaseConfig)

# Create instance of the realtime database
db = firebase.database()


# *********************************** Local Functions ***********************************

def get_avg_pool_temperature():
    global pool_temp_values
    global pooltemp
    
    # A/D pool temperature value in voltage
    x = pooltemp.voltage
    x2 = math.pow(x,2)
    # Equation derived from Excel polynomial line fit
    y = (3.009*x2)-(48.319*x)+178.86
    #print('pool_temp: ', y)
    # Get average temperature of the pool sensor readings 
    avg_pool_temp = get_temp_average(pool_temp_values, y)
    return(avg_pool_temp)


def get_avg_solar_temperature():
    global solar_temp_values
    global solartemp

    # A/D solar temperature value in voltage
    x = solartemp.voltage
    x2 = math.pow(x,2)
    # Equation derived from Excel polynomial line fit
    y = (3.009*x2)-(48.319*x)+178.86
    #print('solar_temp: ', y)
    # Get average temperature of the pool sensor readings 
    avg_solar_temp = get_temp_average(solar_temp_values, y)
    return(avg_solar_temp)


def get_temp_average(temp_list, temp_value):
    # Shift temp_list items right from 1 to end of the list making room for new temp_value
    for x in range(len(temp_list)-1, 0, -1):
        temp_list[x] = temp_list[x-1]

     # Add new temp_value to position 0    
    temp_list[0]= temp_value
    
    # Calculate the average of the values in temp_list
    avg_temp = math.fsum(temp_list) / len(temp_list)

    # Return the average of the values in temp_list
    return(avg_temp)

         
def get_status_data():
    global pool_temp_setpoint
    global valve_control_mode
    global heating_cooling_mode
    global system_status

    system_status = db.child("solar_controller").child("system_status").get(system_status).val()
    pool_temp_setpoint = db.child("solar_controller").child("pool_temp_setpoint").get(pool_temp_setpoint).val()
    valve_control_mode = db.child("solar_controller").child("valve_control_mode").get(valve_control_mode).val()
    heating_cooling_mode = db.child("solar_controller").child("heating_cooling_mode").get(heating_cooling_mode).val()  


def send_temperature_data():    
    global solar_temp
    global pool_temp

    # Get temperature data
    avg_solar_temp = get_avg_solar_temperature()
    avg_pool_temp = get_avg_pool_temperature()
    # Update firebase dB with pool and solar average temperatures
    solartemp = f'{avg_solar_temp: .1f}'
    pooltemp = f'{avg_pool_temp: .1f}'
    solar_temp = float(solartemp)
    pool_temp = float(pooltemp)
    db.child("solar_controller").child("solar_temp").set(solar_temp)
    db.child("solar_controller").child("pool_temp").set(pool_temp)



def main():
    global system_status            # INITIALIZING
    global heating_cooling_mode     # HEATING_MODE
    global pool_temp_setpoint       # MIN_CONTROL_TEMP
    global valve_control_mode       # VALVE_CONTROL_OFF
    global pool_temp
    global solar_temp
    valve_state = VALVE_OFF         # Initialize valve state to OFF

    try:
        # Set state machine starting state
        state = cur_state.INIT_STATE

        while True:

            if system_status == RUNNING:
                # Get system_status, pool_temp_setpoint, valve_control_mode, 
                # and heating_cooling_mode from firebase
                get_status_data()
                # Get temperature sensor data from A/D and send to firebase
                send_temperature_data()

                match valve_control_mode:
                    
                    case cur_valve_control_mode.VALVE_CONTROL_ON:
                        if (valve_control_mode != last_valve_control_mode):
                            print('Digital Output > Turn valve ON')
                            valve_state = VALVE_ON
                            valve_state_LED.ChangeDutyCycle(85)
                            db.child("solar_controller").child("valve_state").set(valve_state)
                        
                        last_valve_control_mode  = cur_valve_control_mode.VALVE_CONTROL_ON
                        # Set state to the manual valve ON state
                        state = cur_state.MANUAL_VALVE_ON_STATE

                    case cur_valve_control_mode.VALVE_CONTROL_OFF:
                        if (valve_control_mode != last_valve_control_mode):
                            print('Digital Output > Turn valve OFF')
                            valve_state = VALVE_OFF
                            valve_state_LED.ChangeDutyCycle(100)
                            db.child("solar_controller").child("valve_state").set(valve_state)

                        last_valve_control_mode = cur_valve_control_mode.VALVE_CONTROL_OFF    
                        # Set state to the manual valve OFF state
                        state = cur_state.MANUAL_VALVE_OFF_STATE

                    case cur_valve_control_mode.VALVE_CONTROL_AUTO:
                        # IF heating mode is heating 
                        # AND state != HEATING_STATE or HEATING_POOL_STATE
                        # THEN set state to HEATING_STATE
                        # ELSE IF heating mode is cooling
                        # AND state != COOLING_STATE or COOLING_POOL_STATE
                        # THEN set state to COOLING_STATE
                        if (heating_cooling_mode == HEATING_MODE):
                            if(state != cur_state.HEATING_STATE and state != cur_state.HEATING_POOL_STATE):
                                cooling_mode_LED.ChangeDutyCycle(100)
                                heating_mode_LED.ChangeDutyCycle(90)
                                state = cur_state.HEATING_STATE
                        elif (heating_cooling_mode == COOLING_MODE):
                            if (state != cur_state.COOLING_STATE and state != cur_state.COOLING_POOL_STATE):
                                heating_mode_LED.ChangeDutyCycle(100)
                                cooling_mode_LED.ChangeDutyCycle(0)
                                state = cur_state.COOLING_STATE
                            
            # Solar Controller State Machine
            match state:
                #-----------------------------------
                #          INIT_STATE
                #-----------------------------------
                case cur_state.INIT_STATE:
                    print('state = INIT_STATE')
                    # Get average of 20 pool and solar temperature readings
                    for i in range(20):
                        avg_pool_temp = get_avg_pool_temperature()
                        avg_solar_temp = get_avg_solar_temperature()
                        print(i)

                    # Set system_status to running
                    system_status = RUNNING

                    # Set control values
                    db.child("solar_controller").child("system_status").set(system_status)
                    db.child("solar_controller").child("pool_temp_setpoint").set(pool_temp_setpoint)
                    db.child("solar_controller").child("valve_control_mode").set(valve_control_mode)
                    db.child("solar_controller").child("heating_cooling_mode").set(heating_cooling_mode)  
                    # Turn valve off
                    print('Digital Output > Turn valve OFF')
                    valve_state = VALVE_OFF
                    valve_state_LED.ChangeDutyCycle(100)
                    db.child("solar_controller").child("valve_state").set(valve_state)
                    last_valve_control_mode = cur_valve_control_mode.VALVE_CONTROL_OFF
                    state = cur_state.MANUAL_VALVE_OFF_STATE
                
                #-----------------------------------
                #          HEATING_STATE
                #-----------------------------------
                case cur_state.HEATING_STATE:
                    print('state = HEATING_STATE')
                    # IF the solar temp is 4 degrees higher than the pool temperature
                    # AND the pool temp is less than the pool temp control setting 
                    # THEN turn the valve on and transition to the HEATING_POOL_STATE 
                    # ELSE stay in this state
                    if ((solar_temp - pool_temp > HEATING_STATE_TRANSITION_TEMP) and (pool_temp < pool_temp_setpoint)):
                        if(valve_state == VALVE_OFF):
                            print('Digital Output > Turn valve ON')
                            valve_state = VALVE_ON
                            valve_state_LED.ChangeDutyCycle(85)
                            db.child("solar_controller").child("valve_state").set(valve_state)
                            state = cur_state.HEATING_POOL_STATE
                            pool_temp_count = 0
                    elif (valve_state == VALVE_ON):
                        print('Digital Output > Turn valve OFF')
                        valve_state = VALVE_OFF
                        valve_state_LED.ChangeDutyCycle(100)
                        db.child("solar_controller").child("valve_state").set(valve_state)
                    
                #-----------------------------------
                #          HEATING_POOL_STATE
                #-----------------------------------
                case cur_state.HEATING_POOL_STATE:
                    print('state = HEATING_POOL_STATE')
                    # IF the solar temp - pool temp is less than 1.5 degrees 
                    # OR the pool temp is greater than the pool temp control setting
                    #    Note: The HEATING_COOLING_HYSTERESIS_TEMP value should prevent 
                    #    the system from switching from HEATING_POOL_STATE to HEATING_STATE 
                    #    on small changes of pool_temp causing valve chatter (on/off/on/off)
                    # THEN turn the valve off and transition to the HEATING_STATE
                    # ELSE stay in this state
                    if (solar_temp - pool_temp < 1.5): 
                        if(valve_state == VALVE_ON):
                            print('Digital Output > Turn valve OFF')
                            valve_state = VALVE_OFF
                            valve_state_LED.ChangeDutyCycle(100)
                            db.child("solar_controller").child("valve_state").set(valve_state)
                        state = cur_state.HEATING_STATE
                    elif ((pool_temp > pool_temp_setpoint)):
                        pool_temp_count += 1
                        print('pool_temp_count: ', pool_temp_count)
                        if(pool_temp_count >= 10):
                            if(valve_state == VALVE_ON):
                                print('Digital Output > Turn valve OFF')
                                valve_state = VALVE_OFF
                                valve_state_LED.ChangeDutyCycle(100)
                                db.child("solar_controller").child("valve_state").set(valve_state)
                            state = cur_state.HEATING_STATE
                    elif ((pool_temp <= pool_temp_setpoint)):
                        pool_temp_count = 0

                #-----------------------------------
                #          COOLING_STATE
                #-----------------------------------
                case cur_state.COOLING_STATE:
                    print('state = COOLING_STATE')
                    # IF the solar temp is greater than the pool control setting
                    # AND the pool temp is greater than the pool control setting
                    # THEN turn the valve on and transition to the COOLING_POOL_STATE
                    # ELSE stay in this state
                    if (solar_temp > pool_temp_setpoint and pool_temp > pool_temp_setpoint):
                        if (valve_state == VALVE_OFF):
                            print('Digital Output > Turn valve ON')
                            valve_state = VALVE_ON
                            valve_state_LED.ChangeDutyCycle(85)
                            db.child("solar_controller").child("valve_state").set(valve_state)
                            state = cur_state.COOLING_POOL_STATE
                            pool_temp_count = 0
                    elif (valve_state == VALVE_ON):
                        print('Digital Output > Turn valve OFF')
                        valve_state = VALVE_OFF
                        valve_state_LED.ChangeDutyCycle(100)
                        db.child("solar_controller").child("valve_state").set(valve_state)
                
                #-----------------------------------
                #          COOLING_POOL_STATE
                #-----------------------------------
                case cur_state.COOLING_POOL_STATE:
                    print('state = COOLING_POOL_STATE')
                    # IF solar temp is 3 degrees less than the pool temp
                    # OR the pool temp is less than the pool control setting
                    #    Note: The HEATING_COOLING_HYSTERESIS_TEMP value should prevent 
                    #    the system from switching from COOLING_POOL_STATE to COOLING_STATE 
                    #    on small changes of pool_temp causing valve chatter (on/off/on/off)
                    # THEN turn the valve off and transition to the COOLING_STATE
                    # ELSE stay in this state
                    if (pool_temp - solar_temp > 3.0): 
                        if(valve_state == VALVE_OFF):
                            print('Digital Output > Turn valve OFF')
                            valve_state = VALVE_OFF
                            valve_state_LED.ChangeDutyCycle(100)
                            db.child("solar_controller").child("valve_state").set(valve_state)
                        state = cur_state.COOLING_STATE
                    elif ((pool_temp < pool_temp_setpoint)):
                        pool_temp_count += 1
                        print('pool_temp_count: ', pool_temp_count)
                        if(pool_temp_count >= 10):
                            if(valve_state == VALVE_OFF):
                                print('Digital Output > Turn valve OFF')
                                valve_state = VALVE_OFF
                                valve_state_LED.ChangeDutyCycle(100)
                                db.child("solar_controller").child("valve_state").set(valve_state)
                            state = cur_state.COOLING_STATE
                    elif ((pool_temp >= pool_temp_setpoint)):
                        pool_temp_count = 0

                #-----------------------------------
                #       MANUAL_VALVE_OFF_STATE
                #-----------------------------------
                case cur_state.MANUAL_VALVE_OFF_STATE:
                    print('state = MANUAL_VALVE_OFF_STATE')
                    if valve_control_mode == cur_valve_control_mode.VALVE_CONTROL_ON:
                        state = cur_state.MANUAL_VALVE_ON_STATE
                    elif valve_control_mode == cur_valve_control_mode.VALVE_CONTROL_AUTO:
                        if heating_cooling_mode == HEATING_MODE:
                            state = cur_state.HEATING_STATE
                        else: # heating_cooling_mode = COOLING_MODE
                            state = cur_state.COOLING_STATE                   
               
                #-----------------------------------
                #         MANUAL_VALVE_ON_STATE
                #-----------------------------------
                case cur_state.MANUAL_VALVE_ON_STATE:
                    print('state = MANUAL_VALVE_ON_STATE')
                    if valve_control_mode == cur_valve_control_mode.VALVE_CONTROL_OFF:
                        state = cur_state.MANUAL_VALVE_OFF_STATE
                    elif valve_control_mode == cur_valve_control_mode.VALVE_CONTROL_AUTO:
                        if heating_cooling_mode == HEATING_MODE:
                            state = cur_state.HEATING_STATE
                        else: # heating_cooling_mode = COOLING_MODE
                            state = cur_state.COOLING_STATE                  
               
                #-----------------------------------
                #          FAULT_STATE
                #-----------------------------------
                case cur_state.FAULT_STATE:
                    print('state = FAULT_STATE')

            # Sleep for a little bit. Don't want to stress system or 
            # firebase dB and website
            time.sleep(DATA_CAPTURE_INTERVAL)

    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
    # Clean up I/O by turning LEDs off and stopping PWM
    GPIO.output(VALVE_STATE_IO, GPIO.HIGH)
    GPIO.output(HEATING_MODE_IO, GPIO.HIGH)
    GPIO.output(COOLING_MODE_IO, GPIO.HIGH)
    GPIO.cleanup()
