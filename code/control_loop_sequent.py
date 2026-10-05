

# Temperature feedback loop with fixed setpoint

# As temperature gets closer to setpoint, pwm duty cycle reduced
# If temperature is hotter, switch to cool, if cooler, switch to heat

import RPi.GPIO as GPIO
import sys
import time
from datetime import datetime
import matplotlib.pyplot as plt
import numpy
from collections import deque

# Enable interactive mode
plt.ion()

import sm_tc

# Paramters -----------
setpoint = 40 # celsius
peltier_or_exchanger = 'sample'

# Peltier 1
EN = 13
FW = 26
BK = 5

# Peltier 2
EN2 = 18
FW2 = 27
BK2 = 23

# ---------------------

GPIO.setmode(GPIO.BCM)

def calc_duty_cycle(temp_diff):
    duty = 100
    if temp_diff < 1:
        duty = round(temp_diff, 2)*100
        
    print(temp_diff, duty)
         
    return duty

class Peltier:
    def __init__(self, enable_pin=13, forward_pin=26, backward_pin=5):
        self.enable_pin = enable_pin
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin

        GPIO.setup(self.enable_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.enable_pin, 1000)# 1KHz

        GPIO.setup(self.forward_pin, GPIO.OUT)
        GPIO.setup(self.backward_pin, GPIO.OUT)

        self.pwm.start(0) # start at 0% duty cycle

    def heat(self, duty_cycle):
        GPIO.output(self.forward_pin, GPIO.HIGH)
        GPIO.output(self.backward_pin, GPIO.LOW)
        self.pwm.ChangeDutyCycle(duty_cycle)

    def cool(self, duty_cycle):
        GPIO.output(self.backward_pin, GPIO.HIGH)
        GPIO.output(self.forward_pin, GPIO.LOW)     
        self.pwm.ChangeDutyCycle(duty_cycle)        


# Control Loop ----------
# Setup thermocouple
sensor = sm_tc.SMtc(0)
peltier1 = Peltier(EN, FW, BK)
peltier2 = Peltier(EN2, FW2, BK2)

fig, (ax1, ax2) = plt.subplots(2, 1)

DT = deque(maxlen=1000)
duty = deque(maxlen=1000)
sample_T = deque(maxlen=1000)
peltier_T = deque(maxlen=1000)
ambient_T = deque(maxlen=1000)

l1, = ax1.plot(DT, sample_T, color='C0', label='Sample')
l2, = ax1.plot(DT, peltier_T, color='C1', label='Peltier1')
l3, = ax1.plot(DT, ambient_T, color='C2', label='Peltier2')

ax1.axhline(y=setpoint, color='r', linestyle='--')
l4, = ax2.plot(DT, duty, 'b')
ax2.set_title('Duty cycle (%)')

ax1.legend(loc='upper left')

# Set up axis limits
ax1.set_ylim(0, 100)  # The x-axis is fixed to 50 points
ax2.set_ylim(0, 105)  # Adjust this according to your data range

counter = 0
while True:
    sample_t = sensor.get_temp(1) 
    peltier_t = sensor.get_temp(2) 
    ambient_t = sensor.get_temp(3) 

    dt = datetime.now()

    if peltier_or_exchanger == 'peltier':
        diff = peltier_t - setpoint
    else:
        diff = sample_t - setpoint
          
    duty_cycle = calc_duty_cycle(abs(diff))

    if diff > 0:
        peltier1.cool(duty_cycle)
        peltier2.cool(duty_cycle)
        print('Cooling')
    else:
        peltier1.heat(duty_cycle)
        peltier2.heat(duty_cycle)
        print('Heating')
        
    DT.append(dt)
    duty.append(duty_cycle)
    sample_T.append(sample_t)
    peltier_T.append(peltier_t)
    ambient_T.append(ambient_t)
    
    l1.set_data(DT, sample_T)
    l2.set_data(DT, peltier_T)
    l3.set_data(DT, ambient_T)
    l4.set_data(DT, duty)
    

    # Adjust x-axis dynamically to match incoming data
    ax1.set_xlim(min(DT, default=0), max(DT, default=50))
    ax2.set_xlim(min(DT, default=0), max(DT, default=50))
    ax1.set_ylim(min(list(sample_T) + list(peltier_T) + list(ambient_T) + [setpoint], default=0)-5, max(list(sample_T) + list(peltier_T) + list(ambient_T) + [setpoint], default=0)+5)


    plt.draw()
    plt.pause(0.25)

    counter += 1


