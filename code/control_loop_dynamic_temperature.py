# Dynamic temperature feedback loop
#
# The requested temperature varies sinusoidally between MIN_TEMP and MAX_TEMP.
# START_SETPOINT determines where in that cycle the experiment begins.
# FREQUENCY_CYCLES_PER_HOUR determines how quickly the temperature cycles.
# MAX_DUTY_CYCLE and DEADBAND_C can be used to reduce overshoot / rapid switching.
#
# Data are plotted live and written continuously to CSV.

import csv
import math
import time
from datetime import datetime
from collections import deque

import matplotlib.pyplot as plt
import RPi.GPIO as GPIO

import sm_tc

# -----------------------------------------------------------------------------
# USER SETTINGS
# -----------------------------------------------------------------------------

START_SETPOINT = 20.0          # deg C; starting point of the dynamic target
MIN_TEMP = 18.0                # deg C; minimum target temperature
MAX_TEMP = 22.0                # deg C; maximum target temperature
FREQUENCY_CYCLES_PER_HOUR = 1  # 1 = one complete min -> max -> min cycle per hour

# Controller tuning
MAX_DUTY_CYCLE = 60.0          # maximum PWM output (%)
PROPORTIONAL_BAND_C = 1.0      # error at which MAX_DUTY_CYCLE is reached
DEADBAND_C = 0.05              # no heating/cooling within +/- this error
LOOP_INTERVAL_S = 0.25

# Use either the sample temperature or Peltier temperature for feedback
PELTIER_OR_EXCHANGER = 'sample'  # 'sample' or 'peltier'

CSV_FILENAME = datetime.now().strftime('temperature_control_%Y%m%d_%H%M%S.csv')

# Peltier 1
EN = 13
FW = 26
BK = 5

# Peltier 2
EN2 = 18
FW2 = 27
BK2 = 23

# -----------------------------------------------------------------------------

if MIN_TEMP > MAX_TEMP:
    raise ValueError('MIN_TEMP must be <= MAX_TEMP')

if not MIN_TEMP <= START_SETPOINT <= MAX_TEMP:
    raise ValueError('START_SETPOINT must lie between MIN_TEMP and MAX_TEMP')

if FREQUENCY_CYCLES_PER_HOUR < 0:
    raise ValueError('FREQUENCY_CYCLES_PER_HOUR must be >= 0')

if not 0 <= MAX_DUTY_CYCLE <= 100:
    raise ValueError('MAX_DUTY_CYCLE must be between 0 and 100')

GPIO.setmode(GPIO.BCM)
plt.ion()


def dynamic_setpoint(elapsed_s):
    """Return the requested temperature at elapsed_s.

    The target is a sine wave bounded by MIN_TEMP and MAX_TEMP.  The phase is
    chosen so that the first target is START_SETPOINT.

    Setting FREQUENCY_CYCLES_PER_HOUR to 0 gives a fixed START_SETPOINT.
    """
    if FREQUENCY_CYCLES_PER_HOUR == 0 or MAX_TEMP == MIN_TEMP:
        return START_SETPOINT

    centre = (MIN_TEMP + MAX_TEMP) / 2.0
    amplitude = (MAX_TEMP - MIN_TEMP) / 2.0

    # Start the sine wave at the requested initial setpoint.
    start_fraction = (START_SETPOINT - centre) / amplitude
    start_fraction = max(-1.0, min(1.0, start_fraction))
    phase = math.asin(start_fraction)

    frequency_hz = FREQUENCY_CYCLES_PER_HOUR / 3600.0
    return centre + amplitude * math.sin(
        2.0 * math.pi * frequency_hz * elapsed_s + phase
    )


def calc_duty_cycle(temp_diff):
    """Simple proportional output with a deadband and user-defined PWM cap."""
    temp_diff = abs(temp_diff)

    if temp_diff <= DEADBAND_C:
        return 0.0

    effective_error = temp_diff - DEADBAND_C
    duty = (effective_error / PROPORTIONAL_BAND_C) * MAX_DUTY_CYCLE
    return round(max(0.0, min(MAX_DUTY_CYCLE, duty)), 2)


class Peltier:
    def __init__(self, enable_pin=13, forward_pin=26, backward_pin=5):
        self.enable_pin = enable_pin
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin

        GPIO.setup(self.enable_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.enable_pin, 1000)  # 1 kHz

        GPIO.setup(self.forward_pin, GPIO.OUT)
        GPIO.setup(self.backward_pin, GPIO.OUT)

        self.pwm.start(0)

    def heat(self, duty_cycle):
        GPIO.output(self.forward_pin, GPIO.HIGH)
        GPIO.output(self.backward_pin, GPIO.LOW)
        self.pwm.ChangeDutyCycle(duty_cycle)

    def cool(self, duty_cycle):
        GPIO.output(self.backward_pin, GPIO.HIGH)
        GPIO.output(self.forward_pin, GPIO.LOW)
        self.pwm.ChangeDutyCycle(duty_cycle)

    def stop(self):
        self.pwm.ChangeDutyCycle(0)
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.backward_pin, GPIO.LOW)


# -----------------------------------------------------------------------------
# CONTROL LOOP
# -----------------------------------------------------------------------------

sensor = sm_tc.SMtc(0)
peltier1 = Peltier(EN, FW, BK)
peltier2 = Peltier(EN2, FW2, BK2)

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
fig.tight_layout(pad=2.0)

DT = deque(maxlen=1000)
target_T = deque(maxlen=1000)
sample_T = deque(maxlen=1000)
peltier_T = deque(maxlen=1000)
peltier2_T = deque(maxlen=1000)
control_output = deque(maxlen=1000)

l1, = ax1.plot([], [], label='Sample')
l2, = ax1.plot([], [], label='Peltier 1')
l3, = ax1.plot([], [], label='Peltier 2')
l4, = ax1.plot([], [], '--', label='Target')
l5, = ax2.plot([], [], label='Control output')

ax1.set_ylabel('Temperature (deg C)')
ax2.set_ylabel('PWM (%)')
ax2.set_xlabel('Time')
ax2.set_ylim(-105, 105)
ax2.axhline(0, linewidth=0.8)
ax1.legend(loc='upper left')
ax2.legend(loc='upper left')

start_time = time.monotonic()

with open(CSV_FILENAME, 'w', newline='') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow([
        'timestamp',
        'elapsed_s',
        'target_temp_C',
        'sample_temp_C',
        'peltier1_temp_C',
        'peltier2_temp_C',
        'mode',
        'duty_cycle_pct',
        'signed_control_pct',
    ])
    csv_file.flush()

    try:
        while True:
            sample_t = sensor.get_temp(1)
            peltier_t = sensor.get_temp(2)
            peltier2_t = sensor.get_temp(3)

            now = datetime.now()
            elapsed_s = time.monotonic() - start_time
            target = dynamic_setpoint(elapsed_s)

            if PELTIER_OR_EXCHANGER == 'peltier':
                measured_t = peltier_t
            else:
                measured_t = sample_t

            # Positive error means the system is hotter than the target.
            error = measured_t - target
            duty_cycle = calc_duty_cycle(error)

            if duty_cycle == 0:
                peltier1.stop()
                peltier2.stop()
                mode = 'off'
                signed_control = 0.0
            elif error > 0:
                peltier1.cool(duty_cycle)
                peltier2.cool(duty_cycle)
                mode = 'cooling'
                signed_control = -duty_cycle
            else:
                peltier1.heat(duty_cycle)
                peltier2.heat(duty_cycle)
                mode = 'heating'
                signed_control = duty_cycle

            print(
                f'Target: {target:5.2f} C | Measured: {measured_t:5.2f} C | '
                f'{mode:7s} | duty: {duty_cycle:5.1f}%'
            )

            writer.writerow([
                now.isoformat(timespec='milliseconds'),
                round(elapsed_s, 3),
                round(target, 3),
                round(sample_t, 3),
                round(peltier_t, 3),
                round(peltier2_t, 3),
                mode,
                duty_cycle,
                signed_control,
            ])
            csv_file.flush()

            DT.append(now)
            target_T.append(target)
            sample_T.append(sample_t)
            peltier_T.append(peltier_t)
            peltier2_T.append(peltier2_t)
            control_output.append(signed_control)

            l1.set_data(DT, sample_T)
            l2.set_data(DT, peltier_T)
            l3.set_data(DT, peltier2_T)
            l4.set_data(DT, target_T)
            l5.set_data(DT, control_output)

            if len(DT) > 1:
                ax1.set_xlim(DT[0], DT[-1])

            all_temperatures = (
                list(sample_T)
                + list(peltier_T)
                + list(peltier2_T)
                + list(target_T)
            )
            if all_temperatures:
                ax1.set_ylim(min(all_temperatures) - 2, max(all_temperatures) + 2)

            fig.canvas.draw_idle()
            plt.pause(LOOP_INTERVAL_S)

    except KeyboardInterrupt:
        print('\nStopping temperature control...')

    finally:
        peltier1.stop()
        peltier2.stop()
        GPIO.cleanup()
        plt.ioff()
        plt.show()
        print(f'Data saved to: {CSV_FILENAME}')
