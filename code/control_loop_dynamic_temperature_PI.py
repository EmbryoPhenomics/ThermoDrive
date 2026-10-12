# Dynamic temperature control using a simple PI controller.
# Positive control output = heating, negative = cooling.

import csv
import math
import time
from datetime import datetime
from collections import deque

import matplotlib.pyplot as plt
import RPi.GPIO as GPIO

import sm_tc


# -----------------------------------------------------------------------------
# SETTINGS
# -----------------------------------------------------------------------------

START_SETPOINT = 20.0
MIN_TEMP = 16.0
MAX_TEMP = 24.0
FREQUENCY_CYCLES_PER_HOUR = 2

# Controller tuning
KP = 25.0                    # PWM % per deg C error
KI = 0.08                    # PWM % per deg C-second
MAX_DUTY_CYCLE = 60.0
OUTPUT_DEADBAND = 3.0        # ignore very small PWM demands
REVERSAL_THRESHOLD = 5.0    # demand required before changing heat/cool direction
FILTER_ALPHA = 0.10          # 0-1; lower = more smoothing
CONTROL_INTERVAL_S = 1.0
LOOP_INTERVAL_S = 0.25

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


if MIN_TEMP > MAX_TEMP:
    raise ValueError('MIN_TEMP must be <= MAX_TEMP')

if not MIN_TEMP <= START_SETPOINT <= MAX_TEMP:
    raise ValueError('START_SETPOINT must lie between MIN_TEMP and MAX_TEMP')

if not 0 <= MAX_DUTY_CYCLE <= 100:
    raise ValueError('MAX_DUTY_CYCLE must be between 0 and 100')


GPIO.setmode(GPIO.BCM)
plt.ion()


def dynamic_setpoint(elapsed_s):
    if FREQUENCY_CYCLES_PER_HOUR == 0 or MIN_TEMP == MAX_TEMP:
        return START_SETPOINT

    centre = (MIN_TEMP + MAX_TEMP) / 2.0
    amplitude = (MAX_TEMP - MIN_TEMP) / 2.0

    start_fraction = (START_SETPOINT - centre) / amplitude
    start_fraction = max(-1.0, min(1.0, start_fraction))
    phase = math.asin(start_fraction)

    frequency_hz = FREQUENCY_CYCLES_PER_HOUR / 3600.0
    return centre + amplitude * math.sin(
        2 * math.pi * frequency_hz * elapsed_s + phase
    )


class Peltier:
    def __init__(self, enable_pin, forward_pin, backward_pin):
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin

        GPIO.setup(enable_pin, GPIO.OUT)
        GPIO.setup(forward_pin, GPIO.OUT)
        GPIO.setup(backward_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(enable_pin, 1000)
        self.pwm.start(0)

    def heat(self, duty):
        GPIO.output(self.forward_pin, GPIO.HIGH)
        GPIO.output(self.backward_pin, GPIO.LOW)
        self.pwm.ChangeDutyCycle(duty)

    def cool(self, duty):
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.backward_pin, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(duty)

    def stop(self):
        self.pwm.ChangeDutyCycle(0)
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.backward_pin, GPIO.LOW)


sensor = sm_tc.SMtc(0)
peltier1 = Peltier(EN, FW, BK)
peltier2 = Peltier(EN2, FW2, BK2)

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
fig.tight_layout(pad=2.0)

DT = deque(maxlen=1000)
target_T = deque(maxlen=1000)
sample_T = deque(maxlen=1000)
filtered_T = deque(maxlen=1000)
peltier_T = deque(maxlen=1000)
peltier2_T = deque(maxlen=1000)
control_history = deque(maxlen=1000)

l1, = ax1.plot([], [], label='Sample')
l2, = ax1.plot([], [], label='Filtered sample')
l3, = ax1.plot([], [], '--', label='Target')
l4, = ax2.plot([], [], label='Control output')

ax1.set_ylabel('Temperature (deg C)')
ax2.set_ylabel('PWM (%)')
ax2.set_xlabel('Time')
ax2.set_ylim(-105, 105)
ax2.axhline(0, linewidth=0.8)
ax1.legend(loc='upper left')
ax2.legend(loc='upper left')

start_time = time.monotonic()
last_control_time = start_time
filtered_temp = None
integral = 0.0
control_output = 0.0
mode = 'off'
last_direction = None


with open(CSV_FILENAME, 'w', newline='') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow([
        'timestamp',
        'elapsed_s',
        'target_temp_C',
        'sample_temp_C',
        'filtered_temp_C',
        'peltier1_temp_C',
        'peltier2_temp_C',
        'error_C',
        'mode',
        'signed_control_pct',
    ])
    csv_file.flush()

    try:
        while True:
            sample_t = sensor.get_temp(1)
            peltier_t = sensor.get_temp(2)
            peltier2_t = sensor.get_temp(3)

            now = datetime.now()
            monotonic_now = time.monotonic()
            elapsed_s = monotonic_now - start_time
            target = dynamic_setpoint(elapsed_s)

            measured_t = peltier_t if PELTIER_OR_EXCHANGER == 'peltier' else sample_t

            if filtered_temp is None:
                filtered_temp = measured_t
            else:
                filtered_temp += FILTER_ALPHA * (measured_t - filtered_temp)

            error = target - filtered_temp

            if monotonic_now - last_control_time >= CONTROL_INTERVAL_S:
                dt = monotonic_now - last_control_time
                last_control_time = monotonic_now

                # PI control with simple anti-windup.
                candidate_integral = integral + error * dt
                candidate_output = KP * error + KI * candidate_integral

                if not (
                    (candidate_output > MAX_DUTY_CYCLE and error > 0) or
                    (candidate_output < -MAX_DUTY_CYCLE and error < 0)
                ):
                    integral = candidate_integral

                control_output = KP * error + KI * integral
                control_output = max(
                    -MAX_DUTY_CYCLE,
                    min(MAX_DUTY_CYCLE, control_output)
                )

                if abs(control_output) < OUTPUT_DEADBAND:
                    control_output = 0.0

                # Require a clear opposite demand before reversing direction.
                if last_direction == 'heating' and -REVERSAL_THRESHOLD < control_output < 0:
                    control_output = 0.0
                elif last_direction == 'cooling' and 0 < control_output < REVERSAL_THRESHOLD:
                    control_output = 0.0

                if control_output > 0:
                    peltier1.heat(control_output)
                    peltier2.heat(control_output)
                    mode = 'heating'
                    last_direction = 'heating'
                elif control_output < 0:
                    duty = abs(control_output)
                    peltier1.cool(duty)
                    peltier2.cool(duty)
                    mode = 'cooling'
                    last_direction = 'cooling'
                else:
                    peltier1.stop()
                    peltier2.stop()
                    mode = 'off'

            print(
                f'Target: {target:5.2f} C | Sample: {sample_t:5.2f} C | '
                f'Filtered: {filtered_temp:5.2f} C | {mode:7s} | '
                f'output: {control_output:6.1f}%'
            )

            writer.writerow([
                now.isoformat(timespec='milliseconds'),
                round(elapsed_s, 3),
                round(target, 3),
                round(sample_t, 3),
                round(filtered_temp, 3),
                round(peltier_t, 3),
                round(peltier2_t, 3),
                round(error, 3),
                mode,
                round(control_output, 2),
            ])
            csv_file.flush()

            DT.append(now)
            target_T.append(target)
            sample_T.append(sample_t)
            filtered_T.append(filtered_temp)
            peltier_T.append(peltier_t)
            peltier2_T.append(peltier2_t)
            control_history.append(control_output)

            l1.set_data(DT, sample_T)
            l2.set_data(DT, filtered_T)
            l3.set_data(DT, target_T)
            l4.set_data(DT, control_history)

            if len(DT) > 1:
                ax1.set_xlim(DT[0], DT[-1])

            temps = list(sample_T) + list(filtered_T) + list(target_T)
            if temps:
                ax1.set_ylim(min(temps) - 2, max(temps) + 2)

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
