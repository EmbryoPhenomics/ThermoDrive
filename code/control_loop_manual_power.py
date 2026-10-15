# Manual power control with temperature logging and live plots.
# Edit POWER_PERCENT and LOG_FOLDER below, then run on the Raspberry Pi.
# Press Ctrl+C to stop both outputs and finish logging.

import csv
import math
import time
from datetime import datetime
from pathlib import Path
from collections import deque

import matplotlib.pyplot as plt
import RPi.GPIO as GPIO

import sm_tc


# -----------------------------------------------------------------------------
# SETTINGS
# -----------------------------------------------------------------------------

# Signed PWM duty cycle applied equally to both Peltiers (not watts).
# +100 = full heating, -100 = full cooling, 0 = off (still logs).
# Choose a value suitable for your hardware; temperature is not regulated.
POWER_PERCENT = 0.0
LOOP_INTERVAL_S = 0.25

# Change this to your desired output folder. Created if it does not exist.
LOG_FOLDER = Path.home() / 'temperature_logs'
CSV_FILENAME = datetime.now().strftime('manual_power_%Y%m%d_%H%M%S_%f.csv')

# Peltier 1
EN = 13
FW = 26
BK = 5

# Peltier 2
EN2 = 18
FW2 = 27
BK2 = 23


# -----------------------------------------------------------------------------

if not math.isfinite(POWER_PERCENT) or not -100 <= POWER_PERCENT <= 100:
    raise ValueError('POWER_PERCENT must be between -100 and 100')

if not math.isfinite(LOOP_INTERVAL_S) or LOOP_INTERVAL_S <= 0:
    raise ValueError('LOOP_INTERVAL_S must be positive')

log_folder = Path(LOG_FOLDER).expanduser()
if not log_folder.is_absolute():
    raise ValueError('LOG_FOLDER must be an absolute path')
log_folder.mkdir(parents=True, exist_ok=True)
csv_path = log_folder / CSV_FILENAME

class Peltier:
    def __init__(self, enable_pin, forward_pin, backward_pin):
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin

        GPIO.setup(enable_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(forward_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(backward_pin, GPIO.OUT, initial=GPIO.LOW)

        self.pwm = GPIO.PWM(enable_pin, 1000)
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

peltiers = []
GPIO.setmode(GPIO.BCM)
plt.ion()
try:
    sensor = sm_tc.SMtc(0)

    peltier1 = Peltier(EN, FW, BK)
    peltiers.append(peltier1)
    peltier2 = Peltier(EN2, FW2, BK2)
    peltiers.append(peltier2)

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    fig.tight_layout(pad=2.0)

    DT = deque(maxlen=1000)
    sample_T = deque(maxlen=1000)
    peltier1_T = deque(maxlen=1000)
    peltier2_T = deque(maxlen=1000)
    control_output = deque(maxlen=1000)

    l1, = ax1.plot([], [], label='Sample')
    l2, = ax1.plot([], [], label='Peltier 1')
    l3, = ax1.plot([], [], label='Peltier 2')
    l5, = ax2.plot([], [], label='Control output')

    ax1.set_ylabel('Temperature (deg C)')
    ax2.set_ylabel('PWM (%)')
    ax2.set_xlabel('Time')

    ax2.set_ylim(-105, 105)
    ax2.axhline(0, linewidth=0.8)

    ax1.legend(loc='upper left')
    ax2.legend(loc='upper left')

    start_time = time.monotonic()


    with open(csv_path, 'x', newline='') as csv_file:
        writer = csv.writer(csv_file)

        writer.writerow([
            'timestamp',
            'elapsed_s',
            'sample_temp_C',
            'peltier1_temp_C',
            'peltier2_temp_C',
            'mode',
            'duty_cycle_pct',
            'signed_control_pct',
        ])

        csv_file.flush()
        print(f'Logging to: {csv_path}')

        while True:
            sample_t = sensor.get_temp(1)
            peltier1_t = sensor.get_temp(2)
            peltier2_t = sensor.get_temp(3)

            now = datetime.now()
            elapsed_s = time.monotonic() - start_time
            # Apply the fixed user setting; sensor values do not control power.
            duty_cycle = abs(POWER_PERCENT)
            signed_control = POWER_PERCENT
            if POWER_PERCENT > 0:
                peltier1.heat(duty_cycle)
                peltier2.heat(duty_cycle)
                mode = 'heating'
            elif POWER_PERCENT < 0:
                peltier1.cool(duty_cycle)
                peltier2.cool(duty_cycle)
                mode = 'cooling'
            else:
                peltier1.stop()
                peltier2.stop()
                mode = 'off'

            print(
                f'Sample: {sample_t:5.2f} C | '
                f'{mode:7s} | duty: {duty_cycle:5.1f}%'
            )

            writer.writerow([
                now.isoformat(timespec='milliseconds'),
                round(elapsed_s, 3),
                round(sample_t, 3),
                round(peltier1_t, 3),
                round(peltier2_t, 3),
                mode,
                duty_cycle,
                signed_control,
            ])

            csv_file.flush()

            DT.append(now)
            sample_T.append(sample_t)
            peltier1_T.append(peltier1_t)
            peltier2_T.append(peltier2_t)
            control_output.append(signed_control)

            l1.set_data(DT, sample_T)
            l2.set_data(DT, peltier1_T)
            l3.set_data(DT, peltier2_T)
            l5.set_data(DT, control_output)

            if len(DT) > 1:
                ax1.set_xlim(DT[0], DT[-1])

            all_temperatures = (
                list(sample_T)
                + list(peltier1_T)
                + list(peltier2_T)
            )

            if all_temperatures:
                ax1.set_ylim(
                    min(all_temperatures) - 2,
                    max(all_temperatures) + 2
                )

            fig.canvas.draw_idle()
            plt.pause(LOOP_INTERVAL_S)

except KeyboardInterrupt:
    print('\nStopping manual power control...')
finally:
    try:
        for peltier in peltiers:
            peltier.stop()
            peltier.pwm.stop()
    finally:
        GPIO.cleanup()
        plt.ioff()
    print(f'Log file: {csv_path}')
    plt.show()
