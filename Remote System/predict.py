import math
import sys

import joblib
import pandas as pd
import serial
import numpy as np
from scipy.signal import find_peaks

model = joblib.load('model.pkl')

port = "/dev/ttyACM0"
baud = 115200
ser = serial.Serial(port, baudrate=baud)
last_ten_seconds_red = []
last_ten_seconds_gsr = []
last_time = 0
current_time = 0
window_slide_time = 500
try:
    while True:
        line = ser.readline()
        line = line.decode().strip()
        line = line[1:]
        line = line.split(":")
        # line[0] = "IR" - data
        # line[1] = "1235423" - value
        # line[2] = "1502" - time
        if len(line) != 3:
            continue
        current_time = int(line[2])
        match line[0]:
            case "RED":
                last_ten_seconds_red.append((int(line[1]), int(line[2])))
            case "IR":
                pass
            case "GSR":
                last_ten_seconds_gsr.append(int(line[1]))
        if (current_time - last_time) > window_slide_time:
            if last_time == 0:
                last_time = current_time
                continue
            if len(last_ten_seconds_red) < 2:
                last_time = current_time
                continue
            last_time = current_time
            start_time = last_time - 10_000
            # last_five_seconds_red = [x for x in last_five_seconds_red if x[1] >= start_time]
            while last_ten_seconds_red and last_ten_seconds_red[0][1] < start_time:
                last_ten_seconds_red.pop(0)
                last_ten_seconds_gsr.pop(0)
            times = [item[1] for item in last_ten_seconds_red]
            times = np.array(times)
            red_arr = [item[0] for item in last_ten_seconds_red]
            red_arr = np.array(red_arr)
            red_arr = red_arr - np.mean(red_arr)
            red_arr = np.convolve(red_arr, np.ones(8) / 8, mode='same')
            sample_rate = len(red_arr) / ((last_ten_seconds_red[-1][1] - last_ten_seconds_red[0][1]) / 1000)
            min_distance = int(0.25 * sample_rate)  # minimum distance between peaks has to be at least 0.25 * sample rate
            peaks, _ = find_peaks(red_arr, distance=min_distance, prominence=np.std(red_arr))

            # print(len(peaks))
            if len(peaks) < 2:
                continue

            peak_times = np.array([last_ten_seconds_red[i][1] for i in peaks])
            rr_intervals = np.diff(peak_times) / 1000.0  # seconds
            if len(rr_intervals) <= 1:
                continue
            avg_rr = np.mean(rr_intervals)
            avg_bpm = 60.0 / avg_rr
            sdnn = np.std(rr_intervals)
            rmssd = math.sqrt(np.mean( np.pow(np.diff(rr_intervals), 2) ))
            gsr_sample = np.array(last_ten_seconds_gsr)
            mean_gsr = np.mean(gsr_sample)
            std_gsr = np.std(gsr_sample, ddof=0)
            dx = np.diff(gsr_sample)
            dt = np.diff(times)
            # print(len(gsr_sample), len(times), len(last_ten_seconds_red))
            mean_absolute_derivative = np.mean(np.abs(dx / dt))
            data_predict = np.array([[avg_bpm, rmssd, sdnn, mean_gsr, mean_absolute_derivative, std_gsr]])
            X_df = pd.DataFrame(data_predict, columns=model.feature_names_in_)

            prediction = model.predict(X_df)
            sys.stdout.write("\033[F\033[F\033[F\033[F\033[F\033[F\033[F\033[F")
            print("Stress  " if prediction == 1 else "\rCalm    ")
            print(f"Predicted: {prediction}")
            print(f"Average BPM: {avg_bpm}")
            print(f"RMSSD: {rmssd}")
            print(f"SDNN: {sdnn}")
            print(f"Mean GSR: {mean_gsr}")
            print(f"Mean Absolute Derivative: {mean_absolute_derivative}")
            print(f"Standard Deviation: {std_gsr}")

except KeyboardInterrupt:
    ser.close()