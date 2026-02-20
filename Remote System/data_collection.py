import serial
import numpy as np
from scipy.signal import find_peaks

import matplotlib.pyplot as plt

testing = "calm"

port = "/dev/ttyACM0"
baud = 115200
ser = serial.Serial(port, baudrate=baud)
red = []
ir = []
last_five_seconds_red = []
last_time = 0
current_time = 0
window_slide_time = 500
try:
    while True:
        line = ser.readline()
        line = line.decode().strip()
        line = line[1:]
        line = line.split(":")
        if len(line) != 3:
            continue
        current_time = int(line[2])
        match line[0]:
            case "RED":
                red.append((int(line[1]), int(line[2])))
                last_five_seconds_red.append((int(line[1]), int(line[2])))
            case "IR":
                ir.append((int(line[1]), int(line[2])))
        if (current_time - last_time) > window_slide_time:
            if last_time == 0:
                last_time = current_time
                continue
            if len(last_five_seconds_red) < 2:
                last_time = current_time
                continue
            last_time = current_time
            start_time = last_time - 10_000
            # last_five_seconds_red = [x for x in last_five_seconds_red if x[1] >= start_time]
            while last_five_seconds_red and last_five_seconds_red[0][1] < start_time:
                last_five_seconds_red.pop(0)
            red_arr = [item[0] for item in last_five_seconds_red]
            red_arr = np.array(red_arr)
            red_arr = red_arr - np.mean(red_arr)
            red_arr = np.convolve(red_arr, np.ones(8)/8, mode='same')
            sample_rate = len(red_arr) / ( (last_five_seconds_red[-1][1] - last_five_seconds_red[0][1]) / 1000 )
            min_distance = int(0.25 * sample_rate)
            peaks, _ = find_peaks(red_arr, distance=min_distance, prominence = np.std(red_arr))
            print(len(peaks))
            if len(peaks) > 1:
                peak_times = np.array([last_five_seconds_red[i][1] for i in peaks])
                rr_intervals = np.diff(peak_times) / 1000.0  # seconds

                if len(rr_intervals) > 0:
                    avg_rr = np.mean(rr_intervals)
                    bpm = 60.0 / avg_rr
                    print(bpm)
except KeyboardInterrupt:
    ser.close()
    f = open(f"red_data_{testing}.txt", "a")
    sliding_window_time = 10_000
    # take a 5 second sliding window over red, moving over a second each time
    time = red[0][1] // sliding_window_time * sliding_window_time
    time_step = 500
    last_sliding_window_time = red[-1][1] // sliding_window_time * sliding_window_time
    while time <= last_sliding_window_time:
        samples_to_take_red = []
        for sample in red:
            if sample[1] >= time and sample[1] < time + sliding_window_time:
                samples_to_take_red.append(sample)
        sample_rate = len(samples_to_take_red) / ((samples_to_take_red[-1][1] - samples_to_take_red[0][1]) / 1000)
        red_values_sample = [x[0] for x in samples_to_take_red]
        red_values_sample = np.array(red_values_sample)
        red_values_sample = red_values_sample - np.mean(red_values_sample)
        red_values_sample = np.convolve(red_values_sample, np.ones(8) / 8, mode='same')
        min_distance = int(0.25 * sample_rate)
        peaks, _ = find_peaks(red_values_sample, distance=min_distance, prominence = np.std(red_values_sample))
        peak_times = np.array([samples_to_take_red[i][1] for i in peaks])
        rr_intervals = np.diff(peak_times) / 1000.0  # seconds
        bpm = 0
        if len(rr_intervals) > 0:
            avg_rr = np.mean(rr_intervals)
            bpm = 60.0 / avg_rr
        if bpm < 45:
            time += time_step
            continue
        # Store this sample in a txt file. First line is the red_values_sample with the mean taken away.
        # The 2nd line is the times
        # the 3rd line is the index of the peaks

        line = ""
        for sample in red_values_sample:
            line += str(sample) + ","
        line = line[:-1]
        f.write(line)
        f.write("\n")
        line = ""
        for sample in samples_to_take_red:
            line += str(sample[1]) + ","
        line = line[:-1]
        f.write(line)
        f.write("\n")
        line = ""
        for peak in peaks:
            line += str(peak) + ","
        line = line[:-1]
        f.write(line)
        f.write("\n")


        if len(peaks) > 1:
            print("PEAKS", len(peaks))
            print("BPM", bpm)

        time += time_step

    plt.figure()
    sample_to_graph = np.array(red_values_sample)
    plt.plot(sample_to_graph)
    plt.scatter(peaks, red_values_sample[peaks], color='red')
    plt.show()
