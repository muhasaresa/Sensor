import matplotlib.pyplot as plt
import numpy as np
import pickle
import struct
from scipy import signal, stats
from scipy.signal import butter, filtfilt
import peakutils
from tkinter import*
from pathlib import Path
import re
import pandas as pd
import os

order = 3
fs = 100.0       # sample rate, Hz
cutoff = 30  # desired cutoff frequency of the filter, Hz
        # Number of points to display
sampling_rate = 20 # ms


def askopen(): # asks for file path
    filenamelist = []
    new_files = 1 #select 1 for new files to be loaded, 0 to replot 
    if new_files:
        
        #while True: # uncomment if files from diff folders aare needed
            filenames = filedialog.askopenfilenames()
            if filenames:
                filenames= root.tk.splitlist(filenames)
                filenamelist = np.append(filenamelist, filenames)
            #else:
                #break
    return filenamelist





def highpass_filter(timestamps, data, cutoff, order):
    time_diffs = np.diff(timestamps)

# Use the mode of time differences as the true sample interval
    rounded_diffs = np.round(time_diffs, decimals=4)  # round to avoid float precision issues
    sampling_interval = stats.mode(rounded_diffs, keepdims=False).mode
    fs = 1.0 / sampling_interval
    print(f"Estimated sampling frequency using mode: {fs:.2f} Hz")
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    filtered = filtfilt(b, a, data)
    return filtered
    
def bandpass_filter(timestamps, data, lowcut, highcut, order):
    time_diffs = np.diff(timestamps)

# Use the mode of time differences as the true sample interval
    rounded_diffs = np.round(time_diffs, decimals=4)  # round to avoid float precision issues
    sampling_interval = stats.mode(rounded_diffs, keepdims=False).mode
    fs = 1.0 / sampling_interval
    print(f"Estimated sampling frequency using mode: {fs:.2f} Hz")
    nyq = 0.5 * fs
    #normal_cutoff = cutoff / nyq
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band')
    filtered = filtfilt(b, a, data)
    return filtered

# This function is called periodically from FuncAnimation
def plot_data(pickle_file):
    text = []
    ppg = []
    cdc = []
    ecg = []
    X = []
    Y = []
    Z = []
    xs1 = []
    xs2 = []
    x = 0
     # Load data from the pickle file
    with open(pickle_file, 'rb') as f:
        sensors1Infile = pickle.load(f) # First line of the pickle file shows the activated sensors
        sensors2Infile = pickle.load(f)
        sensorsInfile = sensors1Infile + sensors2Infile
        #sensorsInfile =    ['Coer Sensor']#,'X','Y','Z'] #to plot only selected sensors
        print(sensors1Infile)
        while True:
                try:
                    text = pickle.load(f).decode('utf-8')
                    
                    for i in range(len(text)):  
                        try:
               
                                
                            if (text[i]) == 'P' and 'P' in sensorsInfile:  
                                PPG = float(text[i+1:i+5])/1024
                                ppg.append(PPG)
                            if ((text[i])) == 'C' and 'Coer Sensor' in sensorsInfile:
                                CDC = float((text[i+1:i+6]))/1000            # CDC bytes

                                cdc.append(CDC)
                            if (text[i]) == 'E' and 'E' in sensorsInfile:  
                                ECG = float(text[i+1:i+5])/1024
                                #print(ECG)
                                ecg.append(ECG)
                            if (text[i]) == 'X' and 'X' in sensorsInfile:  
                                X.append(float(text[i+1:i+5]))
                            if (text[i]) == 'Y' and 'Y' in sensorsInfile:  
                                Y.append(float(text[i+1:i+5]))                
                            if (text[i]) == 'Z' and 'Z' in sensorsInfile:  
                                Z.append(float(text[i+1:i+5]))       
                            if (text[i]) == 'T':
                                xs1.append(float(text[i+1:i+9])/1000)
                            if (text[i]) == 'U':
                                xs2.append(float(text[i+1:i+9])/1000)                                
                        except ValueError:
                            print('VE') 
                        
                            
                except EOFError:
                    break
    #self.sensor = self.butter_lowpass_filter(self.cdc, cutoff, fs, order)
    
    numplots = sum(1 for lst in [ppg, cdc,ecg, X, Y, Z] if len(lst) > 0)
    #no_elements = max([ppg,cdc,ecg,X,Y,Z], key = len)

    #x_len = len(no_elements)
    #max_time = x_len*sampling_rate/1000 # to seconds
    fig, ax = plt.subplots(numplots, sharex=True) # share x axis
    if numplots == 1:
        ax = [ax]
    #xs = np.linspace(0, max_time, x_len)
    i = 0
    if len(ppg) > 0:
        ax[i].set_ylim(ymin = min(ppg), ymax = max(ppg))
        if 'P' in sensors1Infile: 
            xs = xs1
        else:
            xs = xs2
        sensor = bandpass_filter(xs, ppg, lowcut = 0.5, highcut = 40, order = 2)
        y = sensor
        ax[i].plot(xs,sensor, color = "blue")
        ax[i].set_title('PPG')
        ax[i].set_ylim(ymin = min(y), ymax = max(y))
        #ax[i].grid(which='minor',axis='x', color='gray', linestyle='--', linewidth=0.5)
        #ax[i].minorticks_on()  # Enable minor ticks
        i = i +1
    if len(cdc) > 0:
        ax[i].set_ylim(ymin = min(cdc), ymax = max(cdc))
        if 'Coer Sensor' in sensors1Infile: 
            xs = xs1
        else:
            xs = xs2
        sensor = bandpass_filter(xs, cdc, lowcut = 0.5, highcut = 40, order = 2)
        y = cdc
        ax[i].plot(xs,y, color = "red")
        ax[i].set_title('COER Sensor')
        ax[i].set_ylim(ymin = min(y), ymax = max(y))
        #ax[i].grid(which='minor',axis='x', color='gray', linestyle='--', linewidth=0.5)
        #ax[i].minorticks_on()  # Enable minor ticks
        i = i +1
    if len(ecg) > 0:

        ax[i].set_ylim(ymin = min(ecg), ymax = max(ecg))
        if 'E' in sensors1Infile: 
            xs = xs1
        else:
            xs = xs2
        sensor = bandpass_filter(xs, ecg, lowcut = 0.5, highcut = 40, order = 2)
        y = sensor
        ax[i].plot(xs,sensor, color = "black")
        ax[i].set_title('ECG')
        ax[i].set_ylim(ymin = min(y), ymax = max(y))
        i = i +1
    if len(X) > 0:
        ax[i].set_ylim(ymin = min(X), ymax = max(X))
        if 'X' in sensors1Infile: 
            xs = xs1
        else:
            xs = xs2
        ax[i].plot(xs,X, color = "orange")
        ax[i].set_title('X')
        i = i +1        
    if len(Y) > 0:
        ax[i].set_ylim(ymin = min(Y), ymax = max(Y))
        if 'Y' in sensors1Infile: 
            xs = xs1
        else:
            xs = xs2
        ax[i].plot(xs,Y, color = "green")
        ax[i].set_title('Y')
        i = i +1
    if len(Z) > 0:
        ax[i].set_ylim(ymin = min(Z), ymax = max(Z))
        if 'Z' in sensors1Infile: 
            xs = xs1
        else:
            xs = xs2
        ax[i].plot(xs,Z, color = "pink")
        ax[i].set_title('Z')
        i = i +1        
    ax[i-1].set_xlabel('Seconds')
    #plt.grid(which='minor', axis='x', color='gray', linestyle='--', linewidth=0.5)
    #plt.minorticks_on() 
    
    #baseline = peakutils.baseline(self.sensor) 
    #self.sensor = self.sensor - baseline   
    # For exporting data as CSV               
    #plt.figure(2)
    #x_filtered = [value for value in xs if 205.411 <= value <= 213.569]
    #y_filtered = cdc[xs.index(x_filtered[0]):xs.index(x_filtered[-1]) + 1]
    #y2_filtered = ecg[xs.index(x_filtered[0]):xs.index(x_filtered[-1]) + 1]
  
   
    #data = pd.DataFrame({"X": xs, "Y": cdc})
    #directory_path = os.path.dirname(filename)
    # Export to a CSV file
    #csv_filename = os.path.splitext(os.path.basename(filename))[0]  #"test1.csv"
    #csv_file_path = os.path.join(directory_path, csv_filename  + ".csv")
    #data.to_csv(csv_file_path, index=False)
   
    
# Create Tk root
root = Tk()
# Hide the main window
root.withdraw()
root.call('wm', 'attributes', '.', '-topmost', True)
 

from tkinter import filedialog

#filenames = ospath()
filenames = askopen()

for filename in filenames:
    match = re.search(r'SR(\d+)ms', filename)
    if match:
        sampling_rate = match.group(1)  # Extract the number part
        sampling_rate = int(sampling_rate)  # Convert to integer
    plot_data(filename)
plt.show()

