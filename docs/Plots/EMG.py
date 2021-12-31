#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import re
from fileReader import file, computeMotionTimeRange

import scipy as sp
import scipy.fftpack
from scipy import signal


# In[2]:


plt.style.use('ggplot')
plt.rcParams.update({
    "text.usetex": False,
    "font.sans-serif": ["CMU Sans Serif"],
    "font.size" : 8})

## for Palatino and other serif fonts use:
plt.rcParams.update({
    "text.usetex": False,
    "font.family": "CMU Serif",
    "font.serif": ["Palatino"],
})


# In[3]:


def filterEMG(emg, low_pass=10, sfreq=1000, low_band=20, high_band =250):
    """
    time: Time data
    emg: EMG data
    high_band: high-pass cut off frequency
    low_band: low-pass cut off frequency
    low_pass: 
    sfreq: sampling frequency
    
    https://scientificallysound.org/2016/08/22/python-analysing-emg-signals-part-4/
    """
    
    # normalise cut-off frequencies to sampling frequency
    emg_correctmean = emg - np.mean(emg)
    
    # normalise cut-off frequencies to sampling frequency
    nyq = sfreq/2
    low_band = low_band/nyq
    high_band = high_band/nyq
    
    # create bandpass filter for EMG
    b1, a1 = sp.signal.butter(4, [low_band, high_band], btype='bandpass')
    
    # process EMG signal: filter EMG
    emg_filtered = sp.signal.filtfilt(b1, a1, emg_correctmean)    
    
    # process EMG signal: rectify
    emg_rectified = abs(emg_filtered)
    
    # create lowpass filter and apply to rectified signal to get EMG envelope
    low_pass = low_pass/nyq
    b2, a2 = sp.signal.butter(4, low_pass, btype='lowpass')
    emg_envelope = sp.signal.filtfilt(b2, a2, emg_rectified)
    
    return emg_envelope


# In[4]:


# Emg filtering values
low_pass = 3
s_freq = 1000
high_band = 10
low_band = 350

nPoints = 100
startThreshold = 20
endThreshold = 30


# In[5]:


emgFileHeaderLines = 5
emgFileDelimiter = ","
emgFilePath = "../IK/trials/"

motionFileHeaderLines = 10
motionFilePath = "../IK/ikResults/"

muscleChannelsDict = {"GMAX":"C3", "HAMS":"C4", "VAS":"D2", "RF":"D3", 
              "TA":"A1", "GAS":"A3", "SOL":"A4"}

musclesMVCFiles = {"GMAX":"subjectB_mvc_afterCalib2.csv", 
              "HAMS":"subjectB_mvc_afterCalib2.csv",
              "VAS":"subjectB_mvc_afterCalib4.csv", 
              "RF":"subjectB_mvc_afterCalib4.csv", 
              "TA":"subjectB_mvc_afterCalib3.csv", 
              "GAS":"subjectB_mvc_afterCalib3.csv",
              "SOL":"subjectB_mvc_afterCalib3.csv"}

trialFileNames = []
for i in range(1,7):
    emgFileName = emgFilePath+"Trimmed_subjectB_natural"+str(i)+"_EMG.csv"
    motionFileName = motionFilePath+"Trimmed_subjectB_natural"+str(i)+".mot"
    trialFileNames.append((emgFileName, motionFileName))
    
    emgFileName = emgFilePath+"Trimmed_subjectB_folded"+str(i)+"_EMG.csv"
    motionFileName = motionFilePath+"Trimmed_subjectB_folded"+str(i)+".mot"
    trialFileNames.append((emgFileName, motionFileName))
    
    emgFileName = emgFilePath+"Trimmed_subjectB_sides"+str(i)+"_EMG.csv"
    motionFileName = motionFilePath+"Trimmed_subjectB_sides"+str(i)+".mot"
    trialFileNames.append((emgFileName, motionFileName))


# In[6]:


mvcValues = {}

for muscle, mvcFileName in musclesMVCFiles.items():
    yDataNames = (muscleChannelsDict[muscle])
    mvcFileName = emgFilePath+mvcFileName
    emgSignal = file(yDataNames, mvcFileName, emgFileHeaderLines,delimiter=emgFileDelimiter)
    emg = emgSignal.getColumn(muscleChannelsDict[muscle])
    emg_processed = filterEMG(emg, low_pass, s_freq, high_band, low_band)
    mvcValues.update({muscle:emg_processed.max()})


# In[7]:


emgData = {}
for muscle in muscleChannelsDict.keys():
    emgData.update({muscle:np.zeros((nPoints, len(trialFileNames)))})

motionXAxis = np.linspace(0,100, nPoints)
muscleChannels = [channel for channel in muscleChannelsDict.values()]
muscleChannels.append("Time")

for i in range(len(trialFileNames)):
    emgFileName = trialFileNames[i][0]
    motionFileName = trialFileNames[i][1]
    startTime, endTime = computeMotionTimeRange(("time", "hip_flexion_r"),
                                                (startThreshold,endThreshold), 
                                                motionFileName, motionFileHeaderLines)
    
    emgFile = file(muscleChannels, emgFileName, emgFileHeaderLines, emgFileDelimiter)
    time = emgFile.getColumn("Time")
    startInd = np.argmax(time>=startTime-1e-5)
    endInd = np.argmax(time>=endTime-1e-5)
    emgXAxis = np.linspace(0,100, endInd-startInd)
    
    for muscle, muscleMat in emgData.items():
        emgSignal = emgFile.getColumn(muscleChannelsDict[muscle])
        emg_processed = filterEMG(emgSignal, low_pass, s_freq, high_band, low_band)
        emg_normalized = np.divide(emg_processed, mvcValues[muscle])
        emgData[muscle][:,i] = np.interp(motionXAxis, emgXAxis, emg_normalized[startInd:endInd])


# In[8]:


simMuscleDict = {"GMAX" :"glut_max", 
                   "HAMS":"hamstrings",
                   "VAS":"vasti", 
                   "RF":"recfem", 
                   "TA":"tibant",
                   "GAS":"gastroc",
                   "SOL":"soleus"}

regrexHeader = r"(\w+)/activation$"
simFileName = "../ProcessedData/00pct/00pct.sto"
simFileHeaderLines = 8

yDataNames = [muscle for muscle in simMuscleDict.values()]
simActivationData = file(yDataNames, simFileName, simFileHeaderLines, regrexHeader=regrexHeader)

startTime ,endTime = computeMotionTimeRange(("time", "hip_flexion"), 
                                            (np.deg2rad(startThreshold), np.deg2rad(endThreshold)), 
                                            simFileName, simFileHeaderLines)

timeData = simActivationData.getColumn("time")
startInd = np.argmax(timeData>=startTime-1e-3)
endInd = np.argmax(timeData>=endTime-1e-3)

timeEquallySpaced = np.linspace(timeData[startInd], timeData[endInd], nPoints)

simData = {}
for key, value in simMuscleDict.items():
    simData[key] = np.interp(timeEquallySpaced, timeData[startInd:endInd], 
                             simActivationData.getColumn(value)[startInd:endInd])

simXAxis = np.linspace(0,100, nPoints)


# In[12]:


nRows = 2
nCols = 4
#fig, ax = plt.subplots(nRows,nCols, sharex='col', sharey='row',figsize=(12, 12), dpi=300)
fig = plt.figure(figsize=(12,7),dpi=300)
gs = gridspec.GridSpec(2, 8)

for i, muscle in enumerate(emgData.keys()):    
    muscleDataMat = emgData[muscle]
    mean = np.mean(muscleDataMat, axis=1)
    std = np.std(muscleDataMat, axis=1)
    
    if(i<4):
        ax = plt.subplot(gs[0, 2 * i:2 * i + 2])
    else:
        ax = plt.subplot(gs[1, 2 * i - 7:2 * i + 2 - 7])
    
    #for j in range(muscleDataMat.shape[1]):
        #ax.plot(motionXAxis, muscleDataMat[:,j], alpha=0.5)
    
    ax.plot(motionXAxis, mean, label= "Experiment Mean", color='blue', linestyle='dashed', linewidth=1.5)
    ax.fill_between(motionXAxis, mean-2*std, mean+2*std, color = 'blue', label="Experiemnt 2SD", alpha=0.1)
    ax.plot(simXAxis, simData[muscle], color="red",label="Prediction", linewidth=1.0)

    if(i==0 or i==4):
        ax.set_ylabel(r"$a$")
        ax.set_xlabel(r"$\%STS$")


    ax.set_ylim([-0.05,1.05])
    ax.set_xlim([0, 100])
    ax.set_title(muscle)
    ax.legend()
    

#ax[2,0].set_xlabel("%STS")
#ax[2,1].set_xlabel("%STS")
#ax[0,2].set_xlabel("%STS")

#ax[0,0].set_ylabel("Activation")
#ax[1,0].set_ylabel("Activation")
#ax[2,0].set_ylabel("Activation")


fig.set_tight_layout(True)
plt.savefig("figures/ActivationComparison.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[10]:


#N = emg.shape[0]

## sample spacing
#T = 1.0 / 1000.0
#x = np.linspace(0.0, N*T, N)
#yf = sp.fftpack.fft(emg)

#xf = np.linspace(0.0, vin, int(N/2))

#fig, ax = plt.subplots()
#ax.plot(xf, 2.0/N * np.abs(yf[:N//2]))
#ax.set_xlim(1,100)
#ax.set_ylim(0,250)

#plt.show()

