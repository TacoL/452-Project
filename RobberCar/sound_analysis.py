from scipy.io import wavfile
from scipy import signal
import sounddevice
import matplotlib.pyplot as plt
import numpy as np

fs, x = wavfile.read('RobberCar/IMG_0393.wav') # Load a sound from a WAV file.

left_channel = x[:, 0]  # Extract the left channel (first column)
right_channel = x[:, 1]  # Extract the right channel (second column)

sounddevice.play(left_channel, samplerate=fs)
f, t, S = signal.spectrogram(left_channel, fs=fs)
plt.pcolormesh(t, f, np.log(S + 1), shading='gouraud', cmap='jet')
plt.show()