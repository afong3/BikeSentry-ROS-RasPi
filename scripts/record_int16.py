# pyaudio won't let me record in float32... sooo a new package it is!
import sounddevice as sd
import scipy.io.wavfile as wav

# setup
duration = 5 # seconds
fs = 44100
sd.default.samplerate = fs
sd.default.channels = 1
sd.default.dtype = "int16"

rec = sd.rec(int(duration * fs))
sd.wait()

print(rec.dtype)

# save .wav
filename = "recordings/testing_float.wav"
wav.write(filename, fs, rec)

# open .wav
sr, audio = wav.read(filename)
print(audio)
