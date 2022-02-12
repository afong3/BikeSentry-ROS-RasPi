# Purpose: Take data from the microphone, preprocess the audio data for classification model input
# Python3
# Refer to Confluence: StopBikeTheft/Designs/ROS Designs/AudioManager
import os
import scipy.io.wavfile as wav
from pickle import load
import tensorflow as tf
import numpy as np
import time

class AudioManager:
	def __init__(self, model_path = "../tf_models/angle-grinder-detector-2s.tflite"):
		"""
		Initialize class with sample_rate of audio recordings.

		"""
		# Get input and output tensors.
		self.interpreter = tf.lite.Interpreter(model_path = model_path)
		self.interpreter.allocate_tensors()
		self.input_details = self.interpreter.get_input_details()
		self.output_details = self.interpreter.get_output_details()

	def _SET_TENSOR(self, data):
		"""
		Wrapper on tf.lite.Interpreter.set_tensor()
		"""
		self.interpreter.set_tensor(self.input_details[0]['index'], data)

	def classify_audio(self, data):
		"""
		Returns float between 0 - 1, up to you to determine class from this. Can use softmax 
		"""

		self._SET_TENSOR(data)
		output_data = self.interpreter.get_tensor(self.output_details[0]['index'])

		return output_data[0][0]

	def read_wav(self, file):
		'''
		Reads file.wav which will be soon be preprocessed
		'''
		sample_rate, y = wav.read(file) 
		if len(y.shape) > 1: # ignoring the second channel if the .wav file contains it 
			y = y[:,0]
		
		return sample_rate, y

	def raw_audio_to_freq(self, chunk, samplerate): 
		"""
		Converts raw audio in time domain to frequency domain using FFT
		"""
		start = time.time()

		n = len(chunk) # length of the signal
		k = np.arange(n)
		T = n/samplerate
		
		frq = k/T # two sides frequency range
		
		zz=int(n/2)
		freq = frq[range(zz)]  # one side frequency range
		Y0 = np.fft.fft(chunk)/n  # fft computing and normalization
		Y = Y0[range(zz)]
		
		end = time.time()
		print(f"Runtime of raw_audio_to_freq is: {end - start}")
		return abs(Y), freq

	def bin_Y(self, chunk, bins):
		start = time.time()
		
		# sums values within bin edges 
		# TODO: summing may not be the best summary of the data. Max Y or Min Y or a combo of these may catch more variability
		
		chunk_binned = np.array_split(chunk, bins)

		# replace at each index the sum of all the values at the index
		for idx, bin in enumerate(chunk_binned):
			bin_summed = np.sum(bin)
			chunk_binned[idx] = bin_summed
			
		end = time.time()
		print(f"Runtime of bin_chunks_y is: {end - start}")
		return chunk_binned

	def process_recording(self, rec, sample_rate):
		"""
		raw_mic_data must be entered as a .wav file without lossy compression
		Returns: frequency data as a list 
		"""
		scaler = load(open('C:/code/BikeSentry/device/audio_recognition/fft_examples/audio_scaler_speedy.pkl', 'rb'))

		# right now preprocessing looks like this, raw -> FFT -> binning -> scaling 
		# TODO: change this to MFCC's or at least test it out 
		Y = self.raw_audio_to_freq(rec, sample_rate)
		binned = self.bin_Y(Y)
		scaled = scaler.transform(binned)

		return scaled

