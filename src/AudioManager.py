# Purpose: Take data from the microphone, preprocess the audio data for classification model input
# Python3
# Refer to Confluence: StopBikeTheft/Designs/ROS Designs/AudioManager
import os
import scipy.io.wavfile as wav
from pickle import load as p_load
import numpy as np
import time
from joblib import load as j_load
import librosa

class AudioManager:
	def __init__(self, model, scaler):
		"""
		Initialize class with sample_rate of audio recordings.
		"""
		self.model = j_load(model)
		self.scaler = p_load(open(scaler, 'rb'))

	def classify_audio(self, X):
		"""
		Wrapper around prediction which returns a binary 
		"""
		c = self.model.predict(X)
  
		return c[0]

	def read_wav(self, file):
		'''
		Reads file.wav which will be soon be preprocessed
		'''
		sample_rate, audio = wav.read(file) 
		if len(audio.shape) > 1: # ignoring the second channel if the .wav file contains it 
			audio = audio[:,0]
		
		return sample_rate, audio

	def process_recording(self, rec, sample_rate):
		"""
		raw_mic_data must be entered as a .wav file without lossy compression
		Returns: frequency data as a list 
		"""

		# preprocessing is raw -> mfcc's -> scale -> classify
		mfcc = librosa.feature.mfcc(rec, sr = sample_rate).flatten()
		scaled = self.scaler.transform([mfcc])

		return scaled

