# record n second .wav files until the process is stopped

import wave
import pyaudio

def record_audio(seconds, filename):
    '''
    Record a .wav file for a defined amount of seconds. Output to a user defined filename.
    '''
    form_1 = pyaudio.paFloat32 # Float
    chans = 1 # 1 channel
    samp_rate = 44100 # 44.1kHz sampling rate
    chunk = 4096 # 2^12 samples for buffer
    record_secs = seconds # seconds to record
    dev_index = 2 # device index found by p.get_device_info_by_index(ii)
    recordings_directory = "recordings/"
    wav_output_filename = recordings_directory + filename # name of .wav file

    audio = pyaudio.PyAudio() # create pyaudio instantiation

    # create pyaudio stream
    stream = audio.open(format = form_1,rate = samp_rate,channels = chans, \
                        input_device_index = dev_index,input = True, \
                        frames_per_buffer=chunk)
    print("\n\n\n\n\nRecording {} Started".format(filename))
    frames = []

    # loop through stream and append audio chunks to frame array
    for ii in range(0,int((samp_rate/chunk)*record_secs)):
        data = stream.read(chunk)
        frames.append(data)

    print("Finished Recording")

    # stop the stream, close it, and terminate the pyaudio instantiation
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # save the audio frames as .wav file
    wavefile = wave.open(wav_output_filename,'wb')
    wavefile.setnchannels(chans)
    wavefile.setsampwidth(audio.get_sample_size(form_1))
    wavefile.setframerate(samp_rate)
    wavefile.writeframes(b''.join(frames))
    wavefile.close()


if __name__ == "__main__":
    SECONDS = 0.5
    base_file_name = "recording_{n}"
    counter = 0

    record_audio(5, "testing_float.wav")
    # # record audio until told otherwise
    # while(True):
    #     record_audio(SECONDS, base_file_name.format(n = counter) + '.wav')
    #     counter += 1
