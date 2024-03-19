import librosa
import soundfile as sf

# Assuming you're loading an audio file, processing it, and then attempting to save it
y, sr = librosa.load('intermediate_audio.wav', sr=16000, mono=True)

# Use soundfile.write to save the audio file
sf.write('processed_audio.wav', y, sr, format='WAV', subtype='PCM_16')
