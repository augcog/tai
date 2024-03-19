from paddlespeech.server.bin.paddlespeech_client import ASRClientExecutor

asrclient_executor = ASRClientExecutor()
res = asrclient_executor(
    input="processed_audio.wav",
    server_ip="127.0.0.1",
    port=8090,
    sample_rate=16000,
    lang="en_us",
    audio_format="wav")

print(res)