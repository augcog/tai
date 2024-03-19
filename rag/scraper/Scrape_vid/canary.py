from nemo.collections.asr.models import EncDecMultiTaskModel

# load model
canary_model = EncDecMultiTaskModel.from_pretrained('nvidia/canary-1b')

# # update dcode params
# decode_cfg = canary_model.cfg.decoding
# decode_cfg.beam.beam_size = 1
# canary_model.change_decoding_strategy(decode_cfg)


predicted_text = canary_model.transcribe(
    "transcribe_manifest.json",
    batch_size=16,  # batch size to run the inference with
)
print(predicted_text)
#
# # python [NEMO_GIT_FOLDER]/examples/asr/transcribe_speech.py \
# #  pretrained_name="nvidia/canary-1b" \
# #  audio_dir="test.wav"
