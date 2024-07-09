import tensorflow as tf

print("TensorFlow version:", tf.__version__)
print("Num GPUs Available: ", len(tf.config.experimental.list_physical_devices('GPU')))

if len(tf.config.experimental.list_physical_devices('GPU')) == 0:
    print("No GPU found. Please check your installation.")
else:
    for device in tf.config.experimental.list_physical_devices('GPU'):
        print("GPU Device:", device)
