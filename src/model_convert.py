import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, InputLayer
from tensorflow.keras.models import save_model
import os

def convert_tflite_to_h5(tflite_model_path, output_h5_path):
    # Load the TensorFlow Lite model
    interpreter = tf.lite.Interpreter(model_path=tflite_model_path)
    interpreter.allocate_tensors()
    
    # Convert TensorFlow Lite model to a Keras model
    converter = tf.lite.TFLiteConverter.from_saved_model(tflite_model_path)
    model = converter.convert()

    # Save the Keras model as .h5
    with open("converted_model.h5", "wb") as f:
        f.write(model)

    # Save the Keras model
    save_model(model, output_h5_path)

if __name__ == '__main__':
    # Model paths
    real_path = os.path.dirname(os.path.realpath(__file__))
    tflite_model_path = os.path.join(real_path, 'model_mobile_seg_fp32.tflite')
    output_h5_path = os.path.join(real_path, 'converted_model.h5')

    # Convert TensorFlow Lite model to Keras model
    convert_tflite_to_h5(tflite_model_path, output_h5_path)
