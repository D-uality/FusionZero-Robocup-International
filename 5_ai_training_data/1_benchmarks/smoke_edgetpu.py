import pathlib, tflite_runtime.interpreter as tflite

model = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/models/black_edgetpu.tflite"
print('import OK')

tflite.load_delegate('libedgetpu.so.1')

print('delegate loaded OK')

tflite.Interpreter(model_path=str(model),
                   experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])

print('interpreter built OK')
