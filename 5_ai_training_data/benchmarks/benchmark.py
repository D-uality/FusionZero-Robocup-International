#!/usr/bin/env python3
import pathlib, sys, time, numpy as np, tflite_runtime.interpreter as tflite

# ─── configurable section ──────────────────────────────────────────────────────
modelPath  = "/home/frederick/FusionZero-Robocup-International/5_ai_training_data/models/black.tflite"
delegate   = "cpu"          # "cpu" or "tpu"
numRuns    = 50
# ───────────────────────────────────────────────────────────────────────────────

def runInference():
    useTPU = delegate == "tpu"
    if useTPU:
        interpreter = tflite.Interpreter(model_path=str(modelPath),
                                         experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])
    else:
        interpreter = tflite.Interpreter(model_path=str(modelPath))
    interpreter.allocate_tensors()
    inp = interpreter.get_input_details()[0]
    out = interpreter.get_output_details()[0]

    if np.dtype(inp["dtype"]).kind not in ("u", "i"):
        sys.exit("Model isn’t integer-quantised; Edge TPU will reject it.")

    lo, hi = (0, 256) if inp["dtype"] == np.uint8 else (-128, 128)
    sample = np.random.randint(lo, hi, inp["shape"], inp["dtype"])

    for _ in range(10):
        interpreter.set_tensor(inp["index"], sample)
        interpreter.invoke()

    t0 = time.perf_counter()
    for _ in range(numRuns):
        interpreter.set_tensor(inp["index"], sample)
        interpreter.invoke()
    dt = (time.perf_counter() - t0) / numRuns * 1000
    
    print(f"{'TPU' if useTPU else 'CPU'}  {numRuns} runs  mean {dt:.2f} ms   first-5:",
          interpreter.get_tensor(out["index"]).flatten()[:5])

if __name__ == "__main__":
    runInference()