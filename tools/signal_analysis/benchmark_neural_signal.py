"""Same-input CPU/QCOM benchmark. Run each backend in a separate process.

Inputs: NPZ arrays `images` [N,3,416,416], `input` [N,3,224,224].
Times include host input transfer and synchronous output readback, exclude
camera/preprocessing/startup. Output NPZ supports numerical parity comparison.
"""
import argparse
import json
import os
from pathlib import Path
import time

import numpy as np


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--models', type=Path, required=True)
  parser.add_argument('--inputs', type=Path, required=True)
  parser.add_argument('--backend', choices=['cpu', 'qcom'], required=True)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--repeats', type=int, default=3)
  parser.add_argument('--export-qcom', action='store_true', help='Save locally compiled, identity-checked JITs beside the ONNX files')
  args = parser.parse_args()
  if not 1 <= args.repeats <= 20:
    parser.error('repeats must be 1..20')
  for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[name] = '1'
  from tools.signal_analysis.neural_signal import NeuralSignal
  begin = time.monotonic()
  nn = NeuralSignal(args.models/'detector.onnx', args.models/'classifier.onnx', backend=args.backend)
  inputs = np.load(args.inputs, allow_pickle=False)
  output, results = {}, {}
  for name, session, shape in [('images', nn.detector, (3,416,416)), ('input', nn.classifier, (3,224,224))]:
    values = inputs[name]
    if values.dtype != np.float32 or values.shape[1:] != shape or not 1 <= len(values) <= 100 or not np.isfinite(values).all():
      raise ValueError('invalid benchmark inputs')
    warm = time.monotonic()
    for _ in range(3): session.run(None, {name: values[:1]})
    startup = time.monotonic()-warm
    timings, predictions = [], []
    for repeat in range(args.repeats):
      for i in range(len(values)):
        start = time.monotonic()
        prediction = session.run(None, {name: values[i:i+1]})[0]
        timings.append((time.monotonic()-start)*1000)
        if repeat == 0: predictions.append(prediction)
    output[name] = np.concatenate(predictions)
    if args.export_qcom:
      if args.backend != 'qcom':
        raise ValueError('compiled export requires QCOM')
      import hashlib
      import pickle
      from tools.signal_analysis.neural_signal import qcom_runtime_identity
      model = args.models / ('detector.onnx' if name == 'images' else 'classifier.onnx')
      compiled = model.with_suffix('.qcom.pkl')
      temporary = compiled.with_suffix('.tmp')
      with temporary.open('wb') as f:
        pickle.dump((session.input_names, session.forward), f, protocol=5)
      metadata = dict(version=1, model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                      compiled_sha256=hashlib.sha256(temporary.read_bytes()).hexdigest(), runtime=qcom_runtime_identity())
      temporary.replace(compiled)
      temporary = compiled.with_suffix('.json.tmp')
      temporary.write_text(json.dumps(metadata, indent=2)); temporary.replace(compiled.with_suffix('.json'))
    results[name] = dict(warmup_s=startup, mean_ms=float(np.mean(timings)),
                         p50_p95_ms=np.percentile(timings,[50,95]).tolist(), timings_ms=timings)
    print(json.dumps(dict(backend=args.backend, model=name, **results[name])), flush=True)
  np.savez(args.output.with_suffix('.npz'), **output)
  args.output.with_suffix('.json').write_text(json.dumps(dict(backend=args.backend, elapsed_s=time.monotonic()-begin,
    detector_sha256=nn.DETECTOR_SHA, classifier_sha256=nn.CLASSIFIER_SHA,
    cpu_affinity=sorted(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else None, results=results), indent=2))


if __name__ == '__main__':
  main()
