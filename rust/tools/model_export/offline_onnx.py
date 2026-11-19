from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Final

from tinygrad import Context, Tensor, dtypes
from tinygrad.nn.onnx import OnnxRunner, _to_python_const, required_input_python_consts

from .offline_model import InitializedBuffer, OfflineModel
from .offline_devices import host_build
from .schema import Bindings, ExportError


STATIC_OPERATORS: Final = frozenset({
    "Identity", "Cast", "Concat", "Sub", "Div", "Conv", "Gelu", "Mul", "Add", "ReduceMean", "Relu", "Sigmoid",
    "Reshape", "Gemm", "ReduceL2", "Clip", "Expand", "Slice", "Unsqueeze", "Transpose", "GatherND", "MatMul",
    "LayerNormalization", "Squeeze", "Where", "Softmax", "Gather", "MaxPool", "Resize", "GlobalAveragePool", "Flatten", "Constant",
    # Cheddar: constant split sizes and reduction axes, so shapes stay static.
    "Split", "ReduceMax",
})


@dataclass(frozen=True, slots=True)
class OfflineOnnx:
    runner: OnnxRunner
    initialized: tuple[InitializedBuffer, ...]


@host_build
def load_runner(path: Path, expected_sha256: str, *, float16: int = 1) -> OfflineOnnx:
    if float16 not in (0, 1):
        raise ExportError("float16 must be 0 or 1")
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha256:
        raise ExportError("ONNX model identity differs from the pinned object")
    with Context(DEV="CPU:LLVM", IMAGE=1, FLOAT16=float16, OPENPILOT_HACKS=1):
        runner = OnnxRunner(path)
        if runner.is_training:
            raise ExportError("offline ONNX training graph is unsupported")
        original_values = dict(runner.graph_values)
        for node in runner.graph_nodes:
            if node.op not in STATIC_OPERATORS:
                raise ExportError(f"unsupported offline ONNX operator: {node.op}")
            for index in required_input_python_consts.get(node.op, ()):
                if index >= len(node.inputs) or not node.inputs[index]:
                    continue
                name = node.inputs[index]
                if name not in original_values:
                    raise ExportError("offline shape control depends on runtime content")
                runner._get_python_const(name, node.op, index)
            if node.op == "Gather" and len(node.inputs) > 1 and node.inputs[1] in runner.const_names:
                name = node.inputs[1]
                value = original_values[name]
                if not isinstance(value, Tensor):
                    raise ExportError("offline Gather control is not a source Tensor")
                runner._python_const_cache[name] = _to_python_const(value)
        initialized = []
        for name, value in original_values.items():
            if value is None:
                continue
            if not isinstance(value, Tensor):
                raise ExportError("offline ONNX initializer is not a source Tensor")
            data = value.data().tobytes()
            target = Tensor.empty(*value.shape, dtype=value.dtype, device="QCOM")
            initialized.append(InitializedBuffer(target.uop.base, data))
            runner.graph_values[name] = target
    return OfflineOnnx(runner, tuple(initialized))


@host_build
def load_model(path: Path, expected_sha256: str, *, float16: int = 1) -> OfflineModel:
    loaded = load_runner(path, expected_sha256, float16=float16)
    inputs = {}
    for name, spec in loaded.runner.graph_inputs.items():
        if spec.is_optional or spec.is_sequence:
            raise ExportError("offline ONNX input requires a single mandatory Tensor")
        shape = tuple(size if isinstance(size, int) else 1 for size in spec.shape)
        dtype = dtypes.float32 if spec.dtype is dtypes.float16 else spec.dtype
        inputs[name] = Tensor.empty(*shape, dtype=dtype, device="QCOM")
    with Context(DEV="QCOM", IMAGE=1, FLOAT16=float16, NOLOCALS=1, OPENPILOT_HACKS=1):
        outputs = loaded.runner(inputs)
    converted = {}
    for name, output in outputs.items():
        if not isinstance(output, Tensor):
            raise ExportError("offline ONNX output requires a single Tensor")
        converted[name] = output.cast(dtypes.float32).contiguous() if not float16 else output.cast(dtypes.float32)
    return OfflineModel(Bindings(inputs, converted), loaded.initialized)
