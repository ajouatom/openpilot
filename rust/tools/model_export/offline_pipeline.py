from __future__ import annotations

from collections.abc import Callable, Generator
from contextlib import contextmanager
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Literal, assert_never

import numpy as np
from tinygrad import Context, Tensor, dtypes
from tinygrad.engine.realize import capturing
from tinygrad.uop.ops import Ops, UOp

from openpilot.common.transformations.model import DM_INPUT_SIZE, MEDMODEL_INPUT_SIZE
from openpilot.selfdrive.modeld.compile_dm_warp import make_warp_dm
from openpilot.selfdrive.modeld.compile_modeld import NV12Frame, make_input_queues, make_run_policy, make_warp
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

from .offline_capability import static_movement_capability
from .offline_model import InitializedBuffer, OfflineModel
from .offline_devices import host_build
from .offline_onnx import OfflineOnnx, load_runner
from .offline_schedule import export_pipeline as export_pipeline
from .original import Metadata
from .schema import Bindings, ExportError


@dataclass(frozen=True, slots=True)
class OfflinePipelineSpec:
    kind: Literal["driving", "driver"]
    model: Path
    sha256: str
    camera: tuple[int, int]


@dataclass(frozen=True, slots=True)
class OfflineStage:
    name: str
    linear: UOp


@dataclass(frozen=True, slots=True)
class OfflinePipeline:
    spec: OfflinePipelineSpec
    model: OfflineModel
    metadata: Metadata
    stages: tuple[OfflineStage, ...]


class ScheduleCollector:
    def __init__(self) -> None:
        self.linears: list[UOp] = []

    def add_linear(self, linear: UOp, variables: dict[str, int]) -> None:
        if variables:
            raise ExportError("offline pipeline requires static dimensions")
        self.linears.append(linear)


@contextmanager
def collect_schedule(collector: ScheduleCollector) -> Generator[None, None, None]:
    if capturing:
        raise ExportError("offline pipeline capture cannot be nested")
    capturing.append(collector)
    try:
        with static_movement_capability(), Context(DEV="QCOM", CAPTURING=1, NO_MEMORY_PLANNER=1, IMAGE=1,
                                                   FLOAT16=1, NOLOCALS=1, BEAM=0, OPENPILOT_HACKS=1):
            yield
    finally:
        capturing.remove(collector)


def capture_stage(name: str, function: Callable[[], tuple[Tensor, ...]]) -> tuple[OfflineStage, tuple[Tensor, ...]]:
    collector = ScheduleCollector()
    with collect_schedule(collector):
        outputs = function()
        Tensor.realize(*outputs)
    return OfflineStage(name, UOp(Ops.LINEAR, src=tuple(call for linear in collector.linears for call in linear.src))), outputs


def realize_prediction_heads(runner) -> int:
    """Compute each prediction head at its own width before the output Concat.

    Fusing head reductions into the final Concat makes every output lane run
    every head's reduction: Cheddar's r_645_… kernel (70 ms -> 32.7 ms once
    split), and the vision heads of CD210-style combined models (r_1576_…).
    Descends through nested Concats (combine_outputs -> vision/policy).
    """
    producers = {name: node for node in runner.graph_nodes for name in node.outputs}
    heads: list[Tensor] = []

    def visit(name: str) -> None:
        node = producers.get(name)
        if node is not None and node.op in ("Cast", "Identity"):
            visit(node.inputs[0])
        elif node is not None and node.op == "Concat":
            for source in node.inputs:
                visit(source)
        elif name != "outputs" and isinstance(runner.graph_values.get(name), Tensor):
            heads.append(runner.graph_values[name])

    root = producers.get("outputs")
    if root is None or root.op not in ("Cast", "Identity", "Concat"):
        return 0
    visit("outputs")
    if heads:
        Tensor.realize(*heads)
    return len(heads)


def driver_pipeline(loaded: OfflineOnnx, spec: OfflinePipelineSpec, nv12: NV12Frame) -> OfflinePipeline:
    with Context(DEV="CPU:LLVM"):
        metadata = Metadata.from_original(make_metadata_dict(spec.model))
    frame = Tensor.empty(nv12.size, dtype=dtypes.uint8, device="QCOM")
    transform = Tensor.empty(3, 3, dtype=dtypes.float32, device="NPY")
    calib = Tensor.empty(*metadata.input_shapes["calib"], dtype=dtypes.float32, device="NPY")
    warp = make_warp_dm(nv12, *DM_INPUT_SIZE)
    prepare, (warped,) = capture_stage("prepare", lambda: (warp(frame, transform),))
    model_stage, (output,) = capture_stage("model", lambda: (next(iter(loaded.runner({"input_img": warped,
                                                                                     "calib": calib.to("QCOM")}).values()))
                                                              .cast(dtypes.float32),))
    bindings = Bindings({"frame": frame, "transform": transform, "calib": calib}, {"model": output, "warped": warped})
    return OfflinePipeline(spec, OfflineModel(bindings, loaded.initialized), metadata, (prepare, model_stage))


def driving_pipeline(loaded: OfflineOnnx, spec: OfflinePipelineSpec, nv12: NV12Frame) -> OfflinePipeline:
    with Context(DEV="CPU:LLVM"):
        raw = make_metadata_dict(spec.model)
    frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ
    with Context(DEV="CPU:LLVM"):
        original_queues, _ = make_input_queues(raw["input_shapes"], frame_skip, "CPU")
    queues = {}
    initialized = list(loaded.initialized)
    for name, tensor in original_queues.items():
        if name in {"tfm", "big_tfm", "packed_npy_inputs"}:
            queues[name] = tensor
        else:
            queues[name] = Tensor.empty(*tensor.shape, dtype=tensor.dtype, device="QCOM")
            initialized.append(InitializedBuffer(queues[name].uop.base, tensor.data().tobytes()))
    frame = Tensor.empty(nv12.size, dtype=dtypes.uint8, device="QCOM")
    big_frame = Tensor.empty(nv12.size, dtype=dtypes.uint8, device="QCOM")
    warp = make_warp(nv12, *MEDMODEL_INPUT_SIZE, frame_skip)
    prepare, (warped,) = capture_stage("prepare", lambda: (warp(queues["tfm"], queues["big_tfm"], frame, big_frame),))
    policy = make_run_policy(loaded.runner, raw, frame_skip)

    def run_policy() -> tuple[Tensor, ...]:
        outputs = policy(warped, queues["img_q"], queues["big_img_q"],
                         queues["feat_q"], queues["desire_q"], queues["packed_npy_inputs"])
        realize_prediction_heads(loaded.runner)
        return outputs

    policy_stage, (output,) = capture_stage("policy", run_policy)
    bindings = Bindings({"tfm": queues["tfm"], "big_tfm": queues["big_tfm"], "frame": frame, "big_frame": big_frame,
                        **{name: queues[name] for name in ("img_q", "big_img_q", "feat_q", "desire_q", "packed_npy_inputs")}},
                       {"model": output, "warped": warped, **{name: queues[name] for name in ("img_q", "big_img_q", "feat_q", "desire_q")}})
    return OfflinePipeline(spec, OfflineModel(bindings, tuple(initialized)), Metadata.from_original(raw), (prepare, policy_stage))


RECURRENT_STATES = ("state_img_q", "state_desire_q", "state_feat_q")
# desire pulse (8), traffic convention (2), action times (2): the first twelve
# values of CD210's packed inputs, so drivingd's PolicyInputs layout is shared.
RECURRENT_PACKED = 12


def recurrent_driving_pipeline(loaded: OfflineOnnx, spec: OfflinePipelineSpec, nv12: NV12Frame,
                               raw: dict) -> OfflinePipeline:
    """Cheddar: image, desire and feature history live inside the ONNX graph.

    The three state tensors stay resident on the GPU between frames. The policy
    stage first computes the prediction and every next state from the old
    state, then copies the next states over it, so no kernel reads a state
    buffer that another kernel of the same frame has already overwritten.
    Camera frames are bound in place exactly as for CD210 (no NV12 copy).
    """
    shapes = raw["input_shapes"]
    model_w, model_h = MEDMODEL_INPUT_SIZE
    if tuple(shapes["new_img"]) != (2, 6, model_h // 2, model_w // 2) or tuple(shapes["desire"]) != (8,):
        raise ExportError("unsupported recurrent driving input contract")
    names = {"state_img_q": dtypes.uint8, "state_desire_q": dtypes.float32, "state_feat_q": dtypes.float32}
    initialized = list(loaded.initialized)
    states = {}
    for name, dtype in names.items():
        states[name] = Tensor.empty(*shapes[name], dtype=dtype, device="QCOM")
        initialized.append(InitializedBuffer(states[name].uop.base, bytes(math.prod(shapes[name]) * dtype.itemsize)))
    with Context(DEV="CPU:LLVM"):
        host = {name: Tensor(np.zeros(shape, dtype=np.float32), device="NPY").realize()
                for name, shape in (("tfm", (3, 3)), ("big_tfm", (3, 3)), ("packed_npy_inputs", (RECURRENT_PACKED,)))}
    frame = Tensor.empty(nv12.size, dtype=dtypes.uint8, device="QCOM")
    big_frame = Tensor.empty(nv12.size, dtype=dtypes.uint8, device="QCOM")
    frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ
    warp = make_warp(nv12, *MEDMODEL_INPUT_SIZE, frame_skip)
    prepare, (warped,) = capture_stage("prepare", lambda: (warp(host["tfm"], host["big_tfm"], frame, big_frame),))

    def policy() -> tuple[Tensor, ...]:
        packed = host["packed_npy_inputs"].to("QCOM")
        image = warped.to("QCOM")
        Tensor.realize(packed, image)
        outputs = loaded.runner({"new_img": image, "desire": packed[0:8], "traffic_convention": packed[8:10].reshape(1, 2),
                                 "action_t": packed[10:12].reshape(1, 2), **states})
        realize_prediction_heads(loaded.runner)
        model = outputs["outputs"].cast(dtypes.float32).contiguous()
        following = [outputs[f"next_{name}"].contiguous() for name in RECURRENT_STATES]
        Tensor.realize(model, *following)
        Tensor.realize(*(states[name].assign(value) for name, value in zip(RECURRENT_STATES, following, strict=True)))
        return (model, *(states[name] for name in RECURRENT_STATES))

    policy_stage, (output, *_) = capture_stage("policy", policy)
    bindings = Bindings({"tfm": host["tfm"], "big_tfm": host["big_tfm"], "frame": frame, "big_frame": big_frame,
                         "packed_npy_inputs": host["packed_npy_inputs"], **states},
                        {"model": output, "warped": warped, **states})
    # Only the flat prediction is a model output for drivingd; next states are
    # the resident state buffers above.
    metadata = Metadata.from_original({**raw, "output_shapes": {"outputs": raw["output_shapes"]["outputs"]}})
    return OfflinePipeline(spec, OfflineModel(bindings, tuple(initialized)), metadata, (prepare, policy_stage))


@host_build
def build_pipeline(spec: OfflinePipelineSpec) -> OfflinePipeline:
    if spec.camera not in {(1928, 1208), (1344, 760)}:
        raise ExportError("unsupported offline pipeline camera")
    loaded = load_runner(spec.model, spec.sha256)
    nv12 = NV12Frame(*spec.camera, *get_nv12_info(*spec.camera))
    match spec.kind:
        case "driver":
            return driver_pipeline(loaded, spec, nv12)
        case "driving":
            with Context(DEV="CPU:LLVM"):
                raw = make_metadata_dict(spec.model)
            if "new_img" in raw["input_shapes"]:
                return recurrent_driving_pipeline(loaded, spec, nv12, raw)
            return driving_pipeline(loaded, spec, nv12)
        case unreachable:
            assert_never(unreachable)
