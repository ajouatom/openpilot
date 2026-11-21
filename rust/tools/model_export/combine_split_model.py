"""Combine a split comma driving model (vision + policy ONNX) into one supercombo.

Same graph contract as the internal CD210 (producer "carrot-cd210-combiner"),
so the combined model runs through the existing CD210 native pipeline,
input packing and parser unchanged:

  inputs   img, big_img [1,12,128,256] uint8; desire_pulse [1,25,8],
           traffic_convention [1,2], action_t [1,2] (unused, as in CD210),
           features_buffer [1,24,512] fp16 (past frames only)
  graph    vision/* and policy/* subgraphs; the current frame's vision
           hidden_state is appended to features_buffer inside the graph
  output   outputs [1, vision + policy] fp16 with CD210's output_slices

Unused initializers are dropped (POPv2's vision ships 12 unused
"summarizer.*" tensors that would otherwise be uploaded to the GPU).

Usage: combine_split_model.py VISION.onnx POLICY.onnx OUT.onnx --name NAME
"""
from __future__ import annotations

import argparse
import codecs
import pickle

import onnx
from onnx import TensorProto, helper, numpy_helper
import numpy as np


def prune(model: onnx.ModelProto) -> int:
    used = {name for node in model.graph.node for name in node.input}
    used |= {output.name for output in model.graph.output}
    keep = [init for init in model.graph.initializer if init.name in used]
    dropped = len(model.graph.initializer) - len(keep)
    del model.graph.initializer[:]
    model.graph.initializer.extend(keep)
    return dropped


def metadata(model: onnx.ModelProto) -> dict[str, str]:
    return {prop.key: prop.value for prop in model.metadata_props}


def slices(model: onnx.ModelProto) -> dict[str, slice]:
    return pickle.loads(codecs.decode(metadata(model)["output_slices"].encode(), "base64"))


def width(value: onnx.ValueInfoProto, axis: int) -> int:
    return value.type.tensor_type.shape.dim[axis].dim_value


def combine(vision: onnx.ModelProto, policy: onnx.ModelProto, name: str) -> onnx.ModelProto:
    vision_slices, policy_slices = slices(vision), slices(policy)
    hidden = vision_slices["hidden_state"]
    vision_width = width(vision.graph.output[0], 1)
    policy_width = width(policy.graph.output[0], 1)
    policy_inputs = {value.name: value for value in policy.graph.input}
    if set(policy_inputs) - {"desire_pulse", "traffic_convention", "features_buffer", "action_t"}:
        raise ValueError(f"unexpected policy inputs {sorted(policy_inputs)}")
    history = width(policy_inputs["features_buffer"], 1) - 1
    checkpoint = f"{metadata(vision)['model_checkpoint']}+{metadata(policy)['model_checkpoint']}"
    dropped = prune(vision) + prune(policy)

    vision = onnx.compose.add_prefix(vision, "vision/")
    policy = onnx.compose.add_prefix(policy, "policy/")
    fp16 = TensorProto.FLOAT16
    inputs = [
        helper.make_tensor_value_info("img", TensorProto.UINT8, [1, 12, 128, 256]),
        helper.make_tensor_value_info("big_img", TensorProto.UINT8, [1, 12, 128, 256]),
        helper.make_tensor_value_info("desire_pulse", fp16, [1, width(policy_inputs["desire_pulse"], 1), 8]),
        helper.make_tensor_value_info("traffic_convention", fp16, [1, 2]),
        helper.make_tensor_value_info("action_t", fp16, [1, 2]),
        helper.make_tensor_value_info("features_buffer", fp16, [1, history, hidden.stop - hidden.start]),
    ]
    constants = [
        numpy_helper.from_array(np.array([hidden.start], np.int64), "hidden_starts"),
        numpy_helper.from_array(np.array([hidden.stop], np.int64), "hidden_ends"),
        numpy_helper.from_array(np.array([1], np.int64), "hidden_axes"),
        numpy_helper.from_array(np.array([1], np.int64), "hidden_steps"),
        numpy_helper.from_array(np.array([1], np.int64), "hidden_unsqueeze_axes"),
    ]
    bridges = [
        helper.make_node("Identity", ["img"], ["vision/img"], name="bridge_img"),
        helper.make_node("Identity", ["big_img"], ["vision/big_img"], name="bridge_big_img"),
        helper.make_node("Identity", ["desire_pulse"], ["policy/desire_pulse"], name="bridge_desire"),
        helper.make_node("Identity", ["traffic_convention"], ["policy/traffic_convention"], name="bridge_traffic"),
    ]
    if "action_t" in policy_inputs:
        bridges.append(helper.make_node("Identity", ["action_t"], ["policy/action_t"], name="bridge_action"))
    hidden_nodes = [
        helper.make_node("Slice", ["vision/outputs", "hidden_starts", "hidden_ends", "hidden_axes", "hidden_steps"],
                         ["current_hidden_flat"], name="extract_current_hidden"),
        helper.make_node("Unsqueeze", ["current_hidden_flat", "hidden_unsqueeze_axes"], ["current_hidden"],
                         name="unsqueeze_current_hidden"),
        helper.make_node("Concat", ["features_buffer", "current_hidden"], ["policy/features_buffer"], axis=1,
                         name="append_current_hidden"),
    ]
    combine_node = helper.make_node("Concat", ["vision/outputs", "policy/outputs"], ["outputs"], axis=1,
                                    name="combine_outputs")
    graph = helper.make_graph(
        bridges + list(vision.graph.node) + hidden_nodes + list(policy.graph.node) + [combine_node],
        name,
        inputs,
        [helper.make_tensor_value_info("outputs", fp16, [1, vision_width + policy_width])],
        initializer=constants + list(vision.graph.initializer) + list(policy.graph.initializer),
    )
    combined = helper.make_model(graph, producer_name=f"carrot-{name.lower()}-combiner",
                                 opset_imports=list(vision.opset_import))
    combined.ir_version = vision.ir_version
    merged = {key: value for key, value in vision_slices.items() if key != "pad"}
    for key, value in policy_slices.items():
        start, stop, _ = value.indices(policy_width)
        merged[key] = slice(vision_width + start, vision_width + stop)
    onnx.helper.set_model_props(combined, {
        "model_checkpoint": checkpoint,
        "output_slices": codecs.encode(pickle.dumps(merged), "base64").decode(),
    })
    onnx.checker.check_model(combined)
    print(f"combined {name}: {vision_width}+{policy_width} outputs, history {history}, dropped {dropped} unused initializers")
    return combined


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("vision")
    parser.add_argument("policy")
    parser.add_argument("output")
    parser.add_argument("--name", required=True)
    args = parser.parse_args()
    combined = combine(onnx.load(args.vision), onnx.load(args.policy), args.name)
    onnx.save(combined, args.output)


if __name__ == "__main__":
    main()
