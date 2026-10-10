"""Original driving vision tail plus training-only spatial/geometry supervision.

The auxiliary heads are never exported. This is an offline experiment, not a
traffic-permission detector or a vehicle control path.
"""
import onnx
from onnx import numpy_helper
import torch
from torch import nn
from signal_vision_model import VisionTrial, VisionTail, TemporalPolicy


class SpatialTrial(VisionTrial):
  def __init__(self, source):
    nn.Module.__init__(self)
    model = onnx.load(source)
    arrays = {x.name: numpy_helper.to_array(x).copy() for x in model.graph.initializer}
    self.vision = VisionTail(model.graph, arrays, 'vision/conv2d_43', 'vision/add_8')
    self.policy = TemporalPolicy(model.graph, arrays)
    self.color = nn.Linear(512, 2)
    # Separate camera-coordinate predictions on the existing fused spatial map.
    # Class0=reviewed distractor;1=selected approach red;2=selected approach green.
    self.spatial_head = nn.Conv2d(256, 6, 1)
    self.center_head = nn.Conv2d(256, 4, 1)
    self.stop_head = nn.Linear(512, 3)

  def configure(self, temporal=False):
    assert not temporal, 'This trial freezes the temporal policy for scope isolation'
    for name in self.vision.names:
      w = self.vision.w(name)
      if w.is_floating_point():
        w.requires_grad_(name.startswith('vision/vision_model.vision._en.'))
    for w in self.policy.parameters():
      w.requires_grad_(False)

  def auxiliary(self, maps):
    vision, taps = self.vision(maps, capture=('vision/add_9',))
    spatial = taps['vision/add_9']
    return self.spatial_head(spatial), self.center_head(spatial), self.stop_head(vision[:, 1064:1576])

  def auxiliary_parameters(self):
    return [p for head in (self.color, self.spatial_head, self.center_head, self.stop_head) for p in head.parameters()]
