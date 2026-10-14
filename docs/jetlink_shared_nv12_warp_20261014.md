# Shared Jetlink NV12 preparation on C3, C3X and C4

The user requested the direct NV12-to-model-layout optimization on both
carrot-wip and carrot-nav for C3/C3X as well as C4. All three supported device
types now select the same candidate and require the same exact on-device GPU
comparison before using it. The input layout still comes from the camera's
actual width, height, stride and plane padding: current C3/C3X uses 1928x1208
and C4 uses 1344x760. Model input remains two six-channel 128x256 tensors.

The existing 27 identity, projective, border and rounding probes compare every
output byte against the original QCOM preparation. Shape, dtype, pixel or
compilation failure retains the original path; transforms are reset afterward.
Preparation remains in the existing separate startup worker. This does not
change recurrent state, camera pairing, pose validity, driving model choice,
the AMD eGPU preparation path, or the C4-specific DM scheduling helper.

Regression tests exercise selection and rejection for all three devices and
their actual NV12 allocation sizes. Desktop tests are not native C3/C3X speed
measurements or proof that reported deadline misses are fixed. The device-local
GPU check runs at the next Jetlink warp preparation after restarting with the
updated source; the verified log now includes the device and camera dimensions.
There is no new TensorRT engine or Jetson host package required for this change.
