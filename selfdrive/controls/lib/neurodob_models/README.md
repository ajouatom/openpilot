# NeuroDOB Model Files

Place per-car JSON models in this directory.

## File names

- `torque_delta_<CAR>.json`
- `torque_delta_<CAR>_custom.json`
- `angle_delta_<CAR>.json`
- `angle_delta_<CAR>_custom.json`

`_custom` takes priority over the base file.

Examples:

- `torque_delta_HYUNDAI_SONATA.json`
- `angle_delta_HYUNDAI_KONA_EV_custom.json`

## Supported model types

- `mlp_delta`
- `gru_delta`

## Required top-level fields

- `version`
- `model_type`
- `control_type`
- `feature_order`
- `input_mean`
- `input_std`
- `max_delta`

## Torque feature order

```json
[
  "lateral_error",
  "lateral_error_rate",
  "yaw_rate_error",
  "steering_rate_deg",
  "control_output",
  "v_ego",
  "roll",
  "desired_lateral_accel"
]
```

## Angle feature order

```json
[
  "angle_error",
  "angle_error_rate",
  "yaw_rate_error",
  "steering_rate_deg",
  "control_output",
  "v_ego",
  "roll",
  "desired_lateral_accel"
]
```

## Minimal MLP example

```json
{
  "version": "1.0",
  "model_type": "mlp_delta",
  "control_type": "torque",
  "feature_order": [
    "lateral_error",
    "lateral_error_rate",
    "yaw_rate_error",
    "steering_rate_deg",
    "control_output",
    "v_ego",
    "roll",
    "desired_lateral_accel"
  ],
  "input_mean": [0, 0, 0, 0, 0, 0, 0, 0],
  "input_std": [1, 1, 1, 1, 1, 1, 1, 1],
  "max_delta": 0.10,
  "layers": [
    {
      "W": [[0, 0, 0, 0, 0, 0, 0, 0]],
      "b": [0],
      "activation": "identity"
    }
  ]
}
```

## Minimal GRU example

```json
{
  "version": "1.0",
  "model_type": "gru_delta",
  "control_type": "angle",
  "feature_order": [
    "angle_error",
    "angle_error_rate",
    "yaw_rate_error",
    "steering_rate_deg",
    "control_output",
    "v_ego",
    "roll",
    "desired_lateral_accel"
  ],
  "input_mean": [0, 0, 0, 0, 0, 0, 0, 0],
  "input_std": [1, 1, 1, 1, 1, 1, 1, 1],
  "hidden_size": 2,
  "max_delta": 1.0,
  "gru": {
    "Wz": [[0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0]],
    "Wr": [[0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0]],
    "Wh": [[0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0]],
    "Uz": [[0, 0], [0, 0]],
    "Ur": [[0, 0], [0, 0]],
    "Uh": [[0, 0], [0, 0]],
    "bz": [0, 0],
    "br": [0, 0],
    "bh": [0, 0]
  },
  "fc": {
    "W": [[0, 0]],
    "b": [0]
  }
}
```
