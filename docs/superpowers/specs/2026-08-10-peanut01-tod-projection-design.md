# Peanut01 TOD Projection Design

## Goal

Enable `tod_projection` for Peanut01 so the operator HMI displays the vehicle's
predicted left and right body-edge paths from the measured front tire angle.
The projection is visualization-only and must not publish or modify vehicle
control commands.

## Scope

This change covers:

- a rear-axle kinematic projection model for Peanut01;
- selection of `steering_tire_angle` as the steering input;
- Peanut01 projection configuration and launch enablement;
- geometry, configuration, and deployment tests;
- runtime verification on the operator and vehicle computers.

It does not cover odometry, trajectory guidance, pure pursuit, object
detection, or changes to vehicle actuation.

## Existing Behavior

`OperatorLaneProjection` subscribes to `PrimaryVehicleState` and
`SecondaryVehicleState`, then publishes four `nav_msgs/msg/Path` topics for
the front-left, front-right, rear-left, and rear-right vehicle boundaries.

The existing implementation:

- converts `steering_wheel_angle` to a road-wheel angle;
- advances a point treated as the center of mass;
- uses bumper distances in the slip-angle calculation;
- assumes the path coordinate origin is that center point.

Peanut01 instead provides a measured `steering_tire_angle`, and its
`base_footprint` origin is the center of the rear axle. Reusing the legacy
assumptions would produce an incorrect curvature and body offset.

## Compatibility Strategy

Add a configurable projection model instead of replacing the existing model
for every vehicle.

The default remains the legacy model and steering-wheel input. Peanut01
selects:

```yaml
steering_angle_source: steering_tire_angle
kinematic_reference: rear_axle
prediction_length_m: 6.0
prediction_steps: 40
```

This preserves existing EDGAR and simulation behavior while allowing
Peanut01 to use the correct measurement and reference point.

## Vehicle Geometry

The supplied Peanut01 geometry is:

| Parameter | Value |
| --- | ---: |
| Wheel radius | 0.15 m |
| Wheel width | 0.09 m |
| Wheelbase | 0.80 m |
| Wheel tread | 0.889 m |
| Front overhang | 0.295 m |
| Rear overhang | 0.5778 m |
| Left overhang | 0.152 m |
| Right overhang | 0.152 m |
| Vehicle height | 1.316 m |
| Maximum front tire angle | 1.047 rad |

The projection footprint relative to the rear-axle origin is therefore:

- front edge: `0.80 + 0.295 = 1.095 m`;
- rear edge: `-0.5778 m`;
- left edge: `0.889 / 2 + 0.152 = 0.5965 m`;
- right edge: `-(0.889 / 2 + 0.152) = -0.5965 m`.

The existing TOD vehicle parameters represent distances from an assumed
center of mass. For Peanut01 they remain:

```yaml
distance_front_axle: 0.4
distance_rear_axle: 0.4
width_edge_to_edge: 1.193
track_width: 0.889
maximum_road_wheel_angle: 1.047
height: 1.316
distance_front_bumper: 0.695
distance_rear_bumper: 0.9778
```

The rear-axle model derives the wheelbase and overhangs from these values:

```text
wheelbase       = distance_front_axle + distance_rear_axle
front_overhang  = distance_front_bumper - distance_front_axle
rear_overhang   = distance_rear_bumper - distance_rear_axle
```

Wheel radius, wheel width, mass, inertia, and cornering forces are not used by
this visualization model.

## Projection Algorithm

The rear axle is the kinematic state origin. For each projection step:

```text
delta = clamp(measured_steering_tire_angle,
              -maximum_road_wheel_angle,
              +maximum_road_wheel_angle)

x_next   = x + direction * ds * cos(yaw)
y_next   = y + direction * ds * sin(yaw)
yaw_next = yaw + direction * ds * tan(delta) / wheelbase
```

`direction` is `-1` only for reverse gear and `+1` otherwise. The configured
prediction length is divided evenly across the configured step count. The
calculation does not depend on current vehicle speed, matching the existing
constant-length HMI behavior.

Before integration, the four body corners at the current rear-axle pose are
appended as the first path sample. After every step, the transformed corners
are appended again. Consequently, `prediction_steps: 40` produces 40 path
segments and 41 poses per boundary, with no gap between the HMI vehicle model
and the guide lines. Published paths retain the `base_footprint` frame.

## Configuration And Launching

Add `package_config/tod_projection/params.yaml` to both the package defaults
and deployment configuration. The package default selects legacy behavior;
the Peanut01 deployment selects rear-axle behavior and tire-angle input.

Update `tod_projection.launch.py` to load this parameter file from
`config_path`, following the configuration pattern already used by other TOD
packages.

Set `packages_to_launch.operator.tod_projection` to `true` in the Peanut01
video launch profile. Existing remappings already connect:

- primary and secondary vehicle state to the projection node;
- the four projection outputs to `tod_visual` driving-lane inputs.

No new cross-machine transport or ROS domain bridge is required.

## Validation And Failure Handling

The node must validate at startup that:

- wheelbase is greater than zero;
- vehicle width is greater than zero;
- prediction length is greater than zero;
- prediction step count is greater than zero;
- the steering source and reference model values are recognized;
- derived front and rear overhangs are non-negative.

Invalid configuration must produce a clear fatal error and prevent the
projection node from publishing misleading paths. Non-finite steering inputs
must be rejected for that callback. Tire angles outside the configured limit
must be clamped, with a throttled warning.

## Testing

Unit tests cover:

- zero tire angle produces four straight paths;
- positive and negative tire angles bend in opposite directions;
- reverse gear reverses path progression;
- the first path sample uses the rear-axle body offsets and subsequent samples
  span the configured prediction length;
- excessive tire angles are clamped to 1.047 rad;
- invalid geometry and invalid model configuration are rejected;
- legacy steering-wheel selection remains available.

Deployment tests cover:

- the Peanut01 profile enables `tod_projection`;
- the Peanut01 projection parameter file selects the rear-axle model and tire
  angle;
- the launch file loads the projection parameter file;
- the existing four HMI remappings remain present.

Runtime acceptance requires:

1. `LaneProjection` is running on the operator computer.
2. Primary and secondary vehicle-state topics have live publishers.
3. All four projection path topics publish at the vehicle-state update rate.
4. Straight wheels display straight and symmetric lane edges.
5. Left and right steering bend the paths in the correct direction.
6. Reverse gear projects behind the rear axle.
7. The front and rear extents align with the Peanut01 HMI model.
8. Existing video, point-cloud, state, and direct-control paths remain active.

## Safety

`tod_projection` remains an operator-side visualization component. It does not
publish `PrimaryControlCmd`, `SecondaryControlCmd`, or trajectory-guidance
commands. A projection failure must therefore remove only the guide lines and
must not affect command forwarding or the vehicle safety gate.
