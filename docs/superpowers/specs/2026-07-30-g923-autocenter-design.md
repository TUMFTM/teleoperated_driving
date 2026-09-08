# G923 Fixed Autocenter Design

## Goal

Enable a moderate centering force on the Logitech G923 while retaining the
existing ToD steering, throttle, and brake input path.

## Approach

Use the Linux evdev `FF_AUTOCENTER` interface directly. A small runtime helper
will locate the G923 through `/dev/input/by-id/*G923*-event-joystick`, verify
that the resolved input device identifies as a Logitech G923, and write a 30%
autocenter value to that device.

This is preferred over launching ToD's `UsbEventHandler` because that node
selects the first writable `/dev/input/event*` device and expects a dynamic
constant-force ROS topic that is not currently published. It is also preferred
over rebuilding the Docker images because fixed autocenter is a host driver
setting and does not require changes to the vehicle-control stack.

## Behavior

- Default centering strength is 30%.
- Strength is restricted to the range 0-100%.
- The helper refuses to operate unless the selected device is the G923 event
  interface.
- Existing ToD containers continue running without restart.
- The command must be reapplied after the wheel is disconnected, after USB is
  reassigned between Windows and VMware, or after Ubuntu restarts.
- Setting strength to 0 disables autocenter.

## Verification

Verify that the helper resolves the G923 event device, successfully writes the
autocenter event, leaves both ToD containers running, and leaves the joystick
ROS topic with one publisher. Physical confirmation is that the wheel develops
a moderate restoring torque toward its center position.

## Scope

This change enables fixed centering only. Vehicle-state-based force feedback
and modifications to ToD's `UsbEventHandler` are outside this change.
