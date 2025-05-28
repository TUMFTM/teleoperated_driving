 # TOD Visual Application {#tod_visual_application_docs}

## Overview

The Visual application implements a `SceneApplication` of the tod_graphics_library. The Main VisualNode implements the Ros Interface and initiates the layer stack. The Scene is build in the visual layer. The visual layer's framebuffer is used in the viewport layer to generate the docking layer for the rest of the windows. In `tod_direct_control` only the viewport is shown. When `tod_trajectory_guidance` is active the streams are displayed in 3 Windows above the Viewport layer and the scene becomes interactive to plan and monitor the trajectory.  


TODO Screenshots 

Content:


## Executables

- **visual:** (Visual ROS Node)

### Layer

- **VisualLayer:** Scene Construction layer, intialised the entities, transforms and bind scriptable entities 
- **ViewportLayer:** transforms the scene's framebuffer into a texture to be rendered. Contains logic for the mouse position to determine the mouse's position over the viewport
- **VisualIOLayer:** Registers the subscribing components of the application
- **StateLayer:** Registers the scriptable entities and layers for the conditional rendering
- **DriveInfoLayer:** Main UI Layer containing car HMI information such as Speed, Gear, FlashLight, Indicator, and Network Information
- **SettingsLayer:** Debug layer to turn of certain renderes
- **SwitchCameraLayer:** Additional UI Dummy Elements 
- **TrafficSignLayer:** Renders the current traffic sign of the lanelet2 map into the UI
- **VideoLayer:** Videolayer that renders the current image data onto the layer
- **TrajectoryGuidanceStateLayer:** UI Element to render the current state of the vehicle

## Subscribed Topics

- Registered Components in the `VisualIOLayer` 
  1. ControlComponent
  2. OdometryComponent
  3. ImageComponents (x6)
  4. PredictedObjectComponent
  5. DrivingLaneComponent (x4)
  6. TrajectoryComponent
  7. PointCloudComponent
  8. PathComponent
  9. PathControlPointComponent
  10. NetworkMetricsComponent
  11. PrimaryControlComponent
  12. SecondaryControlComponent
  13. TodStatusComponent
  14. VehicleDataComponent
  15. JoyStickComponent
  16. TrajectoryGuidanceStateComponent

## Published Topics
TODO Change in Code

- `/operator/visual/mouse_position_click` 
- `/operator/visual/key_press`
- `/operator/visual/key_release`
- `/operator/visual/mouse_position_moved`
- `/operator/visual/mouse_position_released`

## Parameters

TODO

## Build

'''bash 
colcon build --packages-up-to tod_visual 
'''


## Launch

'''bash 
ros2 run tod_visual vehicleID:=edgar
'''

## Doxygen stuff
\version 1.0
\author Niklas Krauss
\defgroup tod_visual_application TOD Visual 
\brief Operator Interface with a 3D Scene to interact with and control the vehicle
\ingroup tod_operator_interface


