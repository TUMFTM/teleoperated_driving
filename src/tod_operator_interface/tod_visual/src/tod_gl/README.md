# tod_graphics_library {#tod_graphics_library_docs}
 
## Overview
 
ToD Graphics Library is the general framework that powers the operator interface within the ToD Stack. The package provides a function-rich and flexible HMI for the operator to perform the teleoperation. It displays received data from the Perception and Bridge packages in various ways. A 3D world, comparable to the [rviz package](http://wiki.ros.org/rviz), is constructed using the OpenGL API. Inspired by the open source [game engine Hazel](https://github.com/TheCherno/Hazel), this package uses the Entity Component System (ECS) design pattern through the [entt library](https://skypjack.github.io/entt/), based on the composition over inheritance principle. Compared to the ROS1 Version it was extend to include the lightweight GUI Framework [dearImGui](https://github.com/ocornut/imgui). 

The Graphics Library contains both application functions for normal application windows and 3D scene based integrations (SceneApplication). ToD_Graphics_Library is separated into multiple sub packes:

- **Core**: Contains content related to the general application structure e.g. GLFW Application Window with ROS Integration, ImGui Layers as well as a StateManager API for determining the rendered entities based on the current ToD state. The Application implements the general RenderLoop of the Window. 
- **Events**: Event Handling of the Application including mouse and key events
- **Layers**: Basic template for the imGui layer architecture such as a docking structure, a layer stack. If the build application contains a 3D scene the scene layers have access to said scene and can access entity information in it.
- **Renderer**: OpenGl based rendering API based ont Hazel providing data handling of vertices, meshes and textures.
- **RosInterface**: Contais general ROS2 interface of the application and manages ROS Messages for the mouse and keyboard commands that are used in different parts of the ToD stack. Subscribing content contain the data container of an application. Each subscription_ is managed via an entity. Subscriptions have to be registered in the IOLayer. Afterwards they can be accessed via the SubscriptionManager. They provide simple read access and are not designed for publishing. 
- **Scene**: Provides an API to interact with the 3D scenes and its entities and components. It follows the Entity-Component-System proposed by Hazel. The Scene's render loop is contained in Scene.cpp .Scriptable Entities follow an update and render approach where they typically contain a RosInterface Data Container e.g. an ImageComponent and grab the update and then update the data of the RenderableComponent 
- **System**: Contains multiple systems used by the 3D scene e.g. VR, Scenecamera, Shadersystem or the Transform system of the scene
---
## Library Component Overview
---
### Subscribing Components
| Component name                   	| Type 	| Topic 	| Description                                                                                                                                                                                                 	|
|----------------------------------	|------	|-------	|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------	|
| AutomationStatusComponent                 	|    tod_automation_msgs::msg::VehicleAutomationState	|     input/automation_status    	| Subscribing to vehicle automation state and provides access to the current automation status                                                                                                                                          	|
| PointCloudComponent  	|      	|       	| ToD Decompressed Pointcloud from the PointCloudStreaming Package,  converted to 'sensor_msgs/PointCloud2'                                                                                                   	|
| DrivingLaneComponent             	|   nav_msgs::msg::Path   	|    input/driving_lane_front_left,  input/driving_lane_front_right,  input/driving_lane_rear_left,  input/driving_lane_rear_right	| Driving lanes calculated in from ToD Projection as 4 paths based on the current wheel base                                                                                                                  	|
| ImageComponent                   	|     sensor_msgs::msg::Image  sensor_msgs::msg::CameraInfo    	|    input/front_center/image, input/front_right/image, input/front_left/image   	input/front_center/cam_info, input/front_right/cam_info, input/front_left/cam_info | Image Topic Component containing both the image data and the camera information from the image                                                                                                              	|
| JoyStickComponent                	|    sensor_msgs::msg::Joy  	|   input/joystick    	| 'sensor_msgs/joy' msgs for communcation between the input devices and the interface                                                                                                                         	|
| OdometryComponent          	|  nav_msgs::msg::Odometry    	|     input/odom  	| Contains Odometry Information of the vehicle to 'base_link'.  The first message is used to offset the location within the transform system.  See documentation of transform system for further information 
| NetworkMetricsComponent          	|    tod_network_monitoring_msgs::msg::NetworkMetrics  	|    input/network_metrics   	| Information from tod_network_monitoring, specifically latency and link_quality                                                                                      
| PathComponent                    	|   PathMsg   	|       	| Renders a path with equally spaced ticks, generated from the tod_trajectory_guidance                                                                                                                        	|
| PathControlPointComponent        	|   tod_trajectory_guidance_msgs::msg::ControlPoints   	|    input/trajectory_guidance/path_control_points   	| Visualised the mouse clicks used to control the spline for tod_trajectory_guidance                                                                                                                          	|
| PointCloudComponent              	|   sensor_msgs::msg::PointCloud2   	|      input/pointcloud 	| Contains information for colored point clouds and projected images                                                                                                                                          	|
| PredictedObjectComponent                    	|   tod_automation_msgs::msg::PredictedObjects   	|     input/predicted_objects  	| Vector of predicted objects to be rendered in the scene, containing class, trajectory, bounding box size                                                                                                    	|
| PrimaryControlComponent          	|   tod_vehicle_msgs::msg::PrimaryControlCmd   	|       input/primary_control_command	| Control Commands for the vehicle in direct control                                                                                                                                                          	|
| PrimaryVehicleStateComponent         	|   tod_vehicle_msgs::msg::PrimaryVehicleState   	|       input/primary_vehicle_state 	| ubscription and the data for PrimaryControl topics for primary vehicle state                                                                                                                                                          	|
| SecondaryControlComponent        	|   tod_vehicle_msgs::msg::SecondaryControlCmd   	|     input/secondary_control_command  	| Control Commands for the vehicle in direct control for gear, indicator, and flash lights                                                                                                                    	|
| SecondaryVehicleStateComponent         	|   tod_vehicle_msgs::msg::SecondaryVehicleState   	|     input/secondary_vehicle_state 	| Csubscription and the data for SecondaryControl topics for Secondary vehicle state                                                                                                                  	|
| TodStatusComponent               	|  tod_status_msgs::msg::Status    	|   input/tod_status    	| Status of the state machine of the tod system                                                                                                                                                               	|
| TrajectoryComponent              	|    tod_automation_msgs::msg::Trajectory  	|      input/trajectory 	| Contains information about the current trajectory driven by the vehicle                                                                                                                                     	|
| TrajectoryGuidanceStateComponent 	|      tod_trajectory_guidance_msgs::msg::TrajectoryGuidanceState 	|      input/trajectory_guidance/trajectory_guidance_state 	| Status of the TrajectoryGuidance state machine                                                                                                                                                              	|
--- 

### Service Components
| Component name                   	| Type 	| Topic 	| Description                                                                                                                                                                                                 	|
|----------------------------------	|------	|-------	|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------	|
| InputDeviceComponent                	|    tod_operator_msgs::srv::InputDevice|     /InputDevices/InputDevice/change_input_device   	| Change the input device drivers                                                                                                                                         	|
| PacketCaptureComponent              	|    tod_network_monitoring_msgs::srv::PacketCaptureService|    ServiceForwarder/NetworkMonitoring/PacketLogger/set_capture_status   	| requests to enable or disable packet capture on both operator and vehicle nodes                                                                                                                                        	|
| VideoConfigComponent        	|    tod_config_msgs::srv::VideoConfig|    /Operator/Network/Config/ToVehicle/VideoConfig  	| reqfor handling video configuration requests                                                                                                                                        	|
--- 

### Publishing Components
| Component name                   	| Type 	| Topic 	| Description                                                                                                                                                                                                 	|
|----------------------------------	|------	|-------	|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------	|
| ManagerButtonStatusComponent               	|    tod_status_msgs::msg::ManagerButtonStatus|     output/button_status 	|  to communicate in the manager to the state machine which buttons have been clicked by the operator                                                                                                                                       	|                                                                                                                            
--- 

### DataContainer

| Container Type  	| Description                                                                                                      	|
|-----------------	|------------------------------------------------------------------------------------------------------------------	|
| Vertex          	| Information about the texture position and the color                                                             	|
| Texture         	| Wrapper for OpenGl texture, Id, Width, Height, Type, internal format and general format                          	|
| Buffer          	| Wrapper for OpenGl Buffer                                                                                        	|
| Mesh            	| Container for mesh information, i.e. Vertices, Indexes,Textures, VertexBuffer, IndexBuffer and VertexArrayObject 	|
| Character       	| Container to render text on the screen see `Display` entity                                                      	|


### Camera Controls for the Scene

| Key/Mouse Input | Movement Description | Additional Notes |
|-----------------|---------------------|------------------|
| Tab | Switches between Normal and Top view | - |
| O | Returns camera to car position | Based on current view mode |
| P | Prints camera debug information | Shows position, lookAt, up vector etc. |
| U | Toggles camera movement capability | Enables/disables camera controls |
| Arrow Up/Down | Moves camera forward/backward | Uses X-axis as forward |
| Arrow Left/Right | Moves camera left/right | Strafes relative to view direction |
| I/K | Pitches camera forward/backward | Rotates around right axis |
| J/L | Rotates camera around lookAt point | Changes view angle horizontally |
| N/M | Rotates camera position | Steps of 22.5° around up axis |
| Page Up/Down | Moves camera up/down | Vertical movement on Z axis |
| Left Mouse + Drag | Pans camera | Moves parallel to view plane |
| Left Mouse + Shift + Drag | Forward/backward movement | Moves along view direction |
| Right Mouse + Drag | Orbital rotation | Rotates around orbit point |
| Middle Mouse + Drag | Moves orbit point | Adjusts center of rotation |
| Mouse Scroll | Zoom in/out | Limited between 0.1 and 100 units |
| Left Mouse + Drag (Top View) | Translates camera in top view | Special movement for top-down perspective |





