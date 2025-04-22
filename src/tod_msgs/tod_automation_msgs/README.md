# The **tod_automation_msgs** Package

This package contains message definitions relevant for the sharing data with an automation software via the vehicle interface annd/or processing the data further in different control concepts or safety concepts.

The package holds different messages to carry information from the automation:

| Message                 | Purpose                                                             
| --------                | -------                                                             |
| `ObjectData`            | Information about position and motion of a detected object          |
| `ObjectList`            | List of Objects                                                     |
| `PredictedObject`       | Infomration about position and classificataion of a predicted object    |
| `PredictedObjects`      | List of Predicted Objects                                           |
| `Trajectory`            | List of TrajectoryPoints and the their frame                        |
| `TrajectoryPoint`       | Pose and Twist a Point to construct trajestories                    |
| `VehicleAutomationState`| encoded automation status                                           |

## Additional Remarks
Building the packages containing messages and services is done via ```ros2 build tod_automation_msgs```.