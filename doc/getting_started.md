# Detailed Instructions to Build, Run and Develop this Software

This repository contains the documentation, standard parametrization, docker workflow and subrepository setup scripts. Prerequisits for using the tod code are:
- docker compose (for docker workflow, installation see [here (external link)](https://docs.docker.com/engine/install/ubuntu/))
- vcstool to clone subrepository with the actual modules (install via `sudo apt install python3-vcstool`)
- Optional (if you want to run the code withoud docker): ROS2 Humble (other ros2 versions were not tested, installation see [here (external link)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))

## Repository Setup
After cloning the tod repository, follow these steps to setup the repository

1. Setup repositories using *setup_repos.sh* script
   ```bash
   cd tod
   chmod +x setup_repos.sh && ./setup_repos.sh
   ```

## Run the Software in Docker (Recommended)

After setting up the repository, follow these steps to run and test the softoware:
1. Build the deployment images
    ```bash
    cd tod
    docker compose build tod_vehicle tod_operator
    ```
2. Setup the environment variables; open the ```.env``` file in the ```tod``` directory and adjust the variables, recommended changes for local testing:
     - ```DOCKER_CYCLONEDDS_CONFIG=unconfigured```
     - ```MODE=only_sim```
3. Run the deploy docker
    ```bash
    cd tod
    docker compose up tod_vehicle tod_operator
    ```
4. Test the teleoperation software
   1. Open the Manager Window
   2. Select localhost: ``IP Address Vehicle: 127.0.0.1`` and ``IP Address Operator: 127.0.0.1``
   3. Click ``Connect``
   4. Select the teleoperation concept of your choice (``Direct Control`` or ``Waypoint Guidance``)
   5. For ```Direct Control``` click ```Browse``` and select ``virual.yaml``
   6. Click ``Start``
   7. Play around
      - ```Direct Control```
        - In the virtual input device window: 
          - Shift gear up/down using the keys `T` and `G`. (Works only when desired velocity is zero)
          - Increase/decrease desired velocity using the keys `W` and `S`. (Works only the gear position is not in park)
          - Increase/decrease the desired velocity dragging the grey button up and down.
          - To steer the vehicle, click, hold and drag the grey button left and right.
        - In the visual window, see how the desired command values of the display change. If desired velocity is greater zero, the vehicle also starts moving. 
      - ```Trajectory Guidance```
        - select the trajectory planning icon in the right bottom of the visual window
        - plan a trajectory with the left mouse click, `Delete` removes the last point
        - Send the trajectory to the vehicle with `Enter`
        - After receiving the simulated trajectory of the vehicle, a second trajectory in orange should be displayed in the visual window
        - Start the trajectory execution by validating the simulated trajectory using `V`
        - If you want the vehicle to stop, press `space`

## Develop in the Software using Docker (Recommended)
After setting up the repository, follow these steps to develop new software in the docker environment:
1. Build the development image
    ```bash
    cd tod
    docker compose build tod_dev
    ```
2. Setup the environment variables; open the ```.env``` file in the ```tod``` directory and adjust the variables, recommended changes for local testing:
   - ```DOCKER_CYCLONEDDS_CONFIG=unconfigured```
3. Run the development docker
   ```bash
    cd tod
    docker compose up tod_dev
    ```
4. Enter the development docker (if you are using VS Code you can skip this and continue with Step 5)
   ```docker exec -it tod_dev_latest bash```
5. (Optional) Attach to the development docker using VS Code
   - Open VSCode and click ```Open a Remote Window``` (blue button on the bottom left)
   - Select ```Attach to Running Container``` and select ```tod_dev_latest```
6. Setup the parameters, open the ```/home/tum/wsp/config/config/launch_setup.yaml``` file; recommended changes for local development:
   - ```mode: ònly_sim```
7. Build the software
    ```bash
     colcon build                                           # build everything
     colcon build --cmake-args -DVEHICLE=ON -DOPERATOR=OFF  # build just the software for the vehicle side
     colcon build --cmake-args -DVEHICLE=OFF -DOPERATOR=ON  # build just the software for the operator side
    ```
8. Launch the software (in the develoment docker, see step 4)
    ```bash
    source install/setup.bash && ros2 launch tod_launch tod_both.launch.py      # launch operator and vehicle side of the software
    source install/setup.bash && ros2 launch tod_launch tod_vehicle.launch.py   # launch the vehicle side of the software
    source install/setup.bash && ros2 launch tod_launch tod_operator.launch.py  # launch the operator side of the software
    ```

## Run the Software locally
After setting up the repository, follow these steps to develop new software in the docker environment:
1. Install all external dependencies (please refer to ```docker/dockerfile```to see all dependencies)
2. Install all ros dependencies
   ```bash
   cd tod
   rosdep update && rosdep install --from-paths src --ignore-src --rosdistro humble -y
   ```
3. Follow steps 6-7 from above

## Configure the Launch of the Software
Each node defines all modifiable parameters as ROS2 parameters. These can be set via the ROS2 params files in the config folder of this package.
To paraemtrize your tod_code, the following guideline might help you:
  * `config/config/package_config/<package_name>` contains one or multiple parameter files as `.yaml`. Changed parameters in this file will be considered at launch
  * `config/config/vehicle_config/<platform_name>` contains the parameter files for each platform, ranging from sensor to transform configurations. The folder name corresponds to the later vehicleID
  * `config/config/launch_setup.yaml` defines the packages launched at operator or vehicle side via bools
  * `config/config/remappings.yaml` creates necessary remappings between the individual ros2 packages to create one global tod_system

The launch mentioned above can be configured as required.
  * `vehicleID`: Identifier (string) of the vehicle that is being teleoperated. The respective folder and all configs must exist in `config/config/vehicle_config/<platform_name>`
    * `edgar`: platform edgar without additional automation systems
    * `edgarautoware`: platform edgar for simultaneous usage with autoware, uses autoware interfaces
    * `rc-car`: f1tenth research platform with the scale 1:10
  * `mode`: Mode (string) how the software should be launched (relevant for vehicle side only).
    * `vehicle`: To teleoperate an actual vehicle. Launches all drivers and vehicle hardware interfaces as specified in the bridge package of the respective `vehicleID`. 
    * `only_sim`: Launches only the vehicle simulation node.

Remember to build the tod_launch package after update any configs if you are not running in the docker environment (```colcon build --packages-select tod_launch```).

