# Getting started with CARLA in conjunction with the tod software 

## 1 - Why you want to use CARLA and vEDGAR 
You are a (phd) student and want to work with the vEDGAR (vitual EDGAR) in the Carla simulation. 
The following can be done using the vEDGAR:
- Retrieving sensor information (Camera, LiDAR, Odometry and many more) as well as influencing the vehicle's motion through control inputs. The vEDGAR provides the same interfaces as our EDGAR research vehicle and is therefore highly suited for developing, debugging and benchmarking new functions related to teleoperation and automated driving
- Additionally, retrieving ground truth data such as object lists or predicted paths directly from the simulation is possible using vEDGAR
- Through integration with Carla, vEDGAR provides many possibilities to integrate your own maps, spawn other traffic participants, simulate diverse weather conditions and much more.

## 2 - Please read: Important ressources
Carla is a very well documented simulation tool.
If you have any questions regarding this tool, please refer to the [Online Documentation](https://carla.readthedocs.io/en/latest/start_introduction/).
Especially read the sections on the introduction and the Python API.
The Python API is a very useful tool that allows us to retrieve simulation data and manipulate the simulation using rather simple Python scripts.
In fact, much of how data is retrieved and the vEDGAR is manipulated, is handled using the Python API.

If you are working with the chairs hardware, you can skip the sites on Carla installation since you are provided with a Docker image that includes a prebuild version of docker that already includes a EDGAR vehicle model and some maps from Munich.
If you don't know Docker, reading the related docs online and our [Docker workflow](https://gitlab.lrz.de/teleoperiertes_fahren/tod/-/blob/develop2/doc/docker_workflow.md?ref_type%253Dheads) might be useful.
Even if you don't work with the chairs hardware, adopting a docker workflow is highly recommended since it prevents problems arising from dependencies and makes distributing the fancy functionality you just implemented much easier.

Please also note the other ressourced related to Carla which include among others
- The [Carla website](https://carla.org//) and
- The [Carla Youtube Channel](https://www.youtube.com/%2540carlasimulator8782)

## 3 - Getting started

Now that you know some of the basics of Carla and are determined to tackle your (pdh) thesis, you should undergo the following steps to run the vEDGAR on your machine.
Using a computer with a dedicated and capable graphics card is highly recommended.
If you don't have a computer with a dedicated graphics card at hand, asking your thesis supervisor is useful.

- Since the vEDGAR is part of the EDGAR project, you find it in the [vEDGAR repository](). You can clone it into your home folder via
  ```bash 
  cd ~ && git clone https://gitlab.lrz.de/av2.0/edgar_digital_twin.git
   ```
- Change into the cloned repository
   ```bash
   cd edgar_digital_twin
   ```
- Check out the latest branch which is currently (July 2024) 'hil_mwp_milesone'
    ```bash
   git checkout hil_mwp_milestone
   ```
    Don't hesitate to ask your supervisor if there are any new features on other branches if you are not told which version to work with
- You will notice that you won't find vEDGAR right away. This is due to the fact that the repository contains further tools. To get to vEDGAR, you will go further down the folder you just cloned using
   ```bash
   cd tools/CARLA
   ```
- Now you are in the folder that contains all code and scripts needed to run vEDGAR. For your convenience, you only have two run two scripts to startup vEDGAR:
    - 0: If a specific **ROS_DOMAIN_ID** is required, we need to set it first in the docker file. The TOD code usually uses the **ROS_DOMAIN_ID=7**. To provide the ros2 topics in this domain id, open the EdgarRosInterface.Dockerfile by running
        ```bash
        cd docker
        gedit EdgarRosInterface.Dockerfile
        ```
        Inside the file, add the following line after line 6 (```ENV CYCLONEDDS_URI```):
        ```
        ENV ROS_DOMAIN_ID=7
        ```
    - 1: Build the docker image using the build_docker.sh script
        ```bash
        ./build_docker.sh
        ```
        If you have not downloaded the av2.0 Carla docker image already (i.e. because you are running this script the first time), this step might take some time because the docker image has to be downloaded first.
    - 2: Run vEDGAR using the 'run_vedgar.sh' bash script. Beforehand, check if anyting is commented out in this script using 
        ```bash
        cat run_vedgar.sh
        ```
        If the first line containing the docker image 'gitlab.lrz.de:5005/av2.0/carla/*' is commented out and the docker container is not yet running (you can check this using the 'docker ps' command which lists all running containers), you have to uncomment it to start the docker container which containes the Carla simulator. **Please don't forget** to comment this line out once the container is running to prevent errors.

        If you checked this, run the Carla docker and ROS2 interface containers using
        ```bash
        ./run_vedgar.sh
        ```

    - 3: Check if everyting is running as expected. 
        When using 'docker ps' in a seperate terminal window, you should see that two docker containers are running. One container comprises the Carla simulator while the other contains a Carla-to-ROS2 interface that allows us to retrieve all sensor information needed for teleoperation and AV reseach as well as to manipulate the vehicle's motion through control inputs. 

        Now you should be able to see all relevant ROS2 topics running
        ```bash
        ros2 topic list
        ```
        in a seperate termial window.

        **Important**: if you defined a specific ROS_DOMAIN_ID previously, you have to make shure that it also is used in the terminal where you check the topics. You can set it using
        ```bash
        export ROS_DOMAIN_ID=7
        ```

        **Important:** Every time you change something in the Python code related to vEDGAR, you have to rebuild 

## 4 - Developing with Carla
TODO
        
