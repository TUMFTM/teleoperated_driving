# docker_tod

Docker images to build and run the software are prepared. Follow the instructions below to get started. The docker images contain the dependencies that are necessary for the build and run process.
The following images are provided.

* `tod-base` (based on `melodic-ros-base`)
* `tod-operator-base` (based on `tod-base`)
* `tod-vehicle-base` (based on `tod-base`)

## Prerequisites: Docker Installation

* Install and start docker service

  ```bash
  sudo apt-get install containerd docker.io
  sudo systemctl start docker
  ```

* If starting docker fails with "Failed to start docker.service: Unit is masked.", run

  ```bash
  sudo systemctl unmask docker
  ```

* Enable docker service for being started when rebooting the system

  ```bash
  sudo systemctl enable docker
  ```

* Add user to docker group

  ```bash
  sudo groupadd docker
  sudo usermod -aG docker $USER
  ```

* Log out user and log back in

## Getting started

### Using pre-built images from registry

* Pull the pre-built image from the registry:

  ```bash
  docker pull johfei/tod-operator-base:latest
  docker pull johfei/tod-vehicle-base:latest
  ```

* Rename docker images respectively

  ```bash
  docker image tag johfei/tod-operator-base:latest tod-operator-base:latest
  docker image tag johfei/tod-vehicle-base:latest tod-vehicle-base:latest
  ```

### Build images manually

In case, you do not want to download and run the provided container, you can build the images with the following command:

* Run from this directory (replace `tod-base` with `tod-operator-base` or `tod-vehicle-base` for respective images). Build `tod-base` first.

  ```bash
  docker build . -t tod-base:latest -f tod-base/Dockerfile
  ```

### Build and start software in container

* Run a docker container

  ```bash
  ./run_docker.bash  -i DOCKER_IMAGE:TAG -n CONTAINER_NAME -w WORKING_DIR
  ```

  * `DOCKER_IMAGE:TAG`: choose a desired image e.g., `tod-operator-base:latest`
  * `CONTAINER_NAME`: specify a desired container name e.g., `test`
  * `WORKING_DIR`: specify the directory of the ros workspace e.g., `~/teleoperated_driving/wsp`

* Attach to running container

  ```bash
  docker exec -it CONTAINER_NAME /bin/bash
  ```

* Compile the software stack using the container. Use build flags appropriate to respective container.

  `tod-operator-base`:

    ```bash
    catkin build -DOPERATOR:=ON -DVEHICLE:=OFF
    ```

  `tod-vehicle-base`:

    ```bash
    catkin build -DOPERATOR:=OFF -DVEHICLE:=ON
    ```

* Launch software inside container. Extend workspace if necessary

  `tod-operator-base`:

    ```bash
    source devel/setup.bash && roslaunch tod_launch operator.launch
    ```

  `tod-vehicle-base`:

    ```bash
    source devel/setup.bash && roslaunch tod_launch vehicle.launch
    ```
