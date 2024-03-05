# ros-technical-test

## Objectives of the Technical Test

- Object-oriented Python programming;
- Use of ROS (Robot Operating System);
- Use of Docker.

The provided archive is a lab assignment undertaken by students at Polytech as part of the experimental robotics module. This assignment specifically focuses on modeling a 6-DOF (Degrees of Freedom) manipulator with a wrist with concurrent axes. During this lab, students:

1. Perform geometric parameterization of the robot using the modified Denavit-Hartenberg convention (Khalil-Kleinfinger);
2. Implement direct and inverse geometric models as well as the kinematic model;
3. Control the robot using ROS tools;
4. Generate trajectory and interface with ROS.

While this lab assignment is extensive, it contains crucial elements for the position (Docker, ROS, Python), making it suitable as a technical test basis. We will only focus on:

1. Implementation of the direct geometric model;
2. ROS interfacing;
3. Docker usage.

## Using the Docker Container

1. If not already done, install Docker:
   
   - On Ubuntu:
     1. Docker Engine (avoid Docker Desktop): [Installation Guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
     2. Follow post-installation configuration: [Post-installation Guide](https://docs.docker.com/engine/install/linux-postinstall/)
   - On Windows:
     1. Install WSL2
     2. Install Docker Desktop: [Windows Installation Guide](https://docs.docker.com/desktop/install/windows-install/)

2. Clone the repository:
```bash
$ cd $ROS_TEST_DIR
$ git clone https://gitlab.isir.upmc.fr/eurobin/ros-technical-test.git
```

3. Build the container:
```bash
$ cd $ROS_TEST_DIR/ros-technical-test
$ docker build --tag tp_fanuc_docker .
```

4. Launch the container:
```bash
$ cd $ROS_TEST_DIR/ros-technical-test/docker
$ ./start_docker.sh # on Ubuntu
$ sh start_docker_wsl.sh # on WSL
```

Then access the container from other terminals:
```bash
$ docker exec -it tp_fanuc zsh
```

5. Once inside the docker, use the lab assignment. You'll directly land in `catkin_ws`:

Compile, source, and other instructions are in the lab assignment. However, you can already test the general launch file.
```bash
$ catkin_make    # compile
$ chmod +x src/robotique_experimentale/tp_fanuc/scripts/* # make scripts executable
$ source devel/setup.zsh
$ roslaunch tp_fanuc tp_DHm.launch
```

## Test Contents

1. Code the missing methods in the direct geometric model (`mgd.py`): `compute_Ti(self,dh,q):`, `compute_T(self,Q,i,j)` and `compute_robot_state(self,Q)`.

The homogeneous transformation matrix obtained from the Denavit-Hartenberg parameters is as follows:

$$
^{i-1}T_{i} = 
\left[ \begin{array}{cccc}
\cos\theta_i 	& -\sin\theta_i 	& 0 & a_{i-1} \\
\cos\alpha_{i-1} \cdot \sin\theta_i & \cos\alpha_{i-1} \cdot \cos\theta_i & -\sin\alpha_{i-1} & -d \cdot \sin\alpha_{i-1} \\
\sin\alpha_{i-1} \cdot \sin\theta_i & \sin\alpha_{i-1} \cdot \cos\theta_i & \cos\alpha_{i-1} & d \cdot \cos\alpha_{i-1} \\
0 & 0 & 0 & 1
\end{array}\right]
$$

2. Code the node `traj_arti` which allows for interpolation of articulatory trajectory between two positions and publishes commands on the `/joints_state` topic.

3. Create a merge request with the code + markdown documentation for its usage.