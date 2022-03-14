# CAIRA
The Context-Aware Intention Recognition Agent

## Creating the Docker containers
To create the rasberry docker container, you need to place
 - libcudnn7_7.4.2.24-1+cuda9.0_amd64.deb
 - libcudnn7-dev_7.4.2.24-1+cuda9.0_amd64.deb
 in `docker/rasberry/. You can get them from NVIDIA.

 ## Running CAIRA in Simulation
 0.1 Copy the contents of the `ros_node` directory into `/home/rasberry/catkin_ws/src/`
 0.2 Build the catkin workspace `cd /home/rasberry/catking_ws;catkin build`
 1. Start the simulator `tmule -c /home/rasberry/catking_ws/src/RASberry/rasberry_bringup/tmule/rasberry-multisim.yaml`
 2. Enter the simutation environment `tmule a`
 3. Switch to window 0 `CTRL+B 0`
 4. Start the experiment launch file `roslaunch rasberry_hri experiment.launch`


 ## Running CAIRA on a robot
This is too involved to describe here.
Get in touch and we will help you to get CAIRA integrated into your setup.
