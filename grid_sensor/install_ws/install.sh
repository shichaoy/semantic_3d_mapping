sudo apt-get install python-wstool # get wstool
source /opt/ros/indigo/setup.bash # init environment
mkdir -p 10708_ws/src
cd 10708_ws/src
mkdir -p dependency  #store some unused packages
catkin_init_workspace
git clone -b sb_grid_label git@bitbucket.org:castacks/rgbd_grid_sensor.git
ln -s rgbd_grid_sensor/install_ws/crf_3d.rosinstall .rosinstall
wstool info # see status
wstool up # get stuff

# compile
cd ..
source devel/setup.bash
catkin_make


# run
# NOTE change the path of dataset in the launch file
roslaunch rgbd_grid_sensor  rgbd_grid_imgs_rviz.launch
