echo "Building ROS nodes"

cd ROS/
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
# run: export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/TO/curly_state_estimator/ROS