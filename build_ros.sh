echo "Building ROS nodes"

cd ROS/curly_state_estimator
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release -Wno-dev
make -j