echo "Building ROS nodes"

cd ROS/
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j