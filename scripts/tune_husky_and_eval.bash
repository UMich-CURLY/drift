# ----------------------------------------------------------------------------
# Copyright 2023, CURLY Lab, University of Michigan
# All Rights Reserved
# See LICENSE for the license information
# -------------------------------------------------------------------------- */

#
#  @file   run_husky.bash
#  @author Tzu-Yuan Lin
#  @brief  bash file to run the neya dataset (full-size vehicle)
#  @date   August 7, 2023


## This file requires yq to be installed: https://github.com/mikefarah/yq
## This can be down by `brew install yq` on mac or `sudo snap install yq` on linux
data_path="/media/justin/DATA/data/husky_data/2022-05-11_MAir/"
output_path="/media/justin/DATA/result/DRIFT_TRO/husky/"
config_path="../config/husky/"
rosnode_name="husky"
sleep_time=0.5

GREEN='\033[0;32m'
ORANGE='\033[0;33m'
NC='\033[0m' # No Color

echo -e "${ORANGE} Start running the husky data set!! All the best!! ^_^ ${NC}"
for g_std in 0.01 0.05 0.1 0.5 0.8; do
for v_std in 0.01 0.05 0.1 0.5 0.8; do
for a_std in 0.01 0.05 0.1 0.5 0.8; do
  
  for bag in $data_path"/"*.bag; do
    echo -e "${GREEN}===================================================="
    echo -e "${GREEN} Running param: g: $g_std v: $v_std a: $a_std ${NC}"
    # find the depth of the directory
    dir_depth=$(echo $bag | tr "/" "\n" | wc -l)

    # find the filename
    filename=$(echo $bag | cut  -d"/" -f$dir_depth)

    # remove time and general information in the filename
    output_folder=$(echo $filename | cut  -d "." -f1 | rev |rev)
    
    echo -e "${GREEN}----------------------------------------"
    echo -e "${GREEN} Processing sequence: $output_folder ${NC}"
    
    # get the full path of the output folder
    cur_output_path=$output_path"/"$output_folder"/"
    export cur_output_path

    if [ ! -d $cur_output_path ] 
    then
      mkdir $cur_output_path
    fi

    # modify the config file
    yq e -i '.logger.pose_log_file = strenv(cur_output_path)+"InEKF.txt"'  $config_path"inekf_estimator.yaml"
    yq e -i '.logger.vel_log_file = strenv(cur_output_path)+"InEKF_velocity.txt"'  $config_path"inekf_estimator.yaml"

    # modify param
    export g_std
    export v_std
    export a_std
    yq e -i '.noises.gyroscope_std = strenv(g_std)'  $config_path"imu_propagation.yaml"
    yq e -i '.noises.accelerometer_std = strenv(a_std)'  $config_path"imu_propagation.yaml"
    yq e -i '.noises.velocity_std = strenv(v_std)'  $config_path"velocity_correction.yaml"

    rosrun drift husky & 
    
    drift_pid=$!& 
    
    rosbag play $bag --clock -d $sleep_time & 
    
    rosbag_pid=$! 

    # Wait for the rosbag process to finish
    wait $rosbag_pid

    # stop drift
    # kill $drift_pid
    rosnode kill $rosnode_name

    echo -e "${GREEN} Finished running: $output_folder ${NC}"
    echo -e "${GREEN} Results saved to: $cur_output_path ${NC}"

    sleep $sleep_time
  done

  # evaluate
  bash /home/justin/code/plotting/drift_utility/evaluation_tools/evaluate_husky_one_method.bash\
   >> "/media/justin/DATA/result/DRIFT_TRO/husky/tuning/g_"$g_std"_a_"$a_std"_v_"$v_std".txt"

done
done
done