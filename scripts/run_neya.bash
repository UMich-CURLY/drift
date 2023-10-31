# ----------------------------------------------------------------------------
# Copyright 2023, CURLY Lab, University of Michigan
# All Rights Reserved
# See LICENSE for the license information
# -------------------------------------------------------------------------- */

#
#  @file   run_neya.bash
#  @author Tzu-Yuan Lin
#  @brief  bash file to run the neya dataset (full-size vehicle)
#  @date   August 7, 2023


## This file requires yq to be installed: https://github.com/mikefarah/yq
## This can be down by `brew install yq` on mac or `sudo snap install yq` on linux
data_path="/media/justin/DATA/data/neya/"
output_path="/media/justin/DATA/result/DRIFT_TRO/neya"
config_path="../config/neya/"
rosnode_name="neya"
sleep_time=0.5

GREEN='\033[0;32m'
ORANGE='\033[0;33m'
NC='\033[0m' # No Color

echo -e "${ORANGE} Start running neya dataset!! All the best!! ^_^ ${NC}"
for subfolder in paintball pelenor; do
  for bag in $data_path$subfolder"/"*.bag; do
    
    # find the depth of the directory
    dir_depth=$(echo $bag | tr "/" "\n" | wc -l)

    # find the filename
    filename=$(echo $bag | cut  -d"/" -f$dir_depth)

    # remove time and general information in the filename
    num_underscore=$(echo $filename | grep -o "_" | wc -l)
    output_folder=$(echo $filename | cut  -d"_" -f1-$(($num_underscore-2)) | rev |rev)
    

    
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

    rosrun drift neya & 
    
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
done