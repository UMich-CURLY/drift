private:
rclcpp::Node::SharedPtr nh_; // ROS2 node handle
RobotStateQueuePtr
robot_state_queue_ptr_; // Pointer to the robot state queue
std::shared_ptrstd::mutex
robot_state_queue_mutex_; // Pointer to the robot state queue mutex

bool thread_started_; // Flag for thread started

rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_; // Pose publisher
std::string
pose_frame_; // The name ofF a frame which poses are published in
uint32_t pose_seq_ = 0; // Sequence number for pose publisher
double pose_publish_rate_; // Pose publishing rate
std::thread pose_publishing_thread_; // Thread for pose publishing

rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; // Path publisher
uint32_t path_seq_ = 0; // Sequence number for path publisher
double path_publish_rate_; // Path publishing rate
std::thread path_publishing_thread_; // Thread for path publishing

int pose_skip_; // Number of poses to skip while generating a path message

std::array<float, 3> first_pose_; // Initial pose of the robot
std::vector<geometry_msgs::msg::PoseStamped>
poses_; // Path message in ROS2, a list of poses
std::mutex poses_mutex_; // mutex for the path