string feature_type  # Identifies the type (e.g., "PointCloud", "Transform") Can be use to differentiate features of same kind (e.g., point cloud for thermal and RGB cameras)
sensor_msgs/PointCloud2 point_cloud  # Stores point cloud data
geometry_msgs/TransformStamped transform  # Stores a transform
bool is_point_cloud  # Flags indicating which data is used
bool is_transform