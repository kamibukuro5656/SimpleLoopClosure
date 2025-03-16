## Parameters
- mapped_cloud  
Set to "true" if the scan data to subscribe has already been mapped to the same coordinate system as the odometry.  
- time_stamp_tolerance  
Unit : sec  
Tolerance for odometry and scan data timestamps.  
- keyframe_dist_th  
Unit : m  
A frame is registered when the distance from the last registered frame is greater than this value.  
- keyframe_angular_dist_th  
Unit : rad  
A frame is registered when the angular distance from the last registered frame is greater than this value.  
- loop_search_time_diff_th  
Unit : sec  
Nodes with a time difference less than this value are excluded from the search.
- loop_search_dist_diff_th  
Unit : m  
Nodes with a trajectory distance difference less than this value are excluded from the search.  
- loop_search_angular_dist_th  
Unit : rad  
Nodes with angular distances greater than this value are excluded from the search.  
This value is provided because LiDAR with limited FoV often fails in registration when the difference in angles is large.  
- loop_search_frame_interval  
Unit : frame  
If there are multiple frames in the buffer, the loop search is skipped at this interval.  
- search_radius  
Unit : m  
Search radius.  
- target_frame_num  
Unit : frame  
Create a submap for registration by merging the frames before and after this value around the frame resulting from the radius search.  
- target_voxel_leaf_size  
Unit : m  
Leaf Size of Voxel Grid Filter for submap.  
- source_voxel_leaf_size  
Unit : m  
Leaf Size of Voxel Grid Filter for scan frames.  
- vis_map_voxel_leaf_size  
Unit : m  
Leaf Size of Voxel Grid Filter for point cloud map to be published to ROS.  
It affects only the point cloud to be Publish.  
- fitness_score_th  
If the Fitness Score of the ICP is below this value, the loop edge is registered.  
- vis_map_cloud_frame_interval  
Skip nodes at this interval when creating a point cloud map to publish.
It affects only the point cloud to be Publish.  
