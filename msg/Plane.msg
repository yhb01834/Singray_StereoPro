# Frame this data is associated with
string frame_id

# Polygon describing the actual detected area
geometry_msgs/Point[] points

# Plane normal
geometry_msgs/Vector3 normal

# Plane signed distance to world frame origin (according to normal direction)
float64 d

# Plane unique identifier
string id
