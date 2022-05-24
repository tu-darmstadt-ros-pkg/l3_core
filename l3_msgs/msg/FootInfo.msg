uint32 idx                                # foot id
string name                               # name of foot
geometry_msgs/Vector3 size                # size of foot
string link                               # frame in which the foot pose is expressed
geometry_msgs/Pose link_to_sole_offset    # position of planner foot frame (center of sole) given in robot's "foot" link frame
geometry_msgs/Pose neutral_stance         # neutral stance given as array of poses relative to robot's center

bool indirect                             # Indicates the foothold represents the pose of a indirectly determined link such as the base instead of a foot.
                                          # Indirect footholds do not require to maintain contact with the environment and thus will not be created by default during state generation.

uint32 CUBOID       = 0
uint32 SPHERICAL    = 1
uint32 shape                              # shape of foot

std_msgs/ColorRGBA color                  # color of foot used for visualization
string mesh_resource                      # path to mesh used for visualization
geometry_msgs/Pose mesh_offset            # mesh offset relative to foot sole
geometry_msgs/Vector3 mesh_scale          # scale of mesh for correct visualization
