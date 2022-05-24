uint32 idx                                  # base id
string name                                 # name of base
geometry_msgs/Vector3 size                  # bounding box of upper body
string link                                 # fixed tf frame in which the upper body pose is expressed
geometry_msgs/Pose link_to_center_offset    # offset of box relative to robot center

uint32 CUBOID       = 0
uint32 SPHERICAL    = 1
uint32 shape                                # shape of base

std_msgs/ColorRGBA color                    # color of foot used for visualization
string mesh_resource                        # path to mesh used for visualization
geometry_msgs/Pose mesh_offset              # mesh offset relative to link
geometry_msgs/Vector3 mesh_scale            # scale of mesh for correct visualization
