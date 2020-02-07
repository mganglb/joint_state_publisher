
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory # from launch_ros.substitutions import FindPackageShare

from joint_state_publisher.common_roslaunch import create_params_jsp, get_prefix_jsp

from moya.common_roslaunch import create_common_params_node_adv, add_action_list_to_launch_description, create_conditioned_node
from moya.common_roslaunch import param_dict_to_param_list, create_common_node_argument_list, add_prefix, get_common_dirs
from moya.common_ros2 import xacro_to_urdf 

def generate_launch_description():
    ld = LaunchDescription()

    pkg_dir_jsp, launch_dir_jsp, param_dir_jsp, rviz_dir_jsp, data_dir_jsp, urdf_dir_jsp = get_common_dirs('joint_state_publisher')

    # common defaults jsp
    p_use_node_jsp = 'True'                     
    p_namespace_jsp = ''
    p_use_namespace_jsp = 'False'
    p_use_sim_time_jsp = 'False'
    p_nodename_jsp = 'joint_state_publisher'
    p_use_nodename_jsp = 'True' 
    p_logging_level = 'info'
    prefix_jsp = get_prefix_jsp()                
    
    # defaults variables jsp 
    p_rate_jsp  = 10
    p_publish_default_positions_jsp  = 'True'
    p_publish_default_velocities_jsp  = 'False'
    p_publish_default_efforts_jsp  = 'False'
    p_use_mimic_tags_jsp  = 'True'
    p_use_smallest_joint_limits_jsp  = 'True'
    p_source_list_jsp  = ['']                        # [] is not valid , chnanaged code [] -> [''] or ['foo]
    p_delta_jsp  = '0.0'
    p_zeros_jsp  = None                              # not implemented
    p_dependent_joints_jsp  = None                   # not implemented


    # LaunchConfiguration common jsp
    common_param_dict_jsp , common_action_list_jsp = create_common_params_node_adv(p_use_node = p_use_node_jsp, 
        p_nodename = p_nodename_jsp, p_use_nodename = p_use_nodename_jsp,
        p_namespace = p_namespace_jsp,  p_use_namespace = p_use_namespace_jsp,
        p_use_sim_time = p_use_sim_time_jsp, p_logging_level = p_logging_level,
        prefix = prefix_jsp)
    add_action_list_to_launch_description(ld, common_action_list_jsp)
    
    # LaunchConfiguration jsp  
    param_dict_jsp, action_list_jsp = create_params_jsp(p_rate = p_rate_jsp, 
                        p_publish_default_positions = p_publish_default_positions_jsp, 
                        p_publish_default_velocities = p_publish_default_velocities_jsp,
                        p_publish_default_efforts = p_publish_default_efforts_jsp, 
                        p_use_mimic_tags = p_use_mimic_tags_jsp, 
                        p_use_smallest_joint_limits = p_use_smallest_joint_limits_jsp, 
                        p_source_list = p_source_list_jsp, 
                        p_delta = p_delta_jsp, p_zeros = p_zeros_jsp, 
                        p_dependent_joints = p_dependent_joints_jsp,
                        prefix = prefix_jsp)
    add_action_list_to_launch_description(ld, action_list_jsp)

    # jsp Node
    arguments_list_jsp = create_common_node_argument_list(common_param_dict_jsp[add_prefix(prefix_jsp, "logging_level")])
    param_list_jsp = param_dict_to_param_list({**common_param_dict_jsp, **param_dict_jsp}, prefix_jsp)
    remappings_jsp = []

    node_list_jsp = create_conditioned_node('joint_state_publisher',
                                            'joint_state_publisher',
                                            common_param_dict_jsp[add_prefix(prefix_jsp, "nodename")],
                                            common_param_dict_jsp[add_prefix(prefix_jsp, "namespace")],
                                            parameters = param_list_jsp,
                                            arguments = arguments_list_jsp,
                                            remappings = remappings_jsp, 
                                            condition_use_namespace = common_param_dict_jsp[add_prefix(prefix_jsp, "use_namespace")],
                                            condition_use_node = common_param_dict_jsp[add_prefix(prefix_jsp, "use_node")],
                                            condition_use_nodename = common_param_dict_jsp[add_prefix(prefix_jsp, "use_nodename")])

    add_action_list_to_launch_description(ld, node_list_jsp)

    return ld