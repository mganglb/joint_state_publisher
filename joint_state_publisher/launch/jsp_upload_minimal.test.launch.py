import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory # from launch_ros.substitutions import FindPackageShare

from joint_state_publisher.common_roslaunch import create_params_jsp, get_prefix_jsp

from moya.common_roslaunch import create_common_params_node_adv, create_common_params_group
from moya.common_roslaunch import add_action_list_to_launch_description, get_common_dirs

def generate_launch_description():
    ld = LaunchDescription()

    pkg_dir_jsp, launch_dir_jsp, param_dir_jsp, rviz_dir_jsp, data_dir_jsp, urdf_dir_jsp = get_common_dirs('joint_state_publisher')
    launchfile_jsp = os.path.join(launch_dir_jsp, 'jsp_upload.launch.py') 

    # common group defaults
    p_namespace = ""
    p_use_namespace = 'False'

    # LaunchConfiguration jsp  
    param_dict_jsp, action_list_jsp = create_params_jsp()
    add_action_list_to_launch_description(ld, action_list_jsp)

    # jsp Node
    push_ns = PushRosNamespace(
        condition=IfCondition(common_param_dict_group_jsp["use_namespace"]),
        namespace=common_param_dict_group_jsp["namespace"])

    jsp_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launchfile_jsp),
        launch_arguments={**common_param_dict_jsp, **param_dict_jsp}.items())

    jsp_group = GroupAction([push_ns, jsp_ld])
    ld.add_action(jsp_group)

    return ld