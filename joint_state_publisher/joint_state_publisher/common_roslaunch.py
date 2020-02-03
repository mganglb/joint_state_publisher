# Markus Ganglbauer

from moya.common_roslaunch import create_param
from moya.common_roslaunch import add_prefix

### JSP launch ###
def create_params_jsp(p_rate = 10, p_publish_default_positions = 'true', p_publish_default_velocities = 'false',
                        p_publish_default_efforts = 'false', p_use_mimic_tags = 'true', p_use_smallest_joint_limits = 'true', p_source_list = [''], 
                        p_delta = 0.0, p_zeros = None, p_dependent_joints = None, prefix = ''):
    d_rate = "rate (int) - The rate at which to publish updates to the /joint_states topic. Defaults to 10."
    d_publish_default_positions = "publish_default_positions (bool) - Whether to publish a default position for each movable joint to the /joint_states topic. Defaults to True."
    d_publish_default_velocities = "publish_default_velocities (bool) - Whether to publish a default velocity for each movable joint to the /joint_states topic. Defaults to False."
    d_publish_default_efforts= "publish_default_efforts (bool) - Whether to publish a default effort for each movable joint to the /joint_states topic. Defaults to False."
    d_use_mimic_tags = "use_mimic_tags (bool) - Whether to honor <mimic> tags in the URDF. Defaults to True."
    d_use_smallest_joint_limits = "use_smallest_joint_limits (bool) - Whether to honor <safety_controller> tags in the URDF. Defaults to True."
    d_source_list = "source_list (array of strings) - Each string in this array represents a topic name. For each string, create a subscription to the named topic of type sensor_msgs/msg/JointStates. Publication to that topic will update the joints named in the message. Defaults to an empty array."
    d_delta = "delta (double) - How much to automatically move joints during each iteration. Defaults to 0.0."
    # d_zeros = "zeros (dictionary of string -> float) - A dictionary of joint_names to initial starting values for the joint. In Eloquent, pass on the command-line as '-p zeros.joint_name:=value'. Defaults to an empty dictionary, in which case 0 is assumed as the zero for all joints."
    # d_dependent_joints = "dependent_joints (dictionary of string -> dictionary of 'parent', 'factor', 'offset') - A dictionary of joint_names to the joints that they mimic; compare to the <mimic> tag in URDF. A joint listed here will mimic the movements of the 'parent' joint, subject to the 'factor' and 'offset' provided. The 'parent' name must be provided, while the 'factor' and 'offset' parameters are optional (they default to 1.0 and 0.0, respectively). In Eloquent, pass on the command-line as '-p dependent_joints.left_leg.parent:=right_leg -p dependent_joints.left_leg.offset:=0.0 -p dependent_joints.left_leg.factor:=2.0'. Defaults to the empty dictionary, in which case only joints that are marked as <mimic> in the URDF are mimiced."

    rate, declare_rate_cmd = create_param('rate', p_rate, d_rate, prefix)
    publish_default_positions, declare_publish_default_positions_cmd = create_param('publish_default_positions', p_publish_default_positions, d_publish_default_positions, prefix)
    publish_default_velocities, declare_publish_default_velocities_cmd = create_param('publish_default_velocities', p_publish_default_velocities, d_publish_default_velocities, prefix)
    publish_default_efforts, declare_publish_default_efforts_cmd = create_param('publish_default_efforts', p_publish_default_efforts, d_publish_default_efforts, prefix)
    use_mimic_tags, declare_use_mimic_tags_cmd = create_param('use_mimic_tags', p_use_mimic_tags, d_use_mimic_tags, prefix)
    use_smallest_joint_limits, declare_use_smallest_joint_limits_cmd = create_param('use_smallest_joint_limits', p_use_smallest_joint_limits, d_use_smallest_joint_limits, prefix)
    source_list, declare_source_list_cmd = create_param('source_list', p_source_list, d_source_list, prefix)
    delta, declare_delta_cmd = create_param('delta', p_delta, d_delta, prefix)
    # robot_zeros_filepath, declare_zeros_filepath_cmd = create_param('zeros', p_zeros, d_zeros)
    # dependent_joints, declare_dependent_joints_cmd = create_param('dependent_joints', p_dependent_joints, d_dependent_joints)

    parmas_dict = {add_prefix(prefix,'rate') : rate,
                add_prefix(prefix,'publish_default_positions') : publish_default_positions,
                add_prefix(prefix,'publish_default_velocities') : publish_default_velocities,
                add_prefix(prefix,'publish_default_efforts') : publish_default_efforts,
                add_prefix(prefix,'use_mimic_tags') : use_mimic_tags,
                add_prefix(prefix,'use_smallest_joint_limits') : use_smallest_joint_limits,
                add_prefix(prefix,'source_list') : source_list,
                add_prefix(prefix,'delta') : delta}
                # 'robot_zeros_filepath' : robot_zeros_filepath,
                # 'dependent_joints' : dependent_joints}

    action_list = [declare_rate_cmd, declare_publish_default_positions_cmd, declare_publish_default_velocities_cmd, declare_publish_default_efforts_cmd,
                    declare_use_mimic_tags_cmd, declare_use_smallest_joint_limits_cmd, declare_source_list_cmd, declare_delta_cmd] 
                    # , declare_zeros_filepath_cmd, declare_dependent_joints_cmd]
    return parmas_dict, action_list


def get_prefix_jsp():
    return 'jsp'

