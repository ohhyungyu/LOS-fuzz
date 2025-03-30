import os
from zenlog import log as logging
import re
import yaml

mapping = {
    "unique_identifier_msgs": {
        "__LOCATION": "unique_identifier_msgs/msg/",
        "unique_identifier_msgs::msg::UUID": "uuid"
    },
    "stereo_msgs": {
        "__LOCATION": "stereo_msgs/msg/",
        "stereo_msgs::msg::DisparityImage": "disparity_image"
    },
    "libstatistics_collector": {
        "__LOCATION": "libstatistics_collector/msg/",
        "libstatistics_collector::msg::DummyMessage": "dummy_message"
    },
    "rosgraph_msgs": {
        "__LOCATION": "rosgraph_msgs/msg/",
        "rosgraph_msgs::msg::Clock": "clock"
    },
    "builtin_interfaces": {
        "__LOCATION": "builtin_interfaces/msg/",
        "builtin_interfaces::msg::Time": "time",
        "builtin_interfaces::msg::Duration": "duration"
    },
    "actionlib_msgs": {
        "__LOCATION": "actionlib_msgs/msg/",
        "actionlib_msgs::msg::GoalID": "goal_id",
        "actionlib_msgs::msg::GoalStatusArray": "goal_status_array",
        "actionlib_msgs::msg::GoalStatus": "goal_status"
    },
    "rmw_dds_common": {
        "__LOCATION": "rmw_dds_common/msg/",
        "rmw_dds_common::msg::ParticipantEntitiesInfo": "participant_entities_info",
        "rmw_dds_common::msg::NodeEntitiesInfo": "node_entities_info",
        "rmw_dds_common::msg::Gid": "gid"
    },
    "statistics_msgs": {
        "__LOCATION": "statistics_msgs/msg/",
        "statistics_msgs::msg::StatisticDataType": "statistic_data_type",
        "statistics_msgs::msg::MetricsMessage": "metrics_message",
        "statistics_msgs::msg::StatisticDataPoint": "statistic_data_point"
    },
    "tf2_msgs_srv": {
        "__LOCATION": "tf2_msgs/srv/",
        "tf2_msgs::srv::FrameGraph_Request": "frame_graph",
        "tf2_msgs::srv::FrameGraph_Response": "frame_graph"
    },
    "tf2_msgs_msg": {
        "__LOCATION": "tf2_msgs/msg/",
        "tf2_msgs::msg::TFMessage": "tf_message",
        "tf2_msgs::msg::TF2Error": "tf2_error"
    },
    "trajectory_msgs" : {
        "__LOCATION": "trajectory_msgs/msg/",
        "trajectory_msgs::msg::MultiDOFJointTrajectory": "multi_dof_joint_trajectory",
        "trajectory_msgs::msg::JointTrajectoryPoint": "joint_trajectory_point",
        "trajectory_msgs::msg::MultiDOFJointTrajectoryPoint": "multi_dof_joint_trajectory_point",
        "trajectory_msgs::msg::JointTrajectory": "joint_trajectory"
    },
    "shape_msgs": {
        "__LOCATION": "shape_msgs/msg/",
        "shape_msgs::msg::Plane": "plane",
        "shape_msgs::msg::MeshTriangle": "mesh_triangle",
        "shape_msgs::msg::SolidPrimitive": "solid_primitive",
        "shape_msgs::msg::Mesh": "mesh"
    },
    "action_msgs_srv": {
        "__LOCATION": "action_msgs/srv/",
        "action_msgs::srv::CancelGoal_Response": "cancel_goal",
        "action_msgs::srv::CancelGoal_Request": "cancel_goal"
    },
    "action_msgs_msg": {
        "__LOCATION": "action_msgs/msg/",
        "action_msgs::msg::GoalStatusArray": "goal_status_array",
        "action_msgs::msg::GoalStatus": "goal_status",
        "action_msgs::msg::GoalInfo": "goal_info"
    },
    "std_srvs": {
        "__LOCATION": "std_srvs/srv/",
        "std_srvs::srv::Empty_Request": "empty",
        "std_srvs::srv::Empty_Response": "empty",
        "std_srvs::srv::SetBool_Response": "set_bool",
        "std_srvs::srv::SetBool_Request": "set_bool",
        "std_srvs::srv::Trigger_Response": "trigger",
        "std_srvs::srv::Trigger_Request": "trigger"
    },
    "composition_interfaces": {
        "__LOCATION": "composition_interfaces/srv/",
        "composition_interfaces::srv::LoadNode_Response": "load_node",
        "composition_interfaces::srv::UnloadNode_Request": "unload_node",
        "composition_interfaces::srv::UnloadNode_Response": "unload_node",
        "composition_interfaces::srv::ListNodes_Request": "list_nodes",
        "composition_interfaces::srv::ListNodes_Response": "list_nodes",
        "composition_interfaces::srv::LoadNode_Request": "load_node"
    },
    "diagnostic_msgs_srv": {
        "__LOCATION": "diagnostic_msgs/srv/",
        "diagnostic_msgs::srv::SelfTest_Response": "self_test",
        "diagnostic_msgs::srv::AddDiagnostics_Response": "add_diagnostics",
        "diagnostic_msgs::srv::SelfTest_Request": "self_test",
        "diagnostic_msgs::srv::AddDiagnostics_Request": "add_diagnostics"
    },
    "diagnostic_msgs_msg": {
        "__LOCATION": "diagnostic_msgs/msg/",
        "diagnostic_msgs::msg::DiagnosticArray": "diagnostic_array",
        "diagnostic_msgs::msg::KeyValue": "key_value",
        "diagnostic_msgs::msg::DiagnosticStatus": "diagnostic_status"
    },
    "nav_msgs_srv": {
        "__LOCATION": "nav_msgs/srv/",
        "nav_msgs::srv::GetMap_Response": "get_map",
        "nav_msgs::srv::GetPlan_Response": "get_plan",
        "nav_msgs::srv::SetMap_Response": "set_map",
        "nav_msgs::srv::GetPlan_Request": "get_plan",
        "nav_msgs::srv::SetMap_Request": "set_map",
        "nav_msgs::srv::GetMap_Request": "get_map"
    },
    "nav_msgs_msg": {
        "__LOCATION": "nav_msgs/msg/",
        "nav_msgs::msg::Odometry": "odometry",
        "nav_msgs::msg::GridCells": "grid_cells",
        "nav_msgs::msg::OccupancyGrid": "occupancy_grid",
        "nav_msgs::msg::MapMetaData": "map_meta_data",
        'nav_msgs::msg::Path': "path"
    },
    "lifecycle_msgs_srv": {
        "__LOCATION": "lifecycle_msgs/srv/",
        "lifecycle_msgs::srv::ChangeState_Request": "change_state",
        "lifecycle_msgs::srv::GetAvailableStates_Response": "get_available_states",
        "lifecycle_msgs::srv::GetState_Request": "get_state",
        "lifecycle_msgs::srv::GetState_Response": "get_state",
        "lifecycle_msgs::srv::ChangeState_Response": "change_state",
        "lifecycle_msgs::srv::GetAvailableTransitions_Request": "get_available_transitions",
        "lifecycle_msgs::srv::GetAvailableTransitions_Response": "get_available_transitions",
        "lifecycle_msgs::srv::GetAvailableStates_Request": "get_available_states"
    },
    "lifecycle_msgs_msg": {    
        "__LOCATION": "lifecycle_msgs/msg/",        
        "lifecycle_msgs::msg::TransitionEvent": "transition_event",
        "lifecycle_msgs::msg::State": "state",
        "lifecycle_msgs::msg::TransitionDescription": "transition_description",
        "lifecycle_msgs::msg::Transition": "transition"
    },
    "visualization_msgs_srv": {
        "__LOCATION": "visualization_msgs/srv/",
        "visualization_msgs::srv::GetInteractiveMarkers_Response": "get_interactive_markers",
        "visualization_msgs::srv::GetInteractiveMarkers_Request": "get_interactive_markers"
    },
    "visualization_msgs_msg": {
        "__LOCATION": "visualization_msgs/msg/",
        "visualization_msgs::msg::InteractiveMarkerInit": "interactive_marker_init",
        "visualization_msgs::msg::MenuEntry": "menu_entry",
        "visualization_msgs::msg::Marker": "marker",
        "visualization_msgs::msg::InteractiveMarkerFeedback": "interactive_marker_feedback",
        "visualization_msgs::msg::InteractiveMarker": "interactive_marker",
        "visualization_msgs::msg::ImageMarker": "image_marker",
        "visualization_msgs::msg::MarkerArray": "marker_array",
        "visualization_msgs::msg::InteractiveMarkerControl": "interactive_marker_control",
        "visualization_msgs::msg::InteractiveMarkerUpdate": "interactive_marker_update",
        "visualization_msgs::msg::InteractiveMarkerPose": "interactive_marker_pose"
    },
    "rcl_interfaces_srv": {
        "__LOCATION": "rcl_interfaces/srv/",
        "rcl_interfaces::srv::GetParameters_Request": "get_parameters",
        "rcl_interfaces::srv::DescribeParameters_Response": "describe_parameters",
        "rcl_interfaces::srv::GetParameterTypes_Request": "get_parameter_types",
        "rcl_interfaces::srv::SetParameters_Request": "set_parameters",
        "rcl_interfaces::srv::SetParametersAtomically_Response": "set_parameters_atomically",
        "rcl_interfaces::srv::ListParameters_Request": "list_parameters",
        "rcl_interfaces::srv::SetParameters_Response": "set_parameters",
        "rcl_interfaces::srv::GetParameters_Response": "get_parameters",
        "rcl_interfaces::srv::SetParametersAtomically_Request": "set_parameters_atomically",
        "rcl_interfaces::srv::GetParameterTypes_Response": "get_parameter_types",
        "rcl_interfaces::srv::DescribeParameters_Request": "describe_parameters",
        "rcl_interfaces::srv::ListParameters_Response": "list_parameters"
    },
    "rcl_interfaces_msg": {
        "__LOCATION": "rcl_interfaces/msg/",
        "rcl_interfaces::msg::Parameter": "parameter",
        "rcl_interfaces::msg::SetParametersResult": "set_parameters_result",
        "rcl_interfaces::msg::Log": "log",
        "rcl_interfaces::msg::ParameterEvent": "parameter_event",
        "rcl_interfaces::msg::ListParametersResult": "list_parameters_result",
        "rcl_interfaces::msg::ParameterEventDescriptors": "parameter_event_descriptors",
        "rcl_interfaces::msg::ParameterValue": "parameter_value",
        "rcl_interfaces::msg::IntegerRange": "integer_range",
        "rcl_interfaces::msg::ParameterDescriptor": "parameter_descriptor",
        "rcl_interfaces::msg::ParameterType": "parameter_type",
        "rcl_interfaces::msg::FloatingPointRange": "floating_point_range"
    },
    "sensor_msgs_srv": {
        "__LOCATION": "sensor_msgs/srv/",
        "sensor_msgs::srv::SetCameraInfo_Request": "set_camera_info",
        "sensor_msgs::srv::SetCameraInfo_Response": "set_camera_info"
    },
    "sensor_msgs_msg": {
        "__LOCATION": "sensor_msgs/msg/",
        "sensor_msgs::msg::RelativeHumidity": "relative_humidity",
        "sensor_msgs::msg::LaserEcho": "laser_echo",
        "sensor_msgs::msg::TimeReference": "time_reference",
        "sensor_msgs::msg::FluidPressure": "fluid_pressure",
        "sensor_msgs::msg::PointCloud": "point_cloud",
        "sensor_msgs::msg::NavSatFix": "nav_sat_fix",
        "sensor_msgs::msg::JoyFeedback": "joy_feedback",
        "sensor_msgs::msg::JointState": "joint_state",
        "sensor_msgs::msg::JoyFeedbackArray": "joy_feedback_array",
        "sensor_msgs::msg::Joy": "joy",
        "sensor_msgs::msg::PointCloud2": "point_cloud2",
        "sensor_msgs::msg::Image": "image",
        "sensor_msgs::msg::RegionOfInterest": "region_of_interest",
        "sensor_msgs::msg::Range": "range",
        "sensor_msgs::msg::NavSatStatus": "nav_sat_status",
        "sensor_msgs::msg::MagneticField": "magnetic_field",
        "sensor_msgs::msg::CompressedImage": "compressed_image",
        "sensor_msgs::msg::ChannelFloat32": "channel_float32",
        "sensor_msgs::msg::Temperature": "temperature",
        "sensor_msgs::msg::MultiEchoLaserScan": "multi_echo_laser_scan",
        "sensor_msgs::msg::MultiDOFJointState": "multi_dof_joint_state",
        "sensor_msgs::msg::Imu": "imu",
        "sensor_msgs::msg::PointField": "point_field",
        "sensor_msgs::msg::Illuminance": "illuminance",
        "sensor_msgs::msg::LaserScan": "laser_scan",
        "sensor_msgs::msg::BatteryState": "battery_state",
        "sensor_msgs::msg::CameraInfo": "camera_info"
    },
    "example_interfaces_srv": {
        "__LOCATION": "example_interfaces/srv/",
        "example_interfaces::srv::AddTwoInts_Request": "add_two_ints",
        "example_interfaces::srv::AddTwoInts_Response": "add_two_ints",
        "example_interfaces::srv::SetBool_Response": "set_bool",
        "example_interfaces::srv::SetBool_Request": "set_bool",
        "example_interfaces::srv::Trigger_Response": "trigger",
        "example_interfaces::srv::Trigger_Request": "trigger"
    },
    "example_interfaces_msg": {
        "__LOCATION": "example_interfaces/msg/",
        "example_interfaces::msg::Float64MultiArray": "float64_multi_array",
        "example_interfaces::msg::Int32MultiArray": "int32_multi_array",
        "example_interfaces::msg::Int16MultiArray": "int16_multi_array",
        "example_interfaces::msg::Int32": "int32",
        "example_interfaces::msg::WString": "w_string",
        "example_interfaces::msg::MultiArrayDimension": "multi_array_dimension",
        "example_interfaces::msg::UInt32MultiArray": "u_int32_multi_array",
        "example_interfaces::msg::Int64MultiArray": "int64_multi_array",
        "example_interfaces::msg::Float32MultiArray": "float32_multi_array",
        "example_interfaces::msg::ByteMultiArray": "byte_multi_array",
        "example_interfaces::msg::UInt64": "u_int64",
        "example_interfaces::msg::Empty": "empty",
        "example_interfaces::msg::Int8MultiArray": "int8_multi_array",
        "example_interfaces::msg::UInt16": "u_int16",
        "example_interfaces::msg::UInt16MultiArray": "u_int16_multi_array",
        "example_interfaces::msg::UInt8": "u_int8",
        "example_interfaces::msg::Bool": "bool",
        "example_interfaces::msg::Int8": "int8",
        "example_interfaces::msg::Char": "char",
        "example_interfaces::msg::MultiArrayLayout": "multi_array_layout",
        "example_interfaces::msg::UInt32": "u_int32",
        "example_interfaces::msg::Int16": "int16",
        "example_interfaces::msg::Int64": "int64",
        "example_interfaces::msg::UInt64MultiArray": "u_int64_multi_array",
        "example_interfaces::msg::UInt8MultiArray": "u_int8_multi_array",
        "example_interfaces::msg::String": "string",
        "example_interfaces::msg::Float64": "float64",
        "example_interfaces::msg::Float32": "float32",
        "example_interfaces::msg::Byte": "byte"
    },
    "geometry_msgs": {
        "__LOCATION": "geometry_msgs/msg/",
        "geometry_msgs::msg::InertiaStamped": "inertia_stamped",
        "geometry_msgs::msg::PolygonStamped": "polygon_stamped",
        "geometry_msgs::msg::AccelWithCovariance": "accel_with_covariance",
        "geometry_msgs::msg::Point": "point",
        "geometry_msgs::msg::Point32": "point32",
        "geometry_msgs::msg::TwistWithCovariance": "twist_with_covariance",
        "geometry_msgs::msg::PoseStamped": "pose_stamped",
        "geometry_msgs::msg::TransformStamped": "transform_stamped",
        "geometry_msgs::msg::PoseWithCovariance": "pose_with_covariance",
        "geometry_msgs::msg::Vector3": "vector3",
        "geometry_msgs::msg::Accel": "accel",
        "geometry_msgs::msg::Inertia": "inertia",
        "geometry_msgs::msg::AccelWithCovarianceStamped": "accel_with_covariance_stamped",
        "geometry_msgs::msg::Pose2D": "pose2_d",
        "geometry_msgs::msg::Quaternion": "quaternion",
        "geometry_msgs::msg::Pose": "pose",
        "geometry_msgs::msg::QuaternionStamped": "quaternion_stamped",
        "geometry_msgs::msg::Wrench": "wrench",
        "geometry_msgs::msg::PoseArray": "pose_array",
        "geometry_msgs::msg::TwistStamped": "twist_stamped",
        "geometry_msgs::msg::Twist": "twist",
        "geometry_msgs::msg::AccelStamped": "accel_stamped",
        "geometry_msgs::msg::WrenchStamped": "wrench_stamped",
        "geometry_msgs::msg::PointStamped": "point_stamped",
        "geometry_msgs::msg::TwistWithCovarianceStamped": "twist_with_covariance_stamped",
        "geometry_msgs::msg::PoseWithCovarianceStamped": "pose_with_covariance_stamped",
        "geometry_msgs::msg::Transform": "transform",
        "geometry_msgs::msg::Vector3Stamped": "vector3_stamped",
        "geometry_msgs::msg::Polygon": "polygon"
    },
    "std_msgs": {
        "__LOCATION": "std_msgs/msg/",
        "std_msgs::msg::Char": "char",
        "std_msgs::msg::Float64": "float_64",
        "std_msgs::msg::Int32": "int_32",
        "std_msgs::msg::Int8MultiArray": "int_8_multi_array",
        "std_msgs::msg::UInt16MultiArray": "u_int_16_multi_array",
        "std_msgs::msg::UInt8": "u_int_8",
        "std_msgs::msg::ColorRGBA": "color_rgba",
        "std_msgs::msg::Float64MultiArray": "float_64_multi_array",
        "std_msgs::msg::Int32MultiArray": "int_32_multi_array",
        "std_msgs::msg::MultiArrayDimension": "multi_array_dimension",
        "std_msgs::msg::UInt32": "u_int_32",
        "std_msgs::msg::UInt8MultiArray": "u_int_8_multi_array",
        "std_msgs::msg::Bool": "bool",
        "std_msgs::msg::Empty": "empty",
        "std_msgs::msg::Header": "header",
        "std_msgs::msg::Int64": "int_64",
        "std_msgs::msg::MultiArrayLayout": "multi_array_layout",
        "std_msgs::msg::UInt32MultiArray": "u_int_32_multi_array",
        "std_msgs::msg::Byte": "byte",
        "std_msgs::msg::Float32": "float_32",
        "std_msgs::msg::Int16": "int_16",
        "std_msgs::msg::Int64MultiArray": "int_64_multi_array",
        "std_msgs::msg::String": "string",
        "std_msgs::msg::UInt64": "u_int_64",
        "std_msgs::msg::ByteMultiArray": "byte_multi_array",
        "std_msgs::msg::Float32MultiArray": "float_32_multi_array",
        "std_msgs::msg::Int16MultiArray": "int_16_multi_array",
        "std_msgs::msg::Int8": "int_8",
        "std_msgs::msg::UInt16": "u_int_16",
        "std_msgs::msg::UInt64MultiArray": "u_int_64_multi_array"
    }
}


def find_yaml_components(rootDir: str, overwrite: bool) -> None:
    yaml_path = os.path.join(rootDir, "fuzz.yaml")

    if os.path.exists(yaml_path) and not overwrite:
        logging.warning("The file fuzz.yaml already exists")
        logging.warning("Use the -f flag to force overwriting")
        return

    logging.debug("Starting directory exploration")
    cppfiles = []
    for dirName, subdirList, fileList in os.walk(rootDir):
        for fname in fileList:
            if fname.endswith(".cpp"):
                cppfiles.append(os.path.join(dirName, fname))

    logging.debug(f"Logged {len(cppfiles)} .cpp files")



    logging.debug("Checking file contents")
    found_subscription = dict()
    found_services = dict()
    found_actions = dict()

    found_things = (found_subscription, found_services, found_actions)

    for filepath in cppfiles:
        contents = ""
        try:
            with open(filepath) as f:
                contents = f.read()
        except:
            pass
        
        if contents == "":
            continue
        
        backup_content = contents

        while True:
            content_line = contents[: contents.find(';')+1]
            if content_line == "":
                break
            contents = contents[contents.find(';')+1: ]

            instance = find_sub_srv_act_with_regex(content_line, backup_content)
            if type(instance) == tuple:
                logging.debug(f"Found instance at {filepath}")

                com_name = instance[1]["com_name"]
                com_type = instance[1]["com_type"]

                if "::" not in com_type:
                    logging.warning(f"The `{com_type}` type may be incomplete")

                found_things[instance[0]][com_name] = {
                    "headers_file": map_type_to_headers_file(com_type),
                    "source": os.path.relpath(filepath, start=rootDir),
                    "type": com_type,
                    "parameters": [],
                }

    # Generate results
    yaml_result = {}
    if found_subscription:
        yaml_result["topics"] = found_subscription

    if found_services:
        yaml_result["services"] = found_services

    if found_actions:
        yaml_result["actions"] = found_actions

    if len(found_subscription) + len(found_services) + len(found_actions) == 0:
        logging.error(
            "No component has been found\n"
            "Are you in (or have you provided) the correct path?"
        )
        exit(-1)

    if os.path.exists(yaml_path) and overwrite:
        logging.warning("Overwriting the fuzz.yaml file")

    with open(yaml_path, "w") as outfile:
        yaml.dump(yaml_result, outfile, sort_keys=False)

    logging.info("The file `fuzz.yaml` has been generated")
    logging.warning("Fill the TODOs accordingly before continuing!")

# Need to make Sorter about package name 
def find_package_name(filepath) -> str:
    pass

def get_add_declare_find_name(var_name: str, code_all: str):
    get_param_regex = rf"get_parameter\s*\(\s*\"(?P<param_name>[^\"]+)\"\s*,\s*{var_name}\s*\)"
    
    instance = re.search(get_param_regex, code_all)
    if instance:
        param_name = instance.group("param_name")
    else:
        return False
    
    add_param_regex = rf"add_parameter\s*\(\s*\"{param_name}\"\s*,\s*(?:\w+::)?ParameterValue\s*\(\s*\"(?P<name>[^\"]+)\"\s*\)"
    declare_param_regex = rf"declare_parameter\s*\(\s*\"{param_name}\"\s*,\s*(?:\w+::)?ParameterValue\s*\(\s*\"(?P<name>[^\"]+)\"\s*\)"

    instance = re.search(declare_param_regex, code_all)
    if instance:
        return instance.group("name")
    instance = re.search(add_param_regex, code_all)
    if instance:
        return instance.group("name")
    
    return False

def find_sub_srv_act_with_regex(code_line: str, code_all: str) -> tuple:
    '''
    Find_sub_srv_act_with_regex function is find things about regex\n
    Parameter : One Line of Code
    Return Value : Tuple which include sequence about sub, srv, act and instance about re.search
    Sequence : subscription -> 0, service -> 1, action -> 2
    '''

    # Catches expressions of the type create_publisher<A>("B"
    # being A the type being catched, and B the name of the service
    create_subscription_regex = r"create_subscription\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*\"(?P<name>[^\"]+)\""
    create_subscription_string = "create_subscription"
    create_subscription_regex_var = r"create_subscription\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*(?P<var_name>[^,\s]+)"

    # Catches expressions of the type create_service<A>("B"
    # being A the type being catched, and B the name of the service
    create_service_regex = r"create_service\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*\"(?P<name>[^\"]+)\""
    create_service_string = "create_service"
    create_service_regex_var = r"create_service\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*\"(?P<var_name>[^\"]+)\""

    # Catches expressions of the type create_server<A>(..., "B")
    # being A the type being catched, and B the name of the server
    # Have to change !@!!!@!
    create_action_regex = r"create_server\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*[^,]+,\s*\"(?P<name>[^\"]+)\""
    create_action_string = "create_server"
    create_action_regex_var = r"create_server\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*[^,]+,\s*\"(?P<var_name>[^\"]+)\""

    if create_subscription_string in code_line:
        instance = re.search(create_subscription_regex, code_line)
        if instance:
            return (0, {"com_name": instance.group("name"), "com_type": instance.group("type")})
        instance = re.search(create_subscription_regex_var, code_line)
        name = get_add_declare_find_name(instance.group("var_name"), code_all)
        return (0, {"com_name": name, "com_type": instance.group("type")})

    elif create_service_string in code_line:
        instance = re.search(create_service_regex, code_line)
        if instance:
            return (1, {"com_name": instance.group("name"), "com_type": instance.group("type")})
        instance = re.search(create_service_regex_var, code_line)
        name = get_add_declare_find_name(instance.group("var_name"), code_all)
        return (0, {"com_name": name, "com_type": instance.group("type")})

    elif create_action_string in code_line:
        instance = re.search(create_action_regex, code_line)
        if instance:
            return (2, {"com_name": instance.group("name"), "com_type": instance.group("type")})
        instance = re.search(create_action_regex_var, code_line)
        name = get_add_declare_find_name(instance.group("var_name"), code_all)
        return (0, {"com_name": name, "com_type": instance.group("type")})

    else :
        return False

def map_type_to_headers_file(type: str) -> str:
    for _HD in mapping.keys():
        if type in mapping[_HD]:
            return mapping[_HD]["__LOCATION"] + mapping[_HD][type] + ".hpp"
    return "TODO"

