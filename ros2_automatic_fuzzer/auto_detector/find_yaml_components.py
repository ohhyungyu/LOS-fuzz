import os
from zenlog import log as logging
import re
import yaml


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

    # Catches expressions of the type create_publisher<A>("B"
    # being A the type being catched, and B the name of the service
    create_subscription_regex = (
        r"create_subscription\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*\"(?P<name>[^\"]+)\""
    )

    # Catches expressions of the type create_service<A>("B"
    # being A the type being catched, and B the name of the service
    create_service_regex = (
        r"create_service\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*\"(?P<name>[^\"]+)\""
    )

    # Catches expressions of the type create_server<A>(..., "B")
    # being A the type being catched, and B the name of the server
    # Have to change !@!!!@!
    create_action_regex = (
        r"create_server\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*[^,]+,\s*\"(?P<name>[^\"]+)\""
    )

    logging.debug("Checking file contents")
    found_subscription = dict()
    found_services = dict()
    found_actions = dict()

    finding_patterns = [
        (create_subscription_regex, found_subscription),
        (create_service_regex, found_services),
        (create_action_regex, found_actions),
    ]

    for filepath in cppfiles:
        try:
            with open(filepath) as f:
                contents = f.read()
                for (regex, container) in finding_patterns:
                    instance = re.search(regex, contents)
                    if instance:
                        logging.debug(f"Found instance at {filepath}")

                        name = instance.group("name")
                        type = instance.group("type")

                        if "::" not in type:
                            logging.warning(f"The `{type}` type may be incomplete")

                        container[name] = {
                            "headers_file": map_type_to_headers_file(type),
                            "source": os.path.relpath(filepath, start=rootDir),
                            "type": type,
                            "parameters": [],
                        }
        except:
            pass

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


def map_type_to_headers_file(type: str) -> str:
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
            "statistics_msgs::msg::StatisticDataType": "",
            "statistics_msgs::msg::MetricsMessage": "",
            "statistics_msgs::msg::StatisticDataPoint": ""
        },
        "tf2_msgs_srv": {
            "__LOCATION": "tf2_msgs/srv/",
            "tf2_msgs::srv::FrameGraph_Request": "frame_graph",
            "tf2_msgs::srv::FrameGraph_Response": "frame_graph",
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
            "shape_msgs::msg::Plane": "",
            "shape_msgs::msg::MeshTriangle": "",
            "shape_msgs::msg::SolidPrimitive": "",
            "shape_msgs::msg::Mesh": ""
        },
        "action_msgs_srv": {
            "__LOCATION": "action_msgs/srv/",
            "action_msgs::srv::CancelGoal_Response": "",
            "action_msgs::srv::CancelGoal_Request": "",
        },
        "action_msgs_msg": {
            "__LOCATION": "action_msgs/msg/",
            "action_msgs::msg::GoalStatusArray": "",
            "action_msgs::msg::GoalStatus": "",
            "action_msgs::msg::GoalInfo": ""
        },
        "std_srvs": {
            "__LOCATION": "std_srvs/srv/",
            "std_srvs::srv::Empty_Request": "",
            "std_srvs::srv::Empty_Response": "",
            "std_srvs::srv::SetBool_Response": "",
            "std_srvs::srv::SetBool_Request": "",
            "std_srvs::srv::Trigger_Response": "",
            "std_srvs::srv::Trigger_Request": ""
        },
        "composition_interfaces": {
            "__LOCATION": "composition_interfaces/srv/",
            "composition_interfaces::srv::LoadNode_Response": "",
            "composition_interfaces::srv::UnloadNode_Request": "",
            "composition_interfaces::srv::UnloadNode_Response": "",
            "composition_interfaces::srv::ListNodes_Request": "",
            "composition_interfaces::srv::ListNodes_Response": "",
            "composition_interfaces::srv::LoadNode_Request": ""
        },
        "diagnostic_msgs_srv": {
            "__LOCATION": "diagnostic_msgs/srv/",
            "diagnostic_msgs::srv::SelfTest_Response": "",
            "diagnostic_msgs::srv::AddDiagnostics_Response": "",
            "diagnostic_msgs::srv::SelfTest_Request": "",
            "diagnostic_msgs::srv::AddDiagnostics_Request": "",
        },
        "diagnostic_msgs_msg": {
            "__LOCATION": "diagnostic_msgs/msg/",
            "diagnostic_msgs::msg::DiagnosticArray": "",
            "diagnostic_msgs::msg::KeyValue": "",
            "diagnostic_msgs::msg::DiagnosticStatus": ""
        },
        "nav_msgs_srv": {
            "__LOCATION": "nav_msgs/srv/",
            "nav_msgs::srv::GetMap_Response": "",
            "nav_msgs::srv::GetPlan_Response": "",
            "nav_msgs::srv::SetMap_Response": "",
            "nav_msgs::srv::GetPlan_Request": "",
            "nav_msgs::srv::SetMap_Request": "",
            "nav_msgs::srv::GetMap_Request": "",
        },
        "nav_msgs_msg": {
            "__LOCATION": "nav_msgs/msg/",
            "nav_msgs::msg::Odometry": "",
            "nav_msgs::msg::GridCells": "",
            "nav_msgs::msg::OccupancyGrid": "",
            "nav_msgs::msg::MapMetaData": "",
            'nav_msgs::msg::Path': ""
        },
        "lifecycle_msgs_srv": {
            "__LOCATION": "lifecycle_msgs/srv/",
            "lifecycle_msgs::srv::ChangeState_Request": "",
            "lifecycle_msgs::srv::GetAvailableStates_Response": "",
            "lifecycle_msgs::srv::GetState_Request": "",
            "lifecycle_msgs::srv::GetState_Response": "",
            "lifecycle_msgs::srv::ChangeState_Response": "",
            "lifecycle_msgs::srv::GetAvailableTransitions_Request": "",
            "lifecycle_msgs::srv::GetAvailableTransitions_Response": "",
            "lifecycle_msgs::srv::GetAvailableStates_Request": "",
        },
        "lifecycle_msgs_msg": {    
            "__LOCATION": "lifecycle_msgs/msg/",        
            "lifecycle_msgs::msg::TransitionEvent": "",
            "lifecycle_msgs::msg::State": "",
            "lifecycle_msgs::msg::TransitionDescription": "",
            "lifecycle_msgs::msg::Transition": ""
        },
        "visualization_msgs_srv": {
            "__LOCATION": "visualization_msgs/srv/",
            "visualization_msgs::srv::GetInteractiveMarkers_Response": "",
            "visualization_msgs::srv::GetInteractiveMarkers_Request": ""
        },
        "visualization_msgs_msg": {
            "__LOCATION": "visualization_msgs/msg/",
            "visualization_msgs::msg::InteractiveMarkerInit": "",
            "visualization_msgs::msg::MenuEntry": "",
            "visualization_msgs::msg::Marker": "",
            "visualization_msgs::msg::InteractiveMarkerFeedback": "",
            "visualization_msgs::msg::InteractiveMarker": "",
            "visualization_msgs::msg::ImageMarker": "",
            "visualization_msgs::msg::MarkerArray": "",
            "visualization_msgs::msg::InteractiveMarkerControl": "",
            "visualization_msgs::msg::InteractiveMarkerUpdate": "",
            "visualization_msgs::msg::InteractiveMarkerPose": ""
        },
        "rcl_interfaces_srv": {
            "__LOCATION": "rcl_interfaces/srv/",
            "rcl_interfaces::srv::GetParameters_Request": "",
            "rcl_interfaces::srv::DescribeParameters_Response": "",
            "rcl_interfaces::srv::GetParameterTypes_Request": "",
            "rcl_interfaces::srv::SetParameters_Request": "",
            "rcl_interfaces::srv::SetParametersAtomically_Response": "",
            "rcl_interfaces::srv::ListParameters_Request": "",
            "rcl_interfaces::srv::SetParameters_Response": "",
            "rcl_interfaces::srv::GetParameters_Response": "",
            "rcl_interfaces::srv::SetParametersAtomically_Request": "",
            "rcl_interfaces::srv::GetParameterTypes_Response": "",
            "rcl_interfaces::srv::DescribeParameters_Request": "",
            "rcl_interfaces::srv::ListParameters_Response": ""
        },
        "rcl_interfaces_msg": {
            "__LOCATION": "rcl_interfaces/msg/",
            "rcl_interfaces::msg::Parameter": "",
            "rcl_interfaces::msg::SetParametersResult": "",
            "rcl_interfaces::msg::Log": "",
            "rcl_interfaces::msg::ParameterEvent": "",
            "rcl_interfaces::msg::ListParametersResult": "",
            "rcl_interfaces::msg::ParameterEventDescriptors": "",
            "rcl_interfaces::msg::ParameterValue": "",
            "rcl_interfaces::msg::IntegerRange": "",
            "rcl_interfaces::msg::ParameterDescriptor": "",
            "rcl_interfaces::msg::ParameterType": "",
            "rcl_interfaces::msg::FloatingPointRange": ""
        },
        "sensor_msgs_srv": {
            "__LOCATION": "sensor_msgs/srv/",
            "sensor_msgs::srv::SetCameraInfo_Request": "",
            "sensor_msgs::srv::SetCameraInfo_Response": ""
        },
        "sensor_msgs_msg": {
            "__LOCATION": "sensor_msgs/msg/",
            "sensor_msgs::msg::RelativeHumidity": "",
            "sensor_msgs::msg::LaserEcho": "",
            "sensor_msgs::msg::TimeReference": "",
            "sensor_msgs::msg::FluidPressure": "",
            "sensor_msgs::msg::PointCloud": "",
            "sensor_msgs::msg::NavSatFix": "",
            "sensor_msgs::msg::JoyFeedback": "",
            "sensor_msgs::msg::JointState": "",
            "sensor_msgs::msg::JoyFeedbackArray": "",
            "sensor_msgs::msg::Joy": "",
            "sensor_msgs::msg::PointCloud2": "",
            "sensor_msgs::msg::Image": "",
            "sensor_msgs::msg::RegionOfInterest": "",
            "sensor_msgs::msg::Range": "",
            "sensor_msgs::msg::NavSatStatus": "",
            "sensor_msgs::msg::MagneticField": "",
            "sensor_msgs::msg::CompressedImage": "",
            "sensor_msgs::msg::ChannelFloat32": "",
            "sensor_msgs::msg::Temperature": "",
            "sensor_msgs::msg::MultiEchoLaserScan": "",
            "sensor_msgs::msg::MultiDOFJointState": "",
            "sensor_msgs::msg::Imu": "",
            "sensor_msgs::msg::PointField": "",
            "sensor_msgs::msg::Illuminance": "",
            "sensor_msgs::msg::LaserScan": "",
            "sensor_msgs::msg::BatteryState": "",
            "sensor_msgs::msg::CameraInfo": ""
        },
        "example_interfaces_srv": {
            "__LOCATION": "example_interfaces/srv/",
            "example_interfaces::srv::AddTwoInts_Request": "",
            "example_interfaces::srv::AddTwoInts_Response": "",
            "example_interfaces::srv::SetBool_Response": "",
            "example_interfaces::srv::SetBool_Request": "",
            "example_interfaces::srv::Trigger_Response": "",
            "example_interfaces::srv::Trigger_Request": ""
        },
        "example_interfaces_msg": {
            "__LOCATION": "example_interfaces/msg/",
            "example_interfaces::msg::Float64MultiArray": "",
            "example_interfaces::msg::Int32MultiArray": "",
            "example_interfaces::msg::Int16MultiArray": "",
            "example_interfaces::msg::Int32": "",
            "example_interfaces::msg::WString": "",
            "example_interfaces::msg::MultiArrayDimension": "",
            "example_interfaces::msg::UInt32MultiArray": "",
            "example_interfaces::msg::Int64MultiArray": "",
            "example_interfaces::msg::Float32MultiArray": "",
            "example_interfaces::msg::ByteMultiArray": "",
            "example_interfaces::msg::UInt64": "",
            "example_interfaces::msg::Empty": "",
            "example_interfaces::msg::Int8MultiArray": "",
            "example_interfaces::msg::UInt16": "",
            "example_interfaces::msg::UInt16MultiArray": "",
            "example_interfaces::msg::UInt8": "",
            "example_interfaces::msg::Bool": "",
            "example_interfaces::msg::Int8": "",
            "example_interfaces::msg::Char": "",
            "example_interfaces::msg::MultiArrayLayout": "",
            "example_interfaces::msg::UInt32": "",
            "example_interfaces::msg::Int16": "",
            "example_interfaces::msg::Int64": "",
            "example_interfaces::msg::UInt64MultiArray": "",
            "example_interfaces::msg::UInt8MultiArray": "",
            "example_interfaces::msg::String": "",
            "example_interfaces::msg::Float64": "",
            "example_interfaces::msg::Float32": "",
            "example_interfaces::msg::Byte": ""
        },
        "geometry_msgs": {
            "__LOCATION": "geometry_msgs/msg/",
            "geometry_msgs::msg::InertiaStamped": "",
            "geometry_msgs::msg::PolygonStamped": "",
            "geometry_msgs::msg::AccelWithCovariance": "",
            "geometry_msgs::msg::Point": "",
            "geometry_msgs::msg::Point32": "",
            "geometry_msgs::msg::TwistWithCovariance": "",
            "geometry_msgs::msg::PoseStamped": "",
            "geometry_msgs::msg::TransformStamped": "",
            "geometry_msgs::msg::PoseWithCovariance": "",
            "geometry_msgs::msg::Vector3": "",
            "geometry_msgs::msg::Accel": "",
            "geometry_msgs::msg::Inertia": "",
            "geometry_msgs::msg::AccelWithCovarianceStamped": "",
            "geometry_msgs::msg::Pose2D": "",
            "geometry_msgs::msg::Quaternion": "",
            "geometry_msgs::msg::Pose": "",
            "geometry_msgs::msg::QuaternionStamped": "",
            "geometry_msgs::msg::Wrench": "",
            "geometry_msgs::msg::PoseArray": "",
            "geometry_msgs::msg::TwistStamped": "",
            "geometry_msgs::msg::Twist": "",
            "geometry_msgs::msg::AccelStamped": "",
            "geometry_msgs::msg::WrenchStamped": "",
            "geometry_msgs::msg::PointStamped": "",
            "geometry_msgs::msg::TwistWithCovarianceStamped": "",
            "geometry_msgs::msg::PoseWithCovarianceStamped": "",
            "geometry_msgs::msg::Transform": "",
            "geometry_msgs::msg::Vector3Stamped": "",
            "geometry_msgs::msg::Polygon": ""
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

    for _HD in mapping.keys():
        if type in mapping[_HD]:
            return mapping[_HD]["__LOCATION"] + mapping[_HD][type] + ".hpp"
    return "TODO"
