import sys
import os

sys.path.append("..")

from ros2_fuzzer.fuzzing_utils.type_parser import TypeParser
from ros2_fuzzer.fuzzing_utils.fuzzing_descriptor import FuzzTargetProcessor
from ros2_fuzzer.fuzzing_utils.generate_cpp_file import generate_request_code


def generate_service_template(service_name: str, source: str, ros_type_str: str, headers_file: str, count: int) -> tuple:
    '''
    return (request_code, imports)
    '''
    original_file = os.path.basename(source)
    service_type = ros_type_str.replace("::", "/")
    ros_type = TypeParser.parse_type(service_type)

    # ros_type -> ROSType
    # headers_file -> ros_type_str Headerfile (.hpp)
    # original_file -> main.cpp file
    # ros_type_str -> std_msgs/msg/String.msg 
    
    tmp = FuzzTargetProcessor().process(
        ros_type,
        name=service_name,
        headers_file=headers_file,
        original_file=original_file,
        ros_type_str=ros_type_str,
        count=count,
    )
    fuzz_target = tmp[0] # FuzzTarget
    imports = tmp[1] # imports

    return (
        generate_request_code(
        fuzz_target=fuzz_target,
        template_name="fuzzing_templete/__service_template.jinx.cpp",
    ),
    imports
    )
