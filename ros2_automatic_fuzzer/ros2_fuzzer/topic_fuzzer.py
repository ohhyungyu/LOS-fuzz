import sys
import os

sys.path.append("..")

from ros2_fuzzer.fuzzing_utils.type_parser import TypeParser
from ros2_fuzzer.fuzzing_utils.fuzzing_descriptor import FuzzTargetProcessor
from ros2_fuzzer.fuzzing_utils.generate_cpp_file import generate_cpp_file


def generate_topic_template(topic_name: str, source: str, ros_type_str: str, headers_file: str) -> str:
    original_file = os.path.basename(source)
    topic_type = ros_type_str.replace("::", "/")
    ros_type = TypeParser.parse_type(topic_type)


    # ros_type -> ROSType
    # headers_file -> ros_type_str Headerfile (.hpp)
    # original_file -> main.cpp file
    # ros_type_str -> std_msgs/msg/String.msg 
    fuzz_target = FuzzTargetProcessor().process(
        ros_type,
        name=topic_name,
        headers_file=headers_file,
        original_file=original_file,
        ros_type_str=ros_type_str,
    )

    return generate_cpp_file(
        fuzz_target=fuzz_target,
        source_file=source,
        template_name="topic_template.jinx.cpp",
    )
