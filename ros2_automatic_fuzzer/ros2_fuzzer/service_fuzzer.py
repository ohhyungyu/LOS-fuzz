import sys
import os

sys.path.append("..")

from ros2_fuzzer.fuzzing_utils.type_parser import TypeParser
from ros2_fuzzer.fuzzing_utils.fuzzing_descriptor import FuzzTargetProcessor
from ros2_fuzzer.fuzzing_utils.generate_cpp_file import generate_cpp_file


def generate_service_template(service_name: str, source: str, ros_type_str: str, headers_file: str) -> str:
    original_file = os.path.basename(source)
    service_type = ros_type_str.replace("::", "/")
    ros_type = TypeParser.parse_type(service_type)

    fuzz_target = FuzzTargetProcessor().process(
        ros_type,
        name=service_name,
        headers_file=headers_file,
        original_file=original_file,
        ros_type_str=ros_type_str,
    )

    return generate_cpp_file(
        fuzz_target=fuzz_target,
        source_file=source,
        template_name="service_template.jinx.cpp",
    )
