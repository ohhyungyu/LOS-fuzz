import os
from zenlog import log as logging
from pathlib import Path
from jinja2 import FileSystemLoader, Environment
from .fuzzing_descriptor import FuzzTarget


def generate_request_code(
        fuzz_target: FuzzTarget, template_name: str
        )->str:
    __location__ = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(__file__))
    )

    ''' NOT USE
    # plain_source_file_name = Path(source_file).name
    # without_extension = os.path.splitext(plain_source_file_name)[0] '''

    # Read template
    env = Environment(loader=FileSystemLoader(__location__))
    template = env.get_template(template_name)
    logging.debug("Template read")

    # Populate template
    template_arguments = fuzz_target.get_mapping()

    # if each fuzz api, then Need to corporate
    return template.render(template_arguments)

def generate_cpp_file(source_file: str, template_name: str, request_codes: str, importes: str):
    '''
    make FUZZING API and CODE
    Save in target dir, name which GENERATED_main.cpp
    '''
    __location__ = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(__file__))
    )

    # Read template
    env = Environment(loader=FileSystemLoader(__location__))
    template = env.get_template(template_name)
    logging.debug("Template read")
    template_arguments = {
        "FUZZING_API": "",
        "IMPORTES": "",
        "REQUEST_CODES": ""
    }
    
    fuzzing_path = os.path.join(os.path.dirname(__file__), "fuzzing_templete/__fuzzing_api.hpp")
    
    template_arguments["FUZZING_API"] = open(fuzzing_path).read()
    template_arguments["IMPORTES"] = importes
    template_arguments["REQUEST_CODES"] = request_codes

    template = template.render(template_arguments)
    logging.debug("Template populated")

    # Write the populated file
    full_path = os.path.join(
        os.path.dirname(source_file), + "GENERATED_main.cpp"
    )
    try:
        with open(full_path, "w") as fd:
            fd.write(template)
    except Exception:
        logging.error("Couldn't write generated file", exc_info=True)

    return full_path