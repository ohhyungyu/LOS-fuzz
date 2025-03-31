import os
from zenlog import log as logging
from PyInquirer import Separator, prompt


def component_options(components: dict):
    def validate_component(component):
        source = component["source"]
        original_file = os.path.basename(source)
        ros_type_str = component["type"]
        headers_file = component["headers_file"]

        if "TODO" in [source, ros_type_str, headers_file, original_file]:
            return "There are unfilled TODOs!"
        if not os.path.isfile(source):
            return f"The source `{source}` is not a valid path"
        return ""

    mapped_components = [
        (component, component_name, validate_component(component))
        for (component_name, component) in components.items()
    ]
    return [
        {"name": component_name, "disabled": error}
        if error
        else {"name": component_name, "value": [component_name, component]}
        for (component, component_name, error) in mapped_components
    ]


def ask_for_components(name_pkg: str, services: dict, topics: dict, actions: dict):
    services_choices = component_options(services)
    topics_choices = component_options(topics)
    actions_choices = component_options(actions)

    all_choices = services_choices + topics_choices + actions_choices

    there_are_valid_choices = any("disabled" not in choice for choice in all_choices)
    if not there_are_valid_choices:
        logging.error(
            "There are no available components to fuzz!\n"
            "Check for TODOs in the fuzz.yaml file."
        )
        exit(-1)

    choices = []
    if len(topics_choices) !=0:
        choices.append(Separator("Topics"))
        choices += topics_choices
    if len(services_choices) !=0:
        choices.append(Separator("Services"))
        choices += services_choices
    if len(actions_choices) !=0:
        choices.append(Separator("Actions"))
        choices += actions_choices

    questions = [
        {
            "type": "checkbox",
            "message": f"What would you like to fuzz from the \"{name_pkg}\" package?\n",
            "name": "to_fuzz_components",
            "choices": choices,
        }
    ]
    print("(<up>, <down> to move)\n"
    "(<space> to select, <a> to toggle, <i> to invert)")
    
    return prompt(questions)["to_fuzz_components"]
