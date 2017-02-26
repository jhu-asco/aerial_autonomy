#!/usr/bin/env python
# Generate message file based on event input
# Running the script:
#   python generate_messages.py [EVT_FILE] [EVT_FOLDER]

import sys
import os

# %%


def create_event(event_name, accumulate_event_map, out_file):
    """
    Create a class with event_name and a function to trigger
    the event. Also add the event name into a initializer list
    for creating a map of event name to the function
    Params:
      event_name         - Name of the the event class to be created
      accumulate_event_map - Variable to save initializer list into
      out_file           - Write the event class to out_file
    """
    print >>out_file, (
        "  struct {0} {{}};\n"
        "  template<class LogicStateMachine> \n"
        "  void generate_{0}(LogicStateMachine &logic_state_machine) {{\n"
        "    {0} evt;\n"
        "    logic_state_machine.process_event(evt);\n"
        "  }}\n").format(event_name)
    accumulate_event_map.append(
        '{{"{0}", generate_{0}<LogicStateMachine>}}'.format(event_name))


def create_sub_event_managers(
        accumulate_event_manager_classes,
        accumulate_event_manager_names,
        out_file):
    for i, event_manager_class in enumerate(accumulate_event_manager_classes):
        event_manager_name = accumulate_event_manager_names[i]
        print >>out_file, ("    {0} {1};").format(event_manager_class,
                                                  event_manager_name,)


def create_sub_event_manager_triggers(
        accumulate_event_manager_names, out_file):
    for event_manager_name in accumulate_event_manager_names:
        print >>out_file, (
            "      if(!event_found) {{\n"
            "        event_found = {0}.triggerEvent(event_name, logic_state_machine);\n"
            "      }}").format(event_manager_name,)


def create_sub_event_sets(accumulate_event_manager_names, out_file):
    for event_manager_name in accumulate_event_manager_names:
        print >>out_file, (
            "      {{\n"
            "        std::set<std::string> sub_event_set = {0}.getEventSet();\n"
            "        event_set.insert(sub_event_set.begin(), sub_event_set.end());\n"
            "      }}").format(event_manager_name,)


def create_event_manager(event_manager_name, accumulate_event_map,
                         accumulate_event_manager_classes,
                         accumulate_event_manager_names, out_file):
    """
    Create event manager class. This class provides functions to trigger
    an event by name, and print all the events managed by the class. The
    event manager is enclosed by event_file_name namespace to distinguish
    between different event files containing similar events and event managers
    """
    print >>out_file, (
        "  template<class LogicStateMachine>\n"
        "  class {0} {{\n"
        "    typedef std::function<void(LogicStateMachine&)> EventFunction;\n"
        "    std::unordered_map<std::string, EventFunction> event_map = {{\n\t{1},\n"
        "    }};").format(
        event_manager_name, ',\n\t'.join(accumulate_event_map),)

    create_sub_event_managers(accumulate_event_manager_classes,
                              accumulate_event_manager_names, out_file)

    print >>out_file, (
        "    public:\n"
        "    bool triggerEvent(std::string event_name,"
        " LogicStateMachine& logic_state_machine) {\n"
        "      bool event_found = false;\n"
        "      auto node = event_map.find(event_name);\n"
        "      if(node != event_map.end()) {\n"
        "        event_map[event_name](logic_state_machine);\n"
        "        event_found = true;\n"
        "      }")

    create_sub_event_manager_triggers(accumulate_event_manager_names, out_file)

    print >>out_file, (
        "      return event_found;\n"
        "    }\n"
        "    std::set<std::string> getEventSet() {\n"
        "      std::set<std::string> event_set;\n"
        "      for(auto &s : event_map) {\n"
        "         event_set.insert(s.first);\n"
        "      }\n\n"
        "      //Generate Event set for sub machines if present")

    create_sub_event_sets(accumulate_event_manager_names, out_file)

    print >>out_file, (
        "      return event_set;\n"
        "    }\n"
        "  };")


def check_event_manager(event_name):
    event_manager_tuple = event_name.split('/')
    if len(event_manager_tuple) == 2:
        return True
    return False


def create_sub_event_manager(event_name, accumulate_event_manager_classes,
                             accumulate_event_manager_names, out_file):
    event_manager_tuple = event_name.split('/')
    if len(event_manager_tuple) == 2:
        print >>out_file, (
            "#include <aerial_autonomy/{0}.h>"
        ).format(event_manager_tuple[0],)
        accumulate_event_manager_classes.append(
            '::'.join(event_manager_tuple) + '<LogicStateMachine>')
        accumulate_event_manager_names.append(
            event_manager_tuple[0] + '_manager')


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Cannot create a header file without events and event folder"
        print sys.argv
        sys.exit(-1)
    # Open evt file
    f = open(sys.argv[1], 'r')
    evt_file_base = os.path.basename(sys.argv[1])
    evt_file_wext = os.path.splitext(evt_file_base)[0] + '.h'
    # Create header file
    out_file = open(os.path.join(sys.argv[2], evt_file_wext), 'w')
    # Print header
    print >> out_file, (
        "#pragma once\n"
        "/** Auto generated header file from event file\n"
        "  Do not edit this file manually **/\n"
        "#include <functional>\n"
        "#include <iostream>\n"
        "#include <stdexcept>\n"
        "#include <set>\n"
        "#include <unordered_map>"
    )
    # Enclose the events and event manager into namespace
    accumulate_event_map = []
    accumulate_event_manager_classes = []
    accumulate_event_manager_names = []
    event_names_list = f.read().splitlines()
    # Add sub event managers if exist
    print >>out_file, ""
    for event_name in event_names_list[1:]:
        event_name = event_name.strip()
        create_sub_event_manager(event_name, accumulate_event_manager_classes,
                                 accumulate_event_manager_names, out_file)
    print >>out_file, ""
    print >> out_file, "namespace %s {" % (evt_file_base,)
    event_manager_name = event_names_list[0][:-1]
    for event_name in event_names_list[1:]:
        event_name = event_name.strip()
        if not check_event_manager(event_name):
            create_event(event_name, accumulate_event_map, out_file)
    print >>out_file, "\n//Event manager class"
    create_event_manager(event_manager_name, accumulate_event_map,
                         accumulate_event_manager_classes,
                         accumulate_event_manager_names, out_file)
    print >>out_file, "}"
    out_file.close()
