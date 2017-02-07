# Create Event header file based on event file
# The Event file is formatted as follows:
#  EventManagerName:
#     Event1Name
#     Event2Name
#     ...
#     EventNName
# Provide only the file names for event files without any folder or file paths.
# All events are assumed to be in events folder.
# Example generate_event_targets(event_file_name) for event_file_name
# is a file in events folder in project source directory.
# Defines ${PROJECT_NAME}_event_target for adding the event files as a target
# Also extends ${PROJECT_NAME}_EXPORTED_TARGETS} variable which is common for
# any generated files in project: http://wiki.ros.org/catkin/CMakeLists.txt
function(generate_event_targets)
  # If directory to store event files does not exist, create directory
  set(EVENT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}")
  if (NOT (EXISTS ${EVENT_DIRECTORY}))
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E make_directory ${EVENT_DIRECTORY}
        )
  endif(NOT (EXISTS ${EVENT_DIRECTORY}))

  foreach(event_file_name ${ARGN})
    add_custom_command(OUTPUT ${EVENT_DIRECTORY}/${event_file_name}.h
      COMMAND python ${PROJECT_SOURCE_DIR}/scripts/generate_event_file.py ${PROJECT_SOURCE_DIR}/events/${event_file_name} ${EVENT_DIRECTORY}
      COMMENT "Creating custom event: ${event_file_name}"
      DEPENDS ${PROJECT_SOURCE_DIR}/scripts/generate_event_file.py
        ${PROJECT_SOURCE_DIR}/events/${event_file_name}
      )
    set(EVENT_FILE_NAME_LIST ${EVENT_FILE_NAME_LIST} ${EVENT_DIRECTORY}/${event_file_name}.h)
  endforeach(event_file_name)

  add_custom_target(${PROJECT_NAME}_event_target ALL
                    DEPENDS ${EVENT_FILE_NAME_LIST}
                    )
  install(DIRECTORY ${EVENT_DIRECTORY}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
          PATTERN ".svn" EXCLUDE
          )
  set(${PROJECT_NAME}_EXPORTED_TARGETS ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_event_target PARENT_SCOPE)
endfunction(generate_event_targets)
