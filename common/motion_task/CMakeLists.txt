if(COMMAND nuttx_add_library)
  nuttx_add_library(motion_task STATIC motion_task.c)

  target_include_directories(motion_task PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  target_include_directories(motion_task PRIVATE
    ${REPO_ROOT}/common
    ${REPO_ROOT}/boards/stm32f411e-disco/src
  )

  target_link_libraries(motion_task PRIVATE
    motor_pos
    system_state
    safety_task
  )
endif()
