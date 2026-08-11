if(COMMAND nuttx_add_library)
  nuttx_add_library(safety_task STATIC safety_task.c)
  
  target_include_directories(safety_task PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  )
  
  target_include_directories(safety_task PRIVATE
    ${REPO_ROOT}/boards/stm32f411e-disco/src
  )
  
  target_link_libraries(safety_task PRIVATE motor_pos system_state homing_task)
endif()
