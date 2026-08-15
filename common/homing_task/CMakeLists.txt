if(COMMAND nuttx_add_library)
  nuttx_add_library(homing_task STATIC homing_task.c)
  
  target_include_directories(homing_task PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  )
  
  target_include_directories(homing_task PRIVATE
    ${REPO_ROOT}/boards/stm32f411e-disco/src
    ${REPO_ROOT}/common/safety_task
  )
  
  target_link_libraries(homing_task PRIVATE system_state motor_pos safety_task)
endif()
