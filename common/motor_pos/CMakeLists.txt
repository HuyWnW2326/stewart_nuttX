if(COMMAND nuttx_add_library)
  nuttx_add_library(motor_pos STATIC motor_pos.c)
  target_include_directories(motor_pos PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()