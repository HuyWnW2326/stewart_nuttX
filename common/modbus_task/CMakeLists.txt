if(COMMAND nuttx_add_library)
  nuttx_add_library(modbus_task STATIC modbus_task.c)
  
  target_include_directories(modbus_task PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  )
  
  target_link_libraries(modbus_task PRIVATE motor_pos)
endif()
