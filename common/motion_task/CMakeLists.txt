if(EXISTS ${NUTTX_APPS_BINDIR}/Kconfig)
  nuttx_add_application(
    NAME motion_task
    SRCS motion_task.c
    INCLUDE_DIRECTORIES
      ${CMAKE_CURRENT_SOURCE_DIR}
  )

  target_link_libraries(apps_motion_task PRIVATE motor_pos)
endif()
