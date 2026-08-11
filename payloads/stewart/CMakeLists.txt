if(EXISTS ${NUTTX_APPS_BINDIR}/Kconfig)
  nuttx_add_application(
    NAME stewart_payload
    SRCS stewart_main.c
    INCLUDE_DIRECTORIES
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${REPO_ROOT}/common
      ${REPO_ROOT}/boards/stm32f411e-disco/src
  )
  target_link_libraries(apps_stewart_payload PRIVATE motor_pos modbus_task system_state homing_task)

endif()