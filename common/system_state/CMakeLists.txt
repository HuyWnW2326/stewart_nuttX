if(COMMAND nuttx_add_library)
  nuttx_add_library(system_state STATIC system_state.c)
  
  target_include_directories(system_state PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
  )
  
  target_include_directories(system_state PRIVATE
    ${REPO_ROOT}/boards/stm32f411e-disco/src
  )
endif()
