execute_process(COMMAND "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/src/cv_bridge/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/src/cv_bridge/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
