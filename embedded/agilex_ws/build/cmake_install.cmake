# Install script for directory: /home/nvidia/INF3995-104/embedded/agilex_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nvidia/INF3995-104/embedded/agilex_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/INF3995-104/embedded/agilex_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/INF3995-104/embedded/agilex_ws/install" TYPE PROGRAM FILES "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/INF3995-104/embedded/agilex_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/INF3995-104/embedded/agilex_ws/install" TYPE PROGRAM FILES "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/INF3995-104/embedded/agilex_ws/install/setup.bash;/home/nvidia/INF3995-104/embedded/agilex_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/INF3995-104/embedded/agilex_ws/install" TYPE FILE FILES
    "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/catkin_generated/installspace/setup.bash"
    "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/INF3995-104/embedded/agilex_ws/install/setup.sh;/home/nvidia/INF3995-104/embedded/agilex_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/INF3995-104/embedded/agilex_ws/install" TYPE FILE FILES
    "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/catkin_generated/installspace/setup.sh"
    "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/INF3995-104/embedded/agilex_ws/install/setup.zsh;/home/nvidia/INF3995-104/embedded/agilex_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/INF3995-104/embedded/agilex_ws/install" TYPE FILE FILES
    "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/catkin_generated/installspace/setup.zsh"
    "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/nvidia/INF3995-104/embedded/agilex_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/nvidia/INF3995-104/embedded/agilex_ws/install" TYPE FILE FILES "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/gtest/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/limo_ros/limo_bringup/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/detect_mark/ar_track_alvar/ar_track_alvar_msgs/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/voice/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/agilex_pure_pursuit/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/environment_setting/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/src/scripts/identification/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/launch_pkg/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/limo_ros/learning_limo/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/lifter_ctr/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/robot_identification/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/src/cv_bridge/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/src/darknet_ros/darknet_ros_msgs/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/ros_astra_camera/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/src/camera_info_manager_py/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/src/darknet_ros/darknet_ros/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/limo_ros/limo_base/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/detect_mark/ar_track_alvar/ar_track_alvar/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/detect_mark/detect_ros/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/src/scripts/vision/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/ydlidar_ros/cmake_install.cmake")
  include("/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/src/scripts/your_package_name/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/mnt/home/nvidia/INF3995-104/embedded/agilex_ws/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
