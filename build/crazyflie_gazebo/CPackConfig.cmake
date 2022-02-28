# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. The list of available CPACK_xxx variables and their associated
# documentation may be obtained using
#  cpack --help-variable-list
#
# Some variables are common to all generators (e.g. CPACK_PACKAGE_NAME)
# and some are specific to a generator
# (e.g. CPACK_NSIS_EXTRA_INSTALL_COMMANDS). The generator specific variables
# usually begin with CPACK_<GENNAME>_xxxx.


SET(CPACK_BINARY_7Z "")
SET(CPACK_BINARY_BUNDLE "")
SET(CPACK_BINARY_CYGWIN "")
SET(CPACK_BINARY_DEB "OFF")
SET(CPACK_BINARY_DRAGNDROP "")
SET(CPACK_BINARY_FREEBSD "OFF")
SET(CPACK_BINARY_IFW "OFF")
SET(CPACK_BINARY_NSIS "OFF")
SET(CPACK_BINARY_OSXX11 "")
SET(CPACK_BINARY_PACKAGEMAKER "")
SET(CPACK_BINARY_PRODUCTBUILD "")
SET(CPACK_BINARY_RPM "OFF")
SET(CPACK_BINARY_STGZ "ON")
SET(CPACK_BINARY_TBZ2 "OFF")
SET(CPACK_BINARY_TGZ "ON")
SET(CPACK_BINARY_TXZ "OFF")
SET(CPACK_BINARY_TZ "ON")
SET(CPACK_BINARY_WIX "")
SET(CPACK_BINARY_ZIP "")
SET(CPACK_BUILD_SOURCE_DIRS "/home/pascal/dd2419_ws/src/course_packages/sim_cf/crazyflie_gazebo;/home/pascal/dd2419_ws/build/crazyflie_gazebo")
SET(CPACK_CMAKE_GENERATOR "Unix Makefiles")
SET(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
SET(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
SET(CPACK_DEBIAN_PACKAGE_DESCRIPTION "gazebo plugins for cf sitl.")
SET(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")
SET(CPACK_DEBIAN_PACKAGE_SHLIBDEPS "ON")
SET(CPACK_GENERATOR "STGZ;TGZ;TZ")
SET(CPACK_INSTALL_CMAKE_PROJECTS "/home/pascal/dd2419_ws/build/crazyflie_gazebo;crazyflie_gazebo;ALL;/")
SET(CPACK_INSTALL_PREFIX "/home/pascal/dd2419_ws/install")
SET(CPACK_MODULE_PATH "/usr/share/OGRE/cmake/modules;/usr/lib/OGRE/cmake;/usr/CMake;/usr/local/share/cmake/Modules;/home/pascal/dd2419_ws/src/course_packages/sim_cf/crazyflie_gazebo/cmake;/usr/lib/x86_64-linux-gnu/cmake/ignition-cmake0/cmake0;/opt/ros/melodic/share/cmake_modules/cmake/../../../share/cmake_modules/cmake/Modules")
SET(CPACK_NSIS_DISPLAY_NAME "crazyflie_gazebo-9 0.1.1")
SET(CPACK_NSIS_INSTALLER_ICON_CODE "")
SET(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
SET(CPACK_NSIS_PACKAGE_NAME "crazyflie_gazebo-9 0.1.1")
SET(CPACK_OUTPUT_CONFIG_FILE "/home/pascal/dd2419_ws/build/crazyflie_gazebo/CPackConfig.cmake")
SET(CPACK_PACKAGE_CONTACT "pxusers@googlegroups.com")
SET(CPACK_PACKAGE_DEFAULT_LOCATION "/")
SET(CPACK_PACKAGE_DESCRIPTION_FILE "/usr/share/cmake-3.10/Templates/CPack.GenericDescription.txt")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "crazyflie_gazebo built using CMake")
SET(CPACK_PACKAGE_FILE_NAME "crazyflie_gazebo-9-")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "crazyflie_gazebo-9 0.1.1")
SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "crazyflie_gazebo-9 0.1.1")
SET(CPACK_PACKAGE_NAME "crazyflie_gazebo-9")
SET(CPACK_PACKAGE_RELOCATABLE "true")
SET(CPACK_PACKAGE_VENDOR "Humanity")
SET(CPACK_PACKAGE_VERSION "0.1.1")
SET(CPACK_PACKAGE_VERSION_MAJOR "0")
SET(CPACK_PACKAGE_VERSION_MINOR "1")
SET(CPACK_PACKAGE_VERSION_PATCH "1")
SET(CPACK_RESOURCE_FILE_LICENSE "/usr/share/cmake-3.10/Templates/CPack.GenericLicense.txt")
SET(CPACK_RESOURCE_FILE_README "/usr/share/cmake-3.10/Templates/CPack.GenericDescription.txt")
SET(CPACK_RESOURCE_FILE_WELCOME "/usr/share/cmake-3.10/Templates/CPack.GenericWelcome.txt")
SET(CPACK_RPM_PACKAGE_DESCRIPTION "Gazebo plugins for cf sitl.")
SET(CPACK_RPM_PACKAGE_REQUIRES "")
SET(CPACK_SET_DESTDIR "OFF")
SET(CPACK_SOURCE_7Z "")
SET(CPACK_SOURCE_CYGWIN "")
SET(CPACK_SOURCE_GENERATOR "TBZ2;TGZ;TXZ;TZ")
SET(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/home/pascal/dd2419_ws/build/crazyflie_gazebo/CPackSourceConfig.cmake")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "crazyflie_gazebo-9-")
SET(CPACK_SOURCE_RPM "OFF")
SET(CPACK_SOURCE_TBZ2 "ON")
SET(CPACK_SOURCE_TGZ "ON")
SET(CPACK_SOURCE_TXZ "ON")
SET(CPACK_SOURCE_TZ "ON")
SET(CPACK_SOURCE_ZIP "OFF")
SET(CPACK_SYSTEM_NAME "Linux")
SET(CPACK_TOPLEVEL_TAG "Linux")
SET(CPACK_WIX_SIZEOF_VOID_P "8")

if(NOT CPACK_PROPERTIES_FILE)
  set(CPACK_PROPERTIES_FILE "/home/pascal/dd2419_ws/build/crazyflie_gazebo/CPackProperties.cmake")
endif()

if(EXISTS ${CPACK_PROPERTIES_FILE})
  include(${CPACK_PROPERTIES_FILE})
endif()
