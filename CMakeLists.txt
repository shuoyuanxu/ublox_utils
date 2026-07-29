cmake_minimum_required(VERSION 3.16.3)
project(ublox_utils)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
        scripts/rtk_status.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
