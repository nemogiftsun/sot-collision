# Copyright 2010, Florent Lamiraux, CNRS
#
# This file is part of sot-core.
# sot-core is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-core is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph-tutorial. If not, see <http://www.gnu.org/licenses/>.
# This is required by the Python prologue.



# Libraries DEFINITIONS and INCLUDES

INCLUDE(../cmake/python.cmake)
FINDPYTHON()
SET(LIBRARY_NAME ${PROJECT_NAME} )

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include ${PYTHON_INCLUDE_DIRS})

# provide path to dependency libraries
LINK_DIRECTORIES(${DYNAMIC_GRAPH_PYTHON_LIBRARY_DIRS} ${PYTHON_LIBRARY_DIRS})


ADD_LIBRARY(${LIBRARY_NAME}
  sot-collision.cc
  test_fcl_utility.cpp
  )
  

SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH True)

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)



# Add dynamic-graph-python compilation flags and link to dependency libraries
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph-python)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} fcl)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} sot-core)

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

SET(NEW_ENTITY_CLASS
  SotCollision
  )

DYNAMIC_GRAPH_PYTHON_MODULE("sotcollision" ${LIBRARY_NAME} wrap)

PKG_CONFIG_USE_DEPENDENCY(wrap dynamic-graph-python)
PKG_CONFIG_USE_DEPENDENCY(wrap dynamic-graph)
PKG_CONFIG_USE_DEPENDENCY(wrap fcl)

message(dyngrap ${CATKIN_PACKAGE_BIN_DESTINATION})


install(PROGRAMS ${CMAKE_SOURCE_DIR}/script/helloscript
                 ${CMAKE_SOURCE_DIR}/src/sot-collision/demo.py
                 ${CMAKE_SOURCE_DIR}/src/sot-collision/ur.py
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
