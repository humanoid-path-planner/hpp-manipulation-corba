# Copyright (c) 2012 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-manipulation-corba.
# hpp-manipulation-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation-corba.  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})
SET(IDL_SOURCES
  gcommon
  graph
  problem
  robot
)


# search for python
FINDPYTHON(2.7 EXACT REQUIRED)

OMNIIDL_INCLUDE_DIRECTORIES(
  ${HPP_CORBASERVER_DATAROOTDIR}/idl ${CMAKE_SOURCE_DIR}/idl
  )

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)
FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation)

FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_CPP (hpp/corbaserver/manipulation/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation)
  GENERATE_IDL_PYTHON (hpp/corbaserver/manipulation/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation)
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}_idl.py
    DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation)
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver/manipulation)
ENDFOREACH()

# Generate and install common_idl.py from hpp.corbaserver
GENERATE_IDL_PYTHON (hpp/corbaserver/manipulation/common
  ${HPP_CORBASERVER_DATAROOTDIR}/idl/hpp/corbaserver)

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

IF (NOT CLIENT_ONLY)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/gcommon.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/graph.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/robot.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/problem.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/gcommonSK.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/graphSK.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/robotSK.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/problemSK.cc
    graph.impl.cc
    graph.impl.hh
    problem.impl.cc
    problem.impl.hh
    robot.impl.cc
    robot.impl.hh
    server.cc
    client.cc
    tools.cc
    )

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-manipulation)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-manipulation-urdf)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)

  INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

  # Stand alone corba server
  ADD_EXECUTABLE (hpp-manipulation-server hpp-manipulation-corba.cc)
  TARGET_LINK_LIBRARIES (hpp-manipulation-server ${LIBRARY_NAME})
  PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-manipulation)
  PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-manipulation-urdf)
  PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-corbaserver)
  IF(HPP_MANIPULATION_HAS_WHOLEBODY_STEP)
    PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-wholebody-step-corba)
    ADD_DEFINITIONS (-DHPP_MANIPULATION_HAS_WHOLEBODY_STEP=1)
  ENDIF(HPP_MANIPULATION_HAS_WHOLEBODY_STEP)


  INSTALL (TARGETS hpp-manipulation-server DESTINATION ${CMAKE_INSTALL_BINDIR})
ELSE (NOT CLIENT_ONLY)
  ADD_LIBRARY(${LIBRARY_NAME} SHARED
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/gcommon.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/graph.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/robot.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/problem.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/gcommonSK.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/graphSK.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/robotSK.cc
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/problemSK.cc
    client.cc
    )

  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)
  PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-corbaserver)

  INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
ENDIF (NOT CLIENT_ONLY)

ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_cpp)
ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_python)

INSTALL(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/common_idl.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
  )

INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
  )
INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/client.py
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/problem_solver.py
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/robot.py
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/constraints.py
  ${CMAKE_CURRENT_SOURCE_DIR}/hpp/corbaserver/manipulation/constraint_graph.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation
  )
