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

INCLUDE(${HPP_CORBASERVER_CMAKE_PLUGIN})
INCLUDE(${HPP_CORBASERVER_DATAROOTDIR}/hpp-corbaserver/cmake/omniidl.cmake)
OMNIIDL_INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/idl ${HPP_CORBASERVER_DATAROOTDIR}/idl
  )

INCLUDE_DIRECTORIES(${CMAKE_BINARY_DIR}/src)

FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation)
FOREACH(IDL ${IDL_SOURCES})
  GENERATE_IDL_CPP (hpp/corbaserver/manipulation/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation
    HEADER_SUFFIX -idl.hh)
  GENERATE_IDL_PYTHON (${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation
    ENABLE_DOCSTRING
    STUBS hpp_stubs.corbaserver.manipulation
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs
    -Wbextern=robots:hpp_stubs.pinocchio
    -Wbextern=constraints:hpp_stubs.constraints
    -Wbextern=paths:hpp_stubs.core
    -Wbextern=steering_methods:hpp_stubs.core
    -Wbextern=path_validations:hpp_stubs.core
    -Wbextern=_constraints:hpp_stubs.core
    -Wbextern=_graph:hpp_stubs.manipulation
    )
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}-idl.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver/manipulation)
ENDFOREACH()

FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp/manipulation_idl)
FOREACH(IDL steering_methods _graph _problem)
  GENERATE_IDL_CPP (hpp/manipulation_idl/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/manipulation_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_manipulation_idl
    HEADER_SUFFIX -idl.hh)
  GENERATE_IDL_CPP_IMPL (hpp/manipulation_idl/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/manipulation_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_manipulation_idl
    -Wbinc_prefix=hpp/manipulation_idl
    HH_SUFFIX -idl.hh)
  GENERATE_IDL_PYTHON (${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/manipulation_idl
    ENABLE_DOCSTRING
    STUBS hpp_stubs.manipulation
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs
    -Wbextern=gcommon:hpp_stubs.corbaserver.manipulation
    -Wbextern=paths:hpp_stubs.core
    -Wbextern=steering_methods:hpp_stubs.core
    -Wbextern=distances:hpp_stubs.core
    -Wbextern=path_validations:hpp_stubs.core
    -Wbextern=constraints:hpp_stubs.constraints
    -Wbextern=configuration_shooters:hpp_stubs.core
    -Wbextern=_constraints:hpp_stubs.core
    -Wbextern=_problem:hpp_stubs.core
    )
  INSTALL(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/manipulation_idl/${IDL}-idl.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/manipulation_idl/${IDL}.hh
    ${CMAKE_CURRENT_BINARY_DIR}/hpp/manipulation_idl/${IDL}-fwd.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/manipulation_idl)
ENDFOREACH()

INSTALL(
  DIRECTORY ${CMAKE_SOURCE_DIR}/idl/hpp
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl/)
INSTALL(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl/hpp/corbaserver/manipulation
  DESTINATION ${PYTHON_SITELIB}/hpp_idl/hpp/corbaserver)
INSTALL(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl/hpp/manipulation_idl
  DESTINATION ${PYTHON_SITELIB}/hpp_idl/hpp/)
INSTALL(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs/corbaserver/manipulation
  DESTINATION ${PYTHON_SITELIB}/hpp_stubs/corbaserver)
INSTALL(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs/manipulation
  DESTINATION ${PYTHON_SITELIB}/hpp_stubs)

ADD_CUSTOM_TARGET(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS} ${ALL_IDL_CPP_IMPL_STUBS})
ADD_CUSTOM_TARGET(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  ${ALL_IDL_CPP_STUBS}
  client.cc
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-corbaserver)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} omniORB4)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

IF (NOT CLIENT_ONLY)
  # Server plugin
  HPP_ADD_SERVER_PLUGIN(manipulation-corba
    SOURCES
    ${ALL_IDL_CPP_IMPL_STUBS}
    graph.impl.cc
    graph.impl.hh
    problem.impl.cc
    problem.impl.hh
    robot.impl.cc
    robot.impl.hh
    server.cc
    tools.cc # Should be moved into the library
    LINK_DEPENDENCIES hpp-manipulation-corba
    PKG_CONFIG_DEPENDENCIES omniORB4 hpp-corbaserver hpp-manipulation hpp-manipulation-urdf)


  # Stand alone corba server
  ADD_EXECUTABLE (hpp-manipulation-server hpp-manipulation-corba.cc)
  TARGET_LINK_LIBRARIES (hpp-manipulation-server ${LIBRARY_NAME})
  PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-manipulation)
  PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-manipulation-urdf)
  PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-corbaserver)
  PKG_CONFIG_USE_DEPENDENCY (hpp-manipulation-server hpp-template-corba)

  INSTALL (TARGETS hpp-manipulation-server DESTINATION ${CMAKE_INSTALL_BINDIR})
ENDIF (NOT CLIENT_ONLY)

ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_cpp)
ADD_DEPENDENCIES (${LIBRARY_NAME} generate_idl_python)

PYTHON_INSTALL_ON_SITE(hpp/corbaserver/manipulation __init__.py)
PYTHON_INSTALL_ON_SITE(hpp/corbaserver/manipulation client.py)
PYTHON_INSTALL_ON_SITE(hpp/corbaserver/manipulation problem_solver.py)
PYTHON_INSTALL_ON_SITE(hpp/corbaserver/manipulation robot.py)
PYTHON_INSTALL_ON_SITE(hpp/corbaserver/manipulation constraints.py)
PYTHON_INSTALL_ON_SITE(hpp/corbaserver/manipulation constraint_graph.py)
PYTHON_INSTALL_ON_SITE(hpp/corbaserver/manipulation constraint_graph_factory.py)
