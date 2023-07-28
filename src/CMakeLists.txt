# Copyright (c) 2012, 2020 CNRS Author: Florent Lamiraux, Guilhem Saurel
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# 1. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set(LIBRARY_NAME ${PROJECT_NAME})
set(IDL_SOURCES gcommon graph problem robot)

include(${HPP_CORBASERVER_CMAKE_PLUGIN})
include(${HPP_CORBASERVER_OMNIIDL_CMAKE})
omniidl_include_directories(${CMAKE_SOURCE_DIR}/idl
                            ${HPP_CORBASERVER_DATAROOTDIR}/idl)

include_directories(${CMAKE_BINARY_DIR}/src)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation)
foreach(IDL ${IDL_SOURCES})
  generate_idl_cpp(
    hpp/corbaserver/manipulation/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation ARGUMENTS
    -Wbguard_prefix=hpp_manipulation_corba_idl HEADER_SUFFIX -idl.hh)
  generate_idl_python(
    ${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/corbaserver/manipulation
    ENABLE_DOCSTRING
    STUBS
    hpp_stubs.corbaserver.manipulation
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs
    -Wbextern=robots:hpp_stubs.pinocchio
    -Wbextern=constraints:hpp_stubs.constraints
    -Wbextern=paths:hpp_stubs.core
    -Wbextern=steering_methods:hpp_stubs.core
    -Wbextern=path_projectors:hpp_stubs.core
    -Wbextern=path_validations:hpp_stubs.core
    -Wbextern=_constraints:hpp_stubs.core
    -Wbextern=path_planners:hpp_stubs.core
    -Wbextern=distances:hpp_stubs.core
    -Wbextern=_graph:hpp_stubs.manipulation
    -Wbextern=_path_planners:hpp_stubs.manipulation)
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/corbaserver/manipulation/${IDL}-idl.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/corbaserver/manipulation)
endforeach()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp/manipulation_idl)
foreach(IDL steering_methods _graph _problem _path_planners device)
  generate_idl_cpp(
    hpp/manipulation_idl/${IDL} ${CMAKE_SOURCE_DIR}/idl/hpp/manipulation_idl
    ARGUMENTS -Wbguard_prefix=hpp_manipulation_idl HEADER_SUFFIX -idl.hh)
  generate_idl_cpp_impl(
    hpp/manipulation_idl/${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/manipulation_idl
    ARGUMENTS
    -Wbguard_prefix=hpp_manipulation_idl
    -Wbinc_prefix=hpp/manipulation_idl
    HH_SUFFIX
    -idl.hh)
  generate_idl_python(
    ${IDL}
    ${CMAKE_SOURCE_DIR}/idl/hpp/manipulation_idl
    ENABLE_DOCSTRING
    STUBS
    hpp_stubs.manipulation
    ARGUMENTS
    -Wbmodules=hpp_idl
    -Wbextern=common:hpp_stubs
    -Wbextern=robots:hpp_stubs.pinocchio
    -Wbextern=gcommon:hpp_stubs.corbaserver.manipulation
    -Wbextern=paths:hpp_stubs.core
    -Wbextern=steering_methods:hpp_stubs.core
    -Wbextern=distances:hpp_stubs.core
    -Wbextern=path_projectors:hpp_stubs.core
    -Wbextern=path_validations:hpp_stubs.core
    -Wbextern=constraints:hpp_stubs.constraints
    -Wbextern=configuration_shooters:hpp_stubs.core
    -Wbextern=_constraints:hpp_stubs.core
    -Wbextern=_problem:hpp_stubs.core
    -Wbextern=path_planners:hpp_stubs.core)
  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/hpp/manipulation_idl/${IDL}-idl.hh
          ${CMAKE_CURRENT_BINARY_DIR}/hpp/manipulation_idl/${IDL}.hh
          ${CMAKE_CURRENT_BINARY_DIR}/hpp/manipulation_idl/${IDL}-fwd.hh
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hpp/manipulation_idl)
endforeach()

install(DIRECTORY ${CMAKE_SOURCE_DIR}/idl/hpp
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/idl/)
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl/hpp/corbaserver/manipulation
  DESTINATION ${PYTHON_SITELIB}/hpp_idl/hpp/corbaserver)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_idl/hpp/manipulation_idl
        DESTINATION ${PYTHON_SITELIB}/hpp_idl/hpp/)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs/corbaserver/manipulation
        DESTINATION ${PYTHON_SITELIB}/hpp_stubs/corbaserver)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/hpp_stubs/manipulation
        DESTINATION ${PYTHON_SITELIB}/hpp_stubs)

add_custom_target(generate_idl_cpp DEPENDS ${ALL_IDL_CPP_STUBS}
                                           ${ALL_IDL_CPP_IMPL_STUBS})
add_custom_target(generate_idl_python DEPENDS ${ALL_IDL_PYTHON_STUBS})

add_library(${LIBRARY_NAME} SHARED ${ALL_IDL_CPP_STUBS} client.cc)

target_link_libraries(${LIBRARY_NAME} PUBLIC hpp-corbaserver::hpp-corbaserver)
pkg_config_use_dependency(${LIBRARY_NAME} omniORB4)

install(
  TARGETS ${LIBRARY_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

if(NOT CLIENT_ONLY)
  # Server plugin
  hpp_add_server_plugin(
    manipulation-corba
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
    LINK_DEPENDENCIES
    PUBLIC
    ${LIBRARY_NAME}
    hpp-manipulation-urdf::hpp-manipulation-urdf
    PKG_CONFIG_DEPENDENCIES
    omniORB4)

  # Stand alone corba server
  add_executable(hpp-manipulation-server hpp-manipulation-corba.cc)
  target_link_libraries(
    hpp-manipulation-server PUBLIC ${LIBRARY_NAME}
                                   hpp-manipulation-urdf::hpp-manipulation-urdf)

  install(TARGETS hpp-manipulation-server DESTINATION ${CMAKE_INSTALL_BINDIR})
endif(NOT CLIENT_ONLY)

add_dependencies(${LIBRARY_NAME} generate_idl_cpp)
add_dependencies(${LIBRARY_NAME} generate_idl_python)

python_install_on_site(hpp/corbaserver/manipulation __init__.py)
python_install_on_site(hpp/corbaserver/manipulation client.py)
python_install_on_site(hpp/corbaserver/manipulation problem_solver.py)
python_install_on_site(hpp/corbaserver/manipulation robot.py)
python_install_on_site(hpp/corbaserver/manipulation constraints.py)
python_install_on_site(hpp/corbaserver/manipulation constraint_graph.py)
python_install_on_site(hpp/corbaserver/manipulation constraint_graph_factory.py)
python_install_on_site(hpp/corbaserver/manipulation possible_grasps.py)
python_install_on_site(hpp/corbaserver/manipulation security_margins.py)
