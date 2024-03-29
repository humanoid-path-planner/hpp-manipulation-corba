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

cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME hpp-manipulation-corba)
set(PROJECT_DESCRIPTION "Corba server for manipulation planning")

set(CLIENT_ONLY
    FALSE
    CACHE BOOL "Set to true to install the client only")
set(CUSTOM_HEADER_DIR hpp/corbaserver/manipulation)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)
set(CXX_DISABLE_WERROR true)

include(cmake/hpp.cmake)
include(cmake/idl.cmake)
include(cmake/python.cmake)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

list(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)
config_files(doc/main.hh)

set(${PROJECT_NAME}_HEADERS include/hpp/corbaserver/manipulation/client.hh)

if(NOT CLIENT_ONLY)
  set(${PROJECT_NAME}_HEADERS
      include/hpp/corbaserver/manipulation/server.hh
      include/hpp/corbaserver/manipulation/fwd.hh ${${PROJECT_NAME}_HEADERS})
endif(NOT CLIENT_ONLY)

add_project_dependency("hpp-corbaserver" REQUIRED)
if(NOT CLIENT_ONLY)
  add_project_dependency("hpp-manipulation" REQUIRED)
  add_project_dependency("hpp-manipulation-urdf" REQUIRED)
endif(NOT CLIENT_ONLY)
add_required_dependency("omniORB4 >= 4.1.4")

# search for python
if(NOT FINDPYTHON_ALREADY_CALLED)
  findpython()
endif()

add_subdirectory(src)

pkg_config_append_libs(${PROJECT_NAME})

install(FILES package.xml DESTINATION share/${PROJECT_NAME})
