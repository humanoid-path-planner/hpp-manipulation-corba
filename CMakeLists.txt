# Copyright (c) 2012, 2020 CNRS
# Author: Florent Lamiraux, Guilhem Saurel
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

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAME hpp-manipulation-corba)
SET(PROJECT_DESCRIPTION "Corba server for manipulation planning")

SET(CLIENT_ONLY FALSE CACHE BOOL "Set to true to install the client only")
SET(CUSTOM_HEADER_DIR hpp/corbaserver/manipulation)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)
SET(CXX_DISABLE_WERROR true)

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/idl.cmake)
INCLUDE(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

LIST(APPEND PKG_CONFIG_ADDITIONAL_VARIABLES cmake_plugin)
CONFIG_FILES(doc/main.hh)

SET(${PROJECT_NAME}_HEADERS
  include/hpp/corbaserver/manipulation/client.hh
  )

IF(NOT CLIENT_ONLY)
  SET(${PROJECT_NAME}_HEADERS
    include/hpp/corbaserver/manipulation/server.hh
    include/hpp/corbaserver/manipulation/fwd.hh
    ${${PROJECT_NAME}_HEADERS}
    )
ENDIF(NOT CLIENT_ONLY)

# search for python
FINDPYTHON()

ADD_PROJECT_DEPENDENCY("hpp-corbaserver" REQUIRED)
IF(NOT CLIENT_ONLY)
    ADD_PROJECT_DEPENDENCY("hpp-manipulation" REQUIRED)
    ADD_PROJECT_DEPENDENCY("hpp-manipulation-urdf" REQUIRED)
ENDIF(NOT CLIENT_ONLY)
ADD_REQUIRED_DEPENDENCY("omniORB4 >= 4.1.4")

ADD_SUBDIRECTORY(src)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
