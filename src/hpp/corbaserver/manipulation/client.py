#!/usr/bin/env python
#
# Copyright (c) 2014 CNRS
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

from omniORB import CORBA
import CosNaming

from hpp.corbaserver.client import _getIIOPurl
from hpp.corbaserver.manipulation import Graph, Robot, Problem

class CorbaError(Exception):
    """
    Raised when a CORBA error occurs.
    """
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class Client:
  """
  Connect and create clients for hpp-manipulation library.
  """
  def __init__(self, url = None, postContextId = ""):
    """
    Initialize CORBA and create default clients.
    :param url: URL in the IOR, corbaloc, corbalocs, and corbanames formats.
                For a remote corba server, use
                url = "corbaloc:iiop:<host>:<port>/NameService"
    """
    import sys
    self.orb = CORBA.ORB_init (sys.argv, CORBA.ORB_ID)
    if url is None:
        obj = self.orb.string_to_object (_getIIOPurl())
    else:
        obj = self.orb.string_to_object (url)
    self.rootContext = obj._narrow(CosNaming.NamingContext)
    if self.rootContext is None:
        raise CorbaError ('failed to narrow the root context')

    rtContextId = "hpp" + postContextId;

    # client of Graph interface
    name = [CosNaming.NameComponent (rtContextId, "corbaserver"),
            CosNaming.NameComponent ("manipulation", "graph")]

    try:
        obj = self.rootContext.resolve (name)
    except CosNaming.NamingContext.NotFound, ex:
        raise CorbaError ('failed to find manipulation service.')
    try:
        client = obj._narrow (Graph)
    except KeyError:
        raise CorbaError ('invalid service name manipulation')

    if client is None:
      # This happens when stubs from client and server are not synchronized.
        raise CorbaError (
            'failed to narrow client for service manipulation')
    self.graph = client

    # client of Problem interface
    name = [CosNaming.NameComponent (rtContextId, "corbaserver"),
            CosNaming.NameComponent ("manipulation", "problem")]
    
    try:
        obj = self.rootContext.resolve (name)
    except CosNaming.NamingContext.NotFound, ex:
        raise CorbaError ('failed to find manipulation service.')
    try:
        client = obj._narrow (Problem)
    except KeyError:
        raise CorbaError ('invalid service name manipulation')

    if client is None:
      # This happens when stubs from client and server are not synchronized.
        raise CorbaError (
            'failed to narrow client for service manipulation')
    self.problem = client

    # client of Robot interface
    name = [CosNaming.NameComponent (rtContextId, "corbaserver"),
            CosNaming.NameComponent ("manipulation", "robot")]
    
    try:
        obj = self.rootContext.resolve (name)
    except CosNaming.NamingContext.NotFound, ex:
        raise CorbaError ('failed to find manipulation service.')
    try:
        client = obj._narrow (Robot)
    except KeyError:
        raise CorbaError ('invalid service name manipulation')

    if client is None:
      # This happens when stubs from client and server are not synchronized.
        raise CorbaError (
            'failed to narrow client for service manipulation')
    self.robot = client
