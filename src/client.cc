// Copyright (c) 2015, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include "hpp/corbaserver/manipulation/client.hh"

#include <iostream>

namespace hpp {
namespace corbaServer {
namespace manipulation {
using CORBA::Exception;
using CORBA::Object_var;
using CORBA::ORB_init;
using CORBA::PolicyList;
using CORBA::SystemException;
using omniORB::fatalException;

Client::Client(int argc, char* argv[]) : ClientBase(argc, argv) {}

void Client::connect(const char* iiop, const char* context) {
  ClientBase::connect(iiop);

  CORBA::Object_var obj;
  const char* plugin = "manipulation";

  obj = tools()->getServer(context, plugin, "robot");
  robot_ = hpp::corbaserver::manipulation::Robot::_narrow(obj.in());

  obj = tools()->getServer(context, plugin, "problem");
  problem_ = hpp::corbaserver::manipulation::Problem::_narrow(obj.in());

  obj = tools()->getServer(context, plugin, "graph");
  graph_ = hpp::corbaserver::manipulation::Graph::_narrow(obj.in());
}

/// \brief Shutdown CORBA server
Client::~Client() {}
}  // end of namespace manipulation.
}  // end of namespace corbaServer.
}  // end of namespace hpp.
