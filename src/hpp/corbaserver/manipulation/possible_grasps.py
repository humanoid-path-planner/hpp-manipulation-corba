#!/usr/bin/env python
#
# Copyright (c) 2023 CNRS
# Author: Florent Lamiraux
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.


# Define possible grasps for the ConstraintGraphFactory
#
# In some applications, attachment between objects is modeled with grippers and
# handles. Those handles are not meant to be grasped by a robot gripper.
# This class helps defining in a simple way which gripper can grasp which
# handle.
class PossibleGrasps(object):
    # Constructor
    # \param grasps a dictionaty whose keys are the grippers registered in the
    #        factory and whose values are lists of handles also registered in
    #        the factory
    def __init__(self, grippers, handles, grasps):
        self.possibleGrasps = list()
        for ig, gripper in enumerate(grippers):
            handles_ = grasps.get(gripper, list())
            handleIndices = list()
            handleIndices = list(map(handles.index, handles_))
            self.possibleGrasps.append(handleIndices)

    def __call__(self, grasps):
        for ig, ih in enumerate(grasps):
            if not ih is None and not ih in self.possibleGrasps[ig]:
                return False
        return True
