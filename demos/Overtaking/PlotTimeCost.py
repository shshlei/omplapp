#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2012, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Shi Shenglei

import os
import argparse

import numpy as np

if __name__ == "__main__":
    # Create an argument parser
    parser = argparse.ArgumentParser(description='Draw Trajectory.')
    parser.add_argument('-s', '--scenario', default=None, help='Scenario.')
    args = parser.parse_args()

    time1 = []
    for line in open(str('TimeCost/box_time_'+str(args.scenario)+'.txt'), 'r').readlines():
        line = line.strip()
        if not line:
            continue
        t = [float(x) for x in line.split(' ')]
        time1.append(t)

    line = open(str('TimeCost/box_cost_'+str(args.scenario)+'.txt'), 'r').readline()
    line = line.strip()
    costs1 = [float(x) for x in line.split(' ')]

    time2 = []
    for line in open(str('TimeCost/box_ap_time_'+str(args.scenario)+'.txt'), 'r').readlines():
        line = line.strip()
        if not line:
            continue
        t = [float(x) for x in line.split(' ')]
        time2.append(t)

    line = open(str('TimeCost/box_ap_cost_'+str(args.scenario)+'.txt'), 'r').readline()
    line = line.strip()
    costs2 = [float(x) for x in line.split(' ')]

    print('Box')
    meant = np.mean(time1[0])
    print('mean time 0 {}'.format(meant))
    meant = np.mean(time1[1])
    print('mean time 1 {}'.format(meant))
    meanc = np.mean(costs1)
    print('mean cost {}'.format(meanc))

    print('Box-Ap')
    meant = np.mean(time2[0])
    print('mean time 0 {}'.format(meant))
    meant = np.mean(time2[1])
    print('mean time 1 {}'.format(meant))
    meanc = np.mean(costs2)
    print('mean cost {}'.format(meanc))
