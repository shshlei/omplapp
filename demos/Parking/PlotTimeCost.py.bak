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
    parser.add_argument('-s', '--scenario', default=0, help='Scenario.')
    parser.add_argument('-c', '--cases', default=0, help='Cases.')
    args = parser.parse_args()

    line = open(str('TimeCost/mj_time_'+str(args.scenario)+'_'+str(args.cases)+'.txt'), 'r').readline()
    line = line.strip()
    mj_time = [float(x) for x in line.split(' ')]

    line = open(str('TimeCost/mj_cost_'+str(args.scenario)+'_'+str(args.cases)+'.txt'), 'r').readline()
    line = line.strip()
    mj_costs = [float(x) for x in line.split(' ')]

    ac_time = []
    line = open(str('TimeCost/ac_time_'+str(args.scenario)+'_'+str(args.cases)+'.txt'), 'r').readline()
    line = line.strip()
    ac_time = [float(x) for x in line.split(' ')]

    line = open(str('TimeCost/ac_cost_'+str(args.scenario)+'_'+str(args.cases)+'.txt'), 'r').readline()
    line = line.strip()
    ac_costs = [float(x) for x in line.split(' ')]

    line = open(str('TimeCost/duald_time_'+str(args.scenario)+'_'+str(args.cases)+'.txt'), 'r').readline()
    line = line.strip()
    duald_time = [float(x) for x in line.split(' ')]

    line = open(str('TimeCost/duald_cost_'+str(args.scenario)+'_'+str(args.cases)+'.txt'), 'r').readline()
    line = line.strip()
    duald_costs = [float(x) for x in line.split(' ')]

    line = open(str('TimeCost/dualmj_time_'+str(args.scenario)+'_'+str(args.cases)+'.txt'), 'r').readline()
    line = line.strip()
    dualmj_time = [float(x) for x in line.split(' ')]

    line = open(str('TimeCost/dualmj_cost_'+str(args.scenario)+'_'+str(args.cases)+'.txt'), 'r').readline()
    line = line.strip()
    dualmj_costs = [float(x) for x in line.split(' ')]

    print('MJ2')
    meant = np.mean(mj_time[0])
    print('mj mean time {}'.format(meant))
    meanc = np.mean(mj_costs)
    print('mj mean cost {}'.format(meanc))

    print('AC')
    meant = np.mean(ac_time[0])
    print('ac mean time {}'.format(meant))
    meanc = np.mean(ac_costs)
    print('ac mean cost {}'.format(meanc))

    print('DualD')
    meant = np.mean(duald_time[0])
    print('duald mean time {}'.format(meant))
    meanc = np.mean(duald_costs)
    print('duald mean cost {}'.format(meanc))

    print('DualMJ')
    meant = np.mean(dualmj_time[0])
    print('dualmj mean time {}'.format(meant))
    meanc = np.mean(dualmj_costs)
    print('dualmj mean cost {}'.format(meanc))

