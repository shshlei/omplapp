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
    parser.add_argument('-t', '--time', default=None, help='Filename of the time cost.')
    args = parser.parse_args()

    time_cost = []
    if args.time:
        for line in open(args.time, 'r').readlines():
            line = line.strip()
            if not line:
                continue
            state = [float(x) for x in line.split(' ')]
            time_cost.append(state)

        time_cost = np.array(time_cost).T
        samping_time = time_cost[0]
        opti_time = time_cost[1]
        cost = time_cost[2]

        print('sol percentage {}'.format(len(samping_time)))

        meant = np.mean(samping_time)
        print('mean samping time {}'.format(meant))
        meant = np.mean(opti_time)
        print('mean optimization time {}'.format(meant))
        meanc = np.mean(cost)
        print('mean cost {}'.format(meanc))
