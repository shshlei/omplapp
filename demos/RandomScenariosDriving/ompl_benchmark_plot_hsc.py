#!/usr/bin/python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
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
import sqlite3
import sys
import argparse
# Pathlib is part of the standard library in Python 3, but for Python2 you
# may have to `apt install python-pathlib2` or `pip install pathlib2`
from pathlib import Path
from warnings import warn
import matplotlib
#matplotlib.use('pdf')
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.cm import viridis
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import numpy as np
import numpy.ma as ma
import pandas as pd
from math import floor
from ompl_benchmark_db_get import *

def plotProgressAttributeLong(cur, planners, attribute, timelimit):
    plannerNames, timeTables, dataTables = getProgressAttribute(cur, planners, attribute)
    plannermeans = []
    plannerstddevs = []
    plannertimes = []
    for timeTable, dataTable, pnid in zip(timeTables, dataTables, range(len(plannerNames))):
        timelen = [len(time) for time in timeTable]
        maxrid = timelen.index(max(timelen))
        maxlen = timelen[maxrid]
        if timeTable[maxrid][-1] > 1.05 * timelimit:
            ind = maxlen
            while ind >= 0:
                if timeTable[maxrid][ind - 1] <= 1.05 * timelimit:
                    break
                ind -= 1
            temp = list(timeTable[maxrid])
            del temp[ind:]
            timeTable[maxrid] = tuple(temp)
            maxlen = ind
        for rid in range(len(timeTable)):
            if len(timeTable[rid]) < maxlen:
                timeTable[rid] = timeTable[rid] + timeTable[maxrid][len(timeTable[rid]):]
            else:
                temp = list(timeTable[rid])
                del temp[maxlen:]
                timeTable[rid] = tuple(temp)
        for rid in range(len(dataTable)):
            if len(dataTable[rid]) < maxlen:
                dataTable[rid] = dataTable[rid] + (dataTable[rid][-1],) * (maxlen - len(dataTable[rid]))
            else:
                temp = list(dataTable[rid])
                del temp[maxlen:]
                dataTable[rid] = tuple(temp)
        filteredData = ma.masked_array(dataTable, np.equal(dataTable, None), dtype=float)
        means = np.mean(filteredData, axis=0)
        stddevs = np.std(filteredData, axis=0, ddof=1)
        plannermeans.append(means)
        plannerstddevs.append(stddevs)
        plannertimes.append(timeTable[maxrid])
    plt.clf()
    ax = plt.gca()
    ax.set_xlabel('time (s)')
    ax.set_ylabel(attribute.replace('_', ' '))
    for means, stddevs, times in zip(plannermeans, plannerstddevs, plannertimes):
        # plot average with error bars
        plt.errorbar(times, means)#, yerr=2*stddevs, errorevery=max(1, len(times) // 20))
    ax.legend(plannerNames)

def plotProgressAttributeSuccessRate(cur, planners, attribute):
    """time success"""
    plannerNames, timeTables, dataTables = getProgressAttribute(cur, planners, attribute)
    plannersuccesstime = []
    plannersuccessratio= []
    for timeTable, dataTable in zip(timeTables, dataTables):
        successtime = []
        successratio = []
        for rid in range(len(dataTable)):
            for x in range(len(dataTable[rid])):
                if pd.isnull(dataTable[rid][x]) is not True:
                    successtime.append(timeTable[rid][x])
                    break
        successtime.sort()
        for i in range(len(successtime)):
            successratio.append((i + 1.0) / len(dataTable))
        successtime = [0.0, ] + successtime
        successratio = [0.0, ] + successratio
        plannersuccesstime.append(successtime)
        plannersuccessratio.append(successratio)
    plt.clf()
    ax = plt.gca()
    ax.set_xlabel('time (s)')
    ax.set_ylabel('success rate')
    for successtime, successratio in zip(plannersuccesstime, plannersuccessratio):
        plt.plot(successtime, successratio)
    ax.legend(plannerNames)

#########################
###### statistics #######
#########################
def basicStaticis(dbname, cost_threshold):
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')
    c.execute('SELECT id, name FROM plannerConfigs')
    planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '')) \
        for t in c.fetchall()]
    c.execute('PRAGMA table_info(runs)')
    colInfo = c.fetchall()[3:]

    labels, measurements, nanCounts = getAttribute(c, planners, 'solved')
    print(labels)
    meanm = [sum(m) * 100. / len(m) for m in measurements]
    print('sol percentage')
    print(meanm)
    print('\n')

    labels, measurements, nanCounts = getAttribute(c, planners, 'best_cost')
    bc = cost_threshold
    meanm = []
    for i in range(len(measurements)):
        a = 0
        for j in range(len(measurements[i])):
            if measurements[i][j] < bc:
                a += 1
        meanm.append(100.0 * a / (len(measurements[i]) + nanCounts[i]))
    print('best percentage')
    print(meanm)
    print('\n')

    if measurements:
        meanm = [np.mean(m) for m in measurements]
        print('best_cost')
        print(meanm)
        print('\n')

    labels, measurements, nanCounts = getAttribute(c, planners, 'graph_states')
    print('mean-nodes')
    print(np.mean(measurements, axis=1))
    print('\n')

###########################
####### single plot #######
###########################
def plotBoxTime(dbname):
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')
    c.execute('SELECT id, name FROM plannerConfigs')
    planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '')) \
        for t in c.fetchall()]
    c.execute('PRAGMA table_info(runs)')
    colInfo = c.fetchall()[3:]
    labels, measurements, nanCounts = getAttribute(c, planners, 'time')
    timelimit = 0.0
    c.execute("""SELECT id, name, timelimit, memorylimit FROM experiments""")
    experiments = c.fetchall()
    for experiment in experiments:
        timelimit = max(timelimit, experiment[2])

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    #plt.rcParams.update({'figure.autolayout': True})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    #colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    #['#26b9ce', '#7f3b71', '#2a3377', '#9fa0a0', '#85878b', '#73cdc9', '#afceff', '#ffafaf']

    fig, axs = plt.subplots(figsize=(3.0, 2.0))
    bplot = axs.boxplot(measurements, notch=0, patch_artist=True)#, sym='k+', vert=1, whis=1.5, bootstrap=1000)
    if max(nanCounts) > 0:
        maxy = max([max(y) for y in measurements])
        for i in range(len(labels)):
            x = i + 1
            axs.text(x, .95*maxy, str(nanCounts[i]), horizontalalignment='center', size='small')
    #axs.set_ylim(ymin = 0, ymax = 1.0)
    #axs.set_xlabel('Algorithms')
    axs.set_ylabel('Time (s)', fontsize=8)
    #setup(axs)

    #axs.yaxis.set_major_locator(ticker.MultipleLocator(0.5))
    #axs.yaxis.set_major_locator(ticker.FixedLocator([0, 0.2, 0.4, 0.6, 0.8]))
    #axs.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))
    #axs.yaxis.set_minor_formatter(ticker.NullFormatter())
    axs.yaxis.set_tick_params(which='major', direction = 'in', width=0.50, length=3, labelsize=8)
    axs.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)

    #axs.xaxis.set_major_locator(ticker.NullLocator())
    #axs.xaxis.set_minor_locator(ticker.NullLocator())
    #axs.xaxis.set_major_formatter(ticker.NullFormatter())
    axs.xaxis.set_tick_params(which='major', direction = 'in', width=0.50, length=3, labelsize=8)
    ticks = axs.get_xticks()
    texts = axs.set_xticklabels(labels, rotation=30, fontsize=8)
    #axs.set_xticks(np.array([1]), labels=['BiHSC'], rotation=30, fontsize=6,fontweight='bold')
    #axs.set_xticks(np.array(range(2,11)), labels=labels[1:], rotation=30, fontsize=6,)
    texts[0].set_fontweight('bold')
    #texts[6].set_fontweight('bold')
    #axs.axvline(6.5, ls='--', lw=1.0, c='black')

    markers = ['o', '^', 's', 'o', '^', 's', 'o', '^', 's', 's']
    #bcolors = ['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD', '#1f77b4', '#ff7f0e', '#2ca02c', '#17becf', '#8c564b', '#7f7f7f', '#bcbd22']
    bcolors = ['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974']
    bcolors = ['#08d9d6', '#ffbd59', '#3EC1D3', '#FFFD8C', '#F6416C']
    ecolors = ['black', '#1072b4', '#EF7F00', '#1F9C3A', '#D6191B']
    lw = 0.5
    for box, median, color, ecolor in zip(bplot['boxes'], bplot['medians'], bcolors, ecolors):
        box.set_facecolor(color)
        box.set_edgecolor(ecolor)
        median.set_color('dimgray')
        box.set_linewidth(lw)
        median.set_linewidth(lw)
    for whiskers in bplot['whiskers']:
        whiskers.set_linewidth(lw)
    for caps in bplot['caps']:
        caps.set_linewidth(lw)
    for fliers in bplot['fliers']:
        fliers.set_markeredgewidth(lw)
        fliers.set_markersize(3.0)

    #axs.annotate("T", xy = (-0.05, 0.95), xycoords='axes fraction')
    plt.tight_layout()
    plt.savefig('boxtime.svg')
    plt.savefig('boxtime.pdf')
    plt.show()

def plotBoxTimeTwin(dbname):
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')
    c.execute('SELECT id, name FROM plannerConfigs')
    planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '')) \
        for t in c.fetchall()]
    c.execute('PRAGMA table_info(runs)')
    colInfo = c.fetchall()[3:]
    labels, measurements, nanCounts = getAttribute(c, planners, 'time')
    timelimit = 0.0
    c.execute("""SELECT id, name, timelimit, memorylimit FROM experiments""")
    experiments = c.fetchall()
    for experiment in experiments:
        timelimit = max(timelimit, experiment[2])

    #labels =['BiHSC', 'RRT', 'RRT-Ball', 'RRTC', 'RRTC-Ball', 'BiHSC*', 'RRT*', 'RRT*-Ball']
    labels =['BiHSC', 'LBKPIECE', 'PRM', 'RRT', 'RRTConnect', 'BiHSC*', 'PRM*', 'BIT*', 'RRT*', 'InformedRRT*']
    #labels =['BiHSC', 'LBKPIECE', 'RRTConnect', 'PRM', 'RRT', 'BiHSC*', 'PRM*', 'RRT*', 'InformedRRT*', 'BIT*']

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    #plt.rcParams.update({'figure.autolayout': True})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    #colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    #['#26b9ce', '#7f3b71', '#2a3377', '#9fa0a0', '#85878b', '#73cdc9', '#afceff', '#ffafaf']

    pos = 5
    fig, axs = plt.subplots(figsize=(3.2, 2.2))
    bplot = axs.boxplot(measurements[:pos], notch=0, patch_artist=True)#, sym='k+', vert=1, whis=1.5, bootstrap=1000)

    axs2 = axs.twinx()
    bplot2 = axs2.boxplot(measurements[pos:], notch=0, patch_artist=True, positions=range(pos + 1, len(labels) + 1))

    if max(nanCounts) > 0:
        maxy = max([max(y) for y in measurements])
        for i in range(len(labels)):
            x = i + 1
            axs.text(x, .95*maxy, str(nanCounts[i]), horizontalalignment='center', size='small')
    #axs.set_ylim(ymin = 0, ymax = 1.0)
    #axs.set_xlabel('Algorithms')
    axs.set_ylabel('Time (s)', fontsize=8)
    #setup(axs)

    #axs.yaxis.set_major_locator(ticker.MultipleLocator(10))
    #axs.yaxis.set_major_locator(ticker.FixedLocator([0, 0.2, 0.4, 0.6, 0.8]))
    #axs.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))
    #axs.yaxis.set_minor_formatter(ticker.NullFormatter())
    axs.yaxis.set_tick_params(which='major', direction = 'in', width=0.50, length=3, labelsize=8)
    axs.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
    axs2.yaxis.set_tick_params(which='major', direction = 'in', width=0.50, length=3, labelsize=8)

    #axs.xaxis.set_major_locator(ticker.NullLocator())
    #axs.xaxis.set_minor_locator(ticker.NullLocator())
    #axs.xaxis.set_major_formatter(ticker.NullFormatter())
    axs.xaxis.set_tick_params(which='major', direction = 'in', width=0.50, length=3)
    ticks = axs.get_xticks()
    texts = axs.set_xticklabels(labels, rotation=30, fontsize=8)
    texts[0].set_fontweight('bold')
    texts[5].set_fontweight('bold')

    markers = ['o', '^', 's', 'o', '^', 's', 'o', '^', 's', 's']
    bcolors = ['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974']
    #bcolors = ['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD', '#1f77b4', '#ff7f0e', '#2ca02c', '#17becf', '#8c564b', '#7f7f7f', '#bcbd22']
    lw = 0.5
    for box, median, color in zip(bplot['boxes'] + bplot2['boxes'], bplot['medians'] + bplot2['medians'], bcolors):
        box.set_facecolor(color)
        median.set_color('black')
        box.set_linewidth(lw)
        median.set_linewidth(lw)
    for whiskers in bplot['whiskers'] + bplot2['whiskers']:
        whiskers.set_linewidth(lw)
    for caps in bplot['caps'] + bplot2['caps']:
        caps.set_linewidth(lw)
    for fliers in bplot['fliers'] + bplot2['fliers']:
        fliers.set_markeredgewidth(lw)
        fliers.set_markersize(3.0)

    axs.axvline(5.5, ls='--', lw=1.0, c='black')

    #axs.annotate("T", xy = (-0.05, 0.95), xycoords='axes fraction')
    plt.tight_layout()
    plt.savefig('boxtime.svg')
    plt.show()

def plotBoxTimeTwinHusky(dbname):
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')
    c.execute('SELECT id, name FROM plannerConfigs')
    planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '')) \
        for t in c.fetchall()]
    c.execute('PRAGMA table_info(runs)')
    colInfo = c.fetchall()[3:]
    labels, measurements, nanCounts = getAttribute(c, planners, 'time')
    timelimit = 0.0
    c.execute("""SELECT id, name, timelimit, memorylimit FROM experiments""")
    experiments = c.fetchall()
    for experiment in experiments:
        timelimit = max(timelimit, experiment[2])

    labels =['BiHSC', 'LBKPIECE', 'BKPIECE', 'PRM', 'RRTConnect', 'RRT', 'BiHSC*', 'BFMT', 'FMT', 'PRM*'] # Husky-UR

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    #plt.rcParams.update({'figure.autolayout': True})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    #colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    #['#26b9ce', '#7f3b71', '#2a3377', '#9fa0a0', '#85878b', '#73cdc9', '#afceff', '#ffafaf']

    pos = 6
    fig, axs = plt.subplots(figsize=(3.2, 2.2))
    bplot = axs.boxplot(measurements[:pos], notch=0, patch_artist=True)#, sym='k+', vert=1, whis=1.5, bootstrap=1000)

    axs2 = axs.twinx()
    bplot2 = axs2.boxplot(measurements[pos:], notch=0, patch_artist=True, positions=range(pos + 1, len(labels) + 1))

    if max(nanCounts) > 0:
        maxy = max([max(y) for y in measurements])
        for i in range(len(labels)):
            x = i + 1
            axs.text(x, .95*maxy, str(nanCounts[i]), horizontalalignment='center', size='small')
    #axs.set_ylim(ymin = 0, ymax = 1.0)
    #axs.set_xlabel('Algorithms')
    axs.set_ylabel('Time (s)', fontsize=6)
    #setup(axs)

    axs.yaxis.set_major_locator(ticker.MultipleLocator(10))
    #axs.yaxis.set_major_locator(ticker.FixedLocator([0, 0.2, 0.4, 0.6, 0.8]))
    #axs.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))
    #axs.yaxis.set_minor_formatter(ticker.NullFormatter())
    axs.yaxis.set_tick_params(which='major', direction = 'in', width=0.50, length=3, labelsize=6)
    axs.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
    axs2.yaxis.set_major_locator(ticker.MultipleLocator(10))
    axs2.yaxis.set_major_locator(ticker.FixedLocator([0, 10, 20, 30, 40]))
    axs2.yaxis.set_tick_params(which='major', direction = 'in', width=0.50, length=3, labelsize=6)

    #axs.xaxis.set_major_locator(ticker.NullLocator())
    #axs.xaxis.set_minor_locator(ticker.NullLocator())
    #axs.xaxis.set_major_formatter(ticker.NullFormatter())
    axs.xaxis.set_tick_params(which='major', direction = 'in', width=0.50, length=3)
    ticks = axs.get_xticks()
    texts = axs.set_xticklabels(labels, rotation=30, fontsize=6)
    texts[0].set_fontweight('bold')
    texts[6].set_fontweight('bold')

    markers = ['o', '^', 's', 'o', '^', 's', 'o', '^', 's', 's']
    bcolors = ['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD', '#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974']
    #bcolors = ['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD', '#1f77b4', '#ff7f0e', '#2ca02c', '#17becf', '#8c564b', '#7f7f7f', '#bcbd22']
    lw = 0.5
    for box, median, color in zip(bplot['boxes'] + bplot2['boxes'], bplot['medians'] + bplot2['medians'], bcolors):
        box.set_facecolor(color)
        median.set_color('black')
        box.set_linewidth(lw)
        median.set_linewidth(lw)
    for whiskers in bplot['whiskers'] + bplot2['whiskers']:
        whiskers.set_linewidth(lw)
    for caps in bplot['caps'] + bplot2['caps']:
        caps.set_linewidth(lw)
    for fliers in bplot['fliers'] + bplot2['fliers']:
        fliers.set_markeredgewidth(lw)
        fliers.set_markersize(3.0)

    axs.axvline(6.5, ls='--', lw=1.0, c='black')

    #axs.annotate("T", xy = (-0.05, 0.95), xycoords='axes fraction')
    plt.tight_layout()
    plt.savefig('boxtime.svg')
    plt.show()

###########################
#########           #######
###########################
def plotSuccessRate(cur, planners, attribute):
    """using progress data"""
    plannerNames, timeTables, dataTables = getProgressAttribute(cur, planners, attribute)
    plannersuccesstime = []
    plannersuccessratio= []
    for timeTable, dataTable in zip(timeTables, dataTables):
        successtime = []
        successratio = []
        for rid in range(len(dataTable)):
            for x in range(len(dataTable[rid])):
                if pd.isnull(dataTable[rid][x]) is not True:
                    successtime.append(timeTable[rid][x])
                    break
        successtime.sort()
        for i in range(len(successtime)):
            successratio.append((i + 1.0) / len(successtime))
        successtime = [0.0, ] + successtime
        successratio = [0.0, ] + successratio
        plannersuccesstime.append(successtime)
        plannersuccessratio.append(successratio)
    plt.clf()
    ax = plt.gca()
    ax.set_xlabel('time (s)')
    ax.set_ylabel('success rate')
    for successtime, successratio in zip(plannersuccesstime, plannersuccessratio):
        plt.plot(successtime, successratio)

def plotOptRateBasic(dbname, cost_threshold, bestP):
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')
    c.execute('SELECT id, name FROM plannerConfigs')
    planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '')) \
        for t in c.fetchall()]

    labelsT, measurementsT = getFullAttribute(c, planners, 'time')
    labelsC, measurementsC = getFullAttribute(c, planners, 'best_cost')

    optimaltime = []
    optimalratio= []
    labels = []
    for i in range(len(measurementsC)):
        time = []
        for j in range(len(measurementsC[i])):
            if measurementsC[i][j] and measurementsC[i][j] <= cost_threshold and measurementsT[i][j]:
                time.append(measurementsT[i][j])
        time.sort()
        ratio = []
        for j in range(len(time)):
            ratio.append(100. * (j + 1.0) / len(measurementsC[i]))
        if ratio and (labelsT[i] in bestP):
            optimaltime.append([0.0, ] + time)
            optimalratio.append([0.0, ] + ratio)
            labels.append(labelsT[i])

    stime = []
    bestr = 85.0
    timelimit = 0.0
    c.execute("""SELECT id, name, timelimit, memorylimit FROM experiments""")
    experiments = c.fetchall()
    for experiment in experiments:
        timelimit = max(timelimit, experiment[2])
    for ratio, time in zip(optimalratio, optimaltime):
        if ratio[-1] >= bestr:
            ind = len(ratio)
            while ind >= 0:
                if (ratio[ind-1] < bestr):
                    break
                ind -= 1
            mtime = (bestr - ratio[ind-1]) * time[ind] + (ratio[ind] - bestr) * time[ind-1]
            stime.append(mtime / (ratio[ind] - ratio[ind-1]))
        else:
            stime.append(timelimit)

    rank = np.argsort(stime)
    soptimaltime = []
    soptimalratio= []
    slabels = []
    if False:
        stime.sort()
        for r in rank:
            soptimaltime.append(optimaltime[r])
            soptimalratio.append(optimalratio[r])
            slabels.append(labels[r])
    else:
        soptimaltime = optimaltime
        soptimalratio= optimalratio
        slabels = labels

    plt.style.use(['seaborn-v0_8-deep', 'seaborn-v0_8-paper'])
    plt.rcParams.update({'axes.grid': False})
    #plt.rcParams.update({'figure.autolayout': True})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    #colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    #['#26b9ce', '#7f3b71', '#2a3377', '#9fa0a0', '#85878b', '#73cdc9', '#afceff', '#ffafaf']

    fig, axs = plt.subplots(figsize=(3.0, 2.0))
    markers = ['o', '^', 's', 'D', 'p']
    markersevery = [0.05, 0.05, 0.05, 0.05, 0.05]
    #colors = ['#1072b4', '#ef7f00', '#1f9c3a', '#d6191b', '#9763a6', '#965947', '#db79ae', '#c0c205', '#26b9ce', '#2a3377', '#7f3b71', '#9fa0a0']
    colors = ['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD', '#1f77b4', '#ff7f0e', '#2ca02c', '#17becf']
    colors = ['#08d9d6', '#ffbd59', '#3EC1D3', '#8172B2', '#F6416C']
    slabels = ['BiASE*', 'RRT*', 'InformedRRT*', 'BIT*', 'PRM*']
    for time, ratio, st, lab, mk, mke, color in zip(soptimaltime, soptimalratio, stime, slabels, markers, markersevery, colors):
        axs.plot(time, ratio, marker = mk, markevery = mke, label = lab, color = color)
        axs.plot(time[-1], ratio[-1], marker = mk, color = color)
        if ratio[-1] >= bestr:
            axs.plot(st, bestr, marker = mk, color = color)

    axs.set_ylim(ymin=0.0)
    axs.set_xlim(xmin=-50.0)
    xlim = axs.get_xlim()
    axs.hlines(bestr, xlim[0], xlim[1], linestyles = (0, (5, 2)), colors = 'k', lw=1.0)
    axs.set_xlabel('T (s)', fontsize=6)
    axs.set_ylabel('Opt (%)', fontsize=6)
    setup(axs)
    return axs, stime, bestr, colors

def plotTrendBasic(dbname, attribute, cost_threshold, bestP, rank=None):
    conn = sqlite3.connect(dbname)
    c = conn.cursor()
    c.execute('PRAGMA FOREIGN_KEYS = ON')
    c.execute('SELECT id, name FROM plannerConfigs')
    planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '')) \
        for t in c.fetchall()]

    timelimit = 0.0
    c.execute("""SELECT id, name, timelimit, memorylimit FROM experiments""")
    experiments = c.fetchall()
    for experiment in experiments:
        timelimit = max(timelimit, experiment[2])

    labels, measurements, nanCounts = getAttribute(c, planners, 'best_cost')
    optper = []
    for i in range(len(measurements)):
        a = 0
        for j in range(len(measurements[i])):
            if measurements[i][j] <= cost_threshold:
                a += 1
        optper.append(100.0 * a / (len(measurements[i]) + nanCounts[i]))

    bestr = 85.0
    plannerNames, timeTables, dataTables = getProgressAttributeOpt(c, planners, attribute, cost_threshold)
    plannermeans = []
    plannerstddevs = []
    plannertimes = []
    plannerlabels = []
    for timeTable, dataTable, pnid in zip(timeTables, dataTables, range(len(plannerNames))):
        if dataTable and optper[pnid] >= bestr and (plannerNames[pnid] in bestP):
            timelen = [len(time) for time in timeTable]
            maxrid = timelen.index(max(timelen))
            maxlen = timelen[maxrid]
            if timeTable[maxrid][-1] > 1.05 * timelimit:
                ind = maxlen
                while ind >= 0:
                    if timeTable[maxrid][ind - 1] <= 1.05 * timelimit:
                        break
                    ind -= 1
                temp = list(timeTable[maxrid])
                del temp[ind:]
                timeTable[maxrid] = tuple(temp)
                maxlen = ind
            for rid in range(len(timeTable)):
                if len(timeTable[rid]) < maxlen:
                    timeTable[rid] = timeTable[rid] + timeTable[maxrid][len(timeTable[rid]):]
                else:
                    temp = list(timeTable[rid])
                    del temp[maxlen:]
                    timeTable[rid] = tuple(temp)
            for rid in range(len(dataTable)):
                if len(dataTable[rid]) < maxlen:
                    dataTable[rid] = dataTable[rid] + (dataTable[rid][-1],) * (maxlen - len(dataTable[rid]))
                else:
                    temp = list(dataTable[rid])
                    del temp[maxlen:]
                    dataTable[rid] = tuple(temp)
                if attribute == 'best_cost':
                    dataTable[rid] = (None,) + dataTable[rid]
                elif attribute == 'collision_check_time':
                    dataTable[rid] = (0,) + dataTable[rid]
            filteredData = ma.masked_array(dataTable, np.equal(dataTable, None), dtype=float)
            means = np.mean(filteredData, axis=0)
            stddevs = np.std(filteredData, axis=0, ddof=1)

            plannermeans.append(means)
            plannerstddevs.append(stddevs)
            plannertimes.append((0,) + timeTable[maxrid])
            plannerlabels.append(plannerNames[pnid])

    if rank:
        splannermeans = []
        splannerstddevs = []
        splannertimes = []
        splannerlabels = []
        for r in rank:
            splannermeans.append(plannermeans[r])
            splannerstddevs.append(plannerstddevs[r])
            splannertimes.append(plannertimes[r])
            splannerlabels.append(plannerlabels[r])
    else:
        splannermeans = plannermeans
        splannerstddevs = plannerstddevs
        splannertimes = plannertimes
        splannerlabels = plannerlabels

    plt.style.use(['seaborn-deep', 'seaborn-paper'])
    plt.rcParams.update({'axes.grid': False})
    #plt.rcParams.update({'figure.autolayout': True})
    prop_cycle = plt.rcParams['axes.prop_cycle']
    colors = prop_cycle.by_key()['color'] #['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
    #colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    #['#26b9ce', '#7f3b71', '#2a3377', '#9fa0a0', '#85878b', '#73cdc9', '#afceff', '#ffafaf']

    fig, axs = plt.subplots()
    markers = ['o', '^', 's', 'D', 'p']
    markersevery = [2, 10, 10, 10, 10]
    opttime = []
    lines = []
    colors = ['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD', '#1f77b4', '#ff7f0e', '#2ca02c', '#17becf']
    for means, stddevs, times, mk, mke in zip(splannermeans, splannerstddevs, splannertimes, markers, markersevery):
        line = axs.errorbar(times, means, marker=mk, markevery=mke)#, yerr=2*stddevs, errorevery=max(1, len(times) // 20))
        lines.append(line)
        if attribute == 'best_cost':
            ind = len(means.data)
            while ind >= 0:
                if means.data[ind-1] > cost_threshold:
                    break;
                ind -= 1
            opttime.append(round(times[ind], 1))
    return axs, lines, splannerlabels, opttime


############################
######### Barriers ########
############################
def plotOptRateBarriers(dbname, cost_threshold):
    bestP = ['BiHSCstar', 'RRTstar', 'InformedRRTstar', 'BITstar', 'PRMstar', 'LazyPRMstar']
    axs, stime, bestr, colors = plotOptRateBasic(dbname, cost_threshold, bestP)
    axs.legend(loc = 0, bbox_to_anchor=(0.4, 0.5), framealpha=0, fontsize=6)

    if False:
        r = 0
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 1.0, bestr - 1.0), xycoords='data',
                xytext=(15.0, 75.), textcoords='data',
                size=6, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle = 'simple',
                                connectionstyle="arc3,rad=0.0",
                                color = colors[r]),
                )

        r += 2
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] - 1.0, bestr + 2.0), xycoords='data',
                xytext=(20.0, 95.), textcoords='data',
                size=6, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle="simple",
                                connectionstyle="arc3,rad=-0.2",
                                color = colors[r]),
                )

        r += 1
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 2.0, bestr - 2.0), xycoords='data',
                xytext=(60.0, 65.), textcoords='data',
                size=6, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle="simple",
                                connectionstyle="arc3,rad=0.2",
                                color = colors[r]),
                )

    plt.tight_layout()
    plt.savefig('optimal_rate.pdf')
    plt.show()

def plotTrendCCTBarriers(dbname, cost_threshold):
    bestP = ['BiHSCstar', 'RRTstar', 'InformedRRTstar', 'BITstar', 'LBT-RRT']
    bestP = ['BiHSCstar']
    #rank = [0, 1, 2, 3, 4, 5, 6, 7]
    axs, lines, labels, opttime = plotTrendBasic(dbname, 'collision_check_time', cost_threshold, bestP)

    axs.legend(lines, labels, frameon='True', framealpha=0.0, loc = [0.45, 0.3], fontsize=6)

    #axs.set_xlim(xmin = 0, xmax = 50)
    axs.set_ylim(ymin = 0)

    setup(axs)

    #axs.xaxis.set_major_locator(ticker.MultipleLocator(25))
    #axs.xaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))

    #axs.yaxis.set_major_locator(ticker.MultipleLocator(10))
    #axs.yaxis.set_major_locator(ticker.FixedLocator([0, 5, 10, 15, 20, 25]))
    #axs.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))

    axs.set_xlabel('T (s)')
    axs.set_ylabel('CCT (s)')

    plt.tight_layout()
    plt.savefig('optimal_cctt.pdf')
    plt.show()

############################
######### Maze ########
############################
def plotOptRateMaze(dbname, cost_threshold):
    #bestP = ['BiHSC', 'PRMstar', 'BFMTstar']
    bestP = ['BiHSCstar', 'RRTstar', 'InformedRRTstar', 'PRMstar', 'LazyPRMstar', 'BITstar', 'LazyPRMstar']
    axs, stime, bestr, colors = plotOptRateBasic(dbname, cost_threshold, bestP)
    axs.legend(loc = 0, bbox_to_anchor=(0.4, 0.5), framealpha=0, fontsize=6)

    if False:
        r = 0
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 1.0, bestr - 1.0), xycoords='data',
                xytext=(320.0, 75.), textcoords='data',
                size=6, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle = 'simple',
                                connectionstyle="arc3,rad=0.0",
                                color = colors[r]),
                )

        r += 1
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 1.0, bestr + 2.0), xycoords='data',
                xytext=(350.0, 95.), textcoords='data',
                size=6, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle="simple",
                                connectionstyle="arc3,rad=-0.2",
                                color = colors[r]),
                )

        r += 1
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 2.0, bestr - 2.0), xycoords='data',
                xytext=(1200.0, 65.), textcoords='data',
                size=6, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle="simple",
                                connectionstyle="arc3,rad=-0.2",
                                color = colors[r]),
                )

        r += 1
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 2.0, bestr - 2.0), xycoords='data',
                xytext=(1600.0, 75.), textcoords='data',
                size=6, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle="simple",
                                connectionstyle="arc3,rad=0.0",
                                color = colors[r]),
                )
    plt.tight_layout()
    plt.savefig('optimal_rate.pdf')
    plt.show()

def plotTrendCCTMaze(dbname, cost_threshold):
    #bestP = ['LSC-AISstar', 'PRMstar', 'BFMTstar']
    bestP = ['BiHSCstar', 'RRTstar', 'InformedRRTstar', 'PRMstar', 'LazyPRMstar', 'BITstar', 'LBT-RRT']
    #rank = [0, 1, 2, 3, 4, 5, 6, 7]
    axs, lines, labels, opttime = plotTrendBasic(dbname, 'collision_check_time', cost_threshold, bestP)

    axs.legend(lines, labels, frameon='True', framealpha=0.0, loc = [0.45, 0.3], fontsize=6)

    #axs.set_xlim(xmin = 0, xmax = 50)
    axs.set_ylim(ymin = 0)

    setup(axs)

    #axs.xaxis.set_major_locator(ticker.MultipleLocator(25))
    #axs.xaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))

    #axs.yaxis.set_major_locator(ticker.MultipleLocator(10))
    #axs.yaxis.set_major_locator(ticker.FixedLocator([0, 5, 10, 15, 20, 25]))
    #axs.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))

    axs.set_xlabel('T (s)')
    axs.set_ylabel('CCT (s)')

    plt.tight_layout()
    plt.savefig('optimal_cctt.pdf')
    plt.show()


############################
######### Twistycool #######
############################
def plotOptRateTwistycool(dbname, cost_threshold):
    #bestP = ['BiHSC', 'PRMstar', 'BFMTstar']
    #bestP = ['BiHSCstar', 'RRTstar', 'InformedRRTstar', 'BITstar', 'PRMstar', 'LazyPRMstar']
    bestP = ['BiASEstar', 'RRTstar', 'InformedRRTstar', 'BITstar', 'PRMstar', 'LazyPRMstar']
    axs, stime, bestr, colors = plotOptRateBasic(dbname, cost_threshold, bestP)
    labels = axs.legend(loc = 0, bbox_to_anchor=(0.5, 0.55), framealpha=0, fontsize=6).get_texts()
    labels[0].set_fontweight('bold')

    if False:
        r = 0
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 1.0, bestr + 1.0), xycoords='data',
                xytext=(300.0, 95.), textcoords='data',
                size=6, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle = 'simple',
                                connectionstyle="arc3,rad=0.0",
                                color = colors[r]),
                )

        r += 1
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 2.0, bestr - 2.0), xycoords='data',
                xytext=(1200.0, 65.), textcoords='data',
                size=6, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle="simple",
                                connectionstyle="arc3,rad=0.0",
                                color = colors[r]),
                )

        r += 1
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 2.0, bestr - 2.0), xycoords='data',
                xytext=(1600.0, 70.), textcoords='data',
                size=6, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle="simple",
                                connectionstyle="arc3,rad=0.0",
                                color = colors[r]),
                )
    plt.tight_layout()
    plt.savefig('optimal_rate.svg')
    plt.savefig('optimal_rate.pdf')
    plt.show()

def plotTrendCostTwistycool(dbname, cost_threshold):
    #bestP = ['LSC-AISstar', 'PRMstar', 'BFMTstar']
    bestP = ['BiHSCstar', 'RRTstar', 'InformedRRTstar', 'PRMstar', 'LazyPRMstar', 'BITstar', 'LBT-RRT']
    bestP = ['BiHSCstar']
    axs, lines, labels, opttime = plotTrendBasic(dbname, 'best_cost', cost_threshold, bestP)
    axs.legend(lines, labels, framealpha=0.0, loc = (0.6, 0.8), fontsize=6)

    #axs.set_xlim(xmin = 0, xmax = 50)
    #axs.set_ylim(ymax = 2650)

    #axs.xaxis.set_major_locator(ticker.MultipleLocator(25))
    #axs.xaxis.set_minor_locator(ticker.NullLocator())
    #axs.xaxis.set_major_formatter(ticker.StrMethodFormatter("{x} s"))

    #axs.yaxis.set_major_locator(ticker.MultipleLocator(200))

    xlim = axs.get_xlim()
    ylim = axs.get_ylim()

    axs.hlines(cost_threshold, xlim[0], xlim[1], linestyles = 'dashed')#, colors='r')

    for t, time in enumerate(opttime):
        axs.plot([time, time], [ylim[0], cost_threshold], ls = '--', c = 'C{}'.format(t))

    axs.set_xlabel('T (s)')
    axs.set_ylabel('C')
    plt.savefig('optimal_bct.pdf')
    plt.show()

def plotTrendCCTTwistycool(dbname, cost_threshold):
    #bestP = ['LSC-AISstar', 'PRMstar', 'BFMTstar']
    bestP = ['BiHSCstar', 'RRTstar', 'InformedRRTstar', 'BITstar', 'LBT-RRT']
    #bestP = ['BiHSCstar']
    #rank = [0, 1, 2, 3, 4, 5, 6, 7]
    axs, lines, labels, opttime = plotTrendBasic(dbname, 'collision_check_time', cost_threshold, bestP)

    axs.legend(lines, labels, frameon='True', framealpha=0.0, loc = [0.45, 0.3], fontsize=6)

    #axs.set_xlim(xmin = 0, xmax = 50)
    axs.set_ylim(ymin = 0)

    setup(axs)

    #axs.xaxis.set_major_locator(ticker.MultipleLocator(25))
    #axs.xaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))

    #axs.yaxis.set_major_locator(ticker.MultipleLocator(10))
    #axs.yaxis.set_major_locator(ticker.FixedLocator([0, 5, 10, 15, 20, 25]))
    #axs.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))

    axs.set_xlabel('T (s)')
    axs.set_ylabel('CCT (s)')

    plt.tight_layout()
    plt.savefig('optimal_cctt.pdf')
    plt.show()

#############################
######### Twistycool1 #######
#############################
def plotOptRateTwistycool1(dbname, cost_threshold):
    #bestP = ['LSC-AISstar', 'PRMstar', 'FMTstar']
    bestP = ['LSC-AISstar', 'RRTstar', 'InformedRRTstar', 'PRMstar', 'LazyPRMstar', 'BFMTstar', 'BITstar', 'LBT-RRT']

    axs, stime, bestr, colors = plotOptRateBasic(dbname, cost_threshold, bestP)

    axs.legend(loc = (0.4, 0.18), framealpha=0)

    if True:
        r = 0
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] - 1.0, bestr), xycoords='data',
                xytext=(0.0, 70.), textcoords='data',
                size=8, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle = 'simple',
                                connectionstyle="arc3,rad=0.0",
                                color = colors[r]),
                )
        r += 1
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 1.0, bestr), xycoords='data',
                xytext=(53.0, 70.), textcoords='data',
                size=8, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle="simple",
                                connectionstyle="arc3,rad=-0.2",
                                color = colors[r]),
                )
        r += 1
        axs.annotate('({}, *)'.format(round(stime[r], 0)),
                xy=(stime[r] + 1.0, bestr), xycoords='data',
                xytext=(113.0, 70.), textcoords='data',
                size=8, va="center", ha="center",
                color = colors[r],
                arrowprops=dict(arrowstyle="simple",
                                connectionstyle="arc3,rad=-0.2",
                                color = colors[r]),
                )

    plt.savefig('optimal_rate.eps')

    plt.show()

def plotTrendCostTwistycool1(dbname, cost_threshold):
    #bestP = ['LSC-AISstar', 'PRMstar', 'FMTstar']
    bestP = ['LSC-AISstar', 'RRTstar', 'InformedRRTstar', 'PRMstar', 'LazyPRMstar', 'BFMTstar', 'BITstar', 'LBT-RRT']

    axs, lines, labels, opttime = plotTrendBasic(dbname, 'best_cost', cost_threshold, bestP)

    axs.legend(lines, labels, framealpha=0.0, loc = [0.5, 0.7])

    #axs.set_xlim(xmin = 0, xmax = 50)
    #axs.set_ylim(ymax = 2650)

    setup(axs)

    #axs.xaxis.set_major_locator(ticker.MultipleLocator(25))
    #axs.xaxis.set_minor_locator(ticker.NullLocator())
    #axs.xaxis.set_major_formatter(ticker.StrMethodFormatter("{x} s"))

    #axs.yaxis.set_major_locator(ticker.MultipleLocator(200))

    xlim = axs.get_xlim()
    ylim = axs.get_ylim()

    axs.hlines(cost_threshold, xlim[0], xlim[1], linestyles = 'dashed')#, colors='r')

    for t, time in enumerate(opttime):
        axs.plot([time, time], [ylim[0], cost_threshold], ls = '--', c = 'C{}'.format(t))

    axs.set_xlabel('T (s)')
    axs.set_ylabel('C')

    plt.savefig('optimal_bct.eps')

    plt.show()

def plotTrendCCTTwistycool1(dbname, cost_threshold):
    #bestP = ['LSC-AISstar', 'PRMstar', 'FMTstar']
    bestP = ['LSC-AISstar', 'RRTstar', 'InformedRRTstar', 'PRMstar', 'LazyPRMstar', 'BFMTstar', 'BITstar', 'LBT-RRT']

    axs, lines, labels, opttime = plotTrendBasic(dbname, 'collision_check_time', cost_threshold, bestP)

    axs.legend(lines, labels, frameon='True', framealpha=0.0, loc = [0.45, 0.42])

    #axs.set_xlim(xmin = 0, xmax = 50)
    axs.set_ylim(ymin = 0)

    setup(axs)

    #axs.xaxis.set_major_locator(ticker.MultipleLocator(25))
    #axs.xaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))

    #axs.yaxis.set_major_locator(ticker.MultipleLocator(10))
    #axs.yaxis.set_major_locator(ticker.FixedLocator([0, 5, 10, 15, 20, 25]))
    #axs.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))

    axs.set_xlabel('T (s)')
    axs.set_ylabel('CCT (s)')

    plt.savefig('optimal_cctt.eps')

    plt.show()


def plotLegend():
    #plt.style.use('seaborn-paper')
    plt.style.use(['seaborn-deep', 'seaborn-paper'])

    plt.rcParams.update({'axes.grid': False})
    plt.rcParams.update({'figure.autolayout': True})

    fig, axs = plt.subplots(figsize=(1.5, 3.0))

    #setup(axs)
    labels = ['LSC-AIS', 'RRTConnect', 'RRT', 'PRM', 'LazyRRT', 'SBL', 'EST', 'BKPIECE1', 'LBKPIECE1', 'STRIDE']

    values = np.linspace(2, 20, 10)

    for i, yvalue in enumerate(values):
        axs.axhline(y = yvalue, xmin = 0, xmax = 0, color = 'C{}'.format(i))

    axs.legend(labels, loc = 'center', labelspacing = 1.5)

    axs.spines['top'].set_visible(False)
    axs.spines['bottom'].set_visible(False)
    axs.spines['left'].set_visible(False)
    axs.spines['right'].set_visible(False)

    axs.xaxis.set_major_locator(ticker.NullLocator())
    axs.xaxis.set_minor_locator(ticker.NullLocator())
    axs.yaxis.set_major_locator(ticker.NullLocator())
    axs.yaxis.set_minor_locator(ticker.NullLocator())

    plt.savefig('legend.pdf')

    plt.show()

def plotLegend2():
    #plt.style.use('seaborn-paper')
    plt.style.use(['seaborn-deep', 'seaborn-paper'])

    plt.rcParams.update({'axes.grid': False})
    plt.rcParams.update({'figure.autolayout': True})

    fig, axs = plt.subplots()

    labels = ['LSC-AIS', 'RRTConnect', 'RRT', 'PRM', 'LazyRRT', 'SBL', 'EST', 'BKPIECE1', 'LBKPIECE1', 'STRIDE']

    x, y = np.linspace(0, 10, 10), np.zeros(10)

    for i  in range(len(labels)):
        axs.plot(x, y-i, linewidth=1.5, color='C{}'.format(i))

    axs.set_ylim(-len(labels)+0.5, 0.5)

    axs.spines['top'].set_visible(False)
    axs.spines['bottom'].set_visible(False)
    axs.spines['left'].set_visible(False)
    axs.spines['right'].set_visible(False)

    axs.xaxis.set_major_locator(ticker.NullLocator())
    axs.xaxis.set_minor_locator(ticker.NullLocator())
    axs.yaxis.set_major_locator(ticker.NullLocator())
    axs.yaxis.set_minor_locator(ticker.NullLocator())

    axs.yaxis.set_ticks_position('right')
    axs.yaxis.set_tick_params(which='major', direction = 'in', width=1.00, length=0)

    axs.set_yticks([-x for x in range(len(labels))])
    axs.set_yticklabels(labels)

    #plt.savefig('legend.pdf')

    plt.tight_layout()
    plt.show()

##########################
######### Multiple #######
##########################
def plotMutipleBoxTime(dbnames):
    dbcomputationT = []
    dbcomtimenanCounts = []

    for dbname in dbnames:
        conn = sqlite3.connect(dbname)
        c = conn.cursor()
        c.execute('PRAGMA FOREIGN_KEYS = ON')
        c.execute('SELECT id, name FROM plannerConfigs')
        planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '')) \
            for t in c.fetchall()]

        c.execute('PRAGMA table_info(runs)')
        colInfo = c.fetchall()[3:]

        labels, computationT, comtimenanCounts = getAttribute(c, planners, 'time')

        dbcomputationT.append(computationT)
        dbcomtimenanCounts.append(comtimenanCounts)

    #plt.style.use('seaborn-paper')
    plt.style.use(['seaborn-deep', 'seaborn-paper'])

    fig = plt.figure(figsize=(8.0, 4.3))
    spec = gridspec.GridSpec(2, 2)#, width_ratios=[1, 1, 0.1])
    spec.update(left=0.09, right=0.85, bottom = 0.1, top = 0.97, wspace=0.35, hspace=0.3)

    loc = (5, 10, 2, 50)

    for i in range(2):
        for j in range(2):
            axs = fig.add_subplot(spec[i, j])
            ind = 2 * i + j
            measurements = dbcomputationT[ind]
            nanCounts = dbcomtimenanCounts[ind]

            bplot = axs.boxplot(measurements, notch=0)#, sym='k+', vert=1, whis=1.5, bootstrap=1000)

            if max(nanCounts) > 0:
                maxy = max([max(y) for y in measurements])
                for nani in range(len(labels)):
                    x = nani + 1
                    axs.text(x, .95*maxy, str(nanCounts[nani]), horizontalalignment='center', size='small')

            axs.set_ylim(ymin = 0)#, ymax = 11)

            if ind == 2:
                axs.set_ylim(ymax = 11)

            #axs.set_xlabel('Algorithms')
            axs.set_ylabel('Time')

            #setup(axs)

            axs.yaxis.set_major_locator(ticker.MultipleLocator(loc[ind]))
            axs.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f} s"))
            axs.yaxis.set_tick_params(which='major', direction = 'in', width=1.00, length=3)
            axs.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)

            #axs.xaxis.set_major_locator(ticker.NullLocator())
            axs.xaxis.set_minor_locator(ticker.NullLocator())
            axs.xaxis.set_major_formatter(ticker.NullFormatter())
            axs.xaxis.set_tick_params(which='major', direction = 'in', width=1.00, length=3)

            for m, median in enumerate(bplot['medians']):
                median.set_color('C{}'.format(m))

    spec2 = gridspec.GridSpec(1, 1)
    spec2.update(left=0.87, right=0.885, bottom = 0.3, top = 0.85)#, hspace=0.05)

    axs5 = fig.add_subplot(spec2[0, 0])

    labels = ['LSC-AIS', 'RRTConnect', 'RRT', 'PRM', 'LazyRRT', 'SBL', 'EST', 'BKPIECE1', 'LBKPIECE1', 'STRIDE']

    x, y = np.linspace(0, 1, 5), np.zeros(5)

    for i  in range(len(labels)):
        axs5.plot(x, y - i, linewidth=1.5, color='C{}'.format(i))

    axs5.set_ylim(-len(labels)+0.5, 0.5)

    axs5.spines['left'].set_visible(False)
    axs5.spines['right'].set_visible(False)
    axs5.spines['bottom'].set_visible(False)
    axs5.spines['top'].set_visible(False)

    axs5.xaxis.set_major_locator(ticker.NullLocator())
    axs5.xaxis.set_minor_locator(ticker.NullLocator())
    axs5.yaxis.set_major_locator(ticker.NullLocator())
    axs5.yaxis.set_minor_locator(ticker.NullLocator())

    axs5.yaxis.set_ticks_position('right')
    axs5.yaxis.set_tick_params(which='major', direction = 'in', width=1.00, length=0)

    axs5.set_yticks([-x for x in range(len(labels))])
    axs5.set_yticklabels(labels)

    fig.text(0.24, 0.52, '(a)')
    fig.text(0.67, 0.52, '(b)')
    fig.text(0.24, 0.03, '(c)')
    fig.text(0.67, 0.03, '(d)')

    plt.savefig('feasible.eps')

    plt.show()

def plotMutipleFeasibleSolvedTime(dbnames):
    plt.style.use(['seaborn-deep', 'seaborn-paper'])
    plt.rcParams.update({'axes.grid': False})

    fig = plt.figure(figsize=(8.0, 4.3))
    spec = gridspec.GridSpec(2, 2)
    spec.update(left=0.09, right=0.85, bottom = 0.1, top = 0.97, wspace=0.35, hspace=0.3)

    loc = (0, 0, 0, (0.5, 0.1))

    for i in range(2):
        for j in range(2):
            ind = 2 * i + j
            axs = fig.add_subplot(spec[i, j])
            plotFeasibleSolvedTime(axs, dbnames[ind], loc[ind])

    spec2 = gridspec.GridSpec(1, 1)
    spec2.update(left=0.86, right=0.900, bottom = 0.3, top = 0.85)#, hspace=0.05)

    axs5 = fig.add_subplot(spec2[0, 0])

    labels = ['LSC-AIS', 'RRTConnect', 'RRT', 'PRM', 'LazyRRT', 'SBL', 'EST', 'BKPIECE1', 'LBKPIECE1', 'STRIDE']

    w = 2.0
    h = 0.5

    for i in range(len(labels)):
        axs5.add_patch(Rectangle(xy=(0, -i), width=w, height=h, facecolor='C{}'.format(i)))
        axs5.scatter(0.5 * w, 0.5 * h - i, color = 'lime', s = 20, zorder = 5)

    axs5.set_ylim(-len(labels)+0.5+0.5*h, 0.5+0.5*h)

    axs5.spines['left'].set_visible(False)
    axs5.spines['right'].set_visible(False)
    axs5.spines['bottom'].set_visible(False)
    axs5.spines['top'].set_visible(False)

    axs5.xaxis.set_major_locator(ticker.NullLocator())
    axs5.xaxis.set_minor_locator(ticker.NullLocator())
    axs5.yaxis.set_major_locator(ticker.NullLocator())
    axs5.yaxis.set_minor_locator(ticker.NullLocator())

    axs5.yaxis.set_ticks_position('right')
    axs5.yaxis.set_tick_params(which='major', direction = 'in', width=1.00, length=0)

    axs5.set_yticks([-x+0.5*h for x in range(len(labels))])
    axs5.set_yticklabels(labels)

    fig.text(0.21, 0.51, '(a) Home')
    fig.text(0.63, 0.51, '(b) Abstract')
    fig.text(0.18, 0.02, '(c) Apartment')
    fig.text(0.625, 0.02, '(d) Twistycool')

    plt.savefig('feasible.eps')

    plt.show()

def setup(axs):
    xlim = axs.get_xlim()
    ylim = axs.get_ylim()

    #axs.spines["top"].set_visible(False)
    #axs.spines["right"].set_visible(False)
    axs.spines["left"].set_position(("data", xlim[0]))
    axs.spines['left'].set_bounds(ylim[0], ylim[1])
    axs.spines['right'].set_bounds(ylim[0], ylim[1])
    axs.spines["bottom"].set_position(("data", ylim[0]))
    axs.spines['bottom'].set_bounds(xlim[0], xlim[1])
    axs.spines['top'].set_bounds(xlim[0], xlim[1])

    #axs.plot(1, ylim[0], ">k", transform=axs.get_yaxis_transform(), clip_on=False, markersize = 5)
    #axs.plot(xlim[0], 1, "^k", transform=axs.get_xaxis_transform(), clip_on=False, markersize = 5)

    axs.xaxis.set_ticks_position('bottom')
    axs.xaxis.set_tick_params(which='major', direction = 'in', width=1.00, length=3, labelsize=6)
    #axs.xaxis.set_major_formatter(ticker.StrMethodFormatter("{x} s"))

    axs.yaxis.set_ticks_position('left')
    axs.yaxis.set_tick_params(which='major', direction = 'in', width=1.00, length=3, labelsize=6)

    axs.yaxis.grid(True, linestyle=(0, (5, 5)), lw = 1, which='major', color='darkgrey', alpha=0.5)
    axs.set_xlim(xlim)
    axs.set_ylim(ylim)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Process benchmark logs and create an SQLite3 database.')
    parser.add_argument('-d', '--database', default='benchmark.db', \
        help='Filename of benchmark database')
    parser.add_argument('-ct', '--cost_threshold', default=100.0, \
        help='The cost threshold of the problem')
    parser.add_argument('dbfile', nargs='*')

    args = parser.parse_args()

    #basicStaticis(args.database, float(args.cost_threshold))
    #solvedAverageTime(args.database)
    #solvedAverageCollisionTime(args.database)

    #plotTotalTimeBar(args.database)
    #plotCCTVSTime(args.database)

    #plotOptRateTwistycool(args.database, float(args.cost_threshold))

    #plotTrendCCTBarriers(args.database, float(args.cost_threshold))
    #plotTrendCCTMaze(args.database, float(args.cost_threshold))
    #plotTrendCCTTwistycool(args.database, float(args.cost_threshold))
    #plotTrendCCTHome(args.database, float(args.cost_threshold))

    #plotFeasibleSolvedTime(args.database)
    #plotMutipleFeasibleSolvedTime(args.dbfile)

    #plotBoxTime(args.database)
    plotBoxTimeTwinHusky(args.database)
    #plotBoxTimeNodesTwin(args.database)
    #plotLegend2()

    #plotRRTTime(args.database)
    #plotRRTCost(args.database)
    #plotRRTCCTime(args.database)

    #plotTrendCostAbstract(args.database, float(args.cost_threshold))
    #plotMutipleBoxTime(args.dbfile)
