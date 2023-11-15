#!/usr/bin/env python

######################################################################
# Rice University Software Distribution License
#
# Copyright (c) 2010, Rice University
# All Rights Reserved.
#
# For a full description see the file named LICENSE.
#
######################################################################

# Author: Shi Shenglei

import sys
from os.path import abspath, dirname, join
from decimal import Decimal
from inspect import isclass
import OpenGL
OpenGL.ERROR_CHECKING = False
from OpenGL import GL, GLU, GLUT
import webbrowser
from math import cos, sin, asin, acos, atan2, pi, sqrt
import random

USE_DEPRECATED_API = False

try:
    # try PyQt5 first
    from PyQt5 import QtWidgets, QtCore, QtGui
    from PyQt5.QtCore import pyqtSignal as Signal
    from PyQt5.QtGui import QColor, QOpenGLVersionProfile, QOpenGLContext
    from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QOpenGLWidget, QSlider, QWidget)

    from PyQt5.QtCore import (QPoint, QPointF, QRect, QRectF, QSize, Qt, QTime, QTimer)
    from PyQt5.QtGui import (QBrush, QFontMetrics, QImage, QPainter,
            QRadialGradient, QSurfaceFormat)

except ImportError:
    # next try PySide2
    try:
        from PySide2 import QtCore, QtOpenGL, QtWidgets, QtGui
        from PySide2.QtCore import Signal
    except ImportError:
        # next try PyQt4
        # (deprecated, will be removed at some point)
        USE_DEPRECATED_API = True
        try:
            from PyQt4 import QtGui, QtCore, QtGui, QtOpenGL
            from PyQt4.QtCore import pyqtSignal as Signal
        except ImportError:
            # finally try PySide
            from PySide import QtCore, QtGui, QtOpenGL
            from PySide.QtCore import Signal
        QtWidets = QtGui

# The ConfigParser module has been renamed to configparser in Python 3.0
try:
    import ConfigParser
except ImportError:
    import configparser as ConfigParser

#sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'ompl/py-bindings'))
from ompl.util import OutputHandler, useOutputHandler, LogLevel, \
    OMPL_DEBUG, OMPL_INFORM, OMPL_ERROR
from ompl import base as ob
from ompl import app as oa

import ompl

# wrapper for API change in PyQt
def getOpenFileNameAsAstring(qtwindow, title="", directory="", ffilter=""):
    fname = QtWidgets.QFileDialog.getOpenFileName(qtwindow, title, directory, ffilter)
    return str(fname[0]) if isinstance(fname, tuple) else str(fname)

# wrapper for API change in PyQt
def getSaveFileNameAsAstring(qtwindow, title="", directory=""):
    fname = QtWidgets.QFileDialog.getSaveFileName(qtwindow, title, directory)
    return str(fname[0]) if isinstance(fname, tuple) else str(fname)

class LogOutputHandler(OutputHandler):
    def __init__(self, textEdit):
        super(LogOutputHandler, self).__init__()
        self.textEdit = textEdit
        self.redColor = QtGui.QColor(255, 0, 0)
        self.orangeColor = QtGui.QColor(255, 128, 0)
        self.greenColor = QtGui.QColor(0, 255, 64)
        self.blackColor = QtGui.QColor(0, 0, 0)

    def log(self, text, level, filename, line):
        if level == LogLevel.LOG_DEBUG:
            self.debug(text)
        elif level == LogLevel.LOG_INFO:
            self.inform(text)
        elif level == LogLevel.LOG_WARN:
            self.warn(text)
        elif level == LogLevel.LOG_ERROR:
            self.error(text)
        else:
            print(text)

    def debug(self, text):
        self.textEdit.setTextColor(self.greenColor)
        self.textEdit.append(text)
        self.textEdit.repaint()

    def inform(self, text):
        self.textEdit.setTextColor(self.blackColor)
        self.textEdit.append(text)
        self.textEdit.repaint()

    def warn(self, text):
        self.textEdit.setTextColor(self.orangeColor)
        self.textEdit.append(text)
        self.textEdit.repaint()

    def error(self, text):
        self.textEdit.setTextColor(self.redColor)
        self.textEdit.append(text)
        self.textEdit.repaint()

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        QtWidgets.QMainWindow.__init__(self)
        self.createActions()
        self.createMenus()
        self.createRobotTypeList()
        self.mainWidget = MainWidget(self.robotTypes)
        self.setCentralWidget(self.mainWidget)
        self.setWindowTitle('GVIZ')
        self.environmentFile = None
        self.robotFile = None
        self.path = None
        self.isGeometric = True
        self.is3D = True
        self.pd = None
        self.ellipses = []
        self.certificates = []

        self.mainWidget.problemWidget.robotTypeSelect.currentIndexChanged[int].connect(
            self.setRobotType)
#        self.mainWidget.solveWidget.solveButton.clicked.connect(self.solve)
        self.mainWidget.solveWidget.clearButton.clicked.connect(self.clear)
        self.mainWidget.boundsWidget.resetButton.clicked.connect(self.resetBounds)

        # connect timeLimit widgets in geometric and control planning with each other and with the
        # MainWindow.setTimeLimit method
        self.mainWidget.plannerWidget.geometricPlanning.timeLimit.valueChanged[float].connect(
            self.setTimeLimit)
        self.mainWidget.plannerWidget.geometricPlanning.timeLimit.valueChanged[float].connect(
            self.mainWidget.plannerWidget.controlPlanning.timeLimit.setValue)
        self.mainWidget.plannerWidget.controlPlanning.timeLimit.valueChanged[float].connect(
            self.setTimeLimit)
        self.mainWidget.plannerWidget.controlPlanning.timeLimit.valueChanged[float].connect(
            self.mainWidget.plannerWidget.geometricPlanning.timeLimit.setValue)
        self.timeLimit = self.mainWidget.plannerWidget.geometricPlanning.timeLimit.value()

        self.mainWidget.boundsWidget.bounds_low.valueChanged.connect(
            self.mainWidget.glViewer.setLowerBound)
        self.mainWidget.boundsWidget.bounds_high.valueChanged.connect(
            self.mainWidget.glViewer.setUpperBound)
        self.mainWidget.glViewer.boundLowChanged.connect(
            self.mainWidget.boundsWidget.bounds_low.setBounds)
        self.mainWidget.glViewer.boundHighChanged.connect(
            self.mainWidget.boundsWidget.bounds_high.setBounds)

        #self.commandWindow = CommandWindow(self) # not implemented yet
        robotType = [t for t in self.robotTypes].index('GSE3GeometryRender')
        self.mainWidget.problemWidget.robotTypeSelect.setCurrentIndex(robotType)

        # connect to OMPL's console output (via OutputHandlers)
        self.logWindow = LogWindow(self)
        self.oh = LogOutputHandler(self.logWindow.logView)
        useOutputHandler(self.oh)

    def openEnvironment(self):
        fname = getOpenFileNameAsAstring(self, "Open Environment")
        if fname and fname != self.environmentFile:
            self.environmentFile = fname
            self.omplSetup.setEnvironmentMesh(self.environmentFile)
            self.mainWidget.glViewer.setEnvironment(self.omplSetup.renderEnvironment())
            self.resetBounds()
            if self.isGeometric:
                if self.robotFile:
                    self.mainWidget.plannerWidget.geometricPlanning.resolution.setValue(
                        self.omplSetup.getStateValidityCheckingResolution())

    def openRobot(self):
        fname = getOpenFileNameAsAstring(self, "Open Robot")
        if fname and fname != self.robotFile:
            self.robotFile = fname
            self.omplSetup.setRobotMesh(self.robotFile)
            self.mainWidget.glViewer.setRobot(self.omplSetup.renderRobot())
            self.omplSetup.inferEnvironmentBounds()
            # full state
            start = self.omplSetup.getDefaultStartState()
            # just the first geometric component
            start = self.omplSetup.getGeometricComponentState(start, 0)
            self.mainWidget.problemWidget.setStartPose(start, self.is3D)
            self.mainWidget.problemWidget.setGoalPose(start, self.is3D)
            if self.isGeometric:
                self.mainWidget.plannerWidget.geometricPlanning.resolution.setValue(
                    self.omplSetup.getStateValidityCheckingResolution())
            self.mainWidget.glViewer.setBounds(
                self.omplSetup.getGeometricComponentStateSpace().getBounds())

    def openConfig(self):
        fname = getOpenFileNameAsAstring(self, "Open Problem Configuration", "", "*.cfg")
        if fname:
            OMPL_INFORM("Loading " + fname)
            if sys.version_info > (3, 0):
                config = ConfigParser.ConfigParser(strict=False)
            else:
                config = ConfigParser.ConfigParser()
            config.read([fname])
            if config.has_option("problem", "start.z"):
                robotType = [t for t in self.robotTypes].index('GSE3GeometryRender')
                self.mainWidget.problemWidget.robotTypeSelect.setCurrentIndex(robotType)
            else:
                robotType = [t for t in self.robotTypes].index('GSE2GeometryRender')
                self.mainWidget.problemWidget.robotTypeSelect.setCurrentIndex(robotType)
            cfg_dir = dirname(fname)
            self.setRobotType(robotType)
            self.environmentFile = join(cfg_dir, config.get("problem", "world"))
            self.omplSetup.setEnvironmentMesh(self.environmentFile)
            self.mainWidget.glViewer.setEnvironment(self.omplSetup.renderEnvironment())

            self.robotFile1 = join(cfg_dir, config.get("problem", "robot1"))
            self.omplSetup.setRobotMesh(self.robotFile1)
            self.mainWidget.glViewer.setRobot1(self.omplSetup.renderRobot())
            self.robotFile2 = join(cfg_dir, config.get("problem", "robot2"))
            self.omplSetup.setRobotMesh(self.robotFile2)
            self.mainWidget.glViewer.setRobot2(self.omplSetup.renderRobot())

            self.resetBounds()
            if self.isGeometric:
                self.mainWidget.plannerWidget.geometricPlanning.resolution.setValue(
                    self.omplSetup.getStateValidityCheckingResolution())
            start = ob.State(self.omplSetup.getGeometricComponentStateSpace())
            goal = ob.State(self.omplSetup.getGeometricComponentStateSpace())
            if self.is3D:
                start().setX(config.getfloat("problem", "start.x"))
                start().setY(config.getfloat("problem", "start.y"))
                start().setZ(config.getfloat("problem", "start.z"))
                start().rotation().setAxisAngle(config.getfloat("problem", "start.axis.x"),
                                                config.getfloat("problem", "start.axis.y"),
                                                config.getfloat("problem", "start.axis.z"),
                                                config.getfloat("problem", "start.theta"))
                goal().setX(config.getfloat("problem", "goal.x"))
                goal().setY(config.getfloat("problem", "goal.y"))
                goal().setZ(config.getfloat("problem", "goal.z"))
                goal().rotation().setAxisAngle(config.getfloat("problem", "goal.axis.x"),
                                               config.getfloat("problem", "goal.axis.y"),
                                               config.getfloat("problem", "goal.axis.z"),
                                               config.getfloat("problem", "goal.theta"))
            else:
                start().setX(config.getfloat("problem", "start.x"))
                start().setY(config.getfloat("problem", "start.y"))
                start().setYaw(config.getfloat("problem", "start.theta"))
                goal().setX(config.getfloat("problem", "goal.x"))
                goal().setY(config.getfloat("problem", "goal.y"))
                goal().setYaw(config.getfloat("problem", "goal.theta"))
            self.mainWidget.problemWidget.setStartPose(start, self.is3D)
            self.mainWidget.problemWidget.setGoalPose(goal, self.is3D)
            if self.is3D:
                if config.has_option("problem", "volume.min.x") and \
                   config.has_option("problem", "volume.min.y") and \
                   config.has_option("problem", "volume.min.z") and \
                   config.has_option("problem", "volume.max.x") and \
                   config.has_option("problem", "volume.max.y") and \
                   config.has_option("problem", "volume.max.z"):
                    bounds = ob.RealVectorBounds(3)
                    bounds.low[0] = config.getfloat("problem", "volume.min.x")
                    bounds.low[1] = config.getfloat("problem", "volume.min.y")
                    bounds.low[2] = config.getfloat("problem", "volume.min.z")
                    bounds.high[0] = config.getfloat("problem", "volume.max.x")
                    bounds.high[1] = config.getfloat("problem", "volume.max.y")
                    bounds.high[2] = config.getfloat("problem", "volume.max.z")
                    self.omplSetup.getGeometricComponentStateSpace().setBounds(bounds)
                    self.mainWidget.glViewer.setBounds(bounds)
            else:
                if config.has_option("problem", "volume.min.x") and \
                   config.has_option("problem", "volume.min.y") and \
                   config.has_option("problem", "volume.max.x") and \
                   config.has_option("problem", "volume.max.y"):
                    bounds = ob.RealVectorBounds(2)
                    bounds.low[0] = config.getfloat("problem", "volume.min.x")
                    bounds.low[1] = config.getfloat("problem", "volume.min.y")
                    bounds.high[0] = config.getfloat("problem", "volume.max.x")
                    bounds.high[1] = config.getfloat("problem", "volume.max.y")
                    self.omplSetup.getGeometricComponentStateSpace().setBounds(bounds)
                    self.mainWidget.glViewer.setBounds(bounds)
            if config.has_option("problem", "objective"):
                ind = self.mainWidget.problemWidget.objectiveSelect.findText(
                    config.get("problem", "objective").replace("_", " "))
                if ind != -1:
                    self.mainWidget.problemWidget.objectiveSelect.setCurrentIndex(ind)
                    if config.has_option("problem", "objective.threshold"):
                        self.mainWidget.problemWidget.objectiveThreshold.setValue(
                            config.getfloat("problem", "objective.threshold"))

    def saveConfig(self):
        fname = getSaveFileNameAsAstring(self, 'Save Problem Configuration', 'config.cfg')
        if fname:
            config = ConfigParser.ConfigParser()
            config.add_section("problem")
            config.set("problem", "robot", self.robotFile)
            config.set("problem", "world", self.environmentFile)

            startPose = self.mainWidget.problemWidget.getStartPose()
            goalPose = self.mainWidget.problemWidget.getGoalPose()

            ctype = str(self.mainWidget.problemWidget.robotTypeSelect.currentText())
            b = self.omplSetup.getGeometricComponentStateSpace().getBounds()

            if self.is3D:
                config.set("problem", "start.x", str(startPose().getX()))
                config.set("problem", "start.y", str(startPose().getY()))
                config.set("problem", "start.z", str(startPose().getZ()))
                rs = startPose().rotation()
                if rs.w == 1:
                    config.set("problem", "start.theta", "0")
                    config.set("problem", "start.axis.x", "1")
                    config.set("problem", "start.axis.y", "0")
                    config.set("problem", "start.axis.z", "0")
                else:
                    config.set("problem", "start.theta", str(2 * acos(rs.w)))
                    ds = sqrt(1.0 - rs.w * rs.w)
                    config.set("problem", "start.axis.x", str(rs.x / ds))
                    config.set("problem", "start.axis.y", str(rs.y / ds))
                    config.set("problem", "start.axis.z", str(rs.z / ds))
                config.set("problem", "goal.x", str(goalPose().getX()))
                config.set("problem", "goal.y", str(goalPose().getY()))
                config.set("problem", "goal.z", str(goalPose().getZ()))
                rg = goalPose().rotation()
                if rg.w == 1:
                    config.set("problem", "goal.theta", "0")
                    config.set("problem", "goal.axis.x", "1")
                    config.set("problem", "goal.axis.y", "0")
                    config.set("problem", "goal.axis.z", "0")
                else:
                    config.set("problem", "goal.theta", str(2 * acos(rg.w)))
                    dg = sqrt(1.0 - rg.w * rg.w)
                    config.set("problem", "goal.axis.x", str(rg.x / dg))
                    config.set("problem", "goal.axis.y", str(rg.y / dg))
                    config.set("problem", "goal.axis.z", str(rg.z / dg))
                config.set("problem", "volume.min.x", str(b.low[0]))
                config.set("problem", "volume.min.y", str(b.low[1]))
                config.set("problem", "volume.min.z", str(b.low[2]))
                config.set("problem", "volume.max.x", str(b.high[0]))
                config.set("problem", "volume.max.y", str(b.high[1]))
                config.set("problem", "volume.max.z", str(b.high[2]))
            else:
                config.set("problem", "start.x", str(startPose().getX()))
                config.set("problem", "start.y", str(startPose().getY()))
                config.set("problem", "start.theta", str(startPose().getYaw()))
                config.set("problem", "goal.x", str(goalPose().getX()))
                config.set("problem", "goal.y", str(goalPose().getY()))
                config.set("problem", "goal.theta", str(goalPose().getYaw()))
                config.set("problem", "volume.min.x", str(b.low[0]))
                config.set("problem", "volume.min.y", str(b.low[1]))
                config.set("problem", "volume.max.x", str(b.high[0]))
                config.set("problem", "volume.max.y", str(b.high[1]))
            with open(fname, 'w') as configfile:
                config.write(configfile)
            OMPL_INFORM("Saved " + fname)

    def _arrayToSE2State(self, a):
        st = ob.State(self.omplSetup.getGeometricComponentStateSpace())
        st().setXY(a[0], a[1])
        st().setYaw(a[2])
        return st

    def _arrayToSE3State(self, a):
        st = ob.State(self.omplSetup.getGeometricComponentStateSpace())
        st().setXYZ(a[0], a[1], a[2])
        R = st().rotation()
        nrm = 1./sqrt(a[3]*a[3] + a[4]*a[4] + a[5]*a[5] + a[6]*a[6])
        (R.x, R.y, R.z, R.w) = (a[3]*nrm, a[4]*nrm, a[5]*nrm, a[6]*nrm)
        return st

    def openPath(self):
        fname = getOpenFileNameAsAstring(self, "Open Path")
        if fname:
            path = []
            for line in open(fname, 'r').readlines():
                l = line.strip()
                if not l:
                    continue
                path.append([float(x) for x in l.split(' ')])

            self.mainWidget.glViewer.solutionPath = []
            # assume that first 3 components map to SE(2) if 3<len<7
            if len(path[0]) > 2 and len(path[0]) < 7:
                self.path = [self._arrayToSE2State(s) for s in path]
            elif len(path[0]) >= 7:
                self.path = [self._arrayToSE3State(s) for s in path]
            else:
                # unknown state type
                OMPL_ERROR("Wrong state format")
                raise ValueError
            self.mainWidget.glViewer.setSolutionPath(self.path)
            # setStart/GoalPose can change bounds, so save and restore them
            bounds = self.mainWidget.glViewer.getBounds()
            self.mainWidget.problemWidget.setStartPose(self.path[0], self.is3D)
            self.mainWidget.problemWidget.setGoalPose(self.path[-1], self.is3D)
            self.mainWidget.glViewer.setBounds(bounds)

    def savePath(self):
        if self.path:
            fname = getSaveFileNameAsAstring(self, 'Save Path', 'path.txt')
            if fname:
                ind = range(7 if self.is3D else 3)
                pathstr = '\n'.join([' '.join([str(s[i]) for i in ind]) for s in self.path])
                open(fname, 'w').write(pathstr)

    def savePlannerData(self):
        fname = getSaveFileNameAsAstring(self, 'Save Roadmap/Tree', 'plannerData.graphml')
        if fname:
            pds= ob.PlannerDataStorage()
            self.pd.computeEdgeWeights()
            pds.store(self.pd, fname)

    def loadPlannerData(self):
        fname = getOpenFileNameAsAstring(self, "Load PlannerData")
        if fname:
            pd = ob.PlannerData(self.omplSetup)
            pds= ob.PlannerDataStorage()
            pds.load(fname, pd)
            self.pd = pd
            pd.computeEdgeWeights()
            self.mainWidget.glViewer.plannerDataList = self.omplSetup.renderPlannerData(pd)
        self.mainWidget.glViewer.setBounds(self.omplSetup.getGeometricComponentStateSpace().getBounds())

    def loadEllipsoid(self):
        self.ellipses = []
        fname = getOpenFileNameAsAstring(self, "Load Ellipsoid")
        if fname:
            fellipse = open(fname, 'r')
            readnewline = True
            linetemp = None
            stop = False
            while True:
                if stop:
                    break
                if readnewline:
                    line = fellipse.readline()
                    line = line.strip()
                    if not line:
                        break
                    line = line.split('  ')
                else:
                    line = linetemp
                    readnewline = True

                if line[0] == "clear":
                    self.ellipses = []
                elif line[0] == "new":
                    while True:
                        l = fellipse.readline()
                        l = l.strip()
                        if not l:
                            stop = True
                            break
                        l = l.split('  ')
                        if l[0] == "clear" or l[0] == "self-expanded" or l[0] == "expanded":
                            linetemp = l
                            readnewline = False
                            break
                        self.ellipses.append([float(x) for x in l])
                elif line[0] == "self-expanded":
                    self.ellipses.append([float(line[2]) * self.ellipses[int(line[1])][0], float(line[2]) * self.ellipses[int(line[1])][1],
                                          self.ellipses[int(line[1])][2], self.ellipses[int(line[1])][3], self.ellipses[int(line[1])][4]])
                elif line[0] == "expanded":
                    self.ellipses.append([float(line[2]) * self.ellipses[int(line[1])][0], float(line[2]) * self.ellipses[int(line[1])][1],
                                          self.ellipses[int(line[1])][2], self.ellipses[int(line[1])][3], self.ellipses[int(line[1])][4]])

            self.mainWidget.glViewer.ellipses = []
            self.mainWidget.glViewer.setEllipses(self.ellipses)
            fellipse.close()

    def loadCertificate(self):
        self.certificates = []
        fname = getOpenFileNameAsAstring(self, "Load Safety Certificate")
        if fname:
            for line in open(fname, 'r').readlines():
                l = line.strip()
                if not l:
                    continue
                self.certificates.append([float(x) for x in l.split(' ')])

            self.mainWidget.glViewer.setCertificates(self.certificates)

    def showLogWindow(self):
        if self.logWindow.isHidden():
            self.logWindow.show()
            self.logWindow.raise_()
            self.logWindow.activateWindow()
        else:
            self.logWindow.hide()
            self.mainWidget.raise_()
            self.mainWidget.activateWindow()

    def showCommandWindow(self):
        self.commandWindow.show()
        self.commandWindow.raise_()
        self.commandWindow.activateWindow()

    def omplWebSite(self):
        webbrowser.open('http://ompl.kavrakilab.org')
    def contactDevs(self):
        webbrowser.open('mailto:ompl-devel@lists.sourceforge.net')
    def emailList(self):
        webbrowser.open('mailto:ompl-users@lists.sourceforge.net')

    def setRobotType(self, value):
        self.environmentFile = None
        self.robotFile = None
        self.path = None
        self.omplSetup = eval('oa.%s()' % self.robotTypes[value])
        self.clear(True)
#        self.isGeometric = self.robotTypes[value] == oa.
        self.is3D = isinstance(self.omplSetup.getGeometricComponentStateSpace(), ob.SE3StateSpace)
        self.mainWidget.plannerWidget.setCurrentIndex(0 if self.isGeometric else 1)
        self.mainWidget.problemWidget.poses.setCurrentIndex(0 if self.is3D else 1)

    def setTimeLimit(self, value):
        OMPL_DEBUG('Changing time limit from %g to %g' % (self.timeLimit, value))
        self.timeLimit = value

#    def solve(self):

    def clear(self, deepClean=False):
        self.mainWidget.glViewer.clear(deepClean)

    def resetBounds(self):
        if self.is3D:
            b = ob.RealVectorBounds(3)
            self.omplSetup.getGeometricComponentStateSpace().setBounds(b)
        else:
            b = ob.RealVectorBounds(2)
            self.omplSetup.getGeometricComponentStateSpace().setBounds(b)
        self.omplSetup.inferEnvironmentBounds()
        self.mainWidget.glViewer.setBounds(
            self.omplSetup.getGeometricComponentStateSpace().getBounds())

    def createActions(self):
        self.openEnvironmentAct = QtWidgets.QAction('Open &Environment', self, \
            shortcut='Ctrl+E', statusTip='Open an environment model', \
            triggered=self.openEnvironment)
        self.openRobotAct = QtWidgets.QAction('Open &Robot', self, \
            shortcut='Ctrl+R', statusTip='Open a robot model', \
            triggered=self.openRobot)
        self.openConfigAct = QtWidgets.QAction('Open Problem &Configuration', self, \
            shortcut='Ctrl+O', statusTip='Open a problem configuration (.cfg file)', \
            triggered=self.openConfig)
        self.saveConfigAct = QtWidgets.QAction('Save Problem Con&figuration', self, \
            shortcut='Ctrl+S', statusTip='Save a problem configuration (.cfg file)', \
            triggered=self.saveConfig)
        self.openPathAct = QtWidgets.QAction('&Open Path', self, \
            shortcut='Ctrl+Alt+O', statusTip='Open a path', \
            triggered=self.openPath)
        self.savePathAct = QtWidgets.QAction('Save &Path', self, \
            shortcut='Ctrl+Alt+S', statusTip='Save a path', \
            triggered=self.savePath)
        self.savePlannerDataAct = QtWidgets.QAction('Save &Roadmap/Tree', self, \
            statusTip='Save the roadmap/tree that was created by "Solve"', \
            triggered=self.savePlannerData)
        self.loadPlannerDataAct = QtWidgets.QAction('Load &Roadmap/Tree', self, \
            shortcut='Ctrl+D', statusTip='Load the roadmap/tree that was created by planner', \
            triggered=self.loadPlannerData)
        self.loadEllipsoidAct = QtWidgets.QAction('Load &Ellipsoid/Ellipse', self, \
            shortcut='Ctrl+Alt+E', statusTip='Load the ellipsoid/ellipse that was created by the planner', \
            triggered=self.loadEllipsoid)
        self.loadCertificateAct = QtWidgets.QAction('Load &Safety Certificate', self, \
            triggered=self.loadCertificate)
        self.exitAct = QtWidgets.QAction('E&xit', self, shortcut='Ctrl+Q', \
            statusTip='Exit the application', triggered=self.close)

        self.logWindowAct = QtWidgets.QAction('Log Window', self, \
            shortcut='Ctrl+1', triggered=self.showLogWindow)
        self.commandWindowAct = QtWidgets.QAction('Command Window', self, \
            shortcut='Ctrl+2', triggered=self.showCommandWindow)

        self.omplWebAct = QtWidgets.QAction('OMPL Web Site', self, \
            triggered=self.omplWebSite)
        self.contactDevsAct = QtWidgets.QAction('Contact Developers', self, \
            triggered=self.contactDevs)
        self.emailListAct = QtWidgets.QAction('Email OMPL Mailing List', self, \
            triggered=self.emailList)

    def createMenus(self):
        self.fileMenu = self.menuBar().addMenu('&File')
        self.fileMenu.addAction(self.openEnvironmentAct)
        self.fileMenu.addAction(self.openRobotAct)
        self.fileMenu.addAction(self.openConfigAct)
        self.fileMenu.addAction(self.saveConfigAct)
        self.fileMenu.addAction(self.openPathAct)
        self.fileMenu.addAction(self.savePathAct)
        self.fileMenu.addAction(self.savePlannerDataAct)
        self.fileMenu.addAction(self.loadPlannerDataAct)
        self.fileMenu.addAction(self.loadEllipsoidAct)
        self.fileMenu.addAction(self.loadCertificateAct)
        self.fileMenu.addSeparator()
        self.fileMenu.addAction(self.exitAct)

        self.toolsMenu = self.menuBar().addMenu('&Tools')
        self.toolsMenu.addAction(self.logWindowAct)
#        self.toolsMenu.addAction(self.randMotionAct)
#        self.windowMenu.addAction(self.commandWindowAct)

        self.helpMenu = self.menuBar().addMenu('Help')
        self.helpMenu.addAction(self.omplWebAct)
        self.helpMenu.addAction(self.contactDevsAct)
        self.helpMenu.addAction(self.emailListAct)

    def createRobotTypeList(self):
        self.robotTypes = []
        for c in dir(oa):
            if eval('isclass(oa.%s)' \
                    'and issubclass(oa.%s, oa.RenderGeometry)' % (c, c)):
                self.robotTypes.append(c)

class MainWidget(QtWidgets.QWidget):
    def __init__(self, robotTypes, parent=None, flags=QtCore.Qt.WindowFlags(0)):
        super(MainWidget, self).__init__(parent, flags)
        self.glViewer = GLViewer()
        self.problemWidget = ProblemWidget(robotTypes)
        self.plannerWidget = PlannerWidget()
        self.boundsWidget = BoundsWidget()
        self.solveWidget = SolveWidget()
        self.solveWidget.setSizePolicy(QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed))
        tabWidget = QtWidgets.QTabWidget()
        tabWidget.addTab(self.problemWidget, "Problem")
        tabWidget.addTab(self.plannerWidget, "Planner")
        tabWidget.addTab(self.boundsWidget, "Bounding box")
        tabWidget.setSizePolicy(QtWidgets.QSizePolicy(
            QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed))
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.glViewer, 0, 0, 2, 1)
        layout.addWidget(tabWidget, 0, 1)
        layout.addWidget(self.solveWidget, 2, 0, 1, 2)
        self.setLayout(layout)
        self.problemWidget.startChanged.connect(self.glViewer.setStartPose)
        self.problemWidget.goalChanged.connect(self.glViewer.setGoalPose)
        self.solveWidget.explorationVizSelect.currentIndexChanged[int].connect(
            self.glViewer.showPlannerData)
        self.solveWidget.animateCheck.toggled.connect(self.glViewer.toggleAnimation)
        self.solveWidget.speedSlider.valueChanged.connect(self.glViewer.setSpeed)

class LogWindow(QtWidgets.QWidget):
    def __init__(self, parent=None, flags=QtCore.Qt.Tool):
        super(LogWindow, self).__init__(parent, flags)
        self.setWindowTitle('OMPL Log')
        self.resize(640, 320)
        self.logView = QtWidgets.QTextEdit(self)
        self.logView.setReadOnly(True)
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.logView, 0, 0)
        self.setLayout(layout)

class CommandWindow(QtWidgets.QWidget):
    def __init__(self, parent=None, flags=QtCore.Qt.Tool):
        super(CommandWindow, self).__init__(parent, flags)

class GLViewer(QOpenGLWidget):
    boundLowChanged = Signal(list)
    boundHighChanged = Signal(list)

    def __init__(self, parent=None):
        super(GLViewer, self).__init__(parent)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.lastPos = QtCore.QPoint()
        self.environment = None
        self.robot1 = None
        self.robot2 = None
        self.center = [0, 0, 0]
        self.scale = 1
        self.viewheight = 1
        self.cameraPose = [0, 0, 0, 0, 0, 0]
        self.startPose = [0, 0, 0, 0, 0, 0]
        self.goalPose = [0, 0, 0, 0, 0, 0]
        self.solutionPath = None
        self.pathIndex = 0
        self.timer = QtCore.QTimer()
        self.timer.start(1000.)
        self.timer.timeout.connect(self.updatePathPose)
        self.animate = True
        self.drawPlannerData = False
        self.plannerDataList = None
        self.bounds_low = None
        self.bounds_high = None
        #self.elevation = 0
        self.trolltechGreen = QColor.fromCmykF(0.40, 0.0, 1.0, 0.0)
        self.trolltechPurple = QColor.fromCmykF(0.39, 0.39, 0.0, 0.0)
        self.ellipses = []
        self.certificates = []

    def minimumSizeHint(self):
        return QtCore.QSize(500, 300)

    def setRotationAngle(self, axisIndex, angle):
        if angle != self.cameraPose[axisIndex]:
            self.cameraPose[axisIndex] = angle
            self.update()
    def getBounds(self):
        bounds = ob.RealVectorBounds(3)
        for i in range(3):
            bounds.low[i] = self.bounds_low[i]
            bounds.high[i] = self.bounds_high[i]
        return bounds
    def setBounds(self, bounds):
        self.bounds_low = [x for x in bounds.low]
        self.bounds_high = [x for x in bounds.high]
        if len(self.bounds_low) == 2:
            self.bounds_low.append(0)
            self.bounds_high.append(0)
        bbox = list(zip(self.bounds_low, self.bounds_high))
        self.center = [.5 * (p0 + p1) for (p0, p1) in bbox]
        m = max([p1 - p0 for (p0, p1) in bbox])
        self.scale = 1. if m == 0 else 1. / m
        self.viewheight = (self.bounds_high[2]-self.bounds_low[2])*self.scale*3
        self.boundLowChanged.emit(self.bounds_low)
        self.boundHighChanged.emit(self.bounds_high)
    def updateBounds(self, pos):
        lo = False
        hi = False
        if self.bounds_low is None:
            self.bounds_low = pos
            self.bounds_high = pos
            self.boundLowChanged.emit(self.bounds_low)
            self.boundHighChanged.emit(self.bounds_high)
        else:
            for i in range(3):
                if pos[i] < self.bounds_low[i]:
                    self.bounds_low[i] = pos[i]
                    lo = True
                elif pos[i] > self.bounds_high[i]:
                    self.bounds_high[i] = pos[i]
                    hi = True
            if lo:
                self.boundLowChanged.emit(self.bounds_low)
            if hi:
                self.boundHighChanged.emit(self.bounds_high)
    def setLowerBound(self, bound):
        self.bounds_low = bound
        self.update()
    def setUpperBound(self, bound):
        self.bounds_high = bound
        self.update()
    def setStartPose(self, value):
        self.startPose = value
        self.updateBounds(value[3:])
        self.update()
    def setGoalPose(self, value):
        self.goalPose = value
        self.updateBounds(value[3:])
        self.update()
    def showPlannerData(self, value):
        self.drawPlannerData = value
        self.update()
    def toggleAnimation(self, value):
        self.animate = value
        if self.animate:
            self.timer.start()
        else:
            self.timer.stop()
        self.update()
    def setSpeed(self, value):
        if value == 0:
            self.timer.stop()
        else:
            self.timer.start(1000.0/float(value))
            self.updatePathPose()
    def updatePathPose(self):
        if self.solutionPath is not None:
            self.pathIndex = (self.pathIndex + 1) % len(self.solutionPath)
            self.update()
    def setSolutionPath(self, path):
        self.solutionPath = [self.getTransform(state()) for state in path]
        self.pathIndex = 0
        self.update()
    def setRobot1(self, robot):
        if self.robot1:
            self.gl.glDeleteLists(self.robot1, 1)
        self.robot1 = robot
    def setRobot2(self, robot):
        if self.robot2:
            self.gl.glDeleteLists(self.robot2, 1)
        self.robot2 = robot
    def setEnvironment(self, environment):
        if self.environment:
            self.gl.glDeleteLists(self.environment, 1)
        self.environment = environment
    def clear(self, deepClean=False):
        self.solutionPath = None
        self.plannerDataList = None
        self.pathIndex = 0
        self.ellipses = []
        self.certificates = []
        if deepClean:
            self.setRobot1(None)
            self.setRobot2(None)
            self.setEnvironment(None)
            self.bounds_low = None
            self.bounds_high = None
            self.cameraPose = [0, 0, 0, 0, 0, 0]
        self.update()

    def setEllipses(self, ellipses):
        for ellipse in ellipses:
            self.ellipses.append((self.makeEllipse(ellipse[0], ellipse[1], ellipse[2], ellipse[3], ellipse[4])))

    def setCertificates(self, certificates):
        for certificate in certificates:
            if len(certificate) == 3:
                self.certificates.append((self.makeCircle(certificate[0], certificate[1], certificate[2])))
            elif len(certificate) == 4:
                self.certificates.append((self.makeSphere(certificate[0], certificate[1], certificate[2], certificate[3])))

    def initializeGL(self):
        profile = QOpenGLVersionProfile()
        profile.setVersion(2, 0)
        self.gl = QOpenGLContext.currentContext().versionFunctions(profile)
        self.gl.initializeOpenGLFunctions()

        self.gl.glClearColor(0.5, 0.5, 0.5, 1.)
        self.gl.glShadeModel(self.gl.GL_FLAT)
        self.gl.glEnable(self.gl.GL_LIGHTING)
        self.gl.glEnable(self.gl.GL_LIGHT0)
        self.gl.glEnable(self.gl.GL_LIGHT1)
        self.gl.glEnable(self.gl.GL_DEPTH_TEST)
        self.gl.glLightModeli(self.gl.GL_LIGHT_MODEL_TWO_SIDE, self.gl.GL_TRUE)
        self.gl.glEnable(self.gl.GL_NORMALIZE)
        self.gl.glColorMaterial(self.gl.GL_FRONT_AND_BACK, self.gl.GL_DIFFUSE)
        self.gl.glEnable(self.gl.GL_LINE_SMOOTH)
        self.gl.glEnable(self.gl.GL_BLEND)
        #self.gl.glEnable(self.gl.GL_CULL_FACE)
        self.gl.glBlendFunc(self.gl.GL_SRC_ALPHA, self.gl.GL_ONE_MINUS_SRC_ALPHA)

    def setClearColor(self, c):
        self.gl.glClearColor(c.redF(), c.greenF(), c.blueF(), c.alphaF())

    def setColor(self, c):
        self.gl.glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF())

    def transform(self, pose):
        self.gl.glPushMatrix()
        self.gl.glTranslatef(pose[3], pose[4], pose[5])
        self.gl.glRotated(pose[0], 1.0, 0.0, 0.0)
        self.gl.glRotated(pose[1], 0.0, 1.0, 0.0)
        self.gl.glRotated(pose[2], 0.0, 0.0, 1.0)

    def getTransform(self, xform):
        if hasattr(xform, 'rotation'):
            R = xform.rotation()
            (w, x, y, z) = (R.w, -R.x, -R.y, -R.z)
            return [w*w+x*x-y*y-z*z, 2*(x*y-w*z), 2*(x*z+w*y), 0,
                    2*(x*y+w*z), w*w-x*x+y*y-z*z, 2*(y*z-w*x), 0,
                    2*(x*z-w*y), 2*(y*z+w*x), w*w-x*x-y*y+z*z, 0,
                    xform.getX(), xform.getY(), xform.getZ(), 1]
        th = -xform.getYaw()
        return [cos(th), -sin(th), 0, 0,
                sin(th), cos(th), 0, 0,
                0, 0, 1, 0,
                xform.getX(), xform.getY(), 0, 1]

    def drawBounds(self):
        lo = self.bounds_low
        hi = self.bounds_high
        p = [lo, [lo[0], lo[1], hi[2]], [lo[0], hi[1], lo[2]], [lo[0], hi[1], hi[2]], \
            [hi[0], lo[1], lo[2]], [hi[0], lo[1], hi[2]], [hi[0], hi[1], lo[2]], hi]
        ind = [(0, 1), (1, 3), (3, 2), (2, 0), (4, 5), (5, 7), (7, 6), (6, 4), (0, 4), \
            (1, 5), (2, 6), (3, 7)]
        self.gl.glDisable(self.gl.GL_LIGHTING)
        self.gl.glDisable(self.gl.GL_COLOR_MATERIAL)
        self.gl.glColor3f(1, 1, 1)
        self.gl.glBegin(self.gl.GL_LINES)
        for edge in ind:
            self.gl.glVertex3fv(p[edge[0]])
            self.gl.glVertex3fv(p[edge[1]])
        self.gl.glEnd()

    def paintGL(self):
        self.gl.glClear(self.gl.GL_COLOR_BUFFER_BIT | self.gl.GL_DEPTH_BUFFER_BIT)
        self.gl.glMatrixMode(self.gl.GL_MODELVIEW)
        self.gl.glLoadIdentity()
        GLU.gluLookAt(0, 0, 3, 0, 0, -5, 0, 1, 0)
        self.transform(self.cameraPose)
        self.gl.glScalef(self.scale, self.scale, self.scale)
        self.gl.glTranslatef(-self.center[0], -self.center[1], -self.center[2])
        # draw bounding box
        if self.bounds_low:
            self.drawBounds()
        if self.robot1:
            self.transform(self.startPose)
            self.gl.glCallList(self.robot1)
            self.gl.glPopMatrix()

            # draw path pose(s)
            if self.solutionPath is not None:
                if self.animate:
                    self.gl.glPushMatrix()
                    self.gl.glMultMatrixf(self.solutionPath[self.pathIndex])
                    self.gl.glCallList(self.robot1)
                    self.gl.glPopMatrix()
                else:
                    n = len(self.solutionPath)
                    nmax = 100
                    if n < nmax:
                        ind = range(0, n)
                    else:
                        step = float(n - 1.)/float(nmax - 1)
                        ind = [int(step*i) for i in range(nmax)]
                    for i in ind:
                        self.gl.glPushMatrix()
                        self.gl.glMultMatrixf(self.solutionPath[i])
                        self.gl.glCallList(self.robot1)
                        self.gl.glPopMatrix()

        if self.robot2:
            # draw goal pose
            self.transform(self.goalPose)
            self.gl.glCallList(self.robot2)
            self.gl.glPopMatrix()

        # draw environment
        if self.environment:
            self.gl.glColor3f(1, 1, 1)
            self.gl.glCallList(self.environment)

        # draw the planner data
        if self.drawPlannerData and self.plannerDataList:
            self.gl.glCallList(self.drawPlannerData + self.plannerDataList - 1)

        for ellipse in self.ellipses:
            self.gl.glCallList(ellipse)

        for certificate in self.certificates:
            self.gl.glCallList(certificate)

        self.gl.glPopMatrix()

    def resizeGL(self, width, height):
        side = min(width, height)
        if side <= 0:
            return
        self.gl.glMatrixMode(self.gl.GL_PROJECTION)
        self.gl.glLoadIdentity()
        GLU.gluPerspective(45., float(width) / float(height), 1., 1000.)
#        self.gl.glOrtho(-5.00, +5.00, -5.00, +5.00, 1.0, 1000.0)

        self.gl.glViewport(0, 0, width, height)

    def makeEllipse(self, rx, ry, x0, y0, theta):
        genList = self.gl.glGenLists(1)
        self.gl.glNewList(genList, self.gl.GL_COMPILE)

        self.gl.glBegin(self.gl.GL_LINE_LOOP)

        self.ellipse(rx, ry, x0, y0, theta)

        self.gl.glEnd()
        self.gl.glEndList()

        return genList

    def makeCircle(self, x0, y0, r):
        genList = self.gl.glGenLists(1)
        self.gl.glNewList(genList, self.gl.GL_COMPILE)

        self.gl.glBegin(self.gl.GL_LINE_LOOP)

        self.ellipse(r, r, x0, y0, 0.0)

        self.gl.glEnd()
        self.gl.glEndList()

        return genList

    def makeSphere(self, x0, y0, z0, r):
        genList = self.gl.glGenLists(1)
        self.gl.glNewList(genList, self.gl.GL_COMPILE)

        self.gl.glBegin(self.gl.GL_QUADS)

        self.sphere(x0, y0, z0, r)

        self.gl.glEnd()
        self.gl.glEndList()

        return genList

    def ellipse(self, rx, ry, x0, y0, theta):
        self.setColor(self.trolltechGreen)
        num = 100
        dt = 2 * pi / num
        for i in range(100):
            self.gl.glVertex3d(x0 + cos(theta) * rx * cos(i*dt) - sin(theta) * ry * sin(i * dt),
                               y0 + sin(theta)  * rx * cos(i*dt) + cos(theta) * ry * sin(i * dt), 0.0 )

    def sphere(self, x0, y0, z0, r):
        self.setColor(self.trolltechGreen)
        num = 100
        step_z = pi/num
        step_xy = 2 * pi/num

        angle_z = 0.0
        angle_xy = 0.0

        x = [0, 0, 0, 0]
        y = [0, 0, 0, 0]
        z = [0, 0, 0, 0]

        for i in range(num):
            angle_z = i * step_z

            for j in range(num):
                angle_xy = j * step_xy

                x[0] = r * sin(angle_z) * cos(angle_xy)
                y[0] = r * sin(angle_z) * sin(angle_xy)
                z[0] = r * cos(angle_z)

                x[1] = r * sin(angle_z + step_z) * cos(angle_xy)
                y[1] = r * sin(angle_z + step_z) * sin(angle_xy)
                z[1] = r * cos(angle_z + step_z)

                x[2] = r*sin(angle_z + step_z)*cos(angle_xy + step_xy)
                y[2] = r*sin(angle_z + step_z)*sin(angle_xy + step_xy)
                z[2] = r*cos(angle_z + step_z)

                x[3] = r * sin(angle_z) * cos(angle_xy + step_xy)
                y[3] = r * sin(angle_z) * sin(angle_xy + step_xy)
                z[3] = r * cos(angle_z)

                for k in range(4):
                    self.gl.glVertex3f(x0+x[k], y0+y[k], z0+z[k])

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()
        buttons = event.buttons()
        modifiers = event.modifiers()
        if buttons & QtCore.Qt.LeftButton and not modifiers & QtCore.Qt.META:
            if modifiers & QtCore.Qt.SHIFT:
                self.center[0] = self.center[0] - dx
                self.center[1] = self.center[1] + dy
                self.update()
            else:
                self.setRotationAngle(0, self.cameraPose[0] + dy)
                self.setRotationAngle(1, self.cameraPose[1] + dx)
        elif buttons & QtCore.Qt.RightButton or \
            (buttons & QtCore.Qt.LeftButton and modifiers & QtCore.Qt.META):
            self.setRotationAngle(0, self.cameraPose[0] + dy)
            self.setRotationAngle(2, self.cameraPose[2] + dx)
        elif buttons & QtCore.Qt.MidButton:
            if dy > 0:
                self.scale = self.scale*(1. + .01*dy)
            else:
                self.scale = self.scale*(1. - .01*dy)
        self.lastPos = event.pos()

    def wheelEvent(self, event):
        if not USE_DEPRECATED_API:
            self.scale = self.scale * pow(2.0, event.angleDelta().y() / 240.0)
        else:
            self.scale = self.scale * pow(2.0, event.delta() / 240.0)
        self.lastPos = event.pos()
        self.update()

class ProblemWidget(QtWidgets.QWidget):
    startChanged = Signal(list)
    goalChanged = Signal(list)

    def __init__(self, robotTypes):
        super(ProblemWidget, self).__init__()
        robotTypeLabel = QtWidgets.QLabel('Robot type')
        self.robotTypeSelect = QtWidgets.QComboBox()
        for robotType in robotTypes:
            self.robotTypeSelect.addItem(robotType)
        self.robotTypeSelect.setMaximumSize(200, 2000)

        self.startPose3D = Pose3DBox('Start pose')
        self.goalPose3D = Pose3DBox('Goal pose')
        self.startPose2D = Pose2DBox('Start pose')
        self.goalPose2D = Pose2DBox('Goal pose')

        #elevation2Dlabel = QtWidgets.QLabel('Elevation')
        self.elevation2D = QtWidgets.QDoubleSpinBox()
        self.elevation2D.setRange(-1000, 1000)
        self.elevation2D.setSingleStep(1)

        startGoal3D = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.startPose3D)
        layout.addWidget(self.goalPose3D)
        startGoal3D.setLayout(layout)

        startGoal2D = QtWidgets.QWidget()
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.startPose2D, 0, 0, 1, 2)
        layout.addWidget(self.goalPose2D, 1, 0, 1, 2)
        #layout.addWidget(elevation2Dlabel, 2, 0, QtCore.Qt.AlignRight)
        #layout.addWidget(self.elevation2D, 2, 1)
        startGoal2D.setLayout(layout)

        self.poses = QtWidgets.QStackedWidget()
        self.poses.addWidget(startGoal3D)
        self.poses.addWidget(startGoal2D)

        self.objectives = {'length': 'PathLengthOptimizationObjective', \
            'max min clearance': 'MaximizeMinClearanceObjective', \
            'mechanical work': 'MechanicalWorkOptimizationObjective'}
        self.objectiveSelect = QtWidgets.QComboBox()
        self.objectiveSelect.addItems(sorted(self.objectives.keys()))
        self.objectiveThreshold = QtWidgets.QDoubleSpinBox()
        self.objectiveThreshold.setRange(0, 10000)
        self.objectiveThreshold.setSingleStep(1)
        self.objectiveThreshold.setValue(10000)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(robotTypeLabel, 0, 0)
        layout.addWidget(self.robotTypeSelect, 0, 1)
        layout.addWidget(self.poses, 1, 0, 1, 2)
        self.setLayout(layout)

        self.startPose3D.valueChanged.connect(self.startPoseChange)
        self.goalPose3D.valueChanged.connect(self.goalPoseChange)
        self.startPose2D.valueChanged.connect(self.startPoseChange)
        self.goalPose2D.valueChanged.connect(self.goalPoseChange)
        self.elevation2D.valueChanged.connect(self.elevationChange)

    def setStartPose(self, value, is3D):
        self.startPose2D.setPose(value, is3D)
        if is3D:
            self.elevation2D.setValue(value().getZ())
        self.startPose3D.setPose(value, self.elevation2D.value(), is3D)
    def getStartPose(self):
        return self.startPose3D.getPose() \
            if self.poses.currentIndex() == 0 else self.startPose2D.getPose()
    def setGoalPose(self, value, is3D):
        self.goalPose2D.setPose(value, is3D)
        if is3D:
            self.elevation2D.setValue(value().getZ())
        self.goalPose3D.setPose(value, self.elevation2D.value(), is3D)
    def getGoalPose(self):
        return self.goalPose3D.getPose() \
            if self.poses.currentIndex() == 0 else self.goalPose2D.getPose()
    def startPoseChange(self, value):
        if self.poses.currentIndex() == 1:
            value[5] = self.elevation2D.value()
        self.startChanged.emit(value)
    def goalPoseChange(self, value):
        if self.poses.currentIndex() == 1:
            value[5] = self.elevation2D.value()
        self.goalChanged.emit(value)
    def elevationChange(self, value):
        state = [0, 0, self.startPose2D.rot.value(), self.startPose2D.posx.value(), \
            self.startPose2D.posy.value(), value]
        self.startChanged.emit(state)
        state = [0, 0, self.goalPose2D.rot.value(), self.goalPose2D.posx.value(), \
            self.goalPose2D.posy.value(), value]
        self.goalChanged.emit(state)
#    def getObjective(self, si):
#        obj = eval('ob.%s(si)' % self.objectives[self.objectiveSelect.currentText()])
#        obj.setCostThreshold(self.objectiveThreshold.value())
#        return obj

class Pose3DBox(QtWidgets.QGroupBox):
    valueChanged = Signal(list)

    def __init__(self, title):
        super(Pose3DBox, self).__init__(title)
        xlabel = QtWidgets.QLabel('X')
        ylabel = QtWidgets.QLabel('Y')
        zlabel = QtWidgets.QLabel('Z')
        poslabel = QtWidgets.QLabel('Position')
        rotlabel = QtWidgets.QLabel('Rotation')

        self.posx = QtWidgets.QDoubleSpinBox()
        self.posx.setRange(-1000, 1000)
        self.posx.setSingleStep(1)
        self.posy = QtWidgets.QDoubleSpinBox()
        self.posy.setRange(-1000, 1000)
        self.posy.setSingleStep(1)
        self.posz = QtWidgets.QDoubleSpinBox()
        self.posz.setRange(-1000, 1000)
        self.posz.setSingleStep(1)
        self.rotx = QtWidgets.QDoubleSpinBox()
        self.rotx.setRange(-360, 360)
        self.rotx.setSingleStep(1)
        self.roty = QtWidgets.QDoubleSpinBox()
        self.roty.setRange(-360, 360)
        self.roty.setSingleStep(1)
        self.rotz = QtWidgets.QDoubleSpinBox()
        self.rotz.setRange(-360, 360)
        self.rotz.setSingleStep(1)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(poslabel, 0, 1, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignBottom)
        layout.addWidget(rotlabel, 0, 2, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignBottom)
        layout.addWidget(xlabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(ylabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(zlabel, 3, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.posx, 1, 1)
        layout.addWidget(self.posy, 2, 1)
        layout.addWidget(self.posz, 3, 1)
        layout.addWidget(self.rotx, 1, 2)
        layout.addWidget(self.roty, 2, 2)
        layout.addWidget(self.rotz, 3, 2)
        self.setLayout(layout)

        self.posx.valueChanged.connect(self.poseChange)
        self.posy.valueChanged.connect(self.poseChange)
        self.posz.valueChanged.connect(self.poseChange)
        self.rotx.valueChanged.connect(self.poseChange)
        self.roty.valueChanged.connect(self.poseChange)
        self.rotz.valueChanged.connect(self.poseChange)

    def setPose(self, value, elevation, is3D):
        state = value()
        if is3D:
            self.posx.setValue(state.getX())
            self.posy.setValue(state.getY())
            self.posz.setValue(state.getZ())
            q = state.rotation()
            rad2deg = 180/pi
            self.rotx.setValue(rad2deg * atan2(2.*(q.w*q.x+q.y*q.z), 1.-2.*(q.x*q.x+q.y*q.y)))
#            self.roty.setValue(rad2deg * asin(max(min(2.*(q.w*q.y-q.z*q.x), 1.), -1.)))
            self.roty.setValue(rad2deg * asin(2.*(q.w*q.y-q.z*q.x)))
            self.rotz.setValue(rad2deg * atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z)))
        else:
            self.posx.setValue(state.getX())
            self.posy.setValue(state.getY())
            self.posz.setValue(elevation)
            self.rotx.setValue(0)
            self.roty.setValue(0)
            self.rotz.setValue(180 * state.getYaw() / pi)

    def getPose(self):
        state = ob.State(ob.SE3StateSpace())
        state().setX(self.posx.value())
        state().setY(self.posy.value())
        state().setZ(self.posz.value())
        angles = [self.rotx.value(), self.roty.value(), self.rotz.value()]
        c = [cos(angle*pi/180.) for angle in angles]
        s = [sin(angle*pi/180.) for angle in angles]
        rot = state().rotation()
        rot.w = c[0]*c[1]*c[2] + s[0]*s[1]*s[2]
        rot.x = s[0]*c[1]*c[2] - c[0]*s[1]*s[2]
        rot.y = c[0]*s[1]*c[2] + s[0]*c[1]*s[2]
        rot.z = c[0]*c[1]*s[2] - s[0]*s[1]*c[2]
        return state

    def poseChange(self, _):
        self.valueChanged.emit([self.rotx.value(), self.roty.value(), self.rotz.value(), \
            self.posx.value(), self.posy.value(), self.posz.value()])

class Pose2DBox(QtWidgets.QGroupBox):
    valueChanged = Signal(list)

    def __init__(self, title):
        super(Pose2DBox, self).__init__(title)
        xlabel = QtWidgets.QLabel('X')
        ylabel = QtWidgets.QLabel('Y')
        rotlabel = QtWidgets.QLabel('Rotation')

        self.posx = QtWidgets.QDoubleSpinBox()
        self.posx.setRange(-1000, 1000)
        self.posx.setSingleStep(1)
        self.posy = QtWidgets.QDoubleSpinBox()
        self.posy.setRange(-1000, 1000)
        self.posy.setSingleStep(1)
        self.rot = QtWidgets.QDoubleSpinBox()
        self.rot.setRange(-180, 179) # SO2StateSpace is parameterized from [-pi,pi)
        self.rot.setSingleStep(1)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(xlabel, 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(ylabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(rotlabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.posx, 0, 1)
        layout.addWidget(self.posy, 1, 1)
        layout.addWidget(self.rot, 2, 1)
        self.setLayout(layout)

        self.posx.valueChanged.connect(self.poseChange)
        self.posy.valueChanged.connect(self.poseChange)
        self.rot.valueChanged.connect(self.poseChange)

    def setPose(self, value, is3D):
        state = value()
        if is3D:
            self.posx.setValue(state.getX())
            self.posy.setValue(state.getY())
            q = state.rotation()
            self.rot.setValue(atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z)) * 180 / pi)
        else:
            self.posx.setValue(state.getX())
            self.posy.setValue(state.getY())
            self.rot.setValue(state.getYaw() * 180 / pi)

    def getPose(self):
        state = ob.State(ob.SE2StateSpace())
        state().setX(self.posx.value())
        state().setY(self.posy.value())
        state().setYaw(self.rot.value() * pi / 180)
        return state

    def poseChange(self, _):
        self.valueChanged.emit([0, 0, self.rot.value(), self.posx.value(), self.posy.value(), 0])

class PlannerHelperWidget(QtWidgets.QGroupBox):
    def __init__(self, name, planners):
        super(PlannerHelperWidget, self).__init__(name)
        self.setFlat(True)
        self.plannerSelect = QtWidgets.QComboBox()
        self.stackedWidget = QtWidgets.QStackedWidget()

        self.plannerSelect.addItems(sorted(planners))
        #self.plannerList = []

        self.plannerSelect.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLength)
        self.plannerSelect.currentIndexChanged[int].connect(self.stackedWidget.setCurrentIndex)

class GeometricPlannerWidget(PlannerHelperWidget):
    def __init__(self):
        planners = {'ADI', 'RRT'}
        super(GeometricPlannerWidget, self).__init__('Geometric planning', planners)

        timeLimitLabel = QtWidgets.QLabel('Time (sec.)')
        self.timeLimit = QtWidgets.QDoubleSpinBox()
        self.timeLimit.setRange(0, 10000)
        self.timeLimit.setSingleStep(1)
        self.timeLimit.setValue(10.0)

        resolutionLabel = QtWidgets.QLabel('Collision checking\nresolution')
        resolutionLabel.setAlignment(QtCore.Qt.AlignRight)
        self.resolution = QtWidgets.QDoubleSpinBox()
        self.resolution.setRange(0.001, 1.0)
        self.resolution.setSingleStep(.002)
        self.resolution.setValue(0.010)
        self.resolution.setDecimals(3)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(QtWidgets.QLabel('Planner'), 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.plannerSelect, 0, 1)
        layout.addWidget(timeLimitLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.timeLimit, 1, 1)
        layout.addWidget(resolutionLabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.resolution, 2, 1)
        layout.addWidget(self.stackedWidget, 3, 0, 1, 2)
        self.setLayout(layout)

class ControlPlannerWidget(PlannerHelperWidget):
    def __init__(self):
        planners = {'ADI', 'RRT'}
        super(ControlPlannerWidget, self).__init__('Planning with controls', planners)

        # make KPIECE1 the default planner

        timeLimitLabel = QtWidgets.QLabel('Time (sec.)')
        self.timeLimit = QtWidgets.QDoubleSpinBox()
        self.timeLimit.setRange(0, 1000)
        self.timeLimit.setSingleStep(1)
        self.timeLimit.setValue(10.0)

        propagationLabel = QtWidgets.QLabel('Propagation\nstep size')
        propagationLabel.setAlignment(QtCore.Qt.AlignRight)
        self.propagation = QtWidgets.QDoubleSpinBox()
        self.propagation.setRange(0.01, 1000.00)
        self.propagation.setSingleStep(.01)
        self.propagation.setValue(0.2)
        self.propagation.setDecimals(2)

        durationLabel = QtWidgets.QLabel('Control duration\n(min/max #steps)')
        durationLabel.setAlignment(QtCore.Qt.AlignRight)
        self.minControlDuration = QtWidgets.QSpinBox()
        self.minControlDuration.setRange(1, 1000)
        self.minControlDuration.setSingleStep(1)
        self.minControlDuration.setValue(1)
        self.maxControlDuration = QtWidgets.QSpinBox()
        self.maxControlDuration.setRange(1, 1000)
        self.maxControlDuration.setSingleStep(1)
        self.maxControlDuration.setValue(20)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(QtWidgets.QLabel('Planner'), 0, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.plannerSelect, 0, 1, 1, 2)
        layout.addWidget(timeLimitLabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.timeLimit, 1, 1, 1, 2)
        layout.addWidget(propagationLabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.propagation, 2, 1, 1, 2)
        layout.addWidget(durationLabel, 3, 0)
        layout.addWidget(self.minControlDuration, 3, 1)
        layout.addWidget(self.maxControlDuration, 3, 2)
        layout.addWidget(self.stackedWidget, 4, 0, 1, 3)
        self.setLayout(layout)

class PlannerWidget(QtWidgets.QStackedWidget):
    def __init__(self):
        super(PlannerWidget, self).__init__()
        self.geometricPlanning = GeometricPlannerWidget()
        self.controlPlanning = ControlPlannerWidget()
        self.addWidget(self.geometricPlanning)
        self.addWidget(self.controlPlanning)

class BoundsWidget(QtWidgets.QWidget):
    def __init__(self):
        super(BoundsWidget, self).__init__()
        self.bounds_high = BoundsBox('Upper bounds')
        self.bounds_low = BoundsBox('Lower bounds')
        self.resetButton = QtWidgets.QPushButton('Reset')
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.bounds_high, 0, 0)
        layout.addWidget(self.bounds_low, 1, 0)
        layout.addWidget(self.resetButton, 2, 0, QtCore.Qt.AlignRight)
        self.setLayout(layout)

class BoundsBox(QtWidgets.QGroupBox):
    valueChanged = Signal(list)

    def __init__(self, title):
        super(BoundsBox, self).__init__(title)
        xlabel = QtWidgets.QLabel('X')
        ylabel = QtWidgets.QLabel('Y')
        zlabel = QtWidgets.QLabel('Z')

        self.posx = QtWidgets.QDoubleSpinBox()
        self.posx.setRange(-1000, 1000)
        self.posx.setSingleStep(1)
        self.posy = QtWidgets.QDoubleSpinBox()
        self.posy.setRange(-1000, 1000)
        self.posy.setSingleStep(1)
        self.posz = QtWidgets.QDoubleSpinBox()
        self.posz.setRange(-1000, 1000)
        self.posz.setSingleStep(1)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(xlabel, 1, 0, QtCore.Qt.AlignRight)
        layout.addWidget(ylabel, 2, 0, QtCore.Qt.AlignRight)
        layout.addWidget(zlabel, 3, 0, QtCore.Qt.AlignRight)
        layout.addWidget(self.posx, 1, 1)
        layout.addWidget(self.posy, 2, 1)
        layout.addWidget(self.posz, 3, 1)
        self.setLayout(layout)

        self.posx.valueChanged.connect(self.boundsChange)
        self.posy.valueChanged.connect(self.boundsChange)
        self.posz.valueChanged.connect(self.boundsChange)

    def setBounds(self, value):
        self.posx.setValue(value[0])
        self.posy.setValue(value[1])
        self.posz.setValue(value[2])

    def boundsChange(self, _):
        self.valueChanged.emit([self.posx.value(), self.posy.value(), self.posz.value()])

class SolveWidget(QtWidgets.QWidget):
    def __init__(self):
        super(SolveWidget, self).__init__()
        self.solveButton = QtWidgets.QPushButton('Solve')
        self.solveButton.hide()
        self.clearButton = QtWidgets.QPushButton('Clear')
        explorationVizLabel = QtWidgets.QLabel('Show:')
        self.explorationVizSelect = QtWidgets.QComboBox()
        self.explorationVizSelect.addItem('none')
        self.explorationVizSelect.addItem('states')
        self.explorationVizSelect.addItem('states and edges')
        self.animateCheck = QtWidgets.QCheckBox('Animate')
        self.animateCheck.setChecked(True)
        speedlabel = QtWidgets.QLabel('Speed:')
        self.speedSlider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speedSlider.setTickPosition(QtWidgets.QSlider.TicksBothSides)
        self.speedSlider.setTickInterval(10)
        self.speedSlider.setSingleStep(10)
        self.speedSlider.setValue(10)
        self.speedSlider.setMaximum(100)
        self.speedSlider.setMaximumSize(200, 30)

        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.solveButton, 0, 0)
        layout.addWidget(self.clearButton, 0, 1)
        layout.addWidget(explorationVizLabel, 0, 2, QtCore.Qt.AlignRight)
        layout.addWidget(self.explorationVizSelect, 0, 3)
        layout.addWidget(self.animateCheck, 0, 4)
        layout.addWidget(speedlabel, 0, 5)
        layout.addWidget(self.speedSlider, 0, 6)
        self.setLayout(layout)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
