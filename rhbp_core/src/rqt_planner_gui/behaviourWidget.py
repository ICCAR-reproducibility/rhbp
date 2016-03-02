## rqt widget displaying a behaviour
#Created on 10.08.2015
#@author: stephan

import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from behaviour_planner.srv import Activate, ForceStart, SetInteger
from PyQt4.QtCore import pyqtSignal

# Custum Widget for Behaviour
class BehaviourWidget(QWidget):
    updateGUIsignal = pyqtSignal(dict)
    
    def __init__(self, name, plugin):
        super(BehaviourWidget, self).__init__()
        self._name = name
        self._overviewPlugin = plugin # this is where we get the planner prefix from
        # Give QObjects reasonable names
        self.setObjectName(self._name + 'Widget')
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('behaviour_planner'), 'src', 'rqt_planner_gui', 'resource', 'behaviour.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.behaviourGroupBox.setTitle(self._name)
        self.activatedCheckbox.toggled.connect(self.activationCallback)
        self.forceStartButton.toggled.connect(self.forceStartCallback)
        self.priorityPushButton.clicked.connect(self.setPriorityCallback)
        self.executionTimeoutPushButton.clicked.connect(self.setExecutionTimeoutCallback)
        self.updateGUIsignal.connect(self.updateGUI)

    def __del__(self):
        self.__deleted = True
    
    def updateGUI(self, newValues):
        self.activatedCheckbox.setChecked(newValues["activated"])
        self.satisfactionDoubleSpinBox.setValue(newValues["satisfaction"])
        self.satisfactionDoubleSpinBox.setToolTip("{0}".format(newValues["satisfaction"]))
        self.activeLabel.setText(newValues["active"])
        self.wishesLabel.setText(newValues["wishes"])
        self.wishesLabel.setToolTip(newValues["wishesTooltip"])
        self.correlationsLabel.setText(newValues["correlations"])
        self.correlationsLabel.setToolTip(newValues["correlationsTooltip"])
        self.activationDoubleSpinBox.setValue(newValues["activation"])
        self.activationDoubleSpinBox.setToolTip("{0}".format(newValues["activation"]))
        self.readyThresholdDoubleSpinBox.setValue(newValues["readyThreshold"])
        self.readyThresholdDoubleSpinBox.setToolTip("{0}".format(newValues["readyThreshold"]))
        self.executableLabel.setText(newValues["executable"])
        self.isExecutingLabel.setText(newValues["isExecuting"])
        self.progressDoubleSpinBox.setValue(newValues["progress"])
        self.progressDoubleSpinBox.setToolTip("{0}".format(newValues["progress"]))
        self.executionTimeSpinBox.setValue(newValues["executionTime"])
        if not self.prioritySpinBox.hasFocus():
            self.prioritySpinBox.setValue(newValues["priority"])
        if not self.executionTimeoutSpinBox.hasFocus():
            self.executionTimeoutSpinBox.setValue(newValues["executionTimeout"])
        self.interruptableLabel.setText(newValues["interruptable"])
        
    def refresh(self, msg):
        """
        Refreshes the widget with data from the new message.
        """
        assert self._name == msg.name
        self.updateGUIsignal.emit({
                                   "activated" : msg.activated,
                                   "satisfaction" : msg.satisfaction,
                                   "active" : str(msg.active),
                                   "wishes" : "\n".join(map(lambda x: "{0}: {1:.4g}".format(x.sensorName, x.indicator), msg.wishes)),
                                   "wishesTooltip" : "\n".join(map(lambda x: "{0}: {1}".format(x.sensorName, x.indicator), msg.wishes)),
                                   "correlations" : "\n".join(map(lambda x: "{0}: {1:.4g}".format(x.sensorName, x.indicator), msg.correlations)),
                                   "correlationsTooltip" : "\n".join(map(lambda x: "{0}: {1}".format(x.sensorName, x.indicator), msg.correlations)),
                                   "activation" : msg.activation,
                                   "readyThreshold" : msg.threshold,
                                   "executable" : str(msg.executable),
                                   "isExecuting" : str(msg.isExecuting),
                                   "executionTimeout" : msg.executionTimeout,
                                   "executionTime" : msg.executionTime,
                                   "progress" : msg.progress,
                                   "priority" : msg.priority,
                                   "interruptable" : str(msg.interruptable)
                                  })
    
    def activationCallback(self, status):
        rospy.logdebug("Waiting for service %s", self._name + 'Activate')
        rospy.wait_for_service(self._name + 'Activate')
        activateRequest = rospy.ServiceProxy(self._name + 'Activate', Activate)
        activateRequest(status)
        rospy.logdebug("Set activated of %s to %s", self._name, status)
        
    def forceStartCallback(self, status):
        rospy.logdebug("Waiting for service %s", self._overviewPlugin.plannerPrefix + 'ForceStart')
        rospy.wait_for_service(self._overviewPlugin.plannerPrefix + 'ForceStart')
        forceStartRequest = rospy.ServiceProxy(self._overviewPlugin.plannerPrefix + 'ForceStart', ForceStart)
        forceStartRequest(self._name, status)
        if status == True:
            self.forceStartButton.setText("back to normal")
        else:
            self.forceStartButton.setText("start")
        rospy.logdebug("Set forceStart of %s to %s", self._name, status)
    
    def setPriorityCallback(self):
        rospy.logdebug("Waiting for service %s", self._name + 'Priority')
        rospy.wait_for_service(self._name + 'Priority')
        priorityRequest = rospy.ServiceProxy(self._name + 'Priority', SetInteger)
        priorityRequest(self.prioritySpinBox.value())
        rospy.logdebug("Set priority of %s to %s", self._name, self.prioritySpinBox.value())
    
    def setExecutionTimeoutCallback(self):
        rospy.logdebug("Waiting for service %s", self._name + 'ExecutionTimeout')
        rospy.wait_for_service(self._name + 'ExecutionTimeout')
        priorityRequest = rospy.ServiceProxy(self._name + 'ExecutionTimeout', SetInteger)
        priorityRequest(self.executionTimeoutSpinBox.value())
        rospy.logdebug("Set executionTimeout of %s to %s", self._name, self.executionTimeoutSpinBox.value())
        
