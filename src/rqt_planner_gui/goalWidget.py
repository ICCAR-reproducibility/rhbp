## rqt widget displaying a goal
#Created on 10.08.2015
#@author: stephan

import os
import rospy
import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from behaviourPlannerPython.srv import Activate
from PyQt4.QtCore import pyqtSignal

# Custum Widget for goal
class GoalWidget(QWidget):
    updateGUIsignal = pyqtSignal(dict)
    def __init__(self, name):
        super(GoalWidget, self).__init__()
        self._name = name
        # Give QObjects reasonable names
        self.setObjectName(self._name + 'Widget')
        # Get path to UI file which should be in the "resource" folder of this node
        ui_file = os.path.join(rospkg.RosPack().get_path('behaviourPlannerPython'), 'src', 'rqt_planner_gui', 'resource', 'goal.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.goalGroupBox.setTitle(self._name)
        self.activatedCheckbox.toggled.connect(self.activationCallback)
        self.updateGUIsignal.connect(self.updateGUI)

    def __del__(self):
        self.__deleted = True
    
    def updateGUI(self, newValues):
        self.activatedCheckbox.setChecked(newValues["activated"])
        self.fulfillmentDoubleSpinBox.setValue(newValues["fulfillment"])
        self.fulfillmentDoubleSpinBox.setToolTip("{0}".format(newValues["fulfillment"]))
        self.activeCheckbox.setChecked(newValues["active"])
        self.wishesLabel.setText(newValues["wishes"])
        self.wishesLabel.setToolTip(newValues["wishesTooltip"])
        
    def refresh(self, msg):
        """
        Refreshes the widget with data from the new message.
        """
        assert self._name == msg.name
        self.updateGUIsignal.emit({
                                   "activated" : msg.activated,
                                   "fulfillment" : msg.satisfaction,
                                   "active" : msg.active,
                                   "wishes" : "\n".join(map(lambda x: "{0}: {1:.4g}".format(x.sensorName, x.indicator), msg.wishes)),
                                   "wishesTooltip" : "\n".join(map(lambda x: "{0}: {1}".format(x.sensorName, x.indicator), msg.wishes))
                                  })
    
    def activationCallback(self, status):
        rospy.logdebug("Waiting for service %s", self._name + 'Activate')
        rospy.wait_for_service(self._name + 'Activate')
        activateRequest = rospy.ServiceProxy(self._name + 'Activate', Activate)
        activateRequest(status)
        rospy.logdebug("Set activated of %s goal to %s", self._name, status)
       