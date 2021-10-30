import os
import rospy
import rospkg
import json
import math
import numpy as np

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from PyQt5.QtWidgets import QFileDialog


from geometry_msgs.msg import Twist,Pose2D

from std_srvs.srv import Empty as EmptyServiceCall

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from cpp_pkg.srv import *


import roslib
roslib.load_manifest('rqt_mypkg')
import actionlib


from rqt_mypkg.msg import move_to_goalAction, move_to_goalGoal, move_to_goalResult



class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns


        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._widget.pushButton_Reset.clicked.connect(self.btn_reset)
        self._widget.pushButton_Open.clicked.connect(self.btn_open)
        self._widget.pushButton_Pause.clicked.connect(self.btn_pause)
        self._widget.pushButton_Resume.clicked.connect(self.btn_resume)
        # Add widget to the user interface
        context.add_widget(self._widget)


        self.client = actionlib.SimpleActionClient('move_to_target', move_to_goalAction)
        self.client.wait_for_server()
        self.total_points = 0
        self.current_point = 0
        self.points= None


    def btn_open(self):
        print("button_open gets clicked.")
        pathfile = QFileDialog.getOpenFileName(self._widget, 'Open a JSON file', '.', 'All Files (*.*)')
        if pathfile != ('', ''):
            print(pathfile[0])
            f = open (pathfile[0], "r")
            data=f.read()
            f.close()
            try:
                self.points = json.loads(data)
            except ValueError:
                rospy.loginfo("Invalid JSON file")
                return
        else:
            return

        rate = rospy.Rate(10)
        self.total_points = len(self.points)
        self.current_point = 0
        rospy.loginfo("total points %d" % self.total_points)
        self.send_target()
        



    def btn_reset(self):
        self.total_points = 0
        self.current_point = 0
        self.client.cancel_all_goals()
        self.manual_commands_client("turtle1", "reset")

        rospy.wait_for_service('reset')
        reset_simulator = rospy.ServiceProxy('reset', EmptyServiceCall)
        reset_simulator()



    def btn_pause(self):
        self.manual_commands_client("turtle1", "pause")

    def btn_resume(self):
        self.manual_commands_client("turtle1", "resume")

    def send_target(self):
        rospy.loginfo("Target (%d) ==> %s, %s" %  (self.current_point, float(self.points[self.current_point]["x"]) , float(self.points[self.current_point]["y"])))
        goal=move_to_goalGoal(float(self.points[self.current_point]["x"]) , float(self.points[self.current_point]["y"]))
        self.client.send_goal(goal,active_cb=self.callback_active,feedback_cb=self.callback_feedback,done_cb=self.callback_done)
        self.current_point += 1
        

    def manual_commands_client(self, name, order):
        rospy.loginfo("Sending command %s" %order)
        rospy.wait_for_service('manual_commands')
        try:
            manual_commands = rospy.ServiceProxy('manual_commands', ManualCommands)
 
            resp1 = manual_commands(name, order)
            return
        except rospy.ServiceException, e:
            rospy.loginfo("Service call failed: %s"%e)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
    def callback_active(self):
        rospy.loginfo("Action server is processing the goal")


    def callback_done(self, state, result):
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))
        if self.current_point < self.total_points:
            self.send_target()


    def callback_feedback(self, feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))


