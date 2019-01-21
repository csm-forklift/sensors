#!/usr/bin/env python
''' Test code for creating a Qt GUI with two sliders and a text box. The sliders should be used for controlling the accelerator and brake actuator positions and the textbox receives a target position for the stepper motor.'''

import sys
from PySide2.QtCore import *
from PySide2.QtWidgets import *
import rospy
from std_msgs.msg import Float32, Float64


class Sliders(QWidget):
    def __init__(self, parent = None):
        super(Sliders, self).__init__(parent)

        # Create a vertical layout
        self.vlayout1 = QVBoxLayout()

        #===== Slider 1 =====#
        # Add a label above the slider
        self.label1 = QLabel("Accelerator")
        self.label1.setAlignment(Qt.AlignLeft)
        self.vlayout1.addWidget(self.label1)

        # Add the first slider
        self.slider1_divider = 10.0
        self.slider1 = QSlider(Qt.Horizontal)
        self.slider1.setMinimum(0)
        self.slider1.setMaximum(10)
        self.slider1.setValue(0)
        self.slider1.setTickPosition(QSlider.TicksBelow)
        self.slider1.setTickInterval(1)

        # Creat grid layout
        self.gridlayout1 = QGridLayout()

        # Create tick labels
        self.gridlayout1.addWidget(self.slider1, 0, 0, 1, self.slider1_divider+1)

        self.tick_labels1 = []
        for i in range(0,int(self.slider1_divider + 1)):
            self.tick_labels1.append(QLabel(str(i/self.slider1_divider)))
            self.gridlayout1.addWidget(self.tick_labels1[i], 1, i, 1, 1, Qt.AlignCenter)
        self.vlayout1.addLayout(self.gridlayout1)
        self.line1 = QFrame()
        self.line1.setFrameShape(QFrame.HLine)
        self.line1.setLineWidth(2)
        self.vlayout1.addWidget(self.line1)

        #===== Slider 2 =====#
        # Add a label above the second slider
        self.label2 = QLabel("Brake")
        self.label2.setAlignment(Qt.AlignLeft)
        self.vlayout1.addWidget(self.label2)

        # Add the second slider
        self.slider2_divider = 10.0
        self.slider2 = QSlider(Qt.Horizontal)
        self.slider2.setMinimum(0)
        self.slider2.setMaximum(10)
        self.slider2.setValue(0)
        self.slider2.setTickPosition(QSlider.TicksBelow)
        self.slider2.setTickInterval(1)

        # Creat grid layout
        self.gridlayout2 = QGridLayout()

        # Create tick labels
        self.gridlayout2.addWidget(self.slider2, 0, 0, 1, self.slider2_divider+1)

        self.tick_labels2 = []
        for i in range(0,int(self.slider2_divider + 1)):
            self.tick_labels2.append(QLabel(str(i/self.slider2_divider)))
            self.gridlayout2.addWidget(self.tick_labels2[i], 1, i, 1, 1, Qt.AlignCenter)
        self.vlayout1.addLayout(self.gridlayout2)
        self.line2 = QFrame()
        self.line2.setFrameShape(QFrame.HLine)
        self.line2.setLineWidth(2)
        self.vlayout1.addWidget(self.line2)

        # Create a horizontal layout
        self.hlayout1 = QHBoxLayout()

        # Add a label for the textbox
        self.label3 = QLabel("Motor Controller")
        self.position_label = QLabel("Position: ")
        self.vlayout1.addWidget(self.label3)
        self.hlayout1.addWidget(self.position_label)

        # Create and add the textbox
        self.textbox1 = QLineEdit()
        self.textbox1.setPlaceholderText("Enter target position")
        self.textbox1.setMaxLength(25)
        self.hlayout1.addWidget(self.textbox1)
        self.vlayout1.addLayout(self.hlayout1)

        self.error_label = QLabel()
        self.error_label.setAlignment(Qt.AlignLeft)
        self.vlayout1.addWidget(self.error_label)

        # Add spacer
        self.vlayout1.addStretch()

        # Set signals and widget setup
        self.slider1.valueChanged.connect(self.slider1Change)
        self.slider2.valueChanged.connect(self.slider2Change)
        self.textbox1.returnPressed.connect(self.checkTargetInput)
        self.setLayout(self.vlayout1)
        self.setWindowTitle("Controls")
        self.resize(500, 300)

        #===== Begin ROS stuff =====#
        rospy.init_node("control_gui")
        self.accel_pub = rospy.Publisher("/throttle_controller/accelerator/input", Float64, queue_size=10)
        self.brake_pub = rospy.Publisher("/throttle_controller/brake/input", Float32, queue_size=10)
        self.steer_pub = rospy.Publisher("/steering_controller/target_position", Float64, queue_size=10)

    def slider1Change(self, value):
        value /= self.slider1_divider
        msg = Float64(value)
        self.accel_pub.publish(msg)

    def slider2Change(self, value):
        value /= self.slider2_divider
        msg = Float32(value)
        self.brake_pub.publish(msg)

    def checkTargetInput(self):
        try:
            target = float(self.textbox1.text())
            msg = Float64(target)
            self.steer_pub.publish(msg)
        except ValueError:
            self.error_label.setText("")
            self.error_label.setText("Invalid input. Position must be a 'float'")

    def closeEvent(self, e):
        rospy.signal_shutdown("GUI window closed")


def main():
    # Qt Startup
    app = QApplication(sys.argv)
    window = Sliders()
    window.show()

    sys.exit(app.exec_())

main()
