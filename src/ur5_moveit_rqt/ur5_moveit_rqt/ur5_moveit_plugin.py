import sys
import threading

import rclpy
from rclpy.node import Node

from rqt_gui_py.plugin import Plugin
from python_qt_binding import QtWidgets, QtCore

from interfaces.srv import MoveRequest, BrainCmd
from std_srvs.srv import Trigger

_rclpy_initialized = False

class MoveitClientNode(Node):
    """Plain ROS2 node that talks to your services."""

    def __init__(self):
        super().__init__('ur5_moveit_rqt_node')

        self.move_cli = self.create_client(MoveRequest, '/moveit_path_plan')
        self.stop_cli = self.create_client(Trigger, '/moveit_stop')
        self.brain_cli = self.create_client(BrainCmd, '/brain_srv')

        self.get_logger().info('Waiting for services...')
        self.move_cli.wait_for_service()
        self.stop_cli.wait_for_service()
        self.brain_cli.wait_for_service()
        self.get_logger().info('Services available.')

    def call_move(self, command, positions):
        req = MoveRequest.Request()
        req.command = command
        req.positions = positions

        self.get_logger().info(f'Calling move: {command} {positions}')
        future = self.move_cli.call_async(req)
        return future
    
    def call_brain(self, command):
        req = BrainCmd.Request()
        req.command = command

        self.get_logger().info(f'Calling brain command: {command}')
        future = self.brain_cli.call_async(req)
        return future

    def call_stop(self):
        req = Trigger.Request()
        self.get_logger().warn('Calling STOP!')
        future = self.stop_cli.call_async(req)
        return future


class UR5MoveItPlugin(Plugin):

    def __init__(self, context):
        super(UR5MoveItPlugin, self).__init__(context)
        self.setObjectName('UR5MoveItPlugin')

        global _rclpy_initialized
        if not _rclpy_initialized:
            try:
                rclpy.init(args=None)
            except RuntimeError:
                pass
            _rclpy_initialized = True

        self.node = MoveitClientNode()

        # Simple QWidget with buttons
        self._widget = QtWidgets.QWidget()
        self._widget.setWindowTitle('Soil Sampler Control')

        layout = QtWidgets.QVBoxLayout(self._widget)

        self.status_label = QtWidgets.QLabel('Ready')
        layout.addWidget(self.status_label)

        btn_home = QtWidgets.QPushButton('Home')
        btn_sample = QtWidgets.QPushButton('Sample')
        btn_topography = QtWidgets.QPushButton('Topography')
        btn_vertical = QtWidgets.QPushButton('Vertical')
        btn_heatmap = QtWidgets.QPushButton('Heat-map')
        btn_stop = QtWidgets.QPushButton('STOP')

        btn_row = QtWidgets.QHBoxLayout()
        for b in (btn_home, btn_sample, btn_topography, btn_vertical, btn_heatmap, btn_stop):
            btn_row.addWidget(b)
        layout.addLayout(btn_row)

        # Connect buttons
        btn_home.clicked.connect(self._on_home)
        btn_sample.clicked.connect(self._on_sample)
        btn_topography.clicked.connect(self._on_topography)
        btn_vertical.clicked.connect(self._on_vertical)
        btn_heatmap.clicked.connect(self._on_heatmap)
        btn_stop.clicked.connect(self._on_stop)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # Timer to spin node without blocking Qt
        self._timer = QtCore.QTimer(self._widget)
        self._timer.timeout.connect(self._spin_once)
        self._timer.start(10)  # 100 Hz-ish

    def shutdown_plugin(self):
        self._timer.stop()
        self.node.destroy_node()
        #rclpy.shutdown()

    # ---------- Button handlers ----------

    def _on_home(self):
        positions = [-1.57, 1.57, -1.83, -1.57, 0.0, 0.0]
        self._call_move('joint', positions)

    def _on_sample(self):
        self._call_brain('soil_sampling')

    def _on_topography(self):
        self._call_brain('topography')

    def _on_vertical(self):
        self._call_brain('vertical')

    def _on_heatmap(self):
        self._call_brain('heat_map')

    def _on_stop(self):
        future = self.node.call_stop()
        self.status_label.setText('Stop requested')

    def _call_move(self, command, positions):
        self.status_label.setText(f'Sending {command}...')
        future = self.node.call_move(command, positions)

    def _call_brain(self, command):
        self.status_label.setText(f'Sending {command}...')
        future = self.node.call_brain(command)

    # ---------- rclpy spinning ----------

    def _spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)


def main():
    from rqt_gui.main import Main
    main = Main()
    sys.exit(main.main(sys.argv, standalone='ur5_moveit_rqt'))
