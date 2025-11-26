import sys
import threading

import rclpy
from rclpy.node import Node

from rqt_gui_py.plugin import Plugin
from python_qt_binding import QtWidgets, QtCore

from interfaces.srv import MoveRequest
from std_srvs.srv import Trigger

_rclpy_initialized = False

class MoveitClientNode(Node):
    """Plain ROS2 node that talks to your services."""

    def __init__(self):
        super().__init__('ur5_moveit_rqt_node')

        self.move_cli = self.create_client(MoveRequest, '/moveit_path_plan')
        self.stop_cli = self.create_client(Trigger, '/moveit_stop')

        self.get_logger().info('Waiting for services...')
        self.move_cli.wait_for_service()
        self.stop_cli.wait_for_service()
        self.get_logger().info('Services available.')

    def call_move(self, command, positions):
        req = MoveRequest.Request()
        req.command = command
        req.positions = positions

        self.get_logger().info(f'Calling move: {command} {positions}')
        future = self.move_cli.call_async(req)
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
        self._widget.setWindowTitle('UR5 MoveIt Control')

        layout = QtWidgets.QVBoxLayout(self._widget)

        self.status_label = QtWidgets.QLabel('Ready')
        layout.addWidget(self.status_label)

        btn_home = QtWidgets.QPushButton('Home')
        btn_pose = QtWidgets.QPushButton('Pose Test')
        btn_cart = QtWidgets.QPushButton('Cartesian Down')
        btn_stop = QtWidgets.QPushButton('STOP')

        btn_row = QtWidgets.QHBoxLayout()
        for b in (btn_home, btn_pose, btn_cart, btn_stop):
            btn_row.addWidget(b)
        layout.addLayout(btn_row)

        # Connect buttons
        btn_home.clicked.connect(self._on_home)
        btn_pose.clicked.connect(self._on_pose)
        btn_cart.clicked.connect(self._on_cart)
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
        positions = [-1.3, 1.57, -1.83, -1.57, 0.0, 0.0]
        self._call_move('joint', positions)

    def _on_pose(self):
        positions = [0.60, 0.35, 0.65, -3.1415, 0.0, 1.57]
        self._call_move('pose', positions)

    def _on_cart(self):
        positions = [0.50, 0.35, 0.30, -3.1415, 0.0, 1.57]
        self._call_move('cartesian', positions)

    def _on_stop(self):
        future = self.node.call_stop()
        self.status_label.setText('Stop requested')

    def _call_move(self, command, positions):
        self.status_label.setText(f'Sending {command}...')
        future = self.node.call_move(command, positions)

    # ---------- rclpy spinning ----------

    def _spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)


def main():
    from rqt_gui.main import Main
    main = Main()
    sys.exit(main.main(sys.argv, standalone='ur5_moveit_rqt'))
