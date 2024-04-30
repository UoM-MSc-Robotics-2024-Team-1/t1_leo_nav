from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLineEdit, QLabel
from PyQt5.QtCore import QTimer, QObject, pyqtSignal
import sys
import rclpy
import subprocess
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from t1_nav.robot_navigator import BasicNavigator, NavigationResult
import subprocess
import threading

class Worker(QObject):
    def __init__(self, window):
        super().__init__()
        self.window = window

    def check_connections(self):
        try:
            subprocess.check_output("ping -c 1 192.168.1.100", shell=True)
            self.window.connection_status_nuc.setText("Connected to NUC")
        except subprocess.CalledProcessError:
            self.window.connection_status_nuc.setText("Not connected to NUC")
        except Exception as e:
            self.window.connection_status_nuc.setText(f"Error: {str(e)}")

        try:
            subprocess.check_output("ping -c 1 10.0.0.1", shell=True)
            self.window.connection_status_robot.setText("Connected to Rover")
        except subprocess.CalledProcessError:
            self.window.connection_status_robot.setText("Not connected to Rover")
        except Exception as e:
            self.window.connection_status_robot.setText(f"Error: {str(e)}")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        nav = BasicNavigator()
        self.setWindowTitle("T1 Rover App")
        self.setGeometry(300, 200, 480, 300)

        # Create a QWidget as the central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        btn6 = QPushButton('Start T1 Rover System (State Machine)', self)

        # Create a vertical layout
        layout = QVBoxLayout()
        self.x_label = QLabel("X:", self)
        self.x_input = QLineEdit(self)
        self.y_label = QLabel("Y:", self)
        self.y_input = QLineEdit(self)
        self.connection_status_nuc = QLabel()
        self.connection_status_robot = QLabel()

        btn1 = QPushButton('Navigate To Position', self)
        btn2 = QPushButton('Stop', self)
        btn4 = QPushButton('Return To Start Position', self)
        btn5 = QPushButton('Set Start Position', self)

        btn2.clicked.connect(lambda: nav.cancelNav())
        btn1.clicked.connect(lambda: nav.goToPose(self.create_pose_stamped(
            float(self.x_input.text()) if self.x_input.text() else 0.0,
            float(self.y_input.text()) if self.y_input.text() else 0.0,
            0.0, 0.0)))
        btn4.clicked.connect(lambda: self.return_home(nav))
        btn5.clicked.connect(lambda: nav._setInitialPose())
        btn6.clicked.connect(self.start_state_machine)
        
        layout.addWidget(self.connection_status_nuc)
        layout.addWidget(self.connection_status_robot)
        layout.addWidget(btn6)
        layout.addWidget(self.x_label)
        layout.addWidget(self.x_input)
        layout.addWidget(self.y_label)
        layout.addWidget(self.y_input)
        layout.addWidget(btn1)
        layout.addWidget(btn2)
        layout.addWidget(btn4)
        layout.addWidget(btn5)
        central_widget.setLayout(layout)

        # Setup QTimer to periodically check connection status
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_connections)
        self.timer.start(2000)  # Check every 2 seconds
        self.check_connections()

    def start_state_machine(self):
        if self.check_connection_nuc():
            subprocess.run(["ssh", "username@nuc_ip_address", "ros2 run t1_nav robot_state_machine"])
        else:
            subprocess.run(["ros2", "run", "t1_nav", "robot_state_machine"])

    def return_home(self, nav):
        if nav.initial_pose:
            nav.goToPose(nav.initial_pose)
        else:
            print("Initial pose not set, please open application when robot is at start position.")

    def check_connections(self):
        worker = Worker(self)
        thread = threading.Thread(target=worker.check_connections)
        thread.start()

    def create_pose_stamped(self, x, y, z, yaw, frame_id='map'):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
