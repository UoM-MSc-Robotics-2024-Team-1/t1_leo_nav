import rclpy
from std_msgs.msg import Bool
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from .patroling_node import PatrolNode
from std_srvs.srv import SetBool
from .robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from t1_interfaces.srv import Command
from tf2_ros import TransformStamped


##! TERMINAL COMMANDS: !##
# Run the state machine node with: ros2 run t1_nav robot_state_machine
# Stop and start patrolling by running (start = true, stop = false): ros2 service call /toggle_patrol std_srvs/SetBool "{data: true}"
# Change state in terminal by running: ros2 service call /change_state t1_interfaces/srv/Command "{command: 'PATROL'}"

# ros2 topic pub /aruco_markers geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}}'

# ros2 service call /toggle_manipulator std_srvs/SetBool "{data: false}"

# ros2 launch explore_lite explore.launch.py 

# ros2 run t1_nav robot_state_machine 

# Run state machine, run t1 nav demo.
# run explore
# when exploring/patrolling publish object position
# once reached object position it should continue patrolling


# States for the state machine
class State:
    PATROL = 'PATROL'
    EXPLORE = 'EXPLORE'
    MOVE_TO_OBJECT = 'MOVE_TO_OBJECT'
    PICKUP_OBJECT = 'PICKUP_OBJECT'
    IDLE = 'IDLE'

# State machine to control the robot's behavior
class RobotStateMachine(LifecycleNode):
    def __init__(self):
        super().__init__('robot_state_machine')
        self.patrol_node = None
        self.nav = BasicNavigator()


        # Initial state
        self.current_state = State.EXPLORE # Start in the patrol state
        self.current_nav_state = State.EXPLORE
        #self.transition_timer = self.create_timer(15, self.transition_states_timer)  # Transition states every 5 seconds for testing

        # Service clients to send commands to other nodes
        self.patrol_service_client = self.create_client(SetBool, '/toggle_patrol') # Service client to send patrol commands (Start/Stop)
        self.explore_service_client = self.create_client(SetBool, '/toggle_exploration') # Service client to send manipulator commands (Start/Stop)
        self.manipulator_service_client = self.create_client(SetBool, '/toggle_manipulator') # Service client to send manipulator commands (Start/Stop)

        # Service to change state
        self.state_service = self.create_service(Command, '/change_state', self.handle_change_state) # Service to change state

        # Check if exploration is finished
        self.subscription = self.create_subscription(Bool, 'exploration_finished', self.check_exploration_status, 10)

        # Listen for object positions
        self.listen_for_object_position()  # Listen for topic to detect object position
        self.listen_for_object_position_tf()  # Listen for TF frame to detect object position

        # Timers to check for navigation and manipulator status
        self.nav_timer = None  # Timer for periodically checking navigation status for when 
        self.manipulator_timer = None  # Timer for periodically checking manipulator status

    ## SEND COMMANDS ##------------------------------------------------------------
        
    # Send patrol command to the patrol node
    def send_patrol_command(self, start_patrolling):
        while not self.patrol_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = SetBool.Request()
        request.data = start_patrolling
        future = self.patrol_service_client.call_async(request)
        future.add_done_callback(self.handle_service_response)
        self.get_logger().info(f"Sent patrol command (state machine): {'start' if start_patrolling else 'stop'}")

    # Send explore command to the exploration node
    def send_explore_command(self, start_exploring):
        while not self.explore_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = SetBool.Request()
        request.data = start_exploring
        future = self.explore_service_client.call_async(request)
        future.add_done_callback(self.handle_service_response)
        self.get_logger().info(f"Sent explore command (state machine): {'start' if start_exploring else 'stop'}")

    # Send manipulator command to start manipulator
    def send_manipulator_command(self, start_manipulator):
        while not self.manipulator_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = SetBool.Request()
        request.data = self.manipulator_service_client.call_async(request)
        future = self.manipulator_service_client.call_async(request)
        future.add_done_callback(self.handle_service_response)
        self.get_logger().info(f"Sent Manipualtor command (state machine): {'start' if start_manipulator else 'stop'}")

    ## CHECK STATUS ##------------------------------------------------------------    
    def check_manipulator_status(self):
        while not self.manipulator_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = SetBool.Request()
        request.data = None  # Do not change the state
        future = self.manipulator_service_client.call_async(request)
        future.add_done_callback(self.handle_manipulator_status_response)

    # Check if exploration is finished
    def check_exploration_status(self, msg):
        if msg.data and self.current_state == State.EXPLORE:
            self.get_logger().info('Exploration complete!')
            self.transition_state(State.PATROL)

    ## SERVICE RESPONSES ##------------------------------------------------------------

    # Handle manipulator status response (true or false)
    def handle_manipulator_status_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Manipulator is active')
            else:
                self.get_logger().info('Manipulator is not active')
                self.transition_state(self.current_nav_state) # Return to the current navigation state

        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

    # Handle service response (true or false)
    def handle_service_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Service call succeeded: {response.message}')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

    # Handle service request to change state
    def handle_change_state(self, request, response):
        new_state = request.command
        if new_state in State.__dict__.values():
            self.transition_state(new_state)
            response.success = True
            response.message = f'Successfully changed state to {new_state}'
        else:
            response.success = False
            response.message = f'Invalid state {new_state}'
        return response

    #Handle logic when changing states
    def transition_state(self, new_state):
        if new_state == State.EXPLORE:
            self.get_logger().info('Starting exploration...')
            self.current_nav_state = new_state

        elif new_state == State.PATROL:
            self.send_patrol_command(True) # Start patrolling
            self.current_nav_state = new_state
            
        elif new_state == State.MOVE_TO_OBJECT:
            self.send_patrol_command(False) # Stop patrolling
            self.send_explore_command(False) # Stop exploring

        elif new_state == State.PICKUP_OBJECT:
            self.pickup_object()
            
        elif new_state == State.IDLE:
            self.send_patrol_command(False) # Stop patrolling
            self.send_explore_command(False) # Stop exploring

        self.current_state = new_state
        self.get_logger().info('Transitioning to state: ' + new_state)


    # Change state to the next one for testing purposes
    def transition_states_timer(self):
        # Define a sequence of states to transition through for testing
        state_sequence = [State.PATROL, State.MOVE_TO_OBJECT]
        current_index = state_sequence.index(self.current_state)
        next_index = (current_index + 1) % len(state_sequence)
        new_state = state_sequence[next_index]
        self.transition_state(new_state)

    # Add logic to move to object here
    def move_to_object(self, msg: PoseStamped):
        if self.current_state == State.MOVE_TO_OBJECT:
            return
        
        self.get_logger().info('Moving to object...')
        
        #if timer exists delete it
        if self.nav_timer is not None:
                self.nav_timer.cancel()
        
        self.transition_state(State.MOVE_TO_OBJECT)

         # Wait for a short delay before transitioning to MOVE_TO_OBJECT state
        time.sleep(0.5)  # Adjust the delay time as needed
    

        self.nav.goToPose(msg) # Move to the object position

        # Start a timer to check if the object has been reached
        self.nav_timer = self.create_timer(1.0, self.check_object_reached)

    # Check if the object has been reached
    def check_object_reached(self):
        if self.nav.isNavComplete():
            self.nav_timer.cancel()
            self.transition_state(State.PICKUP_OBJECT)

    # Add logic to pick up object here (e.g. call a service or action server or script)
    # It needs to let the state machine know when the object has been picked up
    def pickup_object(self):
        self.get_logger().info('Picking up object...')

        #Placeholder until actual pickup logic is fixed
        if self.current_nav_state == State.EXPLORE:
            self.transition_state(State.PATROL)
        else:
            self.transition_state(State.PATROL)


        #self.send_manipulator_command(True) # Start manipulator
        #self.manipulator_timer = self.create_timer(1.0, self.check_manipulator_status)  # Check every 1 second

    # listen for a topic to be published with the object position
    def listen_for_object_position(self):
        self.get_logger().info('Listening for camera position...')
        # Create a subscription to the camera position topic
        self.subscription = self.create_subscription(
            PoseStamped,  # Assuming Position is the message type
            '/aruco_markers',  # Assuming this is the topic name
            self.move_to_object, # Callback function
            10  # Queue size
        )

    # listen for a tf frame to be published with the object position
    def listen_for_object_position_tf(self):
        self.get_logger().info('Listening for AprilTag detection...')
        # Create a subscription to TF frames
        self.tf_subscriber = self.create_subscription(
            TransformStamped,  # Message type for TF frames
            '/tf',  # Assuming this is the topic name for TF frames
            self.tf_callback,  # Callback function
            10  # Queue size
    )

    #check if tf frame is found
    def tf_callback(self, msg):
        # Check if the TF frame corresponding to the AprilTag is available
        if self.tf_buffer.can_transform('base_footprint', 'tagStandard41h12:ID', rclpy.time.Time()):
            self.get_logger().info('AprilTag detected, moving to object...')
            self.move_to_object(msg)  # Call the move_to_object method
            # After detecting the AprilTag, you may want to unsubscribe from further TF messages
            self.tf_subscriber.destroy()  # Unsubscribe from TF messages


def main(args=None):
    rclpy.init(args=args)

    patrol_node = PatrolNode() 
    state_machine = RobotStateMachine()
    state_machine.patrol_node = patrol_node # Pass the patrol node to the state machine

    # Debug: Test if the service is available by listing all services
    time.sleep(2)  # Allow some time for all nodes to initialize

    # Create a multi-threaded executor (allows multiple nodes to run in parallel)
    executor = MultiThreadedExecutor()
    executor.add_node(patrol_node)
    executor.add_node(state_machine)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('Main program interrupted!')
    finally:
        patrol_node.destroy_node()
        state_machine.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()