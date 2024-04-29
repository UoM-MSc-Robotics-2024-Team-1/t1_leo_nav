import rclpy
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from .patroling_node import PatrolNode
from std_srvs.srv import SetBool
from .robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from t1_interfaces.srv import Command


##! TERMINAL COMMANDS: !##
# Run the state machine node with: ros2 run t1_nav robot_state_machine
# Stop and start patrolling by running (start = true, stop = false): ros2 service call /toggle_patrol std_srvs/SetBool "{data: true}"
# Change state in terminal by running: ros2 service call /change_state t1_interfaces/srv/Command "{command: 'PATROL'}"
# States for the state machine
class State:
    PATROL = 'PATROL'
    MOVE_TO_OBJECT = 'MOVE_TO_OBJECT'
    MANIPULATE_OBJECT = 'PICKUP_OBJECT'
    IDLE = 'IDLE'

# State machine to control the robot's behavior
class RobotStateMachine(LifecycleNode):
    def __init__(self):
        super().__init__('robot_state_machine')
        self.patrol_node = None
        self.nav = BasicNavigator() 

        # Initial state
        self.current_state = State.PATROL # Start in the patrol state
        #self.transition_timer = self.create_timer(15, self.transition_states_timer)  # Transition states every 5 seconds for testing
        #self.transition_timer = self.create_timer(15, self.transition_states_timer)  # Transition states every 5 seconds for testing
        self.patrol_service_client = self.create_client(SetBool, '/toggle_patrol') # Service client to send patrol commands (Start/Stop)
        self.state_service = self.create_service(Command, '/change_state', self.handle_change_state) # Service to change state (Need custom service, not working for me)


    # Send patrol command to the patrol node
    def send_patrol_command(self, start_patrolling):
        while not self.patrol_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = SetBool.Request()
        request.data = start_patrolling
        future = self.patrol_service_client.call_async(request)
        future.add_done_callback(self.handle_service_response)
        self.get_logger().info(f"Sent patrol command (state machine): {'start' if start_patrolling else 'stop'}")

    # Handle service response (success or failure)
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

    # Change state to the next one for testing purposes
    def transition_states_timer(self):
        # Define a sequence of states to transition through for testing
        state_sequence = [State.PATROL, State.MOVE_TO_OBJECT]
        current_index = state_sequence.index(self.current_state)
        next_index = (current_index + 1) % len(state_sequence)
        new_state = state_sequence[next_index]
        self.transition_state(new_state)

    #Handle logic when changing states
    def transition_state(self, new_state):
        if new_state == State.PATROL:
            self.send_patrol_command(True) # Start patrolling
            
        elif new_state == State.MOVE_TO_OBJECT:
            self.send_patrol_command(False) # Stop patrolling

        self.current_state = new_state
        self.get_logger().info('Transitioning to state: ' + new_state)

    # Add logic to move to object here
    def move_to_object(self, msg: PoseStamped):
        self.get_logger().info('Moving to object...')
        self.transition_state(State.MOVE_TO_OBJECT)
        self.nav.GoToPose(msg) # Move to the object position

    # Add logic to pick up object here (e.g. call a service or action server or script)
    # It needs to let the state machine know when the object has been picked up
    def pickup_object(self):
        self.get_logger().info('Picking up object...')

    def listen_for_object_position(self):
        self.get_logger().info('Listening for camera position...')
        # Create a subscription to the camera position topic
        self.subscription = self.create_subscription(
            PoseStamped,  # Assuming Position is the message type
            '/camera/position',  # Assuming this is the topic name
            self.move_to_object, # Callback function
            10  # Queue size
        )


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