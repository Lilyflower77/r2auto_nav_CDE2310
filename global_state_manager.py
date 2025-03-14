from r2auto_nav import AutoNav
import threading
import time
import rclpy
from rclpy.node import Node



# defining global constants

# defining states 
inialized = False
finishedMapping = False
foundRamp = False
ballsLaunched = 0

class navigationNodes(Node):
    # Publisher
    def __init__(self):
        super().__init__('navigationNodes')

class navigationNodes(Node):
    def __init__(self):
        super().__init__('launcherNode')


def main_control_loop(node):


    while rclpy.ok():
        with node.lock: # to prevent race condition so control loop and callback don't access the same variable at the same time

            return 
        

    time.sleep(0.1) # run at 10hz to prevent the control loop from running too fast
    return 


def main(args=None):
    # initializing ROS2
    rclpy.init(args=args)
    # initializing the node
    navigationNodes = navigationNodes()

    # Start ROS commuinication in its own thread to avoid blocking
    ros_thread = threading.Thread(target=rclpy.spin, args=(navigationNodes,), daemon=True)
    ros_thread.start() # starts the thread

    # Start the main control loop
    control_thread = threading.Thread(target=main_control_loop, daemon=True)
    control_thread.start() # starts the thread

    try:
        ros_thread.join() # neither of these threads should ever finish
        control_thread.join()
    except KeyboardInterrupt:
        navigationNodes.get_logger().info('Keyboard interrupt detected. Shutting down...')
    finally:
        # cleanup
        navigationNodes.destroy_node()



    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
