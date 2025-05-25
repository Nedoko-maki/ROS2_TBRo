import rclpy
import rclpy.executors
import rclpy.logging
from rclpy.node import Node

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from sensor_msgs.msg import BatteryState
from cosmo_msgs.msg import SystemCommand

import cosmo.rpio as rpio

from cosmo.rpio import (hex_to_dec, 
                        detect_i2c,
                        write_register, 
                        read_register, 
                        write_json,
                        read_json, 
                        )
import threading 


from cosmo.battery_node import BatteryNode


class TestNode(Node):

    # bridge = CvBridge()

    def __init__(self, node_config=[]):
        super().__init__("test_node_framework")

        self.executor = rclpy.executors.MultiThreadedExecutor()

        if not node_config:
            node_config = ["battery_node", "control_node", "flask_node", "motor_driver_node", "model_node"]

        for node in node_config:
            match node:
                case "battery_node":
                    from cosmo.battery_node import BatteryNode
                    _n = BatteryNode
                case "control_node":
                    from cosmo.control_node import ControlNode
                    _n = ControlNode
                case "flask_node":
                    from cosmo.flask_node import FlaskNode
                    _n = FlaskNode
                case "motor_driver_node":
                    from cosmo.motor_driver_node import MotorDriverNode
                    _n = MotorDriverNode
                case "model_node":
                    from cosmo.model_node import ModelNode
                    _n = ModelNode
            
            self.nodes[node] = _n()
            self.executor.add_node(self.nodes[node])
        
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        self.test_node("battery_node", ["_conv"])


    def get_active_nodes(self):
        for node in self.nodes:
            pass    


    def test_node(self, node, mut_list=[]):  # mut = method under test (MUT)

        if not node in self.nodes and not self.wait_for_node(node, timeout=5):
            self.get_logger().warn(f"{node} not included in config/not initialised.")
            self.get_logger().warn(f"Active nodes: {self.get_node_names()}")
            return

        for method in mut_list:
            
            if isinstance(method, str):
                mut = getattr(self.nodes[node], method)
            else:
                mut = method

                
            *inputs, outputs = self._read_test_data(node, mut.__name__)  # sets all but the last value as inputs, 
            # and the last comma seperated value as the output


            for i, o in zip(inputs, outputs):
                if isinstance(i, list):
                    assert mut(*i) == o, f"input(s): {inputs}, do not equal output: {outputs}"
                else:
                    assert mut(i) == o, f"input(s): {inputs}, do not equal output: {outputs}"
            
            self.get_logger().info(f"NODE: {node}.{mut.__name__} test passed.")


    def _read_test_data(node_name, method_name, filepath="./test_data/"):
        
        with open(rpio.relpath(filepath+f"{node_name}.{method_name}.txt")) as fs:
            test_data = fs.readlines()
            
        test_data = [line.rstrip("\n").split(",") for line in test_data]

        input_types = test_data.pop(0)  # pop the first index
        _tmp_types = []

        for _type in input_types:
            _type = _type.strip() # strip any leading whitespace
            match _type:
                case "int":
                    _tmp = int
                case "str":
                    _tmp = str
                case "float": 
                    _tmp = float
                case _:
                    raise NotImplementedError(f"Type {_type} does not exist. File {rpio.relpath(filepath+f'{node_name}.{method_name}.txt')})")
            _tmp_types.append(_tmp)

        test_data = [[_type(x) for _type in _tmp_types] for x in test_data] # for each line list in the test_data, set their value to the typecast value.

        # The test data is in a text-readable file with inputs and outputs of the mut separated by a comma.
        # the first line defines the input types.
        # e.g. filename: battery_node._conv.txt

        # contents:
        # int, str
        # 32, Voltage, 0.0025
        # 64, Voltage, 0.005



    def __del__(self):
        self.executor.shutdown(5)
        self.executor_thread.join()


def static_functions():
    pass





def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    _sleep_node = rclpy.create_node("sleep_node")
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(test_node)
    executor.add_node(_sleep_node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:  # blocking operation to make sure the script doesn't leave early and end rclpy.
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        test_node.get_logger().warn(f"KeyboardInterrupt triggered.")
    finally:
        _sleep_node.destroy_node()
        test_node.destroy_node()
        rclpy.try_shutdown()  
        executor_thread.join()