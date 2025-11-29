#import sys
#import os


#ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


#ROBO_LIB = os.path.join(ROOT, "robolab_lecture_materials", "src", "lib")

# Add it to sys.path
#if ROBO_LIB not in sys.path:
 #   sys.path.insert(0, ROBO_LIB)

#pytest_plugins = [
#    "check_publisher_subscriber_tests.conftest"
#]
import sys
import types

# -------------------------
# Mock rclpy for GitLab CI
# -------------------------
mock_rclpy = types.ModuleType("rclpy")
mock_rclpy.init = lambda *args, **kwargs: None
mock_rclpy.shutdown = lambda *args, **kwargs: None

mock_node = types.ModuleType("rclpy.node")

class MockNode:
    def __init__(self, name):
        pass

    def create_publisher(self, *args, **kwargs):
        return lambda msg: None

    def create_subscription(self, *args, **kwargs):
        return None

    def create_timer(self, *args, **kwargs):
        return None

    def get_logger(self):
        class L:
            def info(self, *args, **kwargs):
                pass
        return L()

mock_node.Node = MockNode
mock_rclpy.node = mock_node

sys.modules["rclpy"] = mock_rclpy
sys.modules["rclpy.node"] = mock_node

# Load RWU pub/sub test plugin
def pytest_configure(config):
    config.pluginmanager.import_plugin("lib.check_publisher_subscriber_tests.conftest")
