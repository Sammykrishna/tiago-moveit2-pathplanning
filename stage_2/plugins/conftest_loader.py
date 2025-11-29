import sys
import os


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


ROBO_LIB = os.path.join(ROOT, "robolab_lecture_materials", "src", "lib")

# Add it to sys.path
if ROBO_LIB not in sys.path:
    sys.path.insert(0, ROBO_LIB)

pytest_plugins = [
    "check_publisher_subscriber_tests.conftest"
]
