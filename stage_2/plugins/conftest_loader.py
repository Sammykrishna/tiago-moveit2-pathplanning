import sys
import os

# Compute path to robolab_lecture_materials/lib/check_publisher_subscriber_tests
BASE = os.path.dirname(__file__)             # stage_2/plugins/check_publisher_subscriber_tests
ROBO_LIB = os.path.abspath(
    os.path.join(BASE, "..", "..", "robolab_lecture_materials", "src", "lib")
)

# Add path to sys.path
if ROBO_LIB not in sys.path:
    sys.path.append(ROBO_LIB)

# Load the real plugin
pytest_plugins = [
    "check_publisher_subscriber_tests.conftest"
]
