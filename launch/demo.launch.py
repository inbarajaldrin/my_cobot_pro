from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="my_cobot_pro").to_moveit_configs()
    return generate_demo_launch(moveit_config)
