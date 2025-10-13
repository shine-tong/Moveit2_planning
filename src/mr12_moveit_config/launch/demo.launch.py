from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("mr12urdf20240605", package_name="mr12_moveit_config").sensors_3d(file_path="config/sensors_3d.yaml").to_moveit_configs()
    return generate_demo_launch(moveit_config)