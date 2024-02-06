from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("rb10", package_name="rb_moveit_config")
        .robot_description(file_path="config/rb10.urdf.xacro")
        .robot_description_semantic(file_path="config/rb10.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"],
            default_planning_pipeline="ompl",
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
