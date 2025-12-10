from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    image_stitching_pkg = get_package_share_directory('image_stitching_pkg')  # 按实际包名改
    marnav_vis_pkg = get_package_share_directory('marnav_vis')  # 如果另一个在 marnav_vis

    # 启动 JH_ASM_DS_launch.py
    asm_ds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([image_stitching_pkg, 'launch', 'JH_ASM_DS_launch.py'])
        )
    )

    # 启动 Assemble_JH_launch.py（可按需延时）
    assemble_jh = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([marnav_vis_pkg, 'launch', 'Assemble_JH_launch.py'])
        )
    )

    return LaunchDescription([
        asm_ds,
        TimerAction(period=1.0, actions=[assemble_jh]),  # 若不需要延时，直接写 assemble_jh
    ])