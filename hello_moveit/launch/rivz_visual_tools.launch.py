import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # hello_moveit パッケージの share ディレクトリパスを取得
    package_share_directory = get_package_share_directory('hello_moveit')
    rviz_config_path = PathJoinSubstitution([package_share_directory, 'config', 'rviz_visual_tools.rviz'])

    return LaunchDescription([
        # RViz設定ファイルを指定する引数を宣言（デフォルトはhello_moveitパッケージのmoveit_visual_tools.rviz）
        DeclareLaunchArgument(
            'rviz_config_file', 
            default_value=rviz_config_path,
            description='Path to the RViz configuration file'
        ),
        
        # RVizを起動するノード
        Node(
            package='rviz2',              # RVizのパッケージ
            executable='rviz2',           # 実行可能ファイル
            name='rviz',                  # ノード名
            output='screen',              # 出力をターミナルに表示
            arguments=['-d', LaunchConfiguration('rviz_config_file')],  # 設定ファイルを指定
        ),
    ])
