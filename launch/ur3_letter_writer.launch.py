from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    launch_rviz = LaunchConfiguration('launch_rviz')
    letter = LaunchConfiguration('letter')

    declared_args = [
        DeclareLaunchArgument('ur_type', default_value='ur3e',
                               description='Loai robot: ur3, ur3e, ur5e, ...'),
        DeclareLaunchArgument('robot_ip', default_value='0.0.0.0',
                               description='IP robot that (khong dung khi use_fake_hardware=true)'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true',
                               description='true = mo phong (fake hardware), '
                                           'false = robot that (can robot_ip that)'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('letter', default_value='S',
                               description='Ten sinh vien hoac 1 ky tu; chu cai dau tien se '
                                           'duoc ve, vd: S -> S, Bao -> B, Minh -> M'),

        DeclareLaunchArgument('plane_x', default_value='0.35',
                               description='Khoang cach (m) tu goc robot toi mat phang ve, theo truc X'),
        DeclareLaunchArgument('center_y', default_value='0.0',
                               description='Tam chu theo truc Y (m)'),
        DeclareLaunchArgument('center_z', default_value='0.35',
                               description='Tam chu theo truc Z (m)'),
        DeclareLaunchArgument('letter_width', default_value='0.14',
                               description='Be rong chu (m)'),
        DeclareLaunchArgument('letter_height', default_value='0.18',
                               description='Chieu cao chu (m)'),
        DeclareLaunchArgument('pen_lift', default_value='0.05',
                               description='Do nhac dau cong tac khoi mat phang khi chuyen net (m)'),
        DeclareLaunchArgument('velocity_scaling', default_value='0.3'),
        DeclareLaunchArgument('acceleration_scaling', default_value='0.3'),
        # --- Vet quy dao THUC TE cua end-effector ---
        DeclareLaunchArgument('publish_ee_trace', default_value='true',
                               description='true = ve vet quy dao thuc te cua end-effector '
                                           '(doc qua TF) len topic ee_trace_topic trong luc robot chay'),
        DeclareLaunchArgument('ee_trace_topic', default_value='letter_trace',
                               description='Topic Marker (LINE_STRIP) hien vet quy dao thuc te'),
        DeclareLaunchArgument('ee_trace_rate', default_value='30.0',
                               description='Tan so (Hz) lay mau vi tri end-effector qua TF'),
        DeclareLaunchArgument('ee_trace_line_width', default_value='0.006',
                               description='Do day (m) cua duong ve vet quy dao thuc te'),
        DeclareLaunchArgument('trace_pen_down_only', default_value='true',
                               description='true = chi ve vet khi dau cong tac dang cham mat '
                                           'phang (ra dung hinh chu); false = ve ca doan nhac '
                                           'but di chuyen giua cac net (toan bo hanh trinh that)'),
        DeclareLaunchArgument('ee_trace_frame', default_value='',
                               description='TF frame cua dau cong tac; de trong = lay '
                                           'end-effector link cua planning group (vd tool0)'),
        # Cho robot_state_publisher (tu ur_control_launch) publish
        # /robot_description truoc khi mo RViz (RobotModel display doc qua topic nay)
        DeclareLaunchArgument('rviz_startup_delay', default_value='3.0'),
        # Cho controller_manager/fake hardware co du thoi gian khoi dong va
        # active controller truoc khi mo MoveIt (move_group)
        DeclareLaunchArgument('moveit_startup_delay', default_value='6.0'),
        # Cho move_group co du thoi gian khoi dong hoan toan truoc khi
        # letter_writer_node ket noi MoveGroupInterface (tinh tu luc launch
        # bat dau, khong phai tinh tu sau moveit_startup_delay)
        DeclareLaunchArgument('startup_delay', default_value='14.0'),
    ]

    capture_rviz_choice = SetLaunchConfiguration('rviz_enabled', launch_rviz)

    clear_snap_env = [
        UnsetEnvironmentVariable('SNAP_LIBRARY_PATH'),
        UnsetEnvironmentVariable('LOCPATH'),
        UnsetEnvironmentVariable('GTK_PATH'),
        UnsetEnvironmentVariable('GTK_EXE_PREFIX'),
        UnsetEnvironmentVariable('GDK_PIXBUF_MODULE_FILE'),
        UnsetEnvironmentVariable('GDK_PIXBUF_MODULEDIR'),
        UnsetEnvironmentVariable('GIO_MODULE_DIR'),
        UnsetEnvironmentVariable('GTK_IM_MODULE_FILE'),
        UnsetEnvironmentVariable('XDG_DATA_DIRS'),
        UnsetEnvironmentVariable('XDG_DATA_HOME'),
    ]

    # --- Fake hardware + ros2_control controller_manager + controllers ---
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py',
            ])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': 'false',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
        }.items(),
    )

    # --- MoveIt2 (move_group), dung lai launch co san cua UR ---
    ur_moveit_launch = TimerAction(
        period=LaunchConfiguration('moveit_startup_delay'),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('ur_moveit_config'),
                        'launch',
                        'ur_moveit.launch.py',
                    ])
                ),
                launch_arguments={
                    'ur_type': ur_type,
                    'launch_rviz': 'false',
                }.items(),
            )
        ],
    )

    # --- RViz voi cau hinh rieng cua goi nay ---
    letter_rviz_launch = TimerAction(
        period=LaunchConfiguration('rviz_startup_delay'),
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_letter_view',
                output='log',
                condition=IfCondition(LaunchConfiguration('rviz_enabled')),
                arguments=[
                    '-d', PathJoinSubstitution([
                        FindPackageShare('ur3_letter_writer'),
                        'config',
                        'letter_view.rviz',
                    ])
                ],
            )
        ],
    )

    # --- Node dieu khien cua sinh vien, spawn tre hon de doi MoveIt san sang ---
    letter_writer_node = TimerAction(
        period=LaunchConfiguration('startup_delay'),
        actions=[
            Node(
                package='ur3_letter_writer',
                executable='letter_writer_node',
                name='letter_writer_node',
                output='screen',
                parameters=[{
                    'letter_name': letter,
                    'planning_group': 'ur_manipulator',
                    'plane_x': LaunchConfiguration('plane_x'),
                    'center_y': LaunchConfiguration('center_y'),
                    'center_z': LaunchConfiguration('center_z'),
                    'letter_width': LaunchConfiguration('letter_width'),
                    'letter_height': LaunchConfiguration('letter_height'),
                    'pen_lift': LaunchConfiguration('pen_lift'),
                    'velocity_scaling': LaunchConfiguration('velocity_scaling'),
                    'acceleration_scaling': LaunchConfiguration('acceleration_scaling'),
                    'publish_ee_trace': LaunchConfiguration('publish_ee_trace'),
                    'ee_trace_topic': LaunchConfiguration('ee_trace_topic'),
                    'ee_trace_rate': LaunchConfiguration('ee_trace_rate'),
                    'ee_trace_line_width': LaunchConfiguration('ee_trace_line_width'),
                    'trace_pen_down_only': LaunchConfiguration('trace_pen_down_only'),
                    'ee_trace_frame': LaunchConfiguration('ee_trace_frame'),
                }],
            )
        ],
    )

    return LaunchDescription(declared_args + [
        capture_rviz_choice,
        *clear_snap_env,
        ur_control_launch,
        letter_rviz_launch,
        ur_moveit_launch,
        letter_writer_node,
    ])
