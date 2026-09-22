"""
Launch file cho bai tap "UR3/UR3e viet chu cai dau ten".

Luong:
    ur3_letter_writer.launch.py
        -> include ur_robot_driver/launch/ur_control.launch.py
               (khoi dong fake-hardware + ros2_control controller_manager +
                cac controller, KHONG mo RViz o day)
        -> sau vai giay, mo RViz voi cau hinh rieng cua goi (config/letter_view.rviz)
        -> sau vai giay (cho controller_manager + controller san sang),
           include ur_moveit_config/launch/ur_moveit.launch.py
               (khoi dong move_group (MoveIt2))
        -> sau vai giay nua (cho move_group san sang), spawn letter_writer_node
               -> sinh waypoint Cartesian cua chu cai
               -> MoveIt Planning/IK (computeCartesianPath)
               -> Joint Trajectory -> thuc thi tren UR3/UR3e
               -> dong thoi VE VET QUY DAO THUC TE cua end-effector (doc TF)

Ghi chu quan trong: tren ban ur_moveit_config di kem ROS 2 Humble, launch file
ur_moveit.launch.py CHI khoi dong MoveIt (move_group + RViz), KHONG tu khoi
dong fake hardware/controller_manager (khong co tham so use_fake_hardware/
robot_ip). Phan fake hardware + controllers phai duoc khoi dong rieng bang
ur_robot_driver/ur_control.launch.py, neu khong move_group se khong co
controller nao de thuc thi trajectory (execute() se treo/that bai).

Vi du chay:
    ros2 launch ur3_letter_writer ur3_letter_writer.launch.py \
        ur_type:=ur3e letter:=S use_fake_hardware:=true

    (rieng chu S co san launch goi tat: ur3_write_s.launch.py)

Quan sat quy dao trong RViz:
    RViz duoc mo voi config rieng cua goi nay (config/letter_view.rviz) thay
    vi config mac dinh cua ur_moveit_config, gom san:
    - RobotModel + TF + Grid, camera da canh san vao mat phang ve.
    - Marker /letter_preview: hinh chu du kien (mau xanh duong, net mong).
    - Marker /letter_trace: VET QUY DAO THUC TE (mau cyan, net day), duoc noi
      dai dan theo vi tri that cua end-effector (doc qua TF) trong luc robot
      dang ve. Tat bang publish_ee_trace:=false.
    RViz nay KHONG co panel "MotionPlanning". Neu can panel do, dung
    launch_rviz:=false o day roi chay rieng:
        ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e

Neu launch bao loi thieu/thua argument (phien ban ur_robot_driver/
ur_moveit_config khac nhau tuy Humble/Iron/Jazzy), chay:
    ros2 launch ur_robot_driver ur_control.launch.py --show-args
    ros2 launch ur_moveit_config ur_moveit.launch.py --show-args
de xem danh sach argument chinh xac tren may ban va chinh lai launch_arguments
ben duoi cho phu hop.
"""

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
        # Kich thuoc/vi tri mac dinh duoc chon de MOI diem cua chu deu nam
        # trong tam voi (0.5 m) cua UR3/UR3e: mat phang cach goc 0.35 m, o
        # chu 0.14 x 0.18 m quanh z = 0.35 m -> diem xa nhat cach khop vai
        # khoang 0.46 m. Neu ke chu to hon/xa hon, computeCartesianPath se
        # tra ve fraction thap va node se bo qua net do.
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

    # QUAN TRONG: ur_control.launch.py (ur_robot_driver) va ur_moveit.launch.py
    # (ur_moveit_config) deu khai bao mot tham so TRUNG TEN 'launch_rviz'. Vi
    # 'LaunchConfiguration' la 1 khong gian ten dung chung, include
    # ur_control_launch ben duoi (voi launch_rviz='false', chay ngay khong
    # delay) se GHI DE gia tri 'launch_rviz' trong context truoc khi cac action
    # chay tre (trong TimerAction) kip doc gia tri nguoi dung truyen vao. Phai
    # luu gia tri goc vao 1 ten rieng ('rviz_enabled') NGAY TU DAU.
    capture_rviz_choice = SetLaunchConfiguration('rviz_enabled', launch_rviz)

    # Neu terminal chay lenh nay duoc mo tu VSCode (Snap), cac bien moi truong
    # cua Snap (SNAP_LIBRARY_PATH, GTK_PATH, ...) se bi ke thua boi rviz2 va
    # khien no crash ngay luc khoi dong voi loi:
    #   "symbol lookup error: /snap/core20/.../libpthread.so.0: undefined
    #    symbol __libc_pthread_init, version GLIBC_PRIVATE"
    # Xoa cac bien nay khoi moi truong cua toan bo launch truoc khi rviz2
    # duoc spawn de tranh loi tren.
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
    # RViz cua ur_moveit.launch.py bi tat (launch_rviz='false'): goi nay tu mo
    # RViz rieng ben duoi voi config/letter_view.rviz (camera canh san vao mat
    # phang ve + 2 Marker display preview/trace).
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
    # Chi can /robot_description (topic, tu robot_state_publisher trong
    # ur_control_launch) + TF de hien robot - khong phu thuoc move_group nen
    # co the mo som, khong can doi MoveIt.
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
