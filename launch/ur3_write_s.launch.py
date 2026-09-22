"""
Launch file GOI TAT: "UR3/UR3e viet chu S" + ve quy dao THUC TE cua dau cong tac.

Day chi la lop bao mong quanh ur3_letter_writer.launch.py: co dinh letter:='S'
va dat san cac tham so kich thuoc/toc do hop voi chu S, con toan bo phan khoi
dong (fake hardware + ros2_control, MoveIt move_group, RViz) van dung lai
launch tong de tranh trung lap.

Chay mo phong (mac dinh):
    ros2 launch ur3_letter_writer ur3_write_s.launch.py

Chay voi robot that:
    ros2 launch ur3_letter_writer ur3_write_s.launch.py \
        use_fake_hardware:=false robot_ip:=192.168.1.102

Nhung gi se thay trong RViz (config/letter_view.rviz mo san 2 Marker display):
  - /letter_preview : hinh chu S du kien (xanh duong, net mong) - ve 1 lan
                      ngay khi node khoi dong.
  - /letter_trace   : QUY DAO THUC TE (cyan, net day) - node doc TF
                      world -> tool0 o 30 Hz trong luc robot dang chay va noi
                      dai dan duong nay, nen ban thay net chu duoc "viet" theo
                      dung chuyen dong that cua end-effector.

Tham so hay dung:
    ur_type:=ur3            # mac dinh ur3e
    letter_width:=0.12 letter_height:=0.16    # thu nho neu bao qua tam voi
    plane_x:=0.32 center_z:=0.32              # keo bang ve lai gan robot
    velocity_scaling:=0.15                    # ve cham lai cho de quan sat
    trace_pen_down_only:=false                # ve ca doan nhac but di chuyen
    publish_ee_trace:=false                   # tat hoan toan vet quy dao
    launch_rviz:=false                        # khong mo RViz

Chu S duoc dinh nghia trong src/letters.cpp (ham makeS): 2 cung elip tiep xuc
nhau tai tam o chu, 35 diem -> 1 net lien mach, khong nhac but giua chung.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Cac tham so duoc chuyen thang xuong ur3_letter_writer.launch.py.
# (ten tham so, gia tri mac dinh cho chu S, mo ta)
_FORWARDED_ARGS = [
    ('ur_type', 'ur3e', 'Loai robot: ur3, ur3e, ur5e, ...'),
    ('robot_ip', '0.0.0.0', 'IP robot that (bo qua khi use_fake_hardware=true)'),
    ('use_fake_hardware', 'true', 'true = mo phong, false = robot that'),
    ('launch_rviz', 'true', 'Mo RViz voi config/letter_view.rviz'),
    # Chu S nhieu doan cong: de nho hon chu thang mot chut cho chac chan
    # moi diem deu nam trong tam voi va IK giu duoc huong dau cong tac.
    ('plane_x', '0.35', 'Khoang cach (m) tu goc robot toi mat phang ve (truc X)'),
    ('center_y', '0.0', 'Tam chu theo truc Y (m)'),
    ('center_z', '0.35', 'Tam chu theo truc Z (m)'),
    ('letter_width', '0.14', 'Be rong chu S (m)'),
    ('letter_height', '0.18', 'Chieu cao chu S (m)'),
    ('pen_lift', '0.05', 'Do nhac dau cong tac khoi mat phang (m)'),
    # Cham hon mac dinh: chu S la duong cong lien tuc, chay cham giup vet
    # quy dao duoc lay mau day hon va de quan sat hon.
    ('velocity_scaling', '0.2', 'He so ty le van toc (0..1)'),
    ('acceleration_scaling', '0.2', 'He so ty le gia toc (0..1)'),
    ('publish_ee_trace', 'true', 'Ve vet quy dao thuc te cua end-effector (doc qua TF)'),
    ('ee_trace_topic', 'letter_trace', 'Topic Marker hien vet quy dao thuc te'),
    ('ee_trace_rate', '30.0', 'Tan so (Hz) lay mau vi tri end-effector'),
    ('ee_trace_line_width', '0.006', 'Do day (m) duong ve vet quy dao'),
    ('trace_pen_down_only', 'true',
     'true = chi ve vet khi but cham mat phang; false = ve ca hanh trinh nhac but'),
    ('ee_trace_frame', '', 'TF frame dau cong tac; de trong = end-effector link cua group'),
    ('rviz_startup_delay', '3.0', 'Tre (s) truoc khi mo RViz'),
    ('moveit_startup_delay', '6.0', 'Tre (s) truoc khi khoi dong move_group'),
    ('startup_delay', '14.0', 'Tre (s) truoc khi spawn letter_writer_node'),
]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in _FORWARDED_ARGS
    ]

    forwarded = {name: LaunchConfiguration(name) for name, _, _ in _FORWARDED_ARGS}
    # Diem khac biet duy nhat so voi launch tong: chu can ve luon la 'S'.
    forwarded['letter'] = 'S'

    write_s = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur3_letter_writer'),
                'launch',
                'ur3_letter_writer.launch.py',
            ])
        ),
        launch_arguments=forwarded.items(),
    )

    return LaunchDescription(declared_args + [write_s])
