#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, Float32MultiArray
from px4_msgs.msg import VehicleOdometry
from tkinter import Tk, Button, Frame, Label
import threading
import math


class UIController(Node):
    def __init__(self):
        super().__init__('ui_controller')
        self.publisher_ = self.create_publisher(Int32, 'ui_command', 10)
        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.vehicle_odometry_sub_ = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback,
            sensor_qos
        )
        self.reference_trajectory_sub_ = self.create_subscription(
            Float32MultiArray,
            '/reference_trajectory',
            self.reference_trajectory_callback,
            sensor_qos
        )
        self.get_logger().info('UI Controller Node Initialized')

        # Tkinter GUI 설정
        self.root = Tk()
        self.root.title("UI Controller")
        self.root.configure(bg="white")

        # "초기" 크기만 주고, 반응형으로 늘어나게 둠
        self.root.geometry("950x400")
        self.root.minsize(850, 350)  # 너무 작아져서 깨지는 걸 방지
        self.root.wm_attributes("-topmost", 1)

        # ===== 최상위 그리드(2행) =====
        # [Top Pose Display]
        # [Left Panel | Center Panel | Right Panel]
        self.root.grid_rowconfigure(0, weight=0)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=3)  # left
        self.root.grid_columnconfigure(1, weight=2)  # center
        self.root.grid_columnconfigure(2, weight=2)  # right

        # =========================
        # Top Pose Display
        # =========================
        pose_frame = Frame(self.root, bg="white", padx=10, pady=8)
        pose_frame.grid(row=0, column=0, columnspan=3, sticky="ew")
        pose_frame.grid_columnconfigure(0, weight=1)
        pose_frame.grid_columnconfigure(1, weight=1)

        pos_frame = Frame(pose_frame, bg="white")
        rpy_frame = Frame(pose_frame, bg="white")
        pos_frame.grid(row=0, column=0, sticky="ew", padx=(0, 10))
        rpy_frame.grid(row=0, column=1, sticky="ew")

        self.pos_labels = {
            "x": Label(pos_frame, text="x : 0.00", font=("Arial", 12), bg="white"),
            "y": Label(pos_frame, text="y : 0.00", font=("Arial", 12), bg="white"),
            "z": Label(pos_frame, text="z : 0.00", font=("Arial", 12), bg="white"),
        }
        self.rpy_labels = {
            "roll": Label(rpy_frame, text="roll : 0.00 deg", font=("Arial", 12), bg="white"),
            "pitch": Label(rpy_frame, text="pitch : 0.00 deg", font=("Arial", 12), bg="white"),
            "yaw": Label(rpy_frame, text="yaw : 0.00 deg", font=("Arial", 12), bg="white"),
        }

        for i, key in enumerate(["x", "y", "z"]):
            self.pos_labels[key].grid(row=i, column=0, sticky="w")
        for i, key in enumerate(["roll", "pitch", "yaw"]):
            self.rpy_labels[key].grid(row=i, column=0, sticky="w")

        left = Frame(self.root, bg="white", padx=10, pady=10)
        center = Frame(self.root, bg="white", padx=10, pady=10)
        right = Frame(self.root, bg="white", padx=10, pady=10)

        left.grid(row=1, column=0, sticky="nsew")
        center.grid(row=1, column=1, sticky="nsew")
        right.grid(row=1, column=2, sticky="nsew")

        # =========================
        # Left Panel: 상단 모드버튼 + 이동패드 + 원점
        # =========================
        # left 내부 레이아웃: 위(상단 버튼줄), 중(이동패드), 아래(원점)
        left.grid_rowconfigure(0, weight=0)
        left.grid_rowconfigure(1, weight=1)
        left.grid_rowconfigure(2, weight=0)
        left.grid_columnconfigure(0, weight=1)

        top_left = Frame(left, bg="white")
        mid_left = Frame(left, bg="white")
        bot_left = Frame(left, bg="white")

        top_left.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        mid_left.grid(row=1, column=0, sticky="nsew", pady=(0, 10))
        bot_left.grid(row=2, column=0, sticky="ew")

        # --- top_left: Offboard/Arming/Takeoff ---
        for c in range(3):
            top_left.grid_columnconfigure(c, weight=1)

        offboard_button = Button(
            top_left, text="Offboard",
            command=lambda: self.publish_message(2, "offboard"),
            font=("Arial", 14)
        )
        offboard_button.grid(row=0, column=0, sticky="nsew", padx=5)

        arming_button = Button(
            top_left, text="Arming",
            command=lambda: self.publish_message(3, "arming"),
            font=("Arial", 14)
        )
        arming_button.grid(row=0, column=1, sticky="nsew", padx=5)

        takeoff_button = Button(
            top_left, text="Takeoff",
            command=lambda: self.publish_message(4, "takeoff"),
            font=("Arial", 14)
        )
        takeoff_button.grid(row=0, column=2, sticky="nsew", padx=5)

        # --- mid_left: 이동/고도/요 패드 ---
        # 4x3 그리드:
        # row0: [CCW][  ][ CW]
        # row1: [Left][Up][Right]
        # row2: [   ][Down][   ]
        # row3: [   ][Backward][  ]  (Forward는 row1 col1 위쪽에 배치하는 느낌이지만, 여기선 명확히 따로 둠)
        #
        # 네 원래 배치 감각을 살리기 위해:
        # - Forward: row0~1 사이 느낌 대신 row0/1을 조합하기 애매해서 row0 col1에 둠 (시각적으로 중앙 상단)
        # - Backward: row3 col1
        # - Up/Down: row1/2 col1
        # - Left/Right: row1 col0/2
        # - CCW/CW: row2 col0/2 (혹은 row0 col0/2도 가능)
        for r in range(4):
            mid_left.grid_rowconfigure(r, weight=1)
        for c in range(3):
            mid_left.grid_columnconfigure(c, weight=1)

        forward_button = Button(
            mid_left, text="Forward",
            command=lambda: self.publish_message(11, "forward"),
            font=("Arial", 13),
            bg="lightyellow"
        )
        forward_button.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

        backward_button = Button(
            mid_left, text="Backward",
            command=lambda: self.publish_message(12, "backward"),
            font=("Arial", 13),
            bg="lightyellow"
        )
        backward_button.grid(row=3, column=1, sticky="nsew", padx=5, pady=5)

        left_button = Button(
            mid_left, text="Left",
            command=lambda: self.publish_message(13, "Left"),
            font=("Arial", 13),
            bg="lightyellow"
        )
        left_button.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        right_button = Button(
            mid_left, text="Right",
            command=lambda: self.publish_message(14, "Right"),
            font=("Arial", 13),
            bg="lightyellow"
        )
        right_button.grid(row=1, column=2, sticky="nsew", padx=5, pady=5)

        up_button = Button(
            mid_left, text="Up",
            command=lambda: self.publish_message(15, "Up"),
            font=("Arial", 13),
            bg="cyan"
        )
        up_button.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)

        down_button = Button(
            mid_left, text="Down",
            command=lambda: self.publish_message(16, "Down"),
            font=("Arial", 13),
            bg="cyan"
        )
        down_button.grid(row=2, column=1, sticky="nsew", padx=5, pady=5)

        ccw_button = Button(
            mid_left, text="CCW",
            command=lambda: self.publish_message(18, "CCW"),
            font=("Arial", 11),
            bg="lightblue"
        )
        ccw_button.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)

        cw_button = Button(
            mid_left, text="CW",
            command=lambda: self.publish_message(17, "CW"),
            font=("Arial", 11),
            bg="lightblue"
        )
        cw_button.grid(row=2, column=2, sticky="nsew", padx=5, pady=5)

        # --- bot_left: (빈 공간 유지) ---
        bot_left.grid_columnconfigure(0, weight=1)

        # =========================
        # Center Panel: Roll / Pitch
        # =========================
        # center 내부 레이아웃: 3행
        # row0: pitch+
        # row1: roll-  roll+
        # row2: pitch-
        for r in range(3):
            center.grid_rowconfigure(r, weight=1)
        center.grid_columnconfigure(0, weight=1)
        center.grid_columnconfigure(1, weight=1)

        pitch_plus_button = Button(
            center, text="Pitch +",
            command=lambda: self.publish_message(21, "pitch_plus"),
            font=("Arial", 12),
            bg="lightgreen"
        )
        pitch_plus_button.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)

        roll_minus_button = Button(
            center, text="Roll -",
            command=lambda: self.publish_message(20, "roll_minus"),
            font=("Arial", 12),
            bg="lightpink"
        )
        roll_minus_button.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        roll_plus_button = Button(
            center, text="Roll +",
            command=lambda: self.publish_message(19, "roll_plus"),
            font=("Arial", 12),
            bg="lightpink"
        )
        roll_plus_button.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)

        pitch_minus_button = Button(
            center, text="Pitch -",
            command=lambda: self.publish_message(22, "pitch_minus"),
            font=("Arial", 12),
            bg="lightgreen"
        )
        pitch_minus_button.grid(row=2, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)

        # =========================
        # Right Panel: Reference Tracking + Kill Switch
        # =========================
        right.grid_rowconfigure(0, weight=0)  # 상단 버튼들
        right.grid_rowconfigure(1, weight=1)  # 킬 스위치가 크게
        right.grid_columnconfigure(0, weight=1)

        top_right = Frame(right, bg="white")
        top_right.grid(row=0, column=0, sticky="nsew", pady=(0, 10))
        top_right.grid_columnconfigure(0, weight=1)
        for r in range(3):
            top_right.grid_rowconfigure(r, weight=1)

        stop_current_button = Button(
            top_right, text="Stop at current position",
            command=lambda: self.publish_message(23, "stop_at_current_position"),
            font=("Arial", 12)
        )
        stop_current_button.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        go_home_button = Button(
            top_right, text="Go to Home Position",
            command=lambda: self.publish_message(26, "go_to_home_position"),
            font=("Arial", 12)
        )
        go_home_button.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        self.reference_tracking_on = False
        self.reference_tracking_pending = False
        self.reference_tracking_button = Button(
            top_right, text="Reference Tracking (OFF)",
            command=self.toggle_reference_tracking,
            font=("Arial", 12),
            bg="lightgray",
            activebackground="gainsboro",
            activeforeground="black"
        )
        self.reference_tracking_button.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)

        kill_button = Button(
            right,
            text="Emergency\n\n!! Kill Switch !!",
            command=lambda: self.publish_message(1, "kill"),
            font=("Arial", 14, "bold"),
            bg="red",
            fg="cyan"
        )
        kill_button.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        self.latest_position = None
        self.latest_rpy_deg = None
        self.pose_received = False
        self.last_pose_time = None
        self.pose_timeout_sec = 1.0
        self.last_reference_time = None
        self.reference_timeout_sec = 0.1
        self.root.after(500, self.update_pose_display)

    def vehicle_odometry_callback(self, msg):
        self.pose_received = True
        self.last_pose_time = self.get_clock().now()
        self.latest_position = [
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2])
        ]

        w = float(msg.q[0])
        x = float(msg.q[1])
        y = float(msg.q[2])
        z = float(msg.q[3])

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.latest_rpy_deg = [
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw)
        ]

    def update_pose_display(self):
        if self.reference_tracking_on and self.last_reference_time is not None:
            age_ref = (self.get_clock().now() - self.last_reference_time).nanoseconds / 1e9
            if age_ref > self.reference_timeout_sec:
                self.reference_tracking_on = False
                self.reference_tracking_pending = False
                self.reference_tracking_button.config(
                    text="Reference Tracking (OFF)",
                    bg="lightgray",
                    activebackground="gainsboro",
                    activeforeground="black"
                )
                self.get_logger().info("reference_tracking_off (reference timeout)")

        if not self.pose_received:
            self.pos_labels["x"].config(text="x : Not Recieved")
            self.pos_labels["y"].config(text="y : Not Recieved")
            self.pos_labels["z"].config(text="z : Not Recieved")
            self.rpy_labels["roll"].config(text="roll : Not Recieved")
            self.rpy_labels["pitch"].config(text="pitch : Not Recieved")
            self.rpy_labels["yaw"].config(text="yaw : Not Recieved")
        elif self.last_pose_time is not None:
            age_sec = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
            if age_sec > self.pose_timeout_sec:
                self.pos_labels["x"].config(text="x : Not Recieved")
                self.pos_labels["y"].config(text="y : Not Recieved")
                self.pos_labels["z"].config(text="z : Not Recieved")
                self.rpy_labels["roll"].config(text="roll : Not Recieved")
                self.rpy_labels["pitch"].config(text="pitch : Not Recieved")
                self.rpy_labels["yaw"].config(text="yaw : Not Recieved")
            else:
                x, y, z = self.latest_position
                roll, pitch, yaw = self.latest_rpy_deg

                self.pos_labels["x"].config(text=f"x : {x:.2f}")
                self.pos_labels["y"].config(text=f"y : {y:.2f}")
                self.pos_labels["z"].config(text=f"z : {z:.2f}")

                self.rpy_labels["roll"].config(text=f"roll : {roll:.2f} deg")
                self.rpy_labels["pitch"].config(text=f"pitch : {pitch:.2f} deg")
                self.rpy_labels["yaw"].config(text=f"yaw : {yaw:.2f} deg")
        else:
            self.pos_labels["x"].config(text="x : Not Recieved")
            self.pos_labels["y"].config(text="y : Not Recieved")
            self.pos_labels["z"].config(text="z : Not Recieved")
            self.rpy_labels["roll"].config(text="roll : Not Recieved")
            self.rpy_labels["pitch"].config(text="pitch : Not Recieved")
            self.rpy_labels["yaw"].config(text="yaw : Not Recieved")

        self.root.after(500, self.update_pose_display)

    def reference_trajectory_callback(self, msg):
        self.last_reference_time = self.get_clock().now()
        if self.reference_tracking_pending and not self.reference_tracking_on:
            self.reference_tracking_pending = False
            self.reference_tracking_on = True
            self.reference_tracking_button.config(
                text="Reference Tracking (ON)",
                bg="lightgreen",
                activebackground="palegreen",
                activeforeground="black"
            )
            self.get_logger().info("reference_tracking_on (reference received)")

    def toggle_reference_tracking(self):
        if not self.reference_tracking_on and not self.reference_tracking_pending:
            self.reference_tracking_pending = True
            self.reference_tracking_button.config(
                text="Reference Tracking (HOLD)",
                bg="khaki",
                activebackground="khaki",
                activeforeground="black"
            )
            self.publish_message(24, "reference_tracking_on")
        else:
            self.reference_tracking_on = False
            self.reference_tracking_pending = False
            self.reference_tracking_button.config(
                text="Reference Tracking (OFF)",
                bg="lightgray",
                activebackground="gainsboro",
                activeforeground="black"
            )
            self.publish_message(25, "reference_tracking_off")

    def publish_message(self, data, data_name):
        msg = Int32()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info(f'{data_name} command sent')

    def run(self):
        self.root.mainloop()


def spin_node(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = UIController()

    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt (Ctrl+C) detected.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
