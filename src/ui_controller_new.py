#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from tkinter import Tk, Button, Frame
import threading


class UIController(Node):
    def __init__(self):
        super().__init__('ui_controller')
        self.publisher_ = self.create_publisher(Int32, 'ui_command', 10)
        self.get_logger().info('UI Controller Node Initialized')

        # Tkinter GUI 설정
        self.root = Tk()
        self.root.title("UI Controller")
        self.root.configure(bg="white")

        # "초기" 크기만 주고, 반응형으로 늘어나게 둠
        self.root.geometry("950x400")
        self.root.minsize(850, 350)  # 너무 작아져서 깨지는 걸 방지
        self.root.wm_attributes("-topmost", 1)

        # ===== 최상위 그리드(3열) =====
        # [Left Panel | Center Panel | Right Panel]
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=3)  # left
        self.root.grid_columnconfigure(1, weight=2)  # center
        self.root.grid_columnconfigure(2, weight=2)  # right

        left = Frame(self.root, bg="white", padx=10, pady=10)
        center = Frame(self.root, bg="white", padx=10, pady=10)
        right = Frame(self.root, bg="white", padx=10, pady=10)

        left.grid(row=0, column=0, sticky="nsew")
        center.grid(row=0, column=1, sticky="nsew")
        right.grid(row=0, column=2, sticky="nsew")

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

        # --- bot_left: Set Home Position ---
        bot_left.grid_columnconfigure(0, weight=1)
        go_to_origin_button = Button(
            bot_left, text="Set Home Position",
            command=lambda: self.publish_message(23, "set_home_position"),
            font=("Arial", 14)
        )
        go_to_origin_button.grid(row=0, column=0, sticky="nsew", padx=5)

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
        top_right.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        top_right.grid_columnconfigure(0, weight=1)

        reference_tracking_button = Button(
            top_right, text="Reference Tracking",
            command=lambda: self.publish_message(24, "reference_tracking"),
            font=("Arial", 14)
        )
        reference_tracking_button.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        kill_button = Button(
            right,
            text="Emergency\n\n!! Kill Switch !!",
            command=lambda: self.publish_message(1, "kill"),
            font=("Arial", 14, "bold"),
            bg="red",
            fg="cyan"
        )
        kill_button.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

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
