#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from tkinter import Tk, Button
import threading

class UIController(Node):
    def __init__(self):
        super().__init__('ui_controller')
        self.publisher_ = self.create_publisher(Int32, 'ui_command', 10)
        self.get_logger().info('UI Controller Node Initialized')

        # Tkinter GUI 설정
        self.root = Tk()
        self.root.title("UI Controller")

        # 창 크기 설정
        self.root.configure(bg="white")
        self.root.geometry("950x400")  
        self.root.wm_attributes("-topmost", 1)  

        # 버튼 생성 및 위치 지정
        kill_button = Button(self.root, text="Emergency\n\n!!Kill Switch!!", command=lambda: self.publish_message(1,"kill"), font=("Arial", 14, "bold"), bg="red", fg="blue")
        kill_button.place(x=750, y=150, width=150, height=200)

        offboard_button = Button(self.root, text="Offboard", command=lambda: self.publish_message(2, "offboard"), font=("Arial", 14))
        offboard_button.place(x=25, y=25, width=150, height=50)

        arming_button = Button(self.root, text="Arming", command=lambda: self.publish_message(3, "arming"), font=("Arial", 14))
        arming_button.place(x=200, y=25, width=150, height=50)

        takeoff_button = Button(self.root, text="Takeoff", command=lambda: self.publish_message(4, "takeoff"), font=("Arial", 14))
        takeoff_button.place(x=375, y=25, width=150, height=50)

        forward_button = Button(self.root, text="Forward", command=lambda: self.publish_message(11, "forward"), font=("Arial", 13), bg="lightyellow")
        forward_button.place(x=130, y=140, width=100, height=50)

        backward_button = Button(self.root, text="Backward", command=lambda: self.publish_message(12, "backward"), font=("Arial", 13), bg="lightyellow")
        backward_button.place(x=130, y=310, width=100, height=50)

        left_button = Button(self.root, text="Left", command=lambda: self.publish_message(13, "Left"), font=("Arial", 13), bg="lightyellow")
        left_button.place(x=70, y=200, width=50, height=100)

        right_button = Button(self.root, text="Right", command=lambda: self.publish_message(14, "Right"), font=("Arial", 13), bg="lightyellow")
        right_button.place(x=240, y=200, width=50, height=100)

        up_button = Button(self.root, text="Up", command=lambda: self.publish_message(15, "Up"), font=("Arial", 13), bg="cyan")
        up_button.place(x=130, y=200, width=100, height=45)

        down_button = Button(self.root, text="Down", command=lambda: self.publish_message(16, "Down"), font=("Arial", 13), bg="cyan")
        down_button.place(x=130, y=255, width=100, height=45)

        cw_button = Button(self.root, text="CW", command=lambda: self.publish_message(17, "CW"), font=("Arial", 11), bg="lightblue")
        cw_button.place(x=240, y=140, width=50, height=50)

        ccw_button = Button(self.root, text="CCW", command=lambda: self.publish_message(18, "CCW"), font=("Arial", 11), bg="lightblue")
        ccw_button.place(x=70, y=140, width=50, height=50)

        roll_plus_button = Button(self.root, text="Roll +", command=lambda: self.publish_message(19, "roll_plus"), font=("Arial", 12), bg="lightpink")
        roll_plus_button.place(x=600, y=200, width=100, height=100)

        roll_minus_button = Button(self.root, text="Roll -", command=lambda: self.publish_message(20, "roll_minus"), font=("Arial", 12), bg="lightpink")
        roll_minus_button.place(x=400, y=200, width=100, height=100)

        pitch_plus_button = Button(self.root, text="Pitch +", command=lambda: self.publish_message(21, "pitch_plus"), font=("Arial", 12), bg="lightgreen")
        pitch_plus_button.place(x=500, y=140, width=100, height=50)

        pitch_minus_button = Button(self.root, text="Pitch -", command=lambda: self.publish_message(22, "pitch_minus"), font=("Arial", 12), bg="lightgreen")
        pitch_minus_button.place(x=500, y=310, width=100, height=50)

        go_to_origin_button = Button(self.root, text="Go to Origin", command=lambda: self.publish_message(23, "go_to_origin"), font=("Arial", 14))
        go_to_origin_button.place(x=550, y=25, width=150, height=50)

        reference_tracking_button = Button(self.root, text="Reference Tracking", command=lambda: self.publish_message(24, "reference_tracking"), font=("Arial", 14))
        reference_tracking_button.place(x=720, y=25, width=200, height=50)

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
        node.run()  # Tkinter GUI 실행
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt (Ctrl+C) detected.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()