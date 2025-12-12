import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3

from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

FINGER_TIPS = [8, 12, 16, 20]
FINGER_KNUCKLES = [5, 9, 13, 17]


def is_hand_open(hand_landmarks):
    open_fingers = 0
    for tip, knuckle in zip(FINGER_TIPS, FINGER_KNUCKLES):
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[knuckle].y:
            open_fingers += 1
    return open_fingers, open_fingers >= 3


def compute_wrist_angles(hand_landmarks):
    def lm(i):
        return np.array([
            hand_landmarks.landmark[i].x,
            hand_landmarks.landmark[i].y,
            hand_landmarks.landmark[i].z
        ])

    wrist = lm(0)
    index_mcp = lm(5)
    pinky_mcp = lm(17)

    v_index = index_mcp - wrist
    v_pinky = pinky_mcp - wrist

    # Normal to the hand plane
    hand_normal = np.cross(v_index, v_pinky)
    hand_normal /= np.linalg.norm(hand_normal)

    yaw = np.arctan2(v_index[1], v_index[0])
    pitch = np.arctan2(-v_index[2], np.linalg.norm(v_index[:2]))
    roll = np.arctan2(hand_normal[0], hand_normal[2])

    return yaw, pitch, roll


class HandStateNode(Node):
    def __init__(self):
        super().__init__('hand_state_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )

        self.hands = mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.pub_state = self.create_publisher(Bool, 'hand_open', 10)
        self.pub_angles = self.create_publisher(Vector3, 'hand_wrist_angles', 10)

        self.get_logger().info("Hand state detector started.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        state_text = "No Hand Detected"

        if results.multi_hand_landmarks:
            hand = results.multi_hand_landmarks[0]

            # Finger state
            num_open_fingers, is_open = is_hand_open(hand)
            state_text = "OPEN" if is_open else "CLOSED"
            self.pub_state.publish(Bool(data=is_open))

            # Wrist angles
            yaw, pitch, roll = compute_wrist_angles(hand)
            angle_msg = Vector3(x=yaw, y=pitch, z=roll)
            self.pub_angles.publish(angle_msg)

            mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)
        else:
            self.pub_state.publish(Bool(data=True))

        cv2.putText(frame, state_text, (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4,
                    (0, 255, 0) if state_text == "OPEN" else (0, 0, 255),
                    3)

        cv2.imshow("Hand State Detection (ROS2)", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HandStateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
