import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from cv_bridge import CvBridge
import cv2
import mediapipe as mp

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


class HandStateNode(Node):
    def __init__(self):
        super().__init__('hand_state_detector')

        self.bridge = CvBridge()

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )

        # Initialize MediaPipe Hands
        self.hands = mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.pub = self.create_publisher(Bool, 'hand_open', 10)
        self.get_logger().info("Hand state detector node started. Listening to /camera1/image_raw")

    def image_callback(self, msg):
        # Convert ROS Image → OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is None:
            return
        # MediaPipe wants RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        state_text = "No Hand Detected"

        if results.multi_hand_landmarks:
            hand = results.multi_hand_landmarks[0]

            num_open_fingers, is_open = is_hand_open(hand)
            state_text = "OPEN" if is_open else "CLOSED"

            msg = Bool()
            msg.data = is_open
            self.pub.publish(msg)

            # Draw landmarks
            mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)
        else:
            msg = Bool()
            msg.data = True
            self.pub.publish(msg)

        # Display state
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
