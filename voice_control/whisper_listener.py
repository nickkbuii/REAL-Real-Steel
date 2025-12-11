#!/usr/bin/env python3
"""
ROS2 node: real-time microphone → Whisper → /voice_cmd,
publishing only when the text contains a valid robot command.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd
import numpy as np
from faster_whisper import WhisperModel
import queue
import threading

SAMPLE_RATE = 16000
BLOCK_SIZE = 1024
MODEL = "base" # Can change to tiny if speech-to-text too slow

COMMANDS = ["jab", "uppercut", "hook", "block"]


class whisper_listener(Node):
    """Listens to the microphone and publishes commands on /voice_cmd."""

    def __init__(self):
        super().__init__("whisper_listener")
        self.pub = self.create_publisher(String, "/voice_cmd", 10)
        self.model = WhisperModel(MODEL, device="cpu", compute_type="int8")

        self.audio_q = queue.Queue()
        self.stream = sd.InputStream(
            channels=1,
            samplerate=SAMPLE_RATE,
            blocksize=BLOCK_SIZE,
            callback=self.audio_cb,
        )
        self.stream.start()

        self.running = True
        threading.Thread(target=self.loop, daemon=True).start()

    def audio_cb(self, indata, frames, time, status):
        self.audio_q.put(indata[:, 0].copy())

    def loop(self):
        buf = []
        while self.running:
            buf.extend(self.audio_q.get())

            if len(buf) >= SAMPLE_RATE * 0.5:
                audio = np.array(buf, dtype=np.float32)
                buf = []

                segments, _ = self.model.transcribe(audio, beam_size=1, vad_filter=True)

                for s in segments:
                    text = s.text.strip().lower()

                    if not text:
                        continue

                    if any(cmd in text for cmd in COMMANDS):
                        self.pub.publish(String(data=text))

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = whisper_listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()