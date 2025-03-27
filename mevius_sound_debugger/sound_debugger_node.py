#!/usr/bin/env python3

from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import numpy as np
# import simpleaudio as sa

class CorrectSoundNode(Node):
    def __init__(self):
        super().__init__('correct_sound_node')
        # "correct_sound_trigger" トピックからEmptyメッセージを受信
        self.subscription = self.create_subscription(
            Empty,
            'correct_sound_trigger',
            self.listener_callback,
            10)
        self.get_logger().info('Mevius Sound Debbugger has started.')

    def listener_callback(self, msg: Empty):
        self.get_logger().info('Sound trigger received.')
        self.play_correct_sound()

    def generate_tone(self, frequency, duration, sample_rate=44100, volume=0.5) -> np.ndarray:
        t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
        tone: np.ndarray = np.sin(2 * np.pi * frequency * t)
        audio = tone * volume * (2**15 - 1)
        return audio.astype(np.int16)
    
    def generate_correct_sound(self) -> List[np.ndarray]:
        # 例として、800Hzと600Hzのトーンを連続再生して正解音とする
        tone1 = self.generate_tone(800, 0.3, volume=0.8)
        tone2 = self.generate_tone(600, 0.3, volume=0.8)
        return [tone1, tone2]

    def play_correct_sound(self):
        # 例として、800Hzと600Hzのトーンを連続再生して正解音とする
        tones = self.generate_correct_sound()
        audio_data = np.concatenate(tuple([tones]))
        print("hogehoge")
        # play_obj = sa.play_buffer(audio_data, 1, 2, 44100)
        # play_obj.wait_done()

def main(args=None):
    rclpy.init(args=args)
    node = CorrectSoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mevius Sound Debbugger has stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

