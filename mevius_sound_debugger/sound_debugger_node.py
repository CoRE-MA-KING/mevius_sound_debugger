#!/usr/bin/env python3

import time
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from mevius_massage.msg import MeviusLog

import numpy as np
import simpleaudio as sa

MEVIUS_MAIN_CHECK_BUTTON = 2
REALSENSE_CHECK_BUTTON = 3
VOLUME = 0.3

SCALE = {
    "doL": 261.63,
    "reL": 293.66,
    "miL": 329.63,
    "faL": 349.23,
    "soL": 392.00,
    "laL": 440.00,
    "siL": 493.88,
    "do": 523.25,
    "re": 587.33,
    "mi": 659.25,
    "fa": 698.46,
    "so": 783.99,
    "la": 880.00,
    "si": 987.77,
    "doH": 1046.50,
    "reH": 1174.66,
    "miH": 1318.51,
    "faH": 1396.91,
    "soH": 1567.98,
    "laH": 1760.00,
    "siH": 1975.53,
    "doHH": 2093.00
}

class SoundDebuggerNode(Node):
    def __init__(self):
        super().__init__('sound_debugger_node')

        self.mevius_subscription = self.create_subscription(
            MeviusLog,
            'mevius_log',
            self.mevius_callback,
            1
        )

        self.realsense_subscription = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.realsense_callback,
            1
        )

        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            1
        )

        # 最後に /mevius_log を受信した時刻（初期はノード起動時）
        self.last_mevius_log_time = time.time()
        self.last_realsense_time = time.time()
        self.play_obj = None
        self.get_logger().info('Mevius Sound Debugger has started.')

    def mevius_callback(self, msg: MeviusLog):
        self.last_mevius_log_time = time.time()
        self.get_logger().debug('Received /mevius_log message.')

    def realsense_callback(self, msg: Odometry):
        self.last_realsense_time = time.time()
        self.get_logger().debug('Received /camera/pose/sample message.')

    def joy_callback(self, msg: Joy):
        if msg.buttons:
            if msg.buttons[MEVIUS_MAIN_CHECK_BUTTON] == 1:
                self.report_aliveness(self.last_mevius_log_time, "mevius_log")
            elif msg.buttons[REALSENSE_CHECK_BUTTON] == 1:
                self.report_aliveness(self.last_realsense_time, "realsense")
    
    def report_aliveness(self, last_time: float, log_name: str):
        now = time.time()
        elapsed = now - last_time
        if elapsed > 1.0:
            self.get_logger().info('Dead: (last subscription of %s was %.2f seconds ago)' % (log_name, elapsed))
            self.play_sound(is_alive=False)
        else:
            self.get_logger().info('Alive: (last subscription of %s was %.2f seconds ago)' % (log_name, elapsed))
            if log_name == "mevius_log":
                self.play_sound(is_alive=True, num=0)
            else:
                self.play_sound(is_alive=True, num=1)

    def generate_tone(self, frequency, duration, sample_rate=44100, volume=0.5) -> np.ndarray:
        t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
        tone = np.sin(2 * np.pi * frequency * t)
        audio: np.ndarray = tone * volume * (2**15 - 1)
        return audio.astype(np.int16)
    
    def generate_alive_sound(self, num: int) -> List[np.ndarray]:
        if num == 0:
            # ピタゴラ
            return [
                self.generate_tone(SCALE['re'], 0.1, volume=VOLUME),
                self.generate_tone(SCALE['mi'], 0.1, volume=VOLUME),
                self.generate_tone(0          , 0.1, volume=VOLUME),
                # self.generate_tone(SCALE['re'], 0.1, volume=VOLUME),
                # self.generate_tone(SCALE['mi'], 0.1, volume=VOLUME),
                self.generate_tone(SCALE['do'], 0.1, volume=VOLUME),
                self.generate_tone(SCALE['re'], 0.1, volume=VOLUME),
            ]
        elif num == 1:
            # スイッチ
            return [
                self.generate_tone(SCALE['doH'], 0.1, volume=VOLUME),
                self.generate_tone(SCALE['si'], 0.1, volume=VOLUME),
                self.generate_tone(0          , 0.2, volume=VOLUME),
                # self.generate_tone(SCALE['so'], 0.2, volume=VOLUME),
                self.generate_tone(SCALE['fa'], 0.2, volume=VOLUME),
            ]
        else:
            return [
                self.generate_tone(SCALE['doH'], 0.3, volume=VOLUME),
            ]


    def generate_dead_sound(self) -> List[np.ndarray]:
        return [
            self.generate_tone(SCALE['doH'], 0.1, volume=VOLUME),
            self.generate_tone(SCALE['si'], 0.1, volume=VOLUME),
            self.generate_tone(0          , 0.2, volume=VOLUME),
            self.generate_tone(SCALE['la'], 0.2, volume=VOLUME),
        ]
        # return [
        #     self.generate_tone(SCALE['reL'], 0.1, volume=VOLUME),
        #     self.generate_tone(0          , 0.1, volume=VOLUME),
        #     self.generate_tone(SCALE['reL'], 0.1, volume=VOLUME),
        # ]

    def play_sound(self, is_alive: bool, num: int = 0):
        if is_alive:
            tones = self.generate_alive_sound(num)
        else:
            tones = self.generate_dead_sound()
        audio_data = np.concatenate(tuple(tones))
        self.play_obj = sa.play_buffer(audio_data, 1, 2, 44100)
        self.play_obj.wait_done()

def main(args=None):
    rclpy.init(args=args)
    node = SoundDebuggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
