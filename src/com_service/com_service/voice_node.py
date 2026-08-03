"""
voice 노드 역할:
/tts_request 토픽으로 들어오는 text_key를 받아서,
미리 gTTS로 생성해둔 wav 파일을 aplay로 재생한다.
[설계 이유] SR-023(Wi-Fi 없어도 음성 안내 동작)을 만족하기 위해
클라우드 TTS를 실시간 호출하지 않고, 사전 생성된 wav만 재생한다.
"""

import os
import subprocess
import rclpy
from rclpy.node import Node
from com_interfaces.msg import TtsRequest


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        self.declare_parameter('audio_library_path',
                                os.path.expanduser('~/com_ws/src/com_service/audio'))
        self.declare_parameter('volume', 80)

        self.audio_path = self.get_parameter('audio_library_path').get_parameter_value().string_value
        self.volume = self.get_parameter('volume').get_parameter_value().integer_value

        self.subscription = self.create_subscription(
            TtsRequest,
            '/tts_request',
            self.on_tts_request,
            10
        )

        self.get_logger().info(f'voice 노드 시작, 오디오 경로: {self.audio_path}')

    def on_tts_request(self, msg: TtsRequest):
        wav_file = os.path.join(self.audio_path, f'{msg.text_key}.wav')

        if not os.path.exists(wav_file):
            self.get_logger().warn(f'사전 정의되지 않은 문구 요청: {msg.text_key} (파일 없음, 재생 생략)')
            return

        self.get_logger().info(f'재생: {msg.text_key}')
        try:
            # subprocess로 비동기 실행 -> 재생 중에도 노드가 다른 콜백 처리 가능
            subprocess.Popen(['aplay', wav_file])
        except Exception as e:
            self.get_logger().error(f'오디오 재생 실패: {e} (주행에는 영향 없음)')


def main():
    rclpy.init()
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()