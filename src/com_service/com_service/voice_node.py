"""
voice 노드 역할:
/tts_request 토픽으로 들어오는 text_key를 받아서,
미리 gTTS로 생성해둔 wav 파일을 aplay로 재생한다.
[설계 이유] SR-023(Wi-Fi 없어도 음성 안내 동작)을 만족하기 위해
클라우드 TTS를 실시간 호출하지 않고, 사전 생성된 wav만 재생한다.
"""

import os
import subprocess
from collections import deque

import rclpy
from rclpy.node import Node
from com_interfaces.msg import TtsRequest


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        self.declare_parameter('audio_library_path',
                                os.path.expanduser('~/com_ws/src/com_service/audio'))
        # 기본 100 - 2026-08-11 실측: 로봇 USB 오디오 동글(Jieli UACDemoV1.0)이 ALSA
        # 믹서 100%(-0.94dB, 사실상 유니티 게인)로 올려도 스피커가 겨우 들릴 정도라
        # 낮출 여유가 없음. wav 자체도 생성 시 게인을 올려둠(generate_audio.py 참고).
        self.declare_parameter('volume', 100)

        self.audio_path = self.get_parameter('audio_library_path').get_parameter_value().string_value
        self.volume = self.get_parameter('volume').get_parameter_value().integer_value

        self._apply_volume()

        self.subscription = self.create_subscription(
            TtsRequest,
            '/tts_request',
            self.on_tts_request,
            10
        )

        # aplay는 ALSA 디바이스를 독점으로 열어서, 이전 재생이 끝나기 전에 다음 요청이
        # 오면 "Device or resource busy"로 실패하고 그 안내가 통째로 씹힌다(2026-08-11
        # 통합 디버깅 중 실측: system_start 재생 중 3초 뒤 ready 요청이 겹쳐서 재현됨).
        # Popen의 비동기 특성(재생 중에도 다른 콜백 처리 가능)은 유지하면서, 큐에 쌓고
        # 타이머로 이전 재생 종료를 확인한 뒤 순차 재생한다.
        #
        # (text_key, wav_file, retry_count) 튜플을 큐에 넣는다 - USB 오디오 동글이
        # 간헐적으로 재연결되면서 aplay가 "No such device"로 실패하는 경우가 실측됨
        # (2026-08-11). 실패를 감지 못 하면 안내가 조용히 사라지므로, 종료코드를
        # 확인해서 실패 시 재시도한다.
        self.MAX_RETRIES = 2
        self._queue = deque()
        self._current_proc = None
        self._current_item = None
        self._playback_timer = self.create_timer(0.2, self._pump_queue)

        self.get_logger().info(f'voice 노드 시작, 오디오 경로: {self.audio_path}')

    def on_tts_request(self, msg: TtsRequest):
        wav_file = os.path.join(self.audio_path, f'{msg.text_key}.wav')

        if not os.path.exists(wav_file):
            self.get_logger().warn(f'사전 정의되지 않은 문구 요청: {msg.text_key} (파일 없음, 재생 생략)')
            return

        self._queue.append((msg.text_key, wav_file, 0))

    def _pump_queue(self):
        if self._current_proc is not None:
            if self._current_proc.poll() is None:
                return  # 아직 이전 재생 중

            returncode = self._current_proc.returncode
            key, wav_file, retries = self._current_item
            if returncode != 0:
                if retries < self.MAX_RETRIES:
                    self.get_logger().warn(
                        f'{key} 재생 실패(exit={returncode}) -> 재시도 ({retries + 1}/{self.MAX_RETRIES})'
                    )
                    self._queue.appendleft((key, wav_file, retries + 1))
                else:
                    self.get_logger().error(
                        f'{key} 재생 {self.MAX_RETRIES}회 모두 실패(exit={returncode}) - 포기 (주행에는 영향 없음)'
                    )
            self._current_proc = None
            self._current_item = None

        if not self._queue:
            return

        text_key, wav_file, retries = self._queue.popleft()
        self.get_logger().info(f'재생: {text_key}' + (f' (재시도 {retries})' if retries else ''))
        # USB 오디오 동글이 간헐적으로 재연결되면서 볼륨이 기본값(30%)으로 되돌아가는
        # 현상이 실측됨(sudo 권한이 없어 alsactl store로 영구 저장 불가) - 재생마다
        # 매번 볼륨을 강제로 맞춰서 방어한다.
        self._apply_volume()
        self._current_item = (text_key, wav_file, retries)
        try:
            self._current_proc = subprocess.Popen(['aplay', wav_file])
        except Exception as e:
            self.get_logger().error(f'오디오 재생 실패: {e} (주행에는 영향 없음)')
            self._current_proc = None
            self._current_item = None

    def _apply_volume(self):
        try:
            subprocess.run(
                ['amixer', 'sset', 'PCM', f'{self.volume}%'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2.0
            )
        except Exception as e:
            self.get_logger().warn(f'볼륨 설정 실패: {e} (재생은 계속 시도)')


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