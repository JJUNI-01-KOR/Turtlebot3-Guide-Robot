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
        # 2026-08-12 실측: 100%(-0.94dB, 유니티 게인)에서 USB 오디오 동글(Jieli
        # UACDemoV1.0)이 재생 중 끊김/치직거림(aplay write error: No such device,
        # 이후 ALSA가 카드 자체를 못 찾음)을 반복 유발함 - 30%는 재생은 안정적이나
        # 너무 작아서 안 들림. 80%(-6.36dB)로 고정 - 사용자 실측 확인.
        self.declare_parameter('volume', 80)

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
        # 2026-08-18 코드 리뷰: subprocess.run(...)은 amixer가 끝날 때까지 최대
        # 2초간 이 콜백(=노드 전체, 단일스레드)을 막는다. _pump_queue가 재생마다
        # 이 함수를 호출하므로, USB 오디오 동글이 불안정해서 amixer가 같이
        # 멈칫거리면 그 2초 동안 재생 큐 전체가 멎는다 - 파일 상단 설계 의도인
        # "Popen의 비동기 특성 유지"와 정면으로 모순됐다. 반환코드는 원래도
        # 확인 안 했으므로(예외만 처리) aplay와 동일하게 Popen으로 바꿔도
        # 동작은 그대로다.
        try:
            subprocess.Popen(
                ['amixer', 'sset', 'PCM', f'{self.volume}%'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
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
        # 2026-08-18: SIGINT로 정상 종료될 때 rclpy 시그널 핸들러가 이미 컨텍스트를
        # 닫아둔 상태라, 여기서 또 shutdown하면 RCLError가 터져 exit code 1로 끝난다.
        # PM2는 그걸 '크래시'로 세서 재시작 카운터를 소진시키고 결국 스택 전체를
        # 내려버린다(8/18 시험주행 후 8개 노드 전부 stopped 된 직접 원인).
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()