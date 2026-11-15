"""
main_window.py
--------------
COM 방문자 안내 로봇 관제 대시보드 (다크 테마).

레이아웃
  [헤더]  타이틀 / 연결상태 / 현재시각 / 배터리 배지
  [좌]  SLAM 맵 뷰, 카메라 실시간 영상
  [중]  임무 제어, 로봇 상태, 센서 모니터
  [우]  배터리 상태(도넛+추이), 알람·이벤트, 임무 이력 타임라인, PM2 프로세스 관리
  [하]  디버그용 수동 주행, 측정 로그 조회

설계 근거
  - 화면 요소: HMI-001 4장(UI 구성요소), 5장(상태 전이표)
  - 상태명/전이: SDD-001 3.1
  - 토픽/서비스: ICD-001 2장
  - 수동주행 패널은 정식 기능이 아니라 디버그용. HMI-001 6장에 따라
    안전기능(긴급정지)은 GUI에 두지 않고, 수동주행은 READY 상태 +
    체크박스 활성화라는 이중 잠금이 걸려 있을 때만 /cmd_vel을 발행한다.
  - PM2 프로세스 관리는 임무 제어 카드의 목적지 콤보박스와 같은 방식으로,
    라즈베리파이의 pm2 프로세스 8개(PM2_PROCESSES) 중 하나 또는 맨 아래
    "전체 프로세스"를 골라 시작/종료/재시작한다. GUI(개발 PC)는 SSH로
    라즈베리파이(PM2_SSH_USER@PM2_SSH_HOST)에 접속해 pm2 명령을 보낸다 —
    로컬 pm2를 부르는 게 아니다.
"""

import os
import re
import shutil
import time
from datetime import datetime

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QLabel, QPushButton, QComboBox, QLineEdit,
    QCheckBox, QTableWidget, QTableWidgetItem, QHeaderView, QMessageBox,
    QVBoxLayout, QHBoxLayout, QGridLayout, QFrame, QScrollArea,
    QStatusBar, QSizePolicy, QAbstractItemView
)
from PyQt5.QtGui import QImage
from PyQt5.QtCore import Qt, QTimer, QProcess

from . import theme as T
from .theme import STYLESHEET, STATE_COLOR, STATE_LABEL_KO
from .widgets import (
    Card, Dot, make_pill, BatteryDonut, Sparkline, SensorBar,
    MapView, CameraView, EventRow, TimelineRow, StatusRow, AlertBanner
)
from .ros_worker import RosWorkerThread


# 목적지 8곳 — ~/com_ws/src/com_navigation/config/waypoints.yaml 의
# id 값을 그대로 사용한다 (2026-08-19: room1~8 -> class1~5/info/office/meeting 으로
# id 이름 변경. 좌표는 그대로이고 키 이름만 바뀐 것이라 표시 라벨은 유지했다.
# 이미 등록되어 있어 추가함. 구 LECTURE1 등의 합성 코드는 폐기됨).
# id 는 내부 키일 뿐 라벨 뜻과 1:1 대응이 아니다 (2026-08-19 확인):
#   info -> 회의실, office -> 인포, meeting -> 상담실. 물리 위치는 이름 변경 전과 동일.
DESTINATIONS = [
    ('강의실1', 'class1'),
    ('강의실2', 'class2'),
    ('강의실3', 'class3'),
    ('강의실4', 'class4'),
    ('강의실5', 'class5'),
    ('회의실', 'meeting'),
    ('인포', 'info'),
    ('상담실', 'office'),
]

# 이 상태들에서는 임무 취소 버튼이 활성화 (HMI-001 5장)
CANCELABLE_STATES = (
    'GUIDING', 'RETURNING', 'DOCKING', 'OBSTACLE_STOP', 'ARRIVED', 'PERSON_WAIT')

STATE_TIMEOUT_SEC = 3.0        # /robot_state 끊김 판정 시간
CAMERA_TIMEOUT_SEC = 2.0       # 카메라 프레임 끊김 판정 (SR-025는 1초, 표시용은 여유)
SCAN_TIMEOUT_SEC = 2.0
SENSOR_TIMEOUT_SEC = 3.0       # 불꽃/환경 센서 미수신 판정 (SR-024는 2초, 표시용 여유)

MANUAL_LINEAR_SPEED = 0.10     # m/s (SR-032 최대 0.22 m/s 이내)
MANUAL_ANGULAR_SPEED = 0.60    # rad/s

MAX_ALARMS = 6
MAX_TIMELINE = 40

# SR-014 온도 이상 알림. 센서가 2초마다 값을 보내므로 임계치를 넘은 '순간'에만
# 알람을 남기고, 다시 남기려면 해제 임계치 아래로 내려갔다 와야 한다(히스테리시스).
# 안 그러면 알람 카드 6칸이 같은 온도 알람으로만 채워져 다른 이벤트가 밀려난다.
TEMP_ALARM_ON = 35.0
TEMP_ALARM_OFF = 34.0

# ---- PM2 프로세스 관리 (임무 이력 타임라인 아래 빈 공간) ----
# 라즈베리파이에서 `pm2 list`로 확인한 실제 프로세스 8개 (2026-08-14 확인).
# 임무 제어 카드의 목적지 콤보박스와 같은 방식: 콤보에서 하나 골라 시작/종료/
# 재시작한다. 목록 맨 밑의 "전체 프로세스"를 고르면 8개를 한 번에 제어한다.
# pm2 프로세스가 늘거나 이름이 바뀌면 이 리스트만 고치면 된다.
PM2_PROCESSES = [
    'bringup',
    'hmi_bridge',
    'navigation',
    'sensor_bridge',
    'service_manager',
    'status',
    'vision',
    'voice',
]
PM2_ALL_LABEL = '전체 프로세스'
PM2_ALL_VALUE = 'all'

# GUI는 개발 PC에서 뜨고, pm2는 로봇(라즈베리파이4)에서 돈다 — 그래서
# 로컬 pm2를 부르면 안 되고, 반드시 SSH로 라즈베리파이에 접속해서 그쪽
# pm2에 명령을 보내야 한다. 접속 정보는 환경변수 COM_ROBOT_SSH_USER /
# COM_ROBOT_SSH_HOST 로 바꾼다 (없으면 아래 기본값).
#
# 필수 전제: 이 SSH는 GUI(QProcess)가 비밀번호 프롬프트에 응답할 수
# 없으므로 반드시 "키 기반 무비밀번호 로그인"이 설정돼 있어야 한다.
#   dev PC에서 (키가 없다면) ssh-keygen -t ed25519
#   dev PC에서              ssh-copy-id <계정>@<로봇 주소>
#   확인:                   ssh <계정>@<로봇 주소> 'pm2 list'  ← 비번 안 물어봐야 함
PM2_SSH_USER = os.environ.get('COM_ROBOT_SSH_USER', 'com')
PM2_SSH_HOST = os.environ.get('COM_ROBOT_SSH_HOST', '192.168.210.15')
# -o BatchMode=yes: 키 인증 실패 시 비밀번호를 물어보며 멈추는 대신 즉시 실패시킴
# -o ConnectTimeout=5: 라즈베리파이가 꺼져있거나 네트워크가 끊겼을 때 5초 안에 실패
PM2_RUN_PREFIX = [
    'ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
    f'{PM2_SSH_USER}@{PM2_SSH_HOST}',
]
PM2_ACTION_LABEL = {'start': '시작', 'stop': '종료', 'restart': '재시작'}
# ssh -o ConnectTimeout=5로도 못 막는 경우(방화벽이 SYN만 흘려버리는 등)에 대비한
# 최후의 안전장치. 이 시간 안에 안 끝나면 강제로 죽이고 버튼을 되살린다.
PM2_TIMEOUT_MS = 15000
# 라즈베리파이 쪽 pm2가 쓰는 PM2_HOME. 위 계정으로 SSH 로그인하면 그 계정의
# 기본값($HOME/.pm2)이 알아서 맞기 때문에 보통은 None으로 둬도 된다.
# 라즈베리파이에서 pm2가 다른 계정/PM2_HOME으로 떠 있을 때만 채운다.
PM2_HOME_OVERRIDE = None

# pm2(정확히는 내부 chalk 라이브러리)는 파이프로 실행돼도 색상 코드를
# 섞어 보낼 때가 있어, 화면에 \x1b[31m 같은 글자가 그대로 찍히는 걸 막기 위한 정규식
_ANSI_ESCAPE_RE = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')


def _strip_ansi(text: str) -> str:
    return _ANSI_ESCAPE_RE.sub('', text)


class MainWindow(QMainWindow):

    def __init__(self, ros_domain_id=30):
        super().__init__()
        self.setWindowTitle('COM 방문자 안내 로봇 — 관제 대시보드')
        self.resize(1400, 985)
        self.setStyleSheet(STYLESHEET)

        # ---- 내부 상태 ----
        self.ros_domain_id = ros_domain_id
        self._last_state_time = 0.0
        self._last_camera_time = 0.0
        self._last_scan_time = 0.0
        self._mission_state = 'UNKNOWN'
        self._battery_history = []
        self._last_flame_time = 0.0
        self._flame_on = False       # 현재 불꽃 감지 상태
        self._flame_was_on = False   # 직전 상태 (감지 '시작' 순간만 알람 남기려고)
        self._temp_alarm_on = False  # 온도 알람도 같은 이유로 에지에서만 남긴다
        self._pm2_process = None     # 실행 중인 QProcess 참조 (GC 방지 + 중복실행 방지)

        # ---- ROS 스레드 ----
        self.ros = RosWorkerThread(ros_domain_id=ros_domain_id)
        self.ros.robot_state.connect(self._on_robot_state)
        self.ros.map_received.connect(self._on_map)
        self.ros.camera_frame.connect(self._on_camera_frame)
        self.ros.scan_summary.connect(self._on_scan)
        self.ros.rear_range.connect(self._on_rear_range)
        self.ros.flame_detected.connect(self._on_flame)
        self.ros.person_detected.connect(self._on_person)
        self.ros.temperature.connect(self.temp_signal_handler)
        self.ros.humidity.connect(self.humi_signal_handler)
        self.ros.rear_distance_cm.connect(self.rear_cm_signal_handler)
        self.ros.env_data.connect(self._on_env_data)
        self.ros.system_error.connect(self._on_system_error)
        self.ros.start_result.connect(self._on_start_result)
        self.ros.cancel_result.connect(self._on_cancel_result)
        self.ros.log_result.connect(self._on_log_result)
        self.ros.service_failed.connect(self._on_service_failed)

        self._build_ui()
        self.ros.start()

        # 1초마다 시계 갱신 + 연결 상태 감시
        self._tick_timer = QTimer(self)
        self._tick_timer.timeout.connect(self._tick)
        self._tick_timer.start(1000)

        self._add_timeline('GUI 시작됨', T.TEXT_SUB)

    # ==================================================================
    # UI 조립
    # ==================================================================
    def _build_ui(self):
        # 창이 작아지면 스크롤되도록 전체를 스크롤 영역에 담는다
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        page = QWidget()
        root = QVBoxLayout(page)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(14)

        root.addWidget(self._build_header())

        # ---- 3컬럼 본문 ----
        columns = QHBoxLayout()
        columns.setSpacing(14)
        columns.addLayout(self._build_left_column(), 34)
        columns.addLayout(self._build_center_column(), 33)
        columns.addLayout(self._build_right_column(), 33)
        root.addLayout(columns)

        # ---- 하단 2컬럼 ----
        bottom = QHBoxLayout()
        bottom.setSpacing(14)
        bottom.addWidget(self._build_manual_card(), 34)
        bottom.addWidget(self._build_log_card(), 66)
        root.addLayout(bottom)

        scroll.setWidget(page)
        self.setCentralWidget(scroll)

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage(
            f'ROS_DOMAIN_ID={self.ros_domain_id} · 로봇 상태 수신 대기 중…')

    # ---------------- 헤더 ----------------
    def _build_header(self):
        card = QFrame()
        card.setObjectName('card')
        layout = QHBoxLayout(card)
        layout.setContentsMargins(20, 14, 20, 14)
        layout.setSpacing(14)

        self.conn_dot = Dot(T.TEXT_DIM, 11)
        layout.addWidget(self.conn_dot)

        title_col = QVBoxLayout()
        title_col.setSpacing(2)
        title = QLabel('COM 방문자 안내 로봇 — 관제 대시보드')
        title.setObjectName('appTitle')
        self.conn_sub = QLabel('연결 대기 중 · 로봇 상태 토픽 미수신')
        self.conn_sub.setObjectName('caption')
        title_col.addWidget(title)
        title_col.addWidget(self.conn_sub)
        layout.addLayout(title_col)

        layout.addStretch()

        clock_col = QVBoxLayout()
        clock_col.setSpacing(0)
        clock_cap = QLabel('현재 시각')
        clock_cap.setObjectName('caption')
        clock_cap.setAlignment(Qt.AlignRight)
        self.clock_label = QLabel('--:--:--')
        self.clock_label.setObjectName('clock')
        self.clock_label.setAlignment(Qt.AlignRight)
        clock_col.addWidget(clock_cap)
        clock_col.addWidget(self.clock_label)
        layout.addLayout(clock_col)

        badge = QFrame()
        badge.setObjectName('badge')
        badge_layout = QHBoxLayout(badge)
        badge_layout.setContentsMargins(12, 7, 12, 7)
        badge_layout.setSpacing(7)
        self.batt_dot = Dot(T.TEXT_DIM, 8)
        badge_layout.addWidget(self.batt_dot)
        badge_cap = QLabel('배터리')
        badge_cap.setObjectName('caption')
        badge_layout.addWidget(badge_cap)
        self.batt_badge_value = QLabel('--%')
        self.batt_badge_value.setStyleSheet(
            f'font-weight: bold; color: {T.ACCENT_CYAN};')
        badge_layout.addWidget(self.batt_badge_value)
        layout.addWidget(badge)

        return card

    # ---------------- 좌측 컬럼 ----------------
    def _build_left_column(self):
        col = QVBoxLayout()
        col.setSpacing(14)

        self.map_pill = make_pill('LIVE', 'info')
        map_card = Card('SLAM 맵 뷰', badge=self.map_pill)
        self.map_view = MapView()
        map_card.body.addWidget(self.map_view)
        col.addWidget(map_card)

        self.cam_pill = make_pill('● REC', 'rec')
        cam_card = Card('카메라 실시간 영상', badge=self.cam_pill)
        self.camera_view = CameraView()
        cam_card.body.addWidget(self.camera_view)
        col.addWidget(cam_card)

        col.addStretch()
        return col

    # ---------------- 중앙 컬럼 ----------------
    def _build_center_column(self):
        col = QVBoxLayout()
        col.setSpacing(14)

        # --- 임무 제어 ---
        mission_card = Card('임무 제어')
        self.dest_combo = QComboBox()
        for name, code in DESTINATIONS:
            self.dest_combo.addItem(name, userData=code)
        mission_card.body.addWidget(self.dest_combo)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(9)
        self.start_btn = QPushButton('임무 시작')
        self.start_btn.setObjectName('startBtn')
        self.start_btn.clicked.connect(self._on_start_clicked)
        self.start_btn.setEnabled(False)

        self.cancel_btn = QPushButton('임무 취소')
        self.cancel_btn.setObjectName('cancelBtn')
        self.cancel_btn.clicked.connect(self._on_cancel_clicked)
        self.cancel_btn.setEnabled(False)

        btn_row.addWidget(self.start_btn)
        btn_row.addWidget(self.cancel_btn)
        btn_row.addStretch()
        mission_card.body.addLayout(btn_row)
        col.addWidget(mission_card)

        # --- 로봇 상태 ---
        status_card = Card('로봇 상태')
        grid = QGridLayout()
        grid.setHorizontalSpacing(18)
        grid.setVerticalSpacing(4)

        self.pose_value = QLabel('x=--, y=--')
        self.pose_value.setObjectName('valueBig')
        self.theta_value = QLabel('θ=--°')
        self.theta_value.setObjectName('valueBig')
        self.stage_value = QLabel('--')
        self.stage_value.setObjectName('valueBig')
        self.waypoint_value = QLabel('--')
        self.waypoint_value.setObjectName('valueBig')

        grid.addWidget(self._caption('위치'), 0, 0)
        grid.addWidget(self._caption('방향'), 0, 1)
        grid.addWidget(self.pose_value, 1, 0)
        grid.addWidget(self.theta_value, 1, 1)
        grid.addWidget(self._caption('현재 단계'), 2, 0)
        grid.addWidget(self._caption('현재 경유점'), 2, 1)
        grid.addWidget(self.stage_value, 3, 0)
        grid.addWidget(self.waypoint_value, 3, 1)
        grid.setRowMinimumHeight(2, 22)
        status_card.body.addLayout(grid)
        col.addWidget(status_card)

        # --- 센서 모니터 ---
        sensor_card = Card('센서 모니터')

        # 불꽃 감지 시 카드 맨 위에 빨간 경고 띠가 나타나며 깜빡인다
        self.flame_banner = AlertBanner()
        sensor_card.body.addWidget(self.flame_banner)

        self.temp_bar = SensorBar('온도 (DHT11)', '°C', 0, 50, T.ACCENT_ORANGE)
        self.humi_bar = SensorBar('습도 (DHT11)', '%', 0, 100, '#58a6ff')
        self.dist_bar = SensorBar('후방 거리 (IR/초음파)', 'cm', 0, 100, T.ACCENT_CYAN)
        sensor_card.body.addWidget(self.temp_bar)
        sensor_card.body.addWidget(self.humi_bar)
        sensor_card.body.addWidget(self.dist_bar)

        self.flame_row = StatusRow('불꽃 감지')
        self.lidar_row = StatusRow('LiDAR 스캔')
        self.person_row = StatusRow('사람 감지')
        sensor_card.body.addWidget(self.flame_row)
        sensor_card.body.addWidget(self.lidar_row)
        sensor_card.body.addWidget(self.person_row)
        col.addWidget(sensor_card)

        col.addStretch()
        return col

    # ---------------- 우측 컬럼 ----------------
    def _build_right_column(self):
        col = QVBoxLayout()
        col.setSpacing(14)

        # --- 배터리 상태 ---
        batt_card = Card('배터리 상태')
        row = QHBoxLayout()
        row.setSpacing(16)
        self.batt_donut = BatteryDonut()
        row.addWidget(self.batt_donut)
        self.batt_spark = Sparkline(max_points=90, color=T.ACCENT_CYAN)
        row.addWidget(self.batt_spark, 1)
        batt_card.body.addLayout(row)
        self.batt_caption = QLabel('배터리 데이터 수신 대기 중')
        self.batt_caption.setObjectName('caption')
        batt_card.body.addWidget(self.batt_caption)
        col.addWidget(batt_card)

        # --- 알람 · 이벤트 ---
        alarm_card = Card('알람 · 이벤트')
        self.alarm_layout = QVBoxLayout()
        self.alarm_layout.setSpacing(2)
        alarm_card.body.addLayout(self.alarm_layout)
        self.alarm_empty = QLabel('발생한 알람이 없습니다')
        self.alarm_empty.setObjectName('caption')
        self.alarm_layout.addWidget(self.alarm_empty)
        col.addWidget(alarm_card)

        # --- 임무 이력 타임라인 ---
        timeline_card = Card('임무 이력 타임라인')
        self.timeline_scroll = QScrollArea()
        self.timeline_scroll.setWidgetResizable(True)
        self.timeline_scroll.setFrameShape(QFrame.NoFrame)
        self.timeline_scroll.setFixedHeight(150)
        self.timeline_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        timeline_inner = QWidget()
        self.timeline_layout = QVBoxLayout(timeline_inner)
        self.timeline_layout.setContentsMargins(0, 0, 6, 0)
        self.timeline_layout.setSpacing(0)
        self.timeline_layout.addStretch()
        self.timeline_scroll.setWidget(timeline_inner)
        timeline_card.body.addWidget(self.timeline_scroll)
        col.addWidget(timeline_card)

        # --- PM2 프로세스 관리 (타임라인 카드 바로 아래 빈 공간) ---
        col.addWidget(self._build_pm2_card())

        col.addStretch()
        return col

    # ---------------- 우측: PM2 프로세스 관리 ----------------
    def _build_pm2_card(self):
        card = Card('PM2 프로세스 관리')

        # 임무 제어 카드(목적지 콤보)와 같은 방식: 콤보에서 대상을 고르고
        # 아래 버튼으로 시작/종료/재시작. 목록 맨 밑 "전체 프로세스"를 고르면
        # 8개 다 한 번에 처리한다.
        self.pm2_target_combo = QComboBox()
        for proc_name in PM2_PROCESSES:
            self.pm2_target_combo.addItem(proc_name, userData=proc_name)
        self.pm2_target_combo.addItem(PM2_ALL_LABEL, userData=PM2_ALL_VALUE)
        card.body.addWidget(self.pm2_target_combo)

        # GUI가 실제로 어디로 pm2 명령을 보내는지 항상 눈에 보이게.
        # 라즈베리파이에서 `pm2 list`가 되는 계정/주소와 아래 줄이 다르면
        # 그게 바로 "No process found" / SSH 접속 실패의 원인이다.
        ssh_bin = shutil.which('ssh') or 'ssh (경로 못 찾음)'
        target_text = f'대상: {ssh_bin} → {PM2_SSH_USER}@{PM2_SSH_HOST}'
        if PM2_HOME_OVERRIDE:
            target_text += f'  ·  PM2_HOME={PM2_HOME_OVERRIDE}'
        target_caption = QLabel(target_text)
        target_caption.setObjectName('caption')
        target_caption.setWordWrap(True)
        card.body.addWidget(target_caption)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(9)
        self.pm2_start_btn = QPushButton('시작')
        self.pm2_start_btn.setObjectName('pm2StartBtn')
        self.pm2_stop_btn = QPushButton('종료')
        self.pm2_stop_btn.setObjectName('pm2StopBtn')
        self.pm2_restart_btn = QPushButton('재시작')
        self.pm2_restart_btn.setObjectName('pm2RestartBtn')

        self.pm2_start_btn.clicked.connect(lambda: self._on_pm2_clicked('start'))
        self.pm2_stop_btn.clicked.connect(lambda: self._on_pm2_clicked('stop'))
        self.pm2_restart_btn.clicked.connect(lambda: self._on_pm2_clicked('restart'))

        btn_row.addWidget(self.pm2_start_btn)
        btn_row.addWidget(self.pm2_stop_btn)
        btn_row.addWidget(self.pm2_restart_btn)
        card.body.addLayout(btn_row)

        self.pm2_status_label = QLabel('pm2 명령 대기 중')
        self.pm2_status_label.setObjectName('caption')
        self.pm2_status_label.setWordWrap(True)
        card.body.addWidget(self.pm2_status_label)

        return card

    # ---------------- 하단: 수동 주행 ----------------
    def _build_manual_card(self):
        self.manual_chk = QCheckBox('수동모드 활성화')
        self.manual_chk.setEnabled(False)
        self.manual_chk.stateChanged.connect(self._on_manual_toggled)

        card = Card('디버그용 수동 주행', badge=self.manual_chk)

        warn = QLabel(
            '⚠ READY 상태일 때만 활성화됩니다. 임무 진행 중에는 Nav2가 '
            '/cmd_vel을 사용하므로 절대 같이 누르지 마세요.')
        warn.setWordWrap(True)
        warn.setStyleSheet(f'color: {T.ACCENT_AMBER}; font-size: 11px;')
        card.body.addWidget(warn)

        self.btn_forward = QPushButton('▲ 전진')
        self.btn_left = QPushButton('◀ 좌회전')
        self.btn_stop = QPushButton('■ 정지')
        self.btn_right = QPushButton('▶ 우회전')
        self.btn_backward = QPushButton('▼ 후진')

        for b in (self.btn_forward, self.btn_left, self.btn_right, self.btn_backward):
            b.setObjectName('driveBtn')
        self.btn_stop.setObjectName('stopBtn')

        grid = QGridLayout()
        grid.setSpacing(9)
        grid.addWidget(self.btn_forward, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_backward, 2, 1)

        wrap = QHBoxLayout()
        wrap.addStretch()
        wrap.addLayout(grid)
        wrap.addStretch()
        card.body.addLayout(wrap)

        # 누르고 있는 동안만 이동, 떼면 즉시 정지 (조이스틱처럼)
        self.btn_forward.pressed.connect(lambda: self._drive(MANUAL_LINEAR_SPEED, 0.0))
        self.btn_backward.pressed.connect(lambda: self._drive(-MANUAL_LINEAR_SPEED, 0.0))
        self.btn_left.pressed.connect(lambda: self._drive(0.0, MANUAL_ANGULAR_SPEED))
        self.btn_right.pressed.connect(lambda: self._drive(0.0, -MANUAL_ANGULAR_SPEED))
        for b in (self.btn_forward, self.btn_backward, self.btn_left, self.btn_right):
            b.released.connect(self._drive_stop)
        self.btn_stop.clicked.connect(self._drive_stop)

        self._set_drive_buttons(False)
        return card

    # ---------------- 하단: 측정 로그 ----------------
    def _build_log_card(self):
        card = Card('측정 로그 조회')

        filter_row = QHBoxLayout()
        filter_row.setSpacing(9)
        self.log_wp_filter = QLineEdit()
        self.log_wp_filter.setPlaceholderText('예: WP3 (비우면 전체)')
        self.log_date_filter = QLineEdit()
        self.log_date_filter.setPlaceholderText('YYYY-MM-DD (비우면 전체)')
        query_btn = QPushButton('조회')
        query_btn.setObjectName('queryBtn')
        query_btn.clicked.connect(self._on_query_clicked)

        filter_row.addWidget(self.log_wp_filter, 1)
        filter_row.addWidget(self.log_date_filter, 1)
        filter_row.addWidget(query_btn)
        card.body.addLayout(filter_row)

        self.log_table = QTableWidget(0, 5)
        self.log_table.setHorizontalHeaderLabels(
            ['경유점', '온도(°C)', '습도(%)', '조도(lx)', '측정시각'])
        self.log_table.verticalHeader().setVisible(False)
        self.log_table.setShowGrid(False)
        self.log_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.log_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.log_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.log_table.setMinimumHeight(150)
        card.body.addWidget(self.log_table)
        return card

    @staticmethod
    def _caption(text):
        label = QLabel(text)
        label.setObjectName('caption')
        return label

    # ==================================================================
    # ROS 신호 처리
    # ==================================================================
    def _on_robot_state(self, msg):
        self._last_state_time = time.time()

        # 위치 / 방향
        self.pose_value.setText(f'x={msg.pose.x:.1f}, y={msg.pose.y:.1f}')
        deg = msg.pose.theta * 180.0 / 3.141592653589793
        self.theta_value.setText(f'θ={deg:.0f}°')
        self.map_view.set_robot_pose(msg.pose.x, msg.pose.y, msg.pose.theta)

        # 경유점
        self.waypoint_value.setText(msg.current_waypoint_id or '--')

        # 배터리
        pct = float(msg.battery_percent)
        self.batt_donut.set_percent(pct)
        self.batt_badge_value.setText(f'{pct:.0f}%')
        color = (T.ACCENT_RED if pct <= 20
                 else T.ACCENT_AMBER if pct <= 40 else T.ACCENT_CYAN)
        self.batt_badge_value.setStyleSheet(f'font-weight: bold; color: {color};')
        self.batt_dot.set_color(color)

        self._battery_history.append(pct)
        if len(self._battery_history) > 90:
            self._battery_history.pop(0)
        self.batt_spark.push(pct)
        self.batt_caption.setText(
            f'최근 {len(self._battery_history)}개 샘플 · 현재 {pct:.0f}%')

        # 상태
        state = msg.mission_state
        if state != self._mission_state:
            self._add_timeline(
                f'{STATE_LABEL_KO.get(state, state)} ({state})',
                STATE_COLOR.get(state, T.TEXT_SUB))
            self._mission_state = state

        self.stage_value.setText(STATE_LABEL_KO.get(state, state))
        self.stage_value.setStyleSheet(
            f'font-size: 15px; font-weight: bold; '
            f'color: {STATE_COLOR.get(state, T.TEXT_MAIN)};')

        # HMI-001 5장 상태 전이표에 따른 버튼 활성/비활성
        is_ready = (state == 'READY')
        self.start_btn.setEnabled(is_ready)
        self.cancel_btn.setEnabled(state in CANCELABLE_STATES)

        self.manual_chk.setEnabled(is_ready)
        if not is_ready and self.manual_chk.isChecked():
            # 임무가 시작되면 수동모드를 강제로 끔 (Nav2와 /cmd_vel 충돌 방지)
            self.manual_chk.setChecked(False)

    def _on_map(self, grid_msg):
        self.map_view.set_map(grid_msg)
        self.statusBar().showMessage(
            f'맵 수신: {grid_msg.info.width}×{grid_msg.info.height} '
            f'@ {grid_msg.info.resolution:.3f} m/px', 4000)

    def _on_camera_frame(self, rgb_array, width, height):
        self._last_camera_time = time.time()
        # QImage 생성은 GUI 스레드인 여기서 (numpy 버퍼 수명 주의: copy() 필수)
        qimg = QImage(rgb_array.data, width, height,
                      width * 3, QImage.Format_RGB888).copy()
        self.camera_view.set_frame(qimg)

    def _on_scan(self, min_dist, valid_count):
        self._last_scan_time = time.time()
        if min_dist == min_dist:   # NaN이 아니면 (NaN은 자기 자신과 다름)
            self.lidar_row.set_state(f'최소 {min_dist * 100:.0f}cm', T.ACCENT_CYAN, 'ok')
        else:
            self.lidar_row.set_state('스캔 없음', T.ACCENT_AMBER, 'info')

    def _on_rear_range(self, meters):
        self.dist_bar.set_value(meters * 100.0)  # m -> cm

    def temp_signal_handler(self, celsius):
        self.temp_bar.set_value(celsius)
        self._check_temp_alarm(celsius)

    def _check_temp_alarm(self, celsius):
        """SR-014 온도 이상 알림. 임계치를 처음 넘은 순간에만 알람을 남긴다."""
        if celsius >= TEMP_ALARM_ON and not self._temp_alarm_on:
            self._temp_alarm_on = True
            self._add_alarm(f'온도 임계치 초과 ({celsius:.1f}°C)', T.ACCENT_RED)
            self._add_timeline(f'온도 임계치 초과 ({celsius:.1f}°C)', T.ACCENT_RED)
        elif celsius < TEMP_ALARM_OFF and self._temp_alarm_on:
            self._temp_alarm_on = False
            self._add_timeline(f'온도 정상 복귀 ({celsius:.1f}°C)', T.ACCENT_GREEN)

    def humi_signal_handler(self, percent):
        self.humi_bar.set_value(percent)

    def rear_cm_signal_handler(self, cm):
        self.dist_bar.set_value(cm)

    def _on_flame(self, detected):
        """
        불꽃감지센서 값 수신.
        감지 '시작' 순간에만 알람/타임라인을 남긴다 (계속 True로 오면 도배되니까).
        """
        self._last_flame_time = time.time()
        self._flame_on = detected

        if detected:
            self.flame_row.set_state('불꽃 감지', T.ACCENT_RED, 'rec')
            self.flame_banner.setVisible(True)
            if not self._flame_was_on:   # 방금 막 감지되기 시작한 순간
                self._add_alarm('불꽃 감지 — 즉시 확인 필요', T.ACCENT_RED)
                self._add_timeline('불꽃 감지됨', T.ACCENT_RED)
                self.statusBar().showMessage(
                    '[화재 경보] 불꽃감지센서가 불꽃을 감지했습니다', 10000)
        else:
            self.flame_row.set_state('불꽃 미감지', T.ACCENT_GREEN, 'ok')
            self.flame_banner.setVisible(False)
            if self._flame_was_on:       # 감지 상태가 방금 해제됨
                self._add_timeline('불꽃 감지 해제', T.ACCENT_GREEN)

        self._flame_was_on = detected

    def _on_person(self, detected):
        if detected:
            self.person_row.set_state('사람 감지', '#58a6ff', 'info')
        else:
            self.person_row.set_state('없음', T.ACCENT_GREEN, 'ok')

    def _on_env_data(self, temp, humi, lux):
        self.temp_bar.set_value(temp)
        self.humi_bar.set_value(humi)
        self._check_temp_alarm(temp)

    def _on_system_error(self, node_name, error_code, message, severity):
        color = (T.ACCENT_RED if severity >= 2
                 else T.ACCENT_AMBER if severity == 1 else '#58a6ff')
        self._add_alarm(f'{node_name} · {error_code}', color)
        self.statusBar().showMessage(f'[{error_code}] {message}', 6000)

    def _on_start_result(self, success, message):
        if success:
            self.statusBar().showMessage(f'임무 시작 요청 수락됨 · {message}', 4000)
            self._add_timeline(f'임무 시작 요청 · {message}', T.ACCENT_GREEN)
        else:
            QMessageBox.warning(self, '임무 시작 실패', message or '알 수 없는 이유로 실패했습니다.')
            self._add_alarm(f'임무 시작 실패: {message}', T.ACCENT_RED)

    def _on_cancel_result(self, success, message):
        if success:
            self.statusBar().showMessage(f'임무 취소됨 · {message}', 4000)
            self._add_timeline('임무 취소', T.ACCENT_AMBER)
        else:
            QMessageBox.warning(self, '임무 취소 실패', message or '알 수 없는 이유로 실패했습니다.')

    def _on_log_result(self, success, records):
        self.log_table.setRowCount(0)
        if not success:
            QMessageBox.warning(self, '로그 조회 실패', '측정 로그를 가져오지 못했습니다.')
            return
        for rec in records:
            row = self.log_table.rowCount()
            self.log_table.insertRow(row)
            stamp = datetime.fromtimestamp(
                rec.stamp.sec + rec.stamp.nanosec * 1e-9).strftime('%Y-%m-%d %H:%M')
            values = [rec.waypoint_id,
                      f'{rec.temperature:.1f}',
                      f'{rec.humidity:.0f}',
                      f'{rec.illuminance:.0f}',
                      stamp]
            for col, val in enumerate(values):
                self.log_table.setItem(row, col, QTableWidgetItem(str(val)))
        self.statusBar().showMessage(f'측정 로그 {len(records)}건 조회됨', 4000)

    def _on_service_failed(self, service_name, reason):
        self.statusBar().showMessage(f'[서비스 호출 실패] {service_name} · {reason}', 6000)
        self._add_alarm(f'{service_name} 호출 실패', T.ACCENT_AMBER)

    # ==================================================================
    # 사용자 액션
    # ==================================================================
    def _on_start_clicked(self):
        code = self.dest_combo.currentData()
        name = self.dest_combo.currentText()
        if QMessageBox.question(
                self, '임무 시작 확인', f'"{name}"(으)로 안내를 시작할까요?',
                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.ros.start_mission(code)

    def _on_cancel_clicked(self):
        if QMessageBox.question(
                self, '임무 취소 확인', '진행 중인 임무를 취소할까요?',
                QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.ros.cancel_mission()

    def _on_query_clicked(self):
        self.ros.get_log(self.log_wp_filter.text().strip(),
                         self.log_date_filter.text().strip())

    def _on_manual_toggled(self, state):
        enabled = bool(state)
        self._set_drive_buttons(enabled)
        if not enabled:
            self._drive_stop()
        else:
            self._add_timeline('수동모드 활성화 (디버그)', T.ACCENT_AMBER)

    def _set_drive_buttons(self, enabled):
        for b in (self.btn_forward, self.btn_backward, self.btn_left,
                  self.btn_right, self.btn_stop):
            b.setEnabled(enabled)

    def _drive(self, linear_x, angular_z):
        # 이중 안전장치: 체크박스 OFF거나 READY가 아니면 명령을 아예 보내지 않는다
        if not self.manual_chk.isChecked() or self._mission_state != 'READY':
            return
        self.ros.drive(linear_x, angular_z)

    def _drive_stop(self):
        self.ros.drive_stop()

    # ==================================================================
    # PM2 프로세스 제어
    # ==================================================================
    def _on_pm2_clicked(self, action):
        if self._pm2_process is not None:
            # 버튼이 비활성화돼 있어 보통 여기 안 오지만, 이중 클릭 방어용
            return

        target_value = self.pm2_target_combo.currentData()
        target_label = self.pm2_target_combo.currentText()
        is_all = (target_value == PM2_ALL_VALUE)

        label = PM2_ACTION_LABEL[action]
        confirm_text = (f'pm2로 관리 중인 프로세스 {len(PM2_PROCESSES)}개를 모두 {label}할까요?'
                        if is_all else f'"{target_label}" 프로세스를 {label}할까요?')
        if QMessageBox.question(
                self, f'PM2 {label} 확인', confirm_text,
                QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return

        self._set_pm2_buttons(False)
        self.pm2_status_label.setText(
            f'{PM2_SSH_USER}@{PM2_SSH_HOST}에서 pm2 {action} {target_value} 실행 중…')

        # (완전히 데몬까지 내리고 싶으면 action='stop' 대신 'kill'을 쓰면 되지만,
        #  그러면 재시작 버튼으로 다시 못 살리니 여기서는 stop으로 통일)
        # NO_COLOR/PM2_HOME은 로컬이 아니라 "원격 셸"에서 먹어야 하므로, ssh 뒤에
        # 붙는 트레일링 인자들에 문자 그대로 넣는다 — ssh가 이걸 공백으로 이어붙여
        # 원격 로그인 셸에 "NO_COLOR=1 PM2_HOME=... pm2 restart <대상>"으로 넘겨준다.
        remote_words = ['NO_COLOR=1', 'FORCE_COLOR=0']
        if PM2_HOME_OVERRIDE:
            remote_words.append(f'PM2_HOME={PM2_HOME_OVERRIDE}')
        remote_words += ['pm2', action, target_value]

        proc = QProcess(self)
        proc.setProgram(PM2_RUN_PREFIX[0])
        proc.setArguments(PM2_RUN_PREFIX[1:] + remote_words)
        proc.setProcessChannelMode(QProcess.MergedChannels)
        proc.finished.connect(
            lambda code, status: self._on_pm2_finished(action, target_label, code))
        proc.errorOccurred.connect(
            lambda err: self._on_pm2_error(action, target_label, err))
        self._pm2_process = proc
        proc.start()

        # BatchMode+ConnectTimeout으로 대부분 막히지만, 방화벽이 SYN만 흘려버리는
        # 경우처럼 진짜 안 끝나는 상황에 대비한 마지막 안전장치.
        QTimer.singleShot(
            PM2_TIMEOUT_MS, lambda p=proc: self._on_pm2_timeout(p, target_label))

    def _on_pm2_finished(self, action, target_label, exit_code):
        label = PM2_ACTION_LABEL[action]
        output = ''
        if self._pm2_process is not None:
            raw = bytes(self._pm2_process.readAllStandardOutput()) \
                .decode(errors='replace')
            output = _strip_ansi(raw).strip()
        self._pm2_process = None
        self._set_pm2_buttons(True)

        if exit_code == 0:
            msg = f'{target_label} {label} 완료'
            self.pm2_status_label.setText(msg)
            self._add_timeline(f'pm2 {msg}', T.ACCENT_GREEN)
            self.statusBar().showMessage(f'pm2 {msg}', 4000)
        else:
            detail = output.splitlines()[-1] if output else f'종료 코드 {exit_code}'
            msg = f'{target_label} {label} 실패: {detail}'
            self.pm2_status_label.setText(msg)
            self._add_alarm(f'pm2 {msg}', T.ACCENT_RED)
            self._add_timeline(f'pm2 {target_label} {label} 실패', T.ACCENT_RED)

    def _on_pm2_error(self, action, target_label, error):
        # start() 자체가 실패한 경우(예: pm2 명령을 찾을 수 없음)만 여기서 처리한다.
        # 그 외(Crashed 등)는 finished 시그널이 뒤이어 오므로 거기서 한 번만 처리.
        if error != QProcess.FailedToStart:
            return
        label = PM2_ACTION_LABEL[action]
        reason = self._pm2_process.errorString() if self._pm2_process else '알 수 없는 오류'
        self._pm2_process = None
        self._set_pm2_buttons(True)
        msg = f'{target_label} {label} 실행 실패: {reason}'
        self.pm2_status_label.setText(msg)
        self._add_alarm(f'pm2 {msg}', T.ACCENT_RED)

    def _on_pm2_timeout(self, proc, target_label):
        if self._pm2_process is not proc:
            return  # 이미 정상적으로 끝났음 (finished/error 핸들러가 처리함)
        # kill()이 뒤늦게 finished를 발생시켜 아래에서 설정하는 타임아웃 메시지를
        # 덮어써버리는 걸 막기 위해, 죽이기 전에 시그널 연결부터 끊는다.
        try:
            proc.finished.disconnect()
            proc.errorOccurred.disconnect()
        except TypeError:
            pass
        proc.kill()
        self._pm2_process = None
        self._set_pm2_buttons(True)
        msg = (f'{target_label}: {PM2_TIMEOUT_MS // 1000}초 안에 응답이 없어 중단했습니다 — '
               f'{PM2_SSH_USER}@{PM2_SSH_HOST}에 SSH로 접속되는지 확인하세요.')
        self.pm2_status_label.setText(msg)
        self._add_alarm(msg, T.ACCENT_RED)

    def _set_pm2_buttons(self, enabled):
        for b in (self.pm2_start_btn, self.pm2_stop_btn, self.pm2_restart_btn):
            b.setEnabled(enabled)

    # ==================================================================
    # 알람 / 타임라인
    # ==================================================================
    def _add_alarm(self, text, color):
        if self.alarm_empty is not None:
            self.alarm_empty.setParent(None)
            self.alarm_empty = None

        row = EventRow(color, text, datetime.now().strftime('%H:%M'))
        self.alarm_layout.insertWidget(0, row)

        while self.alarm_layout.count() > MAX_ALARMS:
            item = self.alarm_layout.takeAt(self.alarm_layout.count() - 1)
            if item.widget():
                item.widget().setParent(None)

    def _add_timeline(self, title, color):
        row = TimelineRow(color, title, datetime.now().strftime('%H:%M:%S'))
        # 마지막은 addStretch()라 그 앞에 삽입
        self.timeline_layout.insertWidget(self.timeline_layout.count() - 1, row)

        while self.timeline_layout.count() - 1 > MAX_TIMELINE:
            item = self.timeline_layout.takeAt(0)
            if item.widget():
                item.widget().setParent(None)

        # 최신 항목이 보이도록 스크롤 맨 아래로
        def _scroll_to_bottom():
            try:
                bar = self.timeline_scroll.verticalScrollBar()
                bar.setValue(bar.maximum())
            except RuntimeError:
                # 이 50ms 지연 사이에 창이 닫혀서 위젯이 이미 삭제된 경우
                # ("wrapped C/C++ object has been deleted") - 조용히 무시
                # (2026-08-18 코드 리뷰에서 발견).
                pass
        QTimer.singleShot(50, _scroll_to_bottom)

    # ==================================================================
    # 1초 타이머: 시계 + 연결 감시
    # ==================================================================
    def _tick(self):
        now = time.time()
        self.clock_label.setText(datetime.now().strftime('%H:%M:%S'))

        # --- 로봇 상태 연결 감시 ---
        if self._last_state_time == 0:
            self.conn_dot.set_color(T.TEXT_DIM)
            self.conn_sub.setText(
                f'연결 대기 중 · ROS_DOMAIN_ID={self.ros_domain_id}')
        elif now - self._last_state_time > STATE_TIMEOUT_SEC:
            self.conn_dot.set_color(T.ACCENT_RED)
            # SR-023: Wi-Fi가 끊겨도 로봇 내부 기능은 계속 동작한다
            self.conn_sub.setText(
                '상태확인 불가 · 로봇은 내부적으로 임무를 계속 수행 중입니다')
            self.start_btn.setEnabled(False)
            self.cancel_btn.setEnabled(False)
            self.manual_chk.setEnabled(False)
        else:
            self.conn_dot.set_color(T.ACCENT_GREEN)
            self.conn_sub.setText(
                f'연결됨 · ROS_DOMAIN_ID={self.ros_domain_id} · '
                f'상태 {STATE_LABEL_KO.get(self._mission_state, self._mission_state)}')

        # --- 카메라 감시 ---
        if self._last_camera_time == 0 or now - self._last_camera_time > CAMERA_TIMEOUT_SEC:
            self.cam_pill.setText('● 신호 없음')
            self.camera_view.clear_frame()
        else:
            self.cam_pill.setText('● REC')

        # --- LiDAR 감시 ---
        if self._last_scan_time == 0 or now - self._last_scan_time > SCAN_TIMEOUT_SEC:
            self.lidar_row.set_state('미수신', T.TEXT_DIM, 'info')

        # --- 불꽃감지센서 감시 ---
        # SR-024: 센서 데이터 2초 이상 미수신이면 해당 기능 비활성 표시.
        # 화재 감지는 안전 항목이라 '미수신'을 초록(정상)으로 보이게 두면 위험하므로
        # 회색 '미수신'으로 명확히 구분한다.
        if self._last_flame_time == 0 or now - self._last_flame_time > SENSOR_TIMEOUT_SEC:
            self.flame_row.set_state('미수신', T.TEXT_DIM, 'info')
            self.flame_banner.setVisible(False)
            self._flame_on = False
            self._flame_was_on = False

        # 감지 중이면 경고 띠를 깜빡이게 한다
        self.flame_banner.blink()

        # --- 맵 감시 ---
        self.map_pill.setText('LIVE' if self._last_state_time > 0 else 'OFFLINE')

    # ==================================================================
    def closeEvent(self, event):
        # DDS-001 2.6 종료조건: 창을 닫으면 ROS 노드를 정상 종료
        self.ros.drive_stop()
        self.ros.stop()
        event.accept()
