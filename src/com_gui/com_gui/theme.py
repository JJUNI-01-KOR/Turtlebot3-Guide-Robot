"""
theme.py
--------
다크 대시보드 색상 팔레트 + 전체 스타일시트(QSS).

QSS는 웹의 CSS랑 문법이 거의 똑같아. 위젯에 setObjectName('...')으로
이름을 붙여두면 여기서 '#이름'으로 콕 집어서 스타일을 줄 수 있어.
"""

# ---- 색상 팔레트 (한 곳에서만 고치면 전체가 바뀌도록 상수로 뺌) ----
BG_APP = '#0d1117'        # 앱 전체 배경 (제일 어두움)
BG_CARD = '#161b22'       # 카드(패널) 배경
BG_INPUT = '#0d1117'      # 입력창 배경
BG_HOVER = '#21262d'      # 마우스 올렸을 때
BORDER = '#30363d'        # 카드 테두리

TEXT_MAIN = '#e6edf3'     # 기본 글자
TEXT_SUB = '#8b949e'      # 보조 글자(라벨, 캡션)
TEXT_DIM = '#6e7681'      # 더 흐린 글자(플레이스홀더)

ACCENT_GREEN = '#2ea043'   # 시작/정상
ACCENT_RED = '#da3633'     # 취소/오류
ACCENT_BLUE = '#1f6feb'    # 조회/정보
ACCENT_AMBER = '#d29922'   # 수동주행/경고
ACCENT_CYAN = '#39d353'    # 배터리 도넛
ACCENT_ORANGE = '#f0883e'  # 온도 바

# 상태머신 상태별 색 (SDD-001 3.1 상태 정의 기준)
STATE_COLOR = {
    'READY': ACCENT_GREEN,
    'GUIDING': '#58a6ff', 'RETURNING': '#58a6ff', 'DOCKING': '#58a6ff',
    'ARRIVED': '#39d353', 'PERSON_WAIT': '#39d353',
    'DOCKED': ACCENT_GREEN,
    'PAUSED': ACCENT_AMBER, 'OBSTACLE_STOP': ACCENT_AMBER,
    'EMERGENCY_STOP': ACCENT_RED, 'ERROR': ACCENT_RED,
    'SYSTEM_INIT': TEXT_SUB, 'LOCALIZING': TEXT_SUB, 'SHUTDOWN': TEXT_SUB,
}

# 상태머신 상태 -> 사람이 읽기 쉬운 한글 (대시보드 '현재 단계'에 표시)
STATE_LABEL_KO = {
    'SYSTEM_INIT': '시스템 초기화',
    'LOCALIZING': '위치 추정 중',
    'READY': '대기 중',
    'GUIDING': '주행 중',
    'OBSTACLE_STOP': '장애물 정지',
    'ARRIVED': '목적지 도착',
    'PERSON_WAIT': '사람 대기',
    'RETURNING': '복귀 중',
    'DOCKING': '도킹 중',
    'DOCKED': '충전 중',
    'PAUSED': '일시 정지',
    'EMERGENCY_STOP': '긴급 정지',
    'ERROR': '오류',
    'SHUTDOWN': '종료',
}


STYLESHEET = f"""
QMainWindow, QScrollArea, QScrollArea > QWidget > QWidget {{
    background-color: {BG_APP};
}}
QWidget {{
    color: {TEXT_MAIN};
    font-family: "Noto Sans KR", "Malgun Gothic", "NanumGothic", sans-serif;
    font-size: 12px;
}}

/* ---------- 메시지박스(확인창) ---------- */
/* QMessageBox는 별도 최상위 창이라 QMainWindow 배경 규칙을 안 타서 따로 지정 */
QMessageBox {{
    background-color: {BG_CARD};
}}
QMessageBox QLabel {{
    color: {TEXT_MAIN};
    background-color: transparent;
}}
QMessageBox QPushButton {{
    background-color: {BG_HOVER};
    color: {TEXT_MAIN};
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 6px 16px;
    min-width: 64px;
}}
QMessageBox QPushButton:hover {{
    background-color: #30363d;
}}
QMessageBox QPushButton:default {{
    background-color: {ACCENT_GREEN};
    color: white;
    border: none;
}}
QMessageBox QPushButton:default:hover {{
    background-color: #3fb950;
}}

/* ---------- 카드(패널) ---------- */
QFrame#card {{
    background-color: {BG_CARD};
    border: 1px solid {BORDER};
    border-radius: 12px;
}}
QLabel#cardTitle {{
    color: {TEXT_MAIN};
    font-size: 13px;
    font-weight: bold;
}}
QLabel#caption {{
    color: {TEXT_SUB};
    font-size: 11px;
}}
QLabel#valueBig {{
    color: {TEXT_MAIN};
    font-size: 15px;
    font-weight: bold;
}}
QLabel#mono {{
    font-family: "D2Coding", "Consolas", "DejaVu Sans Mono", monospace;
}}

/* ---------- 헤더 ---------- */
QLabel#appTitle {{
    font-size: 19px;
    font-weight: bold;
    color: {TEXT_MAIN};
}}
QLabel#clock {{
    font-family: "D2Coding", "Consolas", "DejaVu Sans Mono", monospace;
    font-size: 20px;
    font-weight: bold;
    color: {TEXT_MAIN};
}}
QFrame#badge {{
    background-color: {BG_HOVER};
    border: 1px solid {BORDER};
    border-radius: 8px;
}}
QLabel#pill {{
    background-color: rgba(31,111,235,0.18);
    color: #58a6ff;
    border-radius: 6px;
    padding: 3px 9px;
    font-size: 10px;
    font-weight: bold;
}}
QLabel#pillRec {{
    background-color: rgba(218,54,51,0.18);
    color: #ff7b72;
    border-radius: 6px;
    padding: 3px 9px;
    font-size: 10px;
    font-weight: bold;
}}
QLabel#pillOk {{
    background-color: rgba(46,160,67,0.18);
    color: {ACCENT_CYAN};
    border-radius: 6px;
    padding: 3px 9px;
    font-size: 10px;
    font-weight: bold;
}}

/* ---------- 버튼 ---------- */
QPushButton {{
    background-color: {BG_HOVER};
    border: 1px solid {BORDER};
    border-radius: 7px;
    padding: 8px 14px;
    color: {TEXT_MAIN};
}}
QPushButton:hover {{ background-color: #30363d; }}
QPushButton:disabled {{ background-color: #161b22; color: {TEXT_DIM}; border-color: #21262d; }}

QPushButton#startBtn {{
    background-color: {ACCENT_GREEN}; border: none;
    color: white; font-weight: bold; padding: 9px 18px;
}}
QPushButton#startBtn:hover {{ background-color: #3fb950; }}
QPushButton#startBtn:disabled {{ background-color: #1b4721; color: #6e7681; }}

QPushButton#cancelBtn {{
    background-color: {ACCENT_RED}; border: none;
    color: white; font-weight: bold; padding: 9px 18px;
}}
QPushButton#cancelBtn:hover {{ background-color: #f85149; }}
QPushButton#cancelBtn:disabled {{ background-color: #4a1d1c; color: #6e7681; }}

QPushButton#queryBtn {{
    background-color: {ACCENT_BLUE}; border: none;
    color: white; font-weight: bold; padding: 9px 14px;
}}
QPushButton#queryBtn:hover {{ background-color: #388bfd; }}

QPushButton#driveBtn {{
    background-color: rgba(210,153,34,0.15);
    border: 1px solid {ACCENT_AMBER};
    color: {ACCENT_AMBER};
    font-weight: bold;
    min-width: 70px; min-height: 34px;
    font-size: 13px;
}}
QPushButton#driveBtn:hover {{ background-color: rgba(210,153,34,0.30); }}
QPushButton#driveBtn:pressed {{ background-color: {ACCENT_AMBER}; color: #0d1117; }}
QPushButton#driveBtn:disabled {{
    background-color: transparent; border-color: #30363d; color: {TEXT_DIM};
}}

QPushButton#stopBtn {{
    background-color: {BG_HOVER};
    border: 1px solid {BORDER};
    color: {TEXT_MAIN};
    font-weight: bold;
    min-width: 70px; min-height: 34px;
    font-size: 13px;
}}
QPushButton#stopBtn:hover {{ background-color: #30363d; }}
QPushButton#stopBtn:disabled {{ color: {TEXT_DIM}; }}

/* ---------- PM2 프로세스 제어 ---------- */
/* 시작=초록 / 종료=빨강 / 재시작=주황, 세 버튼을 색으로 바로 구분되게 함 */
QPushButton#pm2StartBtn {{
    background-color: {ACCENT_GREEN}; border: none;
    color: white; font-weight: bold;
}}
QPushButton#pm2StartBtn:hover {{ background-color: #3fb950; }}
QPushButton#pm2StartBtn:disabled {{ background-color: #1b4721; color: #6e7681; }}

QPushButton#pm2StopBtn {{
    background-color: {ACCENT_RED}; border: none;
    color: white; font-weight: bold;
}}
QPushButton#pm2StopBtn:hover {{ background-color: #f85149; }}
QPushButton#pm2StopBtn:disabled {{ background-color: #4a1d1c; color: #6e7681; }}

QPushButton#pm2RestartBtn {{
    background-color: {ACCENT_AMBER}; border: none;
    color: #0d1117; font-weight: bold;
}}
QPushButton#pm2RestartBtn:hover {{ background-color: #e3b341; }}
QPushButton#pm2RestartBtn:disabled {{ background-color: #4a3b13; color: #6e7681; }}

/* ---------- 입력 ---------- */
QComboBox, QLineEdit {{
    background-color: {BG_INPUT};
    border: 1px solid {BORDER};
    border-radius: 7px;
    padding: 8px 10px;
    color: {TEXT_MAIN};
    selection-background-color: {ACCENT_BLUE};
}}
QComboBox:focus, QLineEdit:focus {{ border: 1px solid {ACCENT_BLUE}; }}
QComboBox::drop-down {{ border: none; width: 22px; }}
QComboBox QAbstractItemView {{
    background-color: {BG_CARD};
    border: 1px solid {BORDER};
    selection-background-color: {ACCENT_BLUE};
    color: {TEXT_MAIN};
    outline: none;
}}
QLineEdit::placeholder {{ color: {TEXT_DIM}; }}

QCheckBox {{ color: {TEXT_MAIN}; font-weight: bold; spacing: 7px; }}
QCheckBox::indicator {{
    width: 15px; height: 15px;
    border: 1px solid {BORDER}; border-radius: 4px;
    background: {BG_INPUT};
}}
QCheckBox::indicator:checked {{
    background: {ACCENT_AMBER}; border-color: {ACCENT_AMBER};
}}
QCheckBox::indicator:disabled {{ border-color: #21262d; background: #161b22; }}

/* ---------- 표 ---------- */
QTableWidget {{
    background-color: transparent;
    border: none;
    gridline-color: transparent;
    color: {TEXT_MAIN};
}}
QTableWidget::item {{
    border-bottom: 1px solid #21262d;
    padding: 7px 4px;
}}
QTableWidget::item:selected {{ background-color: rgba(31,111,235,0.25); }}
QHeaderView::section {{
    background-color: transparent;
    color: {TEXT_SUB};
    border: none;
    border-bottom: 1px solid {BORDER};
    padding: 7px 4px;
    font-weight: bold;
    font-size: 11px;
}}

/* ---------- 스크롤바 ---------- */
QScrollBar:vertical {{
    background: transparent; width: 9px; margin: 0;
}}
QScrollBar::handle:vertical {{
    background: #30363d; border-radius: 4px; min-height: 22px;
}}
QScrollBar::handle:vertical:hover {{ background: #484f58; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}
QScrollBar::add-page, QScrollBar::sub-page {{ background: transparent; }}

/* ---------- 리스트(알람/타임라인) ---------- */
QListWidget {{
    background: transparent; border: none; outline: none;
}}
QListWidget::item {{ border: none; padding: 0; }}
QListWidget::item:selected {{ background: transparent; }}

/* ---------- 상태바 ---------- */
QStatusBar {{
    background: {BG_CARD};
    color: {TEXT_SUB};
    border-top: 1px solid {BORDER};
}}
QToolTip {{
    background-color: {BG_CARD}; color: {TEXT_MAIN};
    border: 1px solid {BORDER}; padding: 5px;
}}
"""
