"""
widgets.py
----------
기본 PyQt 위젯으로는 안 되는 것들(도넛 게이지, 스파크라인, 맵 뷰 등)을
paintEvent()에서 직접 그려서 만든 커스텀 위젯 모음.

paintEvent()는 "이 위젯을 화면에 그려야 할 때" Qt가 자동으로 불러주는 함수야.
그 안에서 QPainter로 선/원/사각형을 직접 그리면 원하는 모양을 만들 수 있어.
update()를 호출하면 "다시 그려줘"라고 Qt에 요청하는 거고.
"""

import math

import numpy as np

from PyQt5.QtWidgets import (
    QWidget, QFrame, QLabel, QVBoxLayout, QHBoxLayout, QSizePolicy
)
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QImage, QPixmap, QFont, QPainterPath,
    QLinearGradient
)
from PyQt5.QtCore import Qt, QRectF, QPointF, QSize

from . import theme as T


# ======================================================================
# 카드 (모든 패널의 공통 껍데기)
# ======================================================================
class Card(QFrame):
    """제목 + 우측 배지 + 본문 영역을 가진 둥근 카드 패널."""

    def __init__(self, title: str, badge: QWidget = None, parent=None):
        super().__init__(parent)
        self.setObjectName('card')

        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 14, 16, 14)
        outer.setSpacing(11)

        header = QHBoxLayout()
        header.setSpacing(8)
        self.title_label = QLabel(title)
        self.title_label.setObjectName('cardTitle')
        header.addWidget(self.title_label)
        header.addStretch()
        if badge is not None:
            header.addWidget(badge)
        outer.addLayout(header)

        # 본문은 body 레이아웃에 넣는다
        self.body = QVBoxLayout()
        self.body.setSpacing(9)
        outer.addLayout(self.body)


def make_pill(text: str, kind: str = 'info') -> QLabel:
    """'LIVE', 'REC' 같은 작은 알약 모양 배지."""
    label = QLabel(text)
    label.setObjectName({'info': 'pill', 'rec': 'pillRec', 'ok': 'pillOk'}[kind])
    return label


class Dot(QWidget):
    """상태 표시용 작은 원. set_color()로 색을 바꾼다."""

    def __init__(self, color: str = T.ACCENT_GREEN, size: int = 9, parent=None):
        super().__init__(parent)
        self._color = QColor(color)
        self._size = size
        self.setFixedSize(size, size)

    def set_color(self, color: str):
        self._color = QColor(color)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(self._color))
        p.drawEllipse(0, 0, self._size, self._size)


# ======================================================================
# 배터리 도넛 게이지
# ======================================================================
class BatteryDonut(QWidget):
    """가운데 퍼센트 숫자가 있는 원형 게이지."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._percent = 0.0
        self.setFixedSize(88, 88)

    def set_percent(self, percent: float):
        self._percent = max(0.0, min(100.0, percent))
        self.update()

    def _color(self):
        # 잔량에 따라 색이 바뀜 (20% 이하는 SR-022 복귀 임계값이라 빨강)
        if self._percent <= 20:
            return QColor(T.ACCENT_RED)
        if self._percent <= 40:
            return QColor(T.ACCENT_AMBER)
        return QColor(T.ACCENT_CYAN)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        thickness = 11
        rect = QRectF(thickness / 2, thickness / 2,
                      self.width() - thickness, self.height() - thickness)

        # 배경 링 (남은 부분)
        pen_bg = QPen(QColor('#21262d'), thickness, Qt.SolidLine, Qt.RoundCap)
        p.setPen(pen_bg)
        p.drawArc(rect, 0, 360 * 16)

        # 채워진 링 — 12시 방향(90도)에서 시계방향으로
        if self._percent > 0:
            pen_fg = QPen(self._color(), thickness, Qt.SolidLine, Qt.RoundCap)
            p.setPen(pen_fg)
            span = int(-self._percent / 100.0 * 360 * 16)
            p.drawArc(rect, 90 * 16, span)

        # 가운데 숫자
        p.setPen(QColor(T.TEXT_MAIN))
        font = QFont(self.font())
        font.setPointSize(15)
        font.setBold(True)
        p.setFont(font)
        p.drawText(self.rect(), Qt.AlignCenter, f'{self._percent:.0f}%')


class Sparkline(QWidget):
    """최근 값 추이를 보여주는 작은 꺾은선 그래프."""

    def __init__(self, max_points: int = 60, color: str = T.ACCENT_CYAN, parent=None):
        super().__init__(parent)
        self._values = []
        self._max_points = max_points
        self._color = QColor(color)
        self.setMinimumHeight(46)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

    def push(self, value: float):
        self._values.append(value)
        if len(self._values) > self._max_points:
            self._values.pop(0)
        self.update()

    def paintEvent(self, event):
        if len(self._values) < 2:
            return
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        pad = 4
        vmin, vmax = min(self._values), max(self._values)
        # 값이 전부 같으면 0으로 나누게 되니 방어
        vrange = (vmax - vmin) if (vmax - vmin) > 0.5 else 1.0

        points = []
        n = len(self._values)
        for i, v in enumerate(self._values):
            x = pad + (w - 2 * pad) * i / (n - 1)
            y = h - pad - (h - 2 * pad) * (v - vmin) / vrange
            points.append(QPointF(x, y))

        # 선 아래를 옅게 채워서 면적 그래프처럼
        area = QPainterPath()
        area.moveTo(points[0].x(), h)
        for pt in points:
            area.lineTo(pt)
        area.lineTo(points[-1].x(), h)
        area.closeSubpath()
        grad = QLinearGradient(0, 0, 0, h)
        c = QColor(self._color)
        c.setAlpha(70)
        grad.setColorAt(0, c)
        c2 = QColor(self._color)
        c2.setAlpha(0)
        grad.setColorAt(1, c2)
        p.fillPath(area, QBrush(grad))

        # 선
        line = QPainterPath()
        line.moveTo(points[0])
        for pt in points[1:]:
            line.lineTo(pt)
        p.setPen(QPen(self._color, 1.8))
        p.drawPath(line)


# ======================================================================
# 센서 게이지 바 (온도/습도/거리)
# ======================================================================
class SensorBar(QWidget):
    """라벨 + 값 + 진행바가 한 세트인 위젯."""

    def __init__(self, label: str, unit: str, vmin: float, vmax: float,
                 color: str, parent=None):
        super().__init__(parent)
        self._label = label
        self._unit = unit
        self._vmin = vmin
        self._vmax = vmax
        self._color = QColor(color)
        self._value = None  # None = 아직 값 없음
        self.setFixedHeight(38)

    def set_value(self, value):
        self._value = value
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width()

        # 윗줄: 라벨(좌) / 값(우)
        font_label = QFont(self.font())
        font_label.setPointSize(9)
        p.setFont(font_label)
        p.setPen(QColor(T.TEXT_SUB))
        p.drawText(QRectF(0, 0, w * 0.6, 16), Qt.AlignLeft | Qt.AlignVCenter, self._label)

        font_val = QFont(self.font())
        font_val.setPointSize(11)
        font_val.setBold(True)
        p.setFont(font_val)
        p.setPen(QColor(T.TEXT_MAIN))
        text = '--' if self._value is None else f'{self._value:.1f}{self._unit}'
        p.drawText(QRectF(w * 0.4, 0, w * 0.6, 16), Qt.AlignRight | Qt.AlignVCenter, text)

        # 아랫줄: 진행바
        bar_y = 26
        bar_h = 5
        p.setPen(Qt.NoPen)
        p.setBrush(QColor('#21262d'))
        p.drawRoundedRect(QRectF(0, bar_y, w, bar_h), 2.5, 2.5)

        if self._value is not None:
            ratio = (self._value - self._vmin) / max(self._vmax - self._vmin, 1e-6)
            ratio = max(0.0, min(1.0, ratio))
            if ratio > 0:
                p.setBrush(self._color)
                p.drawRoundedRect(QRectF(0, bar_y, w * ratio, bar_h), 2.5, 2.5)


# ======================================================================
# SLAM 맵 뷰
# ======================================================================
class MapView(QWidget):
    """
    /map (nav_msgs/OccupancyGrid) 을 이미지로 그리고,
    그 위에 로봇 현재 위치와 목적지/경유점을 점으로 찍는다.
    맵이 아직 없으면 빗금 플레이스홀더를 보여준다.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(225)
        self._map_pixmap = None
        self._map_info = None      # (resolution, origin_x, origin_y, width, height)
        self._robot_pose = None    # (x, y, theta) — map 좌표계 (m)
        self._markers = []         # [(x, y, color), ...]

    def set_map(self, occupancy_grid):
        """OccupancyGrid 메시지를 받아 흑백 이미지로 변환해 저장."""
        info = occupancy_grid.info
        w, h = info.width, info.height
        if w == 0 or h == 0:
            return

        # 2026-08-18 코드 리뷰: 예전엔 w*h개 픽셀을 파이썬 for문 두 겹으로 하나씩
        # QImage.setPixel()에 찍었다. 캠퍼스 규모 맵(가로세로 1000+ 셀)이면 이게
        # GUI 스레드에서 몇 초씩 걸려서, 리로컬라이즈 등으로 /map이 다시 올 때마다
        # 대시보드 전체가 멈췄다. numpy로 색상 매핑을 한 번에 벡터화하고 QImage를
        # 버퍼에서 바로 만든다(ros_worker.py도 카메라 프레임을 같은 방식으로 처리함).
        grid = np.asarray(occupancy_grid.data, dtype=np.int16).reshape(h, w)
        grid = np.flipud(grid)  # ROS 맵은 아래에서 위로 쌓이므로 위아래 뒤집기

        rgb = np.full((h, w), 0xff1c2128, dtype=np.uint32)   # 미탐색 영역
        rgb[(grid >= 0) & (grid < 25)] = 0xff2d333b            # 빈 공간(주행 가능)
        rgb[grid >= 25] = 0xff8b949e                            # 장애물/벽

        # QImage가 버퍼를 참조만 하므로, numpy 배열이 먼저 GC되지 않게 인스턴스에
        # 붙잡아둔다(QPixmap.fromImage가 바로 복사하긴 하지만 방어적으로 유지).
        self._map_buffer = np.ascontiguousarray(rgb)
        img = QImage(self._map_buffer.data, w, h, QImage.Format_RGB32)

        self._map_pixmap = QPixmap.fromImage(img)
        self._map_info = (info.resolution, info.origin.position.x,
                          info.origin.position.y, w, h)
        self.update()

    def set_robot_pose(self, x: float, y: float, theta: float):
        self._robot_pose = (x, y, theta)
        self.update()

    def set_markers(self, markers):
        """markers: [(x, y, '#rrggbb'), ...] — 경유점/목적지 표시용"""
        self._markers = markers
        self.update()

    def _world_to_screen(self, wx, wy, draw_rect):
        """map 좌표(m) -> 화면 픽셀 좌표로 변환."""
        if self._map_info is None:
            return None
        res, ox, oy, mw, mh = self._map_info
        # 맵 픽셀 좌표
        px = (wx - ox) / res
        py = mh - (wy - oy) / res
        # 화면 좌표 (draw_rect 안에 맞춰 스케일)
        sx = draw_rect.x() + px / mw * draw_rect.width()
        sy = draw_rect.y() + py / mh * draw_rect.height()
        return QPointF(sx, sy)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)

        # 테두리
        p.setPen(QPen(QColor(T.BORDER), 1))
        p.setBrush(QColor('#0d1117'))
        p.drawRoundedRect(rect, 8, 8)

        if self._map_pixmap is None:
            self._draw_placeholder(p, rect, '맵 데이터 (SLAM occupancy grid) 표시 영역')
            return

        # 맵을 비율 유지하며 중앙에 배치
        inner = rect.adjusted(6, 6, -6, -6)
        scaled = self._map_pixmap.scaled(
            int(inner.width()), int(inner.height()),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        draw_rect = QRectF(
            inner.x() + (inner.width() - scaled.width()) / 2,
            inner.y() + (inner.height() - scaled.height()) / 2,
            scaled.width(), scaled.height())
        p.drawPixmap(draw_rect.topLeft(), scaled)

        # 경유점/목적지 마커
        for (mx, my, color) in self._markers:
            pt = self._world_to_screen(mx, my, draw_rect)
            if pt is None:
                continue
            p.setPen(Qt.NoPen)
            p.setBrush(QColor(color))
            p.drawEllipse(pt, 3.5, 3.5)

        # 로봇 위치 (파란 원 + 진행 방향 삼각형)
        if self._robot_pose is not None:
            rx, ry, rtheta = self._robot_pose
            pt = self._world_to_screen(rx, ry, draw_rect)
            if pt is not None:
                glow = QColor('#58a6ff')
                glow.setAlpha(60)
                p.setBrush(glow)
                p.setPen(Qt.NoPen)
                p.drawEllipse(pt, 11, 11)

                p.setBrush(QColor('#58a6ff'))
                p.drawEllipse(pt, 5.5, 5.5)

                # 방향 표시 선
                p.setPen(QPen(QColor('#58a6ff'), 2))
                end = QPointF(pt.x() + 15 * math.cos(-rtheta),
                              pt.y() + 15 * math.sin(-rtheta))
                p.drawLine(pt, end)

    def _draw_placeholder(self, p, rect, text):
        """맵/영상이 없을 때 보여주는 빗금 배경."""
        inner = rect.adjusted(6, 6, -6, -6)
        p.save()
        p.setClipRect(inner)
        p.setPen(QPen(QColor('#1c2128'), 7))
        step = 16
        x = int(inner.left() - inner.height())
        while x < inner.right() + inner.height():
            p.drawLine(x, int(inner.bottom()),
                       int(x + inner.height()), int(inner.top()))
            x += step
        p.restore()

        p.setPen(QColor(T.TEXT_DIM))
        p.drawText(inner, Qt.AlignCenter, text)


# ======================================================================
# 카메라 뷰
# ======================================================================
class CameraView(QWidget):
    """/camera/image_raw 프레임을 그대로 그려주는 위젯."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(195)
        self._pixmap = None

    def set_frame(self, qimage: QImage):
        self._pixmap = QPixmap.fromImage(qimage)
        self.update()

    def clear_frame(self):
        self._pixmap = None
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)

        p.setPen(QPen(QColor(T.BORDER), 1))
        p.setBrush(QColor('#0d1117'))
        p.drawRoundedRect(rect, 8, 8)

        if self._pixmap is None:
            MapView._draw_placeholder(self, p, rect, '카메라 피드 (video stream)')
            return

        inner = rect.adjusted(6, 6, -6, -6)
        scaled = self._pixmap.scaled(
            int(inner.width()), int(inner.height()),
            Qt.KeepAspectRatio, Qt.SmoothTransformation)
        p.drawPixmap(
            QPointF(inner.x() + (inner.width() - scaled.width()) / 2,
                    inner.y() + (inner.height() - scaled.height()) / 2),
            scaled)


# ======================================================================
# 알람 / 타임라인 항목
# ======================================================================
class StatusRow(QWidget):
    """
    점 + 라벨 + 우측 알약배지가 한 줄인 위젯.
    불꽃 감지, LiDAR 상태, 사람 감지처럼 'ON/OFF/미수신' 3상태를 보여줄 때 씀.
    """

    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 3, 0, 3)
        layout.setSpacing(9)

        self.dot = Dot(T.TEXT_DIM, 9)
        layout.addWidget(self.dot)
        layout.addWidget(QLabel(label))
        layout.addStretch()

        self.badge = QLabel('미수신')
        self.badge.setObjectName('pill')
        layout.addWidget(self.badge)

    def set_state(self, text: str, dot_color: str, pill_kind: str = 'info'):
        """
        text      : 배지에 표시할 글자
        dot_color : 왼쪽 점 색
        pill_kind : 'info'(파랑) | 'ok'(초록) | 'rec'(빨강)
        """
        self.dot.set_color(dot_color)
        self.badge.setText(text)
        # objectName을 바꾼 뒤에는 스타일을 다시 적용(unpolish/polish)해야 반영됨
        self.badge.setObjectName(
            {'info': 'pill', 'ok': 'pillOk', 'rec': 'pillRec'}[pill_kind])
        self.badge.style().unpolish(self.badge)
        self.badge.style().polish(self.badge)


class AlertBanner(QFrame):
    """불꽃 감지처럼 즉시 눈에 띄어야 하는 경고를 카드 상단에 크게 띄우는 띠."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setVisible(False)
        self._blink_on = True

        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 9, 12, 9)
        layout.setSpacing(9)

        # 이모지는 폰트에 따라 두부(□)로 깨질 수 있어 순수 문자로 아이콘을 만든다
        self.icon = QLabel('!')
        self.icon.setFixedSize(22, 22)
        self.icon.setAlignment(Qt.AlignCenter)
        self.icon.setStyleSheet(
            'background-color: #ffffff; color: #da3633; border-radius: 11px;'
            'font-weight: bold; font-size: 14px;')
        layout.addWidget(self.icon)

        self.text = QLabel('불꽃 감지됨 — 즉시 확인 필요')
        self.text.setStyleSheet('font-weight: bold; font-size: 13px; color: #ffffff;')
        layout.addWidget(self.text)
        layout.addStretch()

        self._apply(True)

    def _apply(self, bright: bool):
        bg = '#da3633' if bright else '#7d1f1d'
        self.setStyleSheet(
            f'background-color: {bg}; border: 1px solid #f85149; border-radius: 8px;')

    def blink(self):
        """1초 타이머에서 호출해 배경색을 번갈아 바꿔 깜빡이게 한다."""
        if not self.isVisible():
            return
        self._blink_on = not self._blink_on
        self._apply(self._blink_on)


class EventRow(QWidget):
    """점 + 내용 + 시각 한 줄. 알람 카드에서 사용."""

    def __init__(self, color: str, text: str, time_text: str, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)
        layout.setSpacing(9)

        layout.addWidget(Dot(color, 7))

        msg = QLabel(text)
        msg.setWordWrap(True)
        layout.addWidget(msg, 1)

        ts = QLabel(time_text)
        ts.setObjectName('caption')
        layout.addWidget(ts)


class TimelineRow(QWidget):
    """점 + 제목/시각 2줄. 임무 이력 타임라인에서 사용."""

    def __init__(self, color: str, title: str, time_text: str, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 6, 0, 6)
        layout.setSpacing(9)

        dot_col = QVBoxLayout()
        dot_col.setContentsMargins(0, 3, 0, 0)
        dot_col.addWidget(Dot(color, 7))
        dot_col.addStretch()
        layout.addLayout(dot_col)

        text_col = QVBoxLayout()
        text_col.setSpacing(2)
        title_label = QLabel(title)
        title_label.setStyleSheet('font-weight: bold;')
        title_label.setWordWrap(True)
        time_label = QLabel(time_text)
        time_label.setObjectName('caption')
        text_col.addWidget(title_label)
        text_col.addWidget(time_label)
        layout.addLayout(text_col, 1)
