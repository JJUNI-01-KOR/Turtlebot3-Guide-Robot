"""
gui_node.py
-----------
실행 진입점.  ros2 run com_gui gui_node

개발 PC에서 실행하며, 로봇(라즈베리파이)과 같은 네트워크 + 같은
ROS_DOMAIN_ID(기본 30)여야 토픽/서비스가 보인다.
"""

import os
import sys

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt

from .main_window import MainWindow
from .theme import STYLESHEET


def main():
    # 고해상도 모니터에서 글자가 뭉개지지 않도록 (QApplication 생성 전에 설정해야 함)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    ros_domain_id = int(os.environ.get('ROS_DOMAIN_ID', '30'))

    app = QApplication(sys.argv)
    app.setApplicationName('COM 관제 대시보드')
    # 앱 전체(QApplication) 레벨로 적용해야 QMessageBox 같은 별도 최상위
    # 다이얼로그에도 다크 테마가 씌워진다 (MainWindow에만 적용하면 안 먹힘)
    app.setStyleSheet(STYLESHEET)

    window = MainWindow(ros_domain_id=ros_domain_id)
    window.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
