#!/usr/bin/env python3
"""
gTTS로 voice_node가 재생하는 안내 문구 wav 파일을 (재)생성하는 스크립트.

voice_node.py는 실행 시 gTTS를 호출하지 않고 사전 생성된 wav만 aplay로 재생한다
(SR-023, Wi-Fi 없어도 음성 안내 동작). 이 스크립트는 그 wav를 만드는 개발용
도구이며, 로봇에서 실행할 필요는 없다 - 문구를 고치거나 새 text_key를 추가할
때 이 스크립트만 다시 돌리면 된다.

기본 동작은 audio/ 디렉토리를 바로 덮어쓰지 않고 audio/generated/ 스테이징
폴더에 만든다 - 기존에 검증된 wav를 실수로 깨뜨리지 않기 위해서다. 들어보고
문제없으면 audio/로 직접 복사할 것.

사용법:
    python3 generate_audio.py                  # 전체 문구 생성 -> audio/generated/
    python3 generate_audio.py --key ready       # ready.wav만 생성
    python3 generate_audio.py --list            # text_key/문구 목록만 출력
    python3 generate_audio.py --output-dir ../audio   # 바로 audio/에 덮어쓰기(주의)
"""

import argparse
import os
import sys

# 2026-08-11: ~/Downloads/Main_PJ_1/04_시스템소프트웨어설계서.docx(SDD-001) 5장
# "음성·효과음·로봇 눈 표정 매핑" 표의 문구를 그대로 사용(효과음 "시작음/경고음/완료음"
# 표기는 텍스트로 안 옮김 - 별도 sound_effect 필드 몫). person_wait/mission_cancelled는
# 그 표엔 없어서 각각 SDD-001 4.7절, URS-001 3.2절(예외 상황 처리 표)의 문구를 사용.
TEXT_MAP = {
    "system_start": "시스템을 시작합니다.",
    "ready": "주행 준비가 완료되었습니다.",
    "guiding_start": "안내를 시작합니다.",
    "arrived": "목적지에 도착했습니다.",
    "docked_arrive": "충전 위치에 도착했습니다.",
    "mission_complete": "임무를 완료했습니다.",
    "mission_cancelled": "임무를 취소합니다.",
    "obstacle": "장애물이 감지되었습니다.",
    "paused": "임무를 일시 정지합니다.",
    "person_wait": "안내 중입니다.",
    "error": "오류가 발생했습니다.",
    # 원본 표에 긴급정지 전용 문구가 없어 URS-001 UR-016(긴급정지 버튼 0.5초 이내
    # 모터 전원 차단)의 취지에 맞춰 새로 작성(2026-08-11, 기존엔 error.wav를 재사용).
    "emergency_stop": "긴급 정지합니다.",
    "wifi_disconnected": "통신이 끊어졌습니다. 임무는 계속 진행됩니다.",
}

DEFAULT_OUTPUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "audio", "generated"
)


def generate_one(key, text, output_dir, lang):
    from gtts import gTTS

    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, f"{key}.wav")
    tmp_mp3 = os.path.join(output_dir, f"{key}.mp3")

    gTTS(text=text, lang=lang).save(tmp_mp3)

    # aplay는 wav만 재생하므로 mp3 -> wav 변환 (ffmpeg 필요). +20dB 게인과 리미터를
    # 같이 걸어서 볼륨을 키운다 - 2026-08-11 실측: 로봇에 물린 USB 오디오 동글
    # (Jieli UACDemoV1.0)이 ALSA 믹서를 100%(-0.94dB, 사실상 유니티 게인)로 올려도
    # 스피커가 하드웨어적으로 너무 작게 들려서, 콘텐츠 자체를 증폭하는 쪽으로 보완함
    # (게인만 올리면 클리핑 위험 - alimiter로 0dBFS 근처에서 하드 리미팅해서 방지).
    ret = os.system(
        f'ffmpeg -y -loglevel error -i "{tmp_mp3}" '
        f'-af "volume=20dB,alimiter=limit=0.98:attack=1:release=50" '
        f'-ar 24000 "{out_path}"'
    )
    os.remove(tmp_mp3)
    if ret != 0:
        raise RuntimeError(f"{key}: ffmpeg 변환 실패 (ffmpeg 설치 확인)")

    print(f"  {key}.wav <- \"{text}\"")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--key", help="이 text_key 하나만 생성 (미지정 시 전체)")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR,
                         help=f"출력 폴더 (기본: {DEFAULT_OUTPUT_DIR})")
    parser.add_argument("--lang", default="ko", help="gTTS 언어 코드 (기본: ko)")
    parser.add_argument("--list", action="store_true",
                         help="생성하지 않고 text_key/문구 목록만 출력")
    args = parser.parse_args()

    if args.list:
        for k, v in TEXT_MAP.items():
            print(f"{k}: {v}")
        return

    if args.key:
        if args.key not in TEXT_MAP:
            print(f"알 수 없는 key: {args.key} (--list로 목록 확인)", file=sys.stderr)
            sys.exit(1)
        targets = {args.key: TEXT_MAP[args.key]}
    else:
        targets = TEXT_MAP

    output_dir = os.path.abspath(args.output_dir)
    print(f"출력 폴더: {output_dir}")
    for key, text in targets.items():
        generate_one(key, text, output_dir, args.lang)

    print(f"\n완료: {len(targets)}개 생성. 들어보고 문제없으면 audio/로 복사하세요:")
    print(f"  cp {output_dir}/*.wav {os.path.join(os.path.dirname(output_dir), '')}")


if __name__ == "__main__":
    main()
