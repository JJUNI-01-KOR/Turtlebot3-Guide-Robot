/* ============================================================
 *  PATROL-1  ROBOT HMI  v5 (통합: 목적지 메뉴 + 이동중 눈 + 도착화면)
 *  Arduino UNO + 3.5" TFT shield (ARS36125D / MCUFRIEND_kbv)
 *  480x320 landscape, 이 보드는 화면만 그린다.
 *
 *  2026-08-04 재구성: 조이스틱은 이 보드가 아니라 메인센서보드(다른 아두이노,
 *  arduino/flaem_pir_dht11_jul31a.ino)에 물려있음. 그 보드가 raw 조이스틱 값을
 *  파이로 보내면 hmi_bridge_node.py가 디바운스/임계값 판정을 해서 이 보드에는
 *  이미 판정된 MOVE:UP / MOVE:DOWN / CONFIRM 명령만 시리얼로 내려준다 - 로컬
 *  아날로그/디지털 핀을 직접 읽지 않는다.
 *
 *  화면 상태:
 *    WELCOME : 전원 인가 직후 2.5초간 "환영합니다" 웰컴 화면 표시 후 자동으로 SELECT로
 *              전환(2026-08-12 추가, 기존 웰컴 라이트를 대체). 실제 주행상태와 무관한
 *              부팅 시퀀스 전용 화면 - 로컬 타이머만 씀.
 *    SELECT  : waypoints.yaml 목적지 메뉴 표시, 파이가 중계한 조이스틱 명령으로 선택
 *              (목적지 CONFIRM 1번째는 확인화면, 2번째에 실제 출발)
 *    MOVING  : 스캔라인 렌더러로 그린 눈(SCAN 표정) - 이동 중
 *    ARRIVED : 웃는 눈(HAPPY) + "ARRIVED!" 문구, 5초 후 SELECT로 복귀
 *    IDLE_SCREENSAVER : SELECT/확인화면에서 30초간 입력 없으면 진입하는 화면보호기
 *              (IDLE 표정 눈 애니메이션, 실제 주행과 무관 - 로컬 타이머). 아무 조이스틱
 *              입력이나 오면 그 입력은 "깨우기"로 소비하고 SELECT로 복귀.
 *
 *  ---- 시리얼 프로토콜 (개행 종료, 115200 baud) ----
 *   [보드 -> Pi]
 *     "SELECT:<destination_id>"   목적지 선택 확정 (waypoints.yaml의 id와 동일)
 *     "READY"                     부팅 완료
 *   [Pi -> 보드]
 *     "MOVE:UP" / "MOVE:DOWN"     메뉴 커서 이동 (hmi_bridge_node.py가 조이스틱
 *                                 raw 값에 디바운스/임계값 적용해서 보냄)
 *     "CONFIRM"                   버튼 입력 (디바운스됨). 목적지당 두 번 눌러야 출발함:
 *                                 1번째 - 확인화면(목적지 이름 크게 표시), 2번째 - 실제 출발.
 *                                 확인화면 중 MOVE:UP/DOWN이 오면 취소하고 메뉴로 복귀.
 *     "S:MOVING"                  이동 중 화면으로 전환 (state_code==0x03)
 *     "S:ARRIVED"                 도착 화면으로 전환 (state_code==0x04)
 *     "S:SELECT"                  메뉴 화면으로 강제 복귀 (state_code==0x0A ERROR/EMERGENCY_STOP,
 *                                 예전엔 이 처리가 없어서 ERROR 진입 시 화면이 MOVING에 멈춰있었음)
 *     "B:<0-100>"                 배터리 퍼센트 갱신 (미배선 시 기본값 100 유지)
 *     "W:<0-4>"                   WiFi 막대 갱신 (미배선 시 기본값 4 유지)
 *  --------------------------------
 * ============================================================ */
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include "hangul_font.h"

MCUFRIEND_kbv tft;

// ---------------- UI 공통 색 ----------------
#define BLACK   0x0000
#define WHITE   0xFFFF
#define CYAN    0x07FF
#define GREEN   0x07E0
#define YEL     0xFFE0
#define RED     0xF800
#define GRAY    0x8410
#define SHADE1  0xDEDB      // 흰자 옅은 그림자
#define SHADE2  0xC618      // 흰자 진한 그림자
#define LOWLID  0xAD55      // 눈꺼풀 라인
#define LIDLN   0x8C7F      // 감은 눈 선

// ---------------- 목적지 메뉴 (waypoints.yaml is_destination:true 와 동일 순서/id 유지) ----------------
// 라벨은 한글로 표시한다. Adafruit_GFX 기본 폰트엔 한글 글리프가 없어서(하드웨어 제약 -
// Uno 플래시로 완전한 Hangul 폰트를 올리는 건 비현실적), hangul_font.h의 20x20 비트맵
// 글리프를 조합해서 그린다. 숫자는 기존 ASCII 폰트 재사용.
// 2026-08-19: 주석에 "글리프 8종"이라 적혀 있었으나 실제로는 34종이다
// (강의실회인포상담목적지선택다시누르면출발조이스틱으로취소도착환영합니).
// id 는 내부 키일 뿐 라벨 뜻과 1:1 대응이 아니다: info->회의실, office->인포,
// meeting->상담실. 물리 위치는 id 이름 변경 전과 동일하다.
struct Destination { const char* id; uint8_t glyphs[3]; char digit; };
const Destination DESTINATIONS[] = {
  {"class1", {G_GANG, G_UI, G_SIL}, '1'},
  {"class2", {G_GANG, G_UI, G_SIL}, '2'},
  {"class3", {G_GANG, G_UI, G_SIL}, '3'},
  {"class4", {G_GANG, G_UI, G_SIL}, '4'},
  {"class5", {G_GANG, G_UI, G_SIL}, '5'},
  {"info", {G_HOE, G_UI, G_SIL}, 0},
  {"office", {G_IN, G_PO, G_NONE}, 0},
  {"meeting", {G_SANG, G_DAM, G_SIL}, 0},
};
const int NUM_DEST = sizeof(DESTINATIONS) / sizeof(DESTINATIONS[0]);

// 한글 비트맵 글리프 1개를 (x,y)에 scale배로 그린다 (scale=1 -> 20x20, scale=2 -> 40x40).
void drawGlyph(int x, int y, uint8_t glyph, uint16_t color, uint8_t scale) {
  if (glyph == G_NONE) return;
  const uint8_t* bmp = (const uint8_t*)pgm_read_ptr(&HANGUL_BITMAPS[glyph]);
  const uint8_t rowBytes = (HANGUL_GLYPH_W + 7) / 8;
  for (uint8_t row = 0; row < HANGUL_GLYPH_H; row++) {
    for (uint8_t col = 0; col < HANGUL_GLYPH_W; col++) {
      uint8_t byteVal = pgm_read_byte(&bmp[row * rowBytes + (col / 8)]);
      if (byteVal & (0x80 >> (col % 8))) {
        if (scale == 1) tft.drawPixel(x + col, y + row, color);
        else tft.fillRect(x + col * scale, y + row * scale, scale, scale, color);
      }
    }
  }
}

// 글리프 최대 3개 + 숫자 접미사 1개로 구성된 목적지 라벨을 (x,y)부터 그린다.
void drawDestLabel(int x, int y, const Destination& d, uint16_t color, uint8_t scale) {
  const int glyphPx = HANGUL_GLYPH_W * scale;
  const int gap = 2 * scale;
  int cx = x;
  for (uint8_t i = 0; i < 3; i++) {
    if (d.glyphs[i] == G_NONE) break;
    drawGlyph(cx, y, d.glyphs[i], color, scale);
    cx += glyphPx + gap;
  }
  if (d.digit) {
    const uint8_t textSize = (scale == 1) ? 2 : 5;
    tft.setTextSize(textSize);
    tft.setTextColor(color);
    tft.setCursor(cx, y + (glyphPx - 8 * textSize) / 2);
    tft.print(d.digit);
  }
}

// 안내 문구(가변 길이 한글 글리프 배열, PROGMEM)를 (x,y)부터 그린다. G_SPACE는
// 그리지 않고 커서만 진행시켜 단어 사이 띄어쓰기를 표현한다.
void drawHangulPhrase(int x, int y, const uint8_t* phrase, uint8_t count, uint16_t color, uint8_t scale) {
  const int glyphPx = HANGUL_GLYPH_W * scale;
  const int gap = 2 * scale;
  const int spacePx = glyphPx / 2 + gap;
  int cx = x;
  for (uint8_t i = 0; i < count; i++) {
    uint8_t g = pgm_read_byte(&phrase[i]);
    if (g == G_SPACE) { cx += spacePx; continue; }
    drawGlyph(cx, y, g, color, scale);
    cx += glyphPx + gap;
  }
}

int cursorIdx = 0, prevCursor = -1;
bool confirmPending = false;  // true = 목적지 확인 화면 표시 중 (한번 더 눌러야 실제 출발)

// ---------------- 화면 상태 ----------------
enum UIState { WELCOME, SELECT, MOVING, ARRIVED, IDLE_SCREENSAVER };
UIState uiState = SELECT;

#define WELCOME_MS 2500UL
unsigned long welcomeAt = 0;

// 메뉴/확인화면에서 30초간 조이스틱 입력이 없으면 대기용 눈 애니메이션으로 전환
// (실제 주행 중이라는 뜻은 아님 - 그냥 화면보호기, MOVING과 표정만 다름).
#define IDLE_SCREENSAVER_MS 30000UL
unsigned long lastInputAt = 0;

int battery = 100, wifi = 4;
unsigned long arrivedAt = 0;

// ============================================================
//  ---- 아래부터는 눈 렌더러 (PATROL-1 v4) : MOVING/ARRIVED 상태에서 사용 ----
// ============================================================

// ---------------- 팔레트 ----------------
// [0..7] 홍채 세로 그라데이션(위=어두움), [8] 외곽 링, [9] 림 라이트
const uint16_t pal_blue[10] PROGMEM = {
  0x08A9,0x094D,0x1212,0x12F7,0x1C1A,0x2D5D,0x567F,0x875F, 0x0007,0xAFFF };
const uint16_t pal_green[10] PROGMEM = {
  0x00C1,0x0983,0x0A85,0x0B88,0x14CB,0x2E10,0x5715,0x8FDA, 0x0103,0xBFFC };
const uint16_t pal_amber[10] PROGMEM = {
  0x1860,0x38C0,0x5960,0x8220,0xAB40,0xD481,0xF5E5,0xFF2F, 0x2040,0xFFD6 };
const uint16_t pal_red[10] PROGMEM = {
  0x1800,0x3801,0x6022,0x8863,0xB0C5,0xD188,0xF2ED,0xFCB4, 0x1801,0xFE9F };

const uint16_t* const PALS[4] PROGMEM = { pal_blue, pal_green, pal_amber, pal_red };
#define NPAL   4
#define NBANDS 8

// ------- 눈 형상 (전부 눈 중심 기준 로컬 좌표) -------
#define EYE_L   (-65)
#define EYE_R   ( 64)
#define EYE_T   (-75)
#define EYE_B   ( 74)
#define EYE_W   (EYE_R - EYE_L + 1)

#define SCL_HW   60         // 흰자 반폭
#define SCL_TOP (-70)
#define SCL_BOT ( 68)
#define SCL_RT   26         // 윗 모서리 반경 (작을수록 위가 넓음)
#define SCL_RB   48         // 아랫 모서리 반경 (클수록 아래가 좁음)

#define R_RING   47         // 홍채 외곽 링
#define R_BODY   43         // 홍채 몸통
#define PUP_RX   15         // 동공 가로 반경
#define PUP_RY   19         // 동공 세로 반경

#define LID_MAX  75
#define LOOK_X   11
#define LOOK_Y    7

// ---------------- 화면상 눈 위치 ----------------
const int leftEyeX  = 120;
const int rightEyeX = 360;
const int eyeY      = 160;

// ============================================================
//  표정 = 눈꺼풀 포물선 파라미터
// ============================================================
struct Shape {
  int8_t   uY;
  int8_t   uS;      // + = 바깥쪽으로 이동 (좌우 눈 자동 대칭)
  uint16_t uK;
  int8_t   lY;
  uint16_t lK;
  int8_t   irisY;   // 홍채 상하 보정
  int8_t   pupD;    // 동공 크기 보정
  uint8_t  pal;     // 기본 팔레트
};

#define NEXPR 8
const Shape SHAPES[NEXPR] PROGMEM = {
  /* 0 IDLE   */ { -58,   0, 110,  100, 100,   0,  0, 0 },
  /* 1 HAPPY  */ { -30,   0, 100,   -8, 900, -12,  0, 1 },
  /* 2 THINK  */ { -14,   0, 400,  100, 100,  -4, -3, 0 },
  /* 3 ALERT  */ { -66,   0, 250,  100, 100,   0, -6, 2 },
  /* 4 ERROR  */ { -52,  34,  95,  100, 100,   0, -2, 3 },
  /* 5 SAD    */ { -52, -34,  95,  100, 100,   4,  2, 0 },
  /* 6 SLEEP  */ {   2,   0, 900,  100, 100,   0,  2, 1 },
  /* 7 SCAN   */ { -34,   0, 700,   34, 700,   0, -2, 2 }
};

Shape cur;                    // 현재 적용된 표정
byte  curExpr    = 0;
byte  reqExpr    = 0;
bool  exprPending = false;
byte  palOverride = 255;      // 255 = 표정 기본값 사용

// ---------------- 시선 / 깜빡임 상태 ----------------
int  lookX = 0, lookY = 0;
int  tgtX  = 0, tgtY  = 0;
bool gazePending = false;
bool autoGaze    = true;
unsigned long gazeAt = 0;

enum { ST_OPEN, ST_CLOSING, ST_CLOSED, ST_OPENING };
byte eyeState = ST_CLOSED;
int  lid = LID_MAX;
unsigned long tMark = 0, blinkAt = 0, lastCmdMs = 0;
byte blinkQueue = 0;

void readSerial();            // 전방 선언 (렌더러 안에서 호출)

// ============================================================
//  저수준 헬퍼
// ============================================================
static int gClipL, gClipR;

static void span(int x0, int x1, int y, uint16_t c) {
  if (x0 < gClipL) x0 = gClipL;
  if (x1 > gClipR) x1 = gClipR;
  if (x1 < x0) return;
  tft.drawFastHLine(x0, y, x1 - x0 + 1, c);
}

static unsigned int isqrt16(unsigned int x) {
  unsigned int op = x, res = 0, one = 1u << 14;
  while (one > op) one >>= 2;
  while (one != 0) {
    if (op >= res + one) { op -= res + one; res += one << 1; }
    res >>= 1;
    one >>= 2;
  }
  return res;
}

static int chord(int dy, int r) {
  if (dy < 0) dy = -dy;
  if (dy > r) return -1;
  return (int)isqrt16((unsigned int)(r * r - dy * dy));
}

static int chordEll(int dy, int rx, int ry) {
  int h = chord(dy, ry);
  if (h < 0) return -1;
  return (int)(((long)h * rx) / ry);
}

static int lidHalf(int dyRows, unsigned int k) {
  unsigned long q = (unsigned long)dyRows * k;
  if (q > 65535UL) q = 65535UL;
  return (int)isqrt16((unsigned int)q);
}

static uint16_t scleraShade(int y) {
  if (y < -44) return SHADE2;
  if (y < -32) return SHADE1;
  return WHITE;
}

static void glint(int gx, int dy, int r, int sy, int sx0, int sx1) {
  int a = chord(dy, r);
  if (a < 0) return;
  int x0 = gx - a, x1 = gx + a;
  if (x0 < sx0) x0 = sx0;
  if (x1 > sx1) x1 = sx1;
  span(x0, x1, sy, WHITE);
}

// ============================================================
//  한 행의 눈알 내용 (sx0~sx1 구간에만)
// ============================================================
static void rowContent(int sy, int y, int ix, int ic,
                       int sx0, int sx1,
                       const uint16_t* pal, int prx, int pry) {
  if (sx1 < sx0) return;

  const uint16_t wcol = scleraShade(y);

  int aR = chord(y - ic, R_RING);
  if (aR < 0 || ix + aR < sx0 || ix - aR > sx1) {
    span(sx0, sx1, sy, wcol);
    return;
  }

  const uint16_t cRing = pgm_read_word(&pal[8]);
  const uint16_t cRim  = pgm_read_word(&pal[9]);

  int r0 = ix - aR, r1 = ix + aR;
  if (r0 < sx0) r0 = sx0;
  if (r1 > sx1) r1 = sx1;
  span(sx0, r0 - 1, sy, wcol);
  span(r1 + 1, sx1, sy, wcol);

  int aB = chord(y - ic, R_BODY);
  if (aB < 0) {
    span(r0, r1, sy, cRing);
  } else {
    int b0 = ix - aB, b1 = ix + aB;
    if (b0 < r0) b0 = r0;
    if (b1 > r1) b1 = r1;
    span(r0, b0 - 1, sy, cRing);
    span(b1 + 1, r1, sy, cRing);

    int t  = y - ic + R_BODY;                    // 0 .. 2*R_BODY
    int bi = (t * NBANDS) / (2 * R_BODY);
    if (bi < 0) bi = 0;
    if (bi > NBANDS - 1) bi = NBANDS - 1;
    uint16_t bcol = pgm_read_word(&pal[bi]);

    int rt = (y >= ic + 14) ? 6 : 0;             // 아랫쪽만 림 라이트
    int pw = chordEll(y - ic, prx, pry);

    if (pw < 0) {
      if (rt) {
        int lr = b0 + rt - 1; if (lr > b1) lr = b1;
        int rr = b1 - rt + 1; if (rr < b0) rr = b0;
        span(b0, lr, sy, cRim);
        span(lr + 1, rr - 1, sy, bcol);
        span(rr, b1, sy, cRim);
      } else {
        span(b0, b1, sy, bcol);
      }
    } else {
      int p0 = ix - pw, p1 = ix + pw;
      if (p0 < b0) p0 = b0;
      if (p1 > b1) p1 = b1;
      if (rt) {
        int lr = b0 + rt - 1; if (lr > p0 - 1) lr = p0 - 1;
        int rr = b1 - rt + 1; if (rr < p1 + 1) rr = p1 + 1;
        span(b0, lr, sy, cRim);
        span(lr + 1, p0 - 1, sy, bcol);
        span(p1 + 1, rr - 1, sy, bcol);
        span(rr, b1, sy, cRim);
      } else {
        span(b0, p0 - 1, sy, bcol);
        span(p1 + 1, b1, sy, bcol);
      }
      span(p0, p1, sy, BLACK);                   // 동공
    }
  }

  // 반사광
  glint(ix - 17, y - (ic - 18), 14, sy, sx0, sx1);
  glint(ix + 19, y - (ic + 20),  7, sy, sx0, sx1);
  glint(ix - 25, y - (ic + 12),  4, sy, sx0, sx1);
  glint(ix + 23, y - (ic - 26),  3, sy, sx0, sx1);
}

// ============================================================
//  핵심 렌더러 — y0~y1 행을, 가로는 cl~cr 범위만
// ============================================================
void renderEye(int cx, int cy, int outSign, int lx, int ly,
               int y0, int y1, int cl, int cr) {
  if (y0 < EYE_T) y0 = EYE_T;
  if (y1 > EYE_B) y1 = EYE_B;
  if (y1 < y0) return;
  if (cl < EYE_L) cl = EYE_L;
  if (cr > EYE_R) cr = EYE_R;
  if (cr < cl) return;

  gClipL = cx + cl;
  gClipR = cx + cr;

  const uint16_t* pal =
      (const uint16_t*)pgm_read_word(&PALS[(palOverride < NPAL) ? palOverride : cur.pal]);

  const int ix  = cx + lx;
  const int ic  = 10 + ly + cur.irisY;
  const int uSx = cur.uS * outSign;
  int prx = PUP_RX + cur.pupD; if (prx < 5) prx = 5;
  int pry = PUP_RY + cur.pupD; if (pry < 6) pry = 6;

  for (int y = y0; y <= y1; y++) {
    if ((y & 7) == 0) readSerial();      // 렌더 중에도 수신버퍼 비우기
    const int sy = cy + y;

    // ---- 1. 흰자 반폭 ----
    int sw = -1;
    if (y <= SCL_BOT && y >= SCL_TOP) {
      if (y > SCL_BOT - SCL_RB) {
        sw = SCL_HW - SCL_RB + chord(y - (SCL_BOT - SCL_RB), SCL_RB);
      } else if (y < SCL_TOP + SCL_RT) {
        sw = SCL_HW - SCL_RT + chord((SCL_TOP + SCL_RT) - y, SCL_RT);
      } else {
        sw = SCL_HW;
      }
    }
    if (sw < 0) { span(cx + EYE_L, cx + EYE_R, sy, BLACK); continue; }

    int vL = cx - sw, vR = cx + sw;
    span(cx + EYE_L, vL - 1, sy, BLACK);
    span(vR + 1, cx + EYE_R, sy, BLACK);

    // ---- 2. 윗눈꺼풀 ----
    if (y <= cur.uY) { span(vL, vR, sy, BLACK); continue; }
    {
      int w = lidHalf(y - cur.uY, cur.uK);
      int a = cx + uSx - w, b = cx + uSx + w;
      if (a > vL) { span(vL, a - 1, sy, BLACK); vL = a; }
      if (b < vR) { span(b + 1, vR, sy, BLACK); vR = b; }
      if (vR < vL) continue;
    }

    // ---- 3. 눈알 하단 눈꺼풀 라인 ----
    if (y >= SCL_BOT - 5) { span(vL, vR, sy, LOWLID); continue; }

    // ---- 4. 아랫눈꺼풀 (가운데를 아래에서 덮어 ^^ 모양을 만듦) ----
    int lo0 = 1, lo1 = 0;                        // 비어있음
    if (cur.lY < 90 && y >= cur.lY) {
      int w2 = lidHalf(y - cur.lY, cur.lK);
      lo0 = cx - w2; lo1 = cx + w2;
      int c0 = (lo0 > vL) ? lo0 : vL;
      int c1 = (lo1 < vR) ? lo1 : vR;
      if (c1 >= c0) {
        int e0 = lo0 + 2; if (e0 > c1) e0 = c1;
        int e1 = lo1 - 2; if (e1 < c0) e1 = c0;
        if (e1 <= e0) {
          span(c0, c1, sy, LOWLID);
        } else {
          span(c0, e0, sy, LOWLID);
          span(e0 + 1, e1 - 1, sy, BLACK);
          span(e1, c1, sy, LOWLID);
        }
      }
    }

    // ---- 5. 눈알 내용 (아랫꺼풀로 갈라지면 두 구간) ----
    if (lo1 >= lo0) {
      rowContent(sy, y, ix, ic, vL, (lo0 - 1 < vR) ? lo0 - 1 : vR, pal, prx, pry);
      rowContent(sy, y, ix, ic, (lo1 + 1 > vL) ? lo1 + 1 : vL, vR, pal, prx, pry);
    } else {
      rowContent(sy, y, ix, ic, vL, vR, pal, prx, pry);
    }
  }
}

void drawEyeFull(int cx, int cy, int outSign) {
  renderEye(cx, cy, outSign, lookX, lookY, EYE_T, EYE_B, EYE_L, EYE_R);
}
void drawBothFull() {
  drawEyeFull(leftEyeX,  eyeY, -1);
  drawEyeFull(rightEyeX, eyeY, +1);
}

// ============================================================
//  눈꺼풀 애니메이션
// ============================================================
void lidStrips(int cx, int cy, int from, int to) {
  int h = to - from;
  if (h <= 0) return;
  tft.fillRect(cx + EYE_L, cy + EYE_T + from,   EYE_W, h, BLACK);
  tft.fillRect(cx + EYE_L, cy + EYE_B - to + 1, EYE_W, h, BLACK);
}

void openReveal(int cx, int cy, int outSign, int from, int to) {
  renderEye(cx, cy, outSign, lookX, lookY, EYE_T + to, EYE_T + from - 1, EYE_L, EYE_R);
  renderEye(cx, cy, outSign, lookX, lookY, EYE_B - from + 1, EYE_B - to, EYE_L, EYE_R);
}

void drawClosedLine(int cx, int cy) {
  for (int i = -48; i <= 48; i += 8) {
    int yo = (i * i) / 220;
    tft.fillRect(cx + i, cy - 2 - yo, 9, 6, LIDLN);
  }
}
void eraseClosedLine(int cx, int cy) {
  tft.fillRect(cx + EYE_L, cy - 16, EYE_W, 25, BLACK);
}

void saccade(int cx, int cy, int outSign, int ox, int oy, int nx, int ny) {
  int base = 10 + cur.irisY;
  int yT = ((oy < ny) ? oy : ny) + base - R_RING - 1;
  int yB = ((oy > ny) ? oy : ny) + base + R_RING + 1;
  int xL = ((ox < nx) ? ox : nx) - R_RING - 1;
  int xR = ((ox > nx) ? ox : nx) + R_RING + 1;
  renderEye(cx, cy, outSign, nx, ny, yT, yB, xL, xR);
}

void applyExpr(byte n) {
  if (n >= NEXPR) return;
  memcpy_P(&cur, &SHAPES[n], sizeof(Shape));
  curExpr = n;
}

void updateGaze(unsigned long now) {
  if (autoGaze && !gazePending && (long)(now - gazeAt) >= 0) {
    int nx = 0, ny = 0;
    for (byte t = 0; t < 8; t++) {
      nx = random(-LOOK_X, LOOK_X + 1);
      ny = random(-LOOK_Y, LOOK_Y + 1);
      if (abs(nx - lookX) >= 4 || abs(ny - lookY) >= 3) break;
    }
    tgtX = nx; tgtY = ny;
    gazePending = true;
    gazeAt = now + random(2000, 5200);
  }
  if (!gazePending) return;

  if (eyeState == ST_CLOSED) {
    lookX = tgtX; lookY = tgtY;
    gazePending = false;
  } else if (eyeState == ST_OPEN) {
    saccade(leftEyeX,  eyeY, -1, lookX, lookY, tgtX, tgtY);
    saccade(rightEyeX, eyeY, +1, lookX, lookY, tgtX, tgtY);
    lookX = tgtX; lookY = tgtY;
    gazePending = false;
  }
}

void updateBlink(unsigned long now) {
  switch (eyeState) {

    case ST_OPEN:
      if (exprPending) blinkAt = now;
      if ((long)(now - blinkAt) >= 0) {
        eyeState = ST_CLOSING; lid = 0; tMark = now;
      }
      break;

    case ST_CLOSING: {
      if (now - tMark < 8) break;
      tMark = now;
      int prev = lid;
      lid += 13 + (lid >> 3);
      if (lid > LID_MAX) lid = LID_MAX;
      lidStrips(leftEyeX,  eyeY, prev, lid);
      lidStrips(rightEyeX, eyeY, prev, lid);
      if (lid >= LID_MAX) {
        drawClosedLine(leftEyeX,  eyeY);
        drawClosedLine(rightEyeX, eyeY);
        eyeState = ST_CLOSED; tMark = now;
      }
      break;
    }

    case ST_CLOSED:
      if (exprPending) { applyExpr(reqExpr); exprPending = false; }
      if (now - tMark > 55) {
        eraseClosedLine(leftEyeX,  eyeY);
        eraseClosedLine(rightEyeX, eyeY);
        eyeState = ST_OPENING; tMark = now;
      }
      break;

    case ST_OPENING: {
      int prev = lid;
      lid -= 9;
      if (lid < 0) lid = 0;
      openReveal(leftEyeX,  eyeY, -1, prev, lid);
      openReveal(rightEyeX, eyeY, +1, prev, lid);
      if (lid == 0) {
        eyeState = ST_OPEN;
        if (blinkQueue) { blinkQueue--; blinkAt = millis() + 110; }
        else {
          unsigned int lo = 2200, hi = 6000;
          if (curExpr == 7) { lo = 7000; hi = 14000; }   // SCAN: 거의 안 깜빡임
          blinkAt = millis() + random(lo, hi);
          if (random(100) < 22) blinkQueue = 1;
        }
      }
      break;
    }
  }
}

// ============================================================
//  ---- 여기부터 UI (메뉴 / 상태바 / 화면 전환) ----
// ============================================================

const uint8_t PHRASE_SELECT_TITLE[] PROGMEM = { G_MOK, G_JEOK, G_JI, G_SPACE, G_SEON, G_TAEK };           // 목적지 선택
const uint8_t PHRASE_PRESS_AGAIN[]  PROGMEM = { G_DA, G_SI, G_SPACE, G_NU, G_REU, G_MYEON, G_SPACE, G_CHUL, G_BAL };  // 다시 누르면 출발
const uint8_t PHRASE_CANCEL[]       PROGMEM = { G_JO, G_I, G_SEU, G_TIK, G_EU, G_RO, G_SPACE, G_CHWI, G_SO };         // 조이스틱으로 취소
const uint8_t PHRASE_ARRIVED[]      PROGMEM = { G_DO, G_CHAK };                                            // 도착
const uint8_t PHRASE_WELCOME[]      PROGMEM = { G_HWAN, G_YEONG, G_HAP, G_NI, G_DA };                      // 환영합니다

// 전원 인가 직후 2.5초간 보여주는 웰컴 화면 (2026-08-12 추가, 기존 웰컴 라이트 대체).
// setup()에서 한 번 호출되고, loop()의 WELCOME 분기가 시간 경과 후 enterSelect()로 넘긴다.
void enterWelcome() {
  uiState = WELCOME;
  welcomeAt = millis();
  tft.fillScreen(BLACK);
  resetStatusBar();

  // "환영합니다" 큰 글씨 중앙 정렬 (glyph 5개, scale=3 -> 60px/글자 + 6px 간격)
  const uint8_t scale = 3;
  const int glyphPx = HANGUL_GLYPH_W * scale;
  const int gap = 2 * scale;
  const int n = sizeof(PHRASE_WELCOME);
  const int textW = n * glyphPx + (n - 1) * gap;
  drawHangulPhrase((480 - textW) / 2, 110, PHRASE_WELCOME, n, CYAN, scale);

  // 부제(ASCII, 기존 폰트 재사용 - 새 글리프 불필요)
  tft.setTextSize(2);
  tft.setTextColor(GRAY);
  const char* sub = "COM Visitor Guide Robot";
  int subW = strlen(sub) * 12;  // Adafruit_GFX 기본폰트: size 2에서 문자당 12px
  tft.setCursor((480 - subW) / 2, 210);
  tft.print(sub);

  tft.drawFastHLine(140, 240, 200, GRAY);
}

void enterSelect() {
  uiState = SELECT;
  confirmPending = false;
  lastInputAt = millis();
  tft.fillScreen(BLACK);
  resetStatusBar();
  drawHangulPhrase(20, 8, PHRASE_SELECT_TITLE, sizeof(PHRASE_SELECT_TITLE), CYAN, 1);
  prevCursor = -1;
  drawMenu();
}

void drawMenu() {
  // NUM_DEST(현재 8개)가 6개 기준이던 기존 38px 간격으로는 화면(320px 높이) 밖으로
  // 넘쳐서 32px 간격 + 시작 y를 50으로 당김 - 8개까지 여유있게 들어간다.
  for (int i = 0; i < NUM_DEST; i++) {
    int yy = 50 + i * 32;
    tft.fillRect(10, yy - 4, 340, 28, (i == cursorIdx) ? GRAY : BLACK);
    uint16_t color = i == cursorIdx ? YEL : WHITE;
    int lx = 24;
    if (i == cursorIdx) {
      tft.setTextSize(2);
      tft.setTextColor(YEL);
      tft.setCursor(lx, yy + 2);
      tft.print(">");
      lx += 16;
    }
    drawDestLabel(lx, yy, DESTINATIONS[i], color, 1);
  }
}

// 목적지를 처음 CONFIRM 받았을 때: 바로 출발하지 않고 확인 화면을 띄운다
// (실수로 조이스틱 버튼을 잘못 눌러 엉뚱한 곳으로 출발하는 걸 방지).
void drawConfirmPrompt() {
  tft.fillScreen(BLACK);
  resetStatusBar();
  drawHangulPhrase(20, 8, PHRASE_SELECT_TITLE, sizeof(PHRASE_SELECT_TITLE), CYAN, 1);

  drawDestLabel(30, 100, DESTINATIONS[cursorIdx], YEL, 2);

  drawHangulPhrase(30, 180, PHRASE_PRESS_AGAIN, sizeof(PHRASE_PRESS_AGAIN), WHITE, 1);
  drawHangulPhrase(30, 220, PHRASE_CANCEL, sizeof(PHRASE_CANCEL), GRAY, 1);
}

void handleSelect() {
  // 커서 이동/확정은 이제 handleLine()의 MOVE:UP / MOVE:DOWN / CONFIRM 처리에서
  // 일어난다 (파이가 조이스틱을 대신 읽어서 디바운스까지 마친 명령을 보내줌).
  // confirmPending 중엔 확인 화면을 그대로 유지하고 메뉴를 다시 그리지 않는다.
  if (!confirmPending && cursorIdx != prevCursor) { drawMenu(); prevCursor = cursorIdx; }
}

void confirmSelection() {
  Serial.print("SELECT:");
  Serial.println(DESTINATIONS[cursorIdx].id);
}

void enterMoving() {
  uiState = MOVING;
  applyExpr(7);                 // SCAN
  palOverride = 255;
  tft.fillScreen(BLACK);
  resetStatusBar();

  lookX = 0; lookY = 0;
  gazePending = false;
  eyeState = ST_CLOSED;
  lid = LID_MAX;
  tMark = millis();
  blinkQueue = 0;
  gazeAt = millis() + 1500;

  drawClosedLine(leftEyeX,  eyeY);
  drawClosedLine(rightEyeX, eyeY);
}

// 화면보호기: 실제 주행 상태(state_code)와 무관하게 메뉴/확인화면에서 30초간
// 입력이 없을 때만 로컬에서 진입. IDLE 표정으로 MOVING과 구분되게 함.
void enterScreensaver() {
  uiState = IDLE_SCREENSAVER;
  applyExpr(0);                 // IDLE
  palOverride = 255;
  tft.fillScreen(BLACK);
  resetStatusBar();

  lookX = 0; lookY = 0;
  gazePending = false;
  eyeState = ST_CLOSED;
  lid = LID_MAX;
  tMark = millis();
  blinkQueue = 0;
  gazeAt = millis() + 1500;

  drawClosedLine(leftEyeX,  eyeY);
  drawClosedLine(rightEyeX, eyeY);
}

void enterArrived() {
  uiState = ARRIVED;
  arrivedAt = millis();
  applyExpr(1);                 // HAPPY
  palOverride = 255;
  tft.fillScreen(BLACK);
  resetStatusBar();

  lookX = 0; lookY = 0;
  drawBothFull();

  drawHangulPhrase(150, 240, PHRASE_ARRIVED, sizeof(PHRASE_ARRIVED), GREEN, 3);
  tft.setTextColor(GREEN); tft.setTextSize(6);
  tft.setCursor(282, 246); tft.print("!");
}

// 2026-08-12: 화면 전환마다 fillScreen(BLACK)이 상태바 영역까지 지우는데, drawStatusBar()는
// battery/wifi "값이 바뀔 때만" 다시 그려서 값이 마침 그대로면 화면 전환 직후 상태바가 빈
// 채로 남아있던 버그(특히 MOVING 진입 시 실측 확인) - pb/pw를 전역으로 빼서 각 화면 진입
// 함수(enterWelcome/enterSelect/enterMoving/enterScreensaver/enterArrived)에서 강제로
// -1로 리셋해 다음 drawStatusBar() 호출에서 무조건 다시 그리게 한다.
int statusBarPrevBattery = -1, statusBarPrevWifi = -1;
void resetStatusBar() { statusBarPrevBattery = -1; statusBarPrevWifi = -1; }

void drawStatusBar() {
  if (battery == statusBarPrevBattery && wifi == statusBarPrevWifi) return;
  statusBarPrevBattery = battery; statusBarPrevWifi = wifi;
  tft.fillRect(360, 0, 120, 30, BLACK);
  uint16_t bc = battery > 50 ? GREEN : battery > 20 ? YEL : RED;
  tft.drawRect(430, 5, 36, 18, WHITE);
  tft.fillRect(466, 9, 4, 10, WHITE);
  tft.fillRect(432, 7, map(battery, 0, 100, 0, 32), 14, bc);
  for (int i = 0; i < 4; i++)
    tft.fillRect(370 + i * 12, 22 - (i + 1) * 4, 8, (i + 1) * 4 + 2, i < wifi ? CYAN : GRAY);
}

// ============================================================
//  시리얼 (라즈베리파이 <-> 보드)
// ============================================================
void handleLine(const char* line) {
  if (line[0] == 'B' && line[1] == ':') { battery = atoi(line + 2); return; }
  if (line[0] == 'W' && line[1] == ':') { wifi = atoi(line + 2); return; }
  if (strcmp(line, "S:MOVING") == 0)  { enterMoving();  return; }
  if (strcmp(line, "S:ARRIVED") == 0) { enterArrived(); return; }
  if (strcmp(line, "S:SELECT") == 0)  { enterSelect();  return; }

  // 파이(hmi_bridge_node.py)가 메인센서보드의 조이스틱 raw 값을 디바운스/임계값
  // 판정까지 마친 뒤 보내주는 명령 - SELECT 상태에서만 의미 있음.
  // 확인 화면(confirmPending) 중에 스틱을 움직이면 취소하고 메뉴로 돌아간다.
  // 화면보호기 중에 아무 입력이나 오면 이번 입력은 "깨우기"로만 쓰고 메뉴로 복귀한다
  // (바로 이동/확정까지 적용하면 자다 깬 김에 실수로 출발할 수 있어서 한 단계 둠).
  bool isJoystickInput = strcmp(line, "MOVE:UP") == 0 ||
                         strcmp(line, "MOVE:DOWN") == 0 ||
                         strcmp(line, "CONFIRM") == 0;
  if (isJoystickInput) {
    if (uiState == IDLE_SCREENSAVER) {
      enterSelect();
      return;
    }
    if (uiState == SELECT) lastInputAt = millis();
  }

  if (strcmp(line, "MOVE:UP") == 0) {
    if (uiState == SELECT) {
      bool wasConfirming = confirmPending;
      confirmPending = false;
      if (cursorIdx > 0) cursorIdx--;
      if (wasConfirming) prevCursor = -1;
    }
    return;
  }
  if (strcmp(line, "MOVE:DOWN") == 0) {
    if (uiState == SELECT) {
      bool wasConfirming = confirmPending;
      confirmPending = false;
      if (cursorIdx < NUM_DEST - 1) cursorIdx++;
      if (wasConfirming) prevCursor = -1;
    }
    return;
  }
  // 첫 CONFIRM: 확인 화면 표시. 확인 화면 상태에서 두번째 CONFIRM: 실제 출발.
  if (strcmp(line, "CONFIRM") == 0) {
    if (uiState == SELECT) {
      if (!confirmPending) {
        confirmPending = true;
        drawConfirmPrompt();
      } else {
        confirmPending = false;
        confirmSelection();
      }
    }
    return;
  }

  // "T:<idx>" - 조이스틱 미연결 시 테스트용: 실제 버튼을 누른 것과 동일하게
  // cursorIdx를 옮기고 confirmSelection()을 호출한다 (SELECT 상태에서만 동작).
  if (line[0] == 'T' && line[1] == ':') {
    int idx = atoi(line + 2);
    if (uiState == SELECT && idx >= 0 && idx < NUM_DEST) {
      cursorIdx = idx;
      confirmSelection();
    }
    return;
  }
}

void readSerial() {
  static char buf[16];
  static byte i = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (i == 0) continue;
      buf[i] = 0; i = 0;
      handleLine(buf);
    } else if (i < sizeof(buf) - 1) {
      buf[i++] = c;
    }
  }
}

// ============================================================
//  setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0) ^ micros());

  uint16_t id = tft.readID();
  Serial.print("TFT_ID:0x");
  Serial.println(id, HEX);
  if (id == 0x0000 || id == 0xFFFF || id == 0xD3D3) {
    Serial.println("TFT_ID_FALLBACK: 실제 컨트롤러 응답 없음, 0x9486으로 강제 진행");
    id = 0x9486;
  }
  tft.begin(id);
  tft.setRotation(3);

  applyExpr(0);
  enterWelcome();

  Serial.println("READY");
}

void loop() {
  readSerial();

  switch (uiState) {
    case WELCOME:
      if (millis() - welcomeAt > WELCOME_MS) { enterSelect(); }
      break;
    case SELECT:
      if (millis() - lastInputAt > IDLE_SCREENSAVER_MS) {
        enterScreensaver();
      } else {
        handleSelect();
      }
      break;
    case MOVING: {
      unsigned long now = millis();
      updateGaze(now);
      updateBlink(now);
      break;
    }
    case ARRIVED:
      // URS-001/SDD-001 명시값(5초)과 service_manager의 arrival_wait 파라미터에
      // 맞춤(2026-08-06, 이전 3000ms는 문서 기준과 미동기화 상태였음).
      if (millis() - arrivedAt > 5000) { enterSelect(); }
      break;
    case IDLE_SCREENSAVER: {
      unsigned long now = millis();
      updateGaze(now);
      updateBlink(now);
      break;
    }
  }

  drawStatusBar();
}
