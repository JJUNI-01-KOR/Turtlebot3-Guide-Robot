/* ============================================================
 *  PATROL-1  ROBOT HMI  v5 (통합: 목적지 메뉴 + 이동중 눈 + 도착화면)
 *  Arduino UNO + 3.5" TFT shield (ARS36125D / MCUFRIEND_kbv) + 조이스틱
 *  480x320 landscape, 1개 보드로 메뉴/눈/상태바를 모두 처리한다.
 *
 *  화면 상태:
 *    SELECT  : waypoints.yaml 목적지 메뉴 표시, 조이스틱으로 선택
 *    MOVING  : 스캔라인 렌더러로 그린 눈(SCAN 표정) - 이동 중
 *    ARRIVED : 웃는 눈(HAPPY) + "ARRIVED!" 문구, 3초 후 SELECT로 복귀
 *
 *  ---- 시리얼 프로토콜 (개행 종료, 115200 baud) ----
 *   [보드 -> Pi]
 *     "SELECT:<destination_id>"   목적지 선택 확정 (waypoints.yaml의 id와 동일)
 *     "READY"                     부팅 완료
 *   [Pi -> 보드]
 *     "S:MOVING"                  이동 중 화면으로 전환 (com_sensors/hmi_bridge_node.py)
 *     "S:ARRIVED"                 도착 화면으로 전환
 *     "B:<0-100>"                 배터리 퍼센트 갱신 (미배선 시 기본값 100 유지)
 *     "W:<0-4>"                   WiFi 막대 갱신 (미배선 시 기본값 4 유지)
 *  --------------------------------
 *
 *  배선(가정, 실측 후 조정 필요):
 *    - 조이스틱: JOY_Y = A5(위아래), JOY_BTN = 10(선택) - 실드가 A0~A4를 쓰므로
 *      남는 핀인지 실제 실드 핀맵으로 재확인할 것.
 * ============================================================ */
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>

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

// ---------------- 조이스틱 ----------------
#define JOY_Y   A5
#define JOY_BTN 10

// ---------------- 목적지 메뉴 (waypoints.yaml is_destination:true 와 동일 순서/id 유지) ----------------
struct Destination { const char* id; const char* label; };
const Destination DESTINATIONS[] = {
  {"room1", "Room 1"},
  {"room2", "Room 2"},
  {"room3", "Room 3"},
  {"room4", "Room 4"},
  {"room5", "Room 5"},
  {"room6", "Meeting Room"},
};
const int NUM_DEST = sizeof(DESTINATIONS) / sizeof(DESTINATIONS[0]);
int cursorIdx = 0, prevCursor = -1;

// ---------------- 화면 상태 ----------------
enum UIState { SELECT, MOVING, ARRIVED };
UIState uiState = SELECT;

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

void enterSelect() {
  uiState = SELECT;
  tft.fillScreen(BLACK);
  tft.setTextColor(CYAN); tft.setTextSize(2);
  tft.setCursor(20, 10); tft.print("Select Destination");
  prevCursor = -1;
  drawMenu();
}

void drawMenu() {
  tft.setTextSize(2);
  for (int i = 0; i < NUM_DEST; i++) {
    int yy = 65 + i * 38;
    tft.fillRect(10, yy - 4, 340, 32, (i == cursorIdx) ? GRAY : BLACK);
    tft.setTextColor(i == cursorIdx ? YEL : WHITE);
    tft.setCursor(24, yy);
    if (i == cursorIdx) tft.print("> ");
    tft.print(DESTINATIONS[i].label);
  }
}

void handleSelect() {
  int y = analogRead(JOY_Y);
  static unsigned long lastMove = 0;
  if (millis() - lastMove > 250) {
    if (y < 300 && cursorIdx > 0)            { cursorIdx--; lastMove = millis(); }
    if (y > 700 && cursorIdx < NUM_DEST - 1) { cursorIdx++; lastMove = millis(); }
  }
  if (digitalRead(JOY_BTN) == LOW) {
    confirmSelection();
    delay(300);  // 디바운스
  }
  if (cursorIdx != prevCursor) { drawMenu(); prevCursor = cursorIdx; }
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

  lookX = 0; lookY = 0;
  drawBothFull();

  tft.setTextColor(GREEN); tft.setTextSize(4);
  tft.setCursor(90, 250); tft.print("ARRIVED!");
}

void drawStatusBar() {
  static int pb = -1, pw = -1;
  if (battery == pb && wifi == pw) return;
  pb = battery; pw = wifi;
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
  pinMode(JOY_BTN, INPUT_PULLUP);
  randomSeed(analogRead(A0) ^ micros());

  uint16_t id = tft.readID();
  if (id == 0x0000 || id == 0xFFFF || id == 0xD3D3) id = 0x9486;
  tft.begin(id);
  tft.setRotation(1);

  applyExpr(0);
  enterSelect();

  Serial.println("READY");
}

void loop() {
  readSerial();

  switch (uiState) {
    case SELECT:
      handleSelect();
      break;
    case MOVING: {
      unsigned long now = millis();
      updateGaze(now);
      updateBlink(now);
      break;
    }
    case ARRIVED:
      if (millis() - arrivedAt > 3000) { enterSelect(); }
      break;
  }

  drawStatusBar();
}
