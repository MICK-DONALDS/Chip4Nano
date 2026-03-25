#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define JOY_X     A0
#define JOY_Y     A1
#define JOY_SW    2
#define JOY_DEAD  300

#define SERIAL_BAUD 57600
#define CHUNK_SIZE  32
#define MAX_ROM     1024
#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15

#define RAM_BASE   0x200u
#define RAM_SIZE   512u
#define DISP_W     64
#define DISP_H     32
#define DISP_BYTES 256

static uint8_t  ram[RAM_SIZE];
static uint8_t  fb[DISP_BYTES];
static uint8_t  V[16];
static uint16_t stk[12];
static uint16_t I_reg, PC;
static uint8_t  chip8_sp;
static uint8_t  dtimer, stimer;
static uint8_t  keybits;
static bool     drawFlag;

static const uint8_t FONT[80] PROGMEM = {
  0xF0,0x90,0x90,0x90,0xF0, 0x20,0x60,0x20,0x20,0x70,
  0xF0,0x10,0xF0,0x80,0xF0, 0xF0,0x10,0xF0,0x10,0xF0,
  0x90,0x90,0xF0,0x10,0x10, 0xF0,0x80,0xF0,0x10,0xF0,
  0xF0,0x80,0xF0,0x90,0xF0, 0xF0,0x10,0x20,0x40,0x40,
  0xF0,0x90,0xF0,0x90,0xF0, 0xF0,0x90,0xF0,0x10,0xF0,
  0xF0,0x90,0xF0,0x90,0x90, 0xE0,0x90,0xE0,0x90,0xE0,
  0xF0,0x80,0x80,0x80,0xF0, 0xE0,0x90,0x90,0x90,0xE0,
  0xF0,0x80,0xF0,0x80,0xF0, 0xF0,0x80,0xF0,0x80,0x80
};

static const char BL0[] PROGMEM = "Chip4Nano";
static const char BL1[] PROGMEM = "Enjoy!";
static const char BL2[] PROGMEM = "ATmega328P";
static const char BL3[] PROGMEM = "SRAM OK";
static const char BL4[] PROGMEM = "Storage: EEPROM 1024B";
static const char BL5[] PROGMEM = "Disp: SH1106 128x64";
static const char BL6[] PROGMEM = "Press Joystick";

static const char * const BOOT_LINES[] PROGMEM = {BL0,BL1,BL2,BL3,BL4,BL5,BL6};
#define BOOT_LINE_COUNT 7

static void oledMsg(const char *l1, const char *l2 = nullptr) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(0, 10, l1);
    if (l2) u8g2.drawStr(0, 22, l2);
  } while (u8g2.nextPage());
}

static void oledProgress(uint16_t received, uint16_t total) {
  uint8_t pct = (uint8_t)((uint32_t)received * 100UL / total);
  uint8_t filled = pct / 5;
  char bar[24] = {'['};
  for (uint8_t i = 0; i < 20; i++) bar[1+i] = (i < filled) ? '#' : '-';
  bar[21] = ']'; bar[22] = '\0';
  char num[24];
  snprintf(num, sizeof(num), "%u/%u", received, total);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(0, 10, "Flashing ROM...");
    u8g2.drawStr(0, 22, bar);
    u8g2.drawStr(0, 34, num);
  } while (u8g2.nextPage());
}

static void runFlashMode() {
  Serial.begin(SERIAL_BAUD);
  oledMsg("FLASH MODE", "Hold button...");
  Serial.println("READY");

  while (true) {
    if (digitalRead(JOY_SW)) {
      oledMsg("Exiting", "Restarting...");
      delay(800);
      wdt_enable(WDTO_15MS);
      while (true);
    }

    if (Serial.available() && Serial.read() == SOH) break;
    delay(10);
  }

  while (Serial.available() < 2) delay(10);
  uint16_t rom_size = ((uint16_t)Serial.read() << 8) | Serial.read();

  if (rom_size == 0 || rom_size > MAX_ROM) {
    Serial.write(NAK);
    oledMsg("FAIL", "Bad size");
    while (true);
  }
  Serial.write(ACK);

  uint8_t chunk_buf[CHUNK_SIZE];
  uint16_t received = 0;

  while (received < rom_size) {
    uint8_t chunk_len = min((uint16_t)CHUNK_SIZE, rom_size - received);
    unsigned long t0 = millis();

    while (Serial.available() < chunk_len + 1) {
      if (millis() - t0 > 5000) {
        Serial.write(NAK);
        oledMsg("FAIL", "Timeout");
        while (true);
      }
      if (digitalRead(JOY_SW)) {
        oledMsg("Aborted", "");
        while (true);
      }
    }

    for (uint8_t i = 0; i < chunk_len; i++) chunk_buf[i] = Serial.read();
    uint8_t rx_sum = Serial.read();

    uint8_t calc = 0;
    for (uint8_t i = 0; i < chunk_len; i++) calc += chunk_buf[i];

    if (calc != rx_sum) {
      Serial.write(NAK);
      oledMsg("FAIL", "Checksum");
      while (true);
    }

    for (uint8_t i = 0; i < chunk_len; i++)
      EEPROM.update(received + i, chunk_buf[i]);

    received += chunk_len;
    Serial.write(ACK);
    oledProgress(received, rom_size);
  }

  unsigned long t0 = millis();
  while (Serial.available() < 1 && millis() - t0 < 3000);
  if (Serial.read() != EOT) {
    Serial.write(NAK);
    return;
  }

  Serial.write(ACK);
  Serial.println("WRITE OK");
  oledMsg("Flash OK!", "Release btn & reset");
  delay(1500);

  while (!digitalRead(JOY_SW));
  wdt_enable(WDTO_15MS);
  while (true);
}

static uint8_t memRead(uint16_t addr) {
  if (addr < 0x050u) return pgm_read_byte(&FONT[addr]);
  if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE)
    return ram[addr - RAM_BASE];
  return 0;
}

static void memWrite(uint16_t addr, uint8_t val) {
  if (addr >= RAM_BASE && addr < RAM_BASE + RAM_SIZE)
    ram[addr - RAM_BASE] = val;
}

inline void fbClear() { memset(fb, 0, DISP_BYTES); }

inline bool fbGet(uint8_t x, uint8_t y) {
  uint16_t b = (uint16_t)y * DISP_W + x;
  return (fb[b >> 3] >> (7 - (b & 7))) & 1;
}

inline void fbFlip(uint8_t x, uint8_t y) {
  uint16_t b = (uint16_t)y * DISP_W + x;
  fb[b >> 3] ^= (1 << (7 - (b & 7)));
}

#define K_UP 0x01
#define K_DN 0x02
#define K_LF 0x04
#define K_RT 0x08
#define K_SW 0x10

static bool keyDown(uint8_t k) {
  switch (k) {
    case 0x2: return keybits & K_UP;
    case 0x8: return keybits & K_DN;
    case 0x4: return keybits & K_LF;
    case 0x6: return keybits & K_RT;
    case 0x5: return keybits & K_SW;
    default: return false;
  }
}

static void showBootLog() {
  const uint8_t LINE_H = 9;
  char buf[20];
  for (uint8_t i = 0; i < BOOT_LINE_COUNT; i++) {
    uint8_t start = (i + 1 > 6) ? (i + 1 - 6) : 0;
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_5x8_tf);
      for (uint8_t j = start; j <= i; j++) {
        strcpy_P(buf, (const char *)pgm_read_ptr(&BOOT_LINES[j]));
        uint8_t y = (j - start + 1) * LINE_H;
        u8g2.drawStr(0, y, j == i ? ">" : " ");
        u8g2.drawStr(7, y, buf);
      }
    } while (u8g2.nextPage());
    if (i < BOOT_LINE_COUNT - 1) delay(300);
  }

  bool cur = true;
  while (digitalRead(JOY_SW)) {
    u8g2.firstPage();
    do {
      u8g2.setFont(u8g2_font_5x8_tf);
      for (uint8_t j = 0; j < BOOT_LINE_COUNT; j++) {
        strcpy_P(buf, (const char *)pgm_read_ptr(&BOOT_LINES[j]));
        uint8_t y = (j + 1) * LINE_H;
        u8g2.drawStr(0, y, (j == BOOT_LINE_COUNT-1 && cur) ? ">" : " ");
        u8g2.drawStr(7, y, buf);
      }
    } while (u8g2.nextPage());
    cur = !cur;
    delay(400);
  }
  delay(200);
}

static void chip8Init() {
  memset(ram, 0, RAM_SIZE);
  memset(V, 0, 16);
  memset(stk, 0, sizeof(stk));
  fbClear();
  I_reg = 0; PC = 0x200; chip8_sp = 0;
  dtimer = stimer = 0; drawFlag = false;

  for (uint16_t i = 0; i < RAM_SIZE; i++)
    ram[i] = EEPROM.read(i);
}

static void renderDisplay() {
  u8g2.firstPage();
  do {
    for (uint8_t py = 0; py < DISP_H; py++) {
      uint8_t sy = py << 1;
      for (uint8_t px = 0; px < DISP_W; px++) {
        if (fbGet(px, py)) {
          uint8_t sx = px << 1;
          u8g2.drawBox(sx, sy, 2, 2);
        }
      }
    }
  } while (u8g2.nextPage());
}

static void chip8Step() {
  uint16_t op = ((uint16_t)memRead(PC) << 8) | memRead(PC + 1);
  PC += 2;
  uint8_t x = (op >> 8) & 0xF;
  uint8_t y = (op >> 4) & 0xF;
  uint8_t n = op & 0xF;
  uint8_t kk = op & 0xFF;
  uint16_t nnn = op & 0xFFF;

  switch (op & 0xF000) {
    case 0x0000:
      if (op == 0x00E0) { fbClear(); drawFlag = true; }
      else if (op == 0x00EE) PC = stk[--chip8_sp];
      break;
    case 0x1000: PC = nnn; break;
    case 0x2000: stk[chip8_sp++] = PC; PC = nnn; break;
    case 0x3000: if (V[x] == kk) PC += 2; break;
    case 0x4000: if (V[x] != kk) PC += 2; break;
    case 0x5000: if (V[x] == V[y]) PC += 2; break;
    case 0x6000: V[x] = kk; break;
    case 0x7000: V[x] += kk; break;
    case 0x8000:
      switch (n) {
        case 0x0: V[x] = V[y]; break;
        case 0x1: V[x] |= V[y]; break;
        case 0x2: V[x] &= V[y]; break;
        case 0x3: V[x] ^= V[y]; break;
        case 0x4: { uint16_t r = (uint16_t)V[x] + V[y]; V[0xF] = r > 255 ? 1 : 0; V[x] = (uint8_t)r; } break;
        case 0x5: V[0xF] = V[x] >= V[y] ? 1 : 0; V[x] -= V[y]; break;
        case 0x6: V[0xF] = V[x] & 1; V[x] >>= 1; break;
        case 0x7: V[0xF] = V[y] >= V[x] ? 1 : 0; V[x] = V[y] - V[x]; break;
        case 0xE: V[0xF] = (V[x] >> 7) & 1; V[x] <<= 1; break;
      }
      break;
    case 0x9000: if (V[x] != V[y]) PC += 2; break;
    case 0xA000: I_reg = nnn; break;
    case 0xB000: PC = nnn + V[0]; break;
    case 0xC000: V[x] = (uint8_t)random(256) & kk; break;
    case 0xD000: {
      uint8_t xp = V[x] & 63, yp = V[y] & 31;
      V[0xF] = 0;
      for (uint8_t r = 0; r < n; r++) {
        uint8_t spr = memRead(I_reg + r);
        for (uint8_t c = 0; c < 8; c++) {
          if (spr & (0x80 >> c)) {
            uint8_t px = (xp + c) & 63, py = (yp + r) & 31;
            if (fbGet(px, py)) V[0xF] = 1;
            fbFlip(px, py);
          }
        }
      }
      drawFlag = true;
    } break;
    case 0xE000:
      if (kk == 0x9E && keyDown(V[x])) PC += 2;
      if (kk == 0xA1 && !keyDown(V[x])) PC += 2;
      break;
    case 0xF000:
      switch (kk) {
        case 0x07: V[x] = dtimer; break;
        case 0x0A: {
          bool found = false;
          for (uint8_t k = 0; k < 16; k++) if (keyDown(k)) { V[x] = k; found = true; break; }
          if (!found) PC -= 2;
        } break;
        case 0x15: dtimer = V[x]; break;
        case 0x18: stimer = V[x]; break;
        case 0x1E: I_reg += V[x]; break;
        case 0x29: I_reg = V[x] * 5; break;
        case 0x33:
          memWrite(I_reg, V[x]/100);
          memWrite(I_reg+1, (V[x]/10)%10);
          memWrite(I_reg+2, V[x]%10);
          break;
        case 0x55: for(uint8_t i=0;i<=x;i++) memWrite(I_reg+i,V[i]); break;
        case 0x65: for(uint8_t i=0;i<=x;i++) V[i]=memRead(I_reg+i); break;
      }
      break;
  }
}

static void readInputs() {
  keybits = 0;
  int jx = analogRead(JOY_X);
  int jy = analogRead(JOY_Y);
  if (jy < 512 - JOY_DEAD) keybits |= K_UP;
  if (jy > 512 + JOY_DEAD) keybits |= K_DN;
  if (jx < 512 - JOY_DEAD) keybits |= K_LF;
  if (jx > 512 + JOY_DEAD) keybits |= K_RT;
  if (!digitalRead(JOY_SW)) keybits |= K_SW;
}

void setup() {
  pinMode(JOY_SW, INPUT_PULLUP);
  u8g2.begin();
  u8g2.setBusClock(400000);
  delay(50);

  if (!digitalRead(JOY_SW)) {
    runFlashMode();
  }

  showBootLog();
  randomSeed(micros());
  chip8Init();
}

#define CPU_US 2000UL
#define TMR_US 16667UL

static unsigned long lastCpu = 0;
static unsigned long lastTmr = 0;

void loop() {
  unsigned long now = micros();

  if ((long)(now - lastCpu) >= (long)CPU_US) {
    lastCpu += CPU_US;
    readInputs();
    chip8Step();
  }

  if ((long)(now - lastTmr) >= (long)TMR_US) {
    lastTmr += TMR_US;
    if (dtimer) dtimer--;
    if (stimer) stimer--;
  }

  if (drawFlag) {
    drawFlag = false;
    renderDisplay();
  }
}
