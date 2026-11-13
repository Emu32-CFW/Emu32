// ============================================================================
//  LCD streaming: lets the emulator keep running while the picture goes out
//
//  Before: every emulated scanline/band ended in a blocking SPI.writeBytes(), so the
//  CPU sat idle for the whole transfer (a 240x180 NES frame = ~17 ms at 40 MHz, i.e.
//  the entire 60 fps frame budget) and emulation only started afterwards.
//
//  Now (dual-core chips): the core copies each finished line into a small ring buffer
//  and carries on emulating; a worker task on the *other* core drains the ring to the
//  LCD. Emulation and SPI transfer overlap, so a frame costs max(emulation, transfer)
//  instead of emulation + transfer.
//
//  API (replaces the old SPI.beginTransaction / lcdWindow / DC+CS / writeBytes / CS /
//  endTransaction sequence in the cores):
//      lcdStreamBegin(x0, y0, x1, y1)   open a pixel window
//      lcdStreamWrite(buf, bytes)       queue RGB565 (already byte-swapped) pixels
//      lcdStreamEnd()                   close it; returns once the LCD has everything
//  Everything outside the cores (menus, lcdFill, overlays) keeps using SPI directly:
//  that is safe because lcdStreamEnd() waits for the ring to drain first.
//
//  Single-core chips (ESP32-C6) and a failed task start use the plain synchronous path.
//  Force it with  #define EMU32_LCD_ASYNC 0  before this file is included.
// ============================================================================
#include "soc/soc_caps.h"
#ifndef EMU32_LCD_ASYNC
  #if SOC_CPU_CORES_NUM > 1
    #define EMU32_LCD_ASYNC 1
  #else
    #define EMU32_LCD_ASYNC 0
  #endif
#endif

#if EMU32_LCD_ASYNC
#define LCDS_SLOTS       16      // ring depth in slots (16 x ~0.5 KB of internal RAM)
#define LCDS_SLOT_BYTES  512     // >= the widest line any core sends (240 px * 2)
#define LCDS_BATCH       4       // wake the worker once this many slots are queued
enum : uint8_t { LCDS_BEGIN = 1, LCDS_DATA = 2, LCDS_END = 3 };
struct LcdSlot { uint8_t cmd; uint16_t len; uint16_t x0, y0, x1, y1; uint8_t data[LCDS_SLOT_BYTES]; };
static LcdSlot lcdsRing[LCDS_SLOTS];
static volatile uint32_t lcdsHead = 0;     // written by the emulation task only
static volatile uint32_t lcdsTail = 0;     // written by the worker only
static volatile bool     lcdsSleeping = false;
static TaskHandle_t      lcdsTask = nullptr;
static bool              lcdsSyncMode = false;   // task could not be started: fall back to direct SPI

static void lcdsWorker(void *) {
  for (;;) {
    uint32_t t = lcdsTail;
    while (t != __atomic_load_n(&lcdsHead, __ATOMIC_ACQUIRE)) {
      const LcdSlot &s = lcdsRing[t % LCDS_SLOTS];
      switch (s.cmd) {
        case LCDS_BEGIN:
          SPI.beginTransaction(lcdCfg);          // begin/end stay on this task (SPI lock owner)
          lcdWindow(s.x0, s.y0, s.x1, s.y1);
          digitalWrite(PIN_LCD_DC, HIGH);
          digitalWrite(PIN_LCD_CS, LOW);
          break;
        case LCDS_DATA:
          SPI.writeBytes((const uint8_t *)s.data, s.len);
          break;
        case LCDS_END:
          digitalWrite(PIN_LCD_CS, HIGH);
          SPI.endTransaction();
          break;
      }
      t++;
      __atomic_store_n(&lcdsTail, t, __ATOMIC_RELEASE);
    }
    // Ring empty: sleep until the producer queues more. Set the flag, then re-check
    // the head so a slot queued in between is never missed.
    __atomic_store_n(&lcdsSleeping, true, __ATOMIC_SEQ_CST);
    if (t != __atomic_load_n(&lcdsHead, __ATOMIC_SEQ_CST)) { __atomic_store_n(&lcdsSleeping, false, __ATOMIC_SEQ_CST); continue; }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    __atomic_store_n(&lcdsSleeping, false, __ATOMIC_SEQ_CST);
  }
}

static void lcdsStart() {
  // Run the worker on the core the Arduino loop is NOT using.
  const BaseType_t core = xPortGetCoreID() ? 0 : 1;
  if (xTaskCreatePinnedToCore(lcdsWorker, "lcd_push", 4096, nullptr, 4, &lcdsTask, core) != pdPASS) {
    lcdsTask = nullptr;
    lcdsSyncMode = true;
    Serial.println("LCD: worker task failed to start; using synchronous SPI");
  } else {
    Serial.printf("LCD: async push task on core %d\n", (int)core);
  }
}

static inline void lcdsKick() { if (__atomic_load_n(&lcdsSleeping, __ATOMIC_SEQ_CST) && lcdsTask) xTaskNotifyGive(lcdsTask); }

static LcdSlot *lcdsAcquire() {
  const uint32_t h = lcdsHead;
  while ((uint32_t)(h - __atomic_load_n(&lcdsTail, __ATOMIC_ACQUIRE)) >= LCDS_SLOTS) {   // ring full
    lcdsKick();
    taskYIELD();
  }
  return &lcdsRing[h % LCDS_SLOTS];
}

static inline void lcdsCommit(bool force) {
  const uint32_t h = lcdsHead + 1;
  __atomic_store_n(&lcdsHead, h, __ATOMIC_SEQ_CST);
  if (__atomic_load_n(&lcdsSleeping, __ATOMIC_SEQ_CST) && (force || (uint32_t)(h - lcdsTail) >= LCDS_BATCH)) lcdsKick();
}
#endif  // EMU32_LCD_ASYNC

static void lcdStreamBegin(int x0, int y0, int x1, int y1) {
#if EMU32_LCD_ASYNC
  if (!lcdsTask && !lcdsSyncMode) lcdsStart();
  if (!lcdsSyncMode) {
    LcdSlot *s = lcdsAcquire();
    s->cmd = LCDS_BEGIN; s->x0 = x0; s->y0 = y0; s->x1 = x1; s->y1 = y1;
    lcdsCommit(false);
    return;
  }
#endif
  SPI.beginTransaction(lcdCfg);
  lcdWindow(x0, y0, x1, y1);
  digitalWrite(PIN_LCD_DC, HIGH);
  digitalWrite(PIN_LCD_CS, LOW);
}

static void lcdStreamWrite(const void *buf, size_t n) {
#if EMU32_LCD_ASYNC
  if (!lcdsSyncMode) {
    const uint8_t *p = (const uint8_t *)buf;
    while (n) {
      const size_t c = n > LCDS_SLOT_BYTES ? LCDS_SLOT_BYTES : n;
      LcdSlot *s = lcdsAcquire();
      s->cmd = LCDS_DATA; s->len = (uint16_t)c;
      memcpy(s->data, p, c);
      lcdsCommit(false);
      p += c; n -= c;
    }
    return;
  }
#endif
  SPI.writeBytes((const uint8_t *)buf, (uint32_t)n);
}

static void lcdStreamEnd() {
#if EMU32_LCD_ASYNC
  if (!lcdsSyncMode) {
    LcdSlot *s = lcdsAcquire();
    s->cmd = LCDS_END;
    lcdsCommit(true);
    // Wait for the LCD to receive everything: the caller (menus, overlays, lcdFill) may use SPI next.
    while (__atomic_load_n(&lcdsTail, __ATOMIC_ACQUIRE) != lcdsHead) { lcdsKick(); delayMicroseconds(4); }
    return;
  }
#endif
  digitalWrite(PIN_LCD_CS, HIGH);
  SPI.endTransaction();
}
