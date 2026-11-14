#pragma once
// Sound-chip write capture for the lightweight synth in audio_synth.h.
//
// The cores call the tiny always-inline hooks below whenever a game writes a sound
// register. The hooks apply the *event* side of the chip (length-counter loads, envelope
// restarts, key-on/off, sweep reloads, triggers, PSG latch/data, YM key-on and DAC
// samples). audio_synth.h applies the *timed* side (frame sequencer, envelopes, waveforms).
// Before this layer the synth only saw the last value of each register, so notes never
// ended, envelopes never decayed, several chips lost writes, and Genesis PSG was silent.
//
// This header has no dependencies so it can be included before the emulator cores.
#include <stdint.h>
#include <string.h>

#define EMU_HOOK static inline __attribute__((always_inline))

namespace AudioEv {

// ------------------------------------------------------------------ NES APU
static const uint8_t NES_LEN[32] = {
  10,254,20, 2,40, 4,80, 6,160, 8,60,10,14,12,26,14,
  12, 16,24,18,48,20,96,22,192,24,72,26,16,28,32,30 };

struct NesPulse { uint8_t r[4]; uint8_t len; bool envStart; uint8_t envDiv, envDecay; bool swReload; uint8_t swDiv; };
struct NesState {
  NesPulse p[2];
  uint8_t triR[4]; uint8_t triLen, triLin; bool triReload;
  uint8_t noiR[4]; uint8_t noiLen; bool noiEnvStart; uint8_t noiEnvDiv, noiEnvDecay;
  uint8_t enable;          // $4015 low 4 bits
  uint8_t dmcLevel;        // $4011 direct level (DMC sample playback itself is not emulated)
};
static NesState nes;

EMU_HOOK void nesWr(uint8_t reg, uint8_t v) {
  switch (reg) {
    case 0x00: case 0x01: case 0x02: case 0x03:
    case 0x04: case 0x05: case 0x06: case 0x07: {
      NesPulse &p = nes.p[reg >> 2]; const uint8_t i = reg & 3;
      p.r[i] = v;
      if (i == 1) p.swReload = true;
      if (i == 3) { if (nes.enable & (1 << (reg >> 2))) p.len = NES_LEN[v >> 3]; p.envStart = true; }
    } break;
    case 0x08: case 0x09: case 0x0A: nes.triR[reg - 8] = v; break;
    case 0x0B: nes.triR[3] = v; if (nes.enable & 4) nes.triLen = NES_LEN[v >> 3]; nes.triReload = true; break;
    case 0x0C: case 0x0D: case 0x0E: nes.noiR[reg - 0x0C] = v; break;
    case 0x0F: nes.noiR[3] = v; if (nes.enable & 8) nes.noiLen = NES_LEN[v >> 3]; nes.noiEnvStart = true; break;
    case 0x11: nes.dmcLevel = v & 0x7F; break;
    case 0x15:
      nes.enable = v & 0x1F;
      if (!(v & 1)) nes.p[0].len = 0;
      if (!(v & 2)) nes.p[1].len = 0;
      if (!(v & 4)) nes.triLen = 0;
      if (!(v & 8)) nes.noiLen = 0;
      break;
    default: break;
  }
}

// ------------------------------------------------------- Game Boy / Game Boy Color APU
struct GbState {
  uint8_t r[0x30];                 // registers $FF10..$FF3F (wave RAM at index 0x20..0x2F)
  bool power;                      // NR52 bit 7
  bool on[4];                      // channel currently playing
  uint16_t len[4];
  uint8_t vol[4], envTimer[4];
  uint16_t shadow; uint8_t swTimer; bool swEnabled;
  uint16_t lfsr;
};
static GbState gb[2];

EMU_HOOK void gbTrigger(GbState &s, int ch) {
  static const uint16_t maxLen[4] = { 64, 64, 256, 64 };
  static const uint8_t nrx2[4] = { 0x02, 0x07, 0x0A, 0x11 };   // index into r[] of NRx2 (ch3: NR32)
  const uint8_t env = s.r[nrx2[ch]];
  s.on[ch] = true;
  if (s.len[ch] == 0) s.len[ch] = maxLen[ch];
  if (ch != 2) { s.vol[ch] = env >> 4; s.envTimer[ch] = (env & 7) ? (env & 7) : 8; }
  if (ch == 0) {
    const uint8_t nr10 = s.r[0];
    s.shadow = s.r[3] | ((s.r[4] & 7) << 8);
    s.swTimer = ((nr10 >> 4) & 7) ? ((nr10 >> 4) & 7) : 8;
    s.swEnabled = ((nr10 >> 4) & 7) || (nr10 & 7);
  }
  if (ch == 3) s.lfsr = 0x7FFF;
  // A channel whose DAC is off cannot be turned on.
  if (ch == 2 ? !(s.r[0x0A] & 0x80) : !(env & 0xF8)) s.on[ch] = false;
}

// sys: 0 = GB, 1 = GBC; r = register offset within $FF00; called for r in 0x10..0x3F
EMU_HOOK void gbWr(uint8_t sys, uint8_t r, uint8_t v) {
  GbState &s = gb[sys];
  const uint8_t i = r - 0x10;
  if (r == 0x26) {
    s.power = v & 0x80;
    s.r[i] = v;
    if (!s.power) { for (int c = 0; c < 4; c++) s.on[c] = false; }
    return;
  }
  if (!s.power && r < 0x30) return;                    // registers are read-only while the APU is off
  s.r[i] = v;
  switch (r) {
    case 0x11: s.len[0] = 64 - (v & 63); break;
    case 0x16: s.len[1] = 64 - (v & 63); break;
    case 0x1B: s.len[2] = 256 - v; break;
    case 0x20: s.len[3] = 64 - (v & 63); break;
    case 0x12: if (!(v & 0xF8)) s.on[0] = false; break;
    case 0x17: if (!(v & 0xF8)) s.on[1] = false; break;
    case 0x1A: if (!(v & 0x80)) s.on[2] = false; break;
    case 0x21: if (!(v & 0xF8)) s.on[3] = false; break;
    case 0x14: if (v & 0x80) gbTrigger(s, 0); break;
    case 0x19: if (v & 0x80) gbTrigger(s, 1); break;
    case 0x1E: if (v & 0x80) gbTrigger(s, 2); break;
    case 0x23: if (v & 0x80) gbTrigger(s, 3); break;
    default: break;
  }
}

// ------------------------------------------------------------------ Genesis PSG / YM2612
struct PsgState { uint16_t tone[3]; uint8_t vol[4]; uint8_t noise; uint8_t latch; };
static PsgState psg;

EMU_HOOK void psgWr(uint8_t v) {
  if (v & 0x80) {
    psg.latch = (v >> 4) & 7;                          // bit2: 1=volume, 0=tone/noise; bits 1-0: channel
    const uint8_t ch = psg.latch >> 1;
    if (psg.latch & 1) psg.vol[ch] = v & 15;
    else if (ch == 3) psg.noise = v & 7;
    else psg.tone[ch] = (psg.tone[ch] & 0x3F0) | (v & 15);
  } else {
    const uint8_t ch = psg.latch >> 1;
    if (psg.latch & 1) psg.vol[ch] = v & 15;
    else if (ch == 3) psg.noise = v & 7;
    else psg.tone[ch] = (psg.tone[ch] & 0x00F) | ((uint16_t)(v & 0x3F) << 4);
  }
}

static constexpr int YM_DAC_MAX = 512;
struct YmState {
  uint8_t reg[2][256];
  bool keyOn[6][4];                // per channel, per operator
  uint8_t keyEv[6];                // bit0 = key-on edge seen since last frame, bit1 = key-off edge
  uint8_t dac[YM_DAC_MAX]; uint16_t dacN;   // $2A samples written this frame
};
static YmState ym;

EMU_HOOK void ymWr(uint8_t port, uint8_t addr, uint8_t v) {
  ym.reg[port][addr] = v;
  if (port == 0) {
    if (addr == 0x28) {
      const uint8_t c = v & 7; if (c == 3 || c == 7) return;
      const int ch = (c & 4) ? c - 1 : c;              // 0,1,2 | 4,5,6 -> 0..5
      for (int op = 0; op < 4; op++) {
        const bool now = v & (0x10 << op);
        if (now && !ym.keyOn[ch][op]) ym.keyEv[ch] |= 1;
        if (!now && ym.keyOn[ch][op]) ym.keyEv[ch] |= 2;
        ym.keyOn[ch][op] = now;
      }
    } else if (addr == 0x2A) {
      if (ym.dacN < YM_DAC_MAX) ym.dac[ym.dacN++] = v;
    }
  }
}

// ------------------------------------------------------------------ housekeeping
static uint32_t epoch = 0;            // bumped on every core reset so the synth can clear its own state
static void resetAll() {
  epoch++;
  memset(&nes, 0, sizeof(nes));
  memset(gb, 0, sizeof(gb));
  for (int i = 0; i < 2; i++) { gb[i].power = true; gb[i].r[0x24 - 0x10] = 0x77; gb[i].r[0x25 - 0x10] = 0xF3; gb[i].lfsr = 0x7FFF; }
  memset(&psg, 0, sizeof(psg));
  for (int i = 0; i < 4; i++) psg.vol[i] = 15;         // PSG powers up silent
  memset(&ym, 0, sizeof(ym));
}

} // namespace AudioEv
