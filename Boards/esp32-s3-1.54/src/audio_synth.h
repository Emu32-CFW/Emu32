#pragma once
// Register-driven sound generators for the five built-in systems.
//
// Still a compact approximation (not cycle-accurate chip emulation), but much closer to
// the real chips than the first version:
//   * every square/noise/wave channel is *area-sampled* ("box filtered") over each output
//     sample, so high notes no longer alias into the harsh, grainy tone of a naive
//     phase-accumulator square wave;
//   * NES/GB/GBC: real length counters, volume envelopes, sweep units, duty cycles,
//     correct noise periods, linear counter, GB wave RAM, NES non-linear mixer;
//   * Genesis: SN76489 PSG (previously never written, so silent), 4-operator FM with the
//     eight algorithms/feedback/ADSR, correct YM pitch, per-channel key-on, DAC samples;
//   * Atari 2600: correct AUDC/AUDF register mapping and TIA poly/divider waveforms;
//   * output stage: DC blocker, gentle low-pass, soft limiter and a fade-in after reset.
// Sound-register write capture lives in audio_events.h (hooks called by the cores).
#include "audio.h"
#include "audio_events.h"
#include <math.h>
#include <string.h>

#ifndef EMU32_SYNTH_PEAK
#define EMU32_SYNTH_PEAK 12000.0f   // int16 amplitude of a full-scale (1.0) mix, before audio.h gain/volume
#endif
#ifndef EMU32_LP_HZ
#define EMU32_LP_HZ 9000.0f         // output low-pass corner (tames the tiny speaker's harshness)
#endif

namespace EmuSound {
  static constexpr float SRATE = (float)EMU32_AUDIO_SAMPLE_RATE;
  static constexpr size_t MAXN = 1024;
  static int16_t mix[MAXN];
  static float   fbuf[MAXN];
  static uint32_t frameRemainder = 0;
  static uint32_t seenEpoch = 0xFFFFFFFFu;

  // ---------------------------------------------------------------- helpers
  // Average of a piecewise-constant signal over one output sample. 'left' is the time (in
  // samples) until the next generator step, 'perTick' the step length in samples. cur()
  // returns the current level; adv() moves the generator to its next step.
  template <class Cur, class Adv>
  static inline float boxAvg(float &left, float perTick, Cur cur, Adv adv) {
    if (perTick < 0.125f) perTick = 0.125f;          // cap work for ultrasonic / very fast noise
    if (left > perTick) left = perTick;
    float remain = 1.0f, acc = 0.0f;
    while (remain > 0.0f) {
      const float use = left < remain ? left : remain;
      acc += cur() * use; left -= use; remain -= use;
      if (left <= 1e-6f) { adv(); left += perTick; }
    }
    return acc;
  }

  // ---------------------------------------------------------------- NES
  static struct NesSyn {
    float left[4]; uint8_t pos[2]; uint8_t tri; uint16_t lfsr; float seqAcc; uint8_t seqStep;
  } ns;

  static const uint8_t NES_DUTY[4] = { 0x40, 0x60, 0x78, 0x9F };      // 8-step patterns, MSB first
  static const uint16_t NES_NOISE_PER[16] = { 4,8,16,32,64,96,128,160,202,254,380,508,762,1016,2034,4068 };

  static void genNes(float *o, size_t n) {
    using namespace AudioEv;
    NesState &s = nes;
    static const float CPU = 1789773.0f;
    float per[2]; bool act[2]; int vol[2]; int duty[2];

    auto envVol = [](const NesPulse &p) { return (p.r[0] & 0x10) ? (p.r[0] & 15) : p.envDecay; };
    auto prep = [&]() {
      for (int c = 0; c < 2; c++) {
        NesPulse &p = s.p[c];
        const int period = p.r[2] | ((p.r[3] & 7) << 8);
        const int delta = period >> (p.r[1] & 7);
        const int target = (p.r[1] & 8) ? period - delta - (c == 0 ? 1 : 0) : period + delta;
        act[c] = (s.enable & (1 << c)) && p.len && period >= 8 && target <= 0x7FF;
        per[c] = 2.0f * (period + 1) * (SRATE / CPU);
        vol[c] = envVol(p);
        duty[c] = p.r[0] >> 6;
      }
    };
    auto envClock = [](uint8_t ctl, bool &start, uint8_t &div, uint8_t &decay) {
      if (start) { start = false; decay = 15; div = ctl & 15; }
      else if (div == 0) { div = ctl & 15; if (decay) decay--; else if (ctl & 0x20) decay = 15; }
      else div--;
    };
    auto quarter = [&]() {
      for (int c = 0; c < 2; c++) envClock(s.p[c].r[0], s.p[c].envStart, s.p[c].envDiv, s.p[c].envDecay);
      envClock(s.noiR[0], s.noiEnvStart, s.noiEnvDiv, s.noiEnvDecay);
      if (s.triReload) s.triLin = s.triR[0] & 0x7F; else if (s.triLin) s.triLin--;
      if (!(s.triR[0] & 0x80)) s.triReload = false;
    };
    auto half = [&]() {
      for (int c = 0; c < 2; c++) {
        NesPulse &p = s.p[c];
        if (p.len && !(p.r[0] & 0x20)) p.len--;
        const int period = p.r[2] | ((p.r[3] & 7) << 8), sh = p.r[1] & 7;
        const int delta = period >> sh;
        const int target = (p.r[1] & 8) ? period - delta - (c == 0 ? 1 : 0) : period + delta;
        if (p.swDiv == 0 && (p.r[1] & 0x80) && sh && period >= 8 && target <= 0x7FF && target >= 0) {
          p.r[2] = target & 0xFF; p.r[3] = (p.r[3] & ~7) | ((target >> 8) & 7);
        }
        if (p.swDiv == 0 || p.swReload) { p.swDiv = (p.r[1] >> 4) & 7; p.swReload = false; } else p.swDiv--;
      }
      if (s.triLen && !(s.triR[0] & 0x80)) s.triLen--;
      if (s.noiLen && !(s.noiR[0] & 0x20)) s.noiLen--;
      prep();
    };
    prep();
    if (!ns.lfsr) ns.lfsr = 1;

    for (size_t i = 0; i < n; i++) {
      ns.seqAcc += 240.0f / SRATE;
      if (ns.seqAcc >= 1.0f) { ns.seqAcc -= 1.0f; quarter(); if (++ns.seqStep & 1) {} else half(); vol[0] = envVol(s.p[0]); vol[1] = envVol(s.p[1]); }

      float pl[2] = { 0, 0 };
      for (int c = 0; c < 2; c++) if (act[c]) {
        const uint8_t d = NES_DUTY[duty[c]]; const int v = vol[c]; uint8_t &pos = ns.pos[c];
        pl[c] = boxAvg(ns.left[c], per[c], [&] { return ((d >> (7 - pos)) & 1) ? (float)v : 0.0f; }, [&] { pos = (pos + 1) & 7; });
      }

      // triangle (32-step; holds its level when silenced, like the hardware)
      const int tp = s.triR[2] | ((s.triR[3] & 7) << 8);
      const bool tAct = (s.enable & 4) && s.triLen && s.triLin;
      float tri;
      if (tAct) {
        tri = boxAvg(ns.left[2], (tp + 1) * (SRATE / CPU), [&] { return (float)(ns.tri < 16 ? 15 - ns.tri : ns.tri - 16); }, [&] { ns.tri = (ns.tri + 1) & 31; });
      } else tri = (float)(ns.tri < 16 ? 15 - ns.tri : ns.tri - 16);

      // noise
      float noi = 0;
      if ((s.enable & 8) && s.noiLen) {
        const int nv = (s.noiR[0] & 0x10) ? (s.noiR[0] & 15) : s.noiEnvDecay;
        const bool shortMode = s.noiR[2] & 0x80;
        const float npr = NES_NOISE_PER[s.noiR[2] & 15] * (SRATE / CPU);
        noi = boxAvg(ns.left[3], npr, [&] { return (ns.lfsr & 1) ? 0.0f : (float)nv; },
                     [&] { const uint16_t fb = (ns.lfsr & 1) ^ ((ns.lfsr >> (shortMode ? 6 : 1)) & 1); ns.lfsr = (ns.lfsr >> 1) | (fb << 14); });
      }

      // NES non-linear mixer (nesdev formulas)
      const float p = pl[0] + pl[1];
      const float pOut = p > 0.0f ? 95.88f / (8128.0f / p + 100.0f) : 0.0f;
      const float x = tri / 8227.0f + noi / 12241.0f + s.dmcLevel / 22638.0f;
      const float tOut = x > 0.0f ? 159.79f / (1.0f / x + 100.0f) : 0.0f;
      o[i] = (pOut + tOut) * 1.4f;
    }
  }

  // ---------------------------------------------------------------- Game Boy / Game Boy Color
  static struct GbSyn { float left[4]; uint8_t pos[2]; uint8_t wpos; float seqAcc; uint8_t seqStep; } gs[2];
  static const uint8_t GB_DUTY[4] = { 0x01, 0x81, 0x87, 0x7E };      // MSB-first 8-step patterns

  static void gbSweepCalc(AudioEv::GbState &s, bool apply) {
    const uint8_t nr10 = s.r[0], sh = nr10 & 7;
    int nf = s.shadow >> sh;
    nf = (nr10 & 8) ? s.shadow - nf : s.shadow + nf;
    if (nf > 2047) { s.on[0] = false; return; }
    if (apply && sh) { s.shadow = nf; s.r[3] = nf & 0xFF; s.r[4] = (s.r[4] & ~7) | ((nf >> 8) & 7); }
  }

  static void genGb(uint8_t sys, float *o, size_t n) {
    using namespace AudioEv;
    GbState &s = gb[sys]; GbSyn &g = gs[sys];
    static const float CLK = 4194304.0f;
    static const uint8_t nrx4[4] = { 0x04, 0x09, 0x0E, 0x13 };
    static const uint8_t nrx2[4] = { 0x02, 0x07, 0x0C, 0x11 };
    if (!s.power) { for (size_t i = 0; i < n; i++) o[i] = 0; return; }

    const float nr50 = s.r[0x14], nr51 = s.r[0x15];
    const float master = (((int)nr50 >> 4 & 7) + ((int)nr50 & 7) + 2) * (1.0f / 16.0f);
    float w[4];
    for (int c = 0; c < 4; c++) w[c] = ((((int)nr51 >> c) & 1) + (((int)nr51 >> (4 + c)) & 1)) * 0.5f * 0.25f * master;

    for (size_t i = 0; i < n; i++) {
      g.seqAcc += 512.0f / SRATE;
      if (g.seqAcc >= 1.0f) {
        g.seqAcc -= 1.0f; g.seqStep = (g.seqStep + 1) & 7;
        if (!(g.seqStep & 1)) {                                         // length counters
          for (int c = 0; c < 4; c++) if ((s.r[nrx4[c]] & 0x40) && s.len[c]) { if (--s.len[c] == 0) s.on[c] = false; }
        }
        if (g.seqStep == 2 || g.seqStep == 6) {                         // channel 1 sweep
          if (s.swEnabled && s.on[0] && --s.swTimer == 0) {
            const uint8_t per = (s.r[0] >> 4) & 7;
            s.swTimer = per ? per : 8;
            if (per) { gbSweepCalc(s, true); if (s.on[0]) gbSweepCalc(s, false); }
          }
        }
        if (g.seqStep == 7) {                                           // volume envelopes
          for (int c = 0; c < 4; c++) if (c != 2 && s.on[c]) {
            const uint8_t env = s.r[nrx2[c]], per = env & 7;
            if (per && --s.envTimer[c] == 0) {
              s.envTimer[c] = per;
              if (env & 8) { if (s.vol[c] < 15) s.vol[c]++; } else if (s.vol[c]) s.vol[c]--;
            }
          }
        }
      }

      float sum = 0;
      for (int c = 0; c < 2; c++) if (s.on[c] && s.vol[c]) {                 // square 1 / 2
        const int base = c ? 0x05 : 0x00;
        const int f = s.r[base + 3] | ((s.r[base + 4] & 7) << 8);
        const uint8_t d = GB_DUTY[s.r[base + 1] >> 6]; const float v = s.vol[c] * (1.0f / 15.0f); uint8_t &pos = g.pos[c];
        const float a = boxAvg(g.left[c], (2048 - f) * 4.0f * (SRATE / CLK), [&] { return ((d >> (7 - pos)) & 1) ? v : -v; }, [&] { pos = (pos + 1) & 7; });
        sum += a * w[c];
      }
      if (s.on[2] && (s.r[0x0A] & 0x80)) {                                    // wave channel (real wave RAM)
        const int code = (s.r[0x0C] >> 5) & 3;
        if (code) {
          const int sh = code - 1, f = s.r[0x0D] | ((s.r[0x0E] & 7) << 8);
          const float mid = 7.5f / (float)(1 << sh);
          const float a = boxAvg(g.left[2], (2048 - f) * 2.0f * (SRATE / CLK), [&] {
            const uint8_t b = s.r[0x20 + (g.wpos >> 1)];
            const int nib = (g.wpos & 1) ? (b & 15) : (b >> 4);
            return ((float)(nib >> sh) - mid) * (1.0f / 7.5f);
          }, [&] { g.wpos = (g.wpos + 1) & 31; });
          sum += a * w[2];
        }
      }
      if (s.on[3] && s.vol[3]) {                                              // noise
        const uint8_t nr43 = s.r[0x12]; const int shift = nr43 >> 4, code = nr43 & 7;
        if (shift < 14) {
          const float cyc = (float)((code ? code * 16 : 8) << shift);
          const float v = s.vol[3] * (1.0f / 15.0f); const bool narrow = nr43 & 8;
          const float a = boxAvg(g.left[3], cyc * (SRATE / CLK), [&] { return (s.lfsr & 1) ? -v : v; },
                                 [&] { const uint16_t x = (s.lfsr & 1) ^ ((s.lfsr >> 1) & 1); s.lfsr = (s.lfsr >> 1) | (x << 14); if (narrow) s.lfsr = (s.lfsr & ~0x40) | (x << 6); });
          sum += a * w[3];
        }
      }
      o[i] = sum;
    }
  }

  // ---------------------------------------------------------------- Atari 2600 (TIA)
  static struct TiaSyn { float left[2]; uint8_t p4[2], p5[2]; uint16_t p9[2]; uint8_t c31[2], c6[2], c93[2], out[2]; } ts;

  static inline uint8_t tiaTick(int c, uint8_t audc) {
    uint8_t &p4 = ts.p4[c], &p5 = ts.p5[c]; uint16_t &p9 = ts.p9[c];
    auto step4 = [&] { p4 = ((p4 << 1) | (((p4 >> 3) ^ (p4 >> 2)) & 1)) & 0xF; };
    auto step5 = [&] { p5 = ((p5 << 1) | (((p5 >> 4) ^ (p5 >> 2)) & 1)) & 0x1F; };
    auto step9 = [&] { p9 = ((p9 << 1) | (((p9 >> 8) ^ (p9 >> 4)) & 1)) & 0x1FF; };
    uint8_t &c31 = ts.c31[c], &c6 = ts.c6[c], &c93 = ts.c93[c];
    c31 = (c31 + 1) % 31; c6 = (c6 + 1) % 6; c93 = (c93 + 1) % 93;
    switch (audc & 15) {
      case 0: case 11: return 1;
      case 1:  step4(); return p4 & 1;
      case 2:  if (c31 == 0) step4(); return p4 & 1;
      case 3:  step5(); if (p5 & 1) step4(); return p4 & 1;
      case 4: case 5: return ts.out[c] ^ 1;
      case 6: case 10: return c31 >= 18;
      case 7: case 9:  step5(); return p5 & 1;
      case 8:  step9(); return p9 & 1;
      case 12: case 13: return c6 >= 3;
      case 14: return c93 >= 62;
      default: if (c6 % 3 == 0) step5(); return p5 & 1;                   // 15
    }
  }

  static void genTia(float *o, size_t n) {
    static const float TIA_CLK = 31400.0f;
    if (!ts.p4[0]) { for (int c = 0; c < 2; c++) { ts.p4[c] = 0xF; ts.p5[c] = 0x1F; ts.p9[c] = 0x1FF; } }
    // TIA register order is AUDC0, AUDC1, AUDF0, AUDF1, AUDV0, AUDV1 (address - $15).
    const uint8_t audc[2] = { (uint8_t)(audioReg[0] & 15), (uint8_t)(audioReg[1] & 15) };
    const uint8_t audf[2] = { (uint8_t)(audioReg[2] & 31), (uint8_t)(audioReg[3] & 31) };
    const float vol[2] = { (audioReg[4] & 15) * (1.0f / 15.0f), (audioReg[5] & 15) * (1.0f / 15.0f) };
    for (size_t i = 0; i < n; i++) {
      float sum = 0;
      for (int c = 0; c < 2; c++) {
        const float per = (audf[c] + 1) * (SRATE / TIA_CLK);
        const float a = boxAvg(ts.left[c], per, [&] { return ts.out[c] ? 1.0f : -1.0f; }, [&] { ts.out[c] = tiaTick(c, audc[c]); });
        sum += a * vol[c] * 0.45f;
      }
      o[i] = sum;
    }
  }

  // ---------------------------------------------------------------- Genesis: SN76489 PSG + YM2612
  static struct PsgSyn { float left[4]; uint8_t flip[3]; uint16_t lfsr; } ps;
  static float psgAmp[16];

  struct FmOp { uint32_t ph, step; float e, att, amp, tl; uint8_t stage; /*0 off 1 atk 2 d1 3 d2 4 rel*/ float tAtk, dbD1, dbD2, dbRel, sl; };
  struct FmCh { FmOp op[4]; float fb0, fb1; uint8_t alg, fb; };
  static FmCh fm[6];
  static float sinTab[1024], dbRate[32], atkTime[32];
  static bool fmTabs = false;
  static float dacLast = 0;

  static void fmInitTables() {
    for (int i = 0; i < 1024; i++) sinTab[i] = sinf(i * (6.28318530718f / 1024.0f));
    for (int r = 0; r < 32; r++) {
      dbRate[r]  = r ? 96.0f / (0.0035f * powf(2.0f, (31 - r) * 0.5f)) : 0.0f;
      atkTime[r] = r ? 0.002f * powf(2.0f, (31 - r) * 0.5f) : 1e9f;
    }
    for (int v = 0; v < 15; v++) psgAmp[v] = powf(10.0f, -2.0f * v / 20.0f);
    psgAmp[15] = 0.0f;
    fmTabs = true;
  }

  static void fmSetup() {
    using namespace AudioEv;
    static const uint8_t opOff[4] = { 0x00, 0x08, 0x04, 0x0C };      // operator 1..4 register offsets
    for (int ch = 0; ch < 6; ch++) {
      const int bank = ch / 3, c = ch % 3;
      FmCh &f = fm[ch];
      const uint8_t *R = ym.reg[bank];
      const int fnum = R[0xA0 + c] | ((R[0xA4 + c] & 7) << 8), blk = (R[0xA4 + c] >> 3) & 7;
      const float hz = fnum * 0.025401f * (float)(1 << blk);
      f.alg = R[0xB0 + c] & 7; f.fb = (R[0xB0 + c] >> 3) & 7;
      for (int o = 0; o < 4; o++) {
        FmOp &p = f.op[o]; const int a = opOff[o] + c;
        const int mul = R[0x30 + a] & 15;
        p.step = (uint32_t)((double)(hz * (mul ? (float)mul : 0.5f)) * 4294967296.0 / SRATE);
        p.tl = powf(10.0f, -0.75f * (R[0x40 + a] & 127) / 20.0f);
        p.tAtk = atkTime[R[0x50 + a] & 31];
        p.dbD1 = dbRate[R[0x60 + a] & 31]; p.dbD2 = dbRate[R[0x70 + a] & 31];
        p.dbRel = dbRate[((R[0x80 + a] & 15) << 1) | 1];
        const int sl = R[0x80 + a] >> 4; p.sl = sl == 15 ? 93.0f : sl * 3.0f;
        const bool key = ym.keyOn[ch][o];
        if (key) {
          if ((ym.keyEv[ch] & 1) || p.stage == 0 || p.stage == 4) { p.stage = 1; p.e = 0; p.att = 0; p.ph = 0; }
        } else if (p.stage != 0 && p.stage != 4) {
          p.att = p.stage == 1 ? (p.e > 0.001f ? -20.0f * log10f(p.e) : 96.0f) : p.att;
          p.stage = 4;
        }
      }
      ym.keyEv[ch] = 0;
    }
  }

  static void fmEnv(float dt) {
    for (int ch = 0; ch < 6; ch++) for (int o = 0; o < 4; o++) {
      FmOp &p = fm[ch].op[o];
      switch (p.stage) {
        case 1: p.e += dt / p.tAtk; if (p.e >= 1.0f) { p.e = 1.0f; p.att = 0; p.stage = 2; } break;
        case 2: p.att += dt * p.dbD1; if (p.att >= p.sl) { p.att = p.sl; p.stage = 3; } break;
        case 3: p.att += dt * p.dbD2; if (p.att > 96.0f) p.att = 96.0f; break;
        case 4: p.att += dt * p.dbRel; if (p.att >= 96.0f) { p.att = 96.0f; p.stage = 0; } break;
        default: break;
      }
      p.amp = p.stage == 0 ? 0.0f : (p.stage == 1 ? p.e : expf(-p.att * 0.11512925f)) * p.tl;
    }
  }

  static inline float fmSin(uint32_t ph, float modCycles) {
    return sinTab[((ph >> 22) + (int32_t)(modCycles * 1024.0f)) & 1023];
  }

  static void genGenesis(float *o, size_t n) {
    using namespace AudioEv;
    if (!fmTabs) fmInitTables();
    fmSetup();
    if (ps.lfsr == 0) ps.lfsr = 0x8000;
    static const float PSGCLK = 3579545.0f;
    const bool dacOn = ym.reg[0][0x2B] & 0x80;
    const uint16_t dacN = ym.dacN;
    if (dacN) dacLast = (ym.dac[dacN - 1] - 128) * (1.0f / 128.0f);
    const float dacStep = dacN ? (float)dacN / (float)n : 0.0f;
    float dacPos = 0.0f;
    float noisePer = 0; { const uint8_t nr = psg.noise & 3; noisePer = (nr == 3 ? (psg.tone[2] ? psg.tone[2] : 1) : (16 << nr)) * 32.0f * (SRATE / PSGCLK); }
    const bool white = psg.noise & 4;

    for (size_t i = 0; i < n; i++) {
      float sum = 0;
      // ---- PSG (3 tones + noise)
      for (int c = 0; c < 3; c++) {
        const float a = psgAmp[psg.vol[c] & 15];
        if (a <= 0.0f) continue;
        const int per = psg.tone[c] ? psg.tone[c] : 1;
        if (per <= 1) { sum += a * 0.2f; continue; }
        uint8_t &fl = ps.flip[c];
        const float v = boxAvg(ps.left[c], per * 16.0f * (SRATE / PSGCLK), [&] { return fl ? 1.0f : -1.0f; }, [&] { fl ^= 1; });
        sum += v * a * 0.2f;
      }
      { const float a = psgAmp[psg.vol[3] & 15];
        if (a > 0.0f) {
          const float v = boxAvg(ps.left[3], noisePer, [&] { return (ps.lfsr & 1) ? 1.0f : -1.0f; },
                                 [&] { const uint16_t fb = white ? (uint16_t)(__builtin_parity(ps.lfsr & 0x0009)) : (uint16_t)(ps.lfsr & 1); ps.lfsr = (ps.lfsr >> 1) | (fb << 15); });
          sum += v * a * 0.18f;
        } }

      // ---- YM2612 (envelopes at ~control rate)
      if ((i & 15) == 0) fmEnv(16.0f / SRATE);
      for (int ch = 0; ch < 6; ch++) {
        FmCh &f = fm[ch];
        if (ch == 5 && dacOn) continue;
        FmOp *op = f.op;
        if (!op[0].amp && !op[1].amp && !op[2].amp && !op[3].amp) { for (int k = 0; k < 4; k++) op[k].ph += op[k].step; continue; }
        const float fbm = f.fb ? (f.fb0 + f.fb1) * 0.5f * ((float)(1 << f.fb) * (1.0f / 64.0f)) : 0.0f;
        const float o1 = fmSin(op[0].ph, fbm) * op[0].amp;
        f.fb1 = f.fb0; f.fb0 = o1;
        const float MI = 2.0f;                       // modulation depth: output 1.0 -> 2 cycles
        float out;
        switch (f.alg) {
          case 0: { const float o2 = fmSin(op[1].ph, o1 * MI) * op[1].amp, o3 = fmSin(op[2].ph, o2 * MI) * op[2].amp; out = fmSin(op[3].ph, o3 * MI) * op[3].amp; } break;
          case 1: { const float o2 = fmSin(op[1].ph, 0) * op[1].amp, o3 = fmSin(op[2].ph, (o1 + o2) * MI) * op[2].amp; out = fmSin(op[3].ph, o3 * MI) * op[3].amp; } break;
          case 2: { const float o2 = fmSin(op[1].ph, 0) * op[1].amp, o3 = fmSin(op[2].ph, o2 * MI) * op[2].amp; out = fmSin(op[3].ph, (o1 + o3) * MI) * op[3].amp; } break;
          case 3: { const float o2 = fmSin(op[1].ph, o1 * MI) * op[1].amp, o3 = fmSin(op[2].ph, 0) * op[2].amp; out = fmSin(op[3].ph, (o2 + o3) * MI) * op[3].amp; } break;
          case 4: { const float o2 = fmSin(op[1].ph, o1 * MI) * op[1].amp, o3 = fmSin(op[2].ph, 0) * op[2].amp; out = (o2 + fmSin(op[3].ph, o3 * MI) * op[3].amp) * 0.7f; } break;
          case 5: { out = (fmSin(op[1].ph, o1 * MI) * op[1].amp + fmSin(op[2].ph, o1 * MI) * op[2].amp + fmSin(op[3].ph, o1 * MI) * op[3].amp) * 0.55f; } break;
          case 6: { const float o2 = fmSin(op[1].ph, o1 * MI) * op[1].amp; out = (o2 + fmSin(op[2].ph, 0) * op[2].amp + fmSin(op[3].ph, 0) * op[3].amp) * 0.55f; } break;
          default: { out = (o1 + fmSin(op[1].ph, 0) * op[1].amp + fmSin(op[2].ph, 0) * op[2].amp + fmSin(op[3].ph, 0) * op[3].amp) * 0.45f; } break;
        }
        for (int k = 0; k < 4; k++) op[k].ph += op[k].step;
        sum += out * 0.22f;
      }
      // ---- DAC (channel 6 PCM): spread this frame's $2A writes across the frame
      if (dacOn) {
        if (dacN) { uint16_t idx = (uint16_t)dacPos; if (idx >= dacN) idx = dacN - 1; dacLast = (ym.dac[idx] - 128) * (1.0f / 128.0f); dacPos += dacStep; }
        sum += dacLast * 0.3f;
      }
      o[i] = sum;
    }
    ym.dacN = 0;
  }

  // ---------------------------------------------------------------- output stage
  static float hpX = 0, hpY = 0, lpY = 0, fade = 0;

  static void resetState() {
    memset(&ns, 0, sizeof(ns)); memset(gs, 0, sizeof(gs)); memset(&ts, 0, sizeof(ts));
    memset(&ps, 0, sizeof(ps)); memset(fm, 0, sizeof(fm));
    hpX = hpY = lpY = 0; fade = 0; dacLast = 0; frameRemainder = 0;
  }

  static void frame(uint8_t system, uint32_t frameUs) {
    if (!EmuAudio::isReady()) return;
    if (seenEpoch != AudioEv::epoch) { resetState(); seenEpoch = AudioEv::epoch; }
    uint32_t total = (uint64_t)EMU32_AUDIO_SAMPLE_RATE * frameUs + frameRemainder;
    size_t count = total / 1000000UL; frameRemainder = total % 1000000UL;
    if (count > MAXN) count = MAXN;
    if (count == 0) return;

    switch (system) {
      case SYS_NES:     genNes(fbuf, count); break;
      case SYS_GB:      genGb(0, fbuf, count); break;
      case SYS_GBC:     genGb(1, fbuf, count); break;
      case SYS_A2600:   genTia(fbuf, count); break;
      case SYS_GENESIS: genGenesis(fbuf, count); break;
      default: for (size_t i = 0; i < count; i++) fbuf[i] = 0; break;
    }

    static const float hpR = 1.0f - 6.2831853f * 25.0f / SRATE;                       // ~25 Hz DC blocker
    static const float lpA = 1.0f - expf(-6.2831853f * EMU32_LP_HZ / SRATE);
    static const float fadeInc = 1.0f / (0.03f * SRATE);                                // 30 ms fade-in after reset
    for (size_t i = 0; i < count; i++) {
      const float x = fbuf[i];
      const float y = x - hpX + hpR * hpY; hpX = x; hpY = y;
      lpY += lpA * (y - lpY);
      if (fade < 1.0f) fade += fadeInc;
      float v = lpY * (fade < 1.0f ? fade : 1.0f);
      if (v > 0.7f) v = 0.7f + 0.3f * tanhf((v - 0.7f) * 3.3333f);                 // soft limiter instead of hard clip
      else if (v < -0.7f) v = -0.7f - 0.3f * tanhf((-v - 0.7f) * 3.3333f);
      mix[i] = (int16_t)(v * EMU32_SYNTH_PEAK);
    }
    EmuAudio::writeMono(mix, count);
  }
}
