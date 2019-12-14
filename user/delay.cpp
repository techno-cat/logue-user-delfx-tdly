/*
Copyright 2019 Tomoaki Itoh
This software is released under the MIT License, see LICENSE.txt.
//*/

#include "userdelfx.h"
#include "LCWDelay.h"

#define LCW_DELAY_TIME_PARAMS (10)

static __sdram int32_t s_delay_ram[LCW_DELAY_SAMPLING_SIZE];
static const float s_fs_recip = 1.f / 48000.f;

static float s_mix;
static float s_depth;
static uint32_t s_time;

static const float delayTimeParams[LCW_DELAY_TIME_PARAMS] = {
    1.0   ,
    // ---------------
    0.75  , //   3/4
    0.6666, //
    0.5   , //   2/4
    // ---------------
    0.375 , //   3/8
    0.3333, //
    0.25  , //   2/8
    // ---------------
    0.1875, //   3/16
    0.1666, //
    0.125   //   1/16
};

void DELFX_INIT(uint32_t platform, uint32_t api)
{
  LCWDelayInit( s_delay_ram );
  LCWDelayReset();

  s_mix = 0.5f;
  s_depth = 0.f;
  s_time = 0;
}

void DELFX_PROCESS(float *xn, uint32_t frames)
{
  float bpm = fx_get_bpmf();
  bpm = clampfsel( (float)LCW_DELAY_BPM_MIN, bpm, (float)LCW_DELAY_BPM_MAX );
  float delayTime = delayTimeParams[s_time];
  // bps = bpm / 60
  // n = bps / delayTime
  // delaySamples = samplingRate / n
  //              = (samplingRate * delayTime) / bps
  //              = (samplingRate * delayTime * 60) / bpm
  LCWDelayUpdate( (uint32_t)((LCW_DELAY_SAMPLING_RATE * delayTime * 60.f) / bpm) );

  float * __restrict x = xn;
  const float * x_e = x + 2*frames;

  const float dry = 1.f - s_mix;
  const float wet = s_mix;
  
  // todo: L/Rで分ける
  for (; x != x_e; ) {
    float xL = *x;
    //float xR = dry * (*x);
    float wL = LCWDelayOutput() / (float)(1 << 24);

    float fbL = wL * s_depth;
    LCWDelayInput( (int32_t)((xL - fbL) * (1 << 24)) );

    float yL = fx_sat_cubicf( (dry * xL) + (wet * wL) );

    *(x++) = yL;
    *(x++) = yL;
  }
}

void DELFX_PARAM(uint8_t index, int32_t value)
{
  const float valf = q31_to_f32(value);
  switch (index) {
  case k_user_delfx_param_time:
    s_time = clipmaxu32( (uint32_t)(valf * (LCW_DELAY_TIME_PARAMS - 1)), (LCW_DELAY_TIME_PARAMS - 1) );
    break;
  case k_user_delfx_param_depth:
    s_depth = valf;
    break;
  case k_user_delfx_param_shift_depth:
    // Rescale to add notch around 0.5f
    s_mix = (valf <= 0.49f) ? 1.02040816326530612244f * valf : (valf >= 0.51f) ? 0.5f + 1.02f * (valf-0.51f) : 0.5f;
    break;
  default:
    break;
  }
}
