/*
Copyright 2019 Tomoaki Itoh
This software is released under the MIT License, see LICENSE.txt.
//*/

#include "LCWDelay.h"
#include "LCWCommon.h"
#include "LCWDelayFirParamTable.h"

#define LCW_DELAY_INPUT_BITS (5)
#define LCW_DELAY_INPUT_SIZE (1 << LCW_DELAY_INPUT_BITS)
#define LCW_DELAY_INPUT_MASK (LCW_DELAY_INPUT_SIZE - 1)

#define LCW_DELAY_BUFFER_DEC(buf) (((buf).pointer - 1) & (buf).mask)

typedef struct {
    SQ7_24 *buffer;
    uint32_t size;
    uint32_t mask;
    int32_t pointer;
} LCWDelayBuffer;

typedef struct {
    SQ15_16 current;
    SQ15_16 sampling;
    SQ7_24 step;
    int32_t delaySize;
} LCWDelayBlock;

static SQ7_24 delayInputArray[LCW_DELAY_INPUT_SIZE];

static LCWDelayBuffer delayInputBuffer = {
    delayInputArray,
    LCW_DELAY_INPUT_SIZE,
    LCW_DELAY_INPUT_MASK,
    0
};
static LCWDelayBuffer delaySamplingBuffer = {
    (SQ7_24 *)0,
    LCW_DELAY_SAMPLING_SIZE,
    LCW_DELAY_SAMPLING_MASK,
    0
};

// memo: バッファサイズのみ変更するBPM同期ディレイなので step = 1.0 で固定
static const SQ7_24 delayStep = LCW_SQ7_24( 1.f );

static SQ7_24 delaySizeLpfParam = LCW_SQ7_24( 0.9999f );
static uint32_t delaySize = LCW_DELAY_SAMPLING_RATE;

static LCWDelayBlock delayBlock;

static SQ7_24 resampling(const LCWDelayBuffer *p, int32_t i, const SQ3_12 *fir, int32_t n)
{
    // memo:
    // step = 1.0 固定なので、delaySizeに端数を設けない限りは補間処理不要

    const uint32_t mask = p->mask;
#if (0)
    int64_t ret = 0;
    for (int32_t j=0; j<n; j++) {
        ret += ( (int64_t)(p->buffer[(i + j) & mask]) * fir[j] );
    }

    return (SQ7_24)( ret >> LCW_DELAY_FIR_VALUE_BITS );
#else
    return p->buffer[(i + (n >> 1)) & mask];
#endif
}

static void convergeDelaySize(LCWDelayBlock *block, uint32_t dst, SQ7_24 param)
{
    const uint32_t src = block->delaySize;
    if ( src < dst ) {
        const uint32_t diff = dst - src;
        const uint32_t tmp = (uint32_t)( ((uint64_t)diff * param) >> 24 );
        
        block->delaySize = dst - tmp;
    }
    else {
        const uint32_t diff = src - dst;
        const uint32_t tmp = (uint32_t)( ((uint64_t)diff * param) >> 24 );
        
        block->delaySize = dst + tmp;
    }
}

void LCWDelayInit(int32_t *delayBuffer)
{
    delaySamplingBuffer.buffer = delayBuffer;
}

void LCWDelayReset(void)
{
/*
    delayInputBuffer.pointer = 0;
    for (int32_t i=0; i<LCW_DELAY_INPUT_SIZE; i++) {
        delayInputBuffer.buffer[i] = 0;
    }

    delaySamplingBuffer.pointer = 0;
    for (int32_t i=0; i<LCW_DELAY_SAMPLING_SIZE; i++) {
        delaySamplingBuffer.buffer[i] = 0;
    }
*/
    delayBlock.current = LCW_SQ15_16( LCW_DELAY_FIR_TAP + 1 );
    delayBlock.sampling = LCW_SQ15_16( LCW_DELAY_FIR_TAP + 1 );
    delayBlock.step = delayStep;
    delayBlock.delaySize = delaySize;
}

void LCWDelayUpdate(const uint32_t delaySamples)
{
    // step = 1.0 固定
    uint32_t offset = (uint32_t)(delayBlock.sampling >> 16) >> 1;
    delaySize = delaySamples - offset;
}

void LCWDelayInput(int32_t fxSend)
{
    delayInputBuffer.pointer = LCW_DELAY_BUFFER_DEC(delayInputBuffer);
    delayInputBuffer.buffer[delayInputBuffer.pointer] = (SQ7_24)fxSend;
    delayBlock.current += LCW_SQ15_16( 1.0 );

    if ( delayBlock.sampling <= delayBlock.current ) {
        convergeDelaySize( &delayBlock, delaySize, delaySizeLpfParam );
    }

    const SQ15_16 step = (SQ15_16)( delayBlock.step >> 8 );
    while ( delayBlock.sampling <= delayBlock.current ) {

        // pos = current - fir_tap
        const int32_t i = delayInputBuffer.pointer - LCW_DELAY_FIR_TAP + (int32_t)(delayBlock.current >> 16);

        const SQ3_12 *fir = gLcwDelayFirTable[ (delayBlock.current >> (16 - LCW_DELAY_FIR_TABLE_BITS)) & LCW_DELAY_FIR_TABLE_MASK ];
        const SQ7_24 sample = resampling( &delayInputBuffer, i, fir, LCW_DELAY_FIR_TAP );

        delaySamplingBuffer.pointer = LCW_DELAY_BUFFER_DEC(delaySamplingBuffer);
        delaySamplingBuffer.buffer[delaySamplingBuffer.pointer] = sample;
        delayBlock.current -= step;
    }
}

int32_t LCWDelayOutput(void)
{
    // memo:
    // currentがsamplingを超えないことを想定している
    const SQ15_16 tmp = delayBlock.sampling - delayBlock.current;
    const SQ15_16 offset = ((tmp << 8) / (delayBlock.step >> 12)) << 4;
    const LCWDelayBuffer *p = &delaySamplingBuffer;
    const int32_t i = p->pointer + delayBlock.delaySize - (LCW_DELAY_FIR_TAP >> 1) + (int32_t)(offset >> 16);
    const SQ3_12 *fir = gLcwDelayFirTable[ (offset >> (16 - LCW_DELAY_FIR_TABLE_BITS)) & LCW_DELAY_FIR_TABLE_MASK ];

    return resampling( p, i, fir, LCW_DELAY_FIR_TAP );
}
