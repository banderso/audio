// audio.c

#include <AudioToolbox/AudioToolbox.h>
#include <AudioToolbox/AudioFile.h>
#include <CoreFoundation/CFURL.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

#include "../lib/noise/src/noise.h"
#include "../lib/input/src/input.h"

#define internal static
#define global static
#define local static

typedef uint8_t u8;
typedef int8_t i8;
typedef uint16_t u16;
typedef int16_t i16;
typedef uint32_t u32;
typedef int32_t i32;
typedef uint64_t u64;
typedef int64_t i64;
typedef float f32;
typedef double f64;

global const float kTau = 2.0f * M_PI;
enum {
  kNumberBuffers = 4,
  kBufferSizeInFrames = 512,
  kSampleRate = 44100,
};

typedef enum MusicalNote_ {
  A = 0,
  Asharp,
  B,
  C,
  Csharp,
  D,
  Dsharp,
  E,
  F,
  Fsharp,
  G,
  Gsharp,

  Bflat = Asharp,
  Dflat = Csharp,
  Eflat = Dsharp,
  Gflat = Fsharp,
  Aflat = Gsharp
} MusicalNote;

typedef enum ADSRState_ {
  IDLE = 0,
  ATTACK,
  DECAY,
  SUSTAIN,
  RELEASE
} ADSRState;

typedef struct ADSR_ {
  ADSRState state;
  bool gate;

  f32 targetRatioA;
  
  f32 attackRate;
  f32 attackCoef;
  f32 attackBase;

  f32 targetRatioDR;
  
  f32 decayRate;
  f32 decayCoef;
  f32 decayBase;
  
  f32 releaseRate;
  f32 releaseCoef;
  f32 releaseBase;

  f32 sustainLevel;

  f32 output;
} ADSR, *pADSR;

typedef struct AQPlayerState_ {
  AudioStreamBasicDescription  mDataFormat;
  AudioQueueRef                mQueue;
  AudioQueueBufferRef          mBuffers[kNumberBuffers];
  AudioFileID                  mAudioFile;
  UInt32                       bufferByteSize;
  SInt64                       mCurrentPacket;
  UInt32                       mNumPacketsToRead;
  AudioStreamPacketDescription *mPacketDescs;
  pADSR                        adsr;
  bool                         mIsRunning;
} AQPlayerState;

global struct termios gOriginalTermios;
internal void reset_terminal_mode(void) {
  tcsetattr(0, TCSANOW, &gOriginalTermios);
}

internal void set_raw_terminal_mode(void) {
  struct termios newTermios;

  // store termios settings for restoring later
  tcgetattr(0, &gOriginalTermios);
  memcpy(&newTermios, &gOriginalTermios, sizeof(newTermios));

  // restore orignal termios at exit
  atexit(reset_terminal_mode);

  // make the termios raw
  cfmakeraw(&newTermios);
  tcsetattr(0, TCSANOW, &newTermios);
}

internal int kbhit(void) {
  struct timeval tv = { 0L, 0L};
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  return select(1, &fds, NULL, NULL, &tv);
}

internal u8 get_key(void) {
  ssize_t r = -1;
  u8 c = 0;
  if ((r = read(0, &c, sizeof(c))) < 0) {
    return 0;
  } else {
    return c;
  }
}

internal f32 calcCoef(f32 rate, f32 targetRatio) {
    return expf(-logf((1.0f + targetRatio) / targetRatio) / rate);
}

internal void adsr_attack(pADSR adsr, f32 rate) {
  adsr->attackRate = rate;
  adsr->attackCoef = calcCoef(rate, adsr->targetRatioA);
  adsr->attackBase = (1.0f + adsr->targetRatioA) * (1.0f - adsr->attackCoef);
}

internal void adsr_decay(pADSR adsr, f32 rate) {
  adsr->decayRate = rate;
  adsr->decayCoef = calcCoef(rate, adsr->targetRatioDR);
  adsr->decayBase = (adsr->sustainLevel - adsr->targetRatioDR) * (1.0f - adsr->decayCoef);
}

internal void adsr_release(pADSR adsr, f32 rate) {
  adsr->releaseRate = rate;
  adsr->releaseCoef = calcCoef(rate, adsr->targetRatioDR);
  adsr->releaseBase = -adsr->targetRatioDR * (1.0f - adsr->releaseCoef);
}

internal void adsr_sustain(pADSR adsr, f32 level) {
  adsr->sustainLevel = level;
  adsr->decayBase = (adsr->sustainLevel - adsr->targetRatioDR) * (1.0f - adsr->decayCoef);
}

internal void adsr_targetRatioA(pADSR adsr, f32 ratio) {
  if (ratio < 0.000000001f) {
    ratio = 0.000000001f; // -180 dB
  }
  adsr->targetRatioA = ratio;
  adsr->attackBase = (1.0f + adsr->targetRatioA) * (1.0f - adsr->attackCoef);
}

internal void adsr_targetRatioDR(pADSR adsr, f32 ratio) {
  if (ratio < 0.000000001f) {
    ratio = 0.000000001f; // -180 dB
  }
  adsr->targetRatioDR = ratio;
  adsr->decayBase = (adsr->sustainLevel - adsr->targetRatioDR) * (1.0f - adsr->decayCoef);
  adsr->releaseBase = -adsr->targetRatioDR * (1.0f - adsr->releaseCoef);
}

internal f32 adsr_process(pADSR adsr) {
  f32 output = adsr->output;
  switch (adsr->state) {
    case IDLE: break;
    case ATTACK: {
      output = adsr->attackBase + output * adsr->attackCoef;
      if (output >= 1.0f) {
        output = 1.0f;
        adsr->state = DECAY;
        printf("adsr: DECAY\n");
      }
    } break;
    case DECAY: {
      output = adsr->decayBase + output * adsr->decayCoef;
      if (output <= adsr->sustainLevel) {
        output = adsr->sustainLevel;
        adsr->state = SUSTAIN;
        printf("adsr: SUSTAIN\n");
      }
    } break;
    case SUSTAIN: break;
    case RELEASE: {
      output = adsr->releaseBase + output * adsr->releaseCoef;
      if (output <= 0.0f) {
        output = 0.0f;
        adsr->state = IDLE;
        printf("adsr: IDLE\n");
      }
    } break;
  }
  adsr->output = output;
  //printf("adsr: %f\n", output);
  return output;
}

internal void adsr_gate(pADSR adsr) {
  if (!adsr->gate) {
    adsr->state = ATTACK;
    adsr->gate = true;
    printf("adsr: ATTACK\n");
  } else if (adsr->state != IDLE) {
    adsr->state = RELEASE;
    adsr->gate = false;
    printf("adsr: RELEASE\n");
  }
}

internal void adsr_reset(pADSR adsr) {
  adsr->gate = false;
  adsr->state = IDLE;
  adsr->output = 0.0f;
}

internal void adsr_init(pADSR adsr) {
  adsr_reset(adsr);
  adsr_attack(adsr, 0.0f);
  adsr_decay(adsr, 0.0f);
  adsr_release(adsr, 0.0f);
  adsr_sustain(adsr, 1.0f);
  adsr_targetRatioA(adsr, 0.3f);
  adsr_targetRatioDR(adsr, 0.0001f);
}

internal float SineWave(float amplitude, float period, float phase) {
  return amplitude * sinf((kTau / period) + phase);
}

internal float TringleWave(float amplitude, float period, float phase) {
  return ((2.0f * amplitude) / M_PI) * asinf(sinf((kTau / period) * phase));
}

internal float cotf(float x) {
  return 1.0f / tanf(x);
}

internal float SawWave(float amplitude, float period, float phase) {
  return -((2.0f * amplitude) / M_PI) * atanf(cotf((phase * M_PI) / period));
}

internal float PerlinNoise(float amplitude, float period, float phase) {
  return perlin(amplitude, period, phase * 0.05f);
}

internal float WhiteNoise(float amplitude, float period, float phase) {
  return gaussian(0.0f, 1.0f);
}

internal float BrownianNoise(float amplitude, float period, float phase) {
  return brownian(amplitude, period / kTau);
}

global float gVolume = 1.0f;
global float gPhase = 0.0f;
global float gFStep = 0.0f;
internal void SynthCallback(void *aqData,
                            AudioQueueRef outAQ,
                            AudioQueueBufferRef outBuffer) {
  AQPlayerState *pAqData = (AQPlayerState *)aqData;
  if (!pAqData->mIsRunning) return;
  
  SInt16 *coreAudioBuffer = (SInt16 *)outBuffer->mAudioData;
  outBuffer->mAudioDataByteSize = kBufferSizeInFrames * pAqData->mDataFormat.mBytesPerFrame;

  pADSR adsr = pAqData->adsr;

  float volumeL = gVolume;
  float volumeR = gVolume;
  float phaseL = gPhase;
  float phaseR = gPhase;
  float fStepL = gFStep;
  float fStepR = gFStep;

  float envelope = 0.0f;
  for (int s = 0; s < kBufferSizeInFrames * 2; s +=2) {
    if (adsr->gate) {
      envelope = adsr_process(adsr) + 1.0f;
    }
    float sampleL = SineWave(volumeL, fStepL, phaseL) * envelope;
    float sampleR = SineWave(volumeR, fStepR, phaseR) * envelope;

    short sampleIL = (short)(sampleL * 32767.0f);
    short sampleIR = (short)(sampleR * 32767.0f);

    coreAudioBuffer[s] = sampleIL;
    coreAudioBuffer[s + 1] = sampleIR;

    phaseL += fStepL;
    phaseR += fStepR;
  }

  gPhase = fmodf(phaseL, kTau);

  AudioQueueEnqueueBuffer(outAQ, outBuffer, 0, NULL);
}

internal void SetWave(float frequency, float volume) {
  gVolume = volume;
  gFStep = kTau * frequency / kSampleRate;
}

internal float CalcFrequencyFromNote(MusicalNote semiTones, SInt32 octave) {
  semiTones += (12 * (octave - 4));
  float root = 440.0f;
  return powf(2.0f, (float)semiTones / 12.0f) * root;
}

internal void PlaySynth(void) {
  ADSR adsr = {0};
  adsr_init(&adsr);
  adsr_attack(&adsr, 0.1f * kSampleRate);
  adsr_decay(&adsr, 0.3f * kSampleRate);
  adsr_sustain(&adsr, 0.8f);
  adsr_release(&adsr, 5.0f * kSampleRate);
  
  AQPlayerState aqData = {0};
  aqData.mDataFormat.mSampleRate = kSampleRate;
  aqData.mDataFormat.mFormatID = kAudioFormatLinearPCM;
  aqData.mDataFormat.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
  aqData.mDataFormat.mFramesPerPacket = 1;
  aqData.mDataFormat.mChannelsPerFrame = 2;
  aqData.mDataFormat.mBytesPerPacket = aqData.mDataFormat.mBytesPerFrame = sizeof(SInt16) * 2;
  aqData.mDataFormat.mBitsPerChannel = 16;
  aqData.mDataFormat.mReserved = 0;
  
  aqData.adsr = &adsr;

  SetWave(CalcFrequencyFromNote(C, 4), 0.5f);

  OSStatus result = AudioQueueNewOutput(&aqData.mDataFormat,
                                        &SynthCallback,
                                        &aqData,
                                        CFRunLoopGetCurrent(),
                                        kCFRunLoopCommonModes,
                                        0,
                                        &aqData.mQueue);
  if (result) return;

  aqData.mIsRunning = true;
  aqData.bufferByteSize = kBufferSizeInFrames * aqData.mDataFormat.mBytesPerFrame;
  for (int i = 0; i < kNumberBuffers; i++) {
    result = AudioQueueAllocateBuffer(aqData.mQueue,
                                      aqData.bufferByteSize,
                                      &aqData.mBuffers[i]);
    if (result) return;
    //aqData.mBuffers[i]->mAudioDataByteSize = aqData.bufferByteSize;

    SynthCallback(&aqData, aqData.mQueue, aqData.mBuffers[i]);
  }

  result = AudioQueueSetParameter(aqData.mQueue, kAudioQueueParam_Volume, 1.0f);
  if (result) return;

  result = AudioQueueStart(aqData.mQueue, NULL);
  if (result) return;

  const u8 gate_key = 'g';
  const u8 esc_key = 27;
  u8 key = 0;
  bool keyIsDown = false;
  //set_raw_terminal_mode();
  const float loopTime = 1.0f / 60.0f;
  input();
  do {
    CFRunLoopRunInMode(kCFRunLoopDefaultMode, loopTime, false);
    /* if (kbhit()) { */
    /*   key = get_key(); */
    /*   if (key == gate_key) { */
    /*     printf("%c", key); */
    /*     fflush(stdout); */
    /*     if (keyIsDown) { */
    /*     } else { */
    /*       adsr_gate(&adsr); */
    /*     } */
    /*   } else if (key == esc_key) { */
    /*     aqData.mIsRunning = false; */
    /*   } else { */
    /*     key = 0; */
    /*   } */
    /* } */
  } while (aqData.mIsRunning);

  CFRunLoopRunInMode(kCFRunLoopDefaultMode, loopTime, false);

  AudioQueueStop(aqData.mQueue, true);
}

internal void HandleOutputBuffer(void *aqData,
                                 AudioQueueRef inAQ,
                                 AudioQueueBufferRef inBuffer) {
  AQPlayerState *pAqData = (AQPlayerState *)aqData;
  if (!pAqData->mIsRunning) return;

  UInt32 numBytesReadFromFile;
  UInt32 numPackets = pAqData->mNumPacketsToRead;
  AudioFileReadPackets(pAqData->mAudioFile,
                       false,
                       &numBytesReadFromFile,
                       pAqData->mPacketDescs,
                       pAqData->mCurrentPacket,
                       &numPackets,
                       inBuffer->mAudioData);

  if (numPackets > 0) {
    inBuffer->mAudioDataByteSize = numBytesReadFromFile;
    AudioQueueEnqueueBuffer(pAqData->mQueue,
                            inBuffer,
                            (pAqData->mPacketDescs ? numPackets : 0),
                            pAqData->mPacketDescs);
    pAqData->mCurrentPacket += numPackets;
  } else {
    AudioQueueStop(pAqData->mQueue, false);
    pAqData->mIsRunning = false;
  }
}

internal void DeriveBufferSize(AudioStreamBasicDescription *ASBDesc,
                               UInt32                      maxPacketSize,
                               Float64                     seconds,
                               UInt32                      *outBufferSize,
                               UInt32                      *outNumPacketsToRead) {
  local const int maxBufferSize = 0x50000;
  local const int minBufferSize = 0x4000;

  if (ASBDesc->mFramesPerPacket != 0) {
    Float64 numPacketsForTime = ASBDesc->mSampleRate / ASBDesc->mFramesPerPacket * seconds;
    *outBufferSize = numPacketsForTime * maxPacketSize;
  } else {
    *outBufferSize = maxBufferSize > maxPacketSize ? maxBufferSize : maxPacketSize;
  }

  if (*outBufferSize > maxBufferSize && *outBufferSize > maxPacketSize) {
    *outBufferSize = maxBufferSize;
  } else {
    if (*outBufferSize < minBufferSize) {
      *outBufferSize = minBufferSize;
    }
  }

  *outNumPacketsToRead = *outBufferSize / maxPacketSize;
}

internal void PlayFile(const char *filePath) {
  AQPlayerState aqData;
  CFURLRef audioFileURL = CFURLCreateFromFileSystemRepresentation(NULL,
                                                                  (const UInt8 *)filePath,
                                                                  strlen(filePath),
                                                                  false);
  OSStatus result = AudioFileOpenURL(audioFileURL,
                                     0x01, //fsRdPerm,
                                     0,
                                     &aqData.mAudioFile);
  if (result) return;
  
  CFRelease(audioFileURL);

  UInt32 dataFormatSize = sizeof(aqData.mDataFormat);
  AudioFileGetProperty(aqData.mAudioFile,
                       kAudioFilePropertyDataFormat,
                       &dataFormatSize,
                       &aqData.mDataFormat);

  AudioQueueNewOutput(&aqData.mDataFormat,
                      &HandleOutputBuffer,
                      &aqData,
                      CFRunLoopGetCurrent(),
                      kCFRunLoopCommonModes,
                      0,
                      &aqData.mQueue);

  UInt32 maxPacketSize;
  UInt32 propertySize = sizeof(maxPacketSize);
  AudioFileGetProperty(aqData.mAudioFile,
                       kAudioFilePropertyPacketSizeUpperBound,
                       &propertySize,
                       &maxPacketSize);

  DeriveBufferSize(&aqData.mDataFormat,
                   maxPacketSize,
                   0.5,
                   &aqData.bufferByteSize,
                   &aqData.mNumPacketsToRead);

  bool isFormatVBR = (aqData.mDataFormat.mBytesPerPacket == 0 ||
                      aqData.mDataFormat.mFramesPerPacket == 0);
  if (isFormatVBR) {
    aqData.mPacketDescs = (AudioStreamPacketDescription *)malloc(aqData.mNumPacketsToRead * sizeof(AudioStreamPacketDescription));
  } else {
    aqData.mPacketDescs = NULL;
  }

  UInt32 cookieSize = sizeof(UInt32);
  bool couldNotGetProperty = AudioFileGetPropertyInfo(aqData.mAudioFile,
                                                      kAudioFilePropertyMagicCookieData,
                                                      &cookieSize,
                                                      NULL);

  if (!couldNotGetProperty && cookieSize) {
    char *magicCookie = (char *)malloc(cookieSize);
    AudioFileGetProperty(aqData.mAudioFile,
                         kAudioFilePropertyMagicCookieData,
                         &cookieSize,
                         magicCookie);

    AudioQueueSetProperty(aqData.mQueue,
                          kAudioQueueProperty_MagicCookie,
                          magicCookie,
                          cookieSize);

    free(magicCookie);
  }

  aqData.mCurrentPacket = 0;
  aqData.mIsRunning = true;
  for (int i = 0; i < kNumberBuffers; ++i) {
    AudioQueueAllocateBuffer(aqData.mQueue,
                            aqData.bufferByteSize,
                            &aqData.mBuffers[i]);

    HandleOutputBuffer(&aqData,
                       aqData.mQueue,
                       aqData.mBuffers[i]);
  }

  Float32 gain = 1.0f;
  AudioQueueSetParameter(aqData.mQueue,
                         kAudioQueueParam_Volume,
                         gain);

  
  AudioQueueStart(aqData.mQueue, NULL);

  do {
    CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.25f, false);
  } while (aqData.mIsRunning);

  CFRunLoopRunInMode(kCFRunLoopDefaultMode, 1.0f, false);

  AudioQueueDispose(aqData.mQueue, true);
  AudioFileClose(aqData.mAudioFile);
  free(aqData.mPacketDescs);
}

int main(int argc, char** args) {

  perlin_init();
  
  if (argc < 2) {
    PlaySynth();
  } else {
    PlayFile(args[1]);
  }
  
  return 0;
}
