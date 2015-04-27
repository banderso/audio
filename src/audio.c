// audio.c

#include <AudioToolbox/AudioToolbox.h>
#include <AudioToolbox/AudioFile.h>
#include <CoreFoundation/CFURL.h>

#include "../lib/noise/src/noise.h"

#define internal static
#define global static
#define local static

global const float kTau = 2.0f * M_PI;
global const int kNumberBuffers = 3;
global const int kBufferSizeInFrames = 512;
global const int kSampleRate = 44100;

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

typedef struct AQPlayerState_ {
  AudioStreamBasicDescription  mDataFormat;
  AudioQueueRef                mQueue;
  AudioQueueBufferRef          mBuffers[kNumberBuffers];
  AudioFileID                  mAudioFile;
  UInt32                       bufferByteSize;
  SInt64                       mCurrentPacket;
  UInt32                       mNumPacketsToRead;
  AudioStreamPacketDescription *mPacketDescs;
  bool                         mIsRunning;
} AQPlayerState;

internal float SineWave(float amplitude, float period, float phase) {
  return amplitude * sinf((kTau / period) * phase);
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

internal float NoiseWave(float amplitude, float period, float phase) {
  return perlin(amplitude, period, phase);
}

global float gVolume = 1.0f;
global float gPhase = 0.0f;
global float gFStep = 0.0f;
internal void SineQueueCallback(void *aqData,
                                AudioQueueRef outAQ,
                                AudioQueueBufferRef outBuffer) {
  AQPlayerState *pAqData = (AQPlayerState *)aqData;
  if (!pAqData->mIsRunning) return;
  
  SInt16 *coreAudioBuffer = (SInt16 *)outBuffer->mAudioData;
  outBuffer->mAudioDataByteSize = kBufferSizeInFrames * pAqData->mDataFormat.mBytesPerFrame;

  float volumeL = gVolume;
  float volumeR = gVolume;
  float phaseL = gPhase;
  float phaseR = gPhase;
  float fStepL = gFStep;
  float fStepR = gFStep;

  for (int s = 0; s < kBufferSizeInFrames * 2; s +=2) {
    float sampleL = NoiseWave(volumeL, kTau, phaseL);//SawWave(volumeL, kTau, phaseL) + SineWave(volumeL, 128.0f * M_PI, phaseL);
    float sampleR = NoiseWave(volumeR, kTau, phaseR);//SawWave(volumeR, kTau, phaseR) + SineWave(volumeL, 128.0f * M_PI, phaseL);

    short sampleIL = (short)(sampleL * 32767.0f);
    short sampleIR = (short)(sampleR * 32767.0f);

    coreAudioBuffer[s] = sampleIL;
    coreAudioBuffer[s + 1] = sampleIR;

    phaseL += fStepL;
    phaseR += fStepR;
  }

  gPhase = fmodf(phaseL, 65536.0f);

  AudioQueueEnqueueBuffer(outAQ, outBuffer, 0, NULL);
}

internal void SetWave(float frequency, float volume) {
  gVolume = volume;
  gFStep = frequency / kSampleRate;
}

internal float CalcFrequencyFromNote(MusicalNote semiTones, SInt32 octave) {
  semiTones += (12 * (octave - 4));
  float root = 440.0f;
  return powf(2.0f, (float)semiTones / 12.0f) * root;
}

internal void PlaySine(void) {
  AQPlayerState aqData = {};
  aqData.mDataFormat.mSampleRate = kSampleRate;
  aqData.mDataFormat.mFormatID = kAudioFormatLinearPCM;
  aqData.mDataFormat.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
  aqData.mDataFormat.mFramesPerPacket = 1;
  aqData.mDataFormat.mChannelsPerFrame = 2;
  aqData.mDataFormat.mBytesPerPacket = aqData.mDataFormat.mBytesPerFrame = sizeof(SInt16) * 2;
  aqData.mDataFormat.mBitsPerChannel = 16;
  aqData.mDataFormat.mReserved = 0;

  SetWave(CalcFrequencyFromNote(C, 4), 0.5f);

  OSStatus result = AudioQueueNewOutput(&aqData.mDataFormat,
                                        &SineQueueCallback,
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

    SineQueueCallback(&aqData, aqData.mQueue, aqData.mBuffers[i]);
  }

  result = AudioQueueSetParameter(aqData.mQueue, kAudioQueueParam_Volume, 1.0f);
  if (result) return;

  result = AudioQueueStart(aqData.mQueue, NULL);
  if (result) return;

  do {
    CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.25f, false);
  } while (aqData.mIsRunning);

  CFRunLoopRunInMode(kCFRunLoopDefaultMode, 1.0f, false);

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
    PlaySine();
  } else {
    PlayFile(args[1]);
  }
  
  return 0;
}
