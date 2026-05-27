#include "wled.h"

#ifdef ARDUINO_ARCH_ESP32

#include <driver/i2s.h>
#include <driver/adc.h>

#endif

#if defined(ARDUINO_ARCH_ESP32) && (defined(WLED_DEBUG) || defined(SR_DEBUG))
#include <esp_timer.h>
#endif
#define ESPNOW_AUDIO_SYNC_SKIP_PACKET_DECL  // audio_reactive.cpp already declares audioSyncPacket
#include "espnow_audio_sync.h"

/*
 * Usermods allow you to add own functionality to WLED more easily
 * See: https://github.com/wled-dev/WLED/wiki/Add-own-functionality
 *
 * This is an audioreactive v2 usermod.
 * ....
 */

#define FFT_PREFER_EXACT_PEAKS

#if !defined(FFTTASK_PRIORITY)
#define FFTTASK_PRIORITY 1
#endif

#ifdef SR_DEBUG
  #define DEBUGSR_PRINT(x) DEBUGOUT.print(x)
  #define DEBUGSR_PRINTLN(x) DEBUGOUT.println(x)
  #define DEBUGSR_PRINTF(x...) DEBUGOUT.printf(x)
#else
  #define DEBUGSR_PRINT(x)
  #define DEBUGSR_PRINTLN(x)
  #define DEBUGSR_PRINTF(x...)
#endif

#if defined(MIC_LOGGER) || defined(FFT_SAMPLING_LOG)
  #define PLOT_PRINT(x) DEBUGOUT.print(x)
  #define PLOT_PRINTLN(x) DEBUGOUT.println(x)
  #define PLOT_PRINTF(x...) DEBUGOUT.printf(x)
#else
  #define PLOT_PRINT(x)
  #define PLOT_PRINTLN(x)
  #define PLOT_PRINTF(x...)
#endif

#define MAX_PALETTES 3

static volatile bool disableSoundProcessing = false;
static uint8_t audioSyncEnabled = 0;
static bool udpSyncConnected = false;

#define NUM_GEQ_CHANNELS 16

#ifdef ARDUINO_ARCH_ESP32
    #ifndef SR_AGC
    #define SR_AGC 0
    #endif
static float    micDataReal = 0.0f;
static float    multAgc = 1.0f;
static float    sampleAvg = 0.0f;
static float    sampleAgc = 0.0f;
static uint8_t  soundAgc = SR_AGC;
#endif
static float FFT_MajorPeak = 1.0f;
static float FFT_Magnitude = 0.0f;
static bool samplePeak = false;
static bool udpSamplePeak = false;
static unsigned long timeOfPeak = 0;
static uint8_t fftResult[NUM_GEQ_CHANNELS]= {0};

#ifdef UM_AUDIOREACTIVE_DYNAMICS_LIMITER_OFF
static bool limiterOn = false;
#else
static bool limiterOn = true;
#endif
static uint16_t attackTime =  80;
static uint16_t decayTime = 1400;

#ifdef ARDUINO_ARCH_ESP32
static void detectSamplePeak(void);
#endif
static void autoResetPeak(void);
static uint8_t maxVol = 31;
static uint8_t binNum = 8;

#ifdef ARDUINO_ARCH_ESP32
#if !defined(UM_AUDIOREACTIVE_USE_ESPDSP_FFT) && (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32))
#define UM_AUDIOREACTIVE_USE_ARDUINO_FFT
#endif

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 0)
#define UM_AUDIOREACTIVE_USE_ARDUINO_FFT
#endif

#ifdef UM_AUDIOREACTIVE_USE_ARDUINO_FFT
#include <arduinoFFT.h>
#undef UM_AUDIOREACTIVE_USE_INTEGER_FFT
#else
#include "dsps_fft2r.h"
#ifdef FFT_PREFER_EXACT_PEAKS
#include "dsps_wind_blackman_harris.h"
#else
#include "dsps_wind_flat_top.h"
#endif
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3)
#define UM_AUDIOREACTIVE_USE_INTEGER_FFT
#endif
#endif

#if !defined(UM_AUDIOREACTIVE_USE_INTEGER_FFT)
using FFTsampleType = float;
using FFTmathType = float;
#define FFTabs fabsf
#else
using FFTsampleType = int16_t;
using FFTmathType = int32_t;
#define FFTabs abs
#endif
static FFTsampleType* valFFT = nullptr;
#ifdef UM_AUDIOREACTIVE_USE_ARDUINO_FFT
static float* vImag = nullptr;
#endif

static FFTsampleType* windowFFT = nullptr;

#include "audio_source.h"
constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
constexpr int BLOCK_SIZE = 128;

static uint8_t inputLevel = 128;
#ifndef SR_SQUELCH
  static uint8_t soundSquelch = 10;
#else
  static uint8_t soundSquelch = SR_SQUELCH;
#endif
#ifndef SR_GAIN
  static uint8_t sampleGain = 60;
#else
  static uint8_t sampleGain = SR_GAIN;
#endif
static uint8_t FFTScalingMode = 3;

#define AGC_NUM_PRESETS 3
const double agcSampleDecay[AGC_NUM_PRESETS]  = { 0.9994f, 0.9985f, 0.9997f};
const float agcZoneLow[AGC_NUM_PRESETS]       = {      32,      28,      36};
const float agcZoneHigh[AGC_NUM_PRESETS]      = {     240,     240,     248};
const float agcZoneStop[AGC_NUM_PRESETS]      = {     336,     448,     304};
const float agcTarget0[AGC_NUM_PRESETS]       = {     112,     144,     164};
const float agcTarget0Up[AGC_NUM_PRESETS]     = {      88,      64,     116};
const float agcTarget1[AGC_NUM_PRESETS]       = {     220,     224,     216};
const double agcFollowFast[AGC_NUM_PRESETS]   = { 1/192.f, 1/128.f, 1/256.f};
const double agcFollowSlow[AGC_NUM_PRESETS]   = {1/6144.f,1/4096.f,1/8192.f};
const double agcControlKp[AGC_NUM_PRESETS]    = {    0.6f,    1.5f,   0.65f};
const double agcControlKi[AGC_NUM_PRESETS]    = {    1.7f,   1.85f,    1.2f};
const float agcSampleSmooth[AGC_NUM_PRESETS]  = {  1/12.f,   1/6.f,  1/16.f};

static AudioSource *audioSource = nullptr;
static bool useBandPassFilter = false;
static bool useMicFilter = false;

static float fftAddAvg(int from, int to);
void FFTcode(void * parameter);
static void runMicFilter(uint16_t numSamples, FFTsampleType *sampleBuffer);
static void postProcessFFTResults(bool noiseGateOpen, int numberOfChannels);

static TaskHandle_t FFT_Task = nullptr;

static float fftResultPink[NUM_GEQ_CHANNELS] = { 1.70f, 1.71f, 1.73f, 1.78f, 1.68f, 1.56f, 1.55f, 1.63f, 1.79f, 1.62f, 1.80f, 2.06f, 2.47f, 3.35f, 6.83f, 9.55f };

#if defined(WLED_DEBUG) || defined(SR_DEBUG)
static uint64_t fftTime = 0;
static uint64_t sampleTime = 0;
#endif

static float   fftCalc[NUM_GEQ_CHANNELS] = {0.0f};
static float   fftAvg[NUM_GEQ_CHANNELS] = {0.0f};
#ifdef SR_DEBUG
static float   fftResultMax[NUM_GEQ_CHANNELS] = {0.0f};
#endif

constexpr SRate_t SAMPLE_RATE = 22050;
#define FFT_MIN_CYCLE 21

constexpr uint16_t samplesFFT = 512;
constexpr uint16_t samplesFFT_2 = 256;
#ifdef FFT_PREFER_EXACT_PEAKS
#define FFT_DOWNSCALE 0.40f
#else
#define FFT_DOWNSCALE 0.46f
#endif
#define LOG_256  5.54517744f

static float fftAddAvg(int from, int to) {
  FFTmathType result = 0;
  for (int i = from; i <= to; i++) {
    result += valFFT[i];
  }
 #if !defined(UM_AUDIOREACTIVE_USE_INTEGER_FFT)
  result = result * 0.0625;
 #else
  result *= 32;
#endif
  return float(result) / float(to - from + 1);
}

void FFTcode(void * parameter)
{
  DEBUGSR_PRINT("FFT started on core: "); DEBUGSR_PRINTLN(xPortGetCoreID());
#ifdef UM_AUDIOREACTIVE_USE_ARDUINO_FFT
  if (valFFT == nullptr) valFFT = (float*) calloc(samplesFFT, sizeof(float));
  if (vImag == nullptr)  vImag  = (float*) calloc(samplesFFT, sizeof(float));
  if ((valFFT == nullptr) || (vImag == nullptr)) {
    if (valFFT) free(valFFT); valFFT = nullptr;
    if (vImag) free(vImag); vImag = nullptr;
    return;
  }
  ArduinoFFT<float> FFT = ArduinoFFT<float>(valFFT, vImag, samplesFFT, SAMPLE_RATE, true);
#elif !defined(UM_AUDIOREACTIVE_USE_INTEGER_FFT)
  if (valFFT == nullptr) {
    valFFT = (float*)heap_caps_aligned_calloc(16, 2 * samplesFFT, sizeof(float), MALLOC_CAP_8BIT);
    if ((valFFT == nullptr)) return;
  }
  if (windowFFT == nullptr) {
    windowFFT = (float*)heap_caps_aligned_calloc(16, samplesFFT, sizeof(float), MALLOC_CAP_8BIT);
    if ((windowFFT == nullptr)) {
      heap_caps_free(valFFT); valFFT = nullptr;
      return;
    }
  }
  if (dsps_fft2r_init_fc32(NULL, samplesFFT) != ESP_OK) return;
#ifdef FFT_PREFER_EXACT_PEAKS
  dsps_wind_blackman_harris_f32(windowFFT, samplesFFT);
#else
  dsps_wind_flat_top_f32(windowFFT, samplesFFT);
#endif
#else
  if (valFFT == nullptr) valFFT = (int16_t*) heap_caps_aligned_calloc(4, samplesFFT * 2, sizeof(int16_t), MALLOC_CAP_8BIT);
  if (windowFFT == nullptr) windowFFT = (int16_t*) heap_caps_aligned_calloc(4, samplesFFT, sizeof(int16_t), MALLOC_CAP_8BIT);
  float *windowFloat = (float*) heap_caps_aligned_calloc(4, samplesFFT, sizeof(float), MALLOC_CAP_8BIT);
  if (windowFloat == nullptr || windowFFT == nullptr || valFFT == nullptr) {
    if (windowFloat) heap_caps_free(windowFloat);
    if (windowFFT) heap_caps_free(windowFFT); windowFFT = nullptr;
    if (valFFT) heap_caps_free(valFFT); valFFT = nullptr;
    return;
  }
  if (dsps_fft2r_init_sc16(NULL, samplesFFT) != ESP_OK) return;

#ifdef FFT_PREFER_EXACT_PEAKS
  dsps_wind_blackman_harris_f32(windowFloat, samplesFFT);
#else
  dsps_wind_flat_top_f32(windowFloat, samplesFFT);
#endif
  for (int i = 0; i < samplesFFT; i++) {
    windowFFT[i] = (int16_t)(windowFloat[i] * 32767.0f);
  }
  heap_caps_free(windowFloat);
#endif

  const TickType_t xFrequency = FFT_MIN_CYCLE * portTICK_PERIOD_MS;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;) {
    delay(1);

    if (disableSoundProcessing || (audioSyncEnabled & 0x02)) {
      vTaskDelayUntil( &xLastWakeTime, xFrequency);
      continue;
    }

#if defined(WLED_DEBUG) || defined(SR_DEBUG)
    uint64_t start = esp_timer_get_time();
    bool haveDoneFFT = false;
#endif

    if (audioSource) audioSource->getSamples(valFFT, samplesFFT);

#if defined(WLED_DEBUG) || defined(SR_DEBUG)
    if (start < esp_timer_get_time()) {
      uint64_t sampleTimeInMillis = (esp_timer_get_time() - start +5ULL) / 10ULL;
      sampleTime = (sampleTimeInMillis*3 + sampleTime*7)/10;
    }
    start = esp_timer_get_time();
#endif

    xLastWakeTime = xTaskGetTickCount();

    if (useMicFilter) runMicFilter(samplesFFT, valFFT);
    FFTsampleType maxSample = 0;
    for (int i=0; i < samplesFFT; i++) {
	    if ((valFFT[i] <= (INT16_MAX - 1024)) && (valFFT[i] >= (INT16_MIN + 1024)))
        if (FFTabs(valFFT[i]) > maxSample) maxSample = FFTabs(valFFT[i]);
    }
    micDataReal = maxSample;

#ifdef SR_DEBUG
    if (true) {
#else
    if (sampleAvg > 0.25f) {
#endif

#ifdef UM_AUDIOREACTIVE_USE_ARDUINO_FFT
      memset(vImag, 0, samplesFFT * sizeof(float));
      FFT.dcRemoval();
#ifdef FFT_PREFER_EXACT_PEAKS
      FFT.windowing(FFTWindow::Blackman_Harris, FFTDirection::Forward);
#else
      FFT.windowing( FFTWindow::Flat_top, FFTDirection::Forward);
#endif
      FFT.compute( FFTDirection::Forward );
      FFT.complexToMagnitude();
      valFFT[0] = 0;
      FFT.majorPeak(&FFT_MajorPeak, &FFT_Magnitude);
#else
      FFTmathType sum = 0;
      for (int i = 0; i < samplesFFT; i++) sum += valFFT[i];
      FFTmathType mean = sum / (FFTmathType)samplesFFT;
      for (int i = 0; i < samplesFFT; i++) valFFT[i] -= mean;
#if !defined(UM_AUDIOREACTIVE_USE_INTEGER_FFT)
      for (int i = samplesFFT - 1; i >= 0 ; i--) {
        float windowed_sample = valFFT[i] * windowFFT[i];
        valFFT[i * 2] = windowed_sample;
        valFFT[i * 2 + 1] = 0.0;
      }
#ifdef CONFIG_IDF_TARGET_ESP32S3
      dsps_fft2r_fc32_aes3(valFFT, samplesFFT);
#elif defined(CONFIG_IDF_TARGET_ESP32)
      dsps_fft2r_fc32_ae32(valFFT, samplesFFT);
#else
      dsps_fft2r_fc32_ansi(valFFT, samplesFFT);
#endif
      dsps_bit_rev_fc32(valFFT, samplesFFT);
      valFFT[0] = 0;
      FFT_MajorPeak = 0;
      FFT_Magnitude = 0;
      for (int i = 1; i < samplesFFT_2; i++) {
        float real_part = valFFT[i * 2];
        float imag_part = valFFT[i * 2 + 1];
        valFFT[i] = sqrtf(real_part * real_part + imag_part * imag_part);
        if (valFFT[i] > FFT_Magnitude) {
          FFT_Magnitude = valFFT[i];
          FFT_MajorPeak = i*(SAMPLE_RATE/samplesFFT);
        }
      }
#else
      for (int i = samplesFFT - 1; i >= 0 ; i--) {
        int16_t windowed_sample = ((int32_t)valFFT[i] * (int32_t)windowFFT[i]) >> 15;
        valFFT[i * 2] = windowed_sample;
        valFFT[i * 2 + 1] = 0;
      }
      dsps_fft2r_sc16_ansi(valFFT, samplesFFT);
      dsps_bit_rev_sc16_ansi(valFFT, samplesFFT);
      valFFT[0] = 0;
      int FFT_MajorPeak_int = 0;
      int FFT_Magnitude_int = 0;
      for (int i = 1; i < samplesFFT_2; i++) {
        int32_t real_part = valFFT[i * 2];
        int32_t imag_part = valFFT[i * 2 + 1];
        valFFT[i] = sqrt32_bw(real_part * real_part + imag_part * imag_part);
        if (valFFT[i] > FFT_Magnitude_int) {
          FFT_Magnitude_int = valFFT[i];
          FFT_MajorPeak_int = ((i * SAMPLE_RATE)/samplesFFT);
        }
      }
      FFT_Magnitude = FFT_Magnitude_int * 512;
      FFT_MajorPeak = FFT_MajorPeak_int;
#endif
#endif
      FFT_MajorPeak = constrain(FFT_MajorPeak, 1.0f, 11025.0f);
#if defined(WLED_DEBUG) || defined(SR_DEBUG)
      haveDoneFFT = true;
#endif
    } else {
      memset(valFFT, 0, samplesFFT * sizeof(FFTsampleType));
      FFT_MajorPeak = 1;
      FFT_Magnitude = 0.001;
    }

    if (fabsf(sampleAvg) > 0.5f) {
      if (useBandPassFilter) {
        fftCalc[ 0] = 0.8f * fftAddAvg(3,4);
        fftCalc[ 1] = 0.9f * fftAddAvg(4,5);
        fftCalc[ 2] = fftAddAvg(5,6);
        fftCalc[ 3] = fftAddAvg(6,7);
        fftCalc[15] = fftAddAvg(165,205) * 0.75f;
      } else {
        fftCalc[ 0] = fftAddAvg(1,2);
        fftCalc[ 1] = fftAddAvg(2,3);
        fftCalc[ 2] = fftAddAvg(3,5);
        fftCalc[ 3] = fftAddAvg(5,7);
        fftCalc[15] = fftAddAvg(165,215) * 0.70f;
      }
      fftCalc[ 4] = fftAddAvg(7,10);
      fftCalc[ 5] = fftAddAvg(10,13);
      fftCalc[ 6] = fftAddAvg(13,19);
      fftCalc[ 7] = fftAddAvg(19,26);
      fftCalc[ 8] = fftAddAvg(26,33);
      fftCalc[ 9] = fftAddAvg(33,44);
      fftCalc[10] = fftAddAvg(44,56);
      fftCalc[11] = fftAddAvg(56,70);
      fftCalc[12] = fftAddAvg(70,86);
      fftCalc[13] = fftAddAvg(86,104);
      fftCalc[14] = fftAddAvg(104,165) * 0.88f;
    } else {
      for (int i=0; i < NUM_GEQ_CHANNELS; i++) {
        fftCalc[i] *= 0.85f;
        if (fftCalc[i] < 4.0f) fftCalc[i] = 0.0f;
      }
    }

    postProcessFFTResults((fabsf(sampleAvg) > 0.25f)? true : false , NUM_GEQ_CHANNELS);

#if defined(WLED_DEBUG) || defined(SR_DEBUG)
    if (haveDoneFFT && (start < esp_timer_get_time())) {
      uint64_t fftTimeInMillis = ((esp_timer_get_time() - start) +5ULL) / 10ULL;
      fftTime  = (fftTimeInMillis*3 + fftTime*7)/10;
    }
#endif
    autoResetPeak();
    detectSamplePeak();

    #if !defined(I2S_GRAB_ADC1_COMPLETELY)
    if ((audioSource == nullptr) || (audioSource->getType() != AudioSource::Type_I2SAdc))
    #endif
      vTaskDelayUntil( &xLastWakeTime, xFrequency);

  }
}


static void runMicFilter(uint16_t numSamples, FFTsampleType *sampleBuffer)
{
#if !defined(UM_AUDIOREACTIVE_USE_INTEGER_FFT)
  constexpr float alpha = 0.0256f;
  constexpr float beta1 = 0.85f;
  constexpr float beta2 = (1.0f - beta1) / 2.0f;
  static float last_vals[2] = { 0.0f };
  static float lowfilt = 0.0f;

  for (int i=0; i < numSamples; i++) {
        float highFilteredSample;
        if (i < (numSamples-1)) highFilteredSample = beta1*sampleBuffer[i] + beta2*last_vals[0] + beta2*sampleBuffer[i+1];
        else highFilteredSample = beta1*sampleBuffer[i] + beta2*last_vals[0]  + beta2*last_vals[1];
        last_vals[1] = last_vals[0];
        last_vals[0] = sampleBuffer[i];
        sampleBuffer[i] = highFilteredSample;
        lowfilt += alpha * (sampleBuffer[i] - lowfilt);
        sampleBuffer[i] = sampleBuffer[i] - lowfilt;
  }
#else
  constexpr int32_t ALPHA_FP = 840;
  constexpr int32_t BETA1_FP = 55706;
  constexpr int32_t BETA2_FP = (65536 - BETA1_FP) / 2;

  static int32_t last_vals[2] = { 0 };
  static int32_t lowfilt_fp = 0;

  for (int i = 0; i < numSamples; i++) {
    int32_t highFilteredSample_fp;

    if (i < (numSamples - 1))
      highFilteredSample_fp = (BETA1_FP * (int32_t)sampleBuffer[i] + BETA2_FP * last_vals[0] + BETA2_FP * (int32_t)sampleBuffer[i + 1]) >> 16;
    else
      highFilteredSample_fp = (BETA1_FP * (int32_t)sampleBuffer[i] + BETA2_FP * last_vals[0] + BETA2_FP * last_vals[1]) >> 16;
    last_vals[1] = last_vals[0];
    last_vals[0] = (int32_t)sampleBuffer[i];
    lowfilt_fp += ALPHA_FP * (highFilteredSample_fp - (lowfilt_fp >> 15));
    sampleBuffer[i] = highFilteredSample_fp - (lowfilt_fp >> 15);
  }
#endif
}

static void postProcessFFTResults(bool noiseGateOpen, int numberOfChannels)
{
    for (int i=0; i < numberOfChannels; i++) {

      if (noiseGateOpen) {
        fftCalc[i] *= fftResultPink[i];
        if (FFTScalingMode > 0) fftCalc[i] *= FFT_DOWNSCALE;
        fftCalc[i] *= soundAgc ? multAgc : ((float)sampleGain/40.0f * (float)inputLevel/128.0f + 1.0f/16.0f);
        if(fftCalc[i] < 0) fftCalc[i] = 0;
      }

      if(fftCalc[i] > fftAvg[i])
        fftAvg[i] = fftCalc[i] *0.75f + 0.25f*fftAvg[i];
      else {
        if (decayTime < 1000) fftAvg[i] = fftCalc[i]*0.22f + 0.78f*fftAvg[i];
        else if (decayTime < 2000) fftAvg[i] = fftCalc[i]*0.17f + 0.83f*fftAvg[i];
        else if (decayTime < 3000) fftAvg[i] = fftCalc[i]*0.14f + 0.86f*fftAvg[i];
        else fftAvg[i] = fftCalc[i]*0.1f  + 0.9f*fftAvg[i];
      }
      fftCalc[i] = constrain(fftCalc[i], 0.0f, 1023.0f);
      fftAvg[i] = constrain(fftAvg[i], 0.0f, 1023.0f);

      float currentResult;
      if(limiterOn == true)
        currentResult = fftAvg[i];
      else
        currentResult = fftCalc[i];

      switch (FFTScalingMode) {
        case 1:
            currentResult *= 0.42f;
            currentResult -= 8.0f;
            if (currentResult > 1.0f) currentResult = logf(currentResult);
            else currentResult = 0.0f;
            currentResult *= 0.85f + (float(i)/18.0f);
            currentResult = mapf(currentResult, 0, LOG_256, 0, 255);
        break;
        case 2:
            currentResult *= 0.30f;
            currentResult -= 4.0f;
            if (currentResult < 1.0f) currentResult = 0.0f;
            currentResult *= 0.85f + (float(i)/1.8f);
        break;
        case 3:
            currentResult *= 0.38f;
            currentResult -= 6.0f;
            if (currentResult > 1.0f) currentResult = sqrtf(currentResult);
            else currentResult = 0.0f;
            currentResult *= 0.85f + (float(i)/4.5f);
            currentResult = mapf(currentResult, 0.0, 16.0, 0.0, 255.0);
        break;

        case 0:
        default:
            currentResult -= 4;
        break;
      }

      if (soundAgc > 0) {
        float post_gain = (float)inputLevel/128.0f;
        if (post_gain < 1.0f) post_gain = ((post_gain -1.0f) * 0.8f) +1.0f;
        currentResult *= post_gain;
      }
      fftResult[i] = constrain((int)currentResult, 0, 255);
    }
}

static void detectSamplePeak(void) {
  bool havePeak = false;
  if ((sampleAvg > 1) && (maxVol > 0) && (binNum > 4) && (valFFT[binNum] > maxVol) && ((millis() - timeOfPeak) > 100)) {
    havePeak = true;
  }

  if (havePeak) {
    samplePeak    = true;
    timeOfPeak    = millis();
    udpSamplePeak = true;
  }
}

#endif

static void autoResetPeak(void) {
  uint16_t peakDelay = max(uint16_t(50), strip.getFrameTime());
  if (millis() - timeOfPeak > peakDelay) {
    samplePeak = false;
    if (audioSyncEnabled == 0) udpSamplePeak = false;
  }
}


class AudioReactive : public Usermod {

  private:
#ifdef ARDUINO_ARCH_ESP32

    static constexpr uint8_t SR_DMTYPE_NETWORK_ONLY = 254;

    #ifndef AUDIOPIN
    int8_t audioPin = -1;
    #else
    int8_t audioPin = AUDIOPIN;
    #endif
    #ifndef SR_DMTYPE
    uint8_t dmType = 1;
    #define SR_DMTYPE 1
    #else
    uint8_t dmType = SR_DMTYPE;
    #endif
    #ifndef I2S_SDPIN
    int8_t i2ssdPin = 32;
    #else
    int8_t i2ssdPin = I2S_SDPIN;
    #endif
    #ifndef I2S_WSPIN
    int8_t i2swsPin = 15;
    #else
    int8_t i2swsPin = I2S_WSPIN;
    #endif
    #ifndef I2S_CKPIN
    int8_t i2sckPin = 14;
    #else
    int8_t i2sckPin = I2S_CKPIN;
    #endif
    #ifndef MCLK_PIN
    int8_t mclkPin = I2S_PIN_NO_CHANGE;
    #else
    int8_t mclkPin = MCLK_PIN;
    #endif
#endif

    struct __attribute__ ((packed)) audioSyncPacket {
      char    header[6];
      uint8_t reserved1[2];
      float   sampleRaw;
      float   sampleSmth;
      uint8_t samplePeak;
      uint8_t reserved2;
      uint8_t fftResult[16];
      uint16_t reserved3;
      float  FFT_Magnitude;
      float  FFT_MajorPeak;
    };

    struct audioSyncPacket_v1 {
      char header[6];
      uint8_t myVals[32];
      int sampleAgc;
      int sampleRaw;
      float sampleAvg;
      bool samplePeak;
      uint8_t fftResult[16];
      double FFT_Magnitude;
      double FFT_MajorPeak;
    };

    #define UDPSOUND_MAX_PACKET 88

    #ifdef UM_AUDIOREACTIVE_ENABLE
    bool     enabled = true;
    #else
    bool     enabled = false;
    #endif

    bool     initDone = false;
    bool     addPalettes = false;
    int8_t   palettes = 0;

    // variables  for UDP sound sync
    WiFiUDP fftUdp;
    unsigned long lastTime = 0;
    unsigned long lastStateChangeTime = 0;
    const uint16_t delayMs = 10;
    uint16_t audioSyncPort= 11988;
    uint8_t  audioSyncTransport = 0;   // 0 = UDP, 1 = ESP-NOW. Mutually exclusive (see connected()).
    uint8_t  userSyncChannel = 0;      // NEW: 0 = auto (follow current Wi-Fi channel), 1..13 = manual ESP-NOW channel override
    bool     macLockEnabled = true;    // NEW: receive-side MAC lock toggle

    bool updateIsRunning = false;

#ifdef ARDUINO_ARCH_ESP32
    int      last_soundAgc = -1;
    double   control_integrated = 0.0;

    int16_t  micIn = 0;
    double   sampleMax = 0.0;
    double   micLev = 0.0;
    float    expAdjF = 0.0f;
    float    sampleReal = 0.0f;
    int16_t  sampleRaw = 0;
    int16_t  rawSampleAgc = 0;
#endif

    float   volumeSmth = 0.0f;
    int16_t  volumeRaw = 0;
    float my_magnitude =0.0f;

    unsigned long last_UDPTime = 0;
    int receivedFormat = 0;
    float maxSample5sec = 0.0f;
    unsigned long sampleMaxTimer = 0;
    #define CYCLE_SAMPLEMAX 3500

    static const char _name[];
    static const char _enabled[];
    static const char _config[];
    static const char _dynamics[];
    static const char _frequency[];
    static const char _inputLvl[];
#if defined(ARDUINO_ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
    static const char _analogmic[];
#endif
    static const char _digitalmic[];
    static const char _addPalettes[];
    static const char _palName0[];
    static const char _palName1[];
    static const char _palName2[];
    static const char UDP_SYNC_HEADER[];
    static const char UDP_SYNC_HEADER_v1[];
    static const char _syncTransport[];
    static const char _syncChannel[];   // NEW
    static const char _macLock[];       // NEW

    void removeAudioPalettes(void);
    void createAudioPalettes(void);
    CRGB getCRGBForBand(int x, int pal);
    void fillAudioPalettes(void);

    void logAudio()
    {
      if (disableSoundProcessing && (!udpSyncConnected || ((audioSyncEnabled & 0x02) == 0))) return;
    #ifdef MIC_LOGGER
      PLOT_PRINT("micReal:");     PLOT_PRINT(micDataReal); PLOT_PRINT("\t");
      PLOT_PRINT("volumeSmth:");  PLOT_PRINT(volumeSmth);  PLOT_PRINT("\t");
      PLOT_PRINT("DC_Level:");    PLOT_PRINT(micLev);      PLOT_PRINT("\t");
      PLOT_PRINTLN();
    #endif

    #ifdef FFT_SAMPLING_LOG
      const bool mapValuesToPlotterSpace = false;
      const bool scaleValuesFromCurrentMaxVal = false;
      const bool printMaxVal = false;
      const bool printMinVal = false;
      const int defaultScalingFromHighValue = 256;
      const int scalingToHighValue = 256;
      const int minimumMaxVal = 1;

      int maxVal = minimumMaxVal;
      int minVal = 0;
      for(int i = 0; i < NUM_GEQ_CHANNELS; i++) {
        if(fftResult[i] > maxVal) maxVal = fftResult[i];
        if(fftResult[i] < minVal) minVal = fftResult[i];
      }
      for(int i = 0; i < NUM_GEQ_CHANNELS; i++) {
        PLOT_PRINT(i); PLOT_PRINT(":");
        PLOT_PRINTF("%04ld ", map(fftResult[i], 0, (scaleValuesFromCurrentMaxVal ? maxVal : defaultScalingFromHighValue), (mapValuesToPlotterSpace*i*scalingToHighValue)+0, (mapValuesToPlotterSpace*i*scalingToHighValue)+scalingToHighValue-1));
      }
      if(printMaxVal) {
        PLOT_PRINTF("maxVal:%04d ", maxVal + (mapValuesToPlotterSpace ? 16*256 : 0));
      }
      if(printMinVal) {
        PLOT_PRINTF("%04d:minVal ", minVal);
      }
      if(mapValuesToPlotterSpace)
        PLOT_PRINTF("max:%04d ", (printMaxVal ? 17 : 16)*256);
      else {
        PLOT_PRINTF("max:%04d ", 256);
      }
      PLOT_PRINTLN();
    #endif
    }


#ifdef ARDUINO_ARCH_ESP32
    void agcAvg(unsigned long the_time)
    {
      const int AGC_preset = (soundAgc > 0)? (soundAgc-1): 0;

      float lastMultAgc = multAgc;
      float multAgcTemp = multAgc;
      float tmpAgc = sampleReal * multAgc;

      float control_error;

      if (last_soundAgc != soundAgc)
        control_integrated = 0.0;

      static unsigned long last_time = 0;
      unsigned long time_now = millis();
      if ((the_time > 0) && (the_time < time_now)) time_now = the_time;

      if (time_now - last_time > 2)  {
        last_time = time_now;

        if((fabsf(sampleReal) < 2.0f) || (sampleMax < 1.0)) {
          tmpAgc = 0;
          if (fabs(control_integrated) < 0.01)  control_integrated  = 0.0;
          else                                  control_integrated *= 0.91;
        } else {
          if (tmpAgc <= agcTarget0Up[AGC_preset])
            multAgcTemp = agcTarget0[AGC_preset] / sampleMax;
          else
            multAgcTemp = agcTarget1[AGC_preset] / sampleMax;
        }
        if (multAgcTemp > 32.0f)      multAgcTemp = 32.0f;
        if (multAgcTemp < 1.0f/64.0f) multAgcTemp = 1.0f/64.0f;

        control_error = multAgcTemp - lastMultAgc;

        if (((multAgcTemp > 0.085f) && (multAgcTemp < 6.5f))
            && (multAgc*sampleMax < agcZoneStop[AGC_preset]))
          control_integrated += control_error * 0.002 * 0.25;
        else
          control_integrated *= 0.9;

        tmpAgc = sampleReal * lastMultAgc;
        if ((tmpAgc > agcZoneHigh[AGC_preset]) || (tmpAgc < soundSquelch + agcZoneLow[AGC_preset])) {
          multAgcTemp = lastMultAgc + agcFollowFast[AGC_preset] * agcControlKp[AGC_preset] * control_error;
          multAgcTemp += agcFollowFast[AGC_preset] * agcControlKi[AGC_preset] * control_integrated;
        } else {
          multAgcTemp = lastMultAgc + agcFollowSlow[AGC_preset] * agcControlKp[AGC_preset] * control_error;
          multAgcTemp += agcFollowSlow[AGC_preset] * agcControlKi[AGC_preset] * control_integrated;
        }

        if (multAgcTemp > 32.0f)      multAgcTemp = 32.0f;
        if (multAgcTemp < 1.0f/64.0f) multAgcTemp = 1.0f/64.0f;
      }

      tmpAgc = sampleReal * multAgcTemp;
      if (fabsf(sampleReal) < 2.0f) tmpAgc = 0.0f;
      if (tmpAgc > 255) tmpAgc = 255.0f;
      if (tmpAgc < 1)   tmpAgc = 0.0f;

      multAgc = multAgcTemp;
      rawSampleAgc = 0.8f * tmpAgc + 0.2f * (float)rawSampleAgc;
      if (fabsf(tmpAgc) < 1.0f)
        sampleAgc =  0.5f * tmpAgc + 0.5f * sampleAgc;
      else
        sampleAgc += agcSampleSmooth[AGC_preset] * (tmpAgc - sampleAgc);

      sampleAgc = fabsf(sampleAgc);
      last_soundAgc = soundAgc;
    }

    void getSample()
    {
      float    sampleAdj;
      float    tmpSample;
      const float weighting = 0.2f;
      const int   AGC_preset = (soundAgc > 0)? (soundAgc-1): 0;

      #ifdef WLED_DISABLE_SOUND
        micIn = perlin8(millis(), millis());
        micDataReal = micIn;
      #else
        #ifdef ARDUINO_ARCH_ESP32
        micIn = int(micDataReal);
        #else
        static unsigned long lastAnalogTime = 0;
        static float lastAnalogValue = 0.0f;
        if (millis() - lastAnalogTime > 20) {
            micDataReal = analogRead(A0);
            lastAnalogTime = millis();
            lastAnalogValue = micDataReal;
            yield();
        } else micDataReal = lastAnalogValue;
        micIn = int(micDataReal);
        #endif
      #endif

      micLev += (micDataReal-micLev) / 12288.0f;
      if(micIn < micLev) micLev = ((micLev * 31.0f) + micDataReal) / 32.0f;

      micIn -= micLev;
      float micInNoDC = fabsf(micDataReal - micLev);
      expAdjF = (weighting * micInNoDC + (1.0f-weighting) * expAdjF);
      expAdjF = fabsf(expAdjF);

      expAdjF = (expAdjF <= soundSquelch) ? 0: expAdjF;
      if ((soundSquelch == 0) && (expAdjF < 0.25f)) expAdjF = 0;

      tmpSample = expAdjF;
      micIn = abs(micIn);

      sampleAdj = tmpSample * sampleGain / 40.0f * inputLevel/128.0f + tmpSample / 16.0f;
      sampleReal = tmpSample;

      sampleAdj = fmax(fmin(sampleAdj, 255), 0);
      sampleRaw = (int16_t)sampleAdj;

      if ((sampleMax < sampleReal) && (sampleReal > 0.5f)) {
        sampleMax = sampleMax + 0.5f * (sampleReal - sampleMax);
        if (((binNum < 12) || ((maxVol < 1))) && (millis() - timeOfPeak > 80) && (sampleAvg > 1)) {
          samplePeak    = true;
          timeOfPeak    = millis();
          udpSamplePeak = true;
        }
      } else {
        if ((multAgc*sampleMax > agcZoneStop[AGC_preset]) && (soundAgc > 0))
          sampleMax += 0.5f * (sampleReal - sampleMax);
        else
          sampleMax *= agcSampleDecay[AGC_preset];
      }
      if (sampleMax < 0.5f) sampleMax = 0.0f;

      sampleAvg = ((sampleAvg * 15.0f) + sampleAdj) / 16.0f;
      sampleAvg = fabsf(sampleAvg);
    }

#endif

    void limitSampleDynamics(void) {
      const float bigChange = 196;
      static unsigned long last_time = 0;
      static float last_volumeSmth = 0.0f;

      if (limiterOn == false) return;

      long delta_time = millis() - last_time;
      delta_time = constrain(delta_time , 1, 1000);
      float deltaSample = volumeSmth - last_volumeSmth;

      if (attackTime > 0) {
        float maxAttack =   bigChange * float(delta_time) / float(attackTime);
        if (deltaSample > maxAttack) deltaSample = maxAttack;
      }
      if (decayTime > 0) {
        float maxDecay  = - bigChange * float(delta_time) / float(decayTime);
        if (deltaSample < maxDecay) deltaSample = maxDecay;
      }

      volumeSmth = last_volumeSmth + deltaSample;

      last_volumeSmth = volumeSmth;
      last_time = millis();
    }


    void connectUDPSoundSync(void) {
      static unsigned long last_connection_attempt = 0;

      // NEW: skip entirely when ESP-NOW transport is selected -- UDP and
      // ESP-NOW are mutually exclusive.
      if (audioSyncTransport == 1) return;

      if ((audioSyncPort <= 0) || ((audioSyncEnabled & 0x03) == 0)) return;
      if (udpSyncConnected) return;
      if (!(apActive || interfacesInited)) return;
      if (millis() - last_connection_attempt < 15000) return;
      if (updateIsRunning) return;

      last_connection_attempt = millis();
      connected();
    }

#ifdef ARDUINO_ARCH_ESP32
    void transmitAudioData()
    {
      // For UDP we need an active multicast socket. For ESP-NOW the UDP
      // socket is intentionally NOT opened (mutually exclusive transport),
      // so we skip the udpSyncConnected check on that path -- send() will
      // return false if it isn't initialised.
      if (audioSyncTransport == 0 && !udpSyncConnected) return;

      audioSyncPacket transmitData;
      memset(reinterpret_cast<void *>(&transmitData), 0, sizeof(transmitData));

      strncpy_P(transmitData.header, PSTR(UDP_SYNC_HEADER), 6);
      transmitData.sampleRaw   = (soundAgc) ? rawSampleAgc: sampleRaw;
      transmitData.sampleSmth  = (soundAgc) ? sampleAgc   : sampleAvg;
      transmitData.samplePeak  = udpSamplePeak ? 1:0;
      udpSamplePeak            = false;

      for (int i = 0; i < NUM_GEQ_CHANNELS; i++) {
        transmitData.fftResult[i] = (uint8_t)constrain(fftResult[i], 0, 254);
      }

      transmitData.FFT_Magnitude = my_magnitude;
      transmitData.FFT_MajorPeak = FFT_MajorPeak;

      if (audioSyncTransport == 1) {
        if (millis() - lastStateChangeTime > 120) {
          espnowAudioSync::send(&transmitData, sizeof(transmitData));
        }
      } else {
        if (fftUdp.beginMulticastPacket() != 0) {
          fftUdp.write(reinterpret_cast<uint8_t *>(&transmitData), sizeof(transmitData));
          fftUdp.endPacket();
        }
      }
      return;
    }

#endif

    static bool isValidUdpSyncVersion(const char *header) {
      return strncmp_P(header, UDP_SYNC_HEADER, 6) == 0;
    }
    static bool isValidUdpSyncVersion_v1(const char *header) {
      return strncmp_P(header, UDP_SYNC_HEADER_v1, 6) == 0;
    }

    void decodeAudioData(int packetSize, uint8_t *fftBuff) {
      audioSyncPacket receivedPacket;
      memset(&receivedPacket, 0, sizeof(receivedPacket));
      memcpy(&receivedPacket, fftBuff, min((unsigned)packetSize, (unsigned)sizeof(receivedPacket)));

      volumeSmth   = fmaxf(receivedPacket.sampleSmth, 0.0f);
      volumeRaw    = fmaxf(receivedPacket.sampleRaw, 0.0f);
#ifdef ARDUINO_ARCH_ESP32
      sampleRaw    = volumeRaw;
      sampleAvg    = volumeSmth;
      rawSampleAgc = volumeRaw;
      sampleAgc    = volumeSmth;
      multAgc      = 1.0f;
#endif
      autoResetPeak();
      if (!samplePeak) {
            samplePeak = receivedPacket.samplePeak >0 ? true:false;
            if (samplePeak) timeOfPeak = millis();
      }
      for (int i = 0; i < NUM_GEQ_CHANNELS; i++) fftResult[i] = receivedPacket.fftResult[i];
      my_magnitude  = fmaxf(receivedPacket.FFT_Magnitude, 0.0f);
      FFT_Magnitude = my_magnitude;
      FFT_MajorPeak = constrain(receivedPacket.FFT_MajorPeak, 1.0f, 11025.0f);
    }

    void decodeAudioData_v1(int packetSize, uint8_t *fftBuff) {
      audioSyncPacket_v1 *receivedPacket = reinterpret_cast<audioSyncPacket_v1*>(fftBuff);
      volumeSmth   = fmaxf(receivedPacket->sampleAgc, 0.0f);
      volumeRaw    = volumeSmth;
#ifdef ARDUINO_ARCH_ESP32
      sampleRaw    = fmaxf(receivedPacket->sampleRaw, 0.0f);
      sampleAvg    = fmaxf(receivedPacket->sampleAvg, 0.0f);;
      sampleAgc    = volumeSmth;
      rawSampleAgc = volumeRaw;
      multAgc      = 1.0f;
#endif
      autoResetPeak();
      if (!samplePeak) {
            samplePeak = receivedPacket->samplePeak >0 ? true:false;
            if (samplePeak) timeOfPeak = millis();
      }
      for (int i = 0; i < NUM_GEQ_CHANNELS; i++) fftResult[i] = receivedPacket->fftResult[i];
      my_magnitude  = fmaxf(receivedPacket->FFT_Magnitude, 0.0);
      FFT_Magnitude = my_magnitude;
      FFT_MajorPeak = constrain(receivedPacket->FFT_MajorPeak, 1.0, 11025.0);
    }

    bool receiveAudioData()
    {
      if (audioSyncTransport == 1) {
        audioSyncPacket enFrame;
        uint32_t        enDeadline = 0;
        if (espnowAudioSync::poll(&enFrame, sizeof(enFrame), &enDeadline, /*renderDelayUs=*/15000)) {
          decodeAudioData(sizeof(enFrame), reinterpret_cast<uint8_t*>(&enFrame));
          receivedFormat = 2;
          last_UDPTime   = millis();
          return true;
        }
        return false;
      }
      if (!udpSyncConnected) return false;
      bool haveFreshData = false;

      size_t packetSize = fftUdp.parsePacket();
#ifdef ARDUINO_ARCH_ESP32
      if ((packetSize > 0) && ((packetSize < 5) || (packetSize > UDPSOUND_MAX_PACKET))) fftUdp.flush();
#endif
      if ((packetSize > 5) && (packetSize <= UDPSOUND_MAX_PACKET)) {
        uint8_t fftBuff[UDPSOUND_MAX_PACKET+1] = { 0 };
        fftUdp.read(fftBuff, packetSize);

        if (packetSize == sizeof(audioSyncPacket) && (isValidUdpSyncVersion((const char *)fftBuff))) {
          decodeAudioData(packetSize, fftBuff);
          haveFreshData = true;
          receivedFormat = 2;
        } else {
          if (packetSize == sizeof(audioSyncPacket_v1) && (isValidUdpSyncVersion_v1((const char *)fftBuff))) {
            decodeAudioData_v1(packetSize, fftBuff);
            haveFreshData = true;
            receivedFormat = 1;
          } else receivedFormat = 0;
        }
      }
      return haveFreshData;
    }


  public:

    void setup() override
    {
      disableSoundProcessing = true;
      if (!initDone) {
        um_data = new um_data_t;
        um_data->u_size = 8;
        um_data->u_type = new um_types_t[um_data->u_size];
        um_data->u_data = new void*[um_data->u_size];
        um_data->u_data[0] = &volumeSmth;
        um_data->u_type[0] = UMT_FLOAT;
        um_data->u_data[1] = &volumeRaw;
        um_data->u_type[1] = UMT_UINT16;
        um_data->u_data[2] = fftResult;
        um_data->u_type[2] = UMT_BYTE_ARR;
        um_data->u_data[3] = &samplePeak;
        um_data->u_type[3] = UMT_BYTE;
        um_data->u_data[4] = &FFT_MajorPeak;
        um_data->u_type[4] = UMT_FLOAT;
        um_data->u_data[5] = &my_magnitude;
        um_data->u_type[5] = UMT_FLOAT;
        um_data->u_data[6] = &maxVol;
        um_data->u_type[6] = UMT_BYTE;
        um_data->u_data[7] = &binNum;
        um_data->u_type[7] = UMT_BYTE;
      }


#ifdef ARDUINO_ARCH_ESP32

      i2s_driver_uninstall(I2S_NUM_0);
      #if !defined(CONFIG_IDF_TARGET_ESP32C3)
        delay(100);
        periph_module_reset(PERIPH_I2S0_MODULE);
      #endif
      delay(100);
      useBandPassFilter = false;
      useMicFilter = true;

      #if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3)
        if ((i2sckPin == I2S_PIN_NO_CHANGE) && (i2ssdPin >= 0) && (i2swsPin >= 0) && ((dmType == 1) || (dmType == 4)) ) dmType = 5;
      #endif

      switch (dmType) {
      #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S3)
        case 0:
        #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3)
        case 5:
        #endif
      #endif
        case 1:
          DEBUGSR_PRINT(F("AR: Generic I2S Microphone - ")); DEBUGSR_PRINTLN(F(I2S_MIC_CHANNEL_TEXT));
          audioSource = new I2SSource(SAMPLE_RATE, BLOCK_SIZE);
          delay(100);
          if (audioSource) audioSource->initialize(i2swsPin, i2ssdPin, i2sckPin);
          break;
        case 2:
          DEBUGSR_PRINTLN(F("AR: ES7243 Microphone (right channel only)."));
          audioSource = new ES7243(SAMPLE_RATE, BLOCK_SIZE);
          delay(100);
          if (audioSource) audioSource->initialize(i2swsPin, i2ssdPin, i2sckPin, mclkPin);
          break;
        case 3:
          DEBUGSR_PRINT(F("AR: SPH0645 Microphone - ")); DEBUGSR_PRINTLN(F(I2S_MIC_CHANNEL_TEXT));
          audioSource = new SPH0654(SAMPLE_RATE, BLOCK_SIZE);
          delay(100);
          audioSource->initialize(i2swsPin, i2ssdPin, i2sckPin);
          break;
        case 4:
          DEBUGSR_PRINT(F("AR: Generic I2S Microphone with Master Clock - ")); DEBUGSR_PRINTLN(F(I2S_MIC_CHANNEL_TEXT));
          audioSource = new I2SSource(SAMPLE_RATE, BLOCK_SIZE, 1.0f/24.0f);
          useMicFilter = false;
          delay(100);
          if (audioSource) audioSource->initialize(i2swsPin, i2ssdPin, i2sckPin, mclkPin);
          break;
        #if  !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3)
        case 5:
          DEBUGSR_PRINT(F("AR: Generic PDM Microphone - ")); DEBUGSR_PRINTLN(F(I2S_PDM_MIC_CHANNEL_TEXT));
          audioSource = new I2SSource(SAMPLE_RATE, BLOCK_SIZE, 1.0f/4.0f);
          useBandPassFilter = true;
          delay(100);
          if (audioSource) audioSource->initialize(i2swsPin, i2ssdPin);
          break;
        #endif
        case 6:
          DEBUGSR_PRINTLN(F("AR: ES8388 Source"));
          audioSource = new ES8388Source(SAMPLE_RATE, BLOCK_SIZE);
          useMicFilter = false;
          delay(100);
          if (audioSource) audioSource->initialize(i2swsPin, i2ssdPin, i2sckPin, mclkPin);
          break;

        #if  !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
        case 0:
          DEBUGSR_PRINTLN(F("AR: Analog Microphone (left channel only)."));
          audioSource = new I2SAdcSource(SAMPLE_RATE, BLOCK_SIZE);
          delay(100);
          useBandPassFilter = true;
          if (audioSource) audioSource->initialize(audioPin);
          break;
        #endif

        case SR_DMTYPE_NETWORK_ONLY:
          if (audioSource) delete audioSource; audioSource = nullptr;
          disableSoundProcessing = true;
          audioSyncEnabled = 2;
          enabled = true;
          break;

        case 255:
        default:
          if (audioSource) delete audioSource; audioSource = nullptr;
          disableSoundProcessing = true;
          enabled = false;
        break;
      }
      delay(250);

      if (!audioSource && (dmType != SR_DMTYPE_NETWORK_ONLY)) enabled = false;
#endif
      if (enabled) onUpdateBegin(false);


#ifdef ARDUINO_ARCH_ESP32
      if (audioSource && FFT_Task == nullptr) enabled = false;
      if((!audioSource) || (!audioSource->isInitialized())) {
#ifdef WLED_DEBUG
        #define AR_INIT_DEBUG_PRINT DEBUG_PRINTLN
#else
        #define AR_INIT_DEBUG_PRINT DEBUGSR_PRINTLN
#endif
        if (dmType == SR_DMTYPE_NETWORK_ONLY) {
          AR_INIT_DEBUG_PRINT(F("AR: No sound input driver configured - network receive only."));
        } else {
          AR_INIT_DEBUG_PRINT(F("AR: Failed to initialize sound input driver. Please check input PIN settings."));
        }
        #undef AR_INIT_DEBUG_PRINT
        disableSoundProcessing = true;
      }
#endif
      if (enabled) disableSoundProcessing = false;
      if (enabled) connectUDPSoundSync();
      if (enabled && addPalettes) createAudioPalettes();
      initDone = true;
    }


    void connected() override
    {
      // Always tear down any previously-open UDP socket so we start fresh.
      if (udpSyncConnected) {
        udpSyncConnected = false;
        fftUdp.stop();
      }

      // Mutually-exclusive transport selection. UDP and ESP-NOW are never
      // both active at the same time, so the master never broadcasts on
      // both and the receiver only has to listen on one path.
      if (audioSyncTransport == 0) {
        if (audioSyncPort > 0 && (audioSyncEnabled & 0x03)) {
        #ifdef ARDUINO_ARCH_ESP32
          udpSyncConnected = fftUdp.beginMulticast(IPAddress(239, 0, 0, 1), audioSyncPort);
        #else
          udpSyncConnected = fftUdp.beginMulticast(WiFi.localIP(), IPAddress(239, 0, 0, 1), audioSyncPort);
        #endif
        }
      } else if (audioSyncTransport == 1) {
        // ESP-NOW transport. UDP stays closed; we go ESP-NOW only.
        // userSyncChannel: 0 = auto (follow current Wi-Fi channel),
        // 1..13 = manual override (useful for standalone-mode setups
        // where both boards aren't on the same AP).
        const uint8_t syncChannel = (userSyncChannel >= 1 && userSyncChannel <= 13)
                                    ? userSyncChannel
                                    : (uint8_t)WiFi.channel();
        const bool    amSender    = (audioSyncEnabled & 0x01) != 0;
        // Apply MAC-lock preference BEFORE begin() so the receive filter
        // starts in the right state.
        espnowAudioSync::setMacLockEnabled(macLockEnabled);
        if (!espnowAudioSync::begin(syncChannel, amSender)) {
          DEBUG_PRINTLN(F("[ENaudio] espnowAudioSync::begin() failed - check WLED ESP-NOW status."));
        }
      }
    }

    void loop() override
    {
      static unsigned long lastUMRun = millis();

      if (!enabled) {
        disableSoundProcessing = true;
        lastUMRun = millis();
        return;
      }
      if (strip.isUpdating() && (millis() - lastUMRun < 2)) return;

      if (  (realtimeOverride == REALTIME_OVERRIDE_NONE)
          &&( (realtimeMode == REALTIME_MODE_GENERIC)
            ||(realtimeMode == REALTIME_MODE_E131)
            ||(realtimeMode == REALTIME_MODE_UDP)
            ||(realtimeMode == REALTIME_MODE_ADALIGHT)
            ||(realtimeMode == REALTIME_MODE_ARTNET) ) )
      {
        #if defined(ARDUINO_ARCH_ESP32) && defined(WLED_DEBUG)
        if ((disableSoundProcessing == false) && (audioSyncEnabled == 0)) {
          DEBUG_PRINTLN(F("[AR userLoop]  realtime mode active - audio processing suspended."));
          DEBUG_PRINTF_P(PSTR("               RealtimeMode = %d; RealtimeOverride = %d\n"), int(realtimeMode), int(realtimeOverride));
        }
        #endif
        disableSoundProcessing = true;
      } else {
        #if defined(ARDUINO_ARCH_ESP32) && defined(WLED_DEBUG)
        if ((disableSoundProcessing == true) && (audioSyncEnabled == 0) && audioSource && audioSource->isInitialized()) {
          DEBUG_PRINTLN(F("[AR userLoop]  realtime mode ended - audio processing resumed."));
          DEBUG_PRINTF_P(PSTR("               RealtimeMode = %d; RealtimeOverride = %d\n"), int(realtimeMode), int(realtimeOverride));
        }
        #endif
        if ((disableSoundProcessing == true) && (audioSyncEnabled == 0)) lastUMRun = millis();
        disableSoundProcessing = false;
      }

      if (audioSyncEnabled & 0x02) disableSoundProcessing = true;
      if (audioSyncEnabled & 0x01) disableSoundProcessing = false;
#ifdef ARDUINO_ARCH_ESP32
      if (!audioSource || !audioSource->isInitialized()) disableSoundProcessing = true;


      if (!(audioSyncEnabled & 0x02) && !disableSoundProcessing) {
        if (soundAgc > AGC_NUM_PRESETS) soundAgc = 0;

        unsigned long t_now = millis();
        int userloopDelay = int(t_now - lastUMRun);
        if (lastUMRun == 0) userloopDelay=0;

        if (userloopDelay <2) userloopDelay = 0;
        if (userloopDelay >200) userloopDelay = 200;
        do {
          getSample();
          agcAvg(t_now - userloopDelay);
          userloopDelay -= 2;
        } while (userloopDelay > 0);
        lastUMRun = t_now;

        volumeSmth = (soundAgc) ? sampleAgc   : sampleAvg;
        volumeRaw  = (soundAgc) ? rawSampleAgc: sampleRaw;
        my_magnitude = FFT_Magnitude;
        if (soundAgc) my_magnitude *= multAgc;
        if (volumeSmth < 1 ) my_magnitude = 0.001f;

        limitSampleDynamics();
      }
#endif

      autoResetPeak();
      if (!udpSyncConnected) udpSamplePeak = false;

      connectUDPSoundSync();

      // Receive mode - run for either transport. UDP needs udpSyncConnected;
      // ESP-NOW does not (we don't open a UDP socket on that transport).
      if ((audioSyncEnabled & 0x02) && (udpSyncConnected || audioSyncTransport == 1)) {
          static float syncVolumeSmth = 0;
          bool have_new_sample = false;
          if (millis() - lastTime > delayMs) {
            have_new_sample = receiveAudioData();
            if (have_new_sample) last_UDPTime = millis();
#ifdef ARDUINO_ARCH_ESP32
            else if (audioSyncTransport == 0) fftUdp.flush();
#endif
            lastTime = millis();
          }
          if (have_new_sample) syncVolumeSmth = volumeSmth;
          else volumeSmth = syncVolumeSmth;
          limitSampleDynamics();
      }

      #if defined(MIC_LOGGER) || defined(MIC_SAMPLING_LOG) || defined(FFT_SAMPLING_LOG)
      static unsigned long lastMicLoggerTime = 0;
      if (millis()-lastMicLoggerTime > 20) {
        lastMicLoggerTime = millis();
        logAudio();
      }
      #endif

#ifdef ARDUINO_ARCH_ESP32
      if ((millis() -  sampleMaxTimer) > CYCLE_SAMPLEMAX) {
        sampleMaxTimer = millis();
        maxSample5sec = (0.15f * maxSample5sec) + 0.85f *((soundAgc) ? sampleAgc : sampleAvg);
        if (sampleAvg < 1) maxSample5sec = 0;
      } else {
         if ((sampleAvg >= 1)) maxSample5sec = fmaxf(maxSample5sec, (soundAgc) ? rawSampleAgc : sampleRaw);
      }
#else
      if ((millis() -  sampleMaxTimer) > CYCLE_SAMPLEMAX) {
        sampleMaxTimer = millis();
        maxSample5sec = (0.15 * maxSample5sec) + 0.85 * volumeSmth;
        if (volumeSmth < 1.0f) maxSample5sec = 0;
        if (maxSample5sec < 0.0f) maxSample5sec = 0;
      } else {
         if (volumeSmth >= 1.0f) maxSample5sec = fmaxf(maxSample5sec, volumeRaw);
      }
#endif

#ifdef ARDUINO_ARCH_ESP32
      if ((audioSyncEnabled & 0x01) && (millis() - lastTime > 20)) {
        transmitAudioData();
        lastTime = millis();
      }
#endif

      fillAudioPalettes();
    }


    bool getUMData(um_data_t **data) override
    {
      if (!data || !enabled) return false;
      *data = um_data;
      return true;
    }

#ifdef ARDUINO_ARCH_ESP32
    void onUpdateBegin(bool init) override
    {
#ifdef WLED_DEBUG
      fftTime = sampleTime = 0;
#endif
      disableSoundProcessing = true;

      micDataReal = 0.0f;
      volumeRaw = 0; volumeSmth = 0;
      sampleAgc = 0; sampleAvg = 0;
      sampleRaw = 0; rawSampleAgc = 0;
      my_magnitude = 0; FFT_Magnitude = 0; FFT_MajorPeak = 1;
      multAgc = 1;
      memset(fftCalc, 0, sizeof(fftCalc));
      memset(fftAvg, 0, sizeof(fftAvg));
      memset(fftResult, 0, sizeof(fftResult));
      for(int i=(init?0:1); i<NUM_GEQ_CHANNELS; i+=2) fftResult[i] = 16;
      inputLevel = 128;
      autoResetPeak();

      if (init && FFT_Task) {
        delay(25);
        vTaskSuspend(FFT_Task);
        if (udpSyncConnected) {
          udpSyncConnected = false;
          fftUdp.stop();
        }
      } else {
        if (FFT_Task) {
          vTaskResume(FFT_Task);
          connected();
        } else
          xTaskCreateUniversal(
            FFTcode,
            "FFT",
            3592,
            NULL,
            FFTTASK_PRIORITY,
            &FFT_Task
            , 0
          );
      }
      micDataReal = 0.0f;
      if (enabled) disableSoundProcessing = false;
      updateIsRunning = init;
    }

#else
    void onUpdateBegin(bool init)
    {
      disableSoundProcessing = true;
      volumeRaw = 0; volumeSmth = 0;
      for(int i=(init?0:1); i<NUM_GEQ_CHANNELS; i+=2) fftResult[i] = 16;
      autoResetPeak();
      if (init) {
        if (udpSyncConnected) {
          udpSyncConnected = false;
          fftUdp.stop();
          DEBUGSR_PRINTLN(F("AR onUpdateBegin(true): UDP connection closed."));
          receivedFormat = 0;
        }
      }
      if (enabled) disableSoundProcessing = init;
      updateIsRunning = init;
    }
#endif

#ifdef ARDUINO_ARCH_ESP32
    bool handleButton(uint8_t b) override {
      yield();
      if (enabled
          && dmType == 0 && audioPin>=0
          && (buttons[b].type == BTN_TYPE_ANALOG || buttons[b].type == BTN_TYPE_ANALOG_INVERTED)
         ) {
        return true;
      }
      return false;
    }

#endif

    void addToJsonInfo(JsonObject& root) override
    {
#ifdef ARDUINO_ARCH_ESP32
      char myStringBuffer[16];
#endif
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      JsonArray infoArr = user.createNestedArray(FPSTR(_name));

      String uiDomString = F("<button class=\"btn btn-xs\" onclick=\"requestJson({");
      uiDomString += FPSTR(_name);
      uiDomString += F(":{");
      uiDomString += FPSTR(_enabled);
      uiDomString += enabled ? F(":false}});\">") : F(":true}});\">");
      uiDomString += F("<i class=\"icons");
      uiDomString += enabled ? F(" on") : F(" off");
      uiDomString += F("\">&#xe08f;</i>");
      uiDomString += F("</button>");
      infoArr.add(uiDomString);

      if (enabled) {
#ifdef ARDUINO_ARCH_ESP32
        if (disableSoundProcessing == false) {
          if (soundAgc > 0) {
            infoArr = user.createNestedArray(F("GEQ Input Level"));
          } else {
            infoArr = user.createNestedArray(F("Audio Input Level"));
          }
          uiDomString = F("<div class=\"slider\"><div class=\"sliderwrap il\"><input class=\"noslide\" onchange=\"requestJson({");
          uiDomString += FPSTR(_name);
          uiDomString += F(":{");
          uiDomString += FPSTR(_inputLvl);
          uiDomString += F(":parseInt(this.value)}});\" oninput=\"updateTrail(this);\" max=255 min=0 type=\"range\" value=");
          uiDomString += inputLevel;
          uiDomString += F(" /><div class=\"sliderdisplay\"></div></div></div>");
          infoArr.add(uiDomString);
        }
#endif

        infoArr = user.createNestedArray(F("Audio Source"));
        if (audioSyncEnabled & 0x02) {
          infoArr.add(audioSyncTransport == 1 ? F("ESP-NOW sound sync") : F("UDP sound sync"));
          bool connectedish = (audioSyncTransport == 1) || udpSyncConnected;
          if (connectedish) {
            if (millis() - last_UDPTime < 2500)
              infoArr.add(F(" - receiving"));
            else
              infoArr.add(F(" - idle"));
          } else {
            infoArr.add(F(" - no connection"));
          }
#ifndef ARDUINO_ARCH_ESP32
        } else {
          infoArr.add(F("sound sync Off"));
        }
#else
        } else {
          if (audioSource && (audioSource->isInitialized())) {
            if (audioSource->getType() == AudioSource::Type_I2SAdc) {
              infoArr.add(F("ADC analog"));
            } else {
              if (dmType == 5) infoArr.add(F("PDM digital"));
              else infoArr.add(F("I2S digital"));
            }
            if (maxSample5sec > 1.0f) {
              float my_usage = 100.0f * (maxSample5sec / 255.0f);
              snprintf_P(myStringBuffer, 15, PSTR(" - peak %3d%%"), int(my_usage));
              infoArr.add(myStringBuffer);
            } else {
              infoArr.add(F(" - quiet"));
            }
          } else {
            infoArr.add(F("not initialized"));
            infoArr.add(F(" - check pin settings"));
          }
        }

        infoArr = user.createNestedArray(F("Sound Processing"));
        if (audioSource && (disableSoundProcessing == false)) {
          infoArr.add(F("running"));
        } else {
          infoArr.add(F("suspended"));
        }

        if ((soundAgc==0) && (disableSoundProcessing == false) && !(audioSyncEnabled & 0x02)) {
          infoArr = user.createNestedArray(F("Manual Gain"));
          float myGain = ((float)sampleGain/40.0f * (float)inputLevel/128.0f) + 1.0f/16.0f;
          infoArr.add(roundf(myGain*100.0f) / 100.0f);
          infoArr.add("x");
        }
        if (soundAgc && (disableSoundProcessing == false) && !(audioSyncEnabled & 0x02)) {
          infoArr = user.createNestedArray(F("AGC Gain"));
          infoArr.add(roundf(multAgc*100.0f) / 100.0f);
          infoArr.add("x");
        }
#endif

        // Sound Sync overall status (transport-aware)
        infoArr = user.createNestedArray(F("Sound Sync"));
        if (audioSyncEnabled) {
          if (audioSyncTransport == 1) infoArr.add(F("ESP-NOW"));
          else                          infoArr.add(F("UDP"));
          infoArr.add(F(" -"));
          if (audioSyncEnabled & 0x01) infoArr.add(F(" send"));
          else if (audioSyncEnabled & 0x02) infoArr.add(F(" receive"));
        } else {
          infoArr.add(F("off"));
        }
        if (audioSyncEnabled && audioSyncTransport == 0 && !udpSyncConnected) infoArr.add(" <i>(unconnected)</i>");
        if (audioSyncEnabled && udpSyncConnected && (millis() - last_UDPTime < 2500)) {
            if (receivedFormat == 1) infoArr.add(F(" v1"));
            if (receivedFormat == 2) infoArr.add(F(" v2"));
        }

        // ESP-NOW-only stats. Hidden entirely when UDP transport is selected.
        if (audioSyncTransport == 1) {
          auto s = espnowAudioSync::stats();

          static uint32_t      prev_rx = 0, prev_tx = 0;
          static unsigned long prev_ts = 0;
          unsigned long now_ms = millis();
          float rx_pps = 0.0f, tx_pps = 0.0f;
          if (prev_ts > 0 && now_ms > prev_ts + 100UL) {
            float dt = (now_ms - prev_ts) / 1000.0f;
            if (s.rx >= prev_rx) rx_pps = ((float)(s.rx - prev_rx)) / dt;
            if (s.tx >= prev_tx) tx_pps = ((float)(s.tx - prev_tx)) / dt;
          }
          prev_rx = s.rx;
          prev_tx = s.tx;
          prev_ts = now_ms;

          char rateBuf[32];

          infoArr = user.createNestedArray(F("ESP-NOW Channel"));
          uint8_t ch = espnowAudioSync::channel();
          if (ch > 0) infoArr.add(ch);
          else        infoArr.add(F("--"));
          if (userSyncChannel >= 1 && userSyncChannel <= 13) infoArr.add(F(" (manual)"));

          infoArr = user.createNestedArray(F("ESP-NOW Audio RX"));
          infoArr.add(s.rx);
          snprintf(rateBuf, sizeof(rateBuf), " frames (%.0f /s)", rx_pps);
          infoArr.add(rateBuf);

          infoArr = user.createNestedArray(F("ESP-NOW Frame Loss"));
          uint32_t denom = s.rx + s.gaps;
          if (denom > 0) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.2f %%", 100.0f * (float)s.gaps / (float)denom);
            infoArr.add(buf);
          } else {
            infoArr.add(F("--"));
          }

          infoArr = user.createNestedArray(F("ESP-NOW Audio TX"));
          infoArr.add(s.tx);
          snprintf(rateBuf, sizeof(rateBuf), " packets (%.0f /s)", tx_pps);
          infoArr.add(rateBuf);

          if (s.tx_err > 0) {
            infoArr = user.createNestedArray(F("ESP-NOW TX Errors"));
            infoArr.add(s.tx_err);
          }

          if (macLockEnabled && espnowAudioSync::hasLockedSender()) {
            const uint8_t * mac = espnowAudioSync::lockedSenderMac();
            char macBuf[20];
            snprintf(macBuf, sizeof(macBuf),
                     "%02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            infoArr = user.createNestedArray(F("Locked Master MAC"));
            infoArr.add(macBuf);
          } else if (!macLockEnabled) {
            infoArr = user.createNestedArray(F("MAC Lock"));
            infoArr.add(F("disabled"));
          }

          if (s.foreignSender > 0) {
            infoArr = user.createNestedArray(F("ESP-NOW Foreign Senders"));
            infoArr.add(s.foreignSender);
            infoArr.add(F(" packets dropped"));
          }

          infoArr = user.createNestedArray(F("ESP-NOW Mode"));
          infoArr.add(espnowAudioSync::isHosted() ? F("hosted (sharing WLED stack)") : F("standalone"));
        }

        #if defined(WLED_DEBUG) || defined(SR_DEBUG)
        #ifdef ARDUINO_ARCH_ESP32
        infoArr = user.createNestedArray(F("Sampling time"));
        infoArr.add(float(sampleTime)/100.0f);
        infoArr.add(" ms");

        infoArr = user.createNestedArray(F("FFT time"));
        infoArr.add(float(fftTime)/100.0f);
        if ((fftTime/100) >= FFT_MIN_CYCLE)
          infoArr.add("<b style=\"color:red;\">! ms</b>");
        else if ((fftTime/80 + sampleTime/80) >= FFT_MIN_CYCLE)
          infoArr.add("<b style=\"color:orange;\"> ms!</b>");
        else
          infoArr.add(" ms");

        DEBUGSR_PRINTF("AR Sampling time: %5.2f ms\n", float(sampleTime)/100.0f);
        DEBUGSR_PRINTF("AR FFT time     : %5.2f ms\n", float(fftTime)/100.0f);
        #endif
        #endif
      }
    }


    void addToJsonState(JsonObject& root) override
    {
      if (!initDone) return;
      JsonObject usermod = root[FPSTR(_name)];
      if (usermod.isNull()) {
        usermod = root.createNestedObject(FPSTR(_name));
      }
      usermod["on"] = enabled;
    }


    void readFromJsonState(JsonObject& root) override
    {
      if (!initDone) return;
      bool prevEnabled = enabled;
      JsonObject usermod = root[FPSTR(_name)];
      if (!usermod.isNull()) {
        if (usermod[FPSTR(_enabled)].is<bool>()) {
          enabled = usermod[FPSTR(_enabled)].as<bool>();
          if (prevEnabled != enabled) onUpdateBegin(!enabled);
          if (addPalettes) {
            if (prevEnabled && !enabled) removeAudioPalettes();
            if (!prevEnabled && enabled) createAudioPalettes();
          }
        }
#ifdef ARDUINO_ARCH_ESP32
        if (usermod[FPSTR(_inputLvl)].is<int>()) {
          inputLevel = min(255,max(0,usermod[FPSTR(_inputLvl)].as<int>()));
        }
#endif
      }
    }

    void onStateChange(uint8_t callMode) override {
      lastStateChangeTime = millis();
      if (initDone && enabled && addPalettes && palettes==0) {
        createAudioPalettes();
      }
    }

#ifndef WLED_DISABLE_ESPNOW
    // ESP-NOW audio-sync coexistence hook. Passes the sender MAC when
    // MAC locking is enabled; passes nullptr to bypass the filter when
    // disabled (legacy accept-from-anyone behaviour).
    bool onEspNowMessage(uint8_t* sender, uint8_t* data, uint8_t len) override {
      return espnowAudioSync::handleIncomingPacket(
          macLockEnabled ? sender : nullptr, data, len);
    }
#endif

    void addToConfig(JsonObject& root) override
    {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[FPSTR(_enabled)] = enabled;
      top[FPSTR(_addPalettes)] = addPalettes;

#ifdef ARDUINO_ARCH_ESP32
    #if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
      JsonObject amic = top.createNestedObject(FPSTR(_analogmic));
      amic["pin"] = audioPin;
    #endif

      JsonObject dmic = top.createNestedObject(FPSTR(_digitalmic));
      dmic["type"] = dmType;
      JsonArray pinArray = dmic.createNestedArray("pin");
      pinArray.add(i2ssdPin);
      pinArray.add(i2swsPin);
      pinArray.add(i2sckPin);
      pinArray.add(mclkPin);

      JsonObject cfg = top.createNestedObject(FPSTR(_config));
      cfg[F("squelch")] = soundSquelch;
      cfg[F("gain")] = sampleGain;
      cfg[F("AGC")] = soundAgc;

      JsonObject freqScale = top.createNestedObject(FPSTR(_frequency));
      freqScale[F("scale")] = FFTScalingMode;
#endif

      JsonObject dynLim = top.createNestedObject(FPSTR(_dynamics));
      dynLim[F("limiter")] = limiterOn;
      dynLim[F("rise")] = attackTime;
      dynLim[F("fall")] = decayTime;

      JsonObject sync = top.createNestedObject("sync");
      sync["port"] = audioSyncPort;
      sync["mode"] = audioSyncEnabled;
      sync[FPSTR(_syncTransport)] = audioSyncTransport;
      sync[FPSTR(_syncChannel)]   = userSyncChannel;
      sync[FPSTR(_macLock)]       = macLockEnabled;
    }


    bool readFromConfig(JsonObject& root) override
    {
      JsonObject top = root[FPSTR(_name)];
      bool configComplete = !top.isNull();
      bool oldEnabled = enabled;
      bool oldAddPalettes = addPalettes;

      configComplete &= getJsonValue(top[FPSTR(_enabled)], enabled);
      configComplete &= getJsonValue(top[FPSTR(_addPalettes)], addPalettes);

#ifdef ARDUINO_ARCH_ESP32
    #if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
      configComplete &= getJsonValue(top[FPSTR(_analogmic)]["pin"], audioPin);
    #else
      audioPin = -1;
    #endif

      configComplete &= getJsonValue(top[FPSTR(_digitalmic)]["type"],   dmType);
    #if  defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S3)
      if (dmType == 0) dmType = SR_DMTYPE;
      #if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3)
      if (dmType == 5) dmType = SR_DMTYPE;
      #endif
    #endif

      configComplete &= getJsonValue(top[FPSTR(_digitalmic)]["pin"][0], i2ssdPin);
      configComplete &= getJsonValue(top[FPSTR(_digitalmic)]["pin"][1], i2swsPin);
      configComplete &= getJsonValue(top[FPSTR(_digitalmic)]["pin"][2], i2sckPin);
      configComplete &= getJsonValue(top[FPSTR(_digitalmic)]["pin"][3], mclkPin);

      configComplete &= getJsonValue(top[FPSTR(_config)][F("squelch")], soundSquelch);
      configComplete &= getJsonValue(top[FPSTR(_config)][F("gain")],    sampleGain);
      configComplete &= getJsonValue(top[FPSTR(_config)][F("AGC")],     soundAgc);

      configComplete &= getJsonValue(top[FPSTR(_frequency)][F("scale")], FFTScalingMode);

      configComplete &= getJsonValue(top[FPSTR(_dynamics)][F("limiter")], limiterOn);
      configComplete &= getJsonValue(top[FPSTR(_dynamics)][F("rise")],  attackTime);
      configComplete &= getJsonValue(top[FPSTR(_dynamics)][F("fall")],  decayTime);
#endif
      configComplete &= getJsonValue(top["sync"]["port"], audioSyncPort);
      configComplete &= getJsonValue(top["sync"]["mode"], audioSyncEnabled);
      configComplete &= getJsonValue(top["sync"][FPSTR(_syncTransport)], audioSyncTransport, (uint8_t)0);
      // NEW: ESP-NOW channel override (0 = auto, 1..13 = manual)
      configComplete &= getJsonValue(top["sync"][FPSTR(_syncChannel)], userSyncChannel, (uint8_t)0);
      // NEW: MAC lock toggle (default true). If the field is missing on
      // first boot, fall back to the default and let WLED save it.
      if (!getJsonValue(top["sync"][FPSTR(_macLock)], macLockEnabled)) {
        macLockEnabled = true;
        configComplete = false;
      }

      if (initDone) {
        if ((oldAddPalettes && !addPalettes) || (oldAddPalettes && !enabled)) removeAudioPalettes();
        if ((addPalettes && !oldAddPalettes && enabled) || (addPalettes && !oldEnabled && enabled)) createAudioPalettes();
      }
      return configComplete;
    }


    void appendConfigData(Print& uiScript) override
    {
      uiScript.print(F("ux='AudioReactive';"));
#ifdef ARDUINO_ARCH_ESP32
      uiScript.print(F("uxp=ux+':digitalmic:pin[]';"));
      uiScript.print(F("dd=addDropdown(ux,'digitalmic:type');"));
    #if  !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
      uiScript.print(F("addOption(dd,'Generic Analog',0);"));
    #endif
      uiScript.print(F("addOption(dd,'Generic I2S',1);"));
      uiScript.print(F("addOption(dd,'ES7243',2);"));
      uiScript.print(F("addOption(dd,'SPH0654',3);"));
      uiScript.print(F("addOption(dd,'Generic I2S with Mclk',4);"));
    #if  !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3)
      uiScript.print(F("addOption(dd,'Generic PDM',5);"));
    #endif
    uiScript.print(F("addOption(dd,'ES8388',6);"));
      uiScript.print(F("addOption(dd,'None - network receive only',"));
      uiScript.print(SR_DMTYPE_NETWORK_ONLY);
      uiScript.print(F(");"));

      uiScript.print(F("dd=addDropdown(ux,'config:AGC');"));
      uiScript.print(F("addOption(dd,'Off',0);"));
      uiScript.print(F("addOption(dd,'Normal',1);"));
      uiScript.print(F("addOption(dd,'Vivid',2);"));
      uiScript.print(F("addOption(dd,'Lazy',3);"));

      uiScript.print(F("dd=addDropdown(ux,'dynamics:limiter');"));
      uiScript.print(F("addOption(dd,'Off',0);"));
      uiScript.print(F("addOption(dd,'On',1);"));
      uiScript.print(F("addInfo(ux+':dynamics:limiter',0,' On ');"));
      uiScript.print(F("addInfo(ux+':dynamics:rise',1,'ms <i>(&#x266A; effects only)</i>');"));
      uiScript.print(F("addInfo(ux+':dynamics:fall',1,'ms <i>(&#x266A; effects only)</i>');"));

      uiScript.print(F("dd=addDropdown(ux,'frequency:scale');"));
      uiScript.print(F("addOption(dd,'None',0);"));
      uiScript.print(F("addOption(dd,'Linear (Amplitude)',2);"));
      uiScript.print(F("addOption(dd,'Square Root (Energy)',3);"));
      uiScript.print(F("addOption(dd,'Logarithmic (Loudness)',1);"));
#endif

      uiScript.print(F("dd=addDropdown(ux,'sync:mode');"));
      uiScript.print(F("addOption(dd,'Off',0);"));
#ifdef ARDUINO_ARCH_ESP32
      uiScript.print(F("addOption(dd,'Send',1);"));
#endif
      uiScript.print(F("addOption(dd,'Receive',2);"));

      // Transport selector. UDP and ESP-NOW are mutually exclusive: only
      // the selected one starts in connected().
      uiScript.print(F("dd=addDropdown(ux,'sync:transport');"));
      uiScript.print(F("addOption(dd,'UDP (default)',0);"));
      uiScript.print(F("addOption(dd,'ESP-NOW',1);"));

      // NEW: ESP-NOW Wi-Fi channel override. 0 = auto (follow current
      // Wi-Fi channel), 1..13 = manual.
      uiScript.print(F("dd=addDropdown(ux,'sync:channel');"));
      uiScript.print(F("addOption(dd,'Auto (follow Wi-Fi)',0);"));
      for (int ch = 1; ch <= 13; ch++) {
        char buf[40];
        snprintf(buf, sizeof(buf), "addOption(dd,'Channel %d',%d);", ch, ch);
        uiScript.print(buf);
      }

      // NEW: MAC-lock toggle for the receiver (ESP-NOW only).
      uiScript.print(F("dd=addDropdown(ux,'sync:macLock');"));
      uiScript.print(F("addOption(dd,'Off',false);"));
      uiScript.print(F("addOption(dd,'On',true);"));

      uiScript.print(F("addInfo(ux+':sync:transport',1,'<i>UDP and ESP-NOW are exclusive</i>');"));
      uiScript.print(F("addInfo(ux+':sync:channel',1,'<i>ESP-NOW only</i>');"));
      uiScript.print(F("addInfo(ux+':sync:macLock',1,'<i>ESP-NOW only - ignore foreign senders</i>');"));

#ifdef ARDUINO_ARCH_ESP32
      uiScript.print(F("addInfo(ux+':digitalmic:type',1,'<i>requires reboot!</i>');"));
      uiScript.print(F("addInfo(uxp,0,'<i>sd/data/dout</i>','I2S SD');"));
      uiScript.print(F("addInfo(uxp,1,'<i>ws/clk/lrck</i>','I2S WS');"));
      uiScript.print(F("addInfo(uxp,2,'<i>sck/bclk</i>','I2S SCK');"));
      #if !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
        uiScript.print(F("addInfo(uxp,3,'<i>only use -1, 0, 1 or 3</i>','I2S MCLK');"));
      #else
        uiScript.print(F("addInfo(uxp,3,'<i>master clock</i>','I2S MCLK');"));
      #endif
#endif
    }


    uint16_t getId() override
    {
      return USERMOD_ID_AUDIOREACTIVE;
    }
};

void AudioReactive::removeAudioPalettes(void) {
  DEBUG_PRINTLN(F("Removing audio palettes."));
  palettes -= (int8_t)removeUsermodPalettes(_name);
  if (palettes < 0) palettes = 0;
}

void AudioReactive::createAudioPalettes(void) {
  if (palettes) return;
  DEBUG_PRINTLN(F("Adding audio palettes."));
  static const char *const palNames[MAX_PALETTES] PROGMEM = {_palName0, _palName1, _palName2};
  for (int i=0; i<MAX_PALETTES; i++) {
    if (usermodPalettes.size() < WLED_MAX_USERMOD_PALETTES) {
      usermodPalettes.push_back({CRGBPalette16(CRGB(BLACK)), _name, (uint8_t)i, palNames[i]});
      palettes++;
      DEBUG_PRINTLN(palettes);
    } else break;
  }
}

CRGB AudioReactive::getCRGBForBand(int x, int pal) {
  CRGB value;
  CHSV hsv;
  int b;
  switch (pal) {
    case 2:
      b = map(x, 0, 255, 0, NUM_GEQ_CHANNELS/2);
      hsv = CHSV(fftResult[b], 255, x);
      value = hsv;
      break;
    case 1:
      b = map(x, 1, 255, 0, 10);
      hsv = CHSV(fftResult[b], 255, map(fftResult[b], 0, 255, 30, 255));
      value = hsv;
      break;
    default:
      if (x == 1) {
        value = CRGB(fftResult[10]/2, fftResult[4]/2, fftResult[0]/2);
      } else if(x == 255) {
        value = CRGB(fftResult[10]/2, fftResult[0]/2, fftResult[4]/2);
      } else {
        value = CRGB(fftResult[0]/2, fftResult[4]/2, fftResult[10]/2);
      }
      break;
  }
  return value;
}

void AudioReactive::fillAudioPalettes() {
  if (!palettes) return;
  for (auto &ump : usermodPalettes) {
    if (ump.name != _name) continue;
    const int pal = ump.palIndex;
    uint8_t tcp[16];

    tcp[0] = 0;
    tcp[1] = 0;
    tcp[2] = 0;
    tcp[3] = 0;

    CRGB rgb = getCRGBForBand(1, pal);
    tcp[4] = 1;
    tcp[5] = rgb.r;
    tcp[6] = rgb.g;
    tcp[7] = rgb.b;

    rgb = getCRGBForBand(128, pal);
    tcp[8] = 128;
    tcp[9] = rgb.r;
    tcp[10] = rgb.g;
    tcp[11] = rgb.b;

    rgb = getCRGBForBand(255, pal);
    tcp[12] = 255;
    tcp[13] = rgb.r;
    tcp[14] = rgb.g;
    tcp[15] = rgb.b;

    ump.palette.loadDynamicGradientPalette(tcp);
  }
}

const char AudioReactive::_name[]       PROGMEM = "AudioReactive";
const char AudioReactive::_enabled[]    PROGMEM = "enabled";
const char AudioReactive::_config[]     PROGMEM = "config";
const char AudioReactive::_dynamics[]   PROGMEM = "dynamics";
const char AudioReactive::_frequency[]  PROGMEM = "frequency";
const char AudioReactive::_inputLvl[]   PROGMEM = "inputLevel";
#if defined(ARDUINO_ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
const char AudioReactive::_analogmic[]  PROGMEM = "analogmic";
#endif
const char AudioReactive::_digitalmic[] PROGMEM = "digitalmic";
const char AudioReactive::_addPalettes[]          PROGMEM = "add-palettes";
const char AudioReactive::_palName0[]              PROGMEM = "Ratio";
const char AudioReactive::_palName1[]              PROGMEM = "Hue";
const char AudioReactive::_palName2[]              PROGMEM = "Spectrum";
const char AudioReactive::UDP_SYNC_HEADER[]    PROGMEM = "00002";
const char AudioReactive::UDP_SYNC_HEADER_v1[] PROGMEM = "00001";
const char AudioReactive::_syncTransport[]     PROGMEM = "transport";
const char AudioReactive::_syncChannel[]       PROGMEM = "channel";   // NEW
const char AudioReactive::_macLock[]           PROGMEM = "macLock";   // NEW

static AudioReactive ar_module;
REGISTER_USERMOD(ar_module);
