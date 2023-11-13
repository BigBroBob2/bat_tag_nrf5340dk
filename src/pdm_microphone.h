/*
Use I2S interface to transfer bit stream from PDM microphone
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/i2s.h>
#include <nrfx_i2s.h>

#define I2S_MCK    (32+9) 
#define I2S_SCK    (32+10)
#define I2S_LRCK   (32+12)
#define I2S_SDIN   (32+11)

typedef void (*audio_processing_callback_t)(
  void* cb_data,
  unsigned int sample_count,
  float* tx,
  const float* rx
);
typedef void (*audio_dropout_callback_t)(void* cb_data);

typedef struct {
  audio_processing_callback_t processing_cb;
  audio_dropout_callback_t dropout_cb;
  void* cb_data;
} audio_callbacks_t;


