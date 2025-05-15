#pragma once
#include <alsa/asoundlib.h>

snd_pcm_t* init_audio(const char* device, snd_pcm_stream_t stream_type);
void deinit_audio(snd_pcm_t* handle);