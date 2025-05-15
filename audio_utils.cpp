#include "audio_utils.h"
#include <stdexcept>
#include <iostream>

snd_pcm_t* init_audio(const char* device, snd_pcm_stream_t stream_type) {
    snd_pcm_t* handle;
    int err = snd_pcm_open(&handle, device, stream_type, 0);
    if (err < 0) throw std::runtime_error("Failed to open ALSA device");

    // Устанавливаем формат: моно, 44100 Гц, signed 16-bit little-endian
    snd_pcm_set_params(handle,
                       SND_PCM_FORMAT_S16_LE,
                       SND_PCM_ACCESS_RW_INTERLEAVED,
                       1,           // 1 канал (моно)
                       24000,       // частота
                       1,           // 1 = non-blocking mode
                       500'000);      // латенция в микросекундах

    return handle;
}

void deinit_audio(snd_pcm_t* handle) {
    snd_pcm_drain(handle);
    std::cout<<handle<<" drain done"<<std::endl;
    snd_pcm_close(handle);
    std::cout<<handle<<" close done"<<std::endl;
}