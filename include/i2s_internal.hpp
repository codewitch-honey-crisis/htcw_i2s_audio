#pragma once
#include <sfx_core.hpp>
#include "i2s_common.hpp"
#ifndef ESP32
#error "This library only supports the ESP32"
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
namespace arduino {
enum struct i2s_channels {
    left = 0,
    right,
    both
};
template<i2s_channels OutputChannelConfiguration = i2s_channels::both, bool UseApll = true, size_t DmaSamples = 512, size_t DmaBufferCount = 14>
class i2s_internal final : public sfx::audio_destination {
    
public:
    constexpr static const size_t output_sample_rate = 44100;
    constexpr static const size_t output_bit_depth = 16;
    constexpr static const i2s_channels output_channel_configuration = OutputChannelConfiguration;
    constexpr static const size_t output_channels = 1+(output_channel_configuration==i2s_channels::both);
    constexpr static const size_t dma_samples = DmaSamples;
    constexpr static const size_t dma_buffer_count = DmaBufferCount;
    constexpr static const size_t dma_size = dma_samples*output_channels;
    constexpr static const bool use_apll = UseApll;
    using type = i2s_internal;
private:
    bool m_initialized;
    static uint8_t m_out_buffer[];
    size_t copy_raw(const void* samples, size_t sample_count) {
        size_t result = 0;
        const int16_t* p = (const int16_t*)samples;
        uint16_t* out = (uint16_t*)i2s_internal::m_out_buffer;
        while(sample_count) {
            size_t to_write = sample_count<dma_samples?sample_count:dma_samples;
            for(int i = 0;i<to_write;++i) {
                *(out++)=uint16_t(*(p++)+32768);
            }
            size_t written;
            i2s_write((i2s_port_t)0,i2s_internal::m_out_buffer,to_write*2,&written,portMAX_DELAY);
            size_t samples_written = written/2;
            sample_count-=samples_written;
            result+=samples_written;
            if(samples_written!=to_write) {
                return samples_written;
            }
        }
        return result;
    }
public:
    i2s_internal() : m_initialized(false) {

    }
    inline bool initialized() const { return m_initialized; }
    bool initialize() {
        if(!m_initialized) {
            i2s_config_t i2s_config;
            memset(&i2s_config,0,sizeof(i2s_config_t));
            i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN);
            i2s_config.sample_rate = output_sample_rate;
            i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
            i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
                /*(output_channel_configuration==i2s_channels::left?I2S_CHANNEL_FMT_ALL_LEFT:
                        output_channel_configuration==i2s_channels::right?I2S_CHANNEL_FMT_ALL_RIGHT:
                            I2S_CHANNEL_FMT_RIGHT_LEFT);*/
            i2s_config.communication_format = I2S_COMM_FORMAT_STAND_MSB;
            i2s_config.dma_buf_count = dma_buffer_count;
            i2s_config.dma_buf_len = dma_size;
            i2s_config.use_apll = use_apll;
            i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL2;
            if(ESP_OK!=i2s_driver_install((i2s_port_t)0, &i2s_config, 0, NULL)) {
                return false;
            }
            if(ESP_OK!=i2s_set_pin((i2s_port_t)0,nullptr)) {
                return false;
            }
            i2s_set_dac_mode((i2s_dac_mode_t)(output_channel_configuration==i2s_channels::left?I2S_DAC_CHANNEL_LEFT_EN:
                        output_channel_configuration==i2s_channels::right?I2S_DAC_CHANNEL_RIGHT_EN:
                            I2S_DAC_CHANNEL_BOTH_EN));
            m_initialized = true;
        }
        return true;
    }
    virtual size_t bit_depth() const {
        return output_bit_depth;
    }
    virtual size_t sample_rate() const {
        return output_sample_rate;
    }
    virtual size_t channels() const {
        return 2;
    }
    virtual sfx::audio_format format() const {
        return sfx::audio_format::pcm;
    }
    
    virtual size_t write(const void* samples, size_t sample_count) {
        if(samples==nullptr) {
            return 0;
        }
        if(sample_count==0) {
            return 0;
        }
        if(!initialize()) {
            return 0;
        }
        size_t result = 0;
        result = copy_raw(samples,sample_count);
        vTaskDelay(0);
        return result;
    }
    inline bool start() {
        if(!initialize()) {
            return false;
        }
        return ESP_OK==i2s_start((i2s_port_t)0);
    }
    inline bool stop() {
        if(!initialize()) {
            return false;
        }
        return ESP_OK==i2s_stop((i2s_port_t)0);
    }
    inline void clear_dma() {
        i2s_zero_dma_buffer((i2s_port_t)0);
    }
};
template<i2s_channels OutputChannelConfiguration, bool UseApll, size_t DmaSamples, size_t DmaBufferCount>
uint8_t i2s_internal<OutputChannelConfiguration,UseApll,DmaSamples,DmaBufferCount>::m_out_buffer[DmaSamples*2]={0};
}