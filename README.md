# I2S Audio

htcw_i2s_audio provides SFX support for MCUs with I2S hardware.
Currently, only the ESP32 is supported.

Add the following to your platformio.ini

```ini
lib_ldf_mode = deep
lib_deps = 
  codewitch-honey-crisis/htcw_i2s_audio
```