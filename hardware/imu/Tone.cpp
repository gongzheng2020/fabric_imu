#include "Tone.h"

Melody melody1(14, (uint16_t[]){
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4},
    (uint16_t[]){
        4, 4, 4, 4, 4, 4, 2,
        4, 4, 4, 4, 4, 4, 2});

Melody melody2(14, (uint16_t[]){
    NOTE_E4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_G4, NOTE_F4, NOTE_E4, NOTE_D4,
    NOTE_C4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_D4},
    (uint16_t[]){
        4, 4, 4, 4, 4, 4, 4, 4,
        4, 4, 4, 4, 4, 2});

// 定义启动音
Melody startupMelody(3, (uint16_t[]){
    NOTE_C4, NOTE_E4, NOTE_G4},
    (uint16_t[]){
        4, 4, 4});

// 定义错误音
Melody errorMelody(2, (uint16_t[]){
    NOTE_C4, NOTE_C3},
    (uint16_t[]){
        2, 2});

// 定义成功音
Melody successMelody(3, (uint16_t[]){
    NOTE_G4, NOTE_A4, NOTE_B4},
    (uint16_t[]){
        4, 4, 4});

Tone::Tone(int8_t pin, int8_t channel)
{
    _channel = channel;
    _pin = pin;
}

void Tone::tone(uint16_t frequency, uint16_t duration)
{
    writeFreq(frequency);
    if (duration > 0)
    {
        toneDelay(duration);
        noTone();
    }
}

void Tone::toneDelay(uint16_t duration)
{
    vTaskDelay(pdMS_TO_TICKS(duration));
}

void Tone::play(const Melody &melody)
{
    for (uint16_t i = 0; i < melody.length; i++)
    {
        tone(melody.notes[i], 1000 / melody.durations[i]);
        toneDelay(melody.durations[i] * _noteBias); // 延迟来满足音调的持续时间
    }
}

void Tone::noTone()
{
    writeFreq(0);
}

void Tone::writeFreq(uint16_t frequency)
{
    if (frequency)
    {
        if (_channel >= 0)
        {
            ledcAttachChannel(_pin, frequency, 10, _channel);
        }
        else
        {
            ledcAttach(_pin, frequency, 10);
        }
        ledcWriteTone(_pin, frequency);
    }
    else
    {
        ledcWriteTone(_pin, frequency);
        ledcDetach(_pin);
    }
}