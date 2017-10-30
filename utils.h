#ifndef UTILS_h
#define UTILS_h

class TimeCounter{
public:
    unsigned long _prev;
    TimeCounter():_prev(micros()){};
    ~TimeCounter(){};

    float update() {
        auto new_time = micros();
        float dt = (float)(new_time - _prev)/1000000.0;
        _prev = new_time;
        return dt;
    }
};

void to16bit(uint8_t* src, int16_t* tgt, uint tgt_count = 1, bool little_endian = false){
    for (uint i = 0; i < tgt_count; i++){
        uint index = i << 1;           
        tgt[i] = (((int16_t)src[index + little_endian]) << 8) | src[index + 1 - little_endian]; 
    }
}

void setNthBit(byte* dist, byte n, bool value){
    byte mask = (1<<n);
    if (value)
        *dist |= mask;
    else
        *dist &= ~mask; 
}

#endif