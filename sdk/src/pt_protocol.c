#include "pt_sdk.h"
#include <string.h>

static uint16_t update(uint16_t crc, uint8_t byte) {
    unsigned bit; crc^=byte;
    for(bit=0;bit<8;++bit) crc=(uint16_t)((crc>>1)^((crc&1u)?0xA001u:0u));
    return crc;
}
uint16_t pt_crc16(const uint8_t *data, size_t size) {
    uint16_t crc=0xFFFFu; size_t i;
    for(i=0;i<size;++i) crc=update(crc,data[i]);
    return crc;
}
uint16_t pt_frame_crc(const uint8_t *frame, size_t size) {
    uint16_t crc=0xFFFFu; size_t i;
    for(i=0;i<size;++i) if(i!=6 && i!=7) crc=update(crc,frame[i]);
    return crc;
}
size_t pt_encode(uint16_t command, const uint8_t *body, size_t size, uint8_t *out, size_t capacity) {
    size_t total; uint16_t crc;
    if(size>65525u || size+10u>capacity || (size && !body)) return 0;
    total=size+10u;
    out[0]=0xAA;out[1]=0x55;out[2]=(uint8_t)(total>>8);out[3]=(uint8_t)total;
    out[4]=(uint8_t)(command>>8);out[5]=(uint8_t)command;out[6]=out[7]=0;
    if(size) memmove(out+8,body,size);
    out[total-2]=0x55;out[total-1]=0xAA;
    crc=pt_frame_crc(out,total);out[6]=(uint8_t)(crc>>8);out[7]=(uint8_t)crc;
    return total;
}
