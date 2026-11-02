#ifndef RADIO_PROTOCOL_H
#define RADIO_PROTOCOL_H
#include <stdint.h>
#include <string.h>
#define RADIO_PACKET_SIZE 32U
#define RADIO_PROTOCOL_VERSION 2U
#define RADIO_PERIOD_MS 20U
#define RADIO_TIMEOUT_MS 300U
#define RADIO_FLAG_ENABLED 1U
#define RADIO_FLAG_ESTOP 2U
#define RADIO_FLAG_FRESH 4U
#define RADIO_FLAG_ARM_AUTO 8U
#define RADIO_FLAG_ARM_64 16U
#define RADIO_FLAG_SW12_DOWN 32U
#define RADIO_FLAGS_VALID 0x3FU
/* Keys: turnL,turnR,grip1,grip2,speed,estop,enable,rotateB. */
#define RADIO_ACTION_KEYS 0x9FU
#define RADIO_ENABLE_KEY 0x40U
typedef struct {
    uint32_t sequence, arm_token;
    int16_t axis[4];
    uint16_t servo_ddeg[4];
    uint8_t flags, gear, keys;
} RadioPacket_t;
static inline void RadioPut16(uint8_t *p,uint16_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static inline uint16_t RadioGet16(const uint8_t *p)
{ return (uint16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
static inline void RadioPut32(uint8_t *p,uint32_t v)
{ RadioPut16(p,(uint16_t)v); RadioPut16(p+2,(uint16_t)(v>>16)); }
static inline uint32_t RadioGet32(const uint8_t *p)
{ return (uint32_t)RadioGet16(p)|((uint32_t)RadioGet16(p+2)<<16); }
static inline uint16_t RadioCRC(const uint8_t *p,uint32_t n)
{
    uint16_t crc=0xFFFFU;
    while(n--) {
        crc^=(uint16_t)*p++<<8;
        for(unsigned i=0;i<8;i++)
            crc=(uint16_t)((crc&0x8000U)?(crc<<1)^0x1021U:crc<<1);
    }
    return crc;
}
static inline void RadioEncode(uint8_t p[32],const RadioPacket_t *v)
{
    memset(p,0,32);
    p[0]=0x4FU; p[1]=0x44U; p[2]=RADIO_PROTOCOL_VERSION; p[3]=v->flags;
    RadioPut32(p+4,v->sequence); RadioPut32(p+8,v->arm_token);
    for(unsigned i=0;i<4;i++) {
        RadioPut16(p+12+2*i,(uint16_t)v->axis[i]);
        RadioPut16(p+20+2*i,v->servo_ddeg[i]);
    }
    p[28]=v->gear; p[29]=v->keys;
    RadioPut16(p+30,RadioCRC(p,30));
}
static inline uint8_t RadioDecode(const uint8_t p[32],RadioPacket_t *v)
{
    if(p[0]!=0x4F || p[1]!=0x44 || p[2]!=RADIO_PROTOCOL_VERSION || (p[3]&~RADIO_FLAGS_VALID) ||
       RadioGet16(p+30)!=RadioCRC(p,30) || p[28]>2U) return 0U;
    v->flags=p[3]; v->sequence=RadioGet32(p+4); v->arm_token=RadioGet32(p+8);
    v->gear=p[28]; v->keys=p[29];
    for(unsigned i=0;i<4;i++) {
        uint16_t u=RadioGet16(p+12+2*i);
        int32_t s=u>=32768U?(int32_t)u-65536:(int32_t)u;
        if(s < -1000 || s > 1000) return 0U;
        v->axis[i]=(int16_t)s;
        v->servo_ddeg[i]=RadioGet16(p+20+2*i);
    }
    return v->servo_ddeg[0]>=300U && v->servo_ddeg[0]<=1500U &&
           v->servo_ddeg[1]<=1750U && v->servo_ddeg[2]<=700U &&
           v->servo_ddeg[3]<=700U;
}
static inline uint8_t RadioCentered(const RadioPacket_t *p)
{
    for(unsigned i=0;i<4;i++)
        if(p->axis[i] < -50 || p->axis[i] > 50) return 0U;
    return (p->keys & RADIO_ACTION_KEYS)==0U &&
           !(p->flags & RADIO_FLAG_SW12_DOWN);
}
#endif