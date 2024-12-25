//
// Created by Henson on 2023-12-17.
//

#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#include "main.h"
#include "stdint.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif


float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);


#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/

class Dwt
{
private:
    uint32_t CPU_FREQ_Hz=0,CPU_FREQ_Hz_ms=0, CPU_FREQ_Hz_us=0, CYCCNT_RountCount=0, CYCCNT_LAST=0;
    uint64_t CYCCNT64=0;
public:
    typedef struct {
        uint32_t s;
        uint16_t ms;
        uint16_t us;
    } DWT_Time_t;

    DWT_Time_t SysTime{};
public:

    explicit Dwt(uint32_t SYS_Freq_MHz);

    void DWT_SysTimeUpdate(void);
    void DWT_CNT_Update(void);
    void DWT_Delay(float Delay);
    uint64_t DWT_GetTimeline_us(void);
    float DWT_GetDeltaT(uint32_t *cnt_last);
    double DWT_GetDeltaT64(uint32_t *cnt_last);
    float DWT_GetTimeline_s(void);
    float DWT_GetTimeline_ms(void);


};

#endif //__BSP_DWT_H