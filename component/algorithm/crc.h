//
// Created by Henson on 2023-11-18.
//

#ifndef __CRC_H
#define __CRC_H

#include "common_inc.h"

/*---------------------------- C Scope ---------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len);

#ifdef __cplusplus
}
#endif
/*---------------------------- C++ Scope ---------------------------*/


#endif //__CRC_H
