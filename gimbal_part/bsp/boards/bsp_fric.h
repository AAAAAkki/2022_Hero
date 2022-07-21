#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "struct_typedef.h"
// TEST_BULLET: 1 -- use training bullet, 0 -- official game bullet 
#define TEST_BULLET 0
#if TEST_BULLET
#define FRIC_16 5565
#else
#define FRIC_16 5920
#endif

#define FRIC_OFF 0

extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif
