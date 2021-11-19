#ifndef TSTAR_H
#define TSTAR_H

#include "tstar_imp.h"

#ifdef __cplusplus
extern "C" 
{  
#endif

 __declspec(dllimport) void* tstar_create();
 __declspec(dllimport) int tstar_set_map(void* handler, int* map, int width, int height, Coordinate start, Coordinate goal);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // TSTAR_H