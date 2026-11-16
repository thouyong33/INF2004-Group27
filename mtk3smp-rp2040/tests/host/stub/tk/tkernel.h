/*
 * tkernel.h (host stub)
 * Just the micro T-Kernel types the host tests' headers name, so the
 * kernel-free modules (terrain.c, cmd_map.c) build with the PC's gcc.
 */

#ifndef HOST_STUB_TKERNEL_H
#define HOST_STUB_TKERNEL_H

#include <stddef.h>
#include <stdint.h>

typedef int ER;
typedef int INT;
typedef unsigned int UINT;
typedef int ID;
typedef int TMO;
typedef unsigned char UB;

#define E_OK    0

#endif /* HOST_STUB_TKERNEL_H */
