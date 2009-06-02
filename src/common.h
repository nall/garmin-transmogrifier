#ifndef __COMMON_H__
#define __COMMON_H__

#define EXIT_SUCCESS 1
#define EXIT_FAILURE 0

#define IS_SET(reg, bit) (((reg) & _BV(bit)) != 0)
#define IS_CLEAR(reg, bit) (IS_SET(reg, bit) == 0)

#endif // __COMMON_H__
