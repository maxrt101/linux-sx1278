/** ========================================================================= *
 *
 * @file compiler.h
 * @date 20-07-2024
 * @author Maksym Tkachuk <max.r.tkachuk@gmail.com>
 *
 *  ========================================================================= */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ================================================================= */
/* Defines ================================================================== */
#ifndef __STATIC
#define __STATIC static
#endif

#ifndef __INLINE
#define __INLINE static
#endif

#ifndef __STATIC_INLINE
#define __STATIC_INLINE static inline
#endif

#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE __attribute__((always_inline)) static inline
#endif

#ifndef __PACKED
#define __PACKED __attribute__((packed))
#endif

#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT struct __attribute__((packed))
#endif

#ifndef __PACKED_UNION
#define __PACKED_UNION union __attribute__((packed))
#endif

#ifndef __PACKED_ENUM
#define __PACKED_ENUM enum __attribute__((packed))
#endif

#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif

#ifndef __NAKED
#define __NAKED __attribute__((naked))
#endif

#ifndef __PURE
#define __PURE __attribute__((pure))
#endif

#ifndef __CONST
#define __CONST __attribute__((const))
#endif

#ifndef __RAMFUNC
#define __RAMFUNC __attribute__((noinline, section(".RamFunc")))
#endif

#ifndef __OPTIMIZE
#define __OPTIMIZE(N) __attribute__((optimize(STRINGIFY(O##N))))
#endif

#ifndef __UNUSED
#define __UNUSED __attribute__((unused))
#endif

#ifndef __UNREACHABLE
#define __UNREACHABLE() __builtin_unreachable()
#endif

#ifndef __ALIAS
#define __ALIAS(new_name) __attribute__((alias(new_name)))
#endif

#ifndef __ALIGN
#define __ALIGN(n) __attribute__((aligned(n)))
#endif

#ifndef __LOCATION
#define __LOCATION(address) __attribute__((location(address)))
#endif

#ifndef __SECTION
#define __SECTION(__section) __attribute__((section("." #__section)))
#endif

#ifndef __CONSTRUCTOR
#define __CONSTRUCTOR(priority) __attribute__((constructor(priority)))
#endif

#ifndef __DEPRECATED
#define __DEPRECATED __attribute__((deprecated))
#endif

#ifndef offsetof
#define offsetof(__type, __member) __builtin_offsetof(__type, __member)
#endif

#ifndef __STATIC_ASSERT
#define __STATIC_ASSERT(__expr, __msg) _Static_assert(__expr, __msg)
#endif

#ifndef __NON_NULL
#define __NON_NULL(...) __attribute__((__nonnull__ __VA_ARGS__))
#endif

#ifndef __NO_INLINE
#define __NO_INLINE __attribute__((noinline))
#endif

#ifndef __NORETURN
#define __NORETURN __attribute__((noreturn))
#endif

/* Macros =================================================================== */
/* Enums ==================================================================== */
/* Types ==================================================================== */
/* Variables ================================================================ */
/* Shared functions ========================================================= */

#ifdef __cplusplus
}
#endif