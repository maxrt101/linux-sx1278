/** ========================================================================= *
 *
 * @file util.h
 * @date 20-07-2024
 * @author Maksym Tkachuk <max.r.tkachuk@gmail.com>
 *
 * @brief Preprocessor utilities
 *
 *  ========================================================================= */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ================================================================= */
#include <stdint.h>
#include "compiler.h"

/* Defines ================================================================== */
/* Macros =================================================================== */
/**
 * Supress 'unused variable' and 'unused parameter' warnings
 */
#define UTIL_UNUSED(x) (void)(x)

/**
 * Concatenates 2 tokens
 */
#define UTIL_CAT_RAW(a, b) a ## b

/**
 * Concatenates 2 tokens, deferred
 */
#define UTIL_CAT(a, b) UTIL_CAT_RAW(a, b)

/**
 * Converts token to string literal
 */
#define UTIL_STRINGIFY(str) #str

/**
 * Determines smallest number from the 2
 */
#define UTIL_MIN(a, b) ((a) < (b) ? (a) : (b))

/**
 * Determines biggest number from the 2
 */
#define UTIL_MAX(a, b) ((a) > (b) ? (a) : (b))

/**
 * Calculates size of an array
 */
#define UTIL_ARR_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

/**
 * Caps x between min and max
 */
#define UTIL_CAP(x, min, max) \
    ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
 * Maps value of range [min1, max1] to range[min2, max2]
 */
#define UTIL_MAP_RANGE(value, min1, max1, min2, max2)                           \
  (((value) * ((max2) - (min2))) / ((max1) - (min1)))

/**
 * Maps value using table of ranges
 *
 * Table is a list of util_map_range_table_entry_t, which represents
 * range [from, to] and corresponding value for this range
 *
 * @note if value is not in range, result won't be set
 *
 * @param[in]  table A list of util_map_range_table_entry_t
 * @param[in]  value Value to be mapped
 * @param[out] result Variable to put the result in
 *
 */
#define UTIL_MAP_RANGE_TABLE(_table, _value, _result)                           \
  do {                                                                          \
    for (int i = 0; ((_table)[i].from && (_table)[i].to); ++i) {                \
      if ((_value) >= (_table)[i].from && (_value) <= (_table)[i].to) {         \
         (_result) = (_table)[i].value;                                         \
         break;                                                                 \
      }                                                                         \
    }                                                                           \
  } while (0)

/**
 * Finds a range from mapped value, reverse operation to UTIL_MAP_RANGE_TABLE
 *
 * Table is a list of util_map_range_table_entry_t, which represents
 * range [from, to] and corresponding value for this range
 *
 * @note if value is not in range, result won't be set
 *
 * @param[in]  table A list of util_map_range_table_entry_t
 * @param[in]  value Value that corresponds to util_map_range_table_entry_t::value
 * @param[out] result Variable to put the result in
 *
 */
#define UTIL_MAP_RANGE_TABLE_REV(_table, _value, _result)                       \
  do {                                                                          \
    for (int i = 0; ((_table)[i].from && (_table)[i].to); ++i) {                \
      if ((_value) == (_table)[i].value) {                                      \
         (_result) = (_table)[i].from;                                          \
         break;                                                                 \
      }                                                                         \
    }                                                                           \
  } while (0)

/**
 * Gets first arg from the pack
 */
#define UTIL_GET_ARG1(x, ...) x

/**
 * Gets all args but the first (2+)
 */
#define UTIL_GET_ARG_BUT_1(x, ...) __VA_ARGS__

/**
 * Gets second arg from the pack
 */
#define UTIL_GET_ARG2(x, ...) UTIL_GET_ARG1(__VA_ARGS__)

/**
 * Gets second arg from the pack, deferred
 */
#define UTIL_GET_ARG2_OBSTRUCT(...) UTIL_GET_ARG2(__VA_ARGS__)

/**
 * Gets all args for the second one
 */
#define UTIL_GET_ARG_BUT_2(x, ...) UTIL_GET_ARG_BUT_1(__VA_ARGS__)

/**
 * Return value sequence for UTIL_VA_ARGS_COUNT
 */
#define UTIL_VA_ARGS_COUNT_SEQ()                                                      \
    63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48,                   \
    47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32,                   \
    31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16,                   \
    15, 14, 13, 12, 11, 10,  9,  8,  7,  6,  5,  4,  3,  2,  1,  0

/**
 * Counts vargs by passing them and sequence here
 */
#define UTIL_VA_ARGS_COUNT_N( \
     _1,  _2,  _3,  _4,  _5,  _6,  _7,  _8,  _9, _10, _11, _12, _13, _14, _15, _16,   \
    _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32,   \
    _33, _34, _35, _36, _37, _38, _39, _40, _41, _42, _43, _44, _45, _46, _47, _48,   \
    _49, _50, _51, _52, _53, _54, _55, _56, _57, _58, _59, _60, _61, _62, _63, N, ... \
    ) N

/**
 * Implementation of UTIL_VA_ARGS_COUNT
 */
#define UTIL_VA_ARGS_COUNT_IMPL(...)                                            \
    UTIL_VA_ARGS_COUNT_N(__VA_ARGS__)

/**
 * Count number of vargs in a macro
 *
 * @note If empty - returns 1
 */
#define UTIL_VA_ARGS_COUNT(...)                                                 \
    UTIL_VA_ARGS_COUNT_IMPL(__VA_ARGS__, UTIL_VA_ARGS_COUNT_SEQ())

/**
 * Helper macro for UTIL_IF_1
 */
#define UTIL_IF1_1 __X__,

/**
 * Expanded to if_true if condition is 1, __VA_ARGS__ otherwise
 *
 * Works by concatenating value 'condition' with UTIL_IF1_, if 'condition'
 * is 1 - it will produce UTIL_IF1_1, which will produce '__X__,' and
 * second argument will be if_true, if 'condition' is anything but 1
 * (let's say 0), UTIL_IF1_0 is empty, and doesn't contain a comma, so
 * second argument will be __VA_ARGS__, and it will get called
 */
#define UTIL_IF_1_IMPL(condition, if_true, ...)                                 \
    UTIL_GET_ARG2_OBSTRUCT(UTIL_CAT_RAW(UTIL_IF1_, condition) if_true, __VA_ARGS__)

/**
 * Expanded to if_true if condition is 1, __VA_ARGS__ otherwise
 *
 * @note condition must be literally 1
 */
#define UTIL_IF_1(condition, if_true, ...)                                      \
    UTIL_IF_1_IMPL(condition, if_true, __VA_ARGS__)

/**
 * Helper macro for UTIL_IF_0
 */
#define UTIL_IF0_0 __X__,

/**
 * Expanded to if_true if condition is 0, __VA_ARGS__ otherwise
 *
 * Works by concatenating value 'condition' with UTIL_IF0_, if 'condition'
 * is 0 - it will produce UTIL_IF0_0, which will produce '__X__,' and
 * second argument will be if_true, if 'condition' is anything but 0
 * (let's say 1), UTIL_IF0_1 is empty, and doesn't contain a comma, so
 * second argument will be __VA_ARGS__, and it will get called
 */
#define UTIL_IF_0_IMPL(condition, if_true, ...)                                 \
    UTIL_GET_ARG2_OBSTRUCT(UTIL_CAT_RAW(UTIL_IF0_, condition) if_true, __VA_ARGS__)

/**
 * Expanded to if_true if condition is 0, __VA_ARGS__ otherwise
 *
 * @note condition must be literally 0
 */
#define UTIL_IF_0(condition, if_true, ...)                                      \
    UTIL_IF_0_IMPL(condition, if_true, __VA_ARGS__)

/**
 * Helper macro for UTIL_IF_NON0
 */
#define UTIL_IF_NON0_0 __X__,

/**
 * Expanded to if_true if condition is anything but 0, if_false otherwise
 *
 * Works by concatenating value 'condition' with UTIL_COND_NON0_, if
 * 'condition' is 0 - it will produce UTIL_COND_NON0_0, which will
 * produce '__X__,' and second argument will be if_false, if 'condition'
 * is anything but 0 (let's say 1), UTIL_COND_NON0_1 is empty, and
 * doesn't contain a comma, so second argument will be if_true, and
 * it will get called
 */
#define UTIL_IF_NON0(condition, if_true, if_false)                              \
    UTIL_GET_ARG2_OBSTRUCT(UTIL_CAT_RAW(UTIL_IF_NON0_, condition) if_false, if_true)

/**
 * Helper macro for UTIL_IF_EMPTY
 */
#define UTIL_IF_EMPTY_ __X__,

/**
 * Expanded to if_true if condition is empty, if_false otherwise
 *
 * Works by concatenating value 'condition' with UTIL_IF_EMPTY_, if
 * 'condition' is empty - it will produce UTIL_IF_EMPTY_, which will
 * produce '__X__,' and second argument will be if_true, if 'condition'
 * is anything else (let's say 1), UTIL_IF_EMPTY_1 is empty, and doesn't
 * contain a comma, so second argument will be if_false, and it will get
 * called
 */
#define UTIL_IF_EMPTY(condition, if_true, if_false)                             \
    UTIL_GET_ARG2_OBSTRUCT(UTIL_CAT_RAW(UTIL_IF_EMPTY_, condition) if_true, if_false)

/* Enums ==================================================================== */
/* Types ==================================================================== */
/**
 * Table entry for UTIL_MAP_RANGE_TABLE
 *
 * Represents singe table mapping of range ['from', 'to'] to 'value'
 */
typedef struct {
  int32_t from;
  int32_t to;
  int32_t value;
} util_map_range_table_entry_t;

/* Variables ================================================================ */
/* Shared functions ========================================================= */

#ifdef __cplusplus
}
#endif
