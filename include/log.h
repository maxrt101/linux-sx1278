/** ========================================================================= *
 *
 * @file log.h
 * @date 23-07-2024
 * @author Maksym Tkachuk <max.r.tkachuk@gmail.com>
 *
 * @brief Logging support
 *
 *  ========================================================================= */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ================================================================= */
#include <stdarg.h>
#include <stddef.h>
#include "util.h"

/* Defines ================================================================== */
/**
 * Max size of log line
 */
#ifndef LOG_LINE_SIZE
#define LOG_LINE_SIZE 256
#endif

/**
 * Use colors in logs
 */
#ifndef USE_COLOR_LOG
#define USE_COLOR_LOG 1
#endif

/**
 * Where to put logs
 */
#ifndef LOG_OUTPUT_FILE
#define LOG_OUTPUT_FILE stdout
#endif

/**
 * Helper macros to check for LOG_TAG presence
 */
#define LOG_TAG_CHECK_LOG_TAG 0
#define LOG_TAG_CHECK() UTIL_IF_NON0(UTIL_CAT(LOG_TAG_CHECK_, LOG_TAG), 1, 0)

/**
 * Helper macros to check for LOG_ENABLE_* presence
 */
#define LOG_ENABLE_LOG_TAG 0
#define LOG_ENABLE_CHECK() UTIL_IF_NON0(UTIL_CAT(LOG_ENABLE_, LOG_TAG), 1, 0)

/**
 * Log print helper macro, calls log_fmt to print the line
 */
#define log_print_raw(level, fmt, ...) \
    log_fmt(__FILE__, __LINE__, level, NULL, fmt, ##__VA_ARGS__)

/**
 * Log print helper macro, calls log_module_fmt to print the line
 */
#define log_print_module_raw(level, module, fmt, ...) \
    log_fmt(__FILE__, __LINE__, level, UTIL_STRINGIFY(module), fmt, ##__VA_ARGS__)

/**
 * Log print helper macro, calls log_module_fmt to print the line
 * Also checks for log level to be above warning, even if logging is disabled,
 * this will still print warn/error/fatal logs
 */
#define log_print_module_level_check(level, module, fmt, ...)                \
    ((level >= LOG_WARNING)                                                  \
        ? log_fmt(__FILE__, __LINE__, level, UTIL_STRINGIFY(module), fmt, ##__VA_ARGS__)  \
        : 0)

/**
 * Log print helper macro, called when LOG_TAG is defined
 */
#define log_print_module(level, fmt, ...) \
    UTIL_IF_1(LOG_ENABLE_CHECK(),                                         \
        log_print_module_raw(level, LOG_TAG, fmt, ##__VA_ARGS__),         \
        log_print_module_level_check(level, LOG_TAG, fmt, ##__VA_ARGS__))

/**
 * Generic log print macro
 *
 * Checks if LOG_TAG is present, if so LOG_ENABLE_* is also checked
 */
#define log_print(level, fmt, ...)                  \
    UTIL_IF_0(LOG_TAG_CHECK(),                      \
      log_print_raw(level, fmt, ##__VA_ARGS__),     \
      log_print_module(level, fmt, ##__VA_ARGS__))

/**
 * Print log with DEBUG log level
 *
 * @param[in] fmt Format (printf)
 * @param[in] ... Parameters
 */
#define log_debug(fmt, ...) \
    log_print(LOG_DEBUG, fmt, ##__VA_ARGS__)

/**
 * Print log with INFO log level
 *
 * @param[in] fmt Format (printf)
 * @param[in] ... Parameters
 */
#define log_info(fmt, ...) \
    log_print(LOG_INFO, fmt, ##__VA_ARGS__)

/**
 * Print log with WARNING log level
 *
 * @param[in] fmt Format (printf)
 * @param[in] ... Parameters
 */
#define log_warn(fmt, ...) \
    log_print(LOG_WARNING, fmt, ##__VA_ARGS__)

/**
 * Print log with ERROR log level
 *
 * @param[in] fmt Format (printf)
 * @param[in] ... Parameters
 */
#define log_error(fmt, ...) \
    log_print(LOG_ERROR, fmt, ##__VA_ARGS__)

/**
 * Print log with FATAL log level
 *
 * @param[in] fmt Format (printf)
 * @param[in] ... Parameters
 */
#define log_fatal(fmt, ...) \
    log_print(LOG_FATAL, fmt, ##__VA_ARGS__)

/* Macros =================================================================== */
/* Enums ==================================================================== */
/**
 * Log levels
 */
typedef enum {
  LOG_DEBUG = 0,
  LOG_INFO,
  LOG_WARNING,
  LOG_ERROR,
  LOG_FATAL
} log_level_t;

/* Types ==================================================================== */
/* Variables ================================================================ */
/* Shared functions ========================================================= */
/**
 * Convert string to log level enum value
 *
 * @param[in] str Log level string
 */
log_level_t log_level_from_str(const char * str);

/**
 * Convert log level to ansi color string
 *
 * @param level Log level
 * @return ANSI color
 */
const char * log_get_level_color(log_level_t level);

/**
 * Convert log level enum to string representation
 *
 * @param level Log level
 * @return Log level string
 */
const char * log_get_level_string(log_level_t level);

/**
 * Print log (variadic)
 *
 * @note Has WEAK attribute, so can be redefined
 *
 * @param[in] level Log level
 * @param[in] fmt Format (printf)
 * @param[in] args Variadic parameters
 */
void vlog_fmt(const char * file, int line, log_level_t level, const char * tag, const char * fmt, va_list args);

/**
 * Print log
 *
 * @param[in] file  File, where log_* macro was invoked
 * @param[in] line  Line in file, where log_* macro was invoked
 * @param[in] level Log level
 * @param[in] tag   Log module tag, NULL if not present
 * @param[in] fmt   Format (printf)
 * @param[in] ...   Parameters
 */
void log_fmt(const char * file, int line, log_level_t level, const char * tag, const char * fmt, ...);

/**
 * Print string without log formatting (calls log_print_port)
 *
 * @param[in] fmt Format (printf)
 * @param[in] ... Parameters
 */
void log_printf(const char * fmt, ...);

/**
 * Writes formatted buffer to `out` (@ref log_init)
 *
 * @param buffer Buffer to write
 * @param size Buffer size
 */
void log_write_buffer(const uint8_t * buffer, size_t size);

/**
 * Set minimum log level to allow messages to be printed with
 *
 * @param level Log Level
 */
void log_set_level(log_level_t level);

/**
 * Set minimum log level to allow messages to be printed with
 *
 * @param level Log Level
 */
void log_set_level_from_str(const char * level);

#ifdef __cplusplus
}
#endif