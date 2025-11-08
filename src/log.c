/** ========================================================================= *
 *
 * @file log.c
 * @date 23-07-2024
 * @author Maksym Tkachuk <max.r.tkachuk@gmail.com>
 *
 *  ========================================================================= */

/* Includes ================================================================= */
#include <string.h>
#include <ansi.h>
#include <stdio.h>
#include "assertion.h"
#include "util.h"
#include "log.h"

/* Defines ================================================================== */
#define LINE_ENDING "\n"

/* Macros =================================================================== */
/* Exposed macros =========================================================== */
/* Enums ==================================================================== */
/* Types ==================================================================== */
/* Variables ================================================================ */
static log_level_t min_log_level = LOG_WARNING;

/* Private functions ======================================================== */
static void log_write(const uint8_t * buffer, size_t size) {
  fwrite(buffer, 1, size, LOG_OUTPUT_FILE);
}

/* Shared functions ========================================================= */
log_level_t log_level_from_str(const char * str) {
  if (!strcmp(str, "debug")) return LOG_DEBUG;
  if (!strcmp(str, "info"))  return LOG_INFO;
  if (!strcmp(str, "warn"))  return LOG_WARNING;
  if (!strcmp(str, "error")) return LOG_ERROR;
  if (!strcmp(str, "fatal")) return LOG_FATAL;

  return LOG_DEBUG;
}

const char * log_get_level_color(log_level_t level) {
#if USE_COLOR_LOG
  switch (level) {
    case LOG_DEBUG:   return ANSI_COLOR_FG_CYAN;
    case LOG_INFO:    return ANSI_COLOR_FG_BLUE;
    case LOG_WARNING: return ANSI_COLOR_FG_YELLOW;
    case LOG_ERROR:   return ANSI_COLOR_FG_RED;
    case LOG_FATAL:   return ANSI_COLOR_BG_RED;
    default:
      return "";
  }
#else
  return "";
#endif
}

const char * log_get_level_string(log_level_t level) {
  switch (level) {
    case LOG_DEBUG:   return "debug";
    case LOG_INFO:    return "info ";
    case LOG_WARNING: return "warn ";
    case LOG_ERROR:   return "error";
    case LOG_FATAL:   return "fatal";
    default:
      return "";
  }
}

__WEAK void vlog_fmt(
  __UNUSED const char * file,
  __UNUSED int line,
  log_level_t level,
  const char * tag,
  const char * fmt,
  va_list args
) {
  ASSERT_RETURN(level >= min_log_level);

  char buf[LOG_LINE_SIZE] = {'\r'};
  size_t size = 1;

  if (tag) {
    size += snprintf(buf + size, sizeof(buf) - size - 1, "[%s%s%s][%s%s%s] ",
                     log_get_level_color(level),
                     log_get_level_string(level),
                     USE_COLOR_LOG ? ANSI_TEXT_RESET : "",
                     USE_COLOR_LOG ? ANSI_COLOR_FG_MAGENTA : "",
                     tag,
                     USE_COLOR_LOG ? ANSI_TEXT_RESET : "");
  } else {
    size += snprintf(buf + size, sizeof(buf) - size - 1, "[%s%s%s] ",
                     log_get_level_color(level),
                     log_get_level_string(level),
                     USE_COLOR_LOG ? ANSI_TEXT_RESET : "");
  }

  size += vsnprintf(buf + size, sizeof(buf) - size - 1, fmt, args);
  size += snprintf(buf + size, sizeof(buf) - size - 1, LINE_ENDING);

  buf[size] = 0;

  log_write_buffer((const uint8_t *) buf, size);
}

void log_fmt(const char * file, int line, log_level_t level, const char * tag, const char * fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vlog_fmt(file, line, level, tag, fmt, args);
  va_end(args);
}

void log_printf(const char * fmt, ...) {
  char buf[LOG_LINE_SIZE];

  va_list args;
  va_start(args, fmt);
  size_t size = vsnprintf(buf, sizeof(buf) - 1, fmt, args);
  va_end(args);

  buf[size] = 0;

  log_write(buf, size);
}

void log_write_buffer(const uint8_t * buffer, size_t size) {
  log_write(buffer, size);
}

void log_set_level(log_level_t level) {
  min_log_level = level;
}

void log_set_level_from_str(const char * level) {
  min_log_level = log_level_from_str(level);
}

