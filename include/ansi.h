/** ========================================================================= *
*
 * @file ansi.h
 * @date 23-07-2024
 * @author Maksym Tkachuk <max.r.tkachuk@gmail.com>
 *
 * @brief ANSI Escape Sequences
 *
 *  ========================================================================= */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ================================================================= */
#include "util.h"

/* Defines ================================================================== */
#ifndef USE_ANSI_COLORS
#define USE_ANSI_COLORS 1
#endif

#define ANSI_CURSOR_HOME              "\x1b[H"

#define ANSI_TEXT_RESET               "\x1b[0m"
#define ANSI_TEXT_BOLD                "\x1b[1m"
#define ANSI_TEXT_RESET_BOLD          "\x1b[22m"
#define ANSI_TEXT_DIM                 "\x1b[2m"
#define ANSI_TEXT_RESET_DIM           "\x1b[22m"
#define ANSI_TEXT_ITALIC              "\x1b[3m"
#define ANSI_TEXT_RESET_ITALIC        "\x1b[23m"
#define ANSI_TEXT_UNDERLINE           "\x1b[4m"
#define ANSI_TEXT_RESET_UNDERLINE     "\x1b[24m"
#define ANSI_TEXT_BLINK               "\x1b[5m"
#define ANSI_TEXT_RESET_BLINK         "\x1b[25m"
#define ANSI_TEXT_INVERSE             "\x1b[7m"
#define ANSI_TEXT_RESET_INVERSE       "\x1b[27m"
#define ANSI_TEXT_HIDDEN              "\x1b[8m"
#define ANSI_TEXT_RESET_HIDDEN        "\x1b[28m"
#define ANSI_TEXT_STRIKETHROUGH       "\x1b[9m"
#define ANSI_TEXT_RESET_STRIKETHROUGH "\x1b[29m"

#define ANSI_ERASE_IN_DISPLAY                    "\x1b[J"
#define ANSI_ERASE_FROM_CURSOR_TO_SCREEN_END     "\x1b[0J"
#define ANSI_ERASE_FROM_CURSOR_TO_SCREEN_START   "\x1b[1J"
#define ANSI_ERASE_SCREEN                        "\x1b[2J"
#define ANSI_ERASE_SAVED_LINES                   "\x1b[3J"
#define ANSI_ERASE_IN_LINE                       "\x1b[K"
#define ANSI_ERASE_FROM_CURSOR_TO_LINE_END       "\x1b[0K"
#define ANSI_ERASE_FROM_CURSOR_TO_LINE_START     "\x1b[1K"
#define ANSI_ERASE_LINE                          "\x1b[2K"

#if USE_ANSI_COLORS
#define ANSI_COLOR_FG_BLACK   "\x1b[30m"
#define ANSI_COLOR_FG_RED     "\x1b[31m"
#define ANSI_COLOR_FG_GREEN   "\x1b[32m"
#define ANSI_COLOR_FG_YELLOW  "\x1b[33m"
#define ANSI_COLOR_FG_BLUE    "\x1b[34m"
#define ANSI_COLOR_FG_MAGENTA "\x1b[35m"
#define ANSI_COLOR_FG_CYAN    "\x1b[36m"
#define ANSI_COLOR_FG_WHITE   "\x1b[37m"
#define ANSI_COLOR_FG_DEFAULT "\x1b[39m"

#define ANSI_COLOR_BG_BLACK   "\x1b[40m"
#define ANSI_COLOR_BG_RED     "\x1b[41m"
#define ANSI_COLOR_BG_GREEN   "\x1b[42m"
#define ANSI_COLOR_BG_YELLOW  "\x1b[43m"
#define ANSI_COLOR_BG_BLUE    "\x1b[44m"
#define ANSI_COLOR_BG_MAGENTA "\x1b[45m"
#define ANSI_COLOR_BG_CYAN    "\x1b[46m"
#define ANSI_COLOR_BG_WHITE   "\x1b[47m"
#define ANSI_COLOR_BG_DEFAULT "\x1b[49m"
#else
#define ANSI_COLOR_FG_BLACK   ""
#define ANSI_COLOR_FG_RED     ""
#define ANSI_COLOR_FG_GREEN   ""
#define ANSI_COLOR_FG_YELLOW  ""
#define ANSI_COLOR_FG_BLUE    ""
#define ANSI_COLOR_FG_MAGENTA ""
#define ANSI_COLOR_FG_CYAN    ""
#define ANSI_COLOR_FG_WHITE   ""
#define ANSI_COLOR_FG_DEFAULT ""

#define ANSI_COLOR_BG_BLACK   ""
#define ANSI_COLOR_BG_RED     ""
#define ANSI_COLOR_BG_GREEN   ""
#define ANSI_COLOR_BG_YELLOW  ""
#define ANSI_COLOR_BG_BLUE    ""
#define ANSI_COLOR_BG_MAGENTA ""
#define ANSI_COLOR_BG_CYAN    ""
#define ANSI_COLOR_BG_WHITE   ""
#define ANSI_COLOR_BG_DEFAULT ""
#endif

#define ANSI_CURSOR_MOVE_FMT        "\x1b[%d;%dH"
#define ANSI_CURSOR_MOVE_UP_FMT     "\x1b[%dA"
#define ANSI_CURSOR_MOVE_DOWN_FMT   "\x1b[%dB"
#define ANSI_CURSOR_MOVE_RIGHT_FMT  "\x1b[%dC"
#define ANSI_CURSOR_MOVE_LEFT_FMT   "\x1b[%dD"

/* Macros =================================================================== */
/**
 * Creates static string with ANSI escape sequence that will move cursor
 * to (__line, __col)
 */
#define ANSI_CURSOR_MOVE(__line, __col) \
  "\x1b[" UTIL_STRINGIFY(__line) ";" UTIL_STRINGIFY(__col) "H"

/**
 * Creates static string with ANSI escape sequence that will move cursor UP
 * by number of lines specified in __VA_ARGS__. If __VA_ARGS__ are empty -
 * will move by 1 line
 */
#define ANSI_CURSOR_MOVE_UP(...) \
  "\x1b[" UTIL_IF_EMPTY(__VA_ARGS__, "", UTIL_STRINGIFY(__VA_ARGS__)) "A"

/**
 * Creates static string with ANSI escape sequence that will move cursor DOWN
 * by number of lines specified in __VA_ARGS__. If __VA_ARGS__ are empty -
 * will move by 1 line
 */
#define ANSI_CURSOR_MOVE_DOWN(...) \
  "\x1b[" UTIL_IF_EMPTY(__VA_ARGS__, "", UTIL_STRINGIFY(__VA_ARGS__)) "B"

/**
 * Creates static string with ANSI escape sequence that will move cursor RIGHT
 * by number of columns specified in __VA_ARGS__. If __VA_ARGS__ are empty -
 * will move by 1 column
 */
#define ANSI_CURSOR_MOVE_RIGHT(...) \
  "\x1b[" UTIL_IF_EMPTY(__VA_ARGS__, "", UTIL_STRINGIFY(__VA_ARGS__)) "C"

/**
 * Creates static string with ANSI escape sequence that will move cursor LEFT
 * by number of columns specified in __VA_ARGS__. If __VA_ARGS__ are empty -
 * will move by 1 column
 */
#define ANSI_CURSOR_MOVE_LEFT(...) \
  "\x1b[" UTIL_IF_EMPTY(__VA_ARGS__, "", UTIL_STRINGIFY(__VA_ARGS__)) "D"

/* Enums ==================================================================== */
/* Types ==================================================================== */
/* Variables ================================================================ */
/* Shared functions ========================================================= */

#ifdef __cplusplus
}
#endif