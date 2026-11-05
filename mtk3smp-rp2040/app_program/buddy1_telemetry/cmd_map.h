/**
 * @file cmd_map.h
 * @brief Buddy 1: barcode character -> movement mapping, received from
 *        the PC over MQTT before the demo (not hard-coded).
 *
 * The mapping names four Code 39 characters, one per movement, in this
 * order: forward, U-turn, left, right. On MQTT it is 3 bytes (24 bits),
 * least-significant bit first like the telemetry packet: four 6-bit
 * indexes into the 43 Code 39 characters CMD_MAP_CHARSET.
 *   bits  0-5  forward     bits 12-17  left
 *   bits  6-11 U-turn      bits 18-23  right
 * The four characters must differ; a payload that breaks a rule is
 * rejected and the previous mapping stays.
 *
 * A code means the same movement whichever way it is read. Until a
 * mapping arrives every character maps to no movement.
 *
 * The mapping is one 32-bit word (a character per byte), written by the
 * radio task and read by the controller: a single aligned load or store,
 * so a reader never sees half of an update.
 */

#ifndef BUDDY1_CMD_MAP_H
#define BUDDY1_CMD_MAP_H

#include <stdint.h>
#include <stdbool.h>

#define CMD_MAP_LEN          3u      /**< payload bytes on MQTT */
#define CMD_MAP_CHARS        4u      /**< forward, U-turn, left, right */
#define CMD_MAP_CHARSET      "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%"
#define CMD_MAP_CHARSET_LEN  43u

/** @brief Apply a 3-byte MQTT payload; false (mapping unchanged) if it is
 * not a valid mapping. */
bool cmd_map_set_packed(const uint8_t p_in[CMD_MAP_LEN]);

/** @brief Apply four characters (forward, U-turn, left, right), e.g. from
 * the serial console; false (unchanged) if not valid. */
bool cmd_map_set_chars(const char p_chars[CMD_MAP_CHARS]);

/** @brief The current mapping as characters; false if none yet. */
bool cmd_map_get(char p_chars[CMD_MAP_CHARS]);

/** @brief The current mapping packed as on MQTT; false if none yet. */
bool cmd_map_get_packed(uint8_t p_out[CMD_MAP_LEN]);

/** @brief The movement (CTRL_CMD_*) for a decoded character, or
 * CTRL_CMD_NONE if it is not one of the four. */
uint8_t cmd_map_lookup(uint8_t ch);

#endif /* BUDDY1_CMD_MAP_H */
