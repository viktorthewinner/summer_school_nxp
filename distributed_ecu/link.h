/*
 * link.h - UART transport to the Arduino nodes.
 *
 * Two independent point-to-point channels, one per Arduino:
 *
 *   LINK_ARD1   LPUART2   P3_15 TX / P3_14 RX   J2 pin 2  / J2 pin 4
 *   LINK_ARD2   LPUART1   P1_9  TX / P1_8  RX   J2 pin 20 / J2 pin 18
 *   LINK_GW     LPI2C0    P3_27 SCL / P3_28 SDA  mikroBUS J6  (ESP32)
 *
 * The two UART channels receive under interrupt into a ring. The gateway is
 * an I2C slave and cannot initiate, so LINK_PollLine(LINK_GW, ...) performs a
 * bus read: the ESP32 answers with a length byte followed by the payload, or
 * a zero length when it has nothing pending.
 *
 * UART is point-to-point, so three boards means a star with the MCX at the
 * centre - not a shared pair. Each channel has its own interrupt-driven
 * receive ring, so the two Arduinos can talk at the same time without
 * colliding.
 *
 * Frames are newline-terminated ASCII lines. The application never touches
 * LPUART registers; when this project moves to CAN, only link.c changes.
 */
#ifndef _LINK_H_
#define _LINK_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Longest line accepted or produced, including the terminator. */
#define LINK_LINE_MAX 128u

typedef enum
{
    LINK_ARD1 = 0, /* LPUART2, interrupt-driven          */
    LINK_ARD2 = 1, /* LPUART1, interrupt-driven          */
    LINK_GW   = 2, /* LPI2C0 master, polled - see below  */
    LINK_COUNT
} link_id_t;

/*!
 * @brief Bring up both channels at LINK_BAUDRATE, 8N1, RX interrupt enabled.
 *
 * Pin muxing and clock attach happen in BOARD_InitHardware() beforehand.
 */
void LINK_Init(void);

/*! @brief Send one line to one node. A '\n' terminator is appended. */
void LINK_SendLine(link_id_t id, const char *line);

/*! @brief Send one line to every node except @p exclude.
 *         Pass LINK_COUNT to reach all of them. */
void LINK_Broadcast(const char *line, link_id_t exclude);

/*!
 * @brief Non-blocking line receive from one node.
 *
 * @retval true  a complete line was copied into @p out, terminator stripped
 * @retval false nothing complete available yet
 */
bool LINK_PollLine(link_id_t id, char *out, size_t outSize);

/*! @brief Bytes lost on this channel to hardware overrun, ring overflow or
 *         an overlong line. Non-zero means it is being pushed too hard. */
uint32_t LINK_GetDroppedCount(link_id_t id);

/*! @brief Short printable name of a channel, e.g. "ARD1". */
const char *LINK_GetName(link_id_t id);

#endif /* _LINK_H_ */
