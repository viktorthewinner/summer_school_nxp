/*
 * link.h - UART transport to the Arduino nodes.
 *
 * Two independent point-to-point channels, one per Arduino:
 *
 *   LINK_ARD1   LPUART2   P3_15 TX / P3_14 RX   J2 pin 2  / J2 pin 4
 *   LINK_ARD2   LPUART1   P1_9  TX / P1_8  RX   J2 pin 20 / J2 pin 18
 *   LINK_GW     LPI2C0    P3_27 SCL / P3_28 SDA  mikroBUS J5 pins 5 / 6  (ESP32)
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

/* Largest payload the gateway hands back in one read. MUST match GW_CHUNK in
 * esp32_gateway.ino: the slave writes exactly LINK_GW_CHUNK + 1 bytes on every
 * request, so a master that reads a different number leaves the slave's
 * transmit buffer out of step - see LINK_GwResync(). */
#define LINK_GW_CHUNK 48u

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

/*!
 * @brief Take ONE raw byte from a UART channel, bypassing line assembly.
 *
 * For debugging only. LINK_PollLine() throws away anything that never
 * terminates - a stream of garbage from a baud mismatch, or a floating pin
 * chattering - so a channel that is receiving nonsense looks exactly like one
 * receiving nothing. This is how you tell them apart.
 *
 * Bytes taken here do not reach LINK_PollLine(), so do not run both on the
 * same channel at once. Not valid for LINK_GW.
 *
 * @retval true  a byte was available and copied to @p byte
 */
bool LINK_PollByte(link_id_t id, uint8_t *byte);

/*!
 * @brief Did the most recent gateway I2C transaction succeed?
 *
 * The gateway produces no bytes both when it is idle and when it is not
 * there, so this is the only way to tell "nobody is typing in the browser"
 * from "the I2C link is dead" - which is the first thing you need to know
 * when the page shows nothing.
 */
bool LINK_GwOnline(void);

/*! @brief Gateway transaction counters. Either pointer may be NULL. */
void LINK_GwStats(uint32_t *polls, uint32_t *fails);

/*!
 * @brief Name any LPI2C status_t in one word: "NAK", "FIFO ERROR", "ok", ...
 *
 * The count of failures cannot say WHY, but the status can, and the SDK ranks
 * the error flags so the word is meaningful: a NAK outranks a FIFO error, so
 * "FIFO ERROR" means no NAK was seen - the slave DID acknowledge and the
 * transfer broke afterwards. That is a completely different fault from
 * silence, and it is the difference between suspecting a wire and suspecting
 * the far end's timing.
 */
const char *LINK_I2CStatusName(int32_t status);

/*! @brief One sentence of what to check for that status. */
const char *LINK_I2CStatusHint(int32_t status);

/*! @brief LINK_I2CStatusName() of the most recent gateway transaction. */
const char *LINK_GwFailReason(void);

/*! @brief One sentence of what to check for the current LINK_GwFailReason(). */
const char *LINK_GwFailHint(void);

/*! @brief How many times the LPI2C controller has been re-initialised to
 *         recover from a run of failed transfers. Climbing means the bus is
 *         breaking repeatedly rather than being simply absent. */
uint32_t LINK_GwRecoveries(void);

/*!
 * @brief Reset the controller and drop any half-read gateway chunk.
 *
 * The gateway answers every read with a fixed LINK_GW_CHUNK + 1 bytes. Read a
 * different number - as a one-byte bus probe does - and the bytes it queued
 * but did not get to send stay queued, so every later read is one frame behind
 * for as long as the board is powered. Anything that touches the gateway
 * outside the normal poll must call this afterwards.
 */
void LINK_GwResync(void);

/*! @brief Short printable name of a channel, e.g. "ARD1". */
const char *LINK_GetName(link_id_t id);

#endif /* _LINK_H_ */
