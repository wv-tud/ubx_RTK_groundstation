/*
 * Copyright (C) 2016 Wilco Vlenterie, Anand Sundaresan.
 * Contact: Anand Sundaresan <nomail@donotmailme.com>
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBRTCM3_RTCM3_H
#define LIBRTCM3_RTCM3_H

#define RTCM3_PREAMBLE 0xD3
#define RTCM3_MSG_1005 0x69
#define RTCM3_MSG_1077 0xB1
#define RTCM3_MSG_1087 0xBB

#include <errno.h>
#include "common.h"

/** Return value indicating success. */
#define RTCM3_OK              0
/** Return value indicating message decoded and callback executed by rtcm3_process. */
#define RTCM3_OK_CALLBACK_EXECUTED 1
/** Return value indicating message decoded with no associated callback in rtcm3_process. */
#define RTCM3_OK_CALLBACK_UNDEFINED 2
/** Return value indicating an error with the callback (function defined). */
#define RTCM3_CALLBACK_ERROR -1
/** Return value indicating a CRC error. */
#define RTCM3_CRC_ERROR      -2
/** Return value indicating an error occured whilst sending an RTCM3 message. */
#define RTCM3_SEND_ERROR     -3
/** Return value indicating an error occured because an argument was NULL. */
#define RTCM3_NULL_ERROR     -4
/** Default sender ID. Intended for messages sent from the host to the device. */
#define RTCM3_SENDER_ID 0x42
/** RTCM3 callback function prototype definition. */
typedef void (*rtcm3_msg_callback_t)(u16 sender_id, u8 len, u8 msg[], void *context);

/** RTCM3 callback node.
 * Forms a linked list of callbacks.
 * \note Must be statically allocated for use with rtcm3_register_callback().
 */
typedef struct rtcm3_msg_callbacks_node {
  u16 msg_type;                        /**< Message ID associated with callback. */
  rtcm3_msg_callback_t cb;               /**< Pointer to callback function. */
  void *context;                       /**< Pointer to a context */
  struct rtcm3_msg_callbacks_node *next; /**< Pointer to next node in list. */
} rtcm3_msg_callbacks_node_t;

/** State structure for processing RTCM3 messages. */
typedef struct {
  enum {
	READ_PREAMBLE = 0,
	READ_RESERVED,
	READ_LENGTH,
	READ_MESSAGE,
	READ_CHECKSUM
  } state;
  u16 msg_type;
  u16 sender_id;
  u16 crc;
  u8 msg_len;
  u8 n_read;
  u8 msg_buff[1024+6+1];
  void* io_context;
  rtcm3_msg_callbacks_node_t* rtcm3_msg_callbacks_head;
} rtcm3_state_t;

s8                          rtcm3_register_callback(rtcm3_state_t* s, u16 msg_type, rtcm3_msg_callback_t cb, void* context, rtcm3_msg_callbacks_node_t *node);
void                        rtcm3_clear_callbacks(rtcm3_state_t* s);
rtcm3_msg_callbacks_node_t* rtcm3_find_callback(rtcm3_state_t* s, u16 msg_type);
void                        rtcm3_state_init(rtcm3_state_t *s);
void                        rtcm3_state_set_io_context(rtcm3_state_t *s, void* context);
s8                          rtcm3_process(rtcm3_state_t *s, u32 (*read)(unsigned char (*buff)[], u32 n, void* context));
s8                          rtcm3_send_message(rtcm3_state_t *s, u16 msg_type, u16 sender_id, u8 len, u8 *payload, u32 (*write)(u8 *buff, u32 n, void* context));
unsigned int                RTCMgetbitu(unsigned char *, int, int);
int                         RTCMgetbits(unsigned char *, int , int );
static double               RTCMgetbits_38(unsigned char *, int );

int rd_msg_len      = 0;
int rd_msg_len1     = 0;
int byteIndex       = 0;
int checksumCounter = 0;
int rawIndex        = 0;

/** Register a callback for a message type.
 * Register a callback that is called when a message
 * with type msg_type is received.
 *
 * \param msg_type Message type associated with callback
 * \param cb       Pointer to message callback function
 * \param context  Pointer to context for callback function
 * \param node     Statically allocated #rtcm3_msg_callbacks_node_t struct
 * \return `RTCM3_OK` (0) if successful, `RTCM3_CALLBACK_ERROR` if callback was
 *         already registered for that message type.
 */
s8 rtcm3_register_callback(rtcm3_state_t *s, u16 msg_type, rtcm3_msg_callback_t cb, void *context,
                         rtcm3_msg_callbacks_node_t *node)
{
  /* Check our callback function pointer isn't NULL. */
  if (cb == 0)
    return RTCM3_NULL_ERROR;

  /* Check our callback node pointer isn't NULL. */
  if (node == 0)
    return RTCM3_NULL_ERROR;

  /* Check if callback was already registered for this type. */
  if (rtcm3_find_callback(s, msg_type) != 0)
    return RTCM3_CALLBACK_ERROR;

  /* Fill in our new rtcm3_msg_callback_node_t. */
  node->msg_type = msg_type;
  node->cb = cb;
  node->context = context;
  /* The next pointer is set to NULL, i.e. this
   * will be the new end of the linked list.
   */
  node->next = 0;

  /* If our linked list is empty then just
   * add the new node to the start.
   */
  if (s->rtcm3_msg_callbacks_head == 0) {
    s->rtcm3_msg_callbacks_head = node;
    return RTCM3_OK;
  }

  /* Find the tail of our linked list and
   * add our new node to the end.
   */
  rtcm3_msg_callbacks_node_t *p = s->rtcm3_msg_callbacks_head;
  while (p->next)
    p = p->next;

  p->next = node;

  return RTCM3_OK;
}

/** Clear all registered callbacks.
 * This is probably only useful for testing but who knows!
 */
void rtcm3_clear_callbacks(rtcm3_state_t *s)
{
  /* Reset the head of the callbacks list to NULL. */
  s->rtcm3_msg_callbacks_head = 0;
}

/** Find the callback function associated with a message type.
 * Searches through the list of registered callbacks to find the callback
 * associated with the passed message type.
 *
 * \param msg_type Message type to find callback for
 * \return Pointer to callback node (#rtcm3_msg_callbacks_node_t) or `NULL` if
 *         callback not found for that message type.
 */
rtcm3_msg_callbacks_node_t* rtcm3_find_callback(rtcm3_state_t *s, u16 msg_type)
{
  /* If our list is empty, return NULL. */
  if (!s->rtcm3_msg_callbacks_head)
    return 0;

  /* Traverse the linked list and return the callback
   * function pointer if we find a node with a matching
   * message id.
   */
  rtcm3_msg_callbacks_node_t *p = s->rtcm3_msg_callbacks_head;
  do
    if (p->msg_type == msg_type)
      return p;

  while ((p = p->next));

  /* Didn't find a matching callback, return NULL. */
  return 0;
}

/** Initialize an #rtcm3_state_t struct before use.
 * This resets the entire state, including all callbacks.
 * Remember to use this function to initialize the state before calling
 * rtcm3_process() for the first time.
 *
 * \param s State structure
 */
void rtcm3_state_init(rtcm3_state_t *s)
{
  s->state = READ_PREAMBLE;

  /* Set the IO context pointer, passed to read and write functions, to NULL. */
  s->io_context = 0;

  /* Clear the callbacks, if any, currently in s */
  rtcm3_clear_callbacks(s);
}

/** Set a context to pass to all function pointer calls made by rtcm3 functions
 * This helper function sets a void* context pointer in rtcm3_state.
 * Whenever `rtcm3_process` calls the `read` function pointer, it passes this context.
 * Whenever `rtcm3_send_message` calls the `write` function pointer, it passes this context.
 * This allows C++ code to get a pointer to an object inside these functions.
 */
void rtcm3_state_set_io_context(rtcm3_state_t *s, void *context)
{
  s->io_context = context;
}

/** Read and process RTCM3 messages.
 * Reads bytes from an input source using the provided `read` function, decodes
 * the RTCM3.
 *
 * When an RTCM3 message is successfully received then the list of callbacks is
 * searched for a callback corresponding to the received message type. If a
 * callback is found then it is called with the ID of the sender, the message
 * length and the message payload data buffer as arguments.
 *
 * \note rtcm3_process will always call `read` with n > 0
 *       (aka it will attempt to always read something)
 *
 * The supplied `read` function must have the prototype:
 *
 * ~~~
 * u32 read(u8 *buff, u32 n, void* context)
 * ~~~
 *
 * where `n` is the number of bytes requested and `buff` is the buffer into
 * which to write the received data, and `context` is the arbitrary pointer
 * set by `rtcm3_state_set_io_context`.
 * The function should return the number of
 * bytes successfully written into `buff` which may be between 0 and `n`
 * inclusive, but must never be greater than `n`.
 *
 * Note that `rtcm3_process` may not read all available bytes from the `read`
 * function so the caller should loop until all bytes available from the input
 * source have been consumed.
 *
 * \param s State structure
 * \param read Function pointer to a function that reads `n` bytes from the
 *             input source into `buff` and returns the number of bytes
 *             successfully read.
 * \return `RTCM3_OK` (0) if successful but no complete message yet,
 *         `RTCM3_OK_CALLBACK_EXECUTED` (1) if message decoded and callback executed,
 *         `RTCM3_OK_CALLBACK_UNDEFINED` (2) if message decoded with no associated
 *         callback, and `RTCM3_CRC_ERROR` (-2) if a CRC error
 *         has occurred. Thus can check for >0 to ensure good processing.
 */
s8 rtcm3_process(rtcm3_state_t *s, u32 (*read)(unsigned char (*buff)[], u32 n, void *context))
{
	unsigned char buff[1000]; // We are only requesting 1 byte, but have room for 5
/*	FAKE MESSAGE
	buff[s->n_read] = s->n_read;
	s->n_read++;
	int fakeMsgLen = 200;
	if(s->n_read == fakeMsgLen)
	{
		s->n_read = 0;
		rtcm3_msg_callbacks_node_t* node = rtcm3_find_callback(s, RTCM3_MSG_1077);
		(*node->cb)(s->sender_id, fakeMsgLen, buff, node->context);
		return RTCM3_OK_CALLBACK_EXECUTED;
	}else{
		return RTCM3_OK;
	}
*/
	int rdlen;
	rdlen = (*read)(&buff, 1, s->io_context);
	if(s->n_read == (1024 + 6) && s->state != READ_PREAMBLE)
	{
		// We have exceeded the maximum message length (10bit) + 3 opening and 3 closing bytes. And we are not at read_preamble, this is not a proper message, reset!
		s->state = READ_PREAMBLE;
	}
	if (rdlen < 0) {
		printf("Error from read: %d: %s\n", rdlen, strerror(errno));
		return RTCM3_CRC_ERROR;
	}else{
		// Suppose we get more bytes than requested, lets still process them all
		int byteN;
		for(byteN=0; byteN < rdlen; byteN++)
		{
#ifdef DEBUG_PRINT_PACKAGE
			printf("0x%x ", buff[s->n_read]);
#endif
			if(s->state != READ_PREAMBLE) s->msg_buff[s->n_read] = buff[byteN];
			switch (s->state){
			case READ_PREAMBLE:
				s->n_read = 0;
				if (((int) buff[byteN]) == RTCM3_PREAMBLE){
					s->state = READ_RESERVED;
					rawIndex        = 0;
					checksumCounter = 0;
					byteIndex       = 0;
					s->msg_buff[s->n_read] = buff[byteN];
				}
				break;
			case READ_RESERVED:
				rd_msg_len1 = ((int) buff[byteN]) & 0b00000011;
				s->state    = READ_LENGTH;
				break;
			case READ_LENGTH:
				rd_msg_len  = (rd_msg_len1 << 8) + ((int) buff[byteN]) ;
				s->state    = READ_MESSAGE;
				break;
			case READ_MESSAGE:
				if (byteIndex == (rd_msg_len - 1)) s->state = READ_CHECKSUM;
				byteIndex++;
				break;
			case READ_CHECKSUM:
				checksumCounter++;
				if(checksumCounter == 3)
				{
#ifdef DEBUG_PRINT_PACKAGE
			        printf("\n\n");
#endif
					s->state = READ_PREAMBLE;
					// Check what message type it is
					switch(RTCMgetbitu(s->msg_buff, 24 + 0, 12))
					{
					case 1005: s->msg_type = RTCM3_MSG_1005; break;
					case 1077: s->msg_type = RTCM3_MSG_1077; break;
					case 1087: s->msg_type = RTCM3_MSG_1087; break;
					default  : printf("Unknown message type\n"); return RTCM3_OK_CALLBACK_UNDEFINED;
					}
					s->n_read++;
					s->sender_id = RTCM3_SENDER_ID;
					s->msg_len   = s->n_read;
#ifdef NO_CALLBACK
					return RTCM3_OK_CALLBACK_EXECUTED;
#else
					/* Message complete, process its callback. */
					rtcm3_msg_callbacks_node_t* node = rtcm3_find_callback(s, s->msg_type);
					if (node) {
						(*node->cb)(s->sender_id, s->msg_len, s->msg_buff, node->context);
						return RTCM3_OK_CALLBACK_EXECUTED;
					} else {
						return RTCM3_OK_CALLBACK_UNDEFINED;
					}
#endif
				}
				break;
			}
			s->n_read++;
		}
		return RTCM3_OK;
	}
}

/** Send RTCM3 messages.
 * Takes an RTCM3 message payload, type and sender ID then writes a message to
 * the output stream using the supplied `write` function with the correct
 * framing and CRC.
 *
 * The supplied `write` function must have the prototype:
 *
 * ~~~
 * u32 write(u8 *buff, u32 n, void* context)
 * ~~~
 *
 * where `n` is the number of bytes to be written and `buff` is the buffer from
 * which to read the data to be written, and `context` is the arbitrary pointer
 * set by `rtcm3_state_set_io_context`. The function should return the number
 * of bytes successfully written which may be between 0 and `n`. Currently, if
 * the number of bytes written is different from `n` then `rtcm3_send_message`
 * will immediately return with an error.
 *
 * Note that `rtcm3_send_message` makes multiple calls to write and therefore if
 * a `write` call fails then this may result in a partial message being written
 * to the output. This should be caught by the CRC check on the receiving end
 * but will result in lost messages.
 *
 * \param write Function pointer to a function that writes `n` bytes from
 *              `buff` to the output stream  and returns the number of bytes
 *              successfully written.
 * \return `RTCM3_OK` (0) if successful, `RTCM3_WRITE_ERROR` if the message could
 *         not be sent or was only partially sent.
 */
s8 rtcm3_send_message(rtcm3_state_t *s, u16 msg_type, u16 sender_id, u8 len, u8 *payload,
                    u32 (*write)(u8 *buff, u32 n, void *context))
{
  /* Check our payload data pointer isn't NULL unless len = 0. */
  if (len != 0 && payload == 0)
    return RTCM3_NULL_ERROR;

  /* Check our write function pointer isn't NULL. */
  if (write == 0)
    return RTCM3_NULL_ERROR;

  u8 preamble = RTCM3_PREAMBLE;
  if ((*write)(&preamble, 1, s->io_context) != 1)
    return RTCM3_SEND_ERROR;

  if ((*write)((u8*)&msg_type, 2, s->io_context) != 2)
    return RTCM3_SEND_ERROR;

  if ((*write)((u8*)&sender_id, 2, s->io_context) != 2)
    return RTCM3_SEND_ERROR;

  if ((*write)(&len, 1, s->io_context) != 1)
    return RTCM3_SEND_ERROR;

  if (len > 0) {
    if ((*write)(payload, len, s->io_context) != len)
      return RTCM3_SEND_ERROR;
  }
  return RTCM3_OK;
}

unsigned int RTCMgetbitu(unsigned char *buff, int pos, int lenb)
{
	unsigned int bits=0;
	int i;
	for (i=pos;i<pos+lenb;i++) bits=(bits<<1)+((buff[i/8]>>(7-i%8))&1u);
	return bits;
}

int RTCMgetbits(unsigned char *buff, int pos, int lenb)
{
	unsigned int bits=RTCMgetbitu(buff,pos,lenb);
	if (lenb<=0||32<=lenb||!(bits&(1u<<(lenb-1)))) return (int)bits;
	return (int)(bits|(~0u<<lenb)); /* extend sign */
}

static double RTCMgetbits_38(unsigned char *buff, int pos)
{
	return (double)RTCMgetbits(buff,pos,32)*64.0+RTCMgetbitu(buff,pos+32,6);
}

#endif /* LIBRTCM3_RTCM3_H */
