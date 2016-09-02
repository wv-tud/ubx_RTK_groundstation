/*
 * Copyright (C) 2011-2014 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "rtcm3.h"

#define SBP_PREAMBLE 0xD3

/** \addtogroup io Input / Output
 * \{ */

/** \defgroup sbp SBP
 * Send and receive messages using Swift Binary Protocol.
 *
 * Examples
 * ========
 *
 * Receiving
 * ---------
 *
 * First setup a callback for the message you will be receiving. Our callback
 * function must have type #sbp_msg_callback_t, i.e. it must be of the form:
 *
 * ~~~
 * void my_callback(u16 sender_id, u8 len, u8 msg[], void *context)
 * {
 *   // Process msg.
 * }
 * ~~~
 *
 * You must also statically allocate a #sbp_msg_callbacks_node_t that will be
 * used to keep track of the callback function. You do not need to initialize
 * it as this will be done by sbp_register_callback().
 *
 * ~~~
 * static sbp_msg_callbacks_node_t my_callback_node;
 * ~~~
 *
 * Now register your callback function with the SBP library as follows:
 *
 * ~~~
 * sbp_register_callback(&sbp_state, SBP_MY_MSG_TYPE, &my_callback, &context, &my_callback_node);
 * ~~~
 *
 * where `SBP_MY_MSG_TYPE` is the numerical identifier of your message type.
 *
 * You must now call sbp_process() periodically whenever you have received SBP
 * data to be processed, e.g. from the serial port. Remember sbp_process() may
 * not use all available data so keep calling sbp_process() until all the
 * received serial data has been consumed.
 *
 * sbp_process() stores its internal state in an #sbp_state_t struct which must
 * be initialized by calling sbp_state_init() before its first use.
 *
 * Here is an example based on reading from a typical UART interface:
 *
 * ~~~
 * u32 my_read(u8 *buff, u32 n, void *context)
 * {
 *   for (u32 i=0; i<n; i++) {
 *     if (uart_has_data())
 *       buff[i] = uart_read_char();
 *     else
 *       break;
 *   }
 *   return i;
 * }
 *
 * int main()
 * {
 *   ...
 *
 *   sbp_state_t s;
 *   sbp_state_init(&s);
 *
 *   while(uart_has_data()) {
 *     sbp_process(&s, &my_read);
 *   }
 *
 *   ...
 * }
 * ~~~
 *
 * If you're writing C++ code that wants to reference a pointer to
 * an object in the my_read function, you can use the context set
 * by calling sbp_state_set_io_context()
 *
 *
 * Sending
 * -------
 *
 * To send an SBP message simply call the sbp_send_message() function,
 * providing a `write` function that writes data to your output.
 *
 * Often the data to be sent will simply be a struct cast to a `u8` buffer. As
 * a convenience you may want to define a macro that automatically includes
 * your write function and calculates the size of the item to be sent.
 *
 * ~~~
 * // Convenience macro for sending an SBP message.
 * #define SBP_MSG(sbp_state, msg_type, item) \
 *   sbp_send_message(&sbp_state, msg_type, MY_SENDER_ID, \
 *       sizeof(item), (u8 *)&(item), &my_write)
 *
 * typedef struct {
 *   u8 x, y;
 * } my_awesome_struct;
 *
 * u32 my_write(u8 *buff, u32 n, void *context)
 * {
 *   for (u32 i=0; i<n; i++) {
 *     if (uart_write_char(buff[i]) == ERROR)
 *       break;
 *   }
 *   return i;
 * }
 *
 * int main()
 * {
 *   ...
 *
 *   sbp_state_t s;
 *   sbp_state_init(&s);
 *
 *   my_awesome_struct payload = { 0x22, 0x33 };
 *
 *   sbp_send_message(&s, SBP_MY_MSG_TYPE, MY_SENDER_ID,
 *                    sizeof(payload), (u8*)&payload, &my_write);
 *
 *   // or
 *
 *   SBP_MSG(s, SBP_MY_MSG_TYPE, payload);
 *
 *   ...
 * }
 * ~~~
 *
 *
 * \{ */

/** Register a callback for a message type.
 * Register a callback that is called when a message
 * with type msg_type is received.
 *
 * \param msg_type Message type associated with callback
 * \param cb       Pointer to message callback function
 * \param context  Pointer to context for callback function
 * \param node     Statically allocated #sbp_msg_callbacks_node_t struct
 * \return `SBP_OK` (0) if successful, `SBP_CALLBACK_ERROR` if callback was
 *         already registered for that message type.
 */
s8 sbp_register_callback(sbp_state_t *s, u16 msg_type, sbp_msg_callback_t cb, void *context,
                         sbp_msg_callbacks_node_t *node)
{
  /* Check our callback function pointer isn't NULL. */
  if (cb == 0)
    return SBP_NULL_ERROR;

  /* Check our callback node pointer isn't NULL. */
  if (node == 0)
    return SBP_NULL_ERROR;

  /* Check if callback was already registered for this type. */
  if (sbp_find_callback(s, msg_type) != 0)
    return SBP_CALLBACK_ERROR;

  /* Fill in our new sbp_msg_callback_node_t. */
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
  if (s->sbp_msg_callbacks_head == 0) {
    s->sbp_msg_callbacks_head = node;
    return SBP_OK;
  }

  /* Find the tail of our linked list and
   * add our new node to the end.
   */
  sbp_msg_callbacks_node_t *p = s->sbp_msg_callbacks_head;
  while (p->next)
    p = p->next;

  p->next = node;

  return SBP_OK;
}

/** Clear all registered callbacks.
 * This is probably only useful for testing but who knows!
 */
void sbp_clear_callbacks(sbp_state_t *s)
{
  /* Reset the head of the callbacks list to NULL. */
  s->sbp_msg_callbacks_head = 0;
}

/** Find the callback function associated with a message type.
 * Searches through the list of registered callbacks to find the callback
 * associated with the passed message type.
 *
 * \param msg_type Message type to find callback for
 * \return Pointer to callback node (#sbp_msg_callbacks_node_t) or `NULL` if
 *         callback not found for that message type.
 */
sbp_msg_callbacks_node_t* sbp_find_callback(sbp_state_t *s, u16 msg_type)
{
  /* If our list is empty, return NULL. */
  if (!s->sbp_msg_callbacks_head)
    return 0;

  /* Traverse the linked list and return the callback
   * function pointer if we find a node with a matching
   * message id.
   */
  sbp_msg_callbacks_node_t *p = s->sbp_msg_callbacks_head;
  do
    if (p->msg_type == msg_type)
      return p;

  while ((p = p->next));

  /* Didn't find a matching callback, return NULL. */
  return 0;
}

/** Initialize an #sbp_state_t struct before use.
 * This resets the entire state, including all callbacks.
 * Remember to use this function to initialize the state before calling
 * sbp_process() for the first time.
 *
 * \param s State structure
 */
void sbp_state_init(sbp_state_t *s)
{
  s->state = READ_PREAMBLE;

  /* Set the IO context pointer, passed to read and write functions, to NULL. */
  s->io_context = 0;

  /* Clear the callbacks, if any, currently in s */
  sbp_clear_callbacks(s);
}


/** Set a context to pass to all function pointer calls made by sbp functions
 * This helper function sets a void* context pointer in sbp_state.
 * Whenever `sbp_process` calls the `read` function pointer, it passes this context.
 * Whenever `sbp_send_message` calls the `write` function pointer, it passes this context.
 * This allows C++ code to get a pointer to an object inside these functions.
 */
void sbp_state_set_io_context(sbp_state_t *s, void *context)
{
  s->io_context = context;
}

/** Read and process SBP messages.
 * Reads bytes from an input source using the provided `read` function, decodes
 * the SBP framing and performs a CRC check on the message.
 *
 * When an SBP message is successfully received then the list of callbacks is
 * searched for a callback corresponding to the received message type. If a
 * callback is found then it is called with the ID of the sender, the message
 * length and the message payload data buffer as arguments.
 *
 * \note sbp_process will always call `read` with n > 0
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
 * set by `sbp_state_set_io_context`.
 * The function should return the number of
 * bytes successfully written into `buff` which may be between 0 and `n`
 * inclusive, but must never be greater than `n`.
 *
 * Note that `sbp_process` may not read all available bytes from the `read`
 * function so the caller should loop until all bytes available from the input
 * source have been consumed.
 *
 * \param s State structure
 * \param read Function pointer to a function that reads `n` bytes from the
 *             input source into `buff` and returns the number of bytes
 *             successfully read.
 * \return `SBP_OK` (0) if successful but no complete message yet,
 *         `SBP_OK_CALLBACK_EXECUTED` (1) if message decoded and callback executed,
 *         `SBP_OK_CALLBACK_UNDEFINED` (2) if message decoded with no associated
 *         callback, and `SBP_CRC_ERROR` (-2) if a CRC error
 *         has occurred. Thus can check for >0 to ensure good processing.
 */
s8 sbp_process(sbp_state_t *s, u32 (*read)(u8 *buff, u32 n, void *context))
{
	unsigned char buf[3];
	int rdlen;
	rdlen = (*read)(&buf, sizeof(buf) - 1, s->io_context);
	if (rdlen < 0) {
		printf("Error from read: %d: %s\n", rdlen, strerror(errno));
		return SBP_CRC_ERROR;
	}else{
		switch (s->state){
		case READ_PREAMBLE:
			if (((int) buf[0]) == SBP_PREAMBLE){ // int = 211 = 0xD3
				s->state = READ_RESERVED;
				rawIndex = 0;
				s->msg_buff[s->n_read] = buf[0];
				checksumCounter = 0;
				byteIndex = 0;
			}
			break;
		case READ_RESERVED:
			s->msg_buff[s->n_read] = buf[0];
			rd_msg_len1 = ((int) buf[0])  & 0b00000011;
			s->state = READ_LENGTH;
			break;
		case READ_LENGTH:
			s->msg_buff[s->n_read] = buf[0];
			rd_msg_len = (rd_msg_len1 << 8) + ((int) buf[0]) ;
			s->state = READ_MESSAGE;
			break;
		case READ_MESSAGE:
			s->msg_buff[s->n_read] = buf[0];
			if (byteIndex == (rd_msg_len - 1)) s->state = READ_CHECKSUM;
			byteIndex++;
			break;
		case READ_CHECKSUM:
			s->msg_buff[s->n_read] = buf[0];
			checksumCounter++;
			if(checksumCounter == 3)
			{
				s->state = READ_PREAMBLE;
				// Check what message type it is
				s->msg_type  = getbitu(s->msg_buff, 24 + 0, 12);
				s->sender_id = SBP_SENDER_ID;
				s->n_read++;
				s->msg_len   = s->n_read;
#if NO_CALLBACK
				return SBP_OK_CALLBACK_EXECUTED;
#else
				/* Message complete, process it. */
				sbp_msg_callbacks_node_t* node = sbp_find_callback(s, s->msg_type);
				if (node) {
					(*node->cb)(s->sender_id, s->msg_len, s->msg_buff, node->context);
					return SBP_OK_CALLBACK_EXECUTED;
				} else {
					return SBP_OK_CALLBACK_UNDEFINED;
				}
#endif
			}
			break;
		}
		s->n_read++;
		return SBP_OK;
	}
}

/** Send SBP messages.
 * Takes an SBP message payload, type and sender ID then writes a message to
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
 * set by `sbp_state_set_io_context`. The function should return the number
 * of bytes successfully written which may be between 0 and `n`. Currently, if
 * the number of bytes written is different from `n` then `sbp_send_message`
 * will immediately return with an error.
 *
 * Note that `sbp_send_message` makes multiple calls to write and therefore if
 * a `write` call fails then this may result in a partial message being written
 * to the output. This should be caught by the CRC check on the receiving end
 * but will result in lost messages.
 *
 * \param write Function pointer to a function that writes `n` bytes from
 *              `buff` to the output stream  and returns the number of bytes
 *              successfully written.
 * \return `SBP_OK` (0) if successful, `SBP_WRITE_ERROR` if the message could
 *         not be sent or was only partially sent.
 */
s8 sbp_send_message(sbp_state_t *s, u16 msg_type, u16 sender_id, u8 len, u8 *payload,
                    u32 (*write)(u8 *buff, u32 n, void *context))
{
  /* Check our payload data pointer isn't NULL unless len = 0. */
  if (len != 0 && payload == 0)
    return SBP_NULL_ERROR;

  /* Check our write function pointer isn't NULL. */
  if (write == 0)
    return SBP_NULL_ERROR;

  u16 crc;

  u8 preamble = SBP_PREAMBLE;
  if ((*write)(&preamble, 1, s->io_context) != 1)
    return SBP_SEND_ERROR;

  if ((*write)((u8*)&msg_type, 2, s->io_context) != 2)
    return SBP_SEND_ERROR;

  if ((*write)((u8*)&sender_id, 2, s->io_context) != 2)
    return SBP_SEND_ERROR;

  if ((*write)(&len, 1, s->io_context) != 1)
    return SBP_SEND_ERROR;

  if (len > 0) {
    if ((*write)(payload, len, s->io_context) != len)
      return SBP_SEND_ERROR;
  }
/*
  crc = crc16_ccitt((u8*)&(msg_type), 2, 0);
  crc = crc16_ccitt((u8*)&(sender_id), 2, crc);
  crc = crc16_ccitt(&(len), 1, crc);
  crc = crc16_ccitt(payload, len, crc);

  if ((*write)((u8*)&crc, 2, s->io_context) != 2)
    return SBP_SEND_ERROR;
*/
  return SBP_OK;
}

/** \} */
/** \} */
