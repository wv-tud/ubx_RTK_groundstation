/*
 * 			COPYRIGHTS
 *
 * Compilation copyright is held by the GPSD project.  All rights reserved.
 *
 * GPSD project copyrights are assigned to the project lead, currently
 * Eric S. Raymond. Other portions of the GPSD code are Copyright (c)
 * 1997, 1998, 1999, 2000, 2001, 2002 by Remco Treffkorn, and others
 * Copyright (c) 2005 by Eric S. Raymond.  For other copyrights, see
 * individual files.
 */

/* Interface for CRC-24Q cyclic redundancy chercksum code
 *
 * This file is Copyright (c) 2010 by the GPSD project
 * BSD terms apply: see the file COPYING in the distribution root for details.
 */
#ifndef _CRC24Q_H_
#define _CRC24Q_H_


extern void crc24q_sign(unsigned char *data, int len);

extern bool crc24q_check(unsigned char *data, int len);

extern unsigned crc24q_hash(unsigned char *data, int len);
#endif /* _CRC24Q_H_ */
