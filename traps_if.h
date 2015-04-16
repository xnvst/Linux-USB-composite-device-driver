/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief User land interface to USB command/status traps
 *
 * $Id: traps_if.h,v 1.1.14.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file defines structures used by the USB command/status traps. This is
 * the file that should be included by user land applications to write to the
 * command/status trap device. It is shared by both user land and kernel land.
 */
#ifndef TRAPS_IF_H
#define TRAPS_IF_H

/** @brief File data header */
struct trap_file_hdr {
   uint16_t var_id;  /**< Variable ID */
   uint16_t var_idx; /**< Variable index */
};

#endif
