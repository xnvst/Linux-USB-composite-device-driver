/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief USB command/status file operations
 *
 * $Id: file_ops.h,v 1.2.14.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file defines structures used by USB command/status file operations.
 */
#ifndef FILE_OPS_H
#define FILE_OPS_H

#include "device.h"

#define CMD_STAT_FNAME_PREFIX "usb-cs"

int cmd_stat_file_init(struct cmd_stat_dev *dev);
void cmd_stat_file_exit(struct cmd_stat_dev *dev);

#endif
