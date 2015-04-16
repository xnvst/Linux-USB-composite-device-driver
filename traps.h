/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief USB command/status traps
 *
 * $Id: traps.h,v 1.2.14.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file defines structures used by the USB command/status traps.
 */
#ifndef TRAPS_H
#define TRAPS_H

#include "device.h"
#include <linux/device.h>

int cmd_stat_trap_init(struct cmd_stat_dev *mdev, dev_t dev_num);
int cmd_stat_trap_exit(struct cmd_stat_dev *mdev, dev_t dev_num);

#endif
