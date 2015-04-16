/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief USB command/status get
 *
 * $Id: get.h,v 1.4.8.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file defines structures used by the USB command/status get commands.
 */
#ifndef GET_H
#define GET_H

#include "device.h"
#include <linux/device.h>

void do_get(struct work_struct *work);
void do_get_in_buffer(struct cmd_stat_get_dev *dev);
int cmd_stat_get_init(struct cmd_stat_dev *mdev, dev_t dev_num);
int cmd_stat_get_exit(struct cmd_stat_dev *mdev, dev_t dev_num);

#endif
