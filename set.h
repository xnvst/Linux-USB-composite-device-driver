/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief USB command/status set
 *
 * $Id: set.h,v 1.3.8.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file defines structures used by the USB command/status set commands.
 */
#ifndef SET_H
#define SET_H

#include "device.h"
#include <linux/device.h>

void do_set(struct cmd_stat_set_dev *dev);
void do_set_in_buffer(struct cmd_stat_set_dev *dev);
void do_confirm(struct work_struct *work);
void do_clear(struct work_struct *work);
int cmd_stat_set_init(struct cmd_stat_dev *mdev, dev_t dev_num);
int cmd_stat_set_exit(struct cmd_stat_dev *mdev, dev_t dev_num);

#endif
