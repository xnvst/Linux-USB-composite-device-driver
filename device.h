/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief USB command/status device
 *
 * $Id: device.h,v 1.5.8.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file defines the structure that is used by the USB command/status device
 * to maintain internal data.
 */
#ifndef DEVICE_H
#define DEVICE_H

#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <asm/types.h>
#include <linux/semaphore.h>
#include "get_set_if.h"

struct usb_gadget;
struct usb_request;
struct usb_ep;

/** @brief Structure used to store USB data. */
struct cmd_stat_usb_gadget {
	spinlock_t usb_lock; /**< Spin lock for USB ISR */
	struct usb_gadget	*gadget; /**< Device driver's USB gadget */
	struct usb_request *req; /**< Enpoint 0 transfer requests */

	u8 config; /**< Current USB configuraton */
	struct usb_ep *trap_use_ep; /**< Endpoint to use for traps */
	struct usb_ep *trap_ep; /**< Enabled trap endpoint */
};

struct cmd_stat_trap_dev;
struct cmd_stat_get_dev;
struct cmd_stat_set_dev;

/** @brief Number of devices needed for "set" command */
#define NUM_SET_DEV 2

/**
 * @brief Structure used to maintain USB command status device driver data.
 */
struct cmd_stat_dev {
	dev_t dev_num; /**< First device number (major, minor) */
   unsigned num_dev; /**< Number of file devices */

   struct cmd_stat_usb_gadget usb; /**< Used to access USB */

   struct cmd_stat_get_dev *get; /**< "Get" data */
   struct cmd_stat_set_dev *set[NUM_SET_DEV]; /**< "Set" and "set-confirm" data */
   struct cmd_stat_confirm_dev *confirm; /**< "Confirm" data */
   struct cmd_stat_trap_dev *trap; /**< Trap data */

   struct cmd_stat_get_set get_set; /**< Data variable ID to get/set */

   u8 cmd; /**< Setup packet command command */

   struct work_struct get_work;
   struct work_struct set_work;
   struct work_struct set_confirm_work;
   struct work_struct confirm_work;
   struct work_struct clr_confirm_work;
   struct semaphore sem;
};

#endif
