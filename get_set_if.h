/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief User land interface to USB command/status get/set commands
 *
 * $Id: get_set_if.h,v 1.5.14.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file defines structures used by the USB command/status get/set commands.
 * This is the file that should be included by user land applications to write
 * to the command/status get/set device. It is shared by both user land and
 * kernel land.
 */
#ifndef GET_SET_IF_H
#define GET_SET_IF_H

/* "Set" error codes */
#define SET_OK 0
#define SET_INVAL 1
#define SET_NOVAR 2
#define SET_ERR 3

/** @brief Command/status get/set data from USB setup packet */
struct cmd_stat_get_set {
   uint16_t var_id;
   uint16_t var_idx;
   uint16_t len;
   uint16_t pad;
};







#if 0
#include <linux/ioctl.h>

/** @brief GPIO ioctl data */
struct gpio_ioctl_t {
   unsigned sel;  /**< GPIO selection bit mask */
   unsigned data; /**< GPIO bit data */
};

#define GET_SET_IOC_MAGIC  'g'

/**
 * @brief Read GPIO bits
 *
 * 1 bits in sel specify the GPIOs to read. The GPIO values are returned in the
 * same bit positions in data. For example, if GPIO 2 is to be read, bit 2 in
 * sel should be set to 1, and GPIO 2's value will be returned in bit 2 of data.
 */
#define GET_SET_RD _IOR(GET_SET_IOC_MAGIC, 0, struct gpio_ioctl_t)
/**
 * @brief Set GPIO direction
 *
 * A 1 sel bit means the corresponding GPIO's direction is being set.
 * A 0 data bit means GPIO is an output, a 1 data bit means GPIO is an input.
 */
#define GET_SET_DIR _IOW(GET_SET_IOC_MAGIC, 1, struct gpio_ioctl_t)
/** @brief Wait for GPIO ISR trigger */
#define GET_SET_ISR_TRIG_WAIT _IO(GET_SET_IOC_MAGIC, 2)
#endif

#endif
