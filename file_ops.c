/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief USB command/status file operations
 *
 * $Id: file_ops.c,v 1.5.14.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file implements the file operations supported by the USB command/status
 * protocol. It is needed by the user land application to perform gets and sets,
 * and to send traps.
 */
#include "file_ops.h"
#include "device.h"
#include "traps.h"
#include "get.h"
#include "set.h"

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/ioport.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>
//#include <asm/arch/hardware.h>
#include <linux/device.h>
#include <linux/poll.h>

MODULE_LICENSE("Dual BSD/GPL");

//
// Structure definitions
//
/** @brief Initialize device */
typedef int (*f_init)(struct cmd_stat_dev *, dev_t);
/** @brief Terminate device */
typedef int (*f_exit)(struct cmd_stat_dev *, dev_t);
/** @brief Used to maintain all supported devices */
struct dev_def {
   unsigned minor; /**< Device minor number */
   f_init init;    /**< Device init */
   f_exit exit;    /**< Device terminate */
};

//
// Globals
//
/** @brief First minor device number */
#define FIRST_MINOR 0

/** @brief Minor device definitions */
struct dev_def dev_list[] = {
   {0, cmd_stat_trap_init, cmd_stat_trap_exit},
   {0, cmd_stat_get_init, cmd_stat_get_exit},
   {0, cmd_stat_set_init, cmd_stat_set_exit},
   {0, NULL, NULL}
};

/**
 * @brief Initializes the USB command/status file devices.
 *
 * This function is called at module insertion time to initialize all character
 * devices associated with the USB command status protocol.
 * @retval 0 Success
 * @retval <0 Failure
 */
int
cmd_stat_file_init(struct cmd_stat_dev *dev) /**< Command/status device */
{
	dev_t dev_num;
   int major, minor;
   struct dev_def *d;
   int num_dev = 0;
   int result;

   /* Determine number of file devices */
   dev_num = MKDEV(0, 0);
   for (d = dev_list; d->init != NULL; d++) {
      num_dev += d->init(NULL, dev_num);
   }

   /* Registering device */
   result = alloc_chrdev_region(&dev_num, FIRST_MINOR, num_dev, CMD_STAT_FNAME_PREFIX);
   if (result < 0) {
      printk(KERN_ERR "%s: cannot obtain major number\n", CMD_STAT_FNAME_PREFIX);
      return result;
   }
	dev->dev_num = dev_num;
   dev->num_dev = num_dev;

   /* Initialize all minor devices */
   major = MAJOR(dev_num);
   minor = MINOR(dev_num);
   for (d = dev_list; d->init != NULL; d++) {
      d->minor = minor;
      num_dev = d->init(dev, dev_num);
      if (num_dev < 0) {
         return num_dev;
      }
      minor += num_dev;
      dev_num = MKDEV(major, minor);
   }

   return 0;
}

/**
 * @brief Terminates the USB command/status file devices.
 *
 * This function is called at module removal time to tear down all character
 * devices associated with the USB command status protocol.
 */
void
cmd_stat_file_exit(struct cmd_stat_dev *dev) /**< Command/status device */
{
	dev_t dev_num;
   int major, minor;
   struct dev_def *d;
   unsigned num_dev;

   /* Terminate all minor devices */
   dev_num = dev->dev_num;
   major = MAJOR(dev_num);
   minor = MINOR(dev_num);
   for (d = dev_list; d->exit != NULL; d++) {
      num_dev = d->exit(dev, dev_num);
      minor += num_dev;
      dev_num = MKDEV(major, minor);
   }

   /* Freeing the major number */
	unregister_chrdev_region(dev->dev_num, dev->num_dev);
}
