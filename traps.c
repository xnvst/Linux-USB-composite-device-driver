/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief USB command/status traps
 *
 * $Id: traps.c,v 1.5.8.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file defines structures used by the USB command/status traps.
 */
#include "traps.h"
#include "device.h"
#include "cmd_stat.h"
#include "file_ops.h"

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
#include <linux/semaphore.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
//#include <linux/devfs_fs_kernel.h>
#include <linux/sched.h>

#include "traps_if.h"

MODULE_LICENSE("Dual BSD/GPL");

/** @brief Device name as it appears in /dev */
#define DEV_NAME CMD_STAT_FNAME_PREFIX "-trap"

/** @brief Structure used to store USB command status trap device driver data. */
struct cmd_stat_trap_dev {
   struct class *dev_class;         /**< Device class */
	struct cdev cdev;                /**< Used to register as a character device */
   struct cmd_stat_usb_gadget *usb; /**< Used to access USB gadget */
   struct semaphore sem;            /**< Mutual exclusion semaphore */
   wait_queue_head_t ep_req_q;      /**< Endpoint buffer request wait queue */
};

/**
 * @brief Allocates a USB request and buffer for trap transmission
 * @return USB request structure
 * @retval NULL Could not allocate USB request
 */
static inline struct usb_request *
alloc_trap_req(struct usb_ep *ep) /**< Source or sink endpoint */
{
   return alloc_ep_req(ep, 64);
}

/**
 * @brief Trap transmission callback function.
 *
 * This function is called after the USB peripheral has transmitted the trap
 * data to the USB host. It ignores any transmission errors, and just performs
 * clean up.
 */
static void
trap_complete(struct usb_ep *ep,       /**< Trap endpoint */
              struct usb_request *req) /**< USB request */
{
	struct cmd_stat_trap_dev *dev = ep->driver_data;

   free_ep_req(ep, req);

   /* Flag that an endpoint request buffer has been freed, and wake up all waiting processes */
	wake_up_interruptible(&dev->ep_req_q);
}

/**
 * @brief Writes a trap through USB
 * @return Number of bytes written
 * @retval <0 Error
 */
static ssize_t
cmd_stat_trap_write(struct file *filp,      /**< File pointer */
                    const char __user *buf, /**< User land trap buffer */
                    size_t count,           /**< Trap buffer size */
                    loff_t *f_pos)          /**< File position, ignored */
{
	struct cmd_stat_trap_dev *dev = (struct cmd_stat_trap_dev *) filp->private_data;
   struct cmd_stat_usb_gadget *usb = dev->usb;
   struct usb_request *req;
   struct trap_file_hdr user_hdr;
   struct status_t {
      u16 wIntType;
      u16 wLength;
      u16 wVariable;
      u16 wIndex;
   } *header;
   size_t data_size;

   if(usb->trap_ep==0)
   {
      return -EAGAIN;
   }
   
   /* Make sure trap endpoint has been configured for command/status */
   if (usb->trap_ep->driver_data != dev) {
      return -EPERM;
   }

   /* Make sure buffer contains the header */
   if (count < sizeof(user_hdr)) {
      return -EINVAL;
   }
   data_size = count - sizeof(user_hdr);

	if (down_interruptible(&dev->sem)) {
		return -ERESTARTSYS;
   }

   /* Get trap endpoint request buffer, sleeping if necessary */
   req = alloc_trap_req(usb->trap_ep);
   while (req == NULL) {
      /* There is no endpoint request buffer available */
		DEFINE_WAIT(wait);
		
		up(&dev->sem);
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
      }
		prepare_to_wait(&dev->ep_req_q, &wait, TASK_INTERRUPTIBLE);
      req = alloc_trap_req(usb->trap_ep);
      if (req == NULL) {
			schedule();
      }
		finish_wait(&dev->ep_req_q, &wait);
		if (signal_pending(current)) {
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
      }
		if (down_interruptible(&dev->sem)) {
			return -ERESTARTSYS;
      }
      if (req == NULL) {
         req = alloc_trap_req(usb->trap_ep);
      }
	}
   up(&dev->sem);

   /* Set trap header */
	if (copy_from_user(&user_hdr, buf, sizeof(user_hdr))) {
      goto copy_err;
	}
   header = (struct status_t *) req->buf;
   header->wIntType = __constant_cpu_to_be16(0x07);
   header->wLength = __constant_cpu_to_be16(data_size);
   header->wVariable = __constant_cpu_to_be16(user_hdr.var_id);
   header->wIndex = __constant_cpu_to_be16(user_hdr.var_idx);

   /* Set trap data */
	if (copy_from_user(header + 1, buf + sizeof(user_hdr), data_size)) {
      goto copy_err;
	}

   /* Send trap */
   req->zero = 0;
   req->complete = trap_complete;
   usb_ep_queue(usb->trap_ep, req, GFP_ATOMIC);

   return count;

copy_err:
   free_ep_req(usb->trap_ep, req);
   return -EFAULT;
}

/**
 * @brief Opens a trap file
 * @retval 0 Success
 * @retval non-0 Failure
 */
static int
cmd_stat_trap_open(struct inode *inode,
                   struct file *filp)
{
   struct cmd_stat_trap_dev *dev;

   nonseekable_open(inode, filp);

	/* Find the device */
	dev = container_of(inode->i_cdev, struct cmd_stat_trap_dev, cdev);

	/* and use filp->private_data to point to the device data */
	filp->private_data = dev;

   /* Success */
   return 0;
}

/** @brief Structure that declares the usual file access functions */
struct file_operations trap_fops = {
   llseek: no_llseek,
   write: cmd_stat_trap_write,
   open: cmd_stat_trap_open,
};

/**
 * @brief Called at module insertion to initialize the trap device.
 * @return Number of minor devices allocated
 * @retval <0 Failure
 */
int
cmd_stat_trap_init(struct cmd_stat_dev *mdev, /**< Command/status device structure */
                   dev_t dev_num)             /**< Device number */
{
   int result;
   int major;

   if (mdev && MAJOR(dev_num)) {
      struct cmd_stat_trap_dev *dev;
      mdev->trap = NULL;
      dev = kzalloc(sizeof *dev, GFP_KERNEL);
      if (!dev) {
         return -ENOMEM;
      }
      mdev->trap = dev;

      /* Register device */
      cdev_init(&dev->cdev, &trap_fops);
      dev->cdev.owner = THIS_MODULE;
      result = cdev_add(&dev->cdev, dev_num, 1);
      if (result) {
         printk(KERN_NOTICE "Error %d adding %s\n", result, DEV_NAME);
         goto fail;
      }

      /* Create USB command/status trap device class */
      dev->dev_class = class_create(THIS_MODULE, DEV_NAME);
      //dev->dev_class = class_simple_create(THIS_MODULE, DEV_NAME);

      /* Create character device file */
      device_create(dev->dev_class, NULL, dev_num, NULL, DEV_NAME);
      //major = register_chrdev(MAJOR(dev_num), DEV_NAME, &trap_fops);
      //printk("Driver %s uses major %d and minor %d.\n", DEV_NAME, MAJOR(dev_num),MINOR(dev_num));
      //devfs_mk_cdev(dev_num, S_IFCHR | S_IRUGO | S_IWUGO, DEV_NAME);
      

      /* Initialize all structures used for sharing and sleeping */
      sema_init(&dev->sem, 1);
      init_waitqueue_head(&dev->ep_req_q);

      /* Set USB gadget */
      dev->usb = &mdev->usb;
   }

   return 1;

fail:
   cmd_stat_trap_exit(mdev, dev_num);
   return result;
}

/**
 * @brief Called at module removal to tear down the trap device.
 * @return Number of minor devices deallocated
 */
int
cmd_stat_trap_exit(struct cmd_stat_dev *mdev, /**< Command/status device structure */
                   dev_t dev_num)             /**< Device number */
{
   struct cmd_stat_trap_dev *dev = mdev->trap;

   if (dev) {
      device_destroy(dev->dev_class, dev_num);
      class_destroy(dev->dev_class);
      //devfs_remove(DEV_NAME);
      //class_simple_device_remove(dev_num);
      //class_simple_destroy(dev->dev_class);

      cdev_del(&dev->cdev);
      
      kfree(dev);
      mdev->trap = NULL;
   }

   return 1;
}
