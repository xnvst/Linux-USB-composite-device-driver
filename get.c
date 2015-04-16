/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief USB command/status get
 *
 * $Id: get.c,v 1.6.8.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file defines structures used by the USB command/status get commands.
 */
#include "get.h"
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
#include <linux/cdev.h>
#include <asm/system.h>
#include <asm/uaccess.h>
//#include <asm/arch/hardware.h>
#include <linux/semaphore.h>
#include <asm/atomic.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
//#include <linux/devfs_fs_kernel.h>
#include <linux/sched.h>

MODULE_LICENSE("Dual BSD/GPL");

/** @brief Device name as it appears in /dev */
#define DEV_NAME CMD_STAT_FNAME_PREFIX "-get"

/** @brief Wait for incoming "get" request */
#define STATE_WAIT 0
/** @brief "Get" request needs to be processed */
#define STATE_REQ 1
/** @brief Waiting for user land app to respond with data */
#define STATE_RESP 2

#define MAX_GET_BUFFER_ENTRY 80
#define MAX_GET_BUFFER_IN_ENTRY (MAX_GET_BUFFER_ENTRY-20)

/** @brief Command/status "get" file device */
struct cmd_stat_get_dev {
   struct class *dev_class;  /**< Device class */
	struct cdev cdev;                /**< Used to register as a character device */
   struct cmd_stat_usb_gadget *usb; /**< Used to access USB gadget */
   struct cmd_stat_get_set *get_set; /**< Data variable ID to get */
	struct semaphore sem; /**< Mutual exclusion */
	atomic_t state; /**< Pending "get" request */
   wait_queue_head_t req_pend_q; /**< "Get" request wait queue */
   wait_queue_head_t resp_pend_q; /**< "Get" user land app response wait queue */
   struct cmd_stat_get_set m_get_buffer_in[MAX_GET_BUFFER_IN_ENTRY];
   unsigned m_buffer_in_entries;
};

/**
 * @brief put the get data into buffer.
 *    When get data comes from usb host,
 *    first put into buffer so that no data gets lost
 */
void 
do_get_in_buffer(struct cmd_stat_get_dev *dev)
{
   struct cmd_stat_get_set *pentry; 

     if(MAX_GET_BUFFER_IN_ENTRY>dev->m_buffer_in_entries)
     {
         pentry = &dev->m_get_buffer_in[dev->m_buffer_in_entries];
         memcpy((void *)pentry, (void *)dev->get_set, sizeof(struct cmd_stat_get_set));
                  
         dev->m_buffer_in_entries++;
         if(dev->m_buffer_in_entries >=2)
         {
            printk("%s: varid[%d], index[%d], length[%d],buffer entry[%d]\n", 
                     __FUNCTION__,pentry->var_id,pentry->var_idx, pentry->len,dev->m_buffer_in_entries);
         }
     }
     else
     {
         printk("%s: get in buffer full\n", __FUNCTION__);
     }
}

/**
 * @brief Performs work for retrieving parameter value.
 *
 * The retrieval must be done by an app in user space. This device will wake up
 * the app that is waiting for a read operation. The app will read the request,
 * and respond by writing data to this device.
 */
void
do_get(struct work_struct *work) /**< Work structure */
{
   struct cmd_stat_dev *mdev = container_of(work, struct cmd_stat_dev, get_work);
   struct cmd_stat_get_dev *dev = mdev->get;

   //printk("%s: varid[%d], index[%d], length[%d]\n", __FUNCTION__, mdev->get_set.var_id, mdev->get_set.var_idx, mdev->get_set.len);
   
   /* Flag that there is a pending request */
   /* Wake all processes waiting to process a "get" request */
	down(&dev->sem);
	atomic_set(&dev->state, STATE_REQ);
	up(&dev->sem);
	wake_up_interruptible(&dev->req_pend_q);
}

/**
 * @brief Polls device to see if read and write will block
 * @return Bit mask indicating which operations will not block
 */
static unsigned int
cmd_stat_get_poll(struct file *filp,
                  poll_table *wait)
{
	struct cmd_stat_get_dev *dev =
      (struct cmd_stat_get_dev *) filp->private_data;
	unsigned int mask = 0;

    //printk("%s\n",__FUNCTION__);
	down(&dev->sem);
	poll_wait(filp, &dev->req_pend_q, wait);
	poll_wait(filp, &dev->resp_pend_q, wait);
	if (atomic_read(&dev->state) == STATE_REQ) {
		mask |= POLLIN | POLLRDNORM;	/* readable */
   }
	if (atomic_read(&dev->state) == STATE_RESP) {
	//else{
		mask |= POLLOUT | POLLWRNORM;	/* writable */
   }
	up(&dev->sem);
	return mask;
}

/**
 * @brief Return the "get" pending request data to the user land app
 * @return Number of bytes copied to the user land buffer
 * @retval <0 Error
 */
static ssize_t
cmd_stat_get_read(struct file *filp,
                  char __user *buf,
                  size_t count,
                  loff_t *f_pos)
{
	struct cmd_stat_get_dev *dev =
      (struct cmd_stat_get_dev *) filp->private_data;

    //printk("%s\n",__FUNCTION__);

	if (down_interruptible(&dev->sem)) {
		return -ERESTARTSYS;
   }

   /* Wait for incoming "get" request */
   while (atomic_read(&dev->state) != STATE_REQ) {
      up(&dev->sem); /* release the lock */
      if (filp->f_flags & O_NONBLOCK) {
         return -EAGAIN;
      }
      if (wait_event_interruptible(dev->req_pend_q,
               atomic_read(&dev->state) == STATE_REQ))
      {
         return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
      }
      /* otherwise loop, but first reacquire the lock */
      if (down_interruptible(&dev->sem)) {
         return -ERESTARTSYS;
      }
   }

   /* Copy request data to user */
   if (count > sizeof(*dev->get_set)) {
      count = sizeof(*dev->get_set);
   }
	if (copy_to_user(buf, dev->get_set, count)) {
      up(&dev->sem);
		return -EFAULT;
	}

    //dev->m_buffer_in_entries = 0;
    
   /* "Get" request is going to be serviced now. Tell other processes to wait
    * for next request, and flag that device is now waiting for a response
    * from the user land app. */
   atomic_set(&dev->state, STATE_RESP);

   up(&dev->sem);

   /* Wake up all processes waiting to respond to the request. */
	wake_up_interruptible(&dev->resp_pend_q);

	return count;
}

/**
 * @brief Write the "get" request's value to USB
 * @return Number of bytes copied from the user land buffer
 * @retval <0 Error
 */
static ssize_t
cmd_stat_get_write(struct file *filp,
                   const char __user *buf,
                   size_t count,
                   loff_t *f_pos)
{
	struct cmd_stat_get_dev *dev =
      (struct cmd_stat_get_dev *) filp->private_data;
   struct usb_gadget *gadget = dev->usb->gadget;
	struct usb_request *req = dev->usb->req;
   int ret;

	if (down_interruptible(&dev->sem)) {
		return -ERESTARTSYS;
   }

   /* Wait for pending "get" response */
   while (atomic_read(&dev->state) != STATE_RESP) {
      up(&dev->sem); /* release the lock */
      if (filp->f_flags & O_NONBLOCK) {
         return -EAGAIN;
      }
      if (wait_event_interruptible(dev->resp_pend_q,
               atomic_read(&dev->state) == STATE_RESP))
      {
         return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
      }
      /* otherwise loop, but first reacquire the lock */
      if (down_interruptible(&dev->sem)) {
         return -ERESTARTSYS;
      }
   }

   if (count == 0) {
      usb_ep_set_halt(gadget->ep0);
   } else {
      /* Copy user data to USB request buffer */
      if (count > dev->get_set->len) {
         count = dev->get_set->len;
      } else if (count < dev->get_set->len) {
         memset((unsigned char *) req->buf + count, 0, dev->get_set->len - count);
      }
      if (copy_from_user(req->buf, buf, count)) {
         up(&dev->sem);
         return -EFAULT;
      }

      /* Initiate data transmission */
      req->length = dev->get_set->len;
      req->zero = 0;
      req->complete = tx_data_complete;
      ret = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
      if (ret < 0) {
         req->status = 0;
         transfer_complete(gadget->ep0, req);
      }
   }

   /* "Get" response has been transmitted. Go back to idle state */
   atomic_set(&dev->state, STATE_WAIT);

   up(&dev->sem);

	return count;
}

/**
 * @brief Opens a "get" file
 * @retval 0 Success
 * @retval non-0 Failure
 */
static int
cmd_stat_get_open(struct inode *inode,
                  struct file *filp)
{
	struct cmd_stat_get_dev *dev;

   nonseekable_open(inode, filp);

	/*  Find the device */
	dev = container_of(inode->i_cdev, struct cmd_stat_get_dev, cdev);

	/* and use filp->private_data to point to the device data */
	filp->private_data = dev;

   /* Success */
   return 0;
}

/* Structure that declares the usual file access functions */
struct file_operations get_fops = {
   llseek: no_llseek,
   read: cmd_stat_get_read,
   write: cmd_stat_get_write,
	poll:	cmd_stat_get_poll,
   open: cmd_stat_get_open
};

/**
 * @brief Called at module insertion to initialize the trap device.
 * @return Number of minor devices allocated
 * @retval <0 Failure
 */
int
cmd_stat_get_init(struct cmd_stat_dev *mdev, /**< Command/status device structure */
                  dev_t dev_num)             /**< Device number */
{
   int result;
   int major;

   if (mdev && MAJOR(dev_num)) {
      struct cmd_stat_get_dev *dev;
      mdev->get = NULL;
      dev = kzalloc(sizeof *dev, GFP_KERNEL);
      if (!dev) {
         return -ENOMEM;
      }
      mdev->get = dev;

      /* Register device */
      cdev_init(&dev->cdev, &get_fops);
      dev->cdev.owner = THIS_MODULE;
      result = cdev_add(&dev->cdev, dev_num, 1);
      if (result) {
         printk(KERN_NOTICE "Error %d adding %s\n", result, DEV_NAME);
         goto fail;
      }

      /* Create USB command/status get device class */
      dev->dev_class = class_create(THIS_MODULE, DEV_NAME);
      //dev->dev_class = class_simple_create(THIS_MODULE, DEV_NAME);

      /* Create character device file */
      device_create(dev->dev_class, NULL, dev_num, NULL, DEV_NAME);
      //major = register_chrdev(MAJOR(dev_num), DEV_NAME, &get_fops);
      //printk("Driver %s uses major %d and minor %d.\n", DEV_NAME, MAJOR(dev_num),MINOR(dev_num));

      //devfs_mk_cdev(dev_num, S_IFCHR | S_IRUGO | S_IWUGO, DEV_NAME);

      /* Initialize all structures used for sharing and sleeping */
      sema_init(&dev->sem, 1);
      atomic_set(&dev->state, STATE_WAIT);
      init_waitqueue_head(&dev->req_pend_q);
      init_waitqueue_head(&dev->resp_pend_q);

      /* Set USB gadget */
      dev->usb = &mdev->usb;
      dev->get_set = &mdev->get_set;
   }

   return 1;

fail:
   cmd_stat_get_exit(mdev, dev_num);
   return result;
}

/**
 * @brief Called at module removal to tear down the trap device.
 * @return Number of minor devices deallocated
 */
int
cmd_stat_get_exit(struct cmd_stat_dev *mdev, /**< Command/status device structure */
                  dev_t dev_num)             /**< Device number */
{
   struct cmd_stat_get_dev *dev = mdev->get;

   if (dev) {
      device_destroy(dev->dev_class, dev_num);
      class_destroy(dev->dev_class);
      //devfs_remove(DEV_NAME);
      //class_simple_device_remove(dev_num);
      //class_simple_destroy(dev->dev_class);
      cdev_del(&dev->cdev);

      
      kfree(dev);
      mdev->get = NULL;
   }

   return 1;
}

