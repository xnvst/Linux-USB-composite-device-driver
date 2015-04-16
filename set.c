/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief USB command/status set
 *
 * $Id: set.c,v 1.5.8.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file defines structures used by the USB command/status set commands.
 */
#include "set.h"
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

/** @brief Wait for incoming "get" request */
#define STATE_WAIT 0
/** @brief "Set" request needs to be processed */
#define STATE_REQ 1
/** @brief Waiting for user land app to respond with data */
#define STATE_RESP 2

struct set_buffer_entry
{
   struct cmd_stat_get_set m_get_set;
   char m_data[EP0_BUF_SIZE];
};

struct set_buffer_data
{
   struct list_head node;
   int      length;
   char    *data;
};

#define MAX_SET_BUFFER_ENTRY 80
#define MAX_SET_BUFFER_IN_ENTRY (100)

/** @brief Command/status "set" file device */
struct cmd_stat_set_dev {
   struct class *dev_class;          /**< Device class */
	struct cdev cdev;                 /**< Used to register as a character device */
   struct cmd_stat_usb_gadget *usb;  /**< Used to access USB gadget */
   struct cmd_stat_get_set *get_set; /**< Data variable ID to get */
	struct semaphore sem;             /**< Mutual exclusion */
	atomic_t state;                   /**< Pending "set" request */
   wait_queue_head_t req_pend_q;     /**< "Set" request wait queue */
   wait_queue_head_t resp_pend_q;    /**< "Set" user land app response wait queue */
   struct list_head  m_set_buf_list;
   //struct set_buffer_entry m_set_buffer[MAX_SET_BUFFER_ENTRY];
   //unsigned m_buffer_entries;
   struct set_buffer_entry m_set_buffer_in[MAX_SET_BUFFER_IN_ENTRY];
   //unsigned m_buffer_in_entries;
   int m_buffer_write;
   int m_buffer_read;
   //struct semaphore sem_in;            
};

/** @brief "Set" device class name as it appears in /sys/class */
#define SET_CLASS_NAME CMD_STAT_FNAME_PREFIX "_set"
/** @brief "Set" device names as they appears in /dev */
const char * const SET_DEV_NAME[] = {
   CMD_STAT_FNAME_PREFIX "-set",
   CMD_STAT_FNAME_PREFIX "-set_confirm",
};

/** @brief Command/status "confirm" file device */
struct cmd_stat_confirm_dev {
   struct class *dev_class;      /**< Device class */
	struct cdev cdev;             /**< Used to register as a character device */
	struct semaphore sem;         /**< Mutual exclusion */
	atomic_t confirm;             /**< Confirm or clear */
   wait_queue_head_t req_pend_q; /**< "Set" request wait queue */
};

/** @brief "Confirm" device names as they appears in /dev */
#define CONFIRM_DEV_NAME CMD_STAT_FNAME_PREFIX "-confirm"

/**
 * @brief put the set data into buffer.
 *    When set data comes from usb host,
 *    first put into buffer so that no data gets lost
 */
void 
do_set_in_buffer(struct cmd_stat_set_dev *dev)
{
   struct set_buffer_entry *pentry; 

    pentry = &dev->m_set_buffer_in[dev->m_buffer_write];
         memcpy((void *)&pentry->m_get_set, (void *)dev->get_set, sizeof(pentry->m_get_set));
         memcpy((void *)pentry->m_data, (void *)dev->usb->req->buf, dev->get_set->len);
         
    dev->m_buffer_write++;
    if (dev->m_buffer_write >= MAX_SET_BUFFER_IN_ENTRY)
       dev->m_buffer_write = 0;
    //printk("\n *id[%d]:idx[%d]*\n", pentry->m_get_set.var_id, pentry->m_get_set.var_idx);
}

/**
 * @brief Performs work for validating parameter value.
 *
 * The validation must be done by an app in user space. This device will wake up
 * the app that is waiting for a read operation. The app will read the request,
 * and respond by writing an error code to this device.
 */
void
do_set(struct cmd_stat_set_dev *dev) /**< "Set" device */
{
    int wrap_around = 0;
    struct set_buffer_data  *data_node = NULL;
    volatile int read = dev->m_buffer_read;
    volatile int write = dev->m_buffer_write;
    int length = 0;

   /* Flag that there is a pending request */
   /* Wake all processes waiting to process a "set" request */
	down(&dev->sem);
     //state = atomic_read(&dev->state);

    //printk("%s n=%d\n", __FUNCTION__, dev->m_buffer_in_entries);
    //printk("do_set start\n");
    if (write >= read) 
     {
        length = write - read;
        wrap_around = 0;
     }
     else
     {
        length = MAX_SET_BUFFER_IN_ENTRY - read + write;
        wrap_around = 1;
     }
     
    dev->m_buffer_read = write;
    data_node = (struct set_buffer_data  *)kmalloc(sizeof(struct set_buffer_data), GFP_KERNEL);
    if (!data_node)
    {
        printk("%s->kmalloc failed!\n", __FUNCTION__);
        return;
    }
    data_node->length = length * sizeof(struct set_buffer_entry);
    data_node->data = NULL;
    data_node->data = (char *)kmalloc(data_node->length, GFP_KERNEL);
    if (!data_node->data)
        printk("%s->kmalloc[%d] failed!\n", __FUNCTION__, data_node->length);

    if (!wrap_around) 
    {
        memcpy((void *)data_node->data, (void *)&(dev->m_set_buffer_in[read]), data_node->length);
        //for (i=0; i<length; i++)
            //printk("set-idx[%d]\n", ((struct set_buffer_entry *)data_node->data)[i].m_get_set.var_idx);
    }
    else
    {
        //printk("wrap_around\n");
        memcpy((void *)data_node->data, (void *)&(dev->m_set_buffer_in[read]), 
                sizeof(struct set_buffer_entry)*(MAX_SET_BUFFER_IN_ENTRY - read));
        //for (i=0; i<(MAX_SET_BUFFER_IN_ENTRY - read); i++)
            //printk("set-idx[%d]\n", ((struct set_buffer_entry *)data_node->data)[i].m_get_set.var_idx);
        memcpy((void *)(data_node->data+sizeof(struct set_buffer_entry)*(MAX_SET_BUFFER_IN_ENTRY - read)), 
               (void *)&(dev->m_set_buffer_in[0]), sizeof(struct set_buffer_entry)*(write));
        //for (i=0; i<write; i++)
            //printk("set-idx[%d]\n", ((struct set_buffer_entry *)data_node->data)[i].m_get_set.var_idx);
    }
    
    list_add_tail(&data_node->node, &dev->m_set_buf_list);
     atomic_set(&dev->state, STATE_REQ);
	up(&dev->sem);
	wake_up_interruptible(&dev->req_pend_q);
}

/**
 * @brief Performs work for confirming parameter value.
 *
 * The confirm must be done by an app in user space. This device will wake up
 * the app that is waiting for a read operation. The app will read the request,
 * and perform the operation.
 */
void
do_confirm(struct work_struct *work) /**< Work structure */
{
   struct cmd_stat_dev *mdev = container_of(work, struct cmd_stat_dev, confirm_work);
   struct cmd_stat_confirm_dev *dev = mdev->confirm;
   //printk("%s\n",__FUNCTION__);
   return;
   /* Flag that there is a pending request */
   /* Wake all processes waiting to process a "confirm" request */
	down(&dev->sem);
	atomic_set(&dev->confirm, 1);
	up(&dev->sem);
	wake_up_interruptible(&dev->req_pend_q);
}

/**
 * @brief Performs work for unconfirming parameter value.
 *
 * The unconfirm must be done by an app in user space. This device will wake up
 * the app that is waiting for a read operation. The app will read the request,
 * and perform the operation.
 */
void
do_clear(struct work_struct *work) /**< Work structure */
{
   struct cmd_stat_dev *mdev = container_of(work, struct cmd_stat_dev, clr_confirm_work);
   struct cmd_stat_confirm_dev *dev = mdev->confirm;
   //printk("%s\n",__FUNCTION__);
   //printk("%s: varid[%d], index[%d], length[%d]\n", __FUNCTION__, mdev->get_set.var_id, mdev->get_set.var_idx, mdev->get_set.len);
   /* Flag that there is a pending request */
   /* Wake all processes waiting to process a "confirm" request */
	down(&dev->sem);
	atomic_set(&dev->confirm, 0);
	up(&dev->sem);
	wake_up_interruptible(&dev->req_pend_q);
}

/**
 * @brief Polls device to see if read and write will block
 * @return Bit mask indicating which operations will not block
 */
static unsigned int
cmd_stat_set_poll(struct file *filp,
                  poll_table *wait)
{
	struct cmd_stat_set_dev *dev =
      (struct cmd_stat_set_dev *) filp->private_data;
	unsigned int mask = 0;

     //printk("%s\n",__FUNCTION__);
	down(&dev->sem);
	poll_wait(filp, &dev->req_pend_q, wait);
	poll_wait(filp, &dev->resp_pend_q, wait);
    if (atomic_read(&dev->state) == STATE_REQ || !list_empty(&dev->m_set_buf_list))
   {
		mask |= POLLIN | POLLRDNORM;	/* readable */
   }
	if (atomic_read(&dev->state) == STATE_RESP) 
	{
		mask |= POLLOUT | POLLWRNORM;	/* writable */
   }
	up(&dev->sem);
	return mask;
}

/**
 * @brief Return the "set" pending request data to the user land app
 * @return Number of bytes copied to the user land buffer
 * @retval <0 Error
 */
static ssize_t
cmd_stat_set_read(struct file *filp,
                  char __user *buf,
                  size_t count,
                  loff_t *f_pos)
{
	struct cmd_stat_set_dev *dev =
      (struct cmd_stat_set_dev *) filp->private_data;
   int state;
   struct set_buffer_data  *pEntry = NULL;
   struct list_head    *p, *pn;
   int ret_count;

   //state = atomic_read(&dev->state);
   //printk("%s,state[%d]\n",__FUNCTION__,state);
   
    if (down_interruptible(&dev->sem)) 
    {
		return -ERESTARTSYS;
   }

   /* Wait for incoming "set" request */
   while (atomic_read(&dev->state) != STATE_REQ)
   {
      up(&dev->sem); /* release the lock */
      if (filp->f_flags & O_NONBLOCK) {
         return -EAGAIN;
      }
      if (wait_event_interruptible(dev->req_pend_q,atomic_read(&dev->state) == STATE_REQ ))
      {
         return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
      }
      /* otherwise loop, but first reacquire the lock */
      if (down_interruptible(&dev->sem)) {
         return -ERESTARTSYS;
      }
   }

   /* Copy request data to user */
   state = atomic_read(&dev->state);
   //if (state == STATE_REQ) 
   {
      state = STATE_RESP;

        if (list_empty(&dev->m_set_buf_list))
      {
            printk("%s warning -> empty buffer\n", __FUNCTION__);
         up(&dev->sem);
         return -EFAULT;         
      }

        ret_count = 0;
        list_for_each_safe(p, pn, &dev->m_set_buf_list)
      {
            pEntry = (struct set_buffer_data *)p;
            if (copy_to_user(buf + ret_count, pEntry->data, pEntry->length)) 
         {
               printk("%s copy_to_user failed!\n", __FUNCTION__);
               up(&dev->sem);
               return -EFAULT;
      }
      
            list_del(&pEntry->node);
            INIT_LIST_HEAD(&pEntry->node);
            kfree(pEntry->data);
            kfree(pEntry);

            ret_count += pEntry->length;
            if (ret_count >= count)
      {
                break;
            }
      }
      
        //for (i=0; i<pEntry->length/sizeof(struct set_buffer_entry); i++)
        //printk("read[%d]\n", ((struct set_buffer_entry*)pEntry->data)[i].m_get_set.var_idx);
   }

   atomic_set(&dev->state, state);

   up(&dev->sem);

   if (state == STATE_RESP) 
   {
      /* Wake up all processes waiting to respond to the request. */
      wake_up_interruptible(&dev->resp_pend_q);
      wake_up_interruptible(&dev->req_pend_q);
   } 
   else 
   {
      wake_up_interruptible(&dev->req_pend_q);
   }
   
    return ret_count;
}

/**
 * @brief Write the "set" request's value to USB
 * @return Number of bytes copied from the user land buffer
 * @retval <0 Error
 */
static ssize_t
cmd_stat_set_write(struct file *filp,
                   const char __user *buf,
                   size_t count,
                   loff_t *f_pos)
{
	struct cmd_stat_set_dev *dev =
      (struct cmd_stat_set_dev *) filp->private_data;
   struct usb_gadget *gadget = dev->usb->gadget;
   //struct cmd_stat_dev *mdev = get_gadget_data(gadget);
	struct usb_request *req = dev->usb->req;
   unsigned char app_err;
   int ret;
   int state;
   
   //printk("%s\n",__FUNCTION__);
   
   if (down_interruptible(&dev->sem)) 
   {
      printk("Sem error\n");
      return -ERESTARTSYS;
   }

   state = atomic_read(&dev->state);
   if(state==STATE_RESP)atomic_set(&dev->state, STATE_WAIT);
   
   //printk("1-state[%d]\n",state);
#if 0   //don't wait
   /* Wait for pending "set" response */   
   while (atomic_read(&dev->state) != STATE_RESP) {
      up(&dev->sem); /* release the lock */
      if (filp->f_flags & O_NONBLOCK) {
         printk("Non-block socket released\n");
         return -EAGAIN;
      }
      if (wait_event_interruptible(dev->resp_pend_q,
               atomic_read(&dev->state) == STATE_RESP))
      {
         printk("Socket restart\n");
         return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
      }
      /* otherwise loop, but first reacquire the lock */
      if (down_interruptible(&dev->sem)) {
         printk("Can not lock sem\n");
         return -ERESTARTSYS;
      }
   }
#endif

   if (count == 0) 
   {
      /* App didn't send an error code. Assume something is wrong. */
      printk("%s: set write NULL packet\n", __FUNCTION__);
      app_err = 1;
   } 
   else 
   {
      /* copy user data */
      if (copy_from_user(&app_err, buf, 1)) {
         up(&dev->sem);
         printk("Copy from user error\n");
         return -EFAULT;
      }
   }

   if (app_err) 
   {
      /* Application reported an error */
      usb_ep_set_halt(gadget->ep0);
   } 
   else 
   {

      //down(&mdev->sem);
      /* Send 0-length packet to indicate success */
      req->length = 0;
      req->zero = 1;
      req->complete = transfer_complete;
      ret = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
      if (ret < 0) 
      {
         printk("%s: usb ep queue failed[%d]\n", __FUNCTION__, ret);
         req->status = 0;
         transfer_complete(gadget->ep0, req);
      }
      //up(&mdev->sem);
   }

   /* "Set" response has been transmitted. Go back to idle state */
   //atomic_set(&dev->state, STATE_WAIT);

   up(&dev->sem);

   return count;
}

/**
 * @brief Opens a "get" file
 * @retval 0 Success
 * @retval non-0 Failure
 */
static int
cmd_stat_set_open(struct inode *inode,
                  struct file *filp)
{
	struct cmd_stat_set_dev *dev;


   nonseekable_open(inode, filp);

	/*  Find the device */
	dev = container_of(inode->i_cdev, struct cmd_stat_set_dev, cdev);

	/* and use filp->private_data to point to the device data */
	filp->private_data = dev;

   /* Success */
   return 0;
}

/* Structure that declares the usual file access functions */
struct file_operations set_fops = {
   llseek: no_llseek,
   read: cmd_stat_set_read,
   write: cmd_stat_set_write,
	poll:	cmd_stat_set_poll,
   open: cmd_stat_set_open
};

/**
 * @brief Polls device to see if read will block
 * @return Bit mask indicating which operations will not block
 */
static unsigned int
cmd_stat_confirm_poll(struct file *filp,
                      poll_table *wait)
{
	struct cmd_stat_confirm_dev *dev =
      (struct cmd_stat_confirm_dev *) filp->private_data;
	unsigned int mask = 0;

      printk("%s\n",__FUNCTION__);

	down(&dev->sem);
	poll_wait(filp, &dev->req_pend_q, wait);
	if (atomic_read(&dev->confirm) >= 0) {
		mask |= POLLIN | POLLRDNORM;	/* readable */
   }
	up(&dev->sem);
	return mask;
}

/**
 * @brief Return the "confirm" pending request data to the user land app
 * @return Number of bytes copied to the user land buffer
 * @retval <0 Error
 */
static ssize_t
cmd_stat_confirm_read(struct file *filp,
                      char __user *buf,
                      size_t count,
                      loff_t *f_pos)
{
	struct cmd_stat_confirm_dev *dev =
      (struct cmd_stat_confirm_dev *) filp->private_data;
   unsigned char rd;

   printk("%s\n",__FUNCTION__);
	if (down_interruptible(&dev->sem)) {
		return -ERESTARTSYS;
   }

   /* Wait for incoming "set" request */
   while (atomic_read(&dev->confirm) < 0) {
      up(&dev->sem); /* release the lock */
      if (filp->f_flags & O_NONBLOCK) {
         return -EAGAIN;
      }
      if (wait_event_interruptible(dev->req_pend_q,
               atomic_read(&dev->confirm) >= 0))
      {
         return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
      }
      /* otherwise loop, but first reacquire the lock */
      if (down_interruptible(&dev->sem)) {
         return -ERESTARTSYS;
      }
   }

   /* Copy request data to user */
   rd = (unsigned char) atomic_read(&dev->confirm);
	if (copy_to_user(buf, &rd, 1)) {
      up(&dev->sem);
		return -EFAULT;
	}

   /* "Confirm" request is going to be serviced now. Tell other processes to
    * wait for next request. */
   atomic_set(&dev->confirm, -1);

   up(&dev->sem);

	return 1;
}

/**
 * @brief Opens a "get" file
 * @retval 0 Success
 * @retval non-0 Failure
 */
static int
cmd_stat_confirm_open(struct inode *inode,
                      struct file *filp)
{
	struct cmd_stat_confirm_dev *dev;

   nonseekable_open(inode, filp);

	/*  Find the device */
	dev = container_of(inode->i_cdev, struct cmd_stat_confirm_dev, cdev);

	/* and use filp->private_data to point to the device data */
	filp->private_data = dev;

   /* Success */
   return 0;
}

/* Structure that declares the usual file access functions */
struct file_operations confirm_fops = {
   llseek: no_llseek,
   read: cmd_stat_confirm_read,
	poll:	cmd_stat_confirm_poll,
   open: cmd_stat_confirm_open
};

/**
 * @brief Called at module insertion to initialize the trap device.
 * @return Number of minor devices allocated
 * @retval <0 Failure
 */
int
cmd_stat_set_init(struct cmd_stat_dev *mdev, /**< Command/status device structure */
                  dev_t dev_num)             /**< Device number */
{
   int result;
   int num_dev = 0;
   unsigned i;
   int major = MAJOR(dev_num), minor = MINOR(dev_num);
   int err_minor = minor;

   for (i = 0; i < NUM_SET_DEV; i++) {
      if (mdev && major) {
         struct cmd_stat_set_dev *dev;
         minor = MINOR(dev_num);

         mdev->set[i] = NULL;
         dev = kzalloc(sizeof *dev, GFP_KERNEL);
         if (!dev) {
            result = -ENOMEM;
            goto fail;
         }
         mdev->set[i] = dev;
         INIT_LIST_HEAD(&mdev->set[i]->m_set_buf_list);
         /* Register device */
         cdev_init(&dev->cdev, &set_fops);
         dev->cdev.owner = THIS_MODULE;
         result = cdev_add(&dev->cdev, dev_num, 1);
         if (result) {
            printk(KERN_NOTICE "Error %d adding %s\n", result, SET_DEV_NAME[i]);
            goto fail;
         }

         /* Create USB command/status set device class */
         if (i == 0) {
            dev->dev_class = class_create(THIS_MODULE, SET_CLASS_NAME);
            //dev->dev_class = class_simple_create(THIS_MODULE, SET_CLASS_NAME);
         } else {
            dev->dev_class = mdev->set[i - 1]->dev_class;
         }

         /* Create character device file */
         device_create(dev->dev_class, NULL, dev_num, NULL, SET_DEV_NAME[i]);

         //major = register_chrdev(MAJOR(dev_num), SET_DEV_NAME[i], &set_fops);
         //printk("Driver %s uses major %d and minor %d.\n", SET_DEV_NAME[i], MAJOR(dev_num),MINOR(dev_num));
         //devfs_mk_cdev(dev_num, S_IFCHR | S_IRUGO | S_IWUGO, SET_DEV_NAME[i]);      
         

         /* Initialize all structures used for sharing and sleeping */
         sema_init(&dev->sem, 1);
         //sema_init(&dev->sem_in, 1);
         atomic_set(&dev->state, STATE_WAIT);
         init_waitqueue_head(&dev->req_pend_q);
         init_waitqueue_head(&dev->resp_pend_q);

         /* Set USB gadget */
         dev->usb = &mdev->usb;
         dev->get_set = &mdev->get_set;

         /* Increment minor number */
         minor++;
         dev_num = MKDEV(major, minor);
      }

      num_dev++;
   }

   if (mdev && major) {
      struct cmd_stat_confirm_dev *dev;

      mdev->confirm = NULL;
      dev = kzalloc(sizeof *dev, GFP_KERNEL);
      if (!dev) {
         result = -ENOMEM;
         goto fail;
      }
      mdev->confirm = dev;

      /* Register device */
      cdev_init(&dev->cdev, &confirm_fops);
      dev->cdev.owner = THIS_MODULE;
      result = cdev_add(&dev->cdev, dev_num, 1);
      if (result) {
         printk(KERN_NOTICE "Error %d adding %s\n", result, CONFIRM_DEV_NAME);
         goto fail;
      }

      /* Create USB command/status confirm device class */
      dev->dev_class = class_create(THIS_MODULE, CONFIRM_DEV_NAME);
      //dev->dev_class = class_simple_create(THIS_MODULE, CONFIRM_DEV_NAME);

      /* Create character device file */
      device_create(dev->dev_class, NULL, dev_num, NULL, CONFIRM_DEV_NAME);
      //major = register_chrdev(MAJOR(dev_num), CONFIRM_DEV_NAME, &confirm_fops);
      //printk("Driver %s uses major %d and minor %d.\n", CONFIRM_DEV_NAME, MAJOR(dev_num),MINOR(dev_num));

      //devfs_mk_cdev(dev_num, S_IFCHR | S_IRUGO | S_IWUGO, CONFIRM_DEV_NAME);      

      /* Initialize all structures used for sharing and sleeping */
      sema_init(&dev->sem, 1);
      atomic_set(&dev->confirm, -1);
      init_waitqueue_head(&dev->req_pend_q);
   }

   /* Increment minor number */
   num_dev++;

   return num_dev;

fail:
   dev_num = MKDEV(major, err_minor);
   cmd_stat_set_exit(mdev, dev_num);
   return result;
}

/**
 * @brief Called at module removal to tear down the trap device.
 * @return Number of minor devices deallocated
 */
int
cmd_stat_set_exit(struct cmd_stat_dev *mdev, /**< Command/status device structure */
                  dev_t dev_num)             /**< Device number */
{
   int num_dev = 0;
   unsigned i;
   int major = MAJOR(dev_num), minor;
   struct class *dev_class = NULL;

   
   for (i = 0; i < NUM_SET_DEV; i++) {
      struct cmd_stat_set_dev *dev = mdev->set[i];
      minor = MINOR(dev_num);
      if (dev) {
         if (dev->dev_class) {
            dev_class = dev->dev_class;
         }
         device_destroy(dev_class, dev_num);         
         //devfs_remove(SET_DEV_NAME[i]);
         //class_simple_device_remove(dev_num);
         
         cdev_del(&dev->cdev);
         kfree(dev);
         mdev->set[i] = NULL;
      }

      /* Increment minor number */
      num_dev++;
      minor++;
      dev_num = MKDEV(major, minor);
   }

   if (dev_class) {
      class_destroy(dev_class);
      //class_simple_destroy(dev_class);
   }

   {
      struct cmd_stat_confirm_dev *dev = mdev->confirm;
      if (dev) {
         device_destroy(dev->dev_class, dev_num);
         class_destroy(dev->dev_class);
         //devfs_remove(CONFIRM_DEV_NAME);
         //class_simple_device_remove(dev_num);
         //class_simple_destroy(dev->dev_class);
         
         cdev_del(&dev->cdev);   
         
         kfree(dev);
         mdev->confirm = NULL;
      }

      /* Increment minor number */
      num_dev++;
   }

   return num_dev;
}
