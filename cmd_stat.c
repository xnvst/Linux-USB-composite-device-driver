/*
 * cmd_stat.c -- USB command status protocol
 *
 * Copyright (C) 2007 Evertz Microsystems
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * This gadget driver is heavily based on "Gadget Zero" by David Brownell.
 */

#define DEBUG 1
// #define VERBOSE

#include "cmd_stat.h"
#include "device.h"
#include "file_ops.h"
#include "get.h"
#include "set.h"

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/semaphore.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

/*-------------------------------------------------------------------------*/

#define DRIVER_VERSION "May 2009"

static const char shortname[] = "cmd_stat";
static const char longname[] = "Command/status USB protocol";

/*-------------------------------------------------------------------------*/


#define xprintk(d,level,fmt,args...) \
	dev_printk(level , &(d)->usb.gadget->dev , fmt , ## args)

#ifdef DEBUG
#define DBG(dev,fmt,args...) \
	xprintk(dev , KERN_DEBUG , fmt , ## args)
#else
#define DBG(dev,fmt,args...) \
	do { } while (0)
#endif /* DEBUG */

#ifdef VERBOSEdefining __KERNEL__
#define VDBG	DBG
#else
#define VDBG(dev,fmt,args...) \
	do { } while (0)
#endif /* VERBOSE */

#define ERROR(dev,fmt,args...) \
	xprintk(dev , KERN_ERR , fmt , ## args)
#define WARN(dev,fmt,args...) \
	xprintk(dev , KERN_WARNING , fmt , ## args)
#define INFO(dev,fmt,args...) \
	xprintk(dev , KERN_INFO , fmt , ## args)

/*-------------------------------------------------------------------------*/

static unsigned vendor = 0x0836; /* Evertz */
module_param(vendor, uint, S_IRUGO);
//static unsigned product = 0;
static unsigned product = 0xfefe;
//static unsigned product = 0x2c02;
module_param(product, uint, S_IRUGO);

/*-------------------------------------------------------------------------*/

#if 0 // GEOFF
/* GEOFF: Keep the following for easy reference while debugging this driver. */
#define DRIVER_VENDOR_NUM	0x0836		/* Evertz */
#define DRIVER_PRODUCT_NUM	0x2c02		/* 7710ARC */
#endif // GEOFF

/*-------------------------------------------------------------------------*/

/** @brief Command/status USB device descriptor. */
static struct usb_device_descriptor
device_desc = {
	.bLength = sizeof device_desc,
	.bDescriptorType = USB_DT_DEVICE,

	.bcdUSB = __constant_cpu_to_le16(0x0200),
	.bDeviceClass = USB_CLASS_VENDOR_SPEC,

   .bcdDevice = __constant_cpu_to_le16(0x0100),
	.iManufacturer = 0,
	.iProduct = 0,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};

/** @brief Command/status USB configuration descriptor. */
static struct usb_config_descriptor
cmd_status_config = {
	.bLength = sizeof cmd_status_config,
	.bDescriptorType = USB_DT_CONFIG,

	/* compute wTotalLength on the fly */
	.bNumInterfaces =	1, /* Only cmd/status i/f. DFU no longer supported */
	.bConfigurationValue = 1,
	.iConfiguration =	0,
	.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower = 1,	/* self-powered */
};

/** @brief Command/status USB on-the-go descriptor. */
static struct usb_otg_descriptor
otg_descriptor = {
	.bLength = sizeof otg_descriptor,
	.bDescriptorType = USB_DT_OTG,

	.bmAttributes = USB_OTG_SRP,
};

/** @brief Command/status USB interface descriptor. */
static const struct usb_interface_descriptor
cmd_status_intf = {
	.bLength = sizeof cmd_status_intf,
	.bDescriptorType = USB_DT_INTERFACE,

	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
	.iInterface = 0,
};

/** @brief Command/status USB trap endpoint descriptor. */
static struct usb_endpoint_descriptor
cmd_status_trap_desc = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = 1 | USB_DIR_IN,
	.bmAttributes = USB_ENDPOINT_XFER_INT,
   .wMaxPacketSize = cpu_to_le16(64),
   .bInterval = 1, // used to be 25.
};

/** @brief Used to build the USB "get config descriptor" response */
static const struct usb_descriptor_header *cmd_status_function [] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	(struct usb_descriptor_header *) &cmd_status_intf,
	(struct usb_descriptor_header *) &cmd_status_trap_desc,
	NULL,
};

/*-------------------------------------------------------------------------*/
/* Maintain endpoint USB requests                                          */
/*-------------------------------------------------------------------------*/

/**
 * @brief Allocates a USB request and buffer for specified endpoint
 * @return USB request structure
 * @retval NULL Could not allocate USB request
 */
struct usb_request *
alloc_ep_req(struct usb_ep *ep, /**< Source or sink endpoint */
             unsigned length)   /**< Buffer size */
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (req) {
		req->length = length;
		req->buf = kmalloc(length, GFP_ATOMIC);
		if (!req->buf) {
			usb_ep_free_request(ep, req);
			req = NULL;
		}
	}

	return req;
}

/** @brief Returns endpoint's USB request structure to the system */
void
free_ep_req(struct usb_ep *ep,       /**< Source or sink endpoint */
            struct usb_request *req) /**< USB request */
{
	if (req->buf) {
		kfree(req->buf);
   }
	usb_ep_free_request(ep, req);
}

/*-------------------------------------------------------------------------*/
/* Set USB configuration                                                   */
/*-------------------------------------------------------------------------*/

/**
 * @brief Configure USB for command/status configuration
 * @retval 0 Success
 * @retval non-0 Could not enable required endpoint
 */
static int
set_cmd_status_config(struct cmd_stat_dev *dev, /**< Device structure */
                      int gfp_flags)            /**< Unused */
{
	int result = 0;

   if (dev->usb.trap_use_ep) {
      /* Enable trap endpoint */
      result = usb_ep_enable(dev->usb.trap_use_ep, &cmd_status_trap_desc);
      if (result == 0) {
         dev->usb.trap_use_ep->driver_data = dev->trap;
         dev->usb.trap_ep = dev->usb.trap_use_ep;
      } else {
         ERROR (dev, "can't enable %s, result %d\n", dev->usb.trap_use_ep->name, result);
      }
   } else {
      result = -EINVAL;
   }

	/* Caller is responsible for cleanup on error */
	return result;
}

/** @brief Puts USB into the unconfigured state */
static void
reset_config(struct cmd_stat_dev *dev) /**< Device structure */
{
	if (dev->usb.config == 0) {
      /* Already in unconfigured state */
		return;
   }

	DBG(dev, "reset config\n");

   /* Disable all endpoints */
	if (dev->usb.trap_ep) {
		usb_ep_disable(dev->usb.trap_ep);
		dev->usb.trap_ep = NULL;
	}

   /* Now in unconfigured state */
	dev->usb.config = 0;
}

/*
 * @brief Sets USB configuration.
 *
 * Configuration numbers are specified in the USB config descriptors. They
 * are used to select how the USB interface will be configured.
 * @retval 0 Success
 * @retval non-0 Failure. USB will be in unconfigured state.
 */
static int
set_config(struct cmd_stat_dev *dev, /**< Device structure */
           unsigned number,          /**< Which config */
           int gfp_flags)            /**< Unused */
{
	int result = 0;
	struct usb_gadget *gadget = dev->usb.gadget;

   /* If config is not changing, nothing to do */
	if (number == dev->usb.config) {
		return 0;
   }

   /* Unconfigure before setting new config */
	reset_config(dev);

   /* Set new config */
	switch (number) {
	case 1:
		result = set_cmd_status_config(dev, gfp_flags);
      if (!result && !dev->usb.trap_ep) {
         result = -ENODEV;
      }
		break;

	default:
		result = -EINVAL;
		/* FALL THROUGH */

	case 0:
		return result;
	}

	if (result) {
		reset_config(dev);
   } else {
		char *speed;

		switch (gadget->speed) {
		case USB_SPEED_LOW: speed = "low"; break;
		case USB_SPEED_FULL: speed = "full"; break;
		case USB_SPEED_HIGH: speed = "high"; break;
		default: speed = "?"; break;
		}

		dev->usb.config = number;
		INFO(dev, "%s speed config #%d\n", speed, number);
	}

	return result;
}

/*
 * @brief Create USB config descriptor in device's endpoint 0 USB request
 * structure.
 * @return USB config descriptor size
 * @retval <0 error
 */
static int
create_config_desc(struct usb_gadget *gadget, /**< Device's USB gadget */
                   struct cmd_stat_dev *dev,  /**< Device */
		             u8 type,                   /**< Descriptor type */
                   unsigned index)            /**< Descriptor index */
{
	struct usb_request *req = dev->usb.req;
   u8 *buf = req->buf;
	struct usb_config_descriptor *cp = (struct usb_config_descriptor *) buf;
	int len;
	const struct usb_descriptor_header **function;
	u8	*dest = USB_DT_CONFIG_SIZE + buf;

	if (index > 0) {
		return -EINVAL;
   }

   function = cmd_status_function;

	/* For now, don't advertise srp-only devices */
	if (!gadget->is_otg) {
		function++;
   }

	/* Config descriptor first */
	if (EP0_BUF_SIZE < USB_DT_CONFIG_SIZE || !function) {
		return -EINVAL;
   }
   *cp = cmd_status_config; 

   /* Then interface/endpoint/class/vendor/... */
	/* Fill buffer from src[] until null descriptor ptr */
   len = EP0_BUF_SIZE - USB_DT_CONFIG_SIZE;
	for (; 0 != *function; function++) {
		unsigned desc_len = (*function)->bLength;

		if (desc_len > len) {
			return -EINVAL;
      }
		memcpy(dest, *function, desc_len);
		len -= desc_len;
		dest += desc_len;
	}

   /* Patch up the config descriptor */
	len = dest - buf + USB_DT_CONFIG_SIZE;
   cp->bLength = USB_DT_CONFIG_SIZE;
   cp->wTotalLength = cpu_to_le16(len);
   cp->bmAttributes |= USB_CONFIG_ATT_ONE;
	cp->bDescriptorType = type;

	return len;
}

/*-------------------------------------------------------------------------*/
/* Control transfer callback functions - called after a stage completes    */
/*-------------------------------------------------------------------------*/

/**
 * @brief Transfer completion callback.
 *
 * This function is called after the status stage of a control transfer
 * completes. In the case of a control read operation, the USB host will send
 * data to the USB device. This function ignores that data. In the case of a
 * control write operation, the USB device will send data to the USB host.
 * This function ignores any errors in that transmission.
 */
void
transfer_complete(struct usb_ep *ep,       /**< Transfer endpoint */
                  struct usb_request *req) /**< USB request */
{
	if (req->status || req->actual != req->length) {
		DBG((struct cmd_stat_dev *) ep->driver_data,
            "setup complete --> %d, %d/%d\n",
            req->status, req->actual, req->length);
   }
}

/**
 * @brief Data transmission completion callback.
 *
 * This function is called after the USB peripheral has transmitted a data
 * buffer to the USB host as part of an endpoint 0 control transfer (after the
 * IN data stage of a control transfer completes). The next step in the control
 * transfer is the status stage. This callback function simply ignores the data
 * sent by the USB host in this stage.
 */
void
tx_data_complete(struct usb_ep *ep,       /**< Transfer endpoint */
                 struct usb_request *req) /**< USB request */
{
	struct cmd_stat_dev *dev = ep->driver_data;
   int ret;

   /* Throw out status stage */
   req->complete = transfer_complete;
   req->length = 0;
   req->zero = 1;
   ret = usb_ep_queue(ep, req, GFP_ATOMIC);
   if (ret < 0) {
      DBG (dev, "ep_queue --> %d\n", ret);
      req->status = 0;
      transfer_complete (ep, req);
   }
}

/*-------------------------------------------------------------------------*/
/* Command/status protocol "set" and "get" operations                      */
/*-------------------------------------------------------------------------*/

/**
 * @brief "Set" work function.
 *
 * Called by work queue when "set" needs to be performed
 */
void
work_do_set(struct work_struct *work)
{
   struct cmd_stat_dev *dev;

   //printk("%s\n",__FUNCTION__);

	dev = container_of(work, struct cmd_stat_dev, set_work);
   do_set(dev->set[0]);
}

/**
 * @brief "Set-confirm" work function.
 *
 * Called by work queue when "set-confirm" needs to be performed
 */
void
work_do_set_confirm(struct work_struct *work)
{
   struct cmd_stat_dev *dev;
	dev = container_of(work, struct cmd_stat_dev, set_confirm_work);
   do_set(dev->set[1]);
}

/**
 * @brief "Set" data receive completion callback.
 *
 * This function is called by the USB gadget code after the data stage of the
 * command/status "set" operation on endpoint 0 completes, i.e. after the FC
 * sends the data value to be validated to the USB peripheral.
 */
static void
set_data_complete(struct usb_ep *ep,       /**< Transfer endpoint */
                  struct usb_request *req) /**< USB request */
{
	struct cmd_stat_dev *dev = ep->driver_data;
   //printk("...set data completed\n");

   switch (dev->cmd) {
   case 0x03: /* SET */
      do_set_in_buffer(dev->set[0]);
      schedule_work(&dev->set_work);
      break;

   case 0x04: /* SET-CONFIRM */
      schedule_work(&dev->set_confirm_work);
      break;

   default:
      usb_ep_set_halt(ep);
      break;
   }
   //up(&dev->sem);
}

/** @brief Process non-standard endpoint 0 setup packet */
static int
proc_vendor_setup(struct cmd_stat_dev *dev,           /**< Device */
                  const struct usb_ctrlrequest *ctrl, /**< Setup packet */
                  int *defer)                         /**< Processing has been deferred */
{
	int len = -EOPNOTSUPP;
	u16 wVarId = le16_to_cpu(ctrl->wValue);
	u16 wVarIdx = le16_to_cpu(ctrl->wIndex);
	u16 wLength = le16_to_cpu(ctrl->wLength);
     //printk("%s:req[%d], varid[%d], index[%d], length[%d]\n", __FUNCTION__,ctrl->bRequest, wVarId, wVarIdx, wLength);

   *defer = 0;
   switch (ctrl->bRequestType) {
   case USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_OTHER:
      switch (ctrl->bRequest) {
      case 0x02:
         /* GET */
         if (wLength > EP0_BUF_SIZE) {
            wLength = EP0_BUF_SIZE;
         }

         *defer = 1;
         //printk("%s: varid[%d], index[%d], length[%d]\n", __FUNCTION__, wVarId, wVarIdx, wLength);
         dev->get_set.var_id = wVarId;
         dev->get_set.var_idx = wVarIdx;
         dev->get_set.len = wLength;
         len = wLength;
         //do_get_in_buffer(dev->get);
         schedule_work(&dev->get_work);
         break;
      }
      break;

   case USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_OTHER:
      switch (ctrl->bRequest) {
      case 0x03:
         /* SET */
      case 0x04:
         /* SET-CONFIRM */
         /* Retrieve the data value from the USB host (FC). This is done by
          * enqueueing an endpoint 0 request to retrieve the data buffer, and
          * letting the callback function decide what to do with the data.
          */
         if (wLength <= EP0_BUF_SIZE) {
            len = wLength;
            dev->cmd = ctrl->bRequest;
            dev->get_set.var_id = wVarId;
            dev->get_set.var_idx = wVarIdx;
            dev->get_set.len = wLength;
            //down(&dev->sem);
            dev->usb.req->complete = set_data_complete;
            //if(wVarId!=31)
            //printk("%s: varid[%d], index[%d], length[%d] ", __FUNCTION__, wVarId, wVarIdx, wLength);
            //do_set_in_buffer(dev->set[0]);
         } else {
            len = -ENOMEM;
         }
         break;

      case 0x05:
         /* CONFIRM */
         len = 0;
         schedule_work(&dev->confirm_work);
         break;

      case 0x06:
         /* CLEAR_CONFIRM */
         len = 0;
         schedule_work(&dev->clr_confirm_work);
         break;
      }
      break;
   }

   return len;
}

/*-------------------------------------------------------------------------*/
/* USB gadget support functions                                            */
/*-------------------------------------------------------------------------*/

/*
 * @brief Responds to endpoint 0 control packets that could not be handled by
 * lower level code.
 *
 * The setup() callback implements all the ep0 functionality that's
 * not handled lower down, in hardware or the hardware driver (like
 * device and endpoint feature flags, and their status).  It's all
 * housekeeping for the gadget function we're implementing.  Most of
 * the work is in config-specific setup.
 */
static int
cmd_stat_setup(struct usb_gadget *gadget,          /**< Device's USB gadget */
               const struct usb_ctrlrequest *ctrl) /**< Setup packet */
{
	struct cmd_stat_dev *dev = get_gadget_data(gadget);
	struct usb_request *req = dev->usb.req;
	int value = -EOPNOTSUPP;
	u16 wIndex = le16_to_cpu(ctrl->wIndex);
	u16 wValue = le16_to_cpu(ctrl->wValue);
	u16 wLength = le16_to_cpu(ctrl->wLength);
   int defer = 0;

     //down(&dev->sem);
	/* Usually this stores reply data in the pre-allocated ep0 buffer,
	 * but config change events will reconfigure hardware.
	 */
	req->zero = 0;
	req->complete = (ctrl->bRequestType & USB_DIR_IN) ?
      tx_data_complete : transfer_complete;

   //printk("%s: request[%d]\n",__FUNCTION__,ctrl->bRequest);

	switch (ctrl->bRequest) {
	case USB_REQ_GET_DESCRIPTOR:
		if (ctrl->bRequestType != USB_DIR_IN) {
			goto unknown;
      }
		switch (wValue >> 8) {
		case USB_DT_DEVICE:
			value = min(wLength, (u16) sizeof device_desc);
			memcpy(req->buf, &device_desc, value);
			break;

		case USB_DT_CONFIG:
			value = create_config_desc(gadget, dev, wValue >> 8, wValue & 0xff);
			if (value >= 0) {
				value = min(wLength, (u16) value);
         }
			break;
		}
		break;

	case USB_REQ_SET_CONFIGURATION:
		if (ctrl->bRequestType != 0) {
			goto unknown;
      }
		if (gadget->a_hnp_support) {
			DBG(dev, "HNP available\n");
      } else if (gadget->a_alt_hnp_support) {
			DBG(dev, "HNP needs a different root port\n");
      } else {
			VDBG(dev, "HNP inactive\n");
      }
		spin_lock(&dev->usb.usb_lock);
		value = set_config(dev, wValue, GFP_ATOMIC);
		spin_unlock(&dev->usb.usb_lock);
		break;

	case USB_REQ_GET_CONFIGURATION:
		if (ctrl->bRequestType != USB_DIR_IN) {
			goto unknown;
      }
		*(u8 *)req->buf = dev->usb.config;
		value = min(wLength, (u16) 1);
		break;

	case USB_REQ_SET_INTERFACE:
		if (ctrl->bRequestType != USB_RECIP_INTERFACE) {
			goto unknown;
      }
		spin_lock (&dev->usb.usb_lock);
		if (dev->usb.config && wIndex == 0 && wValue == 0) {
			u8 config = dev->usb.config;

			/* resets interface configuration, forgets about
			 * previous transaction state (queued bufs, etc)
			 * and re-inits endpoint state (toggle etc)
			 * no response queued, just zero status == success.
			 * if we had more than one interface we couldn't
			 * use this "reset the config" shortcut.
			 */
			reset_config(dev);
			set_config(dev, config, GFP_ATOMIC);
			value = 0;
		}
		spin_unlock (&dev->usb.usb_lock);
		break;

	case USB_REQ_GET_INTERFACE:
		if (ctrl->bRequestType != (USB_DIR_IN|USB_RECIP_INTERFACE)) {
			goto unknown;
      }
		if (!dev->usb.config) {
			break;
      }
		if (wIndex != 0) {
			value = -EDOM;
			break;
		}
		*(u8 *)req->buf = 0;
		value = min(wLength, (u16) 1);
		break;

	default:
      value = proc_vendor_setup(dev, ctrl, &defer);
unknown:
		VDBG (dev,
			"unknown control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			wValue, wIndex, wLength);
	}

	/* Respond with data transfer before status phase? */
   //printk("8:defer[%d], value[%d]\n",defer, value);
	if (!defer && value >= 0) {
		req->length = value;
		req->zero = value < wLength
				&& (value % gadget->ep0->maxpacket) == 0;
		value = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
		if (value < 0) {
			DBG (dev, "ep_queue --> %d\n", value);
              printk("%s: usb ep queue failed[%d]\n", __FUNCTION__, value);
			req->status = 0;
			transfer_complete(gadget->ep0, req);
		}
	}

     //up(&dev->sem);
     
	/* device either stalls (value < 0) or reports success */
	return value;
}

/** @brief Remove USB connection */
static void
cmd_stat_disconnect(struct usb_gadget *gadget) /**< Device's USB gadget */
{
	struct cmd_stat_dev *dev = get_gadget_data(gadget);
	unsigned long flags;

	spin_lock_irqsave(&dev->usb.usb_lock, flags);
	reset_config(dev);

	/* A more significant application might have some non-usb
	 * activities to quiesce here, saving resources like power
	 * or pushing the notification up a network stack.
	 */
	spin_unlock_irqrestore(&dev->usb.usb_lock, flags);

	/* Next we may get setup() calls to enumerate new connections;
	 * or an unbind() during shutdown (including removing module).
	 */
}

/** @brief Tear down the USB gadget */
static void
cmd_stat_unbind(struct usb_gadget *gadget) /**< Device's USB gadget */
{
	struct cmd_stat_dev *dev = get_gadget_data(gadget);

	DBG(dev, "unbind\n");

   cmd_stat_file_exit(dev);

	/* We've already been disconnected ... no i/o is active */
	if (dev->usb.req) {
		free_ep_req(gadget->ep0, dev->usb.req);
   }
	kfree(dev);
	set_gadget_data(gadget, NULL);
}

/** @brief Initialize USB gadget */
static int
cmd_stat_bind(struct usb_gadget *gadget) /**< Device's USB gadget */
{
	struct cmd_stat_dev *dev;
	struct usb_ep *ep;
   int ret;

   printk("%s\n",__FUNCTION__);
   /* Initialize all gadget endpoints to unused */
	list_for_each_entry(ep, &gadget->ep_list, ep_list) {
		ep->driver_data = NULL;
	}

   /* Find trap endpoint */
   ep = NULL;
	list_for_each_entry(ep, &gadget->ep_list, ep_list) {
		if (0 == strcmp(ep->name, "ep1in")) {
			break;
      }
	}
	if (!ep) {
		printk(KERN_ERR "%s: can't autoconfigure on %s\n", shortname, gadget->name);
		return -ENODEV;
	}
	
	/* OK, we made sense of the hardware ... */
	dev = kzalloc(sizeof *dev, GFP_KERNEL);
	if (!dev) {
		return -ENOMEM;
   }

     sema_init(&dev->sem, 1);

	spin_lock_init(&dev->usb.usb_lock);
	dev->usb.gadget = gadget;
	set_gadget_data(gadget, dev);

   /* Claim the trap endpoint */
	dev->usb.trap_use_ep = ep;
	ep->driver_data = ep;

	/* Preallocate control response and buffer */
	dev->usb.req = usb_ep_alloc_request(gadget->ep0, GFP_KERNEL);
	if (!dev->usb.req) {
		goto enomem;
   }
	dev->usb.req->buf = kmalloc(EP0_BUF_SIZE, GFP_KERNEL);
	if (!dev->usb.req->buf) {
		goto enomem;
   }

	device_desc.idVendor = __constant_cpu_to_le16(vendor);
	device_desc.idProduct = __constant_cpu_to_le16(product);
	device_desc.bMaxPacketSize0 = gadget->ep0->maxpacket;

	if (gadget->is_otg) {
		otg_descriptor.bmAttributes |= USB_OTG_HNP,
		cmd_status_config.bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	usb_gadget_set_selfpowered(gadget);

	gadget->ep0->driver_data = dev;

	INFO(dev, "%s, version: " DRIVER_VERSION "\n", longname);
	INFO(dev, "using %s, TRAP EP %s\n", gadget->name, ep->name);

   ret = cmd_stat_file_init(dev);

   if (ret) {
      cmd_stat_unbind(gadget);
   } else {
      INIT_WORK(&dev->get_work, (void *)do_get);
      INIT_WORK(&dev->set_work, (void *)work_do_set);
      INIT_WORK(&dev->set_confirm_work, (void *)work_do_set_confirm);
      INIT_WORK(&dev->confirm_work, (void *)do_confirm);
      INIT_WORK(&dev->clr_confirm_work, (void *)do_clear);
   }

   //printk("%s: end\n",__FUNCTION__);
   return ret;

enomem:
	cmd_stat_unbind(gadget);
	return -ENOMEM;
}

/** @brief Suspends USB */
static void
cmd_stat_suspend(struct usb_gadget *gadget) /**< Device's USB gadget */
{
   struct cmd_stat_dev *dev = get_gadget_data(gadget);
   DBG(dev, "suspend\n");
   reset_config(dev);
}

/** @brief Resumes suspended USB */
static void
cmd_stat_resume(struct usb_gadget *gadget) /**< Device's USB gadget */
{
	struct cmd_stat_dev *dev = get_gadget_data(gadget);
	DBG(dev, "resume\n");
}

/*-------------------------------------------------------------------------*/
/* Device driver initialization and tear down                              */
/*-------------------------------------------------------------------------*/

/** @brief USB gadget driver definition */
static struct usb_gadget_driver cmd_stat_driver = {
	.speed = USB_SPEED_HIGH,
	.function = (char *) longname,
//	.bind = cmd_stat_bind,
	.unbind = cmd_stat_unbind,

	.setup = cmd_stat_setup,
	.disconnect = cmd_stat_disconnect,

	.suspend = cmd_stat_suspend,
	.resume = cmd_stat_resume,

	.driver = {
		.name = (char *) shortname,
		// .shutdown = ...
		// .suspend = ...
		// .resume = ...
	},
};

MODULE_AUTHOR("Evertz Microsystems");
MODULE_LICENSE("Dual BSD/GPL");

/** @brief Device driver init */
static int __init
init(void)
{
   return usb_gadget_probe_driver(&cmd_stat_driver,cmd_stat_bind);
}
module_init(init);

/** @brief Device driver exit */
static void __exit
cleanup(void)
{
	usb_gadget_unregister_driver(&cmd_stat_driver);
}
module_exit(cleanup);
