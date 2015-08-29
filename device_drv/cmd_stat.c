
#define VERBOSE_DEBUG
#define DEBUG


#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/list.h>

#include "g_cmd_stat.h"
#include "gadget_chips.h"


/*-------------------------------------------------------------------------*/

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "composite.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"

#include "f_sourcesink.c"
#include "f_rndis.c"

/*-------------------------------------------------------------------------*/



#if 0
/*-------------------------------------------------------------------------*/

/* REVISIT there must be a better way than having two sets
 * of debug calls ...
 */

#undef DBG
#undef VDBG
#undef ERROR
#undef INFO

#define xprintk(d, level, fmt, args...) \
        printk(level "%s: " fmt , (d)->gadget->name , ## args)

#ifdef DEBUG
#undef DEBUG
#define DBG(dev, fmt, args...) \
        xprintk(dev , KERN_DEBUG , fmt , ## args)
#else
#define DBG(dev, fmt, args...) \
        do { } while (0)
#endif /* DEBUG */

#ifdef VERBOSE_DEBUG
#define VDBG    DBG
#else
#define VDBG(dev, fmt, args...) \
        do { } while (0)
#endif /* DEBUG */

#define ERROR(dev, fmt, args...) \
        xprintk(dev , KERN_ERR , fmt , ## args)
#define INFO(dev, fmt, args...) \
        xprintk(dev , KERN_INFO , fmt , ## args)

/*-------------------------------------------------------------------------*/
#endif


#define DRIVER_VERSION		"Fynn 2014"

static const char longname[] = "cmd_stat";

unsigned buflen = 4096;
module_param(buflen, uint, 0);

static int config_default = 0;
module_param(config_default, int, S_IRUGO|S_IWUSR);

/*-------------------------------------------------------------------------*/

/* Thanks to NetChip Technologies for donating this product ID.
 *
 * DO NOT REUSE THESE IDs with a protocol-incompatible driver!!  Ever!!
 * Instead:  allocate your own, using normal USB-IF procedures.
 */

#define DRIVER_VENDOR_NUM	0x0525		/* NetChip */
#define DRIVER_PRODUCT_NUM	0xa4a0		/* Linux-USB "Gadget Zero" */

#define DRIVER_VENDOR_NUM       0x0836
#define DRIVER_PRODUCT_NUM      0xfefe       


#define DEFAULT_AUTORESUME      0

/* If the optional "autoresume" mode is enabled, it provides good
 * functional coverage for the "USBCV" test harness from USB-IF.
 * It's always set if OTG mode is enabled.
 */
unsigned autoresume = DEFAULT_AUTORESUME;
module_param(autoresume, uint, S_IRUGO);
MODULE_PARM_DESC(autoresume, "zero, or seconds before remote wakeup");

/*-------------------------------------------------------------------------*/

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		cpu_to_le16(0x0200),
	.bDeviceClass =		USB_CLASS_VENDOR_SPEC,

	//.idVendor =		cpu_to_le16(DRIVER_VENDOR_NUM),
	//.idProduct =		cpu_to_le16(DRIVER_PRODUCT_NUM),
	.bNumConfigurations =	2,
};

#ifdef CONFIG_USB_OTG
static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/* REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};
#endif

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

static char manufacturer[50];

/* default serial number takes at least two packets */
static char serial[] = "0123456789.0123456789.0123456789";

static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer,
	[STRING_PRODUCT_IDX].s = longname,
	[STRING_SERIAL_IDX].s = serial,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};


/*-------------------------------------------------------------------------*/

static int __init do_config(struct usb_configuration *c)
{
	int status = 0;

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	status = rndis_bind_config(c);

	return status;
}

static struct usb_configuration config_driver = {
	.label			= "Gadget Notify",
	.bConfigurationValue	= 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

/*-------------------------------------------------------------------------*/

struct usb_request *alloc_ep_req(struct usb_ep *ep)
{
	struct usb_request	*req;

	req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (req) {
		req->length = buflen;
		req->buf = kmalloc(buflen, GFP_ATOMIC);
		if (!req->buf) {
			usb_ep_free_request(ep, req);
			req = NULL;
		}
	}
	return req;
}

void free_ep_req(struct usb_ep *ep, struct usb_request *req)
{
	kfree(req->buf);
	usb_ep_free_request(ep, req);
}

static void disable_ep(struct usb_composite_dev *cdev, struct usb_ep *ep)
{
	int			value;

	if (ep->driver_data) {
		value = usb_ep_disable(ep);
		if (value < 0)
			DBG(cdev, "disable %s --> %d\n",
					ep->name, value);
		ep->driver_data = NULL;
	}
}

void disable_endpoints(struct usb_composite_dev *cdev,
		struct usb_ep *in, struct usb_ep *out)
{
	disable_ep(cdev, in);
	disable_ep(cdev, out);
}

/*-------------------------------------------------------------------------*/

static struct timer_list	autoresume_timer;

static void zero_autoresume(unsigned long _c)
{
	struct usb_composite_dev	*cdev = (void *)_c;
	struct usb_gadget		*g = cdev->gadget;

	/* unconfigured devices can't issue wakeups */
	if (!cdev->config)
		return;

	/* Normally the host would be woken up for something
	 * more significant than just a timer firing; likely
	 * because of some direct user request.
	 */
	if (g->speed != USB_SPEED_UNKNOWN) {
		int status = usb_gadget_wakeup(g);
		INFO(cdev, "%s --> %d\n", __func__, status);
	}
}

static void zero_suspend(struct usb_composite_dev *cdev)
{
	if (cdev->gadget->speed == USB_SPEED_UNKNOWN)
		return;

	if (autoresume) {
		mod_timer(&autoresume_timer, jiffies + (HZ * autoresume));
		DBG(cdev, "suspend, wakeup in %d seconds\n", autoresume);
	} else
		DBG(cdev, "%s\n", __func__);
}

static void zero_resume(struct usb_composite_dev *cdev)
{
	DBG(cdev, "%s\n", __func__);
	del_timer(&autoresume_timer);
}

/*-------------------------------------------------------------------------*/

static int __init zero_bind(struct usb_composite_dev *cdev)
{
	int			gcnum;
	struct usb_gadget	*gadget = cdev->gadget;
	int			id;
	int			status;

	DBG(cdev, "%s\n", __func__);

/*-------------------------------------------------------------------------*/
	status = rndis_init_setup(cdev->gadget);
	if (status < 0)
		return status;
/*-------------------------------------------------------------------------*/


	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	setup_timer(&autoresume_timer, zero_autoresume, (unsigned long) cdev);

	/* Register primary, then secondary configuration.  Note that
	 * SH3 only allows one config...
	 */
	DBG(cdev, "config_default=%d\n",config_default);
	switch (config_default){
		case 0:
                        device_desc.idVendor = __constant_cpu_to_le16(DRIVER_VENDOR_NUM);
                        device_desc.idProduct = __constant_cpu_to_le16(DRIVER_PRODUCT_NUM);	
			status = usb_add_config(cdev, &config_driver, do_config);
			if (status < 0) return status;
			//sourcesink_add(cdev, autoresume != 0);
			break;
		case 1:
                        device_desc.idVendor = __constant_cpu_to_le16(DRIVER_VENDOR_NUM);
                        device_desc.idProduct = __constant_cpu_to_le16(DRIVER_PRODUCT_NUM);
			sourcesink_add(cdev, autoresume != 0);
			break;
		default:
			break;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = cpu_to_le16(0x9999);
	}


	INFO(cdev, "%s, version: " DRIVER_VERSION "\n", longname);

	snprintf(manufacturer, sizeof manufacturer, "%s %s with %s",
		init_utsname()->sysname, init_utsname()->release,
		gadget->name);

	return 0;
}

static int zero_unbind(struct usb_composite_dev *cdev)
{
	del_timer_sync(&autoresume_timer);
	rndis_cleanup();
	return 0;
}

static struct usb_composite_driver zero_driver = {
	.name		= "zero",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= zero_unbind,
	.suspend	= zero_suspend,
	.resume		= zero_resume,
};

MODULE_AUTHOR("David Brownell");
MODULE_LICENSE("GPL");

static int __init init(void)
{
	return usb_composite_probe(&zero_driver, zero_bind);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&zero_driver);
}
module_exit(cleanup);
