/*-------------------------------------------------------------------------*/
/* Copyright 2007 Evertz Microsystems Ltd. All rights reserved             */
/*-------------------------------------------------------------------------*/
/**
 * @file
 * @brief USB functions supported by the command/status protocol.
 *
 * $Id: cmd_stat.h,v 1.2.8.2 2010-09-15 22:33:52 kjarrah Exp $
 *
 * This file contains functions used by the command/status protocol to access
 * USB.
 */
#ifndef CMD_STAT_H
#define CMD_STAT_H

struct usb_request;
struct usb_ep;

/**
 * @brief Endpoint 0 buffer size.
 *
 * The buffer must be big enough to hold our biggest descriptor
 */
#define EP0_BUF_SIZE 512

struct usb_request *alloc_ep_req(struct usb_ep *ep, unsigned length);
void free_ep_req(struct usb_ep *ep, struct usb_request *req);
void transfer_complete(struct usb_ep *ep, struct usb_request *req);
void tx_data_complete(struct usb_ep *ep, struct usb_request *req);

#endif
