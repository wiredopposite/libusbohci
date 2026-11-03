/**************************************************************************//**
 * @file     usb_core.c
 * @version  V1.10
 * $Revision: 11 $
 * $Date: 14/10/03 1:54p $
 * @brief   USB Host library core.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "usb.h"
#include "hub.h"


/// @cond HIDDEN_SYMBOLS

static UDEV_DRV_T *  _drivers[MAX_UDEV_DRIVER];

static CONN_FUNC  *g_conn_func, *g_disconn_func;


static void enum_init(void);
static void enum_abort(void);

/// @endcond HIDDEN_SYMBOLS


/**
  * @brief       Initialize N9H30 USB Host controller and USB stack.
  *
  * @return      None.
  */
static uint8_t usb_core_initialised = 0;
void  usbh_core_init()
{
    if (usb_core_initialised)
        return;
    usb_core_initialised = 1;

    memset(_drivers, 0, sizeof(_drivers));

    g_conn_func = NULL;
    g_disconn_func = NULL;

    usbh_hub_init();
    usbh_memory_init();
    enum_init();

    for (int i = 0; i < OHCI_HC_CNT; i++)
    {
        HC_DRV_T  *hc = ohci_get_hc(i);

        usbh_ohci_irq_init(i);
        if (hc->init(hc) < 0)
            USB_error("OHCI %d init failed!\n", i);
    }
}

void  usbh_core_start(void)
{
    for (int i = 0; i < OHCI_HC_CNT; i++)
    {
        HC_DRV_T  *hc = ohci_get_hc(i);
        hc->rthub_start(hc);
    }
}

void  usbh_core_deinit()
{
    if (!usb_core_initialised)
        return;

    enum_abort();

    /* Children go with their hub */
    while (g_udev_list != NULL)
        disconnect_device(g_udev_list);

    memset(_drivers, 0, sizeof(_drivers));

    g_conn_func = NULL;
    g_disconn_func = NULL;

    for (int i = 0; i < OHCI_HC_CNT; i++)
    {
        HC_DRV_T  *hc = ohci_get_hc(i);

        hc->rthub_port_reset_cancel(hc);
        hc->shutdown(hc);
        usbh_ohci_irq_deinit(i);
    }

    usbh_memory_deinit();
    usb_core_initialised = 0;
}

/**
  * @brief    Install device connect and disconnect callback function.
  *
  * @param[in]  conn_func       Device connect callback function.
  * @param[in]  disconn_func    Device disconnect callback function.
  * @return      None.
  */
void usbh_install_conn_callback(CONN_FUNC *conn_func, CONN_FUNC *disconn_func)
{
    g_conn_func = conn_func;
    g_disconn_func = disconn_func;
}


/// @cond HIDDEN_SYMBOLS

/**
  * @brief    Register an USB device class or vendor driver to lightweight USB stack.
  *           All USB device drivers must be registered before usbh_core_start() called.
  * @param[in]  udrv   The USB device driver to be registered.
  * @return   Driver registration success or not.
  * @retval   0  Success
  * @retval   USBH_ERR_MEMORY_OUT   Registered drivers have reached MAX_UDEV_DRIVER limitation.
  */
int  usbh_register_driver(UDEV_DRV_T *udrv)
{
    int   i;

    for (i = 0; i < MAX_UDEV_DRIVER; i++)
    {
        if (_drivers[i] == udrv)
            return 0;                  /* already registered, do nothing */

        if (_drivers[i] == NULL)
        {
            _drivers[i] = udrv;        /* register this driver */
            return 0;
        }
    }
    return USBH_ERR_MEMORY_OUT;        /* reached MAX_UDEV_DRIVER limitation, aborted */
}


/*--------------------------------------------------------------------------*/
/*   Control transfers                                                      */
/*--------------------------------------------------------------------------*/

typedef struct ctrl_req_t
{
    UTR_T           utr;
    USBH_TIMER_T    timer;
    USBH_CTRL_FUNC  func;
    void            *context;
    int             timed_out;
} CTRL_REQ_T;

static void ctrl_req_timeout(void *context)
{
    CTRL_REQ_T  *req = (CTRL_REQ_T *)context;

    /* Pulling the ED completes it as aborted, with the rest queued on it */
    req->timed_out = 1;
    usbh_quit_utr(&req->utr);
}

static void ctrl_req_done(UTR_T *utr)
{
    CTRL_REQ_T  *req = (CTRL_REQ_T *)utr->context;
    int         status = utr->status;

    usbh_timer_cancel(&req->timer);
    if (req->timed_out)
        status = USBH_ERR_TIMEOUT;
    if (req->func)
        req->func(utr->udev, status, utr->xfer_len, req->context);
    usbh_free_mem(req, sizeof(*req));
}

int usbh_ctrl_xfer_async(UDEV_T *udev, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
                         uint16_t wLength, uint8_t *buff, uint32_t timeout_ms, USBH_CTRL_FUNC func, void *context)
{
    CTRL_REQ_T  *req;
    UTR_T       *utr;
    int         ret;

    if ((wLength > 0) && (buff == NULL))
        return USBH_ERR_INVALID_PARAM;

    req = (CTRL_REQ_T *)usbh_alloc_mem(sizeof(*req));
    if (req == NULL)
        return USBH_ERR_MEMORY_OUT;

    req->func = func;
    req->context = context;
    usbh_timer_init(&req->timer, ctrl_req_timeout, req);

    utr = &req->utr;
    utr->udev = udev;
    utr->setup.bmRequestType = bmRequestType;
    utr->setup.bRequest   = bRequest;
    utr->setup.wValue     = wValue;
    utr->setup.wIndex     = wIndex;
    utr->setup.wLength    = wLength;
    utr->buff = buff;
    utr->data_len = wLength;
    utr->context = req;
    utr->func = ctrl_req_done;

    ret = udev->hc_driver->ctrl_xfer(utr);
    if (ret < 0)
    {
        usbh_free_mem(req, sizeof(*req));
        return ret;
    }
    if (timeout_ms)
        usbh_timer_start(&req->timer, timeout_ms);
    return 0;
}


/**
  * @brief    Execute a bulk transfer request. This function will return immediatedly after
  *           issued the bulk transfer. USB stack will later call back utr->func() once the bulk
  *           transfer was done or aborted.
  * @param[in]  utr    The bulk transfer request.
  * @retval   0     Transfer success
  * @retval   < 0   Failed. Refer to error code definitions.
  */
int usbh_bulk_xfer(UTR_T *utr)
{
    return utr->udev->hc_driver->bulk_xfer(utr);
}

/**
  * @brief    Execute an interrupt transfer request. This function will return immediatedly after
  *           issued the interrupt transfer. USB stack will later call back utr->func() once the
  *           interrupt transfer was done or aborted.
  * @param[in]  utr    The interrupt transfer request.
  * @retval   0     Transfer success
  * @retval   < 0   Failed. Refer to error code definitions.
  */
int usbh_int_xfer(UTR_T *utr)
{
    return utr->udev->hc_driver->int_xfer(utr);
}

/**
  * @brief    Execute an isochronous transfer request. This function will return immediatedly after
  *           issued the isochronous transfer. USB stack will later call back utr->func() once the
  *           isochronous transfer was done or aborted.
  * @param[in]  utr    The isochronous transfer request.
  * @retval   0     Transfer success
  * @retval   < 0   Failed. Refer to error code definitions.
  */
int usbh_iso_xfer(UTR_T *utr)
{
    if (utr->udev->hc_driver == NULL)
    {
        sysprintf("hc_driver - 0x%x\n", (int)utr->udev->hc_driver);
        return -1;
    }
    if (utr->udev->hc_driver->iso_xfer == NULL)
    {
        sysprintf("iso_xfer - 0x%x\n", (int)utr->udev->hc_driver->iso_xfer);
        return -1;
    }
    return utr->udev->hc_driver->iso_xfer(utr);
}

/**
  * @brief    Force to quit an UTR transfer.
  * @param[in]  utr    The UTR transfer to be quit.
  * @retval   0     Transfer success
  * @retval   < 0   Failed. Refer to error code definitions.
  */
int usbh_quit_utr(UTR_T *utr)
{
    if (!utr || !utr->udev)
        return USBH_ERR_NOT_FOUND;

    return utr->udev->hc_driver->quit_xfer(utr->udev->hc_driver, utr, NULL);
}


/**
  * @brief    Force to quit an endpoint transfer.
  * @param[in]  udev     The USB device.
  * @param[in]  ep       The endpoint to be quit.
  * @retval   0     Transfer success
  * @retval   < 0   Failed. Refer to error code definitions.
  */
int usbh_quit_xfer(UDEV_T *udev, EP_INFO_T *ep)
{
    return udev->hc_driver->quit_xfer(udev->hc_driver, NULL, ep);
}


typedef struct set_iface_req_t
{
    IFACE_T         *iface;
    ALT_IFACE_T     *aif;
    USBH_CTRL_FUNC  func;
    void            *context;
} SET_IFACE_REQ_T;

static void set_interface_done(UDEV_T *udev, int status, uint32_t xfer_len, void *context)
{
    SET_IFACE_REQ_T  *req = (SET_IFACE_REQ_T *)context;

    if (status == 0)
        req->iface->aif = req->aif;         /* change active alternative setting          */
    if (req->func)
        req->func(udev, status, xfer_len, req->context);
    usbh_free_mem(req, sizeof(*req));
}

/**
 *  @brief    Select an alternate setting for the specified interface.
 *  @param[in]  iface        The interface
 *  @param[in]  alt_setting  Desired alternative setting
 *  @retval   0  Submitted, func gets the result
 *  @retval   < 0   Failed. Refer to error code definitions.
 */
int usbh_set_interface_async(IFACE_T *iface, uint16_t alt_setting, USBH_CTRL_FUNC func, void *context)
{
    SET_IFACE_REQ_T  *req;
    ALT_IFACE_T      *aif = NULL;
    int              i, ret;

    for (i = 0; i < iface->num_alt; i++)
    {
        if (iface->alt[i].ifd->bAlternateSetting == alt_setting)
        {
            aif = &iface->alt[i];
            break;
        }
    }
    if (aif == NULL)
        return USBH_ERR_NOT_FOUND;          /* cannot find desired alternative setting    */

    req = (SET_IFACE_REQ_T *)usbh_alloc_mem(sizeof(*req));
    if (req == NULL)
        return USBH_ERR_MEMORY_OUT;
    req->iface = iface;
    req->aif = aif;
    req->func = func;
    req->context = context;

    ret = usbh_ctrl_xfer_async(iface->udev, REQ_TYPE_OUT | REQ_TYPE_STD_DEV | REQ_TYPE_TO_IFACE,
                               USB_REQ_SET_INTERFACE, alt_setting, iface->if_num, 0,
                               NULL, 500, set_interface_done, req);
    if (ret < 0)
        usbh_free_mem(req, sizeof(*req));
    return ret;
}


void  dump_device_descriptor(DESC_DEV_T *desc)
{
    USB_debug("\n[Device Descriptor]\n");
    USB_debug("----------------------------------------------\n");
    USB_debug("  Length              = %2d\n",  desc->bLength);
    USB_debug("  DescriptorType      = 0x%02x\n", desc->bDescriptorType);
    USB_debug("  USB version         = %x.%02x\n",
              desc->bcdUSB >> 8, desc->bcdUSB & 0xff);
    USB_debug("  Vendor:Product      = %04x:%04x\n",
              desc->idVendor, desc->idProduct);
    USB_debug("  MaxPacketSize0      = %d\n",   desc->bMaxPacketSize0);
    USB_debug("  NumConfigurations   = %d\n",   desc->bNumConfigurations);
    USB_debug("  Device version      = %x.%02x\n",
              desc->bcdDevice >> 8, desc->bcdDevice & 0xff);
    USB_debug("  Device Class:SubClass:Protocol = %02x:%02x:%02x\n",
              desc->bDeviceClass, desc->bDeviceSubClass, desc->bDeviceProtocol);
}

void usbh_dump_interface_descriptor(DESC_IF_T *if_desc)
{
    USB_debug("\n    [Interface Descriptor]\n");
    USB_debug("    ----------------------------------------------\n");
    USB_debug("      Length              = %2d\n",  if_desc->bLength);
    USB_debug("      DescriptorType      = %02x\n", if_desc->bDescriptorType);
    USB_debug("      bInterfaceNumber    = %d\n", if_desc->bInterfaceNumber);
    USB_debug("      bAlternateSetting   = %d\n", if_desc->bAlternateSetting);
    USB_debug("      bNumEndpoints       = %d\n", if_desc->bNumEndpoints);
    USB_debug("      bInterfaceClass     = 0x%02x\n", if_desc->bInterfaceClass);
    USB_debug("      bInterfaceSubClass  = 0x%02x\n", if_desc->bInterfaceSubClass);
    USB_debug("      bInterfaceProtocol  = 0x%02x\n", if_desc->bInterfaceProtocol);
    USB_debug("      iInterface          = %d\n", if_desc->iInterface);
}

void usbh_dump_endpoint_descriptor(DESC_EP_T *ep_desc)
{
    USB_debug("\n        [Endpoint Descriptor]\n");
    USB_debug("        ----------------------------------------------\n");
    USB_debug("          Length              = %2d\n",  ep_desc->bLength);
    USB_debug("          DescriptorType      = %02x\n", ep_desc->bDescriptorType);
    USB_debug("          bEndpointAddress    = 0x%02x\n", ep_desc->bEndpointAddress);
    USB_debug("          bmAttributes        = 0x%02x\n", ep_desc->bmAttributes);
    USB_debug("          wMaxPacketSize      = %d\n", ep_desc->wMaxPacketSize);
    USB_debug("          bInterval           = %d\n", ep_desc->bInterval);
    USB_debug("          bRefresh            = %d\n", ep_desc->bRefresh);
    USB_debug("          bSynchAddress       = %d\n", ep_desc->bSynchAddress);
}

void  dump_config_descriptor(DESC_CONF_T *desc)
{
    uint8_t     *bptr = (uint8_t *)desc;
    DESC_HDR_T  *hdr;
    int         tlen = desc->wTotalLength;

    while (tlen > 0)
    {
        switch (bptr[1])
        {
        case USB_DT_CONFIGURATION:
            USB_debug("\n[Configuration Descriptor]\n");
            USB_debug("----------------------------------------------\n");
            USB_debug("  Length              = %2d\n",  desc->bLength);
            USB_debug("  DescriptorType      = %02x\n", desc->bDescriptorType);
            USB_debug("  wTotalLength        = %2d\n", desc->wTotalLength);
            USB_debug("  bNumInterfaces      = %d\n", desc->bNumInterfaces);
            USB_debug("  bConfigurationValue = %d\n", desc->bConfigurationValue);
            USB_debug("  iConfiguration      = %d\n", desc->iConfiguration);
            USB_debug("  bmAttributes        = 0x%02x\n", desc->bmAttributes);
            USB_debug("  MaxPower            = %d\n", desc->MaxPower);
            break;

        case USB_DT_INTERFACE:
            usbh_dump_interface_descriptor((DESC_IF_T *)bptr);
            break;

        case USB_DT_ENDPOINT:
            usbh_dump_endpoint_descriptor((DESC_EP_T *)bptr);
            break;

        default:
            hdr = (DESC_HDR_T *)bptr;
            USB_debug("\n!![Unknown Descriptor]\n");
            USB_debug("----------------------------------------------\n");
            USB_debug("Length              = %2d\n",  hdr->bLength);
            USB_debug("DescriptorType      = %02x\n", hdr->bDescriptorType);
            break;
        }
        if (bptr[0] == 0)
            break;
        tlen -= bptr[0];
        bptr += bptr[0];
    }
}

static int  usbh_parse_endpoint(ALT_IFACE_T *alt, int ep_idx, uint8_t *desc_buff, int len)
{
    DESC_EP_T    *ep_desc;
    int          parsed_len = 0;
    int          pksz;

    while (len > 0)
    {
        ep_desc = (DESC_EP_T *)desc_buff;

        if ((len < ep_desc->bLength) || (ep_desc->bLength < 2))
        {
            USB_error("ERR DESCRIPTOR EP LEN [0x%X %d]\n", ep_desc->bDescriptorType, ep_desc->bLength);
            return USBH_ERR_DESCRIPTOR;
        }

        if (ep_desc->bDescriptorType == USB_DT_ENDPOINT)
            break;     /* endpoint descriptor found */

        /* unrecognized descriptor */
        USB_vdebug("ignore descriptor 0x%X %d\n", ep_desc->bDescriptorType, ep_desc->bLength);
        desc_buff += ep_desc->bLength;
        parsed_len += ep_desc->bLength;
        len -= ep_desc->bLength;
    }

    USB_vdebug("Descriptor Found - Alt: %d, Endpoint 0x%x, remaining len: %d\n", alt->ifd->bAlternateSetting, ep_desc->bEndpointAddress, len);

    alt->ep[ep_idx].bEndpointAddress = ep_desc->bEndpointAddress;
    alt->ep[ep_idx].bmAttributes     = ep_desc->bmAttributes;
    alt->ep[ep_idx].bInterval        = ep_desc->bInterval;
    pksz = ep_desc->wMaxPacketSize;
    pksz = (pksz & 0x07ff) * (1 + ((pksz >> 11) & 3));
    alt->ep[ep_idx].wMaxPacketSize   = pksz;
    alt->ep[ep_idx].hw_pipe          = NULL;

    return parsed_len + ep_desc->bLength;
}

/**
  * @brief    Parse interface descriptor.
  * @param[in]  udev    The USB device.
  * @param[in]  desc_buff   Descriptor buffer
  * @param[in]  len         Remaining length of descriptors in buffer.
  * @return   Driver registration success or not.
  * @retval   0  Success
  * @retval   USBH_ERR_MEMORY_OUT   Registered drivers have reached MAX_UDEV_DRIVER limitation.
  */
static int  usbh_parse_interface(UDEV_T *udev, uint8_t *desc_buff, int len)
{
    int         i, matched, parsed_len = 0;
    DESC_HDR_T  *hdr;
    DESC_IF_T   *if_desc;
    IFACE_T     *iface = NULL;
    int         ret;

    iface = usbh_alloc_mem(sizeof(*iface)); /* create an interface                        */
    if (iface == NULL)
        return USBH_ERR_MEMORY_OUT;
    iface->udev = udev;
    iface->aif = &iface->alt[0];            /* Default active interface should be the
                                               first found alternative interface          */
    iface->if_num = ((DESC_IF_T *)desc_buff)->bInterfaceNumber;

    while (len > 0)
    {
        /*--------------------------------------------------------------------------------*/
        /* Find the first/next interface descriptor                                       */
        /*--------------------------------------------------------------------------------*/
        if_desc = (DESC_IF_T *)desc_buff;

        if (if_desc->bDescriptorType != USB_DT_INTERFACE)
        {
            desc_buff += if_desc->bLength;
            parsed_len += if_desc->bLength;
            len -= if_desc->bLength;
            continue;
        }

        if (if_desc->bInterfaceNumber != iface->if_num)
        {
            goto parse_done;
        }

        if (if_desc->bNumEndpoints > MAX_EP_PER_IFACE)
        {
            USB_error("IF EP LIMITE %d\n", if_desc->bNumEndpoints);
            ret = USBH_ERR_IF_EP_LIMIT;
            goto err_out;
        }

        /* Step over the interface descriptor */
        desc_buff += if_desc->bLength;
        parsed_len += if_desc->bLength;
        len -= if_desc->bLength;
        USB_vdebug("Descriptor Found - Interface %d, Alt: %d, num_alt:%d, remaining len: %d\n", if_desc->bInterfaceNumber, if_desc->bAlternateSetting, iface->num_alt, len);

        /*--------------------------------------------------------------------------------*/
        /* Add to alternative interface list                                              */
        /*--------------------------------------------------------------------------------*/
        if (iface->num_alt >= MAX_ALT_PER_IFACE)
        {
            ret = USBH_ERR_IF_ALT_LIMIT;
            goto err_out;
        }

        /*--------------------------------------------------------------------------------*/
        /* Find the next alternative interface or endpoint descriptor                     */
        /*--------------------------------------------------------------------------------*/
        while (len > 0)
        {
            hdr = (DESC_HDR_T *)desc_buff;

            if ((len < hdr->bLength) || (hdr->bLength < 2))
            {
                USB_error("ERR DESCRIPTOR IF LEN [0x%X %d]\n", hdr->bDescriptorType, hdr->bLength);
                ret = USBH_ERR_DESCRIPTOR;
                goto err_out;
            }

            if (hdr->bDescriptorType == USB_DT_CONFIGURATION)
                goto parse_done;            /* is other configuration, parsing completed  */

            if ((hdr->bDescriptorType == USB_DT_INTERFACE) || (hdr->bDescriptorType == USB_DT_ENDPOINT))
                break;                      /* the first endpoint descriptor found        */

            /* unrecognized descriptor */
            USB_vdebug("ignore descriptor 0x%X %d\n", hdr->bDescriptorType, hdr->bLength);
            desc_buff += hdr->bLength;
            parsed_len += hdr->bLength;
            len -= hdr->bLength;
        }

        iface->alt[iface->num_alt].ifd = if_desc;
        iface->num_alt++;

        if (len == 0)
            goto parse_done;

        if (hdr->bDescriptorType == USB_DT_INTERFACE)
            continue;                       /* is the next interface descriptor           */

        USB_vdebug("Finding %d endpoints of interface %d, alt %d...\n", if_desc->bNumEndpoints, if_desc->bInterfaceNumber, if_desc->bAlternateSetting);

        /* parsign all endpoint descriptors */
        for (i = 0; i < if_desc->bNumEndpoints; i++)
        {
            ret = usbh_parse_endpoint(&iface->alt[iface->num_alt-1], i, desc_buff, len);
            if (ret < 0)
                goto err_out;

            desc_buff += ret;
            parsed_len += ret;
            len -= ret;
            USB_vdebug("EP parse remaining %d\n", len);
        }
    }

parse_done:

    /*
     *  Probing all registered USB device drivers to find a matched driver.
     */
    matched = 0;
    for (i = 0; i < MAX_UDEV_DRIVER; i++)
    {
        if ((_drivers[i] != NULL) && (_drivers[i]->probe(iface) == 0))
        {
            matched = 1;
            break;
        }
    }

    if (matched)
    {
        iface->driver = _drivers[i];        /* have a driver now */
        iface->next = NULL;

        /* Added this interface to USB device interface list */
        if (udev->iface_list == NULL)
            udev->iface_list = iface;
        else
        {
            iface->next = udev->iface_list;
            udev->iface_list = iface;
        }
    }
    else
    {
        usbh_free_mem(iface, sizeof(*iface));
        iface = NULL;
    }

    return parsed_len;

err_out:
    usbh_free_mem(iface, sizeof(*iface));
    return ret;
}


static int  usbh_parse_configuration(UDEV_T *udev, uint8_t *desc_buff)
{
    DESC_CONF_T  *config = (DESC_CONF_T *)desc_buff;
    DESC_HDR_T   *hdr;
    int    i, ret, len;

    len = config->wTotalLength;

    desc_buff += config->bLength;
    len -= config->bLength;

    USB_vdebug("Parsing CONFIG =>\n");

    for (i = 0; i < config->bNumInterfaces; i++)
    {
        /*
         *  find the next interface descriptor
         */
        while (len >= sizeof(DESC_HDR_T))
        {
            hdr = (DESC_HDR_T *)desc_buff;

            if ((hdr->bLength > len) || (hdr->bLength < 2))
            {
                USB_error("ERR DESCRIPTOR CONFIG [%d]\n", hdr->bLength);
                return USBH_ERR_DESCRIPTOR;
            }

            if (hdr->bDescriptorType == USB_DT_INTERFACE)
                break;

            USB_debug("ignore descriptor 0x%X %d\n", hdr->bDescriptorType, hdr->bLength);

            desc_buff += hdr->bLength;
            len -= hdr->bLength;
        }

        ret = usbh_parse_interface(udev, desc_buff, len);
        if (ret < 0)
            return ret;

        desc_buff += ret;
        len -= ret;
        USB_vdebug("IFACE parse remaining %d\n", len);
    }

    if (len > 0)
    {
        USB_debug("ERR DESCRIPTOR CONFIG LEN %d\n", len);
        return USBH_ERR_DESCRIPTOR;
    }
    return len;
}


/*--------------------------------------------------------------------------*/
/*   Enumeration                                                            */
/*--------------------------------------------------------------------------*/

/*
 *  Ports that connected are queued and enumerated one at a time, since a
 *  new device answers at address 0 until it's given one. Each step starts
 *  from the completion of the one before or from a timer, and callbacks carry
 *  a generation so ones from an attempt that was given up on are dropped.
 */

#define ENUM_QUEUE_LEN      16
#define ENUM_BUFF_SIZE      64
#define ENUM_RETRY          2
#define ENUM_XFER_TIMEOUT   500

enum
{
    ES_IDLE = 0,
    ES_DEBOUNCE,
    ES_RESET,
    ES_RESET_RECOVERY,
    ES_GET_DESC_SHORT,
    ES_SET_ADDRESS,
    ES_ADDRESS_RECOVERY,
    ES_GET_DESC,
    ES_GET_CONF_SHORT,
    ES_GET_CONF,
    ES_SET_CONF,
};

static struct
{
    HC_DRV_T          *hc;
    struct hub_dev_t  *hub;
    int               port;
} _enum_queue[ENUM_QUEUE_LEN];
static int _enum_queue_cnt;

static struct
{
    int             state;
    uint32_t        gen;
    HC_DRV_T        *hc;
    HUB_DEV_T       *hub;
    int             port;
    int             retry;
    int             speed;
    int             dev_num;
    UDEV_T          *udev;
    uint8_t         *buff;
    USBH_TIMER_T    timer;
} _enum;

static void * enum_token(void)
{
    return (void *)(uintptr_t)_enum.gen;
}

static int enum_current(void *token)
{
    return (_enum.state != ES_IDLE) && ((uint32_t)(uintptr_t)token == _enum.gen);
}

static HC_DRV_T * hub_hc(HUB_DEV_T *hub)
{
    return hub->iface->udev->hc_driver;
}

/* A root hub port is told apart by its controller, a hub's by the hub */
static UDEV_T * find_child(HC_DRV_T *hc, HUB_DEV_T *hub, int port)
{
    UDEV_T  *udev;

    for (udev = g_udev_list; udev != NULL; udev = udev->next)
    {
        if ((udev->hc_driver == hc) && (udev->parent == hub) && (udev->port_num == port) &&
                (udev != _enum.udev))
            return udev;
    }
    return NULL;
}

UDEV_T * usbh_find_child(HUB_DEV_T *hub, int port)
{
    return find_child(hub_hc(hub), hub, port);
}

/* Frees the device being enumerated, with anything still queued on it */
static void enum_release_device(void)
{
    UDEV_T  *udev = _enum.udev;

    _enum.udev = NULL;
    if (_enum.dev_num)
    {
        free_dev_address(_enum.dev_num);
        _enum.dev_num = 0;
    }
    if (udev != NULL)
    {
        usbh_quit_xfer(udev, &udev->ep0);
        free_device(udev);
    }
}

static void enum_stop_port(void)
{
    usbh_timer_cancel(&_enum.timer);
    if (_enum.state == ES_RESET)
    {
        if (_enum.hub == NULL)
            _enum.hc->rthub_port_reset_cancel(_enum.hc);
        else
            usbh_hub_port_reset_cancel(_enum.hub);
    }
}

static void enum_start_next(void)
{
    if ((_enum.state != ES_IDLE) || (_enum_queue_cnt == 0))
        return;

    _enum.hc = _enum_queue[0].hc;
    _enum.hub = _enum_queue[0].hub;
    _enum.port = _enum_queue[0].port;
    _enum_queue_cnt--;
    memmove(&_enum_queue[0], &_enum_queue[1], _enum_queue_cnt * sizeof(_enum_queue[0]));

    _enum.gen++;
    _enum.retry = 0;
    _enum.state = ES_DEBOUNCE;
    usbh_timer_start(&_enum.timer, 100);
}

static void enum_finish(void)
{
    _enum.gen++;
    _enum.state = ES_IDLE;
    _enum.udev = NULL;
    _enum.dev_num = 0;
    enum_start_next();
}

static void enum_fail(int status)
{
    USB_debug("Enumeration of %s port %d failed at step %d! (%d)\n",
              _enum.hub ? _enum.hub->pos_id : "root", _enum.port, _enum.state, status);

    enum_stop_port();
    enum_release_device();

    if ((status != USBH_ERR_DISCONNECTED) && (++_enum.retry <= ENUM_RETRY))
    {
        /* Start over from the port reset */
        _enum.gen++;
        _enum.state = ES_DEBOUNCE;
        usbh_timer_start(&_enum.timer, 100);
        return;
    }

    _enum.gen++;
    _enum.state = ES_IDLE;
    enum_start_next();
}

static void enum_ctrl_done(UDEV_T *udev, int status, uint32_t xfer_len, void *context);

static void enum_request(int state, uint8_t type, uint8_t request, uint16_t value, uint16_t length, uint8_t *buff)
{
    int ret;

    _enum.state = state;
    ret = usbh_ctrl_xfer_async(_enum.udev, type, request, value, 0, length, buff,
                               ENUM_XFER_TIMEOUT, enum_ctrl_done, enum_token());
    if (ret < 0)
        enum_fail(ret);
}

static void enum_reset_done(int status, int speed, void *context)
{
    UDEV_T  *udev;

    if (!enum_current(context) || (_enum.state != ES_RESET))
        return;

    if (status != USBH_OK)
    {
        enum_fail(status);
        return;
    }

    udev = alloc_device();
    if (udev == NULL)
    {
        enum_fail(USBH_ERR_MEMORY_OUT);
        return;
    }
    udev->parent = _enum.hub;
    udev->port_num = _enum.port;
    udev->speed = (SPEED_E)speed;
    udev->hc_driver = _enum.hc;
    _enum.udev = udev;

    _enum.state = ES_RESET_RECOVERY;
    usbh_timer_start(&_enum.timer, 20);
}

static void enum_timer(void *context)
{
    int  ret;

    switch (_enum.state)
    {
    case ES_DEBOUNCE:
        _enum.state = ES_RESET;
        if (_enum.hub == NULL)
            ret = _enum.hc->rthub_port_reset(_enum.hc, _enum.port - 1, enum_reset_done, enum_token());
        else
            ret = usbh_hub_port_reset(_enum.hub, _enum.port, enum_reset_done, enum_token());
        if (ret < 0)
            enum_fail(ret);
        break;

    case ES_RESET_RECOVERY:
        /* Only the first 8 bytes for now, the max packet size is in them */
        enum_request(ES_GET_DESC_SHORT, REQ_TYPE_IN | REQ_TYPE_STD_DEV | REQ_TYPE_TO_DEV,
                     USB_REQ_GET_DESCRIPTOR, (USB_DT_STANDARD | USB_DT_DEVICE) << 8, 8, _enum.buff);
        break;

    case ES_ADDRESS_RECOVERY:
        enum_request(ES_GET_DESC, REQ_TYPE_IN | REQ_TYPE_STD_DEV | REQ_TYPE_TO_DEV,
                     USB_REQ_GET_DESCRIPTOR, (USB_DT_STANDARD | USB_DT_DEVICE) << 8,
                     sizeof(DESC_DEV_T), _enum.buff);
        break;

    default:
        break;
    }
}

static void enum_ctrl_done(UDEV_T *udev, int status, uint32_t xfer_len, void *context)
{
    DESC_CONF_T  *conf;
    uint8_t      mps;
    int          ret;

    if (!enum_current(context))
        return;

    if (status != 0)
    {
        enum_fail(status);
        return;
    }

    udev = _enum.udev;

    switch (_enum.state)
    {
    case ES_GET_DESC_SHORT:
        if (xfer_len < 8)
        {
            enum_fail(USBH_ERR_DATA_UNDERRUN);
            break;
        }
        mps = _enum.buff[7];
        if ((mps != 8) && (mps != 16) && (mps != 32) && (mps != 64))
            mps = 8;
        udev->descriptor.bMaxPacketSize0 = mps;

        _enum.dev_num = alloc_dev_address();
        enum_request(ES_SET_ADDRESS, REQ_TYPE_OUT | REQ_TYPE_STD_DEV | REQ_TYPE_TO_DEV,
                     USB_REQ_SET_ADDRESS, _enum.dev_num, 0, NULL);
        break;

    case ES_SET_ADDRESS:
        udev->dev_num = _enum.dev_num;
        _enum.state = ES_ADDRESS_RECOVERY;
        usbh_timer_start(&_enum.timer, 10);
        break;

    case ES_GET_DESC:
        if (xfer_len < sizeof(DESC_DEV_T))
        {
            enum_fail(USBH_ERR_DATA_UNDERRUN);
            break;
        }
        memcpy(&udev->descriptor, _enum.buff, sizeof(DESC_DEV_T));

        USB_debug("New %s device %04x:%04x, address %d.\n",
                  (udev->speed == SPEED_LOW) ? "low-speed" : "full-speed",
                  udev->descriptor.idVendor, udev->descriptor.idProduct, udev->dev_num);
#if  defined(DUMP_DESCRIPTOR) && defined(ENABLE_DEBUG_MSG)
        dump_device_descriptor(&udev->descriptor);
#endif
        if (udev->descriptor.bNumConfigurations != 1)
            USB_debug("Warning! This device has multiple configurations [%d].\n", udev->descriptor.bNumConfigurations);

        enum_request(ES_GET_CONF_SHORT, REQ_TYPE_IN | REQ_TYPE_STD_DEV | REQ_TYPE_TO_DEV,
                     USB_REQ_GET_DESCRIPTOR, (USB_DT_STANDARD | USB_DT_CONFIGURATION) << 8,
                     sizeof(DESC_CONF_T), _enum.buff);
        break;

    case ES_GET_CONF_SHORT:
        conf = (DESC_CONF_T *)_enum.buff;
        if ((xfer_len < sizeof(DESC_CONF_T)) || (conf->wTotalLength < sizeof(DESC_CONF_T)))
        {
            enum_fail(USBH_ERR_DESCRIPTOR);
            break;
        }
        if (conf->wTotalLength > MAX_DESC_BUFF_SIZE)
        {
            USB_error("Device configuration %d length > %d!\n", conf->wTotalLength, MAX_DESC_BUFF_SIZE);
            enum_fail(USBH_ERR_DATA_OVERRUN);
            break;
        }

        udev->cfd_buff = (uint8_t *)usbh_alloc_mem(MAX_DESC_BUFF_SIZE);
        if (udev->cfd_buff == NULL)
        {
            enum_fail(USBH_ERR_MEMORY_OUT);
            break;
        }
        enum_request(ES_GET_CONF, REQ_TYPE_IN | REQ_TYPE_STD_DEV | REQ_TYPE_TO_DEV,
                     USB_REQ_GET_DESCRIPTOR, (USB_DT_STANDARD | USB_DT_CONFIGURATION) << 8,
                     conf->wTotalLength, udev->cfd_buff);
        break;

    case ES_GET_CONF:
        conf = (DESC_CONF_T *)udev->cfd_buff;
        if (xfer_len < conf->wTotalLength)
        {
            enum_fail(USBH_ERR_DATA_UNDERRUN);
            break;
        }
#if  defined(DUMP_DESCRIPTOR) && defined(ENABLE_DEBUG_MSG)
        dump_config_descriptor(conf);
#endif
        /* Always select the first configuration */
        enum_request(ES_SET_CONF, REQ_TYPE_OUT | REQ_TYPE_STD_DEV | REQ_TYPE_TO_DEV,
                     USB_REQ_SET_CONFIGURATION, conf->bConfigurationValue, 0, NULL);
        break;

    case ES_SET_CONF:
        conf = (DESC_CONF_T *)udev->cfd_buff;
        udev->cur_conf = (int8_t)conf->bConfigurationValue;

        /* It's a device from here on, drivers probing it may already queue
         * transfers on it */
        _enum.udev = NULL;
        _enum.dev_num = 0;

        ret = usbh_parse_configuration(udev, (uint8_t *)conf);
        if (ret < 0)
            USB_debug("Parse configuration %d failed!\n", conf->bConfigurationValue);

        if (g_conn_func)
            g_conn_func(udev, 0);

        enum_finish();
        break;

    default:
        break;
    }
}

static void enum_init(void)
{
    memset(&_enum, 0, sizeof(_enum));
    _enum_queue_cnt = 0;
    usbh_timer_init(&_enum.timer, enum_timer, NULL);
}

static void enum_abort(void)
{
    if (_enum.state != ES_IDLE)
    {
        enum_stop_port();
        enum_release_device();
        _enum.gen++;
        _enum.state = ES_IDLE;
    }
    _enum_queue_cnt = 0;
    if (_enum.buff)
    {
        usbh_free_mem(_enum.buff, ENUM_BUFF_SIZE);
        _enum.buff = NULL;
    }
}

static void enum_dequeue(HC_DRV_T *hc, HUB_DEV_T *hub, int port)
{
    int  i;

    for (i = 0; i < _enum_queue_cnt; )
    {
        if ((_enum_queue[i].hc == hc) && (_enum_queue[i].hub == hub) && (_enum_queue[i].port == port))
        {
            _enum_queue_cnt--;
            memmove(&_enum_queue[i], &_enum_queue[i+1], (_enum_queue_cnt - i) * sizeof(_enum_queue[0]));
        }
        else
            i++;
    }
}

/* Gives up on the port if it's the one being enumerated */
static void enum_drop_port(HC_DRV_T *hc, HUB_DEV_T *hub, int port)
{
    enum_dequeue(hc, hub, port);

    if ((_enum.state != ES_IDLE) && (_enum.hc == hc) && (_enum.hub == hub) && (_enum.port == port))
    {
        enum_stop_port();
        enum_release_device();
        _enum.gen++;
        _enum.state = ES_IDLE;
    }
}

static void port_connected(HC_DRV_T *hc, HUB_DEV_T *hub, int port)
{
    UDEV_T  *udev;

    enum_drop_port(hc, hub, port);

    /* A device that was there is gone, whatever comes up now is new */
    while ((udev = find_child(hc, hub, port)) != NULL)
        disconnect_device(udev);

    if (_enum.buff == NULL)
        _enum.buff = (uint8_t *)usbh_alloc_mem(ENUM_BUFF_SIZE);

    if ((_enum.buff != NULL) && (_enum_queue_cnt < ENUM_QUEUE_LEN))
    {
        _enum_queue[_enum_queue_cnt].hc = hc;
        _enum_queue[_enum_queue_cnt].hub = hub;
        _enum_queue[_enum_queue_cnt].port = port;
        _enum_queue_cnt++;
    }
    else
        USB_error("Can't queue %s port %d for enumeration!\n", hub ? hub->pos_id : "root", port);

    enum_start_next();
}

static void port_disconnected(HC_DRV_T *hc, HUB_DEV_T *hub, int port)
{
    UDEV_T  *udev;

    enum_drop_port(hc, hub, port);

    while ((udev = find_child(hc, hub, port)) != NULL)
        disconnect_device(udev);

    enum_start_next();
}

void usbh_port_connected(HUB_DEV_T *hub, int port)
{
    port_connected(hub_hc(hub), hub, port);
}

void usbh_port_disconnected(HUB_DEV_T *hub, int port)
{
    port_disconnected(hub_hc(hub), hub, port);
}

void usbh_rh_port_connected(HC_DRV_T *hc, int port)
{
    port_connected(hc, NULL, port);
}

void usbh_rh_port_disconnected(HC_DRV_T *hc, int port)
{
    port_disconnected(hc, NULL, port);
}

void disconnect_device(UDEV_T *udev)
{
    IFACE_T      *iface;

    USB_debug("disconnect device...\n");

    if (g_disconn_func)
        g_disconn_func(udev, 0);

    /* Completes anything queued on it as aborted, after the next frame */
    usbh_quit_xfer(udev, &(udev->ep0));

    /* Notified all actived interface device driver  */
    iface = udev->iface_list;
    while (iface != NULL)
    {
        udev->iface_list = iface->next;
        iface->driver->disconnect(iface);
        usbh_free_mem(iface, sizeof(*iface));
        iface = udev->iface_list;
    }

    /* remove device from global device list */
    free_dev_address(udev->dev_num);
    free_device(udev);
}

EP_INFO_T * usbh_iface_find_ep(IFACE_T *iface, uint8_t ep_addr, uint8_t dir_type)
{
    ALT_IFACE_T  *aif = iface->aif;
    int     i;

    if (ep_addr == 0)      /* find the first EP matched with specified direction and type */
    {
        for (i = 0; i < aif->ifd->bNumEndpoints; i++)
        {
            if (((aif->ep[i].bEndpointAddress & EP_ADDR_DIR_MASK) == (dir_type & EP_ADDR_DIR_MASK)) &&
                    ((aif->ep[i].bmAttributes & EP_ATTR_TT_MASK) == (dir_type & EP_ATTR_TT_MASK)))
                return &aif->ep[i];
        }
    }
    else                   /* find the EP with specified endpoint address                 */
    {
        for (i = 0; i < aif->ifd->bNumEndpoints; i++)
        {
            if (aif->ep[i].bEndpointAddress == ep_addr)
                return &aif->ep[i];
        }
    }
    return NULL;
}

void  usbh_dump_buff_bytes(uint8_t *buff, int nSize)
{
    int     nIdx, i;

    nIdx = 0;
    while (nSize > 0)
    {
        sysprintf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            sysprintf("%02x ", buff[nIdx + i]);
        sysprintf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((buff[nIdx + i] >= 0x20) && (buff[nIdx + i] < 127))
                sysprintf("%c", buff[nIdx + i]);
            else
                sysprintf(".");
            nSize--;
        }
        nIdx += 16;
        sysprintf("\n");
    }
    sysprintf("\n");
}

void usbh_dump_iface(IFACE_T *iface)
{
    USB_debug("\n  [IFACE info] (0x%x)\n", (int)iface);
    USB_debug("  ----------------------------------------------\n");
    USB_debug("  udev       = 0x%x\n", iface->udev);
    USB_debug("  if_num     = %d\n", iface->if_num);
    USB_debug("  driver     = 0x%x\n", (int)iface->driver);
    USB_debug("  next       = 0x%x\n", (int)iface->next);
    usbh_dump_interface_descriptor(iface->aif->ifd);
}

void usbh_dump_ep_info(EP_INFO_T *ep)
{
    USB_debug("\n  [Endpoint Info] (0x%x)\n", (int)ep);
    USB_debug("  ----------------------------------------------\n");
    USB_debug("  bEndpointAddress    = 0x%02x\n", ep->bEndpointAddress);
    USB_debug("  bmAttributes        = 0x%02x\n", ep->bmAttributes);
    USB_debug("  bInterval           = %d\n",     ep->bInterval);
    USB_debug("  wMaxPacketSize      = %d\n",     ep->wMaxPacketSize);
    USB_debug("  hw_pipe             = 0x%x\n",   (int)ep->hw_pipe);
}

/// @endcond HIDDEN_SYMBOLS


/// @endcond HIDDEN_SYMBOLS


/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
