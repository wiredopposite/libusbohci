/**************************************************************************//**
 * @file     hub.c
 * @version  V1.10
 * $Revision: 11 $
 * $Date: 14/10/03 1:54p $
 * @brief   USB Host library hub class driver.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "N9H30.h"

#include "usb.h"
#include "usbh_lib.h"
#include "hub.h"


/// @cond HIDDEN_SYMBOLS


#define HUB_DBGMSG     USB_debug

#define HUB_BUFF_SIZE  16

/*
 *  A hub is set up and its status changes handled as a chain of control
 *  transfers, each one started from the completion of the one before.
 */
enum
{
    HS_IDLE = 0,
    HS_GET_DESC,
    HS_POWER_PORT,
    HS_POWER_WAIT,
    HS_RUNNING,
    HS_GET_HUB_STATUS,
    HS_CLEAR_HUB_CHANGE,
    HS_GET_PORT_STATUS,
    HS_CLEAR_PORT_CHANGE,
};

enum
{
    RS_IDLE = 0,
    RS_SET_RESET,
    RS_WAIT,
    RS_GET_STATUS,
    RS_CLEAR_CHANGE,
};

static HUB_DEV_T  g_hub_dev[MAX_HUB_DEVICE];

static void hub_process(HUB_DEV_T *hub);

/*
 *  Callbacks carry the hub's slot and generation instead of a pointer, so one
 *  that arrives after the hub went away is dropped.
 */
static void * hub_token(HUB_DEV_T *hub)
{
    return (void *)(uintptr_t)(((hub->gen & 0xFFFFFF) << 8) | (uint32_t)(hub - g_hub_dev));
}

static HUB_DEV_T * hub_from_token(void *token)
{
    uint32_t  value = (uint32_t)(uintptr_t)token;
    HUB_DEV_T *hub;

    if ((value & 0xFF) >= MAX_HUB_DEVICE)
        return NULL;
    hub = &g_hub_dev[value & 0xFF];
    if ((hub->iface == NULL) || ((hub->gen & 0xFFFFFF) != (value >> 8)))
        return NULL;
    return hub;
}

static HUB_DEV_T *alloc_hub_device(void)
{
    int       i;
    uint32_t  gen;

    for (i = 0; i < MAX_HUB_DEVICE; i++)
    {
        if (g_hub_dev[i].iface == NULL)
        {
            gen = g_hub_dev[i].gen;
            memset((char *)&g_hub_dev[i], 0, sizeof(HUB_DEV_T));
            g_hub_dev[i].gen = gen;
            return &g_hub_dev[i];
        }
    }
    return NULL;
}

static void free_hub_device(HUB_DEV_T *hub)
{
    usbh_timer_cancel(&hub->timer);
    usbh_timer_cancel(&hub->rst_timer);
    if (hub->buff)
        usbh_free_mem(hub->buff, HUB_BUFF_SIZE);
    if (hub->rst_buff)
        usbh_free_mem(hub->rst_buff, HUB_BUFF_SIZE);
    hub->buff = NULL;
    hub->rst_buff = NULL;
    hub->iface = NULL;
    hub->gen++;
}

static HUB_DEV_T * find_hub_device(IFACE_T *iface)
{
    int     i;
    for (i = 0; i < MAX_HUB_DEVICE; i++)
    {
        if (g_hub_dev[i].iface == iface)
        {
            return &g_hub_dev[i];
        }
    }
    return NULL;
}

static int hub_request(HUB_DEV_T *hub, uint8_t *buff, uint8_t type, uint8_t request, uint16_t value,
                       uint16_t index, uint16_t length, USBH_CTRL_FUNC func)
{
    return usbh_ctrl_xfer_async(hub->iface->udev, type, request, value, index, length,
                                buff, 200, func, hub_token(hub));
}

/*--------------------------------------------------------------------------*/
/*   Status change interrupt                                                */
/*--------------------------------------------------------------------------*/

static void hub_resubmit_status(HUB_DEV_T *hub)
{
    int ret;

    hub->utr->xfer_len = 0;
    hub->utr->bIsTransferDone = 0;
    ret = usbh_int_xfer(hub->utr);
    if (ret < 0)
        USB_error("Failed to re-submit HUB [%s] interrupt-in request (%d)\n", hub->pos_id, ret);
}

static void hub_status_free(UTR_T *utr)
{
    usbh_free_mem(utr->buff, HUB_STATUS_MAX_BYTE);
    free_utr(utr);
}

static void hub_status_irq(UTR_T *utr)
{
    HUB_DEV_T   *hub;
    uint16_t    bitmap = 0;
    int         i;

    hub = hub_from_token(utr->context);
    if (hub == NULL)
        return;

    if (utr->status != 0)
    {
        if (utr->status != USBH_ERR_ABORT)
        {
            USB_error("hub_status_irq - has error: 0x%x\n", utr->status);
            hub_resubmit_status(hub);
        }
        return;
    }

    for (i = 0; i < utr->xfer_len; i++)
        bitmap |= (utr->buff[i] << (i * 8));

    if (bitmap == 0)
    {
        hub_resubmit_status(hub);
        return;
    }

    HUB_DBGMSG("Hub [%s] status change bitmap: 0x%x\n", hub->pos_id, bitmap);

    hub->sc_bitmap = bitmap;
    hub->port = 0;
    hub_process(hub);
}

/*--------------------------------------------------------------------------*/
/*   Setup and status handling                                              */
/*--------------------------------------------------------------------------*/

static void hub_ctrl_done(UDEV_T *udev, int status, uint32_t xfer_len, void *context);

/* Clears the lowest change bit still set, returns 0 once none are left */
static int hub_clear_next_port_change(HUB_DEV_T *hub)
{
    static const uint16_t  bits[]     = { PORT_C_CONNECTION, PORT_C_ENABLE, PORT_C_SUSPEND, PORT_C_OVERCURRENT, PORT_C_RESET };
    static const uint8_t   features[] = { FS_C_PORT_CONNECTION, FS_C_PORT_ENABLE, FS_C_PORT_SUSPEND, FS_C_PORT_OVER_CURRENT, FS_C_PORT_RESET };
    int  i;

    for (i = 0; i < (int)(sizeof(bits) / sizeof(bits[0])); i++)
    {
        if (hub->wChange & bits[i])
        {
            hub->wChange &= ~bits[i];
            hub->state = HS_CLEAR_PORT_CHANGE;
            if (hub_request(hub, NULL, REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_OTHER,
                            USB_REQ_CLEAR_FEATURE, features[i], hub->port, 0, hub_ctrl_done) < 0)
                return -1;
            return 1;
        }
    }
    return 0;
}

static int hub_clear_next_hub_change(HUB_DEV_T *hub)
{
    uint8_t  feature;

    if (hub->wChange & HUB_C_LOCAL_POWER)
    {
        hub->wChange &= ~HUB_C_LOCAL_POWER;
        feature = FS_C_HUB_LOCAL_POWER;
    }
    else if (hub->wChange & HUB_C_OVERCURRENT)
    {
        hub->wChange &= ~HUB_C_OVERCURRENT;
        feature = FS_C_HUB_OVER_CURRENT;
    }
    else
        return 0;

    hub->state = HS_CLEAR_HUB_CHANGE;
    if (hub_request(hub, NULL, REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_DEV,
                    USB_REQ_CLEAR_FEATURE, feature, 0, 0, hub_ctrl_done) < 0)
        return -1;
    return 1;
}

/*
 *  Works through the change bitmap: the hub's own status first, then each
 *  port, and resubmits the status interrupt once they're all handled.
 */
static void hub_process(HUB_DEV_T *hub)
{
    if ((hub->sc_bitmap & 0x1) && (hub->port == 0))
    {
        hub->sc_bitmap &= ~0x1;
        hub->state = HS_GET_HUB_STATUS;
        if (hub_request(hub, hub->buff, REQ_TYPE_IN | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_DEV,
                        USB_REQ_GET_STATUS, 0, 0, 4, hub_ctrl_done) == 0)
            return;
    }

    for (hub->port++; hub->port <= hub->bNbrPorts; hub->port++)
    {
        if (hub->sc_bitmap & (1 << hub->port))
        {
            hub->sc_bitmap &= ~(1 << hub->port);
            hub->state = HS_GET_PORT_STATUS;
            if (hub_request(hub, hub->buff, REQ_TYPE_IN | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_OTHER,
                            USB_REQ_GET_STATUS, 0, hub->port, 4, hub_ctrl_done) == 0)
                return;
        }
    }

    hub->state = HS_RUNNING;
    hub->sc_bitmap = 0;
    hub_resubmit_status(hub);
}

static void hub_power_next_port(HUB_DEV_T *hub)
{
    if (++hub->port <= hub->bNbrPorts)
    {
        hub->state = HS_POWER_PORT;
        if (hub_request(hub, NULL, REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_OTHER,
                        USB_REQ_SET_FEATURE, FS_PORT_POWER, hub->port, 0, hub_ctrl_done) == 0)
            return;
        HUB_DBGMSG("Hub [%s] port %d power enabling failed!\n", hub->pos_id, hub->port);
    }

    /* Wait for the ports to be powered, then start listening for changes */
    hub->state = HS_POWER_WAIT;
    usbh_timer_start(&hub->timer, hub->bPwrOn2PwrGood + 100);
}

static void hub_timer(void *context)
{
    HUB_DEV_T  *hub = hub_from_token(context);
    int        ret;

    if ((hub == NULL) || (hub->state != HS_POWER_WAIT))
        return;

    hub->state = HS_RUNNING;
    ret = usbh_int_xfer(hub->utr);
    if (ret < 0)
        USB_error("Error - failed to submit hub interrupt read request (%d)\n", ret);
    else
        HUB_DBGMSG("Hub [%s] ready.\n", hub->pos_id);
}

static void hub_ctrl_done(UDEV_T *udev, int status, uint32_t xfer_len, void *context)
{
    HUB_DEV_T  *hub = hub_from_token(context);
    DESC_HUB_T *desc;
    int        connected, ret;

    if (hub == NULL)
        return;

    switch (hub->state)
    {
    case HS_GET_DESC:
        desc = (DESC_HUB_T *)hub->buff;
        if ((status == 0) && (xfer_len >= 7))
        {
            hub->bNbrPorts = desc->bNbrPorts;
            hub->bPwrOn2PwrGood = desc->bPwrOn2PwrGood * 2;
            if (hub->bNbrPorts > HUB_STATUS_MAX_BYTE * 8 - 1)
                hub->bNbrPorts = HUB_STATUS_MAX_BYTE * 8 - 1;
            HUB_DBGMSG("Hub has %d ports, power-to-power-good time is %d ms.\n", hub->bNbrPorts, hub->bPwrOn2PwrGood);
        }
        else
            USB_error("Failed to get hub descriptor!\n");

        hub->port = 0;
        hub_power_next_port(hub);
        break;

    case HS_POWER_PORT:
        if (status != 0)
            HUB_DBGMSG("Hub [%s] port %d power enabling failed!\n", hub->pos_id, hub->port);
        hub_power_next_port(hub);
        break;

    case HS_GET_HUB_STATUS:
        if ((status == 0) && (xfer_len == 4))
        {
            hub->wStatus = (hub->buff[1] << 8) | hub->buff[0];
            hub->wChange = (hub->buff[3] << 8) | hub->buff[2];
            if (hub_clear_next_hub_change(hub) > 0)
                break;
        }
        else
            USB_error("Failed to get Hub [%s] status! (%d)\n", hub->pos_id, status);
        hub_process(hub);
        break;

    case HS_CLEAR_HUB_CHANGE:
        if ((status == 0) && (hub_clear_next_hub_change(hub) > 0))
            break;
        hub_process(hub);
        break;

    case HS_GET_PORT_STATUS:
        if ((status != 0) || (xfer_len != 4))
        {
            USB_error("Failed to get Hub [%s] port %d status! (%d)\n", hub->pos_id, hub->port, status);
            hub_process(hub);
            break;
        }
        hub->wStatus = (hub->buff[1] << 8) | hub->buff[0];
        hub->wChange = (hub->buff[3] << 8) | hub->buff[2];
        HUB_DBGMSG("Hub [%s] port %d, status: 0x%x, change: 0x%x\n", hub->pos_id, hub->port, hub->wStatus, hub->wChange);

        /* The connect event goes out once the changes are cleared, keep it */
        hub->pending = hub->wChange;
        ret = hub_clear_next_port_change(hub);
        if (ret > 0)
            break;
        hub_process(hub);
        break;

    case HS_CLEAR_PORT_CHANGE:
        if (status == 0)
        {
            ret = hub_clear_next_port_change(hub);
            if (ret > 0)
                break;
        }
        if (hub->pending & PORT_C_CONNECTION)
        {
            connected = (hub->wStatus & PORT_S_CONNECTION) != 0;
            hub->pending = 0;
            if (connected)
                usbh_port_connected(hub, hub->port);
            else
                usbh_port_disconnected(hub, hub->port);
        }
        hub->pending = 0;
        hub_process(hub);
        break;

    default:
        break;
    }
}

/*--------------------------------------------------------------------------*/
/*   Port reset                                                             */
/*--------------------------------------------------------------------------*/

static void hub_reset_done(UDEV_T *udev, int status, uint32_t xfer_len, void *context);

static void hub_reset_finish(HUB_DEV_T *hub, int status, int speed)
{
    PORT_RESET_DONE  done = hub->rst_done;
    void             *context = hub->rst_context;

    hub->rst_state = RS_IDLE;
    hub->rst_done = NULL;
    if (done)
        done(status, speed, context);
}

static void hub_reset_start(HUB_DEV_T *hub)
{
    hub->rst_state = RS_SET_RESET;
    hub->rst_elapsed = 0;
    if (hub_request(hub, NULL, REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_OTHER,
                    USB_REQ_SET_FEATURE, FS_PORT_RESET, hub->rst_port, 0, hub_reset_done) < 0)
        hub_reset_finish(hub, USBH_ERR_PORT_RESET, 0);
}

static void hub_reset_timer(void *context)
{
    HUB_DEV_T  *hub = hub_from_token(context);

    if ((hub == NULL) || (hub->rst_state != RS_WAIT))
        return;

    hub->rst_state = RS_GET_STATUS;
    if (hub_request(hub, hub->rst_buff, REQ_TYPE_IN | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_OTHER,
                    USB_REQ_GET_STATUS, 0, hub->rst_port, 4, hub_reset_done) < 0)
        hub_reset_finish(hub, USBH_ERR_PORT_RESET, 0);
}

static void hub_reset_done(UDEV_T *udev, int status, uint32_t xfer_len, void *context)
{
    HUB_DEV_T  *hub = hub_from_token(context);
    uint16_t   wPortStatus;
    int        speed;

    if ((hub == NULL) || (hub->rst_state == RS_IDLE))
        return;

    if (status != 0)
    {
        hub_reset_finish(hub, status, 0);
        return;
    }

    switch (hub->rst_state)
    {
    case RS_SET_RESET:
        hub->rst_state = RS_WAIT;
        usbh_timer_start(&hub->rst_timer, 10);
        break;

    case RS_GET_STATUS:
        wPortStatus = (hub->rst_buff[1] << 8) | hub->rst_buff[0];

        if ((wPortStatus & PORT_S_CONNECTION) == 0)
        {
            hub_reset_finish(hub, USBH_ERR_DISCONNECTED, 0);
            break;
        }

        /* Reset is over once the port is enabled and no longer resetting */
        if (((wPortStatus & PORT_S_ENABLE) != 0) && ((wPortStatus & PORT_S_RESET) == 0))
        {
            if (wPortStatus & PORT_S_HIGH_SPEED)
                speed = SPEED_HIGH;
            else if (wPortStatus & PORT_S_LOW_SPEED)
                speed = SPEED_LOW;
            else
                speed = SPEED_FULL;

            hub->rst_state = RS_CLEAR_CHANGE;
            hub->rst_time = speed;
            if (hub_request(hub, NULL, REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_OTHER,
                            USB_REQ_CLEAR_FEATURE, FS_C_PORT_RESET, hub->rst_port, 0, hub_reset_done) < 0)
                hub_reset_finish(hub, USBH_OK, speed);
            break;
        }

        hub->rst_elapsed += 10;
        if (hub->rst_elapsed < PORT_RESET_TIME_MS + hub->rst_retry * PORT_RESET_RETRY_INC_MS)
        {
            hub->rst_state = RS_WAIT;
            usbh_timer_start(&hub->rst_timer, 10);
        }
        else if (++hub->rst_retry < PORT_RESET_RETRY)
            hub_reset_start(hub);
        else
        {
            USB_debug("HUB [%s] port %d - port reset failed!\n", hub->pos_id, hub->rst_port);
            hub_reset_finish(hub, USBH_ERR_PORT_RESET, 0);
        }
        break;

    case RS_CLEAR_CHANGE:
        /* rst_time held the speed while the change was cleared */
        hub_reset_finish(hub, USBH_OK, hub->rst_time);
        break;

    default:
        break;
    }
}

int usbh_hub_port_reset(HUB_DEV_T *hub, int port, PORT_RESET_DONE done, void *context)
{
    if ((hub->iface == NULL) || (hub->rst_state != RS_IDLE))
        return USBH_ERR_PORT_RESET;

    hub->rst_port = port;
    hub->rst_retry = 0;
    hub->rst_done = done;
    hub->rst_context = context;
    hub_reset_start(hub);
    return 0;
}

void usbh_hub_port_reset_cancel(HUB_DEV_T *hub)
{
    usbh_timer_cancel(&hub->rst_timer);
    hub->rst_state = RS_IDLE;
    hub->rst_done = NULL;
}

/*--------------------------------------------------------------------------*/
/*   Driver                                                                 */
/*--------------------------------------------------------------------------*/

int hub_probe(IFACE_T *iface)
{
    UDEV_T      *udev = iface->udev;
    ALT_IFACE_T *aif = iface->aif;
    EP_INFO_T   *ep = NULL;
    HUB_DEV_T   *hub;
    UTR_T       *utr;
    int         i;
    char        str[2] = "0";

    if (aif->ifd->bInterfaceClass != USB_CLASS_HUB)
        return USBH_ERR_NOT_MATCHED;

    for (i = 0; i < aif->ifd->bNumEndpoints; i++)
    {
        if (((aif->ep[i].bmAttributes & EP_ATTR_TT_MASK) == EP_ATTR_TT_INT) &&
                ((aif->ep[i].bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_IN))
        {
            ep = &aif->ep[i];
            break;
        }
    }

    if (ep == NULL)
        return USBH_ERR_NOT_MATCHED;        /* no INT-in endpoints, Ignore this interface */

    hub = alloc_hub_device();
    if (hub == NULL)
        return USBH_ERR_MEMORY_OUT;

    hub->buff = usbh_alloc_mem(HUB_BUFF_SIZE);
    hub->rst_buff = usbh_alloc_mem(HUB_BUFF_SIZE);
    utr = alloc_utr(udev);
    if (utr)
        utr->buff = usbh_alloc_mem(HUB_STATUS_MAX_BYTE);
    if (!hub->buff || !hub->rst_buff || !utr || !utr->buff)
    {
        if (utr)
        {
            if (utr->buff)
                usbh_free_mem(utr->buff, HUB_STATUS_MAX_BYTE);
            free_utr(utr);
        }
        free_hub_device(hub);
        return USBH_ERR_MEMORY_OUT;
    }

    hub->iface = iface;
    iface->context = (void *)hub;

    str[0] += udev->port_num;
    if (udev->parent == NULL)
        strcpy(hub->pos_id, str);
    else
    {
        strcpy(hub->pos_id, udev->parent->pos_id);
        strcat(hub->pos_id, str);
    }

    HUB_DBGMSG("hub found is:[%s] - device (vid=0x%x, pid=0x%x), interface %d.\n", hub->pos_id,
               udev->descriptor.idVendor, udev->descriptor.idProduct, iface->if_num);

    hub->utr = utr;
    utr->context = hub_token(hub);
    utr->ep = ep;
    utr->data_len = HUB_STATUS_MAX_BYTE;
    utr->func = hub_status_irq;

    usbh_timer_init(&hub->timer, hub_timer, hub_token(hub));
    usbh_timer_init(&hub->rst_timer, hub_reset_timer, hub_token(hub));

    hub->state = HS_GET_DESC;
    if (hub_request(hub, hub->buff, REQ_TYPE_IN | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_DEV,
                    USB_REQ_GET_DESCRIPTOR, ((USB_DT_CLASS | 0x9) << 8), 0, sizeof(DESC_HUB_T) < HUB_BUFF_SIZE ? sizeof(DESC_HUB_T) : HUB_BUFF_SIZE,
                    hub_ctrl_done) < 0)
    {
        usbh_free_mem(utr->buff, HUB_STATUS_MAX_BYTE);
        free_utr(utr);
        free_hub_device(hub);
        iface->context = NULL;
        return USBH_ERR_TRANSFER;
    }
    return 0;
}

void hub_disconnect(IFACE_T *iface)
{
    HUB_DEV_T   *hub;
    int         port;

    hub = find_hub_device(iface);
    if (hub == NULL)
    {
        HUB_DBGMSG("hub_disconnect - hub not found!\n");
        return;
    }

    for (port = 1; port <= hub->bNbrPorts; port++)
        usbh_port_disconnected(hub, port);

    if (hub->utr)
    {
        /* Freed once pulling the endpoint completes it */
        hub->utr->func = hub_status_free;
        if (hub->utr->bIsTransferDone || (hub->state != HS_RUNNING) ||
                (usbh_quit_utr(hub->utr) != 0))
            hub_status_free(hub->utr);
        hub->utr = NULL;
    }

    HUB_DBGMSG("Disconnect HUB [%s].\n", hub->pos_id);
    free_hub_device(hub);
}


UDEV_DRV_T  hub_driver =
{
    hub_probe,
    hub_disconnect,
    NULL,
    NULL
};


/**
  * @brief    Initialize USB Hub Class device driver.
  * @return   None
  */
void usbh_hub_init(void)
{
    memset((char *)&g_hub_dev[0], 0, sizeof(g_hub_dev));
    usbh_register_driver(&hub_driver);
}


/// @endcond HIDDEN_SYMBOLS

/**
  * @brief    Find the device under the specified hub port.
  * @param[in]  hub_id    Hub identify ID
  * @param[in]  port      Port number of the specified ub.
  * @retval   NULL   Not found. There's no valid device connected under that hub port.
  * @retval   Otherwise  An UDEV_T pointer reference to the device under specified hub and port.
  */
UDEV_T * usbh_find_device(char *hub_id, int port)
{
    int         i;
    HUB_DEV_T   *hub = NULL;

    for (i = 0; i < MAX_HUB_DEVICE; i++)
    {
        if ((g_hub_dev[i].iface != NULL) && (strcmp(g_hub_dev[i].pos_id, hub_id) == 0))
        {
            hub = &g_hub_dev[i];
            break;
        }
    }
    if (hub == NULL)
        return NULL;

    return usbh_find_child(hub, port);
}


/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
