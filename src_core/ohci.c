/**************************************************************************//**
 * @file     ohci.c
 * @version  V1.10
 * $Revision: 11 $
 * $Date: 14/10/03 1:54p $
 * @brief   USB Host library OHCI (USB 1.1) host controller driver.
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
#include "hub.h"
#include "ohci.h"

/// @cond HIDDEN_SYMBOLS

//#define TD_debug   sysprintf
#define TD_debug(...)

//#define ED_debug   sysprintf
#define ED_debug(...)


typedef struct ohci_t
{
    HC_DRV_T        drv;
    USBH_T          *regs;
    HCCA_T          *hcca;
    ED_T            *Ied[6];
    ED_T            *ed_remove_list;
    int             port_cnt;
    struct
    {
        int             active;
        int             port;
        int             retry;
        int             elapsed;
        PORT_RESET_DONE done;
        void            *context;
        USBH_TIMER_T    timer;
    } rh_reset;
    USBH_TIMER_T    rh_poll_timer;
} OHCI_T;

static OHCI_T  _ohci_hc[OHCI_HC_CNT];

#define OHCI_OF(hc)     ((OHCI_T *)(hc))
#define UTR_OHCI(utr)   OHCI_OF((utr)->udev->hc_driver)

static void ohci_rh_reset_timer(void *context);
static void ohci_rh_poll_timer(void *context);

static void add_to_ED_remove_list(OHCI_T *oh, ED_T *ed)
{
    ED_T  *p;

    ED_debug("add_to_ED_remove_list - 0x%x (0x%x)\n", (int)ed, ed->Info);

    /* check if this ED found in oh->ed_remove_list */
    p = oh->ed_remove_list;
    while (p)
    {
        if (p == ed)
        {
            return;                         /* do nothing                                 */
        }
        p = p->next;
    }

    ed->Info |= ED_SKIP;                    /* ask OHCI controller skip this ED           */
    ed->next = oh->ed_remove_list;
    oh->ed_remove_list = ed;                    /* insert to the head of oh->ed_remove_list       */
    /* The controller is done with it after the next start of frame, which is
     * where it's taken off its list and its transfers completed as aborted */
    oh->regs->HcInterruptStatus = USBH_HcInterruptStatus_SF_Msk;
    oh->regs->HcInterruptEnable |= USBH_HcInterruptEnable_SF_Msk;
}

static int ohci_reset(OHCI_T *oh)
{
    volatile int  t0;

    /* Disable HC interrupts  */
    oh->regs->HcInterruptDisable = USBH_HcInterruptDisable_MIE_Msk;

    /* HC Reset requires max 10 ms delay  */
    oh->regs->HcControl = 0;
    oh->regs->HcCommandStatus = USBH_HcCommandStatus_HCR_Msk;

    delay_us(10000);

    /* Check if OHCI reset completed? */
    if ((oh->regs->HcCommandStatus & USBH_HcCommandStatus_HCR_Msk) != 0)
    {
        USB_error("Error! - USB OHCI reset timed out!\n");
        return -1;
    }

    oh->regs->HcRhStatus = USBH_HcRhStatus_OCI_Msk | USBH_HcRhStatus_LPS_Msk;

    oh->regs->HcControl = HCFS_RESET;

    delay_us(10000);

    /* Check if OHCI reset completed? */
    if ((oh->regs->HcCommandStatus & USBH_HcCommandStatus_HCR_Msk) != 0)
    {
        USB_error("Error! - USB HC reset timed out!\n");
        return -1;
    }
    return 0;
}

static void init_hcca_int_table(OHCI_T *oh)
{
    ED_T   *ed_p;
    int    i, idx, interval;

    memset(oh->hcca->int_table, 0, sizeof(oh->hcca->int_table));

    for (i = 5; i >= 0; i--)                /* interval = i^2                             */
    {
        oh->Ied[i] = alloc_ohci_ED();
        oh->Ied[i]->Info = ED_SKIP;

        interval = 0x1 << i;

        for (idx = interval - 1; idx < 32; idx += interval)
        {
            if (oh->hcca->int_table[idx] == 0)       /* is empty list, insert directly        */
            {
                oh->hcca->int_table[idx] = (uint32_t)virt_to_dma(oh->Ied[i]);
            }
            else
            {
                ed_p = dma_to_virt((ED_T *)oh->hcca->int_table[idx]);

                while (1)
                {
                    if (ed_p == oh->Ied[i])
                        break;                   /* already chained by previous visit     */

                    if (ed_p->NextED == 0)       /* reach end of list?                    */
                    {
                        ed_p->NextED = (uint32_t)virt_to_dma(oh->Ied[i]);
                        break;
                    }
                    ed_p = (ED_T *)dma_to_virt((void *)ed_p->NextED);
                }
            }
        }
    }
}

static ED_T * get_int_tree_head_node(OHCI_T *oh, int interval)
{
    int    i;

    for (i = 0; i < 5; i++)
    {
        interval >>= 1;
        if (interval == 0)
            return oh->Ied[i];
    }
    return oh->Ied[5];                         /* for interval >= 32                         */
}

static int get_ohci_interval(int interval)
{
    int    i, bInterval = 1;

    for (i = 0; i < 5; i++)
    {
        interval >>= 1;
        if (interval == 0)
            return bInterval;
        bInterval *= 2;
    }
    return 32;                              /* for interval >= 32                         */
}


/*
 *  The overcurrent inputs of root hub ports that aren't wired to a connector
 *  are tied active, and while they read active a device on another port is
 *  seen but its port never enables. The indicator is meant to be read only,
 *  but this controller lets it be written.
 */
static void ohci_clear_overcurrent(OHCI_T *oh)
{
    oh->regs->HcRhDescriptorA |= USBH_HcRhDescriptorA_NOCP_Msk;
    oh->regs->HcRhStatus = USBH_HcRhStatus_OCI_Msk | USBH_HcRhStatus_OCIC_Msk;
}

static int  ohci_init(HC_DRV_T *hc)
{
    OHCI_T      *oh = OHCI_OF(hc);
    uint32_t    fminterval;
    volatile int    i;

    oh->hcca = (HCCA_T *)USB_malloc(256, 256);

    if (ohci_reset(oh) < 0)
        return -1;

    oh->ed_remove_list = NULL;
    memset(&oh->rh_reset, 0, sizeof(oh->rh_reset));
    usbh_timer_init(&oh->rh_reset.timer, ohci_rh_reset_timer, oh);
    usbh_timer_init(&oh->rh_poll_timer, ohci_rh_poll_timer, oh);

    oh->port_cnt = oh->regs->HcRhDescriptorA & USBH_HcRhDescriptorA_NDP_Msk;
    if (oh->port_cnt > OHCI_PORT_CNT)
        oh->port_cnt = OHCI_PORT_CNT;

    init_hcca_int_table(oh);

    /* Tell the controller where the control and bulk lists are
     * The lists are empty now. */
    oh->regs->HcControlHeadED = 0;             /* control ED list head                       */
    oh->regs->HcBulkHeadED = 0;                /* bulk ED list head                          */

    oh->regs->HcHCCA = (uint32_t)virt_to_dma(oh->hcca);  /* HCCA area                           */

    /* periodic start 90% of frame interval       */
    fminterval = 0x2edf;                    /* 11,999                                     */
    oh->regs->HcPeriodicStart = (fminterval*9)/10;

    /* set FSLargestDataPacket, 10,104 for 0x2edf frame interval */
    fminterval |= ((((fminterval - 210) * 6) / 7) << 16);
    oh->regs->HcFmInterval = fminterval;

    oh->regs->HcLSThreshold = 0x628;

    /* start controller operations                */
    oh->regs->HcControl = HCFS_OPER | (0x3 << USBH_HcControl_CBSR_Pos);

#ifdef OHCI_PER_PORT_POWER
    oh->regs->HcRhDescriptorB = 0x60000;
    for (int i = 0; i < oh->port_cnt; i++)
    {
        oh->regs->HcRhPortStatus[i] = USBH_HcRhPortStatus_PPS_Msk;
    }
#else
    oh->regs->HcRhDescriptorA = (oh->regs->HcRhDescriptorA | (1<<9)) & ~USBH_HcRhDescriptorA_PSM_Msk;
    ohci_clear_overcurrent(oh);
    oh->regs->HcRhStatus = USBH_HcRhStatus_LPSC_Msk;
#endif

    oh->regs->HcInterruptEnable = USBH_HcInterruptEnable_MIE_Msk | USBH_HcInterruptEnable_WDH_Msk | USBH_HcInterruptEnable_SF_Msk;

    /* POTPGT delay is bits 24-31, in 2 ms units. */
    delay_us((oh->regs->HcRhDescriptorA >> 23 & 0x1FE) * 1000);
    return 0;
}

static void ohci_suspend(HC_DRV_T *hc)
{
    OHCI_T  *oh = OHCI_OF(hc);

    /* set port suspend if connected */
    for (int i = 0; i < oh->port_cnt; i++)
    {
        if (oh->regs->HcRhPortStatus[i] & 0x1)
            oh->regs->HcRhPortStatus[i] = 0x4;
    }

    /* enable Device Remote Wakeup */
    oh->regs->HcRhStatus |= USBH_HcRhStatus_DRWE_Msk;

    /* enable USBH RHSC interrupt for system wakeup */
    oh->regs->HcInterruptEnable |=  USBH_HcInterruptEnable_RHSC_Msk | USBH_HcInterruptEnable_RD_Msk;

    /* set Host Controller enter suspend state */
    oh->regs->HcControl = (oh->regs->HcControl & ~USBH_HcControl_HCFS_Msk) | (3 << USBH_HcControl_HCFS_Pos);
}

static void ohci_resume(HC_DRV_T *hc)
{
    OHCI_T  *oh = OHCI_OF(hc);

    oh->regs->HcControl = (oh->regs->HcControl & ~USBH_HcControl_HCFS_Msk) | (1 << USBH_HcControl_HCFS_Pos);
    oh->regs->HcControl = (oh->regs->HcControl & ~USBH_HcControl_HCFS_Msk) | (2 << USBH_HcControl_HCFS_Pos);

    for (int i = 0; i < oh->port_cnt; i++)
    {
        if (oh->regs->HcRhPortStatus[i] & 0x4)
            oh->regs->HcRhPortStatus[i] = 0x8;
    }
}

static void ohci_shutdown(HC_DRV_T *hc)
{
    OHCI_T  *oh = OHCI_OF(hc);

    usbh_timer_cancel(&oh->rh_poll_timer);
    ohci_suspend(hc);
    oh->regs->HcInterruptDisable = USBH_HcInterruptDisable_MIE_Msk;
    USB_free(oh->hcca);
#ifndef OHCI_PER_PORT_POWER
    oh->regs->HcRhStatus = USBH_HcRhStatus_LPS_Msk;
#endif
    uint32_t HcControl = oh->regs->HcControl;
    oh->regs->HcControl = HcControl & USBH_HcControl_CBSR_Msk;
}


/*
 *  Quit current trasnfer via UTR or hardware EP.
 */
static int ohci_quit_xfer(HC_DRV_T *hc, UTR_T *utr, EP_INFO_T *ep)
{
    OHCI_T     *oh = OHCI_OF(hc);
    ED_T       *ed;

    if (utr != NULL)
    {
        if (utr->ep == NULL)
            return USBH_ERR_NOT_FOUND;

        ed = (ED_T *)(utr->ep->hw_pipe);

        if (!ed)
            return USBH_ERR_NOT_FOUND;

        /* add the endpoint to remove list, it will be removed on the next start of frame */
        add_to_ED_remove_list(oh, ed);
        utr->ep->hw_pipe = NULL;
    }

    if ((ep != NULL) && (ep->hw_pipe != NULL))
    {
        ed = (ED_T *)(ep->hw_pipe);
        /* add the endpoint to remove list, it will be removed on the next start of frame */
        add_to_ED_remove_list(oh, ed);
        ep->hw_pipe = NULL;
    }

    return 0;
}

uint32_t ed_make_info(UDEV_T *udev, EP_INFO_T *ep)
{
    uint32_t  info;

    if (ep == NULL)                             /* is acontrol endpoint                   */
    {
        /* control endpoint direction is from TD  */
        if (udev->descriptor.bMaxPacketSize0 == 0)   /* is 0 if device descriptor still not obtained. */
        {
            if (udev->speed == SPEED_LOW)       /* give a default maximum packet size     */
                udev->descriptor.bMaxPacketSize0 = 8;
            else
                udev->descriptor.bMaxPacketSize0 = 64;
        }
        info = (udev->descriptor.bMaxPacketSize0 << 16) /* Control endpoint Maximum Packet Size from device descriptor */
               | ED_DIR_BY_TD                   /* Direction (Get direction From TD)      */
               | ED_FORMAT_GENERAL              /* General format                         */
               | (0 << ED_CTRL_EN_Pos);         /* Endpoint address 0                     */
    }
    else                                        /* Other endpoint direction is from endpoint descriptor */
    {
        info = (ep->wMaxPacketSize << 16);      /* Maximum Packet Size from endpoint      */

        info |= ((ep->bEndpointAddress & 0xf) << ED_CTRL_EN_Pos);   /* Enpoint Number     */

        if ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_IN)
            info |= ED_DIR_IN;
        else
            info |= ED_DIR_OUT;

        if ((ep->bmAttributes & EP_ATTR_TT_MASK) == EP_ATTR_TT_ISO)
            info |= ED_FORMAT_ISO;
        else
            info |= ED_FORMAT_GENERAL;
    }

    info |= ((udev->speed == SPEED_LOW)  ? ED_SPEED_LOW : ED_SPEED_FULL);  /* Speed       */
    info |= (udev->dev_num);                  /* Function Address                         */

    return info;
}

static void write_td(TD_T *td, uint32_t info, uint8_t *buff, uint32_t data_len)
{
    td->Info = info;
    td->CBP  = (uint32_t)((!buff || !data_len) ? 0 : virt_to_dma(buff));
    td->BE   = (td->CBP == 0) ? 0 : (uint32_t)(td->CBP + data_len - 1);
    td->buff_start = td->CBP;
    // TD_debug("TD [0x%x]: 0x%x, 0x%x, 0x%x\n", (int)td, td->Info, td->CBP, td->BE);
}

/*
 *  Each device keeps one control ED, linked into the control list with the
 *  others for as long as the device is there. Transfers queue on it behind an
 *  empty TD at the tail, which becomes the next transfer's SETUP TD.
 */
static int ohci_ctrl_xfer(UTR_T *utr)
{
    OHCI_T  *oh = UTR_OHCI(utr);
    UDEV_T  *udev;
    ED_T    *ed;
    TD_T    *td_setup, *td_data, *td_status, *td_tail;
    uint32_t  info;
    int     bIsNewED = 0;

    udev = utr->udev;

    td_data = NULL;
    if (utr->data_len > 0)
    {
        td_data = alloc_ohci_TD(utr);
        if (td_data == NULL)
            return USBH_ERR_MEMORY_OUT;
    }
    td_status = alloc_ohci_TD(utr);
    td_tail = alloc_ohci_TD(NULL);
    if ((td_status == NULL) || (td_tail == NULL))
        goto mem_out;

    ed = (ED_T *)udev->ep0.hw_pipe;
    if (ed == NULL)
    {
        ed = alloc_ohci_ED();
        td_setup = alloc_ohci_TD(NULL);
        if ((ed == NULL) || (td_setup == NULL))
        {
            if (ed)
                free_ohci_ED(ed);
            if (td_setup)
                free_ohci_TD(td_setup);
            goto mem_out;
        }
        ed->Info = ed_make_info(udev, NULL);
        ed->HeadP = ed->TailP = (uint32_t)virt_to_dma(td_setup);
        bIsNewED = 1;
    }
    else if ((ed->HeadP & ~0xF) == (ed->TailP & ~0xF))
    {
        /* Idle, so the address and packet size can follow enumeration */
        ed->Info = (ed->Info & ED_SKIP) | ed_make_info(udev, NULL);
    }
    td_setup = (TD_T *)dma_to_virt((void *)(ed->TailP & ~0xF));
    td_setup->utr = utr;

    /*------------------------------------------------------------------------------------*/
    /* prepare SETUP stage TD                                                             */
    /*------------------------------------------------------------------------------------*/
    info = TD_CC | TD_T_DATA0 | TD_TYPE_CTRL;
    write_td(td_setup, info, (uint8_t *)&utr->setup, 8);
    td_setup->ed = ed;

    /*------------------------------------------------------------------------------------*/
    /* prepare DATA stage TD                                                              */
    /*------------------------------------------------------------------------------------*/
    if (td_data != NULL)
    {
        if ((utr->setup.bmRequestType & 0x80) == REQ_TYPE_OUT)
            info = (TD_CC | TD_R | TD_DP_OUT | TD_T_DATA1 | TD_TYPE_CTRL | TD_CTRL_DATA);
        else
            info = (TD_CC | TD_R | TD_DP_IN | TD_T_DATA1 | TD_TYPE_CTRL | TD_CTRL_DATA);

        write_td(td_data, info, utr->buff, utr->data_len);
        td_data->ed = ed;
        td_setup->NextTD = (uint32_t)virt_to_dma(td_data);
        td_data->NextTD  = (uint32_t)virt_to_dma(td_status);
    }
    else
    {
        td_setup->NextTD = (uint32_t)virt_to_dma(td_status);
    }

    /*------------------------------------------------------------------------------------*/
    /* prepare STATUS stage TD                                                            */
    /*------------------------------------------------------------------------------------*/
    if ((utr->setup.bmRequestType & 0x80) == REQ_TYPE_OUT)
        info = (TD_CC | TD_DP_IN | TD_T_DATA1 | TD_TYPE_CTRL);
    else
        info = (TD_CC | TD_DP_OUT | TD_T_DATA1 | TD_TYPE_CTRL);

    write_td(td_status, info, NULL, 0);
    td_status->ed = ed;
    td_status->NextTD = (uint32_t)virt_to_dma(td_tail);

    td_tail->ed = ed;
    td_tail->NextTD = 0;

    utr->td_cnt = (td_data != NULL) ? 3 : 2;
    utr->xfer_len = 0;
    utr->status = 0;
    utr->ep = &udev->ep0;                   /* driver can find EP from UTR                */
    udev->ep0.hw_pipe = (void *)ed;         /* driver can find ED from EP                 */

    ED_debug("Xfer ED 0x%x: 0x%x 0x%x 0x%x 0x%x\n", (int)ed, ed->Info, ed->TailP, ed->HeadP, ed->NextED);

    /*------------------------------------------------------------------------------------*/
    /*  Start transfer                                                                    */
    /*------------------------------------------------------------------------------------*/
    ed->TailP = (uint32_t)virt_to_dma(td_tail);
    if (bIsNewED)
    {
        ed->NextED = oh->regs->HcControlHeadED;
        oh->regs->HcControlHeadED = (uint32_t)virt_to_dma(ed);
    }
    oh->regs->HcControl |= USBH_HcControl_CLE_Msk;              /* enable control list       */
    oh->regs->HcCommandStatus = USBH_HcCommandStatus_CLF_Msk;   /* start Control list        */

    return 0;

mem_out:
    if (td_data)
        free_ohci_TD(td_data);
    if (td_status)
        free_ohci_TD(td_status);
    if (td_tail)
        free_ohci_TD(td_tail);
    return USBH_ERR_MEMORY_OUT;
}

static int ohci_bulk_xfer(UTR_T *utr)
{
    OHCI_T     *oh = UTR_OHCI(utr);
    UDEV_T     *udev = utr->udev;
    EP_INFO_T  *ep = utr->ep;
    ED_T       *ed;
    TD_T       *td, *td_p, *td_list = NULL;
    uint32_t   info;
    uint32_t   data_len, xfer_len;
    int8_t     bIsNewED = 0;
    uint8_t    *buff;

    /*------------------------------------------------------------------------------------*/
    /*  Check if there's uncompleted transfer on this endpoint...                         */
    /*  Prepare ED                                                                        */
    /*------------------------------------------------------------------------------------*/
    info = ed_make_info(udev, ep);

    /* Check if there's any transfer pending on this endpoint... */
    ed = dma_to_virt((ED_T *)oh->regs->HcBulkHeadED);  /* get the head of bulk endpoint list         */
    while (ed != NULL)
    {
        if (ed->Info == info)               /* have transfer of this EP not completed?    */
        {
            if ((ed->HeadP & 0xFFFFFFF0) != (ed->TailP & 0xFFFFFFF0))
                return USBH_ERR_OHCI_EP_BUSY;     /* endpoint is busy                     */
            else
                break;                      /* ED already there...                        */
        }
        ed = dma_to_virt((ED_T *)ed->NextED);
    }

    if (ed == NULL)
    {
        bIsNewED = 1;
        ed = alloc_ohci_ED();               /* allocate an Endpoint Descriptor            */
        if (ed == NULL)
            return USBH_ERR_MEMORY_OUT;
        ed->Info = info;
        ed->HeadP = 0;
        ED_debug("Link BULK ED 0x%x: 0x%x 0x%x 0x%x 0x%x\n", (int)ed, ed->Info, ed->TailP, ed->HeadP, ed->NextED);
    }

    ep->hw_pipe = (void *)ed;

    /*------------------------------------------------------------------------------------*/
    /*  Prepare TDs                                                                       */
    /*------------------------------------------------------------------------------------*/
    utr->td_cnt = 0;
    data_len = utr->data_len;
    buff = utr->buff;

    do
    {
        if ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_OUT)
            info = (TD_CC | TD_R | TD_DP_OUT | TD_TYPE_BULK);
        else
            info = (TD_CC | TD_R | TD_DP_IN | TD_TYPE_BULK);

        info &= ~(1 << 25);                 /* Data toggle from ED toggleCarry bit        */

        if (data_len > 4096)                /* maximum transfer length is 4K for each TD  */
            xfer_len = 4096;
        else
            xfer_len = data_len;            /* remaining data length < 4K                 */

        td = alloc_ohci_TD(utr);            /* allocate a TD                              */
        if (td == NULL)
            goto mem_out;
        /* fill this TD                               */
        write_td(td, info, buff, xfer_len);
        td->ed = ed;

        utr->td_cnt++;                      /* increase TD count, for recalim counter     */

        buff += xfer_len;                   /* advanced buffer pointer                    */
        data_len -= xfer_len;

        /* chain to end of TD list */
        if (td_list == NULL)
        {
            td_list = td;
        }
        else
        {
            td_p = td_list;
            while (td_p->NextTD != 0)
                td_p = (TD_T *)dma_to_virt((void *)td_p->NextTD);
            td_p->NextTD = (uint32_t)virt_to_dma(td);
        }

    }
    while (data_len > 0);

    /*------------------------------------------------------------------------------------*/
    /*  Start transfer                                                                    */
    /*------------------------------------------------------------------------------------*/
    utr->status = 0;
    ed->HeadP = (ed->HeadP & 0x2) | (uint32_t)virt_to_dma(td_list);       /* keep toggleCarry bit      */
    if (bIsNewED)
    {
        ed->HeadP = (uint32_t)virt_to_dma(td_list);
        /* Link ED to OHCI Bulk List */
        ed->NextED = oh->regs->HcBulkHeadED;
        oh->regs->HcBulkHeadED = (uint32_t)virt_to_dma(ed);
    }
    oh->regs->HcControl |= USBH_HcControl_BLE_Msk;              /* enable bulk list          */
    oh->regs->HcCommandStatus = USBH_HcCommandStatus_BLF_Msk;   /* start bulk list           */

    return 0;

mem_out:
    while (td_list != NULL)
    {
        td = td_list;
        td_list = (TD_T *)dma_to_virt((void *)td_list->NextTD);
        free_ohci_TD(td);
    }
    free_ohci_ED(ed);
    return USBH_ERR_MEMORY_OUT;
}

static int ohci_int_xfer(UTR_T *utr)
{
    OHCI_T     *oh = UTR_OHCI(utr);
    UDEV_T     *udev = utr->udev;
    EP_INFO_T  *ep = utr->ep;
    ED_T       *ed, *ied;
    TD_T       *td, *td_new;
    uint32_t   info;
    int8_t     bIsNewED = 0;

    if (utr->data_len > 64)             /* USB 1.1 interrupt transfer maximum packet size is 64 */
        return USBH_ERR_INVALID_PARAM;

    td_new = alloc_ohci_TD(utr);        /* allocate a TD for dummy TD                     */
    if (td_new == NULL)
        return USBH_ERR_MEMORY_OUT;

    ied = get_int_tree_head_node(oh, ep->bInterval);  /* get head node of this interval       */

    /*------------------------------------------------------------------------------------*/
    /*  Find if this ED was already in the list                                           */
    /*------------------------------------------------------------------------------------*/
    info = ed_make_info(udev, ep);
    ed = ied;
    while (ed != NULL)
    {
        if (ed->Info == info)
            break;                          /* Endpoint found                             */
        ed = dma_to_virt((ED_T *)ed->NextED);
    }

    if (ed == NULL)                         /* ED not found, create it                    */
    {
        bIsNewED = 1;
        ed = alloc_ohci_ED();               /* allocate an Endpoint Descriptor            */
        if (ed == NULL)
            return USBH_ERR_MEMORY_OUT;
        ed->Info = info;
        ed->HeadP = 0;
        ed->bInterval = ep->bInterval;

        td = alloc_ohci_TD(NULL);           /* allocate the initial  dummy TD for ED      */
        if (td == NULL)
        {
            free_ohci_ED(ed);
            free_ohci_TD(td_new);
            return USBH_ERR_MEMORY_OUT;
        }
        ed->HeadP = (uint32_t)virt_to_dma(td);           /* Let both HeadP and TailP point to dummy TD */
        ed->TailP = (uint32_t)virt_to_dma((void *)ed->HeadP);
    }
    else
    {
        td = dma_to_virt((TD_T *)(ed->TailP & ~0xf));    /* TailP always point to the dummy TD         */
    }
    ep->hw_pipe = (void *)ed;

    /*------------------------------------------------------------------------------------*/
    /*  Prepare TD                                                                        */
    /*------------------------------------------------------------------------------------*/
    if ((ep->bEndpointAddress & EP_ADDR_DIR_MASK) == EP_ADDR_DIR_OUT)
        info = (TD_CC | TD_R | TD_DP_OUT | TD_TYPE_INT);
    else
        info = (TD_CC | TD_R | TD_DP_IN | TD_TYPE_INT);

    /* Keep data toggle                               */
    info = (info & ~(1<<25)) | (td->Info & (1<<25));

    /* fill this TD                                   */
    write_td(td, info, utr->buff, utr->data_len);
    td->ed = ed;
    td->NextTD = (uint32_t)virt_to_dma(td_new);
    td->utr = utr;
    utr->td_cnt = 1;                    /* increase TD count, for recalim counter     */
    utr->status = 0;

    /*------------------------------------------------------------------------------------*/
    /*  Hook ED and TD list to HCCA interrupt table                                       */
    /*------------------------------------------------------------------------------------*/

    ed->TailP = (uint32_t)virt_to_dma(td_new);
    if (bIsNewED)
    {
        /* Add to list of the same interval */
        ed->NextED = ied->NextED;
        ied->NextED = (uint32_t)virt_to_dma(ed);
    }


    //printf("Link INT ED 0x%x: 0x%x 0x%x 0x%x 0x%x\n", (int)ed, ed->Info, ed->TailP, ed->HeadP, ed->NextED);

    oh->regs->HcControl |= USBH_HcControl_PLE_Msk;              /* periodic list enable      */
    return 0;
}

static int ohci_iso_xfer(UTR_T *utr)
{
    OHCI_T     *oh = UTR_OHCI(utr);
    UDEV_T     *udev = utr->udev;
    EP_INFO_T  *ep = utr->ep;
    ED_T       *ed, *ied;
    TD_T       *td, *td_list, *last_td;
    int        i;
    uint32_t   info;
    uint32_t   buff_addr;
    int8_t     bIsNewED = 0;

    ied = get_int_tree_head_node(oh, ep->bInterval);  /* get head node of this interval       */

    /*------------------------------------------------------------------------------------*/
    /*  Find if this ED was already in the list                                           */
    /*------------------------------------------------------------------------------------*/
    info = ed_make_info(udev, ep);
    ed = ied;
    while (ed != NULL)
    {
        if (ed->Info == info)
            break;                          /* Endpoint found                             */
        ed = dma_to_virt((ED_T *)ed->NextED);
    }

    if (ed == NULL)                         /* ED not found, create it                    */
    {
        bIsNewED = 1;
        ed = alloc_ohci_ED();               /* allocate an Endpoint Descriptor            */
        if (ed == NULL)
            return USBH_ERR_MEMORY_OUT;
        ed->Info = info;
        ed->HeadP = 0;
        ed->bInterval = ep->bInterval;
    }
    else

        ep->hw_pipe = (void *)ed;

    /*------------------------------------------------------------------------------------*/
    /*  Prepare TDs                                                                       */
    /*------------------------------------------------------------------------------------*/
    if (utr->bIsoNewSched)                   /* Is the starting of isochronous streaming?  */
        ed->next_sf = oh->hcca->frame_no + OHCI_ISO_DELAY;

    utr->td_cnt = 0;
    utr->iso_sf = ed->next_sf;

    last_td = NULL;
    td_list = NULL;

    for (i = 0; i < IF_PER_UTR; i++)
    {
        utr->iso_status[i] = USBH_ERR_NOT_ACCESS1;

        td = alloc_ohci_TD(utr);            /* allocate a TD                              */
        if (td == NULL)
            goto mem_out;
        /* fill this TD                               */
        buff_addr = (uint32_t)(utr->iso_buff[i]);
        td->Info = (TD_CC | TD_TYPE_ISO) | ed->next_sf;
        ed->next_sf += get_ohci_interval(ed->bInterval);
        td->CBP  = (uint32_t)virt_to_dma((void *)(buff_addr & ~0xFFF));
        td->BE   = (uint32_t)virt_to_dma((void *)(buff_addr + utr->iso_xlen[i] - 1));
        td->PSW[0] = 0xE000 | (buff_addr & 0xFFF);

        td->ed = ed;
        utr->td_cnt++;                      /* increase TD count, for recalim counter     */

        /* chain to end of TD list */
        if (td_list == NULL)
            td_list = td;
        else
            last_td->NextTD = (uint32_t)virt_to_dma(td);

        last_td = td;
    };

    /*------------------------------------------------------------------------------------*/
    /*  Hook ED and TD list to HCCA interrupt table                                       */
    /*------------------------------------------------------------------------------------*/
    utr->status = 0;

    if ((ed->HeadP & ~0x3) == 0)
        ed->HeadP = (ed->HeadP & 0x2) | (uint32_t)virt_to_dma(td_list);   /* keep toggleCarry bit      */
    else
    {
        /* find the tail of TDs under this ED */
        td = dma_to_virt((TD_T *)(ed->HeadP & ~0x3));
        while (td->NextTD != 0)
        {
            td = dma_to_virt((TD_T *)td->NextTD);
        }
        td->NextTD = (uint32_t)virt_to_dma(td_list);
    }

    if (bIsNewED)
    {
        /* Add to list of the same interval */
        ed->NextED = ied->NextED;
        ied->NextED = (uint32_t)virt_to_dma(ed);
    }

    ED_debug("Link ISO ED 0x%x: 0x%x 0x%x 0x%x 0x%x\n", (int)ed, ed->Info, ed->TailP, ed->HeadP, ed->NextED);
    oh->regs->HcControl |= USBH_HcControl_PLE_Msk | USBH_HcControl_IE_Msk;  /* enable periodic list and isochronous transfer  */

    return 0;

mem_out:
    while (td_list != NULL)
    {
        td = td_list;
        td_list = dma_to_virt((TD_T *)td_list->NextTD);
        free_ohci_TD(td);
    }
    free_ohci_ED(ed);
    return USBH_ERR_MEMORY_OUT;
}

/*
 *  Root hub port reset. One runs at a time, enumeration only resets one port
 *  at once, and it's polled from a timer until the port comes up enabled.
 */

static void ohci_rh_reset_finish(OHCI_T *oh, int status, int speed)
{
    PORT_RESET_DONE  done = oh->rh_reset.done;
    void             *context = oh->rh_reset.context;

    oh->rh_reset.active = 0;
    oh->rh_reset.done = NULL;
    if (done)
        done(status, speed, context);
}

static void ohci_rh_reset_timer(void *context)
{
    OHCI_T    *oh = (OHCI_T *)context;
    uint32_t  sts;
    int       port = oh->rh_reset.port;

    if (!oh->rh_reset.active)
        return;

    sts = oh->regs->HcRhPortStatus[port];

    if ((sts & USBH_HcRhPortStatus_CCS_Msk) == 0)
    {
        ohci_rh_reset_finish(oh, USBH_ERR_DISCONNECTED, 0);
        return;
    }

    if (((sts & USBH_HcRhPortStatus_PRS_Msk) == 0) && (sts & USBH_HcRhPortStatus_PES_Msk))
    {
        oh->regs->HcRhPortStatus[port] = USBH_HcRhPortStatus_PRSC_Msk;
        ohci_rh_reset_finish(oh, USBH_OK, (sts & USBH_HcRhPortStatus_LSDA_Msk) ? SPEED_LOW : SPEED_FULL);
        return;
    }

    oh->rh_reset.elapsed += 10;
    if (oh->rh_reset.elapsed >= PORT_RESET_TIME_MS + oh->rh_reset.retry * PORT_RESET_RETRY_INC_MS)
    {
        if (++oh->rh_reset.retry >= PORT_RESET_RETRY)
        {
            USB_debug("OHCI port %d - port reset failed!\n", port+1);
            ohci_rh_reset_finish(oh, USBH_ERR_PORT_RESET, 0);
            return;
        }
        oh->rh_reset.elapsed = 0;
        oh->regs->HcRhPortStatus[port] = USBH_HcRhPortStatus_PRS_Msk;
    }
    usbh_timer_start(&oh->rh_reset.timer, 10);
}

static int ohci_rh_port_reset(HC_DRV_T *hc, int port, PORT_RESET_DONE done, void *context)
{
    OHCI_T  *oh = OHCI_OF(hc);

    if (oh->rh_reset.active || (port < 0) || (port >= oh->port_cnt))
        return USBH_ERR_PORT_RESET;

    oh->rh_reset.active = 1;
    oh->rh_reset.port = port;
    oh->rh_reset.retry = 0;
    oh->rh_reset.elapsed = 0;
    oh->rh_reset.done = done;
    oh->rh_reset.context = context;

    oh->regs->HcRhPortStatus[port] = USBH_HcRhPortStatus_PRS_Msk;
    usbh_timer_start(&oh->rh_reset.timer, 10);
    return 0;
}

static void ohci_rh_port_reset_cancel(HC_DRV_T *hc)
{
    OHCI_T  *oh = OHCI_OF(hc);

    usbh_timer_cancel(&oh->rh_reset.timer);
    oh->rh_reset.active = 0;
    oh->rh_reset.done = NULL;
}

/* From the root hub status change interrupt */
static void ohci_rh_change(OHCI_T *oh)
{
    uint32_t  sts;
    int       i;

    if (oh->regs->HcRhStatus & USBH_HcRhStatus_OCIC_Msk)
    {
        ohci_clear_overcurrent(oh);
#ifndef OHCI_PER_PORT_POWER
        oh->regs->HcRhStatus = USBH_HcRhStatus_LPSC_Msk;
#endif
    }

    for (i = 0; i < oh->port_cnt; i++)
    {
        /* clear unwanted port change status */
        oh->regs->HcRhPortStatus[i] = USBH_HcRhPortStatus_OCIC_Msk | USBH_HcRhPortStatus_PSSC_Msk |
                                   USBH_HcRhPortStatus_PESC_Msk;

        sts = oh->regs->HcRhPortStatus[i];
        if ((sts & USBH_HcRhPortStatus_CSC_Msk) == 0)
            continue;

        oh->regs->HcRhPortStatus[i] = USBH_HcRhPortStatus_CSC_Msk;
        USB_debug("OHCI port%d status change: 0x%x\n", i+1, sts);

        if (sts & USBH_HcRhPortStatus_CCS_Msk)
            usbh_rh_port_connected(&oh->drv, i+1);
        else
            usbh_rh_port_disconnected(&oh->drv, i+1);
    }
}

static void ohci_rh_poll_timer(void *context)
{
    OHCI_T  *oh = (OHCI_T *)context;

    ohci_rh_change(oh);
    usbh_timer_start(&oh->rh_poll_timer, 50);
}

static void ohci_rh_start(HC_DRV_T *hc)
{
    OHCI_T  *oh = OHCI_OF(hc);
    uint32_t  sts;
    int       i;

    /* A device that was already there doesn't always leave a connect change */
    for (i = 0; i < oh->port_cnt; i++)
    {
        sts = oh->regs->HcRhPortStatus[i];
        oh->regs->HcRhPortStatus[i] = USBH_HcRhPortStatus_CSC_Msk | USBH_HcRhPortStatus_PESC_Msk |
                                   USBH_HcRhPortStatus_PSSC_Msk | USBH_HcRhPortStatus_OCIC_Msk |
                                   USBH_HcRhPortStatus_PRSC_Msk;
        if (sts & USBH_HcRhPortStatus_CCS_Msk)
            usbh_rh_port_connected(&oh->drv, i+1);
    }

    /* The second controller's ports past the ones it reports have change bits
     * that can't be cleared and keep the status change set, so it's polled */
    oh->regs->HcInterruptStatus = USBH_HcInterruptStatus_RHSC_Msk;
    if (oh->regs->HcInterruptStatus & USBH_HcInterruptStatus_RHSC_Msk)
    {
        usbh_timer_start(&oh->rh_poll_timer, 50);
        return;
    }

    oh->regs->HcInterruptEnable = USBH_HcInterruptEnable_RHSC_Msk;
}

static void td_done(TD_T *td)
{
    UTR_T       *utr = td->utr;
    uint32_t    info;
    int         cc;

    info = td->Info;

    TD_debug("td_done: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", (int)td, td->Info, td->CBP, td->NextTD, td->BE);

    /* ISO ... drivers see per-TD length/status */
    if ((info & TD_TYPE_Msk) == TD_TYPE_ISO)
    {
        uint16_t    sf;
        int         idx;

        sf = info & 0xFFFF;
        idx = ((sf + 0x10000 - utr->iso_sf) & 0xFFFF) / get_ohci_interval(td->ed->bInterval);
        if (idx >= IF_PER_UTR)
        {
            USB_error("ISO invalid index!! %d, %d\n", sf, utr->iso_sf);
            goto td_out;
        }

        cc = (td->PSW[0] >> 12) & 0xF;
        if (cc == 0xF)                      /* this frame was not transferred             */
        {
            USB_debug("ISO F %d N/A!\n", sf);
            utr->iso_status[idx] = USBH_ERR_SCH_OVERRUN;
            goto td_out;
        }
        if ((cc != 0) && (cc != CC_DATA_UNDERRUN))
        {
            utr->iso_status[idx] = USBH_ERR_CC_NO_ERR - cc;
            goto td_out;
        }
        utr->iso_status[idx] = 0;
        utr->iso_xlen[idx] = td->PSW[0] & 0x7FF;
    }
    else
    {
        cc = TD_CC_GET(info);

        /* short packet is fine */
        if ((cc != CC_NOERROR) && (cc != CC_DATA_UNDERRUN))
        {
            USB_error("TD error, CC = 0x%x\n", cc);
            utr->status = USBH_ERR_TRANSFER;
        }

        switch (info & TD_TYPE_Msk)
        {
        case TD_TYPE_CTRL:
            if (info & TD_CTRL_DATA)
            {
                if (td->CBP == 0)
                    utr->xfer_len += td->BE - td->buff_start + 1;
                else
                    utr->xfer_len += td->CBP - td->buff_start;
            }
            break;

        case TD_TYPE_BULK:
        case TD_TYPE_INT:
            if (td->CBP == 0)
                utr->xfer_len += td->BE - td->buff_start + 1;
            else
                utr->xfer_len += td->CBP - td->buff_start;
            break;
        }
    }

td_out:

    utr->td_cnt--;

    /* If all TDs are done, call-back to requester. */
    if (utr->td_cnt == 0)
    {
        utr->bIsTransferDone = 1;
        if (utr->func)
            utr->func(utr);
    }
}

/* Unlinks an ED the controller has skipped since the last start of frame */
static void unlink_ed(OHCI_T *oh, ED_T *ed_p)
{
    ED_T      *ed, *ied;

    if ((ed_p->Info & ED_EP_ADDR_Msk) == 0)
    {
        if (oh->regs->HcControlHeadED == (uint32_t)virt_to_dma(ed_p))
        {
            oh->regs->HcControlHeadED = (uint32_t)ed_p->NextED;
            return;
        }
        ed = (ED_T *)dma_to_virt((void *)oh->regs->HcControlHeadED);
    }
    else if (ed_p->bInterval > 0)
    {
        ied = get_int_tree_head_node(oh, ed_p->bInterval);
        ed = ied;
    }
    else
    {
        if (oh->regs->HcBulkHeadED == (uint32_t)virt_to_dma(ed_p))
        {
            oh->regs->HcBulkHeadED = ed_p->NextED;
            return;
        }
        ed = (ED_T *)dma_to_virt((void *)oh->regs->HcBulkHeadED);
    }

    while (ed != NULL)
    {
        if (dma_to_virt((void *)ed->NextED) == ed_p)
        {
            ed->NextED = ed_p->NextED;
            return;
        }
        ed = (ED_T *)dma_to_virt((void *)ed->NextED);
    }
}

/* in IRQ context */
static void remove_ed(OHCI_T *oh)
{
    ED_T      *list, *ed_p;
    TD_T      *td, *td_next;
    UTR_T     *utr;

    /* Ones added while completing these wait for the next start of frame */
    list = oh->ed_remove_list;
    oh->ed_remove_list = NULL;

    while (list != NULL)
    {
        ed_p = list;
        list = ed_p->next;
        ED_debug("Remove ED: 0x%x, %d\n", (int)ed_p, ed_p->bInterval);

        unlink_ed(oh, ed_p);

        /*
         *  Complete the transfers still on it as aborted, and free its TDs,
         *  the empty one at the tail included.
         */
        td = (TD_T *)dma_to_virt((void *)(ed_p->HeadP & ~0xF));
        while (td != NULL)
        {
            utr = td->utr;
            td_next = (TD_T *)dma_to_virt((void *)(td->NextTD & ~0xF));
            free_ohci_TD(td);
            td = td_next;

            if (utr != NULL)
            {
                utr->td_cnt--;
                if (utr->td_cnt == 0)
                {
                    utr->status = USBH_ERR_ABORT;
                    utr->bIsTransferDone = 1;
                    if (utr->func)
                        utr->func(utr);
                }
            }
        }
        free_ohci_ED(ed_p);
    }
}

/*
 *  A TD that failed halts its ED, with the rest of the same transfer still
 *  on it. Those are taken off and the ED restarted, so the transfer completes
 *  with the error and anything queued after it goes ahead.
 */
static void flush_halted_ed(OHCI_T *oh, TD_T *failed)
{
    ED_T    *ed = failed->ed;
    UTR_T   *utr = failed->utr;
    TD_T    *td, *td_next, *tail;

    if ((ed == NULL) || (utr == NULL) || ((ed->HeadP & ED_HEADP_HALT) == 0))
        return;

    tail = (TD_T *)dma_to_virt((void *)(ed->TailP & ~0xF));
    td = (TD_T *)dma_to_virt((void *)(ed->HeadP & ~0xF));
    while ((td != NULL) && (td != tail) && (td->utr == utr))
    {
        td_next = (TD_T *)dma_to_virt((void *)(td->NextTD & ~0xF));
        free_ohci_TD(td);
        utr->td_cnt--;
        td = td_next;
    }

    /* Keep the toggle carry, drop the halt */
    ed->HeadP = (ed->HeadP & 0x2) | (uint32_t)virt_to_dma(td);

    if ((ed->Info & ED_EP_ADDR_Msk) == 0)
        oh->regs->HcCommandStatus = USBH_HcCommandStatus_CLF_Msk;
    else if (ed->bInterval == 0)
        oh->regs->HcCommandStatus = USBH_HcCommandStatus_BLF_Msk;
}

//static irqreturn_t ohci_irq (struct usb_hcd *hcd)
void OHCI_IRQHandler(int index)
{
    OHCI_T     *oh = &_ohci_hc[index];
    TD_T       *td, *td_prev, *td_next;
    uint32_t   int_sts, cc;

    int_sts = oh->regs->HcInterruptStatus;

    //USB_debug("ohci int_sts = 0x%x\n", int_sts);

    /* Done TDs go first, while the EDs they came from are still there */
    if (int_sts & USBH_HcInterruptStatus_WDH_Msk)
    {
        int_sts &= ~USBH_HcInterruptStatus_WDH_Msk;
        /*
         *  reverse done list
         */
        td = dma_to_virt((TD_T *)(oh->hcca->done_head & TD_ADDR_MASK));
        oh->hcca->done_head = 0;
        td_prev = NULL;
        oh->regs->HcInterruptStatus = USBH_HcInterruptStatus_WDH_Msk;

        while (td != NULL)
        {
            //TD_debug("Done list TD 0x%x => 0x%x\n", (int)td, (int)td->NextTD);
            td_next = dma_to_virt((TD_T *)(td->NextTD & TD_ADDR_MASK));
            td->NextTD = (uint32_t)virt_to_dma(td_prev);
            td_prev = td;
            td = td_next;
        }
        td = td_prev;               /* first TD of the reversed done list */

        /*
         *  reclaim TDs
         */
        while (td != NULL)
        {
            TD_debug("Reclaim TD 0x%x, next 0x%x\n", (int)td, td->NextTD);
            td_next = dma_to_virt((TD_T *)td->NextTD);
            cc = TD_CC_GET(td->Info);
            if (((td->Info & TD_TYPE_Msk) != TD_TYPE_ISO) &&
                    (cc != CC_NOERROR) && (cc != CC_DATA_UNDERRUN))
                flush_halted_ed(oh, td);
            td_done(td);
            free_ohci_TD(td);
            td = td_next;
        }
    }

    if ((oh->regs->HcInterruptEnable & USBH_HcInterruptEnable_SF_Msk) &&
            (int_sts & USBH_HcInterruptStatus_SF_Msk))
    {
        int_sts &= ~USBH_HcInterruptStatus_SF_Msk;

        oh->regs->HcInterruptDisable = USBH_HcInterruptDisable_SF_Msk;
        oh->regs->HcInterruptStatus = USBH_HcInterruptStatus_SF_Msk;
        remove_ed(oh);
    }

    if ((oh->regs->HcInterruptEnable & USBH_HcInterruptEnable_RHSC_Msk) &&
            (int_sts & USBH_HcInterruptStatus_RHSC_Msk))
    {
        oh->regs->HcInterruptStatus = USBH_HcInterruptStatus_RHSC_Msk;
        int_sts &= ~USBH_HcInterruptStatus_RHSC_Msk;
        ohci_rh_change(oh);
    }

    oh->regs->HcInterruptStatus = int_sts;
}

#ifdef ENABLE_DEBUG_MSG

void dump_ohci_int_table(HC_DRV_T *hc)
{
    OHCI_T *oh = OHCI_OF(hc);
    int    i;
    ED_T   *ed;

//    for (i = 0; i < 32; i++)
    for (i = 0; i < 1; i++)

    {
        USB_debug("%02d: ", i);

        ed = dma_to_virt((ED_T *)oh->hcca->int_table[i]);

        while (ed != NULL)
        {
            USB_debug("0x%x (0x%x) => ", (int)ed, ed->HeadP);
            ed = dma_to_virt((ED_T *)ed->NextED);
        }
        sysprintf("0\n");
    }
}

void dump_ohci_regs(HC_DRV_T *hc)
{
    OHCI_T *oh = OHCI_OF(hc);

    USB_debug("Dump OCHI registers:\n");
    USB_debug("    HcRevision         = 0x%x\n", oh->regs->HcRevision);
    USB_debug("    HcControl          = 0x%x\n", oh->regs->HcControl);
    USB_debug("    HcCommandStatus    = 0x%x\n", oh->regs->HcCommandStatus);
    USB_debug("    HcInterruptStatus  = 0x%x\n", oh->regs->HcInterruptStatus);
    USB_debug("    HcInterruptEnable  = 0x%x\n", oh->regs->HcInterruptEnable);
    USB_debug("    HcInterruptDisable = 0x%x\n", oh->regs->HcInterruptDisable);
    USB_debug("    HcHCCA             = 0x%x\n", oh->regs->HcHCCA);
    USB_debug("    HcPeriodCurrentED  = 0x%x\n", oh->regs->HcPeriodCurrentED);
    USB_debug("    HcControlHeadED    = 0x%x\n", oh->regs->HcControlHeadED);
    USB_debug("    HcControlCurrentED = 0x%x\n", oh->regs->HcControlCurrentED);
    USB_debug("    HcBulkHeadED       = 0x%x\n", oh->regs->HcBulkHeadED);
    USB_debug("    HcBulkCurrentED    = 0x%x\n", oh->regs->HcBulkCurrentED);
    USB_debug("    HcDoneHead         = 0x%x\n", oh->regs->HcDoneHead);
    USB_debug("    HcFmInterval       = 0x%x\n", oh->regs->HcFmInterval);
    USB_debug("    HcFmRemaining      = 0x%x\n", oh->regs->HcFmRemaining);
    USB_debug("    HcFmNumber         = 0x%x\n", oh->regs->HcFmNumber);
    USB_debug("    HcPeriodicStart    = 0x%x\n", oh->regs->HcPeriodicStart);
    USB_debug("    HcLSThreshold      = 0x%x\n", oh->regs->HcLSThreshold);
    USB_debug("    HcRhDescriptorA    = 0x%x\n", oh->regs->HcRhDescriptorA);
    USB_debug("    HcRhDescriptorB    = 0x%x\n", oh->regs->HcRhDescriptorB);
    USB_debug("    HcRhStatus         = 0x%x\n", oh->regs->HcRhStatus);
    for (int i = 0; i < oh->port_cnt; i++)
    {
        USB_debug("    HcRhPortStatus0    = 0x%x\n", oh->regs->HcRhPortStatus[i]);
    }
    USB_debug("    HcPhyControl       = 0x%x\n", oh->regs->HcPhyControl);
    USB_debug("    HcMiscControl      = 0x%x\n", oh->regs->HcMiscControl);
}

void dump_ohci_ports(HC_DRV_T *hc)
{
    OHCI_T *oh = OHCI_OF(hc);

    USB_debug("_ohci port0=0x%x, port1=0x%x\n", oh->regs->HcRhPortStatus[0], oh->regs->HcRhPortStatus[1]);
}

#endif  // ENABLE_DEBUG_MSG

static const HC_DRV_T  ohci_driver =
{
    ohci_init,               /* init               */
    ohci_shutdown,           /* shutdown           */
    ohci_suspend,            /* suspend            */
    ohci_resume,             /* resume             */
    ohci_ctrl_xfer,          /* ctrl_xfer          */
    ohci_bulk_xfer,          /* bulk_xfer          */
    ohci_int_xfer,           /* int_xfer           */
    ohci_iso_xfer,           /* iso_xfer           */
    ohci_quit_xfer,          /* quit_xfer          */
    ohci_rh_port_reset,          /* rthub_port_reset        */
    ohci_rh_port_reset_cancel,   /* rthub_port_reset_cancel */
    ohci_rh_start                /* rthub_start             */
};

HC_DRV_T * ohci_get_hc(int index)
{
    OHCI_T  *oh;

    if ((index < 0) || (index >= OHCI_HC_CNT))
        return NULL;

    oh = &_ohci_hc[index];
    if (oh->regs == NULL)
    {
        oh->drv = ohci_driver;
        oh->drv.index = index;
        oh->regs = USBH_BASE(index);
    }
    return &oh->drv;
}

/// @endcond HIDDEN_SYMBOLS

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
