/**************************************************************************//**
 * @file     support.c
 * @version  V1.10
 * $Revision: 11 $
 * $Date: 14/10/03 1:54p $
 * @brief  Functions to support USB host driver.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>

#include "usb.h"

/// @cond HIDDEN_SYMBOLS


#ifndef USB_MEM_BLOCK_SIZE
#define USB_MEM_BLOCK_SIZE 256
#endif

/* The pool starts with one chunk of USB_MEMORY_POOL_SIZE and grows by
 * chunks of at least USB_MEMORY_POOL_GROW_SIZE when it runs out. A chunk
 * added later is released again once nothing in it is allocated. */
#ifndef USB_MEMORY_POOL_SIZE
#define USB_MEMORY_POOL_SIZE (32*1024)
#endif

#ifndef USB_MEMORY_POOL_GROW_SIZE
#define USB_MEMORY_POOL_GROW_SIZE (16*1024)
#endif

#ifndef USB_MEMORY_POOL_MAX_CHUNKS
#define USB_MEMORY_POOL_MAX_CHUNKS 16
#endif


static uint32_t  _FreeMemorySize;
uint32_t  _AllocatedMemorySize;


#define USB_MEM_ALLOC_MAGIC     0x19685788    /* magic number in leading block */

typedef struct USB_mhdr
{
    uint32_t  flag;  /* 0:free, 1:allocated, 0x3:first block */
    uint32_t  bcnt;  /* if allocated, the block count of allocated memory block */
    uint32_t  magic;
    uint32_t  reserved;
}  USB_MHDR_T;

/* Data right after the header in the leading block is aligned to this.
 * A bigger boundary starts the data at the next block instead. */
#define  BOUNDARY_WORD          sizeof(USB_MHDR_T)

typedef struct USB_mchunk
{
    uint8_t     *mem;        /* as returned by usbh_allocate_memory_pool, NULL if unused */
    uint32_t    base;
    uint32_t    end;
    USB_MHDR_T  *current;
    uint32_t    free_size;
}  USB_MCHUNK_T;

static USB_MCHUNK_T  _chunks[USB_MEMORY_POOL_MAX_CHUNKS];


static int  chunk_create(USB_MCHUNK_T *chunk, uint32_t size)
{
    chunk->mem = usbh_allocate_memory_pool(size, USB_MEM_BLOCK_SIZE);
    if (chunk->mem == NULL)
        return 0;
    chunk->base = (UINT32)chunk->mem | NON_CACHE_MASK;
    chunk->end = chunk->base + size;
    chunk->current = (USB_MHDR_T *)chunk->base;
    chunk->free_size = size;
    memset((char *)chunk->base, 0, size);
    _FreeMemorySize += size;
    return 1;
}

static void  chunk_destroy(USB_MCHUNK_T *chunk)
{
    _FreeMemorySize -= chunk->free_size;
    usbh_free_memory_pool(chunk->mem);
    memset(chunk, 0, sizeof(*chunk));
}

static void  *chunk_alloc(USB_MCHUNK_T *chunk, INT wanted_size, INT boundary)
{
    USB_MHDR_T  *pPrimitivePos;
    USB_MHDR_T  *pFound = NULL;
    INT   found_size = -1;
    INT   i, block_count = 0;
    INT   wrap = 0;

    if ((uint32_t)wanted_size >= chunk->free_size)
        return NULL;

    if ((UINT32)chunk->current >= chunk->end)
        chunk->current = (USB_MHDR_T *)chunk->base;   /* wrapped */
    pPrimitivePos = chunk->current;

    do
    {
        if (chunk->current->flag)          /* is not a free block */
        {
            if (chunk->current->magic != USB_MEM_ALLOC_MAGIC)
            {
                sysprintf("\nUSB_malloc - incorrect magic number! C:%x F:%x, wanted:%d, Base:0x%x, End:0x%x\n", (UINT32)chunk->current, chunk->free_size, wanted_size, chunk->base, chunk->end);
                return NULL;
            }

            if (chunk->current->flag == 0x3)
                chunk->current = (USB_MHDR_T *)((UINT32)chunk->current + chunk->current->bcnt * USB_MEM_BLOCK_SIZE);
            else
            {
                sysprintf("USB_malloc warning - not the first block!\n");
                chunk->current = (USB_MHDR_T *)((UINT32)chunk->current + USB_MEM_BLOCK_SIZE);
            }

            if ((UINT32)chunk->current > chunk->end)
                sysprintf("USB_malloc - behind limit!!\n");

            if ((UINT32)chunk->current >= chunk->end)
            {
                wrap = 1;
                chunk->current = (USB_MHDR_T *)chunk->base;   /* wrapped */
            }

            found_size = -1;          /* reset the accumlator */
        }
        else                         /* is a free block */
        {
            if (found_size == -1)     /* the leading block */
            {
                pFound = chunk->current;
                block_count = 1;

                if (boundary > (INT)BOUNDARY_WORD)
                    found_size = 0;    /* not use the data area of the leading block */
                else
                    found_size = USB_MEM_BLOCK_SIZE - sizeof(USB_MHDR_T);

                /* check boundary -
                 * If boundary > BOUNDARY_WORD, the start of next block should
                 * be the beginning address of allocated memory. Thus, we check
                 * the boundary of the next block. The leading block will be
                 * used as a header only.
                 */
                if ((boundary > (INT)BOUNDARY_WORD) &&
                        ((((UINT32)chunk->current)+USB_MEM_BLOCK_SIZE >= chunk->end) ||
                         ((((UINT32)chunk->current)+USB_MEM_BLOCK_SIZE) % boundary != 0)))
                    found_size = -1;   /* violate boundary, reset the accumlator */
            }
            else                      /* not the leading block */
            {
                found_size += USB_MEM_BLOCK_SIZE;
                block_count++;
            }

            if (found_size >= wanted_size)
            {
                pFound->bcnt = block_count;
                pFound->magic = USB_MEM_ALLOC_MAGIC;
                chunk->free_size -= block_count * USB_MEM_BLOCK_SIZE;
                _FreeMemorySize -= block_count * USB_MEM_BLOCK_SIZE;
                _AllocatedMemorySize += block_count * USB_MEM_BLOCK_SIZE;
                chunk->current = pFound;
                for (i=0; i<block_count; i++)
                {
                    chunk->current->flag = 1;     /* allocate block */
                    chunk->current = (USB_MHDR_T *)((UINT32)chunk->current + USB_MEM_BLOCK_SIZE);
                }
                pFound->flag = 0x3;

                if (boundary > (INT)BOUNDARY_WORD)
                    return (void *)((UINT32)pFound + USB_MEM_BLOCK_SIZE);
                else
                    return (void *)((UINT32)pFound + sizeof(USB_MHDR_T));
            }

            /* advance to the next block */
            chunk->current = (USB_MHDR_T *)((UINT32)chunk->current + USB_MEM_BLOCK_SIZE);
            if ((UINT32)chunk->current >= chunk->end)
            {
                wrap = 1;
                chunk->current = (USB_MHDR_T *)chunk->base;   /* wrapped */
                found_size = -1;     /* reset accumlator */
            }
        }
    }
    while ((wrap == 0) || (chunk->current < pPrimitivePos));

    return NULL;
}

static USB_MCHUNK_T  *chunk_from_address(UINT32 addr)
{
    INT  i;

    for (i = 0; i < USB_MEMORY_POOL_MAX_CHUNKS; i++)
    {
        if (_chunks[i].mem && (addr >= _chunks[i].base) && (addr < _chunks[i].end))
            return &_chunks[i];
    }
    return NULL;
}


void  USB_InitializeMemoryPool()
{
    memset(_chunks, 0, sizeof(_chunks));
    _FreeMemorySize = 0;
    _AllocatedMemorySize = 0;
    if (!chunk_create(&_chunks[0], USB_MEMORY_POOL_SIZE))
        assert(0);
}

void USB_UninitializeMemoryPool()
{
    INT  i;

    for (i = 0; i < USB_MEMORY_POOL_MAX_CHUNKS; i++)
    {
        if (_chunks[i].mem)
            chunk_destroy(&_chunks[i]);
    }
    _FreeMemorySize = 0;
    _AllocatedMemorySize = 0;
}


int  USB_available_memory()
{
    return _FreeMemorySize;
}


int  USB_allocated_memory()
{
    return _AllocatedMemorySize;
}


void  *USB_malloc(INT wanted_size, INT boundary)
{
    void      *p = NULL;
    uint32_t  size;
    INT       i;
    int       disable_ohci_irq, disable_ehci_irq;

    if (IS_OHCI_IRQ_ENABLED())
        disable_ohci_irq = 1;
    else
        disable_ohci_irq = 0;

    if (IS_EHCI_IRQ_ENABLED())
        disable_ehci_irq = 1;
    else
        disable_ehci_irq = 0;

    if (disable_ohci_irq)
        DISABLE_OHCI_IRQ();
    if (disable_ehci_irq)
        DISABLE_EHCI_IRQ();

    for (i = 0; (i < USB_MEMORY_POOL_MAX_CHUNKS) && (p == NULL); i++)
    {
        if (_chunks[i].mem)
            p = chunk_alloc(&_chunks[i], wanted_size, boundary);
    }

    if (p == NULL)
    {
        /* room for the data, its leading block and the boundary */
        size = ((uint32_t)wanted_size + 2 * USB_MEM_BLOCK_SIZE + USB_MEM_BLOCK_SIZE - 1) & ~(USB_MEM_BLOCK_SIZE - 1);
        if (size < USB_MEMORY_POOL_GROW_SIZE)
            size = USB_MEMORY_POOL_GROW_SIZE;

        for (i = 0; i < USB_MEMORY_POOL_MAX_CHUNKS; i++)
        {
            if (_chunks[i].mem == NULL)
            {
                if (chunk_create(&_chunks[i], size))
                    p = chunk_alloc(&_chunks[i], wanted_size, boundary);
                break;
            }
        }
    }

    if (p == NULL)
        sysprintf("USB_malloc - No free memory! want=%d, free=%d\n", wanted_size, _FreeMemorySize);

    if (disable_ohci_irq)
        ENABLE_OHCI_IRQ();
    if (disable_ehci_irq)
        ENABLE_EHCI_IRQ();
    return p;
}


void  USB_free(void *alloc_addr)
{
    USB_MCHUNK_T  *chunk;
    USB_MHDR_T  *pMblk;
    UINT32  addr = (UINT32)alloc_addr;
    INT     i, count;
    int     disable_ohci_irq, disable_ehci_irq;

    if (IS_OHCI_IRQ_ENABLED())
        disable_ohci_irq = 1;
    else
        disable_ohci_irq = 0;

    if (IS_EHCI_IRQ_ENABLED())
        disable_ehci_irq = 1;
    else
        disable_ehci_irq = 0;

    //sysprintf("USB_free: 0x%x\n", (int)alloc_addr);

    if (disable_ohci_irq)
        DISABLE_OHCI_IRQ();
    if (disable_ehci_irq)
        DISABLE_EHCI_IRQ();

    chunk = chunk_from_address(addr);
    if (chunk == NULL)
    {
        if (disable_ohci_irq)
            ENABLE_OHCI_IRQ();
        if (disable_ehci_irq)
            ENABLE_EHCI_IRQ();
        if (addr)
            free(alloc_addr);
        return;
    }

    //sysprintf("USB_free:%x\n", (INT)addr+USB_MEM_BLOCK_SIZE);

    /* get the leading block address */
    if (addr % USB_MEM_BLOCK_SIZE == 0)
        addr -= USB_MEM_BLOCK_SIZE;
    else
        addr -= sizeof(USB_MHDR_T);

    if (addr % USB_MEM_BLOCK_SIZE != 0)
    {
        sysprintf("USB_free fatal error on address: %x!!\n", (UINT32)alloc_addr);
        if (disable_ohci_irq)
            ENABLE_OHCI_IRQ();
        if (disable_ehci_irq)
            ENABLE_EHCI_IRQ();
        return;
    }

    pMblk = (USB_MHDR_T *)addr;
    if (pMblk->flag == 0)
    {
        sysprintf("USB_free(), warning - try to free a free block: %x\n", (UINT32)alloc_addr);
        if (disable_ohci_irq)
            ENABLE_OHCI_IRQ();
        if (disable_ehci_irq)
            ENABLE_EHCI_IRQ();
        return;
    }
    if (pMblk->magic != USB_MEM_ALLOC_MAGIC)
    {
        sysprintf("USB_free(), warning - try to free an unknow block at address:%x.\n", addr);
        if (disable_ohci_irq)
            ENABLE_OHCI_IRQ();
        if (disable_ehci_irq)
            ENABLE_EHCI_IRQ();
        return;
    }

    //sysprintf("+ 0x%x, %d\n", (int)pMblk, pMblk->bcnt);

    count = pMblk->bcnt;
    for (i = 0; i < count; i++)
    {
        pMblk->flag = 0;     /* release block */
        pMblk = (USB_MHDR_T *)((UINT32)pMblk + USB_MEM_BLOCK_SIZE);
    }

    chunk->free_size += count * USB_MEM_BLOCK_SIZE;
    _FreeMemorySize += count * USB_MEM_BLOCK_SIZE;
    _AllocatedMemorySize -= count * USB_MEM_BLOCK_SIZE;

    /* the first chunk stays, a grown one goes back once it's empty */
    if ((chunk != &_chunks[0]) && (chunk->free_size == chunk->end - chunk->base))
        chunk_destroy(chunk);

    if (disable_ohci_irq)
        ENABLE_OHCI_IRQ();
    if (disable_ehci_irq)
        ENABLE_EHCI_IRQ();
    return;
}

//Tick in 10ms blocks
uint32_t get_ticks(void)
{
    return usbh_get_ticks();
}

void delay_us(int usec)
{
    usbh_delay_us(usec);
}

void *dma_to_virt(void *physical_address)
{
    return usbh_dma_to_virt(physical_address);
}

void *virt_to_dma(void *virtual_address)
{
    return usbh_virt_to_dma(virtual_address);
}

void sysprintf(const char *format, ...)
{
#ifdef ENABLE_DEBUG_MSG
    char buffer[512];
    va_list argList;
    va_start(argList, format);
    vsnprintf(buffer, sizeof(buffer), format, argList);
    va_end(argList);
    usbh_sysprintf(buffer);
#endif
}


/// @endcond HIDDEN_SYMBOLS

