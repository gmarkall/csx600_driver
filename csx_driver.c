//      Copyright (c) ClearSpeed Technology plc 2000,2002,2006,2007
//      All rights reserved.
//
//   This file is part of the ClearSpeed Open Source Driver
//
//   The ClearSpeed Open Source Driver is free software; you can redistribute it
//   and/or modify it under the terms of the GNU General Public License as
//   published by the Free Software Foundation; either version 2 of the License,
//   or (at your option) any later version.
//
//   The ClearSpeed Open Source Driver is distributed in the hope that it will
//   be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
//   Public License for more details.
//
//   You should have received a copy of the GNU General Public License along
//   with The ClearSpeed Open Source Driver; if not, write to the Free Software
//   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//

/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2006 Silicon Graphics, Inc.  All Rights Reserved.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/version.h>
#include <asm/atomic.h>
#include <asm/page.h>

/* Description of user side interface, shared with user space. */
#include "csx_driver.h"

#define CSX_DEBUG 0
#define CSX_DEBUG_IOCTLS 0
#define CSX_DEBUG_EXTREME 0

/* set to 1 to get kernel mapping for memory aperture and read/write syscall to it */
/*
   NB you cannot afford the kernel vmalloc space with more than 1 card
   and the read/write syscalls are not required
 */
#define CS_NOMAPMEM 0

#if defined (CONFIG_ARCH_IA64_SN2) || defined (CONFIG_ARCH_IA64_GENERIC)
#define MMIOWB() do { mmiowb(); } while (0)
#else
#define MMIOWB()
#endif

/* Prefix to use for name of /proc file entry */
#define PROC_PREFIX  driver/
#define istr(s) # s
#define str(s) istr(s)

/******************************
 * Definitions and data types *
 ******************************/

#define PCI_VENDOR_ID_CLEARSPEED 0x1942
#define PCI_DEVICE_ID_CLEARSPEED_CSX_E511 0xe511
#define PCI_DEVICE_ID_CLEARSPEED_CSX_E521 0xe521
#define PCI_DEVICE_ID_CLEARSPEED_CSX_E701 0xe701
#define PCI_DEVICE_ID_CLEARSPEED_CSX_E711 0xe711

#define DMA_MAX_CHANNELS 2
#define DMA_MAX_BUFFERS 2

#define CSX_CTL_REGSIZE sizeof(uint32_t)

#define CSX_CTL_DEVNUM	0
#define CSX_MEM_DEVNUM	1
#define CSX_NUM_DEVS	2	/* Number of char special devices per card */


///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Register map defs
//  Given that we're not C++ here we need to define a big list and look them all up by group using
//  indexes.
//
///////////////////////////////////////////////////////////////////////////////////////////////////

// All the different reg set groups
#define REGSET_NULL                     0
#define REGSET_PCIX                     1
#define REGSET_DIOCLES                  2
#define REGSET_CALLANISH                3

// A list of all the indexes which we might want to look up
#define REG_INDEX_NULL                  0
#define REG_INDEX_HIF_VERSION           1
#define REG_INDEX_PCI_INT_STATUS        2
#define REG_INDEX_PCI_INT_MASK          3
#define REG_INDEX_DMA_INT_STATUS        4
#define REG_INDEX_DMA_INT_MASK          5
#define REG_INDEX_HBDMA_BYTECOUNT       6       // Halfbridge ones must be in order
#define REG_INDEX_HBDMA_CONTROL         7
#define REG_INDEX_HBDMA_FPGA_ADDR_LO    8
#define REG_INDEX_HBDMA_FPGA_ADDR_HI    9
#define REG_INDEX_HBDMA_PCI_ADDR_LO     10
#define REG_INDEX_HBDMA_PCI_ADDR_HI     11
#define REG_INDEX_HBDMA_LINK_LO         12
#define REG_INDEX_HBDMA_LINK_HI         13
#define REG_INDEX_DMA_CTRL_STATUS       16
#define REG_INDEX_DMA_START_ADDR_HI     17
#define REG_INDEX_DMA_START_ADDR_LO     18
#define REG_INDEX_DMA_SEMWAIT_HI        19
#define REG_INDEX_DMA_SEMWAIT_LO        20
#define REG_INDEX_DMA_SEMSIG_HI         21
#define REG_INDEX_DMA_SEMSIG_LO         22
#define REG_INDEX_GIU2_STATUS           23
#define REG_INDEX_GIU2_MASK0            24
#define REG_INDEX_GIU2_MASK1            25
#define REG_INDEX_DMA_CHANNELS          26
#define REG_INDEX_INT_CTRL              27
#define REG_INDEX_DMA_MAX_HOST_REQ_SIZE 28
#define REG_INDEX_DMA_MAX_SUP_HOST_SIZE 29


struct RegSetEntry
{
    unsigned int        regset;
    unsigned int        channel;        // or 0 if not part of DMA
    unsigned int        index;
    unsigned int        address;
};

static const struct RegSetEntry reglist[] = {
    { REGSET_PCIX,      0,  REG_INDEX_HIF_VERSION,          0x00080000 },
    { REGSET_PCIX,      0,  REG_INDEX_PCI_INT_STATUS,       0x00000000 },
    { REGSET_PCIX,      0,  REG_INDEX_DMA_INT_STATUS,       0x00000004 },
    { REGSET_PCIX,      0,  REG_INDEX_PCI_INT_MASK,         0x00000010 },
    { REGSET_PCIX,      0,  REG_INDEX_DMA_INT_MASK,         0x00000014 },
    { REGSET_PCIX,      0,  REG_INDEX_HBDMA_BYTECOUNT,      0x00001000 },
    { REGSET_PCIX,      0,  REG_INDEX_HBDMA_CONTROL,        0x00001004 },
    { REGSET_PCIX,      0,  REG_INDEX_HBDMA_FPGA_ADDR_LO,   0x00001008 },
    { REGSET_PCIX,      0,  REG_INDEX_HBDMA_FPGA_ADDR_HI,   0x0000100C },
    { REGSET_PCIX,      0,  REG_INDEX_HBDMA_PCI_ADDR_LO,    0x00001010 },
    { REGSET_PCIX,      0,  REG_INDEX_HBDMA_PCI_ADDR_HI,    0x00001014 },
    { REGSET_PCIX,      0,  REG_INDEX_HBDMA_LINK_LO,        0x00001018 },
    { REGSET_PCIX,      0,  REG_INDEX_HBDMA_LINK_HI,        0x0000101C },

    { REGSET_DIOCLES,   0,  REG_INDEX_HIF_VERSION,          0x00080000 },
    { REGSET_DIOCLES,   0,  REG_INDEX_INT_CTRL,             0x00082800 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_MAX_SUP_HOST_SIZE,0x000C0200 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_MAX_HOST_REQ_SIZE,0x000C0500 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_INT_STATUS,       0x000C0B00 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_INT_MASK,         0x000C0C00 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_CTRL_STATUS,      0x000C0D00 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_START_ADDR_HI,    0x000C0E00 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_START_ADDR_LO,    0x000C0F00 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_SEMWAIT_HI,       0x000C1000 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_SEMWAIT_LO,       0x000C1100 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_SEMSIG_HI,        0x000C1200 },
    { REGSET_DIOCLES,   0,  REG_INDEX_DMA_SEMSIG_LO,        0x000C1300 },
    { REGSET_DIOCLES,   0,  REG_INDEX_GIU2_STATUS,          0x00042000 },
    { REGSET_DIOCLES,   0,  REG_INDEX_GIU2_MASK0,           0x00042100 },
    { REGSET_DIOCLES,   0,  REG_INDEX_GIU2_MASK1,           0x00042200 },

    { REGSET_CALLANISH, 0,  REG_INDEX_HIF_VERSION,          0x00004000 },
    { REGSET_CALLANISH, 0,  REG_INDEX_INT_CTRL,             0x00004200 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_MAX_SUP_HOST_SIZE,0x00002080 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_MAX_HOST_REQ_SIZE,0x00002140 },
    { REGSET_CALLANISH, 0,  REG_INDEX_GIU2_STATUS,          0x00000800 },
    { REGSET_CALLANISH, 0,  REG_INDEX_GIU2_MASK0,           0x00000840 },
    { REGSET_CALLANISH, 0,  REG_INDEX_GIU2_MASK1,           0x00000880 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_CHANNELS,         0x000020C0 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_INT_STATUS,       0x000022C0 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_INT_MASK,         0x00002300 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_CTRL_STATUS,      0x00002380 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_START_ADDR_HI,    0x000023C0 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_START_ADDR_LO,    0x00002400 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_SEMWAIT_HI,       0x00002440 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_SEMWAIT_LO,       0x00002480 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_SEMSIG_HI,        0x000024C0 },
    { REGSET_CALLANISH, 0,  REG_INDEX_DMA_SEMSIG_LO,        0x00002500 },
    { REGSET_CALLANISH, 1,  REG_INDEX_DMA_INT_STATUS,       0x00002540 },
    { REGSET_CALLANISH, 1,  REG_INDEX_DMA_INT_MASK,         0x00002580 },
    { REGSET_CALLANISH, 1,  REG_INDEX_DMA_CTRL_STATUS,      0x00002600 },
    { REGSET_CALLANISH, 1,  REG_INDEX_DMA_START_ADDR_HI,    0x00002640 },
    { REGSET_CALLANISH, 1,  REG_INDEX_DMA_START_ADDR_LO,    0x00002680 },
    { REGSET_CALLANISH, 1,  REG_INDEX_DMA_SEMWAIT_HI,       0x000026C0 },
    { REGSET_CALLANISH, 1,  REG_INDEX_DMA_SEMWAIT_LO,       0x00002700 },
    { REGSET_CALLANISH, 1,  REG_INDEX_DMA_SEMSIG_HI,        0x00002740 },
    { REGSET_CALLANISH, 1,  REG_INDEX_DMA_SEMSIG_LO,        0x00002780 },

    { REGSET_NULL, 0, 0, 0 }
};


// PCIX register values & masks
#define CSX_PCIX_CTL_PCIINTR_MASK       0x1             /* MTAP generated interrupts */
#define CSX_PCIX_CTL_DMAINTR_MASK       0x7             /* DMA engine interrupts */

// PCIe register values & masks
#define CSX_PCIE_INT_GIU2_DMA_MASK_DIOCLES        0x10
#define CSX_PCIE_INT_GIU2_DMA_MASK_CALLANISH      0x800
#define CSX_PCIE_INT_DMA_MASK           0x70F3          /* DMA Status bits that indicate that the DMA engine has stopped */

/*
    RegMap Unit: HIF_ELBI_DMA_PCIE - PCI-express DMA controller registers:
    interrupt_status0 - DMA channel Sticky interrupt status bits: - from Jeff

       bit	interrupt	default
    0	xfer_done		n/a
    1	cpl_ep			fatal
    2	cpl_dllp_abort	non-fatal
    3	cpl_tlp_abort 	non-fatal
    4	cpl_ecrc_err 	fatal
    5	align_err		fatal
    6	cmd_err 		fatal
    7	cpl_timeout 	fatal
    8	soft_reset		n/a
    9	soft_error_E1	non-fatal
    10	soft_error_E2	non-fatal
    11	soft_error_C1	non-fatal
    12	cpl_status_ur	fatal
    13	cpl_status_crs	fatal
    14	cpl_status_ca	fatal
    15	unexpected_cpl	non-fatal
*/



/* initially copied from st.c. */
struct csx_dma_region
{
	struct scatterlist *sglist[DMA_MAX_BUFFERS];        /* IOMMU mapping */
	unsigned int pages_pinned[DMA_MAX_BUFFERS];	        /* Actual # to pci_map_sg (nents) */
	enum dma_data_direction direction;                  /* Direction of DMA transfer */

    /* Information on the coherent descriptor mapping */
	dma_addr_t desc_bus_address[DMA_MAX_BUFFERS];	    /* pci bus address for device. */
	void *desc_kaddr;	                                /* kernel map address for descriptor buffer. */
	unsigned desc_buf_size;	                            /* Size of the coherent descriptor buffer. */
};

/* All the driver-tracked information about this device instance */
struct csx_device_data;
struct csx_device_stats; 

struct csx_ctl_data
{
	struct csx_device_data *maindata;
	dev_t dev;
	struct cdev cdev;
	struct class_device classdev;
	atomic_t intr_count;
	wait_queue_head_t intr_queue;
	unsigned long bar;	/* PCI address map */
	unsigned long len;
	uint32_t *regs;	/* from ioremap_nocache */
    struct csx_device_stats * stats; /* Per connection performance information */
};

struct csx_mem_data
{
	struct csx_device_data *maindata;
	dev_t dev;
	struct cdev cdev;
	struct class_device classdev;
	atomic_t intr_count0;	// DMA channel 0
	atomic_t intr_count1;	// DMA channel 1
	wait_queue_head_t intr_queue;
	unsigned long bar;	/* PCI address map */
	unsigned long len;
	unsigned char *mem;	/* from ioremap_nocache */
    unsigned long dma_num_channels;
	struct csx_dma_region region[DMA_MAX_CHANNELS];
    struct csx_device_stats * stats; /* Per connect performance information */
};

struct csx_device_data
{
	struct pci_dev *pcidev;
	dev_t dev_base;
	struct csx_ctl_data *ctl;
	struct csx_mem_data *mem;
    int pci_dev_id;
};

/* Per device performance information. This is linked to from both csx_mem_data and csx_ctl_data for convenience but for control
   operations (housekeeping) we associate it with the ctl device. */
struct csx_device_stats
{
    unsigned pid;  /* Owning process id, zero if not in use */
    int      devid; /* device id, useful in construcing the name to remove entry */
    unsigned kbytes_to_device;
    unsigned kbytes_from_device;
    atomic_t interrupts_taken;
    unsigned temp_cpu0;
    unsigned temp_cpu1;
};

/* Note: to get to the generic dev handle do: &xxx->pcidev.dev */

/* Per-file data for control register device */
struct csx_ctl_file
{
	__s32 intr_count;
};

/* Per-file data for memory device */
struct csx_mem_file
{
	__s32 intr_count0;	// DMA channel 0
	__s32 intr_count1;	// DMA channel 1
};

/***************
 * Module data *
 ***************/

static dev_t firstdev;		/* First dynamic device number */
static dev_t nextdev;		/* Next available dynamic device number */
static spinlock_t nextdev_lock;

struct semaphore ioctl_sem; /* locks the ioctl code */


/***************
 * MSI Control *
 ***************/

//#define CSX_MSI_ENABLE /*default off */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8))
static inline void pci_disable_msi(struct pci_dev* dev) {}
#endif

#ifdef CSX_MSI_ENABLE
/* Controls whether MSI should be used or not, default to false, don't use.  */
static int csx_use_msi = 0;
module_param(csx_use_msi, int, S_IRUGO);
#endif

/*******************
 * Register access *
 *******************/

unsigned int RegSetFromID( struct csx_device_data *p_dev )
{
    switch( p_dev->pci_dev_id )
    {
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E511:
        return REGSET_PCIX;

    case PCI_DEVICE_ID_CLEARSPEED_CSX_E521:
        return REGSET_DIOCLES;

    case PCI_DEVICE_ID_CLEARSPEED_CSX_E701:
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E711:
        return REGSET_CALLANISH;

    default:
        printk(KERN_WARNING "%s: pci id not known (0x%x)\n", __FUNCTION__, p_dev->pci_dev_id );
        return REGSET_NULL;
    }
}


void csx_write_register( struct csx_ctl_data *p_ctl, unsigned int regset,
    unsigned int dma_channel, unsigned int index, uint32_t val )
{
    const struct RegSetEntry *p_regentry;

    // Look up register address based on indicees.

    p_regentry = reglist;
    while( p_regentry->regset != REGSET_NULL )
    {
        if( p_regentry->regset == regset &&
            p_regentry->channel == dma_channel &&
            p_regentry->index == index )
        {
            writel( val, &p_ctl->regs[ p_regentry->address >> 2 ] ); 
            return;
        }

        p_regentry++;
    }
    printk(KERN_WARNING "%s: Register not found ! Regset = %d, chan = %d, index = %d, val = %d\n", __FUNCTION__, regset, dma_channel, index, val );
}


void csx_read_register( struct csx_ctl_data *p_ctl, unsigned int regset,
    unsigned int dma_channel, unsigned int index, uint32_t *p_val )
{
    const struct RegSetEntry *p_regentry;

    // Look up register address based on indicees.

    p_regentry = reglist;
    while( p_regentry->regset != REGSET_NULL )
    {
        if( p_regentry->regset == regset &&
            p_regentry->channel == dma_channel &&
            p_regentry->index == index )
        {
            *p_val = readl( &p_ctl->regs[ p_regentry->address >> 2 ] ); 
            return;
        }

        p_regentry++;
    }
    printk(KERN_WARNING "%s: Register not found ! Regset = %d, chan = %d, index = %d\n", __FUNCTION__, regset, dma_channel, index );
}

/***************
 * Helper Funcs *
 ***************/

void pcie_program_request_size( struct csx_device_data *cdd, enum dma_data_direction direction )
{
    int read_size;
    int write_size;
    int max_supported_host_req_size; // this is the maximum that the DMA engine can cope with
    int supported_read_size;
    int supported_write_size;
    u8 val;
    u8 max_read_req_size = 0;
    u8 max_payload_size = 0;

    csx_read_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_DMA_MAX_SUP_HOST_SIZE, &max_supported_host_req_size);
    supported_read_size  = max_supported_host_req_size & 0xffff;

    if( cdd->pci_dev_id == PCI_DEVICE_ID_CLEARSPEED_CSX_E521 )
    {
        supported_write_size = supported_read_size;
    }
    else
    {
        supported_write_size = max_supported_host_req_size >> 16;
    }

    // Card is reading from host
    pci_read_config_byte( cdd->pcidev, 0x79, &val );
    max_read_req_size = (val & 0x70) >> 4;
    read_size = 128 << max_read_req_size;
    if( read_size >  supported_read_size)
    {
        read_size =  supported_read_size;
    }
      
    // Card is writing to host
    pci_read_config_byte( cdd->pcidev, 0x78, &val ); 
    max_payload_size = (val & 0xe0) >> 5;
    write_size = 128 << max_payload_size;   
    if( write_size >  supported_write_size)
    {
        write_size =  supported_write_size;
    }

    // Program the DMA transfer size
    switch( cdd->pci_dev_id )
    {  
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E521:
        if( direction == DMA_TO_DEVICE )
        {
            csx_write_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_DMA_MAX_HOST_REQ_SIZE, read_size );
        }
        else
        {    
            csx_write_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_DMA_MAX_HOST_REQ_SIZE, write_size );
        }        

        break;
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E701:
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E711: 
        csx_write_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_DMA_MAX_HOST_REQ_SIZE, ((write_size << 16) | read_size) );
        //csx_write_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_DMA_MAX_HOST_REQ_SIZE, ((write_size << 16) | 0x200) );
        break;
    default:
	    printk(KERN_WARNING "%s: pci id not known (0x%x)\n", __FUNCTION__, cdd->pci_dev_id );
        break;
    }
}

/******************
 * Class routines *
 ******************/

/* Frees per-instance device data when all references to it go away. */
static void csx_ctl_class_release(struct class_device *classdev)
{
	struct csx_ctl_data *ccd;

	ccd = container_of(classdev, struct csx_ctl_data, classdev);

	cdev_del(&ccd->cdev);
	iounmap(ccd->regs);
	release_mem_region(ccd->bar, ccd->len);
	kfree(ccd);
}

static struct class csx_ctl_class =
{
	.name = "csxctl",
	.release = csx_ctl_class_release,
};

/* Frees per-instance device data when all references to it go away. */
static void csx_mem_class_release(struct class_device *classdev)
{
	struct csx_mem_data *cmd;
	struct csx_device_data *cdd;
    int chan;

	cmd = container_of(classdev, struct csx_mem_data, classdev);

	cdd = cmd->maindata;

	cdev_del(&cmd->cdev);
#if CS_NOMAPMEM
	iounmap(cmd->mem);
#endif
	release_mem_region(cmd->bar, cmd->len);

    for( chan=0; chan<DMA_MAX_CHANNELS; chan++ )
    {
	    dma_free_coherent(&cdd->pcidev->dev, cdd->mem->region[chan].desc_buf_size * 2,
			cdd->mem->region[chan].desc_kaddr, cdd->mem->region[chan].desc_bus_address[0]);
    }

	kfree(cmd);
}

static struct class csx_mem_class = {
	.name = "csxmem",
	.release = csx_mem_class_release,
};

/*********************
 * Interrupt handler *
 *********************/

/* Determines which type(s) of interrupt were triggered, and wakes
 * up the appropriate waiters.
 */
static irqreturn_t csx_intr_handler(int irq, void *arg, struct pt_regs *regs)
{
	struct csx_device_data *cdd;
	uint32_t pcistatus, dmastatus, giu2_mask0, giu2_mask1, giu2_status;
	irqreturn_t ret = IRQ_NONE;

    cdd = (struct csx_device_data *) arg;

    switch( cdd->pci_dev_id )
    {
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E511:
        csx_read_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_PCI_INT_STATUS, &pcistatus );
        csx_read_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_DMA_INT_STATUS, &dmastatus );

	    if (pcistatus & CSX_PCIX_CTL_PCIINTR_MASK)
        {
		    uint32_t pcimask;
		    atomic_inc(&cdd->ctl->intr_count);
		    // unmask the interrupt - user code will sort it later
            csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_PCI_INT_MASK, 0 );
		    MMIOWB();
		    // need some sort of memory barrier on x86 to avoid spurious interrupts
            csx_read_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_PCI_INT_MASK, &pcimask );
		    wake_up_all(&cdd->ctl->intr_queue);
		    ret = IRQ_HANDLED;
            atomic_inc(&cdd->ctl->stats->interrupts_taken);
	    }

	    if (dmastatus & CSX_PCIX_CTL_DMAINTR_MASK)
        {
		    atomic_inc(&cdd->mem->intr_count0);
		    /* Ack the interrupt, leave the mask alone */
		    // dma is done by kernel so this is different from pci interrupt
            csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_DMA_INT_STATUS, dmastatus );
		    MMIOWB();
		    // need some sort of memory barrier on x86 to avoid spurious interrupts
            csx_read_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_DMA_INT_STATUS, &dmastatus );
		    wake_up_all(&cdd->mem->intr_queue);
		    ret = IRQ_HANDLED;
            atomic_inc(&cdd->ctl->stats->interrupts_taken);
	    }
        break;

    case PCI_DEVICE_ID_CLEARSPEED_CSX_E521:
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E701:
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E711:
        csx_read_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_GIU2_STATUS, &giu2_status );
        csx_read_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_GIU2_MASK0, &giu2_mask0 );
        csx_read_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_GIU2_MASK1, &giu2_mask1 );

        if( giu2_status )
        {
            if( giu2_status & giu2_mask0 )        // Anything not DMA int bit:
            {
                // Mask out all of the non DMA interrupts on mask0.
                csx_write_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_GIU2_MASK0, 0 );
		        MMIOWB();
		        // need some sort of memory barrier on x86 to avoid spurious interrupts
                csx_read_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_GIU2_MASK0, &giu2_mask0 );

                if( giu2_mask0 )
                {
        	        printk(KERN_WARNING "%s: GIU2 mask0 failed to clear non DMA int bits\n", __FUNCTION__ );
                }

		        atomic_inc(&cdd->ctl->intr_count);
		        wake_up_all(&cdd->ctl->intr_queue);
		        ret = IRQ_HANDLED;
                atomic_inc(&cdd->ctl->stats->interrupts_taken);
            }
			
			// Check GIU2 interrupt status (DMA only):
            if( giu2_status & giu2_mask1 )	// just DMA int bit on & mask set? (could be disabled while handling other channel)
            {								// (mask enabled in csx_fire_pcie_dma(), re-enabled in llpci handler; cleared below)
                int chan;
                int found = 0;
				
				// check all DMA channels:
                for( chan=0; chan < cdd->mem->dma_num_channels; chan++ )
                {
					// read DMA status register for each channel:
                    csx_read_register( cdd->ctl, RegSetFromID( cdd ), chan, REG_INDEX_DMA_INT_STATUS, &dmastatus );
					//csx_read_register( cdd->ctl, RegSetFromID( cdd ), chan, REG_INDEX_DMA_INT_MASK, &dmamask );
					
					// Check PCIE interrupt mask & status
	                //if (dmastatus & dmamask)		// if the status & mask for this channel has bits set:
                    if (dmastatus & CSX_PCIE_INT_DMA_MASK)  // if the status & mask for this channel has bits set:
                    {								// (mask is set by csx_fire_pcie_dma() & cleared below)

						// count interrupt detected for each channel separately:
						switch(chan)
						{
						case 0: atomic_inc(&cdd->mem->intr_count0);
								break;
						case 1: atomic_inc(&cdd->mem->intr_count1);
								break;
						default: printk(KERN_WARNING "%s: DMA channel %d not supported\n", __FUNCTION__ , chan );
								break;
						};
						
						found = 1;

		                // Clear the mask for this channel - llpci will check and clear status:
						// (needed to prevent /this channel/ interrupt re-firing - this mask is ONLY set in csx_fire_pcie_dma() )

                        //csx_write_register( cdd->ctl, RegSetFromID( cdd ), chan, REG_INDEX_DMA_INT_MASK, 0 );

                        /*
		                MMIOWB();	// need some sort of memory barrier on x86 to avoid spurious interrupts - Itanium only?
						//wmb();		// memory barrier to make sure write takes place before read
                        csx_read_register( cdd->ctl, RegSetFromID( cdd ), chan, REG_INDEX_DMA_INT_MASK, &dmastatus );
                        if( dmastatus )
                        {
        	                printk(KERN_WARNING "%s: failed to clear DMA interrupt mask for chan %d\n", __FUNCTION__, chan );
                        }
                        */

                        atomic_inc(&cdd->ctl->stats->interrupts_taken);     // debug info for /proc entry

						ret = IRQ_HANDLED;
					}
                }

				if(found == 1)
				{
					// Disable all DMA interrupts at GIU2 level (prevents any more DMA interrupts for a while):
					// ( this mask is enabled in csx_fire_pcie_dma() AND re-enabled in llpci, 
					//	 when DMA interrupt is successfully handled - for other channel )
					// ( N.B. could be re-enabled by csx_fire_pcie_dma() for other channel /before/ this one is handled - ho hum )
                    
					csx_write_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_GIU2_MASK1, 0 );

                    /*
					MMIOWB();	// need some sort of memory barrier on x86 to avoid spurious interrupts - Itanium only?
					csx_read_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_GIU2_MASK1, &giu2_mask1 );
					if( giu2_mask1 )
					{
						printk(KERN_WARNING "%s: failed to clear gui2_mask1 DMA interrupt mask\n", __FUNCTION__ );
					}
					*/
					wake_up_all(&cdd->mem->intr_queue);		// wake up poll()

					ret = IRQ_HANDLED;
					atomic_inc(&cdd->ctl->stats->interrupts_taken);
				}

            }	// if( giu2_status & giu2_mask1 )

            if( ret == IRQ_HANDLED )
            {
                // If we handled any interrupts then we should tell the PCIe core 
				// that we have done "end of interrupt" which potentially resends the int message.
                csx_write_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_INT_CTRL, 1 );
            }
        }
        break;

    default:
	    printk(KERN_WARNING "%s: pci id not known (0x%x)\n", __FUNCTION__, cdd->pci_dev_id );
        break;
    }

	return ret;
}

/****************
 * Entry points *
 ****************/

/* Control register operations */

/* Seek to a particular position in the control register aperture */
static loff_t csx_ctl_llseek(struct file *filp, loff_t off, int whence)
{
	struct csx_ctl_data *ccd;
	loff_t pos;

	ccd = container_of(filp->f_dentry->d_inode->i_cdev,
			   struct csx_ctl_data, cdev);

	switch (whence) {
		case 0:	/* SEEK_SET */
			pos = off;
			break;
		case 1:	/* SEEK_CUR */
			pos = filp->f_pos + off;
			break;
		case 2:	/* SEEK_END */
			pos = ccd->len + off;
			break;
		default:
			return -EINVAL;
	}

	/* Verify seek does not land outside control register area */
	if (pos < 0 || pos > ccd->len)
    {
		return -EINVAL;
    }

	filp->f_pos = pos;
	return pos;
}

/* Memory-map the control register aperture
 * XXX: On SGI Altix, the user mapping is unreliable before Linux 2.6.17.
 */
static int csx_ctl_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct csx_ctl_data *ccd;
	unsigned long off;
	unsigned long vsize;
	int ret;

	ccd = container_of(filp->f_dentry->d_inode->i_cdev, struct csx_ctl_data, cdev);

	/* Verify request does not extend past end of control register aperture. */
	off = vma->vm_pgoff << PAGE_SHIFT;
	vsize = vma->vm_end - vma->vm_start;
	if (off + vsize > ccd->len)
    {
		return -EINVAL;
    }

	/* Ensure mapping is suitable for IO */
	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

#if LINUX_VERSION_CODE <=  KERNEL_VERSION(2,6,12) 	 
    ret = io_remap_page_range(vma, vma->vm_start,
        ccd->bar + off,
        vsize, vma->vm_page_prot);
#else 	 
     ret = remap_pfn_range(vma, vma->vm_start, (unsigned long)(ccd->bar+off)>>PAGE_SHIFT, 	 
        vsize, vma->vm_page_prot); 	 
#endif 	 
 
	if (ret)
		return -EAGAIN;

	return 0;
}

/* Open the control register aperture */
static int
csx_ctl_open(struct inode *inode, struct file *filp)
{
	struct csx_ctl_data *ccd;
	struct csx_ctl_file *ccf;

	ccd = container_of(inode->i_cdev, struct csx_ctl_data, cdev);
#if CSX_DEBUG
	printk(KERN_DEBUG "%s: ccd %p\n", __FUNCTION__, ccd);
#endif

	ccf = kmalloc(sizeof(struct csx_ctl_file), GFP_KERNEL);
	if (NULL == ccf)
		return -ENOMEM;

	filp->private_data = ccf;
	atomic_set(&ccd->intr_count, 0);
	ccf->intr_count = atomic_read(&ccd->intr_count);

	MMIOWB();
    return 0;
}


/* Wait for a PCI interrupt (i.e. FPGA program or status/fault interrupt) */
/*
	the pci interrupt mask must be opened in user code before the poll
	- this is a different strategy to the dma interrupt
 */
static unsigned int
csx_ctl_poll(struct file *filp, struct poll_table_struct *polltab)
{
	struct csx_ctl_data *ccd;
	struct csx_ctl_file *ccf;
	__s32 sample;

	ccd = container_of(filp->f_dentry->d_inode->i_cdev,
		struct csx_ctl_data, cdev);
	ccf = filp->private_data;

	poll_wait(filp, &ccd->intr_queue, polltab);

	/* CSX poll implements a wait for interrupt */
	sample = atomic_read(&ccd->intr_count);
	if (ccf->intr_count != sample)
	{
		ccf->intr_count = sample;
		return (POLLIN | POLLRDNORM);
	}
	return 0;
}

/* Read control register */
static ssize_t
csx_ctl_read(struct file *filp, char __user * data, size_t size, loff_t * off)
{
	uint32_t val;
	struct csx_ctl_data *ccd;
	size_t index;

	ccd = container_of(filp->f_dentry->d_inode->i_cdev,
			   struct csx_ctl_data, cdev);

	/* Verify size and offset are correct multiples of register size */
	if ((size & (CSX_CTL_REGSIZE - 1)) || (*off & (CSX_CTL_REGSIZE - 1)))
		return -EINVAL;

	/* Verify access doesn't fall outside control register area */
	if ((*off < 0) || (*off + size > ccd->len))
		return -EINVAL;

	for (index = 0; index < size / CSX_CTL_REGSIZE; index += CSX_CTL_REGSIZE)
    {
		/* Using intermediate value allows unaligned user data, and checks against user space pointer */
		val = readl((void *)((size_t)(ccd->regs) + (size_t)*off + index));

        // TODO SGE - What the hell is this ?
		if (unlikely
		    (copy_to_user(&data[index], &val, CSX_CTL_REGSIZE)))
			return -EFAULT;
	}

	*off += size;
	return size;
}

/* Release per-file resources for control register aperture */
static int
csx_ctl_release(struct inode *inode, struct file *filp)
{
	struct csx_ctl_file *ccf;
	struct csx_ctl_data *ccd;

    ccd = container_of(inode->i_cdev, struct csx_ctl_data, cdev);

	MMIOWB();
	ccf = filp->private_data;
	kfree(ccf);
	return 0;
}

/* Write control register */
static ssize_t
csx_ctl_write(struct file *filp, const char __user * data, size_t size,
	      loff_t * off)
{
	uint32_t val;
	struct csx_ctl_data *ccd;
	size_t index;

	ccd = container_of(filp->f_dentry->d_inode->i_cdev,
			   struct csx_ctl_data, cdev);

	/* Verify size and offset are correct multiples of register size */
	if ((size & (CSX_CTL_REGSIZE - 1)) || (*off & (CSX_CTL_REGSIZE - 1)))
		return -EINVAL;

	/* Verify access doesn't fall outside control register area */
	if ((*off < 0) || (*off + size > ccd->len))
		return -EINVAL;

	for (index = 0; index < size / CSX_CTL_REGSIZE;
	     index += CSX_CTL_REGSIZE) {
		/* Using intermediate value allows unaligned user data,
		 * and checks against user space pointer
		 */
		if (unlikely(copy_from_user(&val, &data[index],
					    CSX_CTL_REGSIZE))) {
			MMIOWB();
			return -EFAULT;
		}
		writel(val, (void *)((size_t)(ccd->regs) + (size_t)*off + index));
	}

	/* Ensure writes arrive at hardware before proceeding */
	MMIOWB();

	*off += size;
	return size;
}

struct file_operations csx_ctl_fops = {
	.owner = THIS_MODULE,
	.llseek = csx_ctl_llseek,
	.mmap = csx_ctl_mmap,
	.open = csx_ctl_open,
	.poll = csx_ctl_poll,
	.read = csx_ctl_read,
	.release = csx_ctl_release,
	.write = csx_ctl_write,
};


/* Memory operations */

/* DMA routines */

/* On buffer locking, we need to create a scatter/gather list and other
 * information.  We cannot rely on the user giving us back the required
 * information so we record this in a structure in the driver.
 */

static int
sgl_map_user_pages(struct scatterlist *sgl, const unsigned int max_pages,
		   unsigned long user_addr, size_t count,
		   enum dma_data_direction direction)
{
	int res, i, j;
	struct page **pages;
	const unsigned long end =
		(user_addr + count + PAGE_SIZE - 1) >> PAGE_SHIFT;
	const unsigned long start = user_addr >> PAGE_SHIFT;
	const int nr_pages = end - start;
	const int read_from_card = (direction == DMA_FROM_DEVICE ? 1 : 0);
        
	unsigned int offset; 
        unsigned int length;

	if( sgl == NULL )
	{
		return -EINVAL;
	}

	/* User attempted Overflow! */
	if ((user_addr + count) < user_addr)
	{
		return -EINVAL;
	}

	/* Too big */
	if (nr_pages > max_pages)
	{
		return -ENOMEM;
	}

	/* Hmm? */
	if (count == 0)
	{
		return 0;
	}

	// This memory allocation doesn't live beyond the end of this function no specific cleanup needed.
	if ((pages = kmalloc(max_pages * sizeof(*pages), GFP_KERNEL)) == NULL)
	{
		return -ENOMEM;
	}

	memset(pages, 0, max_pages * sizeof(*pages));

#if CSX_DEBUG
	printk(KERN_DEBUG "%s: calling get_user_pages ua:%p, nr_pages %d, %s max_pages=%d\n",
		__FUNCTION__, (void *) user_addr, nr_pages, (read_from_card ? "read" : "write"), max_pages);
#endif

	/* Try to fault in all of the necessary pages */
	down_read(&current->mm->mmap_sem);

	/* read_from_card means reading from board */
	// Which means write to memory = 1
	res = get_user_pages(current, current->mm, user_addr, nr_pages, read_from_card, 0, pages, NULL);

	up_read(&current->mm->mmap_sem);

#if CSX_DEBUG
	printk(KERN_DEBUG "%s: get_user_pages return %d\n", __FUNCTION__, res);
#endif

	/* Errors and no page mapped should return here */
	if (res < nr_pages)
	{
        goto out_unmap; /* Consider this an error */
	}

//	for (i = 0; i < nr_pages; i++)
//	{
		/* FIXME: flush superflous for rw==READ,
		 * probably wrong function for rw==WRITE
		 */
		/*     flush_dcache_page(pages[i]); QQQ */
//	}

	/* Populate the scatter/gather list */
        offset = user_addr & ~PAGE_MASK;
	
	if (nr_pages > 1)
	{
                length = PAGE_SIZE - offset;
                sg_set_page(&sgl[0], pages[0], length, offset);
		count -= sgl[0].length;

		for (i = 1; i < nr_pages; i++)
		{
			sg_set_page(&sgl[i], pages[i], count < PAGE_SIZE ? count : PAGE_SIZE, 0);
			count -= PAGE_SIZE;

#if CSX_DEBUG
			if( (count < 0) && (i != (nr_pages-1)) )
			{
				printk(KERN_DEBUG "%s: ASSERT FAILURE: count going negaticve with more pages to go ?!\n", __FUNCTION__);
			}
#endif
		}
	}
	else
	{
                length = count;
                sg_set_page(&sgl[0], pages[0], length, offset);

#if CSX_DEBUG
		if( (sgl[0].offset + sgl[0].length) > PAGE_SIZE )
		{
			printk(KERN_DEBUG "%s: ASSERT FAILURE: sg offset + size > page_size ?!\n", __FUNCTION__);
		}
#endif
	}

	kfree(pages);
	return nr_pages;

out_unmap:
	if (res > 0)
	{
		for (j = 0; j < res; j++)
		{
			page_cache_release(pages[j]);
		}
		res = 0;
	}
	kfree(pages);
	return -ENOMEM; /* We consider this a fail */
}

static int
sgl_unmap_user_pages(struct scatterlist *sgl, const unsigned int nr_pages,
		     int dirtied)
{
	int i;

	for (i = 0; i < nr_pages; i++)
	{
                struct page *page = sg_page(&sgl[i]);

#if CSX_DEBUG
		if( page == NULL )
		{
			printk(KERN_DEBUG "%s: ASSERT FAILURE: page is NULL for sg[%d] ?!\n", __FUNCTION__, i);
		}
#endif

		if (dirtied)
		{
			SetPageDirty(page);
		}
		/* FIXME: cache flush missing for rw==READ
		 * FIXME: call the correct reference counting function
		 */
		page_cache_release(page);
	}

	return 0;
}

/* Pin down user pages and put them into a scatter gather list.
 * Returns <= 0 if:
 * - mapping of all pages not successful
 * - any page is above max_pfn
 * (i.e., either completely successful or fails)
 */
static int
csx_map_user_pages(struct scatterlist *sgl,
		   const unsigned int max_pages,
		   unsigned long user_addr, size_t count,
		   enum dma_data_direction dma_direction,
		   unsigned long max_pfn)
{
	int nr_pages;

#if CSX_DEBUG
	printk(KERN_DEBUG "%s called\n", __FUNCTION__);
#endif

	nr_pages = sgl_map_user_pages(sgl, max_pages, user_addr, count, dma_direction);

#if CSX_DEBUG
	printk(KERN_DEBUG "%s sgl_map_user_pages returned %d for %x\n", __FUNCTION__, nr_pages, (unsigned) count);
#endif
	if (nr_pages <= 0)
	{
		return nr_pages;	/* no cleanup required. */
	}

	/* QQQ Not sure about the max_pfn piece. */
#ifdef MAX_PFN
	for (i = 0; i < nr_pages; i++)
	{
		if (page_to_pfn(sgl[i].page) > max_pfn)
		{
			goto out_unmap;
		}
	}
#endif
	return nr_pages;

#ifdef MAX_PFN
out_unmap:
#endif

	sgl_unmap_user_pages(sgl, nr_pages, 0);
	return -EINVAL;
}

/* Take the scatterlist and map to the IOMMU */
static int
csx_map_to_iommu(struct pci_dev *pdev, struct scatterlist *sgl,
		 unsigned pages_pinned, struct dma_desc *desc_array,
		 enum dma_data_direction direction)
{
	int pages_mapped;
	int i;

	/* map to bus */
#if CSX_DEBUG
//	printk(KERN_DEBUG "%s: requesting the following mappings from dma_map_sg:\n", __FUNCTION__);
//	for (i = 0; i < pages_pinned; i++)
//	{
//		printk(KERN_DEBUG "\tsgl[%d] page=%p offset=%u length=%u dma_address=%llx dma_length=%u\n",
//		       i, sgl[i].page, sgl[i].offset, sgl[i].length,  sg_dma_address(&sgl[i]), sg_dma_len(&sgl[i]));
//	}
#endif
	pages_mapped = dma_map_sg(&pdev->dev, sgl, pages_pinned, direction);

	if (pages_mapped == 0) {
#if CSX_DEBUG
		printk(KERN_DEBUG "%s: dma_map_sg returned %d\n",
		       __FUNCTION__, pages_mapped);
#endif
		return -ENOSPC;
	}

	/* Now copy out to user's desc_array buffer. */
	/* This check should be unecessary but this is the point at which we write to the
	   desc_arrary so we should just check that it's big enough */
	if (pages_mapped > MAX_SG_SIZE) {
#if CSX_DEBUG
	  /* This is really an internal error. */
	  printk(KERN_DEBUG "%s: pages_mapped (%d) larger than max (%d)\n",
                 __FUNCTION__, pages_mapped, (int)MAX_SG_SIZE);
#endif
	  return -ENOSPC;
	}

	for (i = 0; i < pages_mapped; i++)
	{
		/* QQQ what about the offset field.  */
	    desc_array[i].bus_address = (bus_address_t)sg_dma_address(&sgl[i]);
		desc_array[i].length = sg_dma_len(&sgl[i]);

        if( desc_array[i].bus_address == 0 )
        {
            printk(KERN_DEBUG "WARNING: page descriptor %d of %d has zero bus address.\n",
                i, pages_mapped );
        }

#if CSX_DEBUG
//		printk(KERN_DEBUG "%s:%d: %llx %x\n", __FUNCTION__, i,
//		       desc_array[i].bus_address,
//		       desc_array[i].length);
#endif
	}

	return pages_mapped;	/* which may be different from pages_pinned */
}

/* Data buffer lock function, specification as user call */
static status
csx_lock_buffer_for_dma(struct csx_device_data *cdd,
                        int chan,
                        int buffer,
                        void *user_address,
                        unsigned user_size,
                        int *desc_count,
                        struct dma_desc *desc_array,
                        unsigned flags)
{
	int pages_pinned = 0;
	int pages_mapped = 0;
	const unsigned long max_pfn = 4000;	/*QQQ */
	status ret;
	char *errstring = "";
	enum dma_data_direction dma_direction;

#if CSX_DEBUG
	printk(KERN_DEBUG "%s called\n", __FUNCTION__);
#endif

    if( chan < 0 || chan >= DMA_MAX_CHANNELS )
    {
  		printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, buffer );
		return -EINVAL;
    }

    if( buffer < 0 || buffer >= DMA_MAX_BUFFERS )
    {
  		printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, buffer );
		return -EINVAL;
    }

	/* Allocate an initialise the dma_region. Freed in csx_unlock_buffer_for_dma */
	cdd->mem->region[chan].sglist[buffer] =
		kmalloc(MAX_SG_SIZE * sizeof(struct scatterlist), GFP_KERNEL);

	if (!cdd->mem->region[chan].sglist[buffer])
    {
		errstring = "Allocation of scatter/gather failed";
		ret = -ENOMEM;
		goto alloc_sglist_failed;
	}

	/* Clear the scatterlist */
        sg_init_table(cdd->mem->region[chan].sglist[buffer], MAX_SG_SIZE);

	/* Translate from private flags to DMA enum types.  */
	if (flags & DMA_FLAG_FROM_DEVICE)
	{
		dma_direction = DMA_FROM_DEVICE;
	}
	else
	{
		dma_direction = DMA_TO_DEVICE;
	}

	/* Pin the pages */
	pages_pinned = csx_map_user_pages(cdd->mem->region[chan].sglist[buffer], MAX_SG_SIZE,
	    (unsigned long) user_address, user_size, dma_direction, max_pfn);

    if (pages_pinned <= 0)
    {
		errstring = "csx_map_user_pages failed";
		ret = (status) pages_pinned;
		goto pin_failed;
	}

    /* Pages pinned, now map to bus addresses */
	pages_mapped = csx_map_to_iommu(cdd->pcidev, cdd->mem->region[chan].sglist[buffer],
        pages_pinned, desc_array, dma_direction);

	if (pages_mapped <= 0)
    {
		ret = (status) pages_mapped;
		goto iomapping_failed;
	}

	/* Record actual pages_pinned. We need this rather than pages_mapped which
	 * is what pci_map_sg returns. */
	cdd->mem->region[chan].pages_pinned[buffer] = pages_pinned;
	cdd->mem->region[chan].direction = dma_direction;

	/* Successfully programmed the IOMMU
	 * Gather up results and pass back to user side caller. */
	*desc_count = pages_mapped;

    /* Record the number of bytes transferred */
    if (DMA_FROM_DEVICE == dma_direction)
    {
        cdd->ctl->stats->kbytes_from_device += user_size;
    }
    else
    {
        cdd->ctl->stats->kbytes_to_device += user_size;
    }
        
	return 0; /* success */

iomapping_failed:
pin_failed:
alloc_sglist_failed:
	printk(KERN_WARNING "%s: %s (%d)\n", __FUNCTION__, errstring, pages_mapped);
	return ret;
}

static int
csx_unmap_to_iommu(struct pci_dev *pdev, struct scatterlist *sgl,
		   unsigned pages_mapped, enum dma_data_direction direction)
{
	int i;

	dma_unmap_sg(&pdev->dev, sgl, pages_mapped, direction);

	/* This is essentially put_user_pages */
	for (i = 0; i < pages_mapped; i++)
	{
		if (direction == DMA_FROM_DEVICE)
		{
			if(!PageReserved(sg_page(&sgl[i])))
			{
				SetPageDirty(sg_page(&sgl[i]));
			}
		}
		page_cache_release(sg_page(&sgl[i]));
	}
	return 0;		/* can't fail I guess. */
}

static status
csx_unlock_buffer_for_dma(struct csx_device_data *cdd,
    int chan, int buffer, int pages_mapped,
    struct dma_desc *desc_array, unsigned flags)
{
	int reso;
	enum dma_data_direction direction;

    if( chan < 0 || chan >= DMA_MAX_CHANNELS )
    {
  		printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, buffer );
		return -EINVAL;
    }

    if( buffer < 0 || buffer >= DMA_MAX_BUFFERS )
    {
  		printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, buffer );
		return -EINVAL;
    }

	if (flags & DMA_FLAG_FROM_DEVICE)
	{
		direction = DMA_FROM_DEVICE;
	}
	else
	{
		direction = DMA_TO_DEVICE;
	}

	reso = csx_unmap_to_iommu(cdd->pcidev, cdd->mem->region[chan].sglist[buffer],
	    pages_mapped, direction);

	/* If we have read something from the device we need to
	   synch to allow the cpu to read it. */
	if (direction == DMA_FROM_DEVICE)
    {
	    dma_sync_sg_for_cpu(&cdd->pcidev->dev, cdd->mem->region[chan].sglist[buffer],
		    pages_mapped, direction);
    }

	/* Now release the scatter/gather list */
	kfree(cdd->mem->region[chan].sglist[buffer]);
	cdd->mem->region[chan].sglist[buffer] = NULL;

	return reso;
}

#define DMA_PCI_ADDRESS_SPACE 0x2

/* Set this for pur chain mode - else it will copy first descriptor in */
#define FIRE_DMA_PURE_CHAIN   0

static int
csx_fire_pcix_dma(struct csx_device_data *cdd, int chan, int buffer, bus_address_t bus_addr,
	     unsigned flags)
{
	int reso;

#if !FIRE_DMA_PURE_CHAIN
    unsigned int n;
    unsigned int byte_count = 0;
    unsigned int *p_desc;
#endif

    if( chan < 0 || chan >= DMA_MAX_CHANNELS )
    {
  		printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, chan );
		return -EINVAL;
    }

    if( buffer < 0 || buffer >= DMA_MAX_BUFFERS )
    {
  		printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, buffer );
		return -EINVAL;
    }

    if (bus_addr == cdd->mem->region[chan].desc_bus_address[buffer])
	{
        /* call the sync function to ensure that the data buffer is ready for the device */
		dma_sync_sg_for_device(&cdd->pcidev->dev,
				       cdd->mem->region[chan].sglist[buffer],
				       cdd->mem->region[chan].pages_pinned[buffer],
				       cdd->mem->region[chan].direction);

#if FIRE_DMA_PURE_CHAIN
		/* Say the address is in PCI space.  */
		bus_addr |= DMA_PCI_ADDRESS_SPACE;

		/* Set up for Pure chain & zero byte count */
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_BYTECOUNT, 0 );
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_CONTROL, 0x00c00000 );
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_FPGA_ADDR_LO, 0 );
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_FPGA_ADDR_HI, 0 );
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_PCI_ADDR_LO, 0 );
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_PCI_ADDR_HI, 0 );
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_LINK_LO, bus_addr & 0xffffffff );
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_LINK_HI, bus_addr >> 32 );

		/* Finally, Fire DMA by setting the valid bit */
		/* Probably a good ideal to request a memory barrier here. */
		wmb();
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_BYTECOUNT, 0x01000000 );
		MMIOWB();
#else
        if (buffer == 0)
        {
            p_desc = (unsigned int *)cdd->mem->region[chan].desc_kaddr;
        }
        else
        {
            p_desc = (unsigned int *)( (char *)cdd->mem->region[chan].desc_kaddr + MAX_DESCRIPTOR_BUFFER_SIZE );
        }
       
        for( n=REG_INDEX_HBDMA_BYTECOUNT; n<=REG_INDEX_HBDMA_LINK_HI; n++ )
        {
            if( n == REG_INDEX_HBDMA_BYTECOUNT )
            {
                /* Strip off valid bit for now */
                byte_count = *p_desc & 0xFFFFFF;
                csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_BYTECOUNT, byte_count );
            }
            else
            {
                csx_write_register( cdd->ctl, REGSET_PCIX, 0, n, *p_desc );
            }

            p_desc++;
        }

		/* Finally, Fire DMA by or-ing in the valid bit */
		wmb();
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_HBDMA_BYTECOUNT, byte_count | 0x01000000 );
		MMIOWB();
#endif

        // Switch on the channels DMA interrupts
        csx_write_register( cdd->ctl, REGSET_PCIX, 0, REG_INDEX_DMA_INT_MASK, CSX_PCIX_CTL_DMAINTR_MASK );
		MMIOWB();

        reso = 0;
	}
	else
	{
		reso = -EINVAL;
	}

	return reso;
}

static int
csx_fire_pcie_dma(struct csx_device_data *cdd, int chan, int buffer, bus_address_t bus_addr,
	     unsigned flags)
{
	int reso;
    unsigned int *p_desc;

    if( chan < 0 || chan >= DMA_MAX_CHANNELS )
    {
  		printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, chan );
		return -EINVAL;
    }

    if( buffer < 0 || buffer >= DMA_MAX_BUFFERS )
    {
  		printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, buffer );
		return -EINVAL;
    }

    pcie_program_request_size( cdd, cdd->mem->region[chan].direction );
    if (bus_addr == cdd->mem->region[chan].desc_bus_address[buffer])
	{
        /* call the sync function to ensure that the data buffer is ready for
		 * the device
		 */
		dma_sync_sg_for_device(&cdd->pcidev->dev,
				       cdd->mem->region[chan].sglist[buffer],
				       cdd->mem->region[chan].pages_pinned[buffer],
				       cdd->mem->region[chan].direction);

        if (buffer == 0)
        {
            p_desc = (unsigned int *)cdd->mem->region[chan].desc_kaddr;
        }
        else
        {
            p_desc = (unsigned int *)( (char *)cdd->mem->region[chan].desc_kaddr + MAX_DESCRIPTOR_BUFFER_SIZE);
        }

        csx_write_register( cdd->ctl, RegSetFromID( cdd ), chan, REG_INDEX_DMA_START_ADDR_LO, bus_addr & 0xFFFFFFFF );
        csx_write_register( cdd->ctl, RegSetFromID( cdd ), chan, REG_INDEX_DMA_START_ADDR_HI, bus_addr >> 32 );

/*
		//check the interrupt status for this channel:
		csx_read_register( cdd->ctl, RegSetFromID( cdd ), chan, REG_INDEX_DMA_INT_STATUS, &dmastatus );
		if(dmastatus != 0)
		{
			printk(KERN_WARNING "%s: DMA interrupt status for channel %d is set (0x%x) - resetting\n", __FUNCTION__, chan , dmastatus );
			csx_write_register( cdd->ctl, RegSetFromID( cdd ), chan, REG_INDEX_DMA_INT_STATUS, dmastatus );
		}
*/

        /* Fire DMA by writing the start_transfer bit (bit 0) of the control status reg and the flush incomming
           (bit 1) which is used on reads */
		wmb();
        csx_write_register( cdd->ctl, RegSetFromID( cdd ), chan, REG_INDEX_DMA_CTRL_STATUS, 3 | flags );  

        // Switch on the channel's DMA interrupts
        csx_write_register( cdd->ctl, RegSetFromID( cdd ), chan, REG_INDEX_DMA_INT_MASK, CSX_PCIE_INT_DMA_MASK );
	
		if( cdd->pci_dev_id == PCI_DEVICE_ID_CLEARSPEED_CSX_E521 )
		{
        	csx_write_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_GIU2_MASK1, CSX_PCIE_INT_GIU2_DMA_MASK_DIOCLES );
		}
		else // E701 & E711
		{
			csx_write_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_GIU2_MASK1, CSX_PCIE_INT_GIU2_DMA_MASK_CALLANISH );
		}

		MMIOWB();
        reso = 0;
	}
	else
	{
		reso = -EINVAL;
	}

	return reso;
}

/* End of DMA routines */

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11))
// New function(s) for kernel 2.6.11+
//  Do ioctl but WITHOUT using kernel ioctl BKL ( Big Kernel Lock )
//  Instead we have to do our own locking within here
static long
csx_mem_unlocked_ioctl(struct file *file, unsigned int cmd_in, unsigned long arg)
{
	struct csx_mem_data *cmd;
	struct csx_device_data *cdd;
	void __user *p = (void __user *) arg;
    
    static long ret_value = 0;

    // Protect all the calls inside here
    int lock_retval = 0;

#if CSX_DEBUG_IOCTLS
    printk(KERN_DEBUG "Calling down_interruptible()\n");
#endif
    lock_retval = down_interruptible(&ioctl_sem);
    if(lock_retval)
    {
        printk(KERN_DEBUG "ERROR: didnt get semaphore lock\n");
        ret_value = -EFAULT;
        goto exit_point;
    }
#if CSX_DEBUG_IOCTLS
    printk(KERN_DEBUG "After taking lock\n");
#endif

	cmd = container_of(file->f_dentry->d_inode->i_cdev, struct csx_mem_data, cdev);
	cdd = cmd->maindata;

	switch (cmd_in)
    {
	case CSX_GET_COHERENT_BUFFER_INFO:
	{
		struct ioctl_param_get_coherent_map_info this_call;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_GET_COHERENT_BUFFER_INFO:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call) != 0 )
        {
            ret_value = -EFAULT;
            goto exit_point;
        }
        
        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            ret_value = -EINVAL;
            goto exit_point;
        }
        chan = this_call.channel;

		/* No input parameters.  */
		this_call.descriptor_chain1 = cdd->mem->region[chan].desc_bus_address[0];
		this_call.descriptor_chain2 = cdd->mem->region[chan].desc_bus_address[1];

		//printk(KERN_DEBUG "%s: about to copy to user p = %p\n", __FUNCTION__, p);
		if (copy_to_user(p, &this_call, sizeof this_call) != 0)
        {
            ret_value = -EFAULT;
            goto exit_point;
        }

        ret_value = 0;
        goto exit_point;
	}

	case CSX_COPY_TO_DESC_BUFFER:
	{
		struct ioctl_param_copy_to_coherent_descriptor_buffer this_call;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_COPY_TO_DESC_BUFFER:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call) != 0 )
        {
            ret_value = -EFAULT;
            goto exit_point;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            ret_value = -EINVAL;
            goto exit_point;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
            ret_value = -EINVAL;
            goto exit_point;
        }

		/* check size */
		if( (this_call.size == 0) || (this_call.size > cdd->mem->region[chan].desc_buf_size) )
        {
			reso = -EINVAL;
        }
		else
        {
            unsigned long copy_ret;

			/* safe to do the copy */
            if( this_call.buffer == 0 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[0] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_TO_DESC_BUFFER: copy to buffer A.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( cdd->mem->region[chan].desc_kaddr,
					this_call.user_descriptor_buffer, this_call.size );
            }
            else if( this_call.buffer == 1 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[1] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_TO_DESC_BUFFER: copy to buffer B.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( (char *)cdd->mem->region[chan].desc_kaddr +
                    MAX_DESCRIPTOR_BUFFER_SIZE, this_call.user_descriptor_buffer, this_call.size );
            }
            else
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_TO_DESC_BUFFER: bad copy location.\n", __FUNCTION__ );
#endif
                ret_value = -EINVAL;
                goto exit_point;
            }

			if( copy_ret != 0 )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_TO_DESC_BUFFER: copy_from_user failed.\n", __FUNCTION__ );
#endif
                ret_value = -EINVAL;
                goto exit_point;
            }

#if CSX_DEBUG_EXTREME
			{
				unsigned int *ptr =	(unsigned int *) cdd->mem->region[chan].desc_kaddr;
				int i;

				for (i = 0; i <	this_call.size / (sizeof(unsigned int)); i++)
                {
					printk(KERN_DEBUG "%x: %x\n", i, *ptr);
					ptr++;
				}
			}
#endif
			reso = 0;
		}
        ret_value = reso;
        goto exit_point;
	}

	case CSX_LOCK_DATA_BUFFER:
	{
		struct ioctl_param_buffer_lock this_call;
		struct dma_desc *desc_array;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_LOCK_DATA_BUFFER:\n" );
#endif

		desc_array = (struct dma_desc*)kmalloc( MAX_SG_SIZE * sizeof(struct dma_desc), GFP_KERNEL);
		if (NULL == desc_array)
        {
            ret_value = -ENOMEM;
            goto exit_point;
        }

		if (copy_from_user(&this_call, p, sizeof this_call) != 0)
        {
            kfree( desc_array );
            ret_value = -EFAULT;
            goto exit_point;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            ret_value = -EINVAL;
            goto exit_point;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
            ret_value = -EINVAL;
            goto exit_point;
        }

#if CSX_DEBUG
		printk(KERN_DEBUG "%s: csx_lock_data_buffer for %x bytes\n", __FUNCTION__, this_call.user_size);
#endif

		/* a few checks */
		if (this_call.user_size > MAX_SINGLE_DMA_TRANSFER)
        {
            kfree( desc_array );
            ret_value = -EINVAL;
            goto exit_point;
		}

		/* clear the desc_array */
		memset(desc_array, 0, MAX_SG_SIZE * sizeof(struct dma_desc));

		reso = csx_lock_buffer_for_dma(cdd,
			this_call.channel,
			this_call.buffer,
			this_call.user_address,
			this_call.user_size,
			&this_call.desc_count,
			desc_array,
			this_call.flags);

		if (!reso)
		{
		    /* Copy the data buffer back. */
		    if (copy_to_user(this_call.desc_array, desc_array, this_call.desc_count *
                sizeof(struct dma_desc)) != 0)
            {
                kfree( desc_array );
                ret_value = -EFAULT;
                goto exit_point;
            }

            if (copy_to_user(p, &this_call, sizeof(struct ioctl_param_buffer_lock)) != 0)
            {
                kfree( desc_array );
                ret_value = -EFAULT;
                goto exit_point;
            }
		}

		kfree( desc_array );
        ret_value = reso;
        goto exit_point;
	}

	case CSX_UNLOCK_DATA_BUFFER:
	{
		struct ioctl_param_buffer_unlock this_call;
		struct dma_desc *desc_array;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_UNLOCK_DATA_BUFFER:\n" );
#endif

		if (copy_from_user(&this_call, p, sizeof this_call) != 0)
        {
            ret_value = -EFAULT;
            goto exit_point;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            ret_value = -EINVAL;
            goto exit_point;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
            ret_value = -EINVAL;
            goto exit_point;
        }

#if CSX_DEBUG
		printk(KERN_DEBUG "%s: csx_unlock_data_buffer for %d descriptors\n", __FUNCTION__, this_call.desc_count);
#endif

		/* How do we check that this unlock corresponds to an outstanding lock??
		 * check if pages_pinned is sensible compared to what the user has given us
		 * pages_pinned can be the same as or less than what we're given back as the 
		 * putative size. 
		 */
		if (cdd->mem->region[chan].pages_pinned[this_call.buffer] < this_call.desc_count)
        {
            ret_value = -EINVAL;
            goto exit_point;
        }

		/* We have to use the pages pinned number rather then the desc_count that we delivered to the user */
		if (!cdd->mem->region[chan].pages_pinned[this_call.buffer])
        {
            ret_value = -EINVAL;
            goto exit_point;
        }

		desc_array = (struct dma_desc*)kmalloc(	MAX_SG_SIZE * sizeof(struct dma_desc), GFP_KERNEL);
		if (NULL == desc_array)
        {
            ret_value = -ENOMEM;
            goto exit_point;
        }

		/* clear the desc_array */
		memset(desc_array, 0, MAX_SG_SIZE * sizeof(struct dma_desc));

		reso = csx_unlock_buffer_for_dma(cdd,
            this_call.channel,
            this_call.buffer,
			cdd->mem->region[chan].pages_pinned[this_call.buffer],
			desc_array,
			this_call.flags);

		/* Use this to prevent dangling releases.  */
		cdd->mem->region[chan].pages_pinned[this_call.buffer] = 0;

		kfree(desc_array);

        ret_value = reso;
        goto exit_point;
	}

	case CSX_FIRE_PCIX_DMA:
	{
		struct ioctl_param_buffer_fire_pcix this_call;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_FIRE_PCIX_DMA:\n" );
#endif

		if (copy_from_user(&this_call, p, sizeof this_call) != 0)
        {
            ret_value = -EFAULT;
            goto exit_point;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            ret_value = -EINVAL;
            goto exit_point;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
            ret_value = -EINVAL;
            goto exit_point;
        }

#if CSX_DEBUG
		printk(KERN_DEBUG "%s: csx_fire_pcix_dma for desc address %llx\n", __FUNCTION__, this_call.desc_address);
#endif
		reso = csx_fire_pcix_dma(cdd, chan, this_call.buffer, this_call.desc_address, this_call.flags);

        ret_value = reso;
        goto exit_point;
	}

    case CSX_COPY_AND_FIRE_PCIX:
    {
		struct ioctl_param_copy_and_fire_pcix this_call;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_COPY_AND_FIRE_PCIX:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call ) != 0 )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: copy_from_user failed\n", __FUNCTION__ );
#endif
            ret_value = -EFAULT;
            goto exit_point;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            ret_value = -EINVAL;
            goto exit_point;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
            ret_value = -EINVAL;
            goto exit_point;
        }

		/* check size */
		if ((this_call.size == 0) || (this_call.size > cdd->mem->region[chan].desc_buf_size))
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: size error\n", __FUNCTION__ );
#endif
			reso = -EINVAL;
        }
		else
        {
            unsigned long copy_ret;

            /* safe to do the copy */
            if( this_call.buffer == 0 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[0] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIX: copy to buffer A.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( cdd->mem->region[chan].desc_kaddr,
					this_call.user_descriptor_buffer, this_call.size );
            }
            else if( this_call.buffer == 1 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[1] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIX: copy to buffer B.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( (char *)cdd->mem->region[chan].desc_kaddr +
                    MAX_DESCRIPTOR_BUFFER_SIZE,	this_call.user_descriptor_buffer, this_call.size );
            }
            else
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIX: bad copy location.\n", __FUNCTION__ );
#endif
                ret_value = -EINVAL;
                goto exit_point;
            }

			if( copy_ret != 0 )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIX: copy_from_user failed.\n", __FUNCTION__ );
#endif
                ret_value = -EINVAL;
                goto exit_point;
            }
            else
            {
  		        reso = csx_fire_pcix_dma(cdd, chan, this_call.buffer, this_call.desc_address, this_call.flags);
                if( reso < 0 )
                {
            		printk(KERN_DEBUG "%s: csx_fire_pcix_dma failed.\n", __FUNCTION__ );
                }
            }
		}

        ret_value = reso;
        goto exit_point;
    }

    case CSX_GET_PAGE_SIZE:
    {
		struct ioctl_param_get_page_size this_call;

        this_call.page_size_in_bytes = PAGE_SIZE;

        if (copy_to_user(p, &this_call, sizeof this_call) != 0)
        {
            ret_value = -EFAULT;
            goto exit_point;
        }
        else
        {
            ret_value = 0;
            goto exit_point;
        }
    }

	case CSX_FIRE_PCIE_DMA:
	{
		struct ioctl_param_buffer_fire_pcie this_call;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_FIRE_PCIE_DMA:\n" );
#endif

		if (copy_from_user(&this_call, p, sizeof this_call) != 0)
        {
            ret_value = -EFAULT;
            goto exit_point;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            ret_value = -EINVAL;
            goto exit_point;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
            ret_value = -EINVAL;
            goto exit_point;
        }

#if CSX_DEBUG
		printk(KERN_DEBUG
			"%s: csx_fire_pcie_dma on channel %d for desc address %llx\n",
			__FUNCTION__, this_call.channel, this_call.desc_address);
#endif

		reso = csx_fire_pcie_dma(cdd, chan, this_call.buffer, this_call.desc_address, this_call.flags);

        ret_value = reso;
        goto exit_point;
	}

    case CSX_COPY_AND_FIRE_PCIE:
    {
		struct ioctl_param_copy_and_fire_pcie this_call;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_COPY_AND_FIRE_PCIE:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call ) != 0 )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: copy_from_user failed\n", __FUNCTION__ );
#endif
            ret_value = -EFAULT;
            goto exit_point;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            ret_value = -EINVAL;
            goto exit_point;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
            ret_value = -EINVAL;
            goto exit_point;
        }

		if ((this_call.size == 0) || (this_call.size > cdd->mem->region[chan].desc_buf_size))
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: size error\n", __FUNCTION__ );
#endif
			reso = -EINVAL;
        }
		else
        {
            unsigned long copy_ret;

            /* safe to do the copy */
            if( this_call.buffer == 0 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[0] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIE: copy to buffer A.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( cdd->mem->region[chan].desc_kaddr,
					this_call.user_descriptor_buffer, this_call.size );
            }
            else if( this_call.buffer == 1 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[1] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIE: copy to buffer B.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( (char *)cdd->mem->region[chan].desc_kaddr +
                    MAX_DESCRIPTOR_BUFFER_SIZE, this_call.user_descriptor_buffer, this_call.size );
            }
            else
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIE: bad copy location.\n", __FUNCTION__ );
#endif
                ret_value = -EINVAL;
                goto exit_point;
            }

			if( copy_ret != 0 )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIE: copy_from_user failed.\n", __FUNCTION__ );
#endif
                ret_value = -EINVAL;
                goto exit_point;
            }
            else
            {
        		reso = csx_fire_pcie_dma(cdd, chan, this_call.buffer, this_call.desc_address, this_call.flags);
                if( reso < 0 )
                {
            		printk(KERN_DEBUG "%s: csx_fire_pcie_dma failed.\n", __FUNCTION__ );
                }
            }
		}

        ret_value = reso;
        goto exit_point;
    }
	case CSX_PCIE_DMA_INTERRUPT_COUNT:
    {
		struct ioctl_param_pcie_dma_interrupt_count this_call;
		struct csx_mem_file *cmf;
		int count;

		if( copy_from_user(&this_call, p, sizeof this_call ) != 0 )
        {
            ret_value = -EINVAL;
            goto exit_point;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            ret_value = -EINVAL;
            goto exit_point;
        }

		cmf = file->private_data;
		switch(this_call.channel)
		{
		case 0:
			count = atomic_read(&cmd->intr_count0);	// DMA channel 0
			if (cmf->intr_count0 != count)
			{
				cmf->intr_count0 = count;
			}
			else
			{
				printk(KERN_WARNING "%s: no interrupt counted for DMA channel (%d)\n", __FUNCTION__, this_call.channel );
			}
			break;
		case 1:
			count = atomic_read(&cmd->intr_count1);	// DMA channel 1
			if (cmf->intr_count1 != count)
			{
				cmf->intr_count1 = count;
			}
			else
			{
				printk(KERN_WARNING "%s: no interrupt counted for DMA channel (%d)\n", __FUNCTION__, this_call.channel );
			}
			break;
		default:
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            ret_value = -EINVAL;
            goto exit_point;
		};
		ret_value = 0;
        goto exit_point;
	}

    case CSX_PCI_CONFIG_SPACE:
    {
		struct ioctl_param_pci_config_space this_call;
        u8 *buffer;
        u8 *p_buffer;
        int i;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_PCI_CONFIG_SPACE:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call ) != 0 )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: copy_from_user (1) failed\n", __FUNCTION__ );
#endif
            ret_value = -EINVAL;
            goto exit_point;
        }

        // allocate memory to copy the data across
        buffer = kmalloc(this_call.length, GFP_KERNEL);
        if( buffer == NULL )
        {
            ret_value = -ENOMEM;
            goto exit_point;
        }

        if( copy_from_user(buffer, this_call.data, this_call.length ) != 0 )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: copy_from_user (2) failed\n", __FUNCTION__ );
#endif
            ret_value = -EFAULT;
            goto exit_point;
        }
        
        p_buffer = buffer;

        for( i=this_call.offset; i<(this_call.offset+this_call.length); i++ )
        {
            if( this_call.is_read )
            {
                pci_read_config_byte( cdd->pcidev, i, p_buffer );
            }
            else
            {
                pci_write_config_byte( cdd->pcidev, i, *p_buffer );
            }
            
            p_buffer++;
        }

        if( this_call.is_read )
        {
            if (copy_to_user(this_call.data, buffer, this_call.length) != 0)
            {
                kfree(buffer);
                ret_value = -EFAULT;
                goto exit_point;
            }

            if (copy_to_user(p, &this_call, sizeof this_call) != 0)
            { 
                kfree(buffer);
                ret_value = -EFAULT;
                goto exit_point;
            }
        }
        kfree(buffer);
        ret_value = 0;
        goto exit_point;
    }

    case CSX_ESCAPE:
    {
		struct ioctl_param_kernel_escape this_call;
        u8 val;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_ESCAPE:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call ) != 0 )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: copy_from_user failed\n", __FUNCTION__ );
#endif
            ret_value = -EFAULT;
            goto exit_point;
        }

        /* check checksum */
        if( (this_call.escape_index + this_call.escape_param1 + this_call.escape_param2 +
            this_call.escape_return1 + this_call.escape_return2) != this_call.checksum )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: Escape checksum error - rejecting it.\n", __FUNCTION__ );
#endif
            ret_value = -EINVAL;
            goto exit_point;
        }

        /* look at escape index now */
        switch( this_call.escape_index )
        {
        case CSX_KERNEL_ESCAPE_VERSION:
#if CSX_DEBUG_IOCTLS
            printk(KERN_DEBUG "CSX_KERNEL_ESCAPE_VERSION:\n" );
#endif
            this_call.escape_return1 = CSX_IOCTL_VERSION;
            break;

        case CSX_KERNEL_ESCAPE_CARD_TYPE:
#if CSX_DEBUG_IOCTLS
            printk(KERN_DEBUG "CSX_KERNEL_ESCAPE_CARD_TYPE:\n" );
#endif
            this_call.escape_return1 = cdd->pci_dev_id;
            pci_read_config_byte( cdd->pcidev, 0x8, (u8 *) &(this_call.escape_return2) );
            break;

        case CSX_KERNEL_ESCAPE_REQUEST_CAPABILITY:
#if CSX_DEBUG_IOCTLS
            printk(KERN_DEBUG "CSX_KERNEL_ESCAPE_REQUEST_CAPABILITY:\n" );
#endif
		    printk(KERN_WARNING "%s: Escape capability not implemented!\n", __FUNCTION__ );
            ret_value = -EINVAL;
            goto exit_point;
            break;

        case CSX_KERNEL_ESCAPE_PCIE_LANE_WIDTH:
#if CSX_DEBUG_IOCTLS
            printk(KERN_DEBUG "CSX_KERNEL_ESCAPE_PCIE_LANE_WIDTH:\n" );
#endif
            pci_read_config_byte( cdd->pcidev, 0x82, &val );
            this_call.escape_return1 = val >> 4;
            break;

        default:
    		printk(KERN_WARNING "%s: Escape index out of range - rejecting it.\n", __FUNCTION__ );
            ret_value = -EINVAL;
            goto exit_point;
        }

        if (copy_to_user(p, &this_call, sizeof this_call) != 0)
        {
    		printk(KERN_WARNING "%s: Failed to copy data back to user.\n", __FUNCTION__ );
            ret_value = -EFAULT;
            goto exit_point;
        }

        ret_value = 0;
        goto exit_point;
    }

	default:
        printk(KERN_WARNING "Bad ioctl command !\n" );
        ret_value = -ENOTTY;
        goto exit_point;
    }

exit_point:

    // Un-Protect all the calls inside here
#if CSX_DEBUG_IOCTLS
    printk(KERN_DEBUG "Will do unlock()\n");
    printk(KERN_DEBUG "ioctl_sem.count:%d\n", ioctl_sem.count);
#endif
    if( lock_retval == 0 )
        up (&ioctl_sem);
#if CSX_DEBUG_IOCTLS
    printk(KERN_DEBUG "After releasing lock\n");
#endif

    return ret_value;

} // end - csx_mem_unlocked_ioctl()
#endif  // #if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11))


// The original ( pre 2.6.11 kernel ) version - ioctl using kernel locking ( BigKernelLock )
static int
csx_mem_ioctl(struct inode *inode, struct file *file, unsigned int cmd_in, unsigned long arg)
{
	struct csx_mem_data *cmd;
	struct csx_device_data *cdd;
	void __user *p = (void __user *) arg;

#if CSX_DEBUG_IOCTLS
printk(KERN_DEBUG "csx_mem_ioctl()\n" );
#endif

	cmd = container_of(file->f_dentry->d_inode->i_cdev, struct csx_mem_data, cdev);
	cdd = cmd->maindata;

	switch (cmd_in)
    {
	case CSX_GET_COHERENT_BUFFER_INFO:
	{
		struct ioctl_param_get_coherent_map_info this_call;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_GET_COHERENT_BUFFER_INFO:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call) != 0 )
        {
			return -EFAULT;
        }
        
        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
		    return -EINVAL;
        }
        chan = this_call.channel;

		/* No input parameters.  */
		this_call.descriptor_chain1 = cdd->mem->region[chan].desc_bus_address[0];
		this_call.descriptor_chain2 = cdd->mem->region[chan].desc_bus_address[1];

		//printk(KERN_DEBUG "%s: about to copy to user p = %p\n", __FUNCTION__, p);
		if (copy_to_user(p, &this_call, sizeof this_call) != 0)
        {
			return -EFAULT;
        }

		return 0;
	}

	case CSX_COPY_TO_DESC_BUFFER:
	{
		struct ioctl_param_copy_to_coherent_descriptor_buffer this_call;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_COPY_TO_DESC_BUFFER:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call) != 0 )
        {
			return -EFAULT;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
		    return -EINVAL;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
		    return -EINVAL;
        }

		/* check size */
		if( (this_call.size == 0) || (this_call.size > cdd->mem->region[chan].desc_buf_size) )
        {
			reso = -EINVAL;
        }
		else
        {
            unsigned long copy_ret;

			/* safe to do the copy */
            if( this_call.buffer == 0 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[0] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_TO_DESC_BUFFER: copy to buffer A.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( cdd->mem->region[chan].desc_kaddr,
					this_call.user_descriptor_buffer, this_call.size );
            }
            else if( this_call.buffer == 1 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[1] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_TO_DESC_BUFFER: copy to buffer B.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( (char *)cdd->mem->region[chan].desc_kaddr +
                    MAX_DESCRIPTOR_BUFFER_SIZE, this_call.user_descriptor_buffer, this_call.size );
            }
            else
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_TO_DESC_BUFFER: bad copy location.\n", __FUNCTION__ );
#endif
				return -EINVAL;
            }

			if( copy_ret != 0 )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_TO_DESC_BUFFER: copy_from_user failed.\n", __FUNCTION__ );
#endif
				return -EINVAL;
            }

#if CSX_DEBUG_EXTREME
			{
				unsigned int *ptr =	(unsigned int *) cdd->mem->region[chan].desc_kaddr;
				int i;

				for (i = 0; i <	this_call.size / (sizeof(unsigned int)); i++)
                {
					printk(KERN_DEBUG "%x: %x\n", i, *ptr);
					ptr++;
				}
			}
#endif
			reso = 0;
		}
		return reso;
	}

	case CSX_LOCK_DATA_BUFFER:
	{
		struct ioctl_param_buffer_lock this_call;
		struct dma_desc *desc_array;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_LOCK_DATA_BUFFER:\n" );
#endif

		desc_array = (struct dma_desc*)kmalloc( MAX_SG_SIZE * sizeof(struct dma_desc), GFP_KERNEL);
		if (NULL == desc_array)
        {
            return -ENOMEM;
        }

		if (copy_from_user(&this_call, p, sizeof this_call) != 0)
        {
            kfree( desc_array );
            return -EFAULT;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
		    return -EINVAL;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
		    return -EINVAL;
        }

#if CSX_DEBUG
		printk(KERN_DEBUG "%s: csx_lock_data_buffer for %x bytes\n", __FUNCTION__, this_call.user_size);
#endif

		/* a few checks */
		if (this_call.user_size > MAX_SINGLE_DMA_TRANSFER)
        {
            kfree( desc_array );
            return -EINVAL;
		}

		/* clear the desc_array */
		memset(desc_array, 0, MAX_SG_SIZE * sizeof(struct dma_desc));

		reso = csx_lock_buffer_for_dma(cdd,
			this_call.channel,
			this_call.buffer,
			this_call.user_address,
			this_call.user_size,
			&this_call.desc_count,
			desc_array,
			this_call.flags);

		if (!reso)
		{
		    /* Copy the data buffer back. */
		    if (copy_to_user(this_call.desc_array, desc_array, this_call.desc_count *
                sizeof(struct dma_desc)) != 0)
            {
                kfree( desc_array );
                return -EFAULT;
            }

            if (copy_to_user(p, &this_call, sizeof(struct ioctl_param_buffer_lock)) != 0)
            {
                kfree( desc_array );
                return -EFAULT;
            }
		}

		kfree( desc_array );
		return reso;
	}

	case CSX_UNLOCK_DATA_BUFFER:
	{
		struct ioctl_param_buffer_unlock this_call;
		struct dma_desc *desc_array;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_UNLOCK_DATA_BUFFER:\n" );
#endif

		if (copy_from_user(&this_call, p, sizeof this_call) != 0)
			return -EFAULT;

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
		    return -EINVAL;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
		    return -EINVAL;
        }

#if CSX_DEBUG
		printk(KERN_DEBUG "%s: csx_unlock_data_buffer for %d descriptors\n", __FUNCTION__, this_call.desc_count);
#endif

		/* How do we check that this unlock corresponds to an outstanding lock??
		 * check if pages_pinned is sensible compared to what the user has given us
		 * pages_pinned can be the same as or less than what we're given back as the 
		 * putative size. 
		 */
		if (cdd->mem->region[chan].pages_pinned[this_call.buffer] < this_call.desc_count)
        {
			return -EINVAL;
        }

		/* We have to use the pages pinned number rather then the desc_count that we delivered to the user */
		if (!cdd->mem->region[chan].pages_pinned[this_call.buffer])
        {
			return -EINVAL;
        }

		desc_array = (struct dma_desc*)kmalloc(	MAX_SG_SIZE * sizeof(struct dma_desc), GFP_KERNEL);
		if (NULL == desc_array)
        {
			return -ENOMEM;
        }

		/* clear the desc_array */
		memset(desc_array, 0, MAX_SG_SIZE * sizeof(struct dma_desc));

		reso = csx_unlock_buffer_for_dma(cdd,
            this_call.channel,
            this_call.buffer,
			cdd->mem->region[chan].pages_pinned[this_call.buffer],
			desc_array,
			this_call.flags);

		/* Use this to prevent dangling releases.  */
		cdd->mem->region[chan].pages_pinned[this_call.buffer] = 0;

		kfree(desc_array);

		return reso;
	}

	case CSX_FIRE_PCIX_DMA:
	{
		struct ioctl_param_buffer_fire_pcix this_call;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_FIRE_PCIX_DMA:\n" );
#endif

		if (copy_from_user(&this_call, p, sizeof this_call) != 0)
        {
			return -EFAULT;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
		    return -EINVAL;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
		    return -EINVAL;
        }

#if CSX_DEBUG
		printk(KERN_DEBUG "%s: csx_fire_pcix_dma for desc address %llx\n", __FUNCTION__, this_call.desc_address);
#endif
		reso = csx_fire_pcix_dma(cdd, chan, this_call.buffer, this_call.desc_address, this_call.flags);

		return reso;
	}

    case CSX_COPY_AND_FIRE_PCIX:
    {
		struct ioctl_param_copy_and_fire_pcix this_call;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_COPY_AND_FIRE_PCIX:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call ) != 0 )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: copy_from_user failed\n", __FUNCTION__ );
#endif
			return -EFAULT;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
		    return -EINVAL;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
		    return -EINVAL;
        }

		/* check size */
		if ((this_call.size == 0) || (this_call.size > cdd->mem->region[chan].desc_buf_size))
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: size error\n", __FUNCTION__ );
#endif
			reso = -EINVAL;
        }
		else
        {
            unsigned long copy_ret;

            /* safe to do the copy */
            if( this_call.buffer == 0 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[0] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIX: copy to buffer A.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( cdd->mem->region[chan].desc_kaddr,
					this_call.user_descriptor_buffer, this_call.size );
            }
            else if( this_call.buffer == 1 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[1] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIX: copy to buffer B.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( (char *)cdd->mem->region[chan].desc_kaddr +
                    MAX_DESCRIPTOR_BUFFER_SIZE,	this_call.user_descriptor_buffer, this_call.size );
            }
            else
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIX: bad copy location.\n", __FUNCTION__ );
#endif
				return -EINVAL;
            }

			if( copy_ret != 0 )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIX: copy_from_user failed.\n", __FUNCTION__ );
#endif
				return -EINVAL;
            }
            else
            {
  		        reso = csx_fire_pcix_dma(cdd, chan, this_call.buffer, this_call.desc_address, this_call.flags);
                if( reso < 0 )
                {
            		printk(KERN_DEBUG "%s: csx_fire_pcix_dma failed.\n", __FUNCTION__ );
                }
            }
		}

		return reso;
    }

    case CSX_GET_PAGE_SIZE:
    {
		struct ioctl_param_get_page_size this_call;

        this_call.page_size_in_bytes = PAGE_SIZE;

        if (copy_to_user(p, &this_call, sizeof this_call) != 0)
        {
			return -EFAULT;
        }
        else
        {
            return 0;
        }
    }

	case CSX_FIRE_PCIE_DMA:
	{
		struct ioctl_param_buffer_fire_pcie this_call;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_FIRE_PCIE_DMA:\n" );
#endif

		if (copy_from_user(&this_call, p, sizeof this_call) != 0)
			return -EFAULT;

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
		    return -EINVAL;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
		    return -EINVAL;
        }

#if CSX_DEBUG
		printk(KERN_DEBUG
			"%s: csx_fire_pcie_dma on channel %d for desc address %llx\n",
			__FUNCTION__, this_call.channel, this_call.desc_address);
#endif

		reso = csx_fire_pcie_dma(cdd, chan, this_call.buffer, this_call.desc_address, this_call.flags);

		return reso;
	}

    case CSX_COPY_AND_FIRE_PCIE:
    {
		struct ioctl_param_copy_and_fire_pcie this_call;
		int reso;
        int chan;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_COPY_AND_FIRE_PCIE:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call ) != 0 )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: copy_from_user failed\n", __FUNCTION__ );
#endif
			return -EFAULT;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
		    return -EINVAL;
        }
        chan = this_call.channel;

        if( this_call.buffer < 0 || this_call.buffer >= DMA_MAX_BUFFERS )
        {
  		    printk(KERN_WARNING "%s: buffer is out of range (%d)\n", __FUNCTION__, this_call.buffer );
		    return -EINVAL;
        }

		if ((this_call.size == 0) || (this_call.size > cdd->mem->region[chan].desc_buf_size))
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: size error\n", __FUNCTION__ );
#endif
			reso = -EINVAL;
        }
		else
        {
            unsigned long copy_ret;

            /* safe to do the copy */
            if( this_call.buffer == 0 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[0] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIE: copy to buffer A.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( cdd->mem->region[chan].desc_kaddr,
					this_call.user_descriptor_buffer, this_call.size );
            }
            else if( this_call.buffer == 1 && this_call.desc_address == cdd->mem->region[chan].desc_bus_address[1] )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIE: copy to buffer B.\n", __FUNCTION__ );
#endif
                copy_ret = copy_from_user( (char *)cdd->mem->region[chan].desc_kaddr +
                    MAX_DESCRIPTOR_BUFFER_SIZE, this_call.user_descriptor_buffer, this_call.size );
            }
            else
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIE: bad copy location.\n", __FUNCTION__ );
#endif
				return -EINVAL;
            }

			if( copy_ret != 0 )
            {
#if CSX_DEBUG
        		printk(KERN_DEBUG "%s: CSX_COPY_AND_FIRE_PCIE: copy_from_user failed.\n", __FUNCTION__ );
#endif
				return -EINVAL;
            }
            else
            {
        		reso = csx_fire_pcie_dma(cdd, chan, this_call.buffer, this_call.desc_address, this_call.flags);
                if( reso < 0 )
                {
            		printk(KERN_DEBUG "%s: csx_fire_pcie_dma failed.\n", __FUNCTION__ );
                }
            }
		}

		return reso;
    }
    case CSX_PCIE_DMA_INTERRUPT_COUNT:
    {
		struct ioctl_param_pcie_dma_interrupt_count this_call;
		struct csx_mem_file *cmf;
		int count;

		if( copy_from_user(&this_call, p, sizeof this_call ) != 0 )
        {
            return -EINVAL;
        }

        if( this_call.channel < 0 || this_call.channel >= DMA_MAX_CHANNELS )
        {
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            return -EINVAL;
        }

		cmd = container_of(file->f_dentry->d_inode->i_cdev, struct csx_mem_data, cdev);
		cmf = file->private_data;
		switch(this_call.channel)
		{
		case 0:
			count = atomic_read(&cmd->intr_count0);	// DMA channel 0
			if (cmf->intr_count0 != count)
			{
				cmf->intr_count0 = count;
			}
			else
			{
				printk(KERN_WARNING "%s: no interrupt counted for DMA channel (%d)\n", __FUNCTION__, this_call.channel );
			}
			break;
		case 1:
			count = atomic_read(&cmd->intr_count1);	// DMA channel 1
			if (cmf->intr_count1 != count)
			{
				cmf->intr_count1 = count;
			}
			else
			{
				printk(KERN_WARNING "%s: no interrupt counted for DMA channel (%d)\n", __FUNCTION__, this_call.channel );
			}
			break;
		default:
  		    printk(KERN_WARNING "%s: channel is out of range (%d)\n", __FUNCTION__, this_call.channel );
            return -EINVAL;
		};
		return 0;
	}

    case CSX_PCI_CONFIG_SPACE:
    {
		struct ioctl_param_pci_config_space this_call;
        u8 *buffer;
        u8 *p_buffer;
        int i;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_PCI_CONFIG_SPACE:\n" );
#endif

        

		if( copy_from_user(&this_call, p, sizeof this_call ) != 0 )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: copy_from_user (1) failed\n", __FUNCTION__ );
#endif
			return -EFAULT;
        }


        // allocate memory to copy the data across
        buffer = kmalloc(this_call.length, GFP_KERNEL);
        if( buffer == NULL )
        {
            return -ENOMEM;
        }


        if( copy_from_user(buffer, this_call.data, this_call.length ) != 0 )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: copy_from_user (2) failed\n", __FUNCTION__ );
#endif
			return -EFAULT;
        }
        
        p_buffer = buffer;

        for( i=this_call.offset; i<(this_call.offset+this_call.length); i++ )
        {
            if( this_call.is_read )
            {
                pci_read_config_byte( cdd->pcidev, i, p_buffer );
            }
            else
            {
                pci_write_config_byte( cdd->pcidev, i, *p_buffer );
            }
            
            p_buffer++;
        }

        if( this_call.is_read )
        {
            if (copy_to_user(this_call.data, buffer, this_call.length) != 0)
            {
                kfree(buffer);
                return -EFAULT;
            }

            if (copy_to_user(p, &this_call, sizeof this_call) != 0)
            { 
                kfree(buffer);
                return -EFAULT;
            }
        }
        kfree(buffer);
		return 0;
    }

    case CSX_ESCAPE:
    {
		struct ioctl_param_kernel_escape this_call;
        u8 val;

#if CSX_DEBUG_IOCTLS
        printk(KERN_DEBUG "CSX_ESCAPE:\n" );
#endif

		if( copy_from_user(&this_call, p, sizeof this_call ) != 0 )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: copy_from_user failed\n", __FUNCTION__ );
#endif
			return -EFAULT;
        }

        /* check checksum */
        if( (this_call.escape_index + this_call.escape_param1 + this_call.escape_param2 +
            this_call.escape_return1 + this_call.escape_return2) != this_call.checksum )
        {
#if CSX_DEBUG
    		printk(KERN_DEBUG "%s: Escape checksum error - rejecting it.\n", __FUNCTION__ );
#endif
			return -EINVAL;
        }

        /* look at escape index now */
        switch( this_call.escape_index )
        {
        case CSX_KERNEL_ESCAPE_VERSION:
#if CSX_DEBUG_IOCTLS
            printk(KERN_DEBUG "CSX_KERNEL_ESCAPE_VERSION:\n" );
#endif
            this_call.escape_return1 = CSX_IOCTL_VERSION;
            break;

        case CSX_KERNEL_ESCAPE_CARD_TYPE:
#if CSX_DEBUG_IOCTLS
            printk(KERN_DEBUG "CSX_KERNEL_ESCAPE_CARD_TYPE:\n" );
#endif
            this_call.escape_return1 = cdd->pci_dev_id;
            pci_read_config_byte( cdd->pcidev, 0x8, (u8 *) &(this_call.escape_return2) );
            break;

        case CSX_KERNEL_ESCAPE_REQUEST_CAPABILITY:
#if CSX_DEBUG_IOCTLS
            printk(KERN_DEBUG "CSX_KERNEL_ESCAPE_REQUEST_CAPABILITY:\n" );
#endif
		    printk(KERN_WARNING "%s: Escape capability not implemented!\n", __FUNCTION__ );
		    return -EINVAL;
            break;

        case CSX_KERNEL_ESCAPE_PCIE_LANE_WIDTH:
#if CSX_DEBUG_IOCTLS
            printk(KERN_DEBUG "CSX_KERNEL_ESCAPE_PCIE_LANE_WIDTH:\n" );
#endif
            pci_read_config_byte( cdd->pcidev, 0x82, &val );
            this_call.escape_return1 = val >> 4;
            break;

        default:
    		printk(KERN_WARNING "%s: Escape index out of range - rejecting it.\n", __FUNCTION__ );
			return -EINVAL;
        }

        if (copy_to_user(p, &this_call, sizeof this_call) != 0)
        {
    		printk(KERN_WARNING "%s: Failed to copy data back to user.\n", __FUNCTION__ );
			return -EFAULT;
        }

		return 0;
    }

	default:
        printk(KERN_WARNING "Bad ioctl command !\n" );
		return -ENOTTY;
	}
}

/* Seek to a particular position in the memory aperture */
static loff_t
csx_mem_llseek(struct file *filp, loff_t off, int whence)
{
	struct csx_mem_data *cmd;
	loff_t pos;

	cmd = container_of(filp->f_dentry->d_inode->i_cdev,
			   struct csx_mem_data, cdev);

	switch (whence) {
		case 0:	/* SEEK_SET */
			pos = off;
			break;
		case 1:	/* SEEK_CUR */
			pos = filp->f_pos + off;
			break;
		case 2:	/* SEEK_END */
			pos = cmd->len + off;
			break;
		default:
			return -EINVAL;
	}

	/* Verify seek does not land outside memory area */
	if (pos < 0 || pos > cmd->len)
		return -EINVAL;
	filp->f_pos = pos;
	return pos;
}

/* Memory-map the memory aperture.
 * XXX: On SGI Altix, the user mapping is unreliable before Linux 2.6.17.
 */
static int
csx_mem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct csx_mem_data *cmd;
	unsigned long off;
	unsigned long vsize;
	int ret;

	cmd = container_of(filp->f_dentry->d_inode->i_cdev,
			   struct csx_mem_data, cdev);

	/* Verify request does not extend past end of memory aperture. */
	off = vma->vm_pgoff << PAGE_SHIFT;
	vsize = vma->vm_end - vma->vm_start;
	if (off + vsize > cmd->len)
		return -EINVAL;

	/* Ensure mapping is suitable for IO */
	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

#if LINUX_VERSION_CODE <=  KERNEL_VERSION(2,6,12) 	 
    ret = io_remap_page_range(vma, vma->vm_start,
        cmd->bar + off,
        vsize, vma->vm_page_prot);
#else 	 
    ret = remap_pfn_range(vma, vma->vm_start, ((unsigned long) cmd->bar+off)>>PAGE_SHIFT, 	 
        vsize, vma->vm_page_prot); 	 
#endif 	 

	return ret;
}

/* Open the memory aperture */
static int
csx_mem_open(struct inode *inode, struct file *filp)
{
	struct csx_mem_data *cmd;
	struct csx_mem_file *cmf;

	cmd = container_of(inode->i_cdev, struct csx_mem_data, cdev);
#if CSX_DEBUG
	printk(KERN_DEBUG "%s: cmd %p\n", __FUNCTION__, cmd);
#endif

	cmf = kmalloc(sizeof(struct csx_mem_file), GFP_KERNEL);
	if (NULL == cmf)
		return -ENOMEM;

	filp->private_data = cmf;
	atomic_set(&cmd->intr_count0, 0);	// reset DMA channel 0 counter
	atomic_set(&cmd->intr_count1, 0);	// reset channel 1
	cmf->intr_count0 = atomic_read(&cmd->intr_count0);
	cmf->intr_count1 = atomic_read(&cmd->intr_count1);

	MMIOWB();
	return 0;
}

/* Wait for a DMA interrupt */
static unsigned int
csx_mem_poll(struct file *filp, struct poll_table_struct *polltab)
{
	struct csx_mem_data *cmd;
	struct csx_mem_file *cmf;
	__s32 sample;

	unsigned int return_flags = 0;

	cmd = container_of(filp->f_dentry->d_inode->i_cdev,
		struct csx_mem_data, cdev);
	cmf = filp->private_data;

	poll_wait(filp, &cmd->intr_queue, polltab);

	sample = atomic_read(&cmd->intr_count0);	// DMA channel 0
	if (cmf->intr_count0 != sample)
	{
        //cmf->intr_count0 = sample;
		return_flags |= (POLLIN | POLLRDNORM);
	}
	sample = atomic_read(&cmd->intr_count1);	// DMA channel 1
	if (cmf->intr_count1 != sample)
	{
        //cmf->intr_count1 = sample;
		return_flags |= (POLLOUT | POLLWRNORM);
	}

    return return_flags;
}

/* Read device memory.
 * XXX: Ideally this code would know how to do DMAs.
 * XXX: Without DMAs, it should at least pin the target memory pages instead
 *	of blindly hoping they don't get remapped while the DMA is in flight.
 */
static ssize_t
csx_mem_read(struct file *filp, char __user * data, size_t size, loff_t * off)
{
	struct csx_mem_data *cmd;
	struct csx_mem_data *cmf;

	cmd = container_of(filp->f_dentry->d_inode->i_cdev,
			   struct csx_mem_data, cdev);
	cmf = filp->private_data;

	/* Verify access doesn't fall outside memory area */
	if ((*off < 0) || (*off + size > cmd->len))
		return -EINVAL;

#if CS_NOMAPMEM
	/* Read from IO space */
	memcpy_fromio(data, &cmd->mem[*off], size);
#endif

	*off += size;
	return size;
}

/* Release per-file resources for memory aperture */
static int
csx_mem_release(struct inode *inode, struct file *filp)
{
	struct csx_mem_file *cmf;
	struct csx_mem_data *cmd;

	cmd = container_of(inode->i_cdev, struct csx_mem_data, cdev);

	MMIOWB();
	cmf = filp->private_data;
	kfree(cmf);
	return 0;
}

/* Write device memory
 * XXX: Ideally this code would know how to do DMAs.
 * XXX: Without DMAs, it should at least pin the source system memory pages
 *	instead of blindly hoping they don't get remapped while the DMA is in
 *	flight.
 */
static ssize_t
csx_mem_write(struct file *filp, const char __user * data, size_t size,
	      loff_t * off)
{
	struct csx_mem_data *cmd;
	struct csx_mem_file *cmf;

	cmd = container_of(filp->f_dentry->d_inode->i_cdev,
			   struct csx_mem_data, cdev);
	cmf = filp->private_data;

	/* Verify access doesn't fall outside memory area */
	if ((*off < 0) || (*off + size > cmd->len))
		return -EINVAL;

#if CS_NOMAPMEM
	/* Write from kernel space to IO space */
	/* XXX: On Altix this is dangerous, as you can run into the
	 * "PIO write storm" problem, which will panic the machine.
	 * Basically, PIO writes need to be slowed down.
	 */
	memcpy_toio(&cmd->mem[*off], (void *) data, size);

	/* Ensure writes arrive at hardware before proceeding */
	MMIOWB();
#endif

	*off += size;
	return size;
}

/******************
 * ioctl dispatch *
 ******************/
struct file_operations csx_mem_fops = {
	.owner = THIS_MODULE,
	.llseek = csx_mem_llseek,
	.mmap = csx_mem_mmap,
	.open = csx_mem_open,
	.poll = csx_mem_poll,
	.ioctl = csx_mem_ioctl,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11))
// The new version, with its own locking, not using kernel locking
	.unlocked_ioctl = csx_mem_unlocked_ioctl,
// Not sure about this at present
//	.compat_ioctl = csx_mem_unlocked_ioctl,
#endif
	.read = csx_mem_read,
	.release = csx_mem_release,
	.write = csx_mem_write,
};

/********************
 * Class attributes *
 ********************/

/* Implements /sys/class/csx#c/dev */
static ssize_t
csx_show_ctl_dev(struct class_device *class_dev, char *buf)
{
	struct csx_device_data *cdd = class_get_devdata(class_dev);

	return print_dev_t(buf, cdd->ctl->dev);
}

/* Change this version whenver a released driver changes the format of
 * the /sys/class/csx/csx#c/status file.
 */
#define STATUS_VERSION "1"

/* Implements /sys/class/csx#c/status */
static ssize_t
csx_show_status(struct class_device *class_dev, char *buf)
{
	struct csx_device_data *cdd = class_get_devdata(class_dev);
	ssize_t len = 0;
	uint32_t fpga_version;

	/* Emit a status version, so that userland utilities can
	 * adapt to any changes in the output.
	 */
	len += snprintf(&buf[len], PAGE_SIZE - len, "Status version: %s\n", STATUS_VERSION);

	/* Driver build information */
	len += snprintf(&buf[len], PAGE_SIZE - len, "Driver build time: %s %s\n", __DATE__, __TIME__);

	/* Read FPGA version register */
    csx_read_register( cdd->ctl, RegSetFromID( cdd ), 0, REG_INDEX_HIF_VERSION, &fpga_version );
	len += snprintf(&buf[len], PAGE_SIZE - len, "FPGA version: 0x%x\n", fpga_version);

	/* Additional items of interest (voltages, temps, etc.) follow
	 * this form:
	 *      value = readl(&cdd->ctl->regs[SOME_OFFSET]);
	 *      item = some_calculation(value);
	 *      len += snprintf(&buf[len], PAGE_SIZE-len,
	 *                      "Item name: %d\n", item);
	 *
	 * Ensure that the total buffer length doesn't exceed a single
	 * page.  The above code knows that the output is currently tiny,
	 * so it doesn't perform any such bounds checking.
	 */

	return len;
}

#define NULLATTR { .show = NULL, .store = NULL }

static struct class_device_attribute csx_ctl_attrs[] = {
	{
	 .attr = {.owner = THIS_MODULE,
		  .name = "dev",
		  .mode = 0444},
	 .show = csx_show_ctl_dev,
	 .store = NULL,
	 },
	{
	 .attr = {.owner = THIS_MODULE,
		  .name = "status",
		  .mode = 0444},
	 .show = csx_show_status,
	 .store = NULL,
	 },
	NULLATTR,
};

/* Implements /sys/class/csx#m/dev */
static ssize_t
csx_show_mem_dev(struct class_device *class_dev, char *buf)
{
	struct csx_device_data *cdd = class_get_devdata(class_dev);

	return print_dev_t(buf, cdd->mem->dev);
}

static struct class_device_attribute csx_mem_attrs[] = {
	{
	 .attr = {.owner = THIS_MODULE,
		  .name = "dev",
		  .mode = 0444},
	 .show = csx_show_mem_dev,
	 .store = NULL,
	 },
	NULLATTR
};

/*******************************
 * /proc entry management      *
 *******************************/

/*******************************
 * Main display function       *
 *******************************/
static int
csx_proc_info ( char *page, char **start, off_t offset, int count, int *eof, void *data)
{
  struct csx_device_stats *stats = (struct csx_device_stats*)data;
  int len = 0;
  
  if (stats) {
    len += sprintf(page+len, "Pid         : %d\n", stats->pid);
    len += sprintf(page+len, "Bytestodev  : %d\n", stats->kbytes_to_device);
    len += sprintf(page+len, "Bytesfromdev: %d\n", stats->kbytes_from_device);
    len += sprintf(page+len, "Interrupts  : %d\n", atomic_read( &stats->interrupts_taken ) );
    len += sprintf(page+len, "Tempcpu0    : %d\n", stats->temp_cpu0);
    len += sprintf(page+len, "Tempcpu1    : %d\n", stats->temp_cpu1);
  }
  
  if (len <= offset+count) 
    *eof = 1;
  
  *start = page + offset;
  len -= offset;
  if (len>count) 
    len = count;
  if (len<0) 
    len = 0;
  return len;
}

/*******************************
 * Device creation/destruction *
 *******************************/

/* Create device attribute files (e.g. "dev", "status") */
static int
csx_add_classdev_attrs(struct class_device *classdev,
		       struct class_device_attribute *attrs)
{
	struct class_device_attribute *curr;
	const char *attrname;
	int ret;

/*  the class_device_register() will create the /sys/.../dev entry for us
 *  which udev will pick up and create the /dev/csx?c/m file
 *  ( therefore we DONT create it here for later kernels - currently RHE5 and SLES10 )
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13))
	for (curr = attrs; curr->show || curr->store; curr++) {
		ret = class_device_create_file(classdev, curr);
		if (ret) {
			attrname = curr->attr.name;
			goto out;
		}
	}
#else
	for (curr = attrs; curr->show || curr->store; curr++) {
		if(strncmp(curr->attr.name, "dev", 3) != 0) {
			ret = class_device_create_file(classdev, curr);
			if (ret) {
				attrname = curr->attr.name;
				goto out;
			}
		}
	}
#endif

	return 0;
      out:
	while (curr != attrs) {
		class_device_remove_file(classdev, curr);
		curr--;
	}
	printk(KERN_WARNING "%s: Failed to add class device attribute "
	       "\"%s\"\n", __FUNCTION__, attrname);
	return ret;
}

/* Remove device attribute files */
static void
csx_remove_classdev_attrs(struct class_device *classdev,
			  struct class_device_attribute *attrs)
{
	struct class_device_attribute *curr;

/* if we have an 'older' kernel we DIDNT create the /sys/.../dev entry, dont try to delete it */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13))
	for (curr = attrs; curr->show || curr->store; curr++)
		class_device_remove_file(classdev, curr);
#else
	for (curr = attrs; curr->show || curr->store; curr++) {
    if(strncmp(curr->attr.name, "dev", 3) != 0) {
      class_device_remove_file(classdev, curr);
    }
  }
#endif
}

/*************************
 * PCI driver management *
 *************************/

static int
csx_setup_ctl(struct csx_device_data *cdd)
{
	int ret;
	char *errmsg;

	cdd->ctl = kmalloc(sizeof(struct csx_ctl_data), GFP_KERNEL);
	if (NULL == cdd->ctl)
		return -ENOMEM;
	memset(cdd->ctl, 0, sizeof(struct csx_ctl_data));
	cdd->ctl->maindata = cdd;

	/* Establish control register mappings */
	cdd->ctl->bar = pci_resource_start(cdd->pcidev, 0);
	if (0 == cdd->ctl->bar) {
		errmsg = "Unable to find control register resource";
		ret = -ENODEV;
		goto out_bar;
	}
	cdd->ctl->len = pci_resource_len(cdd->pcidev, 0);
	if (!request_mem_region(cdd->ctl->bar, cdd->ctl->len, "csx_ctl")) {
		errmsg = "Unable to obtain control register resource";
		ret = -ENODEV;
		goto out_bar;
	}
	cdd->ctl->regs = ioremap_nocache(cdd->ctl->bar, cdd->ctl->len);
	if (NULL == cdd->ctl->regs) {
		errmsg = "Unable to remap control register resource";
		ret = -ENODEV;
		goto out_remap;
	}

	/* Initialize interrupt count and wait queue */
	atomic_set(&cdd->ctl->intr_count, 0);
	init_waitqueue_head(&cdd->ctl->intr_queue);

	/* Create character special device */
	cdd->ctl->dev = cdd->dev_base + CSX_CTL_DEVNUM;
	cdd->ctl->cdev.owner = THIS_MODULE;
	kobject_set_name(&cdd->ctl->cdev.kobj, "csx%dc",
			 MINOR(cdd->ctl->dev) / CSX_NUM_DEVS);
	cdev_init(&cdd->ctl->cdev, &csx_ctl_fops);
	ret = cdev_add(&cdd->ctl->cdev, cdd->ctl->dev, 1);
	if (ret) {
		errmsg = "Failed to add control register cdev";
		goto out_cdev;
	}

	/* Add to device class */
	cdd->ctl->classdev.class = &csx_ctl_class;
	cdd->ctl->classdev.dev = &cdd->pcidev->dev;
	snprintf(cdd->ctl->classdev.class_id, BUS_ID_SIZE, "csx%dc",
		 MINOR(cdd->ctl->dev) / CSX_NUM_DEVS);

	class_set_devdata(&cdd->ctl->classdev, cdd);

/* allow for RHE5 and SLES10, so udev will create the /dev/csx?c/m file */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13))
	cdd->ctl->classdev.devt = cdd->ctl->dev;
#endif

	ret = class_device_register(&cdd->ctl->classdev);
	if (ret)
    {
		errmsg = "Failed to add control register class device";
		goto out_classdev;
	}

	/* Create attribute files */
	ret = csx_add_classdev_attrs(&cdd->ctl->classdev, csx_ctl_attrs);
	if (ret)
    {
		errmsg = "Failed to add control register class device attributes";
		goto out_attr;
	}

    /* Create stats entry */
    cdd->ctl->stats = kmalloc(sizeof(struct csx_device_stats), GFP_KERNEL);
    /* Unusually we don't need to check for failure. If this fails then we just
    don't get any /proc information */
    if (cdd->ctl->stats)
    {
        memset(cdd->ctl->stats, 0, sizeof(struct csx_device_stats));
    }

    /* Connect to /proc entry */        
    if (cdd->ctl->stats)
    {
        char buff[20]; /* Enough room for driver/csx99. */
        const int devid =  MINOR(cdd->ctl->dev) / CSX_NUM_DEVS;
        sprintf(buff, str(PROC_PREFIX) "csx%d", devid);
        cdd->ctl->stats->devid = devid;

        /* Create the /proc entry. */
        create_proc_read_entry(buff, 0, NULL, csx_proc_info, cdd->ctl->stats);
    }

    MMIOWB();
	return 0;

	/* Before the class device is registered, we need
	 * to perform error teardown ourself.
	 */
out_classdev:
	cdev_del(&cdd->ctl->cdev);
out_cdev:
	iounmap(cdd->ctl->regs);
out_remap:
	release_mem_region(cdd->ctl->bar, cdd->ctl->len);
out_bar:
	kfree(cdd->ctl);
	goto out;

	/* After the class device is registered, the release
	 * method performs error teardown.
	 */
out_attr:
	class_device_unregister(&cdd->ctl->classdev);
out:
	printk(KERN_WARNING "%s: %s for pci_dev 0x%p\n",
	       __FUNCTION__, errmsg, cdd->pcidev);
	return ret;
}

static void
csx_teardown_ctl(struct csx_device_data *cdd)
{
	/* Remove control register device */
	csx_remove_classdev_attrs(&cdd->ctl->classdev, csx_ctl_attrs);
        /* Remove /proc entry if there is one */
        if (cdd->ctl->stats)
        {
          char buf[20];
          sprintf(buf, str(PROC_PREFIX) "csx%d", cdd->ctl->stats->devid);
          remove_proc_entry(buf, NULL /* Parent Directory */);
        }
  class_device_unregister(&cdd->ctl->classdev);
}

static int
csx_setup_mem(struct csx_device_data *cdd)
{
	int ret;
    int chan;
	char *errmsg;
	void *desc_coherent_buffer;
	dma_addr_t desc_bus_address;

	cdd->mem = kmalloc(sizeof(struct csx_mem_data), GFP_KERNEL);
	if (NULL == cdd->mem) {
		errmsg = "Failed to allocate memory device data structure";
		ret = -ENOMEM;
		goto out;
	}
	memset(cdd->mem, 0, sizeof(struct csx_mem_data));
	cdd->mem->maindata = cdd;

	/* Set DMA address mask */
	dma_set_mask(&cdd->pcidev->dev, ~0ULL);
	if( pci_set_consistent_dma_mask(cdd->pcidev, ~0ULL) )
    {
        errmsg = "Failed to set consistent DMA mask";
		ret = -ENOMEM;
		goto out_desc;
    }

    switch( cdd->pci_dev_id )
    {
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E511:
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E521:
        cdd->mem->dma_num_channels = 1;
        break;

    case PCI_DEVICE_ID_CLEARSPEED_CSX_E701:
    case PCI_DEVICE_ID_CLEARSPEED_CSX_E711:
        cdd->mem->dma_num_channels = 2;
        break;

    default:
        cdd->mem->dma_num_channels = 1;
        printk(KERN_WARNING "Unknown device ID, setting dma channels to 1.\n" );
        break;
    }

	MMIOWB();

	/* Obtain coherent mapping for the dma descriptor buffer.
	 * this will be remapped to user space in the dma mmap function
	 * These coherent mappings remain for the life of the driver. 
	 */

    /* Allocating two buffers so that we may prepare the second
     * buffer whilst DMA'ing the first
     */
    for( chan=0; chan< DMA_MAX_CHANNELS; chan++ )
    {
	    desc_coherent_buffer = dma_alloc_coherent(&cdd->pcidev->dev,
            MAX_DESCRIPTOR_BUFFER_SIZE * 2, &desc_bus_address, GFP_KERNEL);

	    if (!desc_coherent_buffer)
        {
		    errmsg = "Failed to allocate descriptor coherent buffer";
		    ret = -ENOMEM;
		    goto out_desc;
	    }

	    cdd->mem->region[chan].desc_bus_address[0] = desc_bus_address;
	    cdd->mem->region[chan].desc_bus_address[1] = desc_bus_address + MAX_DESCRIPTOR_BUFFER_SIZE;
	    cdd->mem->region[chan].desc_kaddr = desc_coherent_buffer;
	    cdd->mem->region[chan].desc_buf_size = MAX_DESCRIPTOR_BUFFER_SIZE;
    }

	/* Establish memory mappings */
	cdd->mem->bar = pci_resource_start(cdd->pcidev, 2);
	if (0 == cdd->mem->bar) {
		errmsg = "Unable to find memory resource";
		ret = -ENODEV;
		goto out_bar;
	}
	cdd->mem->len = pci_resource_len(cdd->pcidev, 2);
	if (!request_mem_region(cdd->mem->bar, cdd->mem->len, "csx_mem")) {
		errmsg = "Unable to obtain memory resource";
		ret = -ENODEV;
		goto out_bar;
	}
#if CS_NOMAPMEM
	cdd->mem->mem = ioremap(cdd->mem->bar, cdd->mem->len);
	if (NULL == cdd->mem->mem) {
		errmsg = "Unable to remap memory resource";
		ret = -ENODEV;
		goto out_remap;
	}
#else
	cdd->mem->mem = 0;
#endif

	/* Initialize interrupt count and wait queues */
	atomic_set(&cdd->mem->intr_count0, 0);		// DMA channel 0
	atomic_set(&cdd->mem->intr_count1, 0);		// DMA channel 1
	init_waitqueue_head(&cdd->mem->intr_queue);

	/* Create character special device */
	cdd->mem->dev = cdd->dev_base + CSX_MEM_DEVNUM;
	cdd->mem->cdev.owner = THIS_MODULE;
	kobject_set_name(&cdd->mem->cdev.kobj, "csx%dm",
			 MINOR(cdd->mem->dev) / CSX_NUM_DEVS);
	cdev_init(&cdd->mem->cdev, &csx_mem_fops);
	ret = cdev_add(&cdd->mem->cdev, cdd->mem->dev, 1);
	if (ret)
    {
		errmsg = "Failed to add memory cdev";
		goto out_cdev;
	}

	/* Add to device class */
	cdd->mem->classdev.class = &csx_mem_class;
	cdd->mem->classdev.dev = &cdd->pcidev->dev;
	snprintf(cdd->mem->classdev.class_id, BUS_ID_SIZE, "csx%dm",
		 MINOR(cdd->mem->dev) / CSX_NUM_DEVS);
	class_set_devdata(&cdd->mem->classdev, cdd);

/* allow for RHE5 and SLES10, so udev will create the /dev/csx?c/m file */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13))
	cdd->mem->classdev.devt = cdd->mem->dev;
#endif

	ret = class_device_register(&cdd->mem->classdev);
	if (ret)
    {
		errmsg = "Failed to add memory class device";
		goto out_classdev;
	}

	/* Create attribute files */
	ret = csx_add_classdev_attrs(&cdd->mem->classdev, csx_mem_attrs);
	if (ret) {
		errmsg = "Failed to add memory class device attributes";
		goto out_attr;
	}

	return 0;

	/* Before the class device is registered, we need
	 * to perform error teardown ourself.
	 */
out_classdev:
	cdev_del(&cdd->mem->cdev);
out_cdev:
#if CS_NOMAPMEM
	iounmap(cdd->mem->mem);
out_remap:
#endif
	release_mem_region(cdd->mem->bar, cdd->mem->len);
out_bar:
    for( chan=0; chan< DMA_MAX_CHANNELS; chan++ )
    {
        if( cdd->mem->region[chan].desc_kaddr != NULL )
        {
            desc_bus_address = (dma_addr_t)cdd->mem->region[chan].desc_bus_address[0];

	        dma_free_coherent(&cdd->pcidev->dev, MAX_DESCRIPTOR_BUFFER_SIZE * 2,
                cdd->mem->region[chan].desc_kaddr, desc_bus_address);

            cdd->mem->region[chan].desc_kaddr = NULL;
            cdd->mem->region[chan].desc_bus_address[0] = 0;
        }
    }
out_desc:
	kfree(cdd->mem);
	goto out;

	/* After the class device is registered, the release method performs error teardown. */
out_attr:
	class_device_unregister(&cdd->mem->classdev);
out:
	printk(KERN_WARNING "%s: %s for pci_dev 0x%p\n", __FUNCTION__, errmsg, cdd->pcidev);
	return ret;
}

static void
csx_teardown_mem(struct csx_device_data *cdd)
{
	/* Remove memory device */
	csx_remove_classdev_attrs(&cdd->mem->classdev, csx_mem_attrs);
	class_device_unregister(&cdd->mem->classdev);
}

/* Sets up driver resources upon discovery of a new CSX device */
static int
csx_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	int ret;
	char *errmsg;
	struct csx_device_data *cdd;

    /* Enable device and turn on bus mastering */
	ret = pci_enable_device(pdev);
	if (ret) {
		errmsg = "Failed to enable device";
		goto out_enable;
	}
	pci_set_master(pdev);

	/* Create per-CSX data */
	cdd = kmalloc(sizeof(struct csx_device_data), GFP_KERNEL);
	if (NULL == cdd) {
		errmsg = "Failed to allocate device data";
		ret = -ENOMEM;
		goto out_kmalloc;
	}

#if CSX_DEBUG
        printk(KERN_DEBUG "%s: created cdd %p\n", __FUNCTION__, cdd);
#endif
	pci_set_drvdata(pdev, cdd);
	cdd->pcidev = pdev;
    cdd->pci_dev_id = pci_id->device;

	/* Reserve minor device numbers for this card */
	spin_lock(&nextdev_lock);
	cdd->dev_base = nextdev;
	nextdev += CSX_NUM_DEVS;
	spin_unlock(&nextdev_lock);

	/* Set up individual devices */
	if (0 != (ret = csx_setup_ctl(cdd))) {
		errmsg = "Failed to setup control device";
		goto out_ctl;
	}
	if (0 != (ret = csx_setup_mem(cdd))) {
		errmsg = "Failed to setup memory device";
		goto out_mem;
	}

#ifdef CSX_MSI_ENABLE
    if (csx_use_msi)
    {
        int msi_status;

        msi_status = pci_enable_msi(pdev);
        if (msi_status)
        {
        errmsg = "Failed to enable msi";
        goto out_msi_handler;
        }
    }
#endif

	/* Register interrupt handler */
	ret = request_irq(pdev->irq, csx_intr_handler, IRQF_SHARED, "csx", cdd);
	if (ret) {
		errmsg = "Failed to register interrupt handler";
		goto out_handler;
	}
	return 0;

out_handler:
#ifdef CSX_MSI_ENABLE
    if (csx_use_msi)
    {
        pci_disable_msi(pdev);
    }
out_msi_handler:
#endif
	csx_teardown_mem(cdd);
out_mem:
	csx_teardown_ctl(cdd);
out_ctl:
	kfree(cdd);
    /* Try to keep successful devices in numerical sequence */
	nextdev -= CSX_NUM_DEVS;
out_kmalloc:
	pci_disable_device(pdev);
out_enable:
	printk(KERN_WARNING "%s: %s for pci_dev 0x%p\n",
	       __FUNCTION__, errmsg, pdev);
	return ret;
}

/* Removes driver resources upon removal of an existing CSX device */
static void
csx_remove(struct pci_dev *pdev)
{
	struct csx_device_data *cdd;

	cdd = pci_get_drvdata(pdev);

	free_irq(cdd->pcidev->irq, cdd);
	csx_teardown_mem(cdd);
	csx_teardown_ctl(cdd);
#ifdef CSX_MSI_ENABLE
    if (csx_use_msi)
    {
        pci_disable_msi(pdev);
    }
#endif
	pci_disable_device(pdev);
	kfree(cdd);
}

static struct pci_device_id csx_id_table[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_CLEARSPEED, PCI_DEVICE_ID_CLEARSPEED_CSX_E511)},
	{PCI_DEVICE(PCI_VENDOR_ID_CLEARSPEED, PCI_DEVICE_ID_CLEARSPEED_CSX_E521)},
	{PCI_DEVICE(PCI_VENDOR_ID_CLEARSPEED, PCI_DEVICE_ID_CLEARSPEED_CSX_E701)},
	{PCI_DEVICE(PCI_VENDOR_ID_CLEARSPEED, PCI_DEVICE_ID_CLEARSPEED_CSX_E711)},
	{0}
};

MODULE_DEVICE_TABLE(pci, csx_id_table);

static struct pci_driver __devinitdata csx_driver = {
	.name = "CSX",
	.id_table = csx_id_table,
	.probe = csx_probe,
	.remove = csx_remove,
};

/*********************
 * Module management *
 *********************/

/* Get dynamic device numbers and register device class */
static int __devinit
csx_init(void)
{
	char *errmsg;
	int ret;

#if CSX_DEBUG
    printk(KERN_DEBUG "Initialising OSD Driver.\n" );
#endif

    // Set up the ioctl locki semaphore
    sema_init(&ioctl_sem, 1);

	/* Get block of dynamic device numbers */
	ret = alloc_chrdev_region(&firstdev, 0, 256, "csx");
	if (ret) {
		errmsg = "Failed to allocate device number block";
		goto out_dev;
	}
	nextdev = firstdev;
	nextdev_lock = SPIN_LOCK_UNLOCKED;

	/* Class to allow userland to find the device numbers and
	 * automatically create /dev entries.
	 */
	ret = class_register(&csx_ctl_class);
	if (ret) {
		errmsg = "Failed to create control device class";
		goto out_ctl_class;
	}
	ret = class_register(&csx_mem_class);
	if (ret) {
		errmsg = "Failed to create memory device class";
		goto out_mem_class;
	}

	/* Register device driver */
	pci_register_driver(&csx_driver);

	return 0;

out_mem_class:
	class_unregister(&csx_ctl_class);
out_ctl_class:
	unregister_chrdev_region(firstdev, 256);
out_dev:
	printk(KERN_WARNING "%s: %s (%d)\n", __FUNCTION__, errmsg, ret);
	return ret;
}

/* Unregister device class and free dynamic device numbers. */
static void __devexit
csx_exit(void)
{
	/* Unregister device driver */
	pci_unregister_driver(&csx_driver);

	/* Remove device classes */
	class_unregister(&csx_mem_class);
	class_unregister(&csx_ctl_class);

	/* Release device number block */
	unregister_chrdev_region(firstdev, 256);

#if CSX_DEBUG
    printk(KERN_DEBUG "Exiting OSD Driver.\n" );
#endif
}

module_init(csx_init);
module_exit(csx_exit);

MODULE_AUTHOR("Silicon Graphics, Inc. / ClearSpeed Technology Plc");
MODULE_DESCRIPTION("ClearSpeed Advance device driver");
MODULE_LICENSE("GPL");
