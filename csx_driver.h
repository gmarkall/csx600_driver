//
//
//      Copyright (c) ClearSpeed Technology plc 2000,2002,2006
//      All rights reserved.
//
//      Copyright (C) 2006 Silicon Graphics, Inc.  All Rights Reserved.
//
// This material is provided "as is", with absolutely no warranty expressed
// or implied. Any use is at your own risk. Permission to use or copy this
// software for any purpose is hereby granted without fee, provided the
// above notices are retained on all copies. Permission to modify the code
// and to distribute modified code is granted, provided the above notices
// are retained, and a notice that the code was modified is included with
// the above copyright notice.
//
//
#ifndef LIBKERNELUSER_H
#define LIBKERNELUSER_H


///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Common definitions
//
///////////////////////////////////////////////////////////////////////////////////////////////////
#define MAX_SINGLE_DMA_TRANSFER                 (512 * 1024)

// Halfbridge = 32, PCIe primary is 128 (rounded up)
#define MAX_DESCRIPTOR_SIZE                     128


///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  These definitons then follow
//  Maximum number of scatter gather entries, also maximum number of descriptor entires
//
//  The number of SG entries is the size of the largest buffer divided by the amount we can
//  fit into a descriptor. The max read PCI-X size is 32K and the page size can range from
//  4K up to about 64K. Therefore if 4K then we need a descriptor per page but if page size
//  is > 32K then we need a descriptor per 32K.
//
//  Descriptor buffer is rounded up to next page boundary
///////////////////////////////////////////////////////////////////////////////////////////////////
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX_SG_SIZE                             ((MAX_SINGLE_DMA_TRANSFER/ MIN( PAGE_SIZE, (32 * 1024) )) + 2)
#define MAX_DESCRIPTOR_BUFFER_SIZE              ((MAX_SG_SIZE * MAX_DESCRIPTOR_SIZE + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1))


///////////////////////////////////////////////////////////////////////////////////
//
//   IMPORTANT !!!
//
//   Update this version number every time you check in the LLPCI or LLOSD KERNEL
//   with a DIFFERENT ioctl parameter passing set !
//
///////////////////////////////////////////////////////////////////////////////////
#define CSX_IOCTL_VERSION 3004

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  ioctl definitions
//
///////////////////////////////////////////////////////////////////////////////////////////////////
#define CSX_IOCTL_MAGIC                         0xC5   // aka clearspeed

#define CSX_GET_COHERENT_BUFFER_INFO _IOR(CSX_IOCTL_MAGIC, 0x90, struct ioctl_param_get_coherent_map_info)
#define CSX_LOCK_DATA_BUFFER         _IOWR(CSX_IOCTL_MAGIC, 0x91, struct ioctl_param_buffer_lock)
#define CSX_UNLOCK_DATA_BUFFER       _IOWR(CSX_IOCTL_MAGIC, 0x92, struct ioctl_param_buffer_unlock)
#define CSX_FIRE_PCIX_DMA            _IOR(CSX_IOCTL_MAGIC, 0x93, struct ioctl_param_buffer_fire_pcix)
#define CSX_FIRE_PCIE_DMA            _IOR(CSX_IOCTL_MAGIC, 0x94, struct ioctl_param_buffer_fire_pcie)
#define CSX_COPY_TO_DESC_BUFFER      _IOR(CSX_IOCTL_MAGIC, 0x95, struct ioctl_param_copy_to_coherent_descriptor_buffer)
#define CSX_COPY_AND_FIRE_PCIX       _IOR(CSX_IOCTL_MAGIC, 0x96, struct ioctl_param_copy_and_fire_pcix)
#define CSX_COPY_AND_FIRE_PCIE       _IOR(CSX_IOCTL_MAGIC, 0x97, struct ioctl_param_copy_and_fire_pcie)
#define CSX_GET_PAGE_SIZE            _IOR(CSX_IOCTL_MAGIC, 0x99, struct ioctl_param_get_page_size)
#define CSX_PCIE_DMA_INTERRUPT_COUNT _IOR(CSX_IOCTL_MAGIC, 0x9A, struct ioctl_param_pcie_dma_interrupt_count)
#define CSX_ESCAPE                   _IOR(CSX_IOCTL_MAGIC, 0x9B, struct ioctl_param_kernel_escape)
#define CSX_PCI_CONFIG_SPACE         _IOWR(CSX_IOCTL_MAGIC, 0x9C, struct ioctl_param_pci_config_space)

///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Page descriptor, typedefs and flags
//
///////////////////////////////////////////////////////////////////////////////////////////////////
typedef unsigned long long bus_address_t;
typedef int status;

struct dma_desc {
        void            *virt_address;
        bus_address_t   bus_address;
        unsigned        offset;
        unsigned        length;
};

#define DMA_FLAG_TO_DEVICE                      0x1
#define DMA_FLAG_FROM_DEVICE                    0x2
#define DMA_FLAG_BIDIRECTONAL                   (DMA_FLAG_TO_DEVICE|DMA_FLAG_FROM_DEVICE)
#define DMA_FLAG_CONTROL                        0x4


///////////////////////////////////////////////////////////////////////////////////////////////////
//
// The ioctl structures
//
///////////////////////////////////////////////////////////////////////////////////////////////////
struct ioctl_param_get_coherent_map_info
{
    int channel;
    bus_address_t descriptor_chain1;
    bus_address_t descriptor_chain2;
};

struct ioctl_param_buffer_lock 
{
    int channel;
    int buffer;
    void * user_address;
    unsigned int user_size;
    int desc_count;
    struct dma_desc * desc_array;
    unsigned int flags;
    unsigned int padding;
};

struct ioctl_param_buffer_unlock
{
    int channel;
    int buffer;
    int desc_count;
    unsigned int flags;
    struct dma_desc *desc_array;
};

struct ioctl_param_buffer_fire_pcix
{
    int channel;
    int buffer;
    bus_address_t desc_address;
    unsigned int flags;
};

struct ioctl_param_buffer_fire_pcie
{
    int channel;
    int buffer;
    bus_address_t desc_address;
    unsigned int flags;
};

struct ioctl_param_copy_to_coherent_descriptor_buffer
{
    int channel;
    int buffer;
    void * user_descriptor_buffer;
    unsigned int size;
    bus_address_t desc_address;
    unsigned int flags;
    unsigned int padding;
};

struct ioctl_param_copy_and_fire_pcix
{
    int channel;
    int buffer;
    void * user_descriptor_buffer;
    unsigned int size;
    bus_address_t desc_address;
    unsigned int flags;
};

struct ioctl_param_copy_and_fire_pcie
{
    int channel;
    int buffer;
    void * user_descriptor_buffer;
    unsigned int size;
    bus_address_t desc_address;
    unsigned int flags;
};

struct ioctl_param_pcie_dma_interrupt_count
{
    int channel;
};

struct ioctl_param_get_page_size
{
    unsigned int page_size_in_bytes;
};

struct ioctl_param_kernel_escape
{
    int escape_index;
    int escape_param1;
    int escape_param2;
    int escape_return1;
    int escape_return2;
    int checksum;
};

struct ioctl_param_pci_config_space
{
    int is_read;
    unsigned int offset;
    unsigned int length;
    unsigned int padding;
    void * data;
};

// Kernel escapes for setting / getting simple parameter values
// Can send / recv up to 2 int types in one call.
// If a bool type is returned then 0 is not supported else
// it will return the version number of what is supported.

// Escape Index
enum {  CSX_KERNEL_ESCAPE_NULL               = 0,
        CSX_KERNEL_ESCAPE_VERSION            = 1,
        CSX_KERNEL_ESCAPE_CARD_TYPE          = 2,
        CSX_KERNEL_ESCAPE_REQUEST_CAPABILITY = 3,
        CSX_KERNEL_ESCAPE_PCIE_LANE_WIDTH    = 4 };

#endif // LIBKERNELUSER_H
