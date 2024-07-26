/*
 * pcie_ats.h:
 *
 * Implementation of ATS emulation support.
 *
 * Copyright (c) 2024 Klaus Jensen <k.jensen@samsung.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#ifndef QEMU_PCIE_ATS_H
#define QEMU_PCIE_ATS_H

#include "hw/pci/pci.h"

struct PCIEATC;
typedef struct PCIEATC PCIEATC;

typedef struct PCIEATSIOMMU {
    PCIDevice *dev;
    int iommu_idx;
    MemoryRegion *mr;
    IOMMUNotifier notifier;
    QLIST_ENTRY(PCIEATSIOMMU) iommu_next;
} PCIEATSIOMMU;

typedef struct PCIEATCEntry {
    IOMMUTLBEntry iotlbe;
    QTAILQ_ENTRY(PCIEATCEntry) lru;
} PCIEATCEntry;

struct PCIEATC {
    bool enabled;
    uint32_t size;

    MemoryListener iommu_listener;
    /* Track IOMMUs whose translations we've cached in the ATC */
    QLIST_HEAD(, PCIEATSIOMMU) iommu_list;

    GHashTable *h;
    QTAILQ_HEAD(, PCIEATCEntry) lru;
    QemuMutex lock;

    struct {
        uint64_t hits, misses, evictions;
    } stats;
};

IOMMUTLBEntry pcie_ats_translate(PCIDevice *dev, IOMMUTLBEntry entry);

MemTxResult pcie_ats_dma_rw(PCIDevice *dev, dma_addr_t addr, void *buf,
                            dma_addr_t len, DMADirection dir,
                            MemTxAttrs attrs);

static inline bool pci_iommu_enabled(PCIDevice *dev)
{
    AddressSpace *dma_as = pci_device_iommu_address_space(dev);

    if (dma_as == &address_space_memory) {
        return false;
    }

    return true;
}

void pcie_ats_reset(PCIDevice *dev);

void pcie_ats_iommu_region_add(MemoryListener *listener,
                               MemoryRegionSection *section);
void pcie_ats_iommu_region_del(MemoryListener *listener,
                               MemoryRegionSection *section);

int pcie_ats_page_request(PCIDevice *dev, hwaddr addr, QEMUBH *bh,
                          IOMMUAccessFlags flags);

#endif /* QEMU_PCIE_ATS_H */
