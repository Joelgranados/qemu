/*
 * pcie_ats.c:
 *
 * Implementation of ATS emulation support.
 *
 * Copyright (c) 2024 Klaus Jensen <k.jensen@samsung.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

#include "qemu/osdep.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_ats.h"
#include "hw/pci/pci_bus.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"
#include "qemu/range.h"
#include "qapi/error.h"
#include "trace.h"

static PCIEATCEntry *pcie_ats_lookup(PCIDevice *dev, IOMMUTLBEntry *info)
{
    static const uint64_t masks[] = {0xfff, 0x1fffff, 0x3fffffff};
    PCIEATCEntry *atce;
    PCIEATC *atc = &dev->exp.atc;

    uint32_t devfn = PCI_BUILD_BDF(pci_bus_num(pci_get_bus(dev)), dev->devfn);

    QEMU_LOCK_GUARD(&dev->exp.atc.lock);

    for (unsigned int i = 0; i < ARRAY_SIZE(masks); i++) {
        uint64_t key = info->iova & ~masks[i];

        atce = g_hash_table_lookup(atc->h, &key);
        if (atce && atce->iotlbe.addr_mask == masks[i]) {
            if (!(atce->iotlbe.perm & info->perm)) {
                QTAILQ_REMOVE(&atc->lru, atce, lru);
                assert(g_hash_table_remove(atc->h, &atce->iotlbe.iova));

                goto miss;
            }

            trace_pcie_ats_lookup_hit(devfn, info->iova);

            QTAILQ_REMOVE(&atc->lru, atce, lru);
            QTAILQ_INSERT_HEAD(&atc->lru, atce, lru);

            return atce;
        }
    }

miss:
    trace_pcie_ats_lookup_miss(devfn, info->iova);

    return NULL;
}

/* must be called with atc lock held */
static void pcie_ats_evict_entry(PCIDevice *dev)
{
    PCIEATC *atc = &dev->exp.atc;
    PCIEATCEntry *victim = QTAILQ_LAST(&atc->lru);

    trace_pcie_ats_evict_entry(PCI_BUILD_BDF(pci_bus_num(pci_get_bus(dev)),
                                             dev->devfn), victim->iotlbe.iova);

    QTAILQ_REMOVE(&atc->lru, victim, lru);
    assert(g_hash_table_remove(atc->h, &victim->iotlbe.iova));
}

static void pcie_ats_update(PCIDevice *dev, const IOMMUTLBEntry *iotlbe)
{
    PCIEATCEntry *atce = g_new(PCIEATCEntry, 1);
    PCIEATC *atc = &dev->exp.atc;

    QEMU_LOCK_GUARD(&dev->exp.atc.lock);

    if (g_hash_table_size(atc->h) == atc->size) {
        pcie_ats_evict_entry(dev);
    }

    trace_pcie_ats_update(PCI_BUILD_BDF(pci_bus_num(pci_get_bus(dev)),
                                        dev->devfn), iotlbe->iova);

    memcpy(&atce->iotlbe, iotlbe, sizeof(atce->iotlbe));

    /* add to atc table */
    assert(g_hash_table_lookup(atc->h, &atce->iotlbe.iova) == NULL);
    g_hash_table_replace(atc->h, &atce->iotlbe.iova, atce);

    /* insert into lru */
    QTAILQ_INSERT_HEAD(&atc->lru, atce, lru);
}

IOMMUTLBEntry pcie_ats_translate(PCIDevice *dev, IOMMUTLBEntry in)
{
    PCIEATCEntry *atce = pcie_ats_lookup(dev, &in);

    if (!atce) {
        AddressSpace *as = pci_device_iommu_address_space(dev);
        MemoryRegion *mr = address_space_get_memory_region(as, in.iova, false);

        IOMMUMemoryRegion *iommu_mr = memory_region_get_iommu(mr);
        IOMMUMemoryRegionClass *imrc =
            memory_region_get_iommu_class_nocheck(iommu_mr);

        int iommu_idx =
            memory_region_iommu_attrs_to_index(iommu_mr,
                                               MEMTXATTRS_TRANSLATION);

        IOMMUTLBEntry iotlbe = imrc->translate(iommu_mr, in.iova, in.perm,
                                               iommu_idx);

        /* only cache translations with R and/or W permissions */
        if (iotlbe.perm) {
            pcie_ats_update(dev, &iotlbe);
        }

        return iotlbe;
    }

    return atce->iotlbe;
}

MemTxResult pcie_ats_dma_rw(PCIDevice *dev, dma_addr_t addr, void *buf,
                            dma_addr_t len, DMADirection dir, MemTxAttrs attrs)
{
    bool no_write = dir == DMA_DIRECTION_TO_DEVICE;

    IOMMUTLBEntry out, in = {
        .iova = addr,
        .perm = no_write ? IOMMU_RO : IOMMU_RW,
    };

    out = pcie_ats_translate(dev, in);

    if (!(out.perm & (1 << !no_write))) {
        return MEMTX_ACCESS_ERROR;
    }

    addr = out.translated_addr + (addr & out.addr_mask);

    return dma_memory_rw(out.target_as, addr, buf, len, dir, attrs);
}

void pcie_ats_reset(PCIDevice *dev)
{
    while (!QTAILQ_EMPTY(&dev->exp.atc.lru)) {
        PCIEATCEntry *atce = QTAILQ_FIRST(&dev->exp.atc.lru);
        QTAILQ_REMOVE(&dev->exp.atc.lru, atce, lru);
    }

    g_hash_table_remove_all(dev->exp.atc.h);
}

typedef struct PCIEATCPurgeInfo {
    PCIEATC *atc;
    IOMMUTLBEntry *iotlbe;
} PCIEATCPurgeInfo;

static gboolean pcie_ats_purge_by_mask(gpointer key, gpointer value,
                                       gpointer user_data)
{
    PCIEATCEntry *atce = (PCIEATCEntry *)value;
    PCIEATCPurgeInfo *info = (PCIEATCPurgeInfo *)user_data;
    IOMMUTLBEntry *iotlbe = info->iotlbe;

    if ((atce->iotlbe.iova & ~iotlbe->addr_mask) == iotlbe->iova) {
        QTAILQ_REMOVE(&info->atc->lru, atce, lru);
        return true;
    }

    return false;
}

static void pcie_ats_iommu_unmap_notify(IOMMUNotifier *n, IOMMUTLBEntry *entry)
{
    PCIEATSIOMMU *iommu = container_of(n, PCIEATSIOMMU, notifier);
    PCIDevice *dev = iommu->dev;
    PCIEATC *atc = &dev->exp.atc;
    PCIEATCPurgeInfo info = {
        .atc = atc,
        .iotlbe = entry,
    };

    uint32_t devfn = PCI_BUILD_BDF(pci_bus_num(pci_get_bus(dev)), dev->devfn);

    trace_pcie_ats_iommu_unmap_notify(devfn, entry->iova, entry->addr_mask);

    QEMU_LOCK_GUARD(&atc->lock);

    g_hash_table_foreach_remove(atc->h, pcie_ats_purge_by_mask, &info);
}

void pcie_ats_iommu_region_add(MemoryListener *listener,
                               MemoryRegionSection *section)
{
    PCIDevice *pci_dev = container_of(listener, PCIDevice,
                                      exp.atc.iommu_listener);
    IOMMUMemoryRegion *iommu_mr;
    Int128 end;
    int ret;

    if (!memory_region_is_iommu(section->mr)) {
        return;
    }

    iommu_mr = IOMMU_MEMORY_REGION(section->mr);
    end = int128_add(int128_make64(section->offset_within_region),
                     section->size);
    end = int128_sub(end, int128_one());

    PCIEATSIOMMU *iommu = g_malloc0(sizeof(*iommu));
    iommu->iommu_idx =
        memory_region_iommu_attrs_to_index(iommu_mr, MEMTXATTRS_UNSPECIFIED);
    iommu->mr = section->mr;
    iommu->dev = pci_dev;

    iommu_notifier_init(&iommu->notifier,
            pcie_ats_iommu_unmap_notify,
            IOMMU_NOTIFIER_DEVIOTLB_UNMAP,
            section->offset_within_region,
            int128_get64(end),
            iommu->iommu_idx);

    ret = memory_region_register_iommu_notifier(section->mr, &iommu->notifier,
                                                &error_fatal);
    if (ret) {
        abort();
    }

    QLIST_INSERT_HEAD(&pci_dev->exp.atc.iommu_list, iommu, iommu_next);
}

void pcie_ats_iommu_region_del(MemoryListener *listener,
                               MemoryRegionSection *section)
{
    PCIDevice *pci_dev = container_of(listener, PCIDevice,
                                      exp.atc.iommu_listener);

    if (!memory_region_is_iommu(section->mr)) {
        return;
    }

    PCIEATSIOMMU *iommu;
    QLIST_FOREACH(iommu, &pci_dev->exp.atc.iommu_list, iommu_next) {
        if (iommu->mr == section->mr &&
            iommu->notifier.start == section->offset_within_region) {
            memory_region_unregister_iommu_notifier(iommu->mr,
                                                    &iommu->notifier);
            QLIST_REMOVE(iommu, iommu_next);
            g_free(iommu);
            break;
        }
    }
}

int pcie_ats_page_request(PCIDevice *dev, hwaddr addr, QEMUBH *bh,
                          IOMMUAccessFlags flags)
{
    AddressSpace *as = pci_device_iommu_address_space(dev);
    MemoryRegion *mr = address_space_get_memory_region(as, addr, false);

    IOMMUMemoryRegion *iommu_mr = memory_region_get_iommu(mr);
    IOMMUMemoryRegionClass *imrc =
        memory_region_get_iommu_class_nocheck(iommu_mr);

    if (trace_event_get_state(TRACE_PCIE_ATS_PAGE_REQUEST)) {
        uint32_t devfn = PCI_BUILD_BDF(pci_bus_num(pci_get_bus(dev)),
                                       dev->devfn);

        trace_pcie_ats_page_request(devfn, addr, flags);
    }

    return imrc->page_request(iommu_mr, addr, bh, flags);
}
