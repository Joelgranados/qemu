#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/pci/pci_device.h"
#include "hw/pci/msix.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"
#include "hw/acpi/acpi_aml_interface.h"
#include "hw/acpi/aml-build.h"

#define TYPE_PCIE_ATS_DEVICE "pcie-ats-testdev"
OBJECT_DECLARE_SIMPLE_TYPE(PCIeATSState, PCIE_ATS_DEVICE)

struct PCIeATSState {
    PCIDevice pdev;

    MemoryRegion mmio;

    struct dma_state {
        uint64_t addr;

#define PCIE_ATS_DMA_RUN             0x1
#define PCIE_ATS_DMA_DIR(cmd)        (((cmd) >> 1) & 0x1)
# define PCIE_ATS_DMA_FROM_PCI       0
# define PCIE_ATS_DMA_TO_PCI         1
        uint32_t cmd;

#define BUF_SIZE 4096
        uint8_t buf[BUF_SIZE];
    } dma;

    QEMUBH *dma_bh;

    /* ACPI Power Management */
    uint32_t power_state;
#define PCIE_ATS_POWER_D0            0  /* Fully On */
#define PCIE_ATS_POWER_D1            1  /* Low Power */  
#define PCIE_ATS_POWER_D2            2  /* Lower Power */
#define PCIE_ATS_POWER_D3            3  /* Off */
};

static void pcie_ats_dma_bh(void *opaque)
{
    PCIeATSState *pcie_ats = opaque;
    MemTxResult res;
    DMADirection dir =
        PCIE_ATS_DMA_DIR(pcie_ats->dma.cmd) == PCIE_ATS_DMA_FROM_PCI ?
            DMA_DIRECTION_FROM_DEVICE : DMA_DIRECTION_TO_DEVICE;

    if (!(pcie_ats->dma.cmd & PCIE_ATS_DMA_RUN)) {
        return;
    }

    /* Check power state - only allow DMA in D0 state */
    if (pcie_ats->power_state != PCIE_ATS_POWER_D0) {
        pcie_ats->dma.cmd &= ~PCIE_ATS_DMA_RUN;
        return;
    }

    res = pci_dma_rw(&pcie_ats->pdev, pcie_ats->dma.addr, pcie_ats->dma.buf,
                     BUF_SIZE, dir, MEMTXATTRS_UNSPECIFIED);

    if (res == MEMTX_ACCESS_ERROR) {
        IOMMUAccessFlags flags =
            PCIE_ATS_DMA_DIR(pcie_ats->dma.cmd) == PCIE_ATS_DMA_FROM_PCI ?
                IOMMU_WO : IOMMU_RO;

        int ret = pcie_ats_page_request(&pcie_ats->pdev, pcie_ats->dma.addr,
                                        pcie_ats->dma_bh, flags);
        if (ret == 0) {
            return;
        }
    }

    pcie_ats->dma.cmd &= ~PCIE_ATS_DMA_RUN;
}

static uint64_t pcie_ats_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    PCIeATSState *pcie_ats = opaque;
    uint64_t val = ~0ULL;

    /* Return all 1s if device is in D3 state */
    if (pcie_ats->power_state == PCIE_ATS_POWER_D3) {
        return val;
    }

    switch (addr) {
    case 0x0:
        val = ldn_le_p(&pcie_ats->dma.addr, size);
        break;
    case 0x4:
        val = ldl_le_p((uint8_t *)&pcie_ats->dma.addr + 4);
        break;
    case 0x8:
        val = ldl_le_p(&pcie_ats->dma.cmd);
        break;
    case 0xc:
        /* Power management status register */
        val = pcie_ats->power_state;
        break;
    }

    return val;
}

static void pcie_ats_set_power_state(PCIeATSState *pcie_ats, uint32_t state)
{
    PCIDevice *pdev = &pcie_ats->pdev;

    if (pcie_ats->power_state == state) {
        return;
    }

    pcie_ats->power_state = state;

    switch (state) {
    case PCIE_ATS_POWER_D0:
        /* Device fully powered - enable all functionality */
        pci_set_power(pdev, true);
        memory_region_set_enabled(&pcie_ats->mmio, true);
        break;
    case PCIE_ATS_POWER_D1:
    case PCIE_ATS_POWER_D2:
        /* Lower power states - reduce functionality but keep device accessible */
        pci_set_power(pdev, true);
        memory_region_set_enabled(&pcie_ats->mmio, true);
        break;
    case PCIE_ATS_POWER_D3:
        /* Device powered off - disable functionality */
        if (pcie_ats->dma.cmd & PCIE_ATS_DMA_RUN) {
            pcie_ats->dma.cmd &= ~PCIE_ATS_DMA_RUN;
        }
        memory_region_set_enabled(&pcie_ats->mmio, false);
        pci_set_power(pdev, false);
        break;
    }
}

static void pcie_ats_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                                unsigned size)
{
    PCIeATSState *pcie_ats = opaque;

    /* Ignore writes if device is in D3 state */
    if (pcie_ats->power_state == PCIE_ATS_POWER_D3) {
        return;
    }

    switch (addr) {
    case 0x0:
        stn_le_p(&pcie_ats->dma.addr, size, val);
        break;
    case 0x4:
        stl_le_p((uint8_t *)&pcie_ats->dma.addr + 4, val);
        break;
    case 0x8:
        stl_le_p(&pcie_ats->dma.cmd, val);

        if (val & PCIE_ATS_DMA_RUN) {
            qemu_bh_schedule(pcie_ats->dma_bh);
        }

        break;
    case 0xc:
        /* Power management control register */
        if (val <= PCIE_ATS_POWER_D3) {
            pcie_ats_set_power_state(pcie_ats, val);
        }
        break;
    }
}

static const MemoryRegionOps pcie_ats_mmio_ops = {
    .read = pcie_ats_mmio_read,
    .write = pcie_ats_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void build_pcie_ats_aml(AcpiDevAmlIf *adev, Aml *scope)
{
    Aml *method;

    /* _PS0 - Power State 0 (D0 - Fully On) */
    method = aml_method("_PS0", 0, AML_NOTSERIALIZED);
    aml_append(method, aml_debug());
    aml_append(scope, method);

    /* _PS3 - Power State 3 (D3 - Off) */
    method = aml_method("_PS3", 0, AML_NOTSERIALIZED);
    aml_append(method, aml_debug());
    aml_append(scope, method);

    /* _S1D - Highest D-state for S1 system state */
    method = aml_method("_S1D", 0, AML_NOTSERIALIZED);
    aml_append(method, aml_return(aml_int(PCIE_ATS_POWER_D1)));
    aml_append(scope, method);

    /* _S2D - Highest D-state for S2 system state */
    method = aml_method("_S2D", 0, AML_NOTSERIALIZED);
    aml_append(method, aml_return(aml_int(PCIE_ATS_POWER_D2)));
    aml_append(scope, method);

    /* _S3D - Highest D-state for S3 system state (suspend-to-RAM) */
    method = aml_method("_S3D", 0, AML_NOTSERIALIZED);
    aml_append(method, aml_return(aml_int(PCIE_ATS_POWER_D3)));
    aml_append(scope, method);

    /* _S4D - Highest D-state for S4 system state (suspend-to-disk) */
    method = aml_method("_S4D", 0, AML_NOTSERIALIZED);
    aml_append(method, aml_return(aml_int(PCIE_ATS_POWER_D3)));
    aml_append(scope, method);
}

static void pcie_ats_realize(PCIDevice *pdev, Error **errp)
{
    PCIeATSState *pcie_ats = PCIE_ATS_DEVICE(pdev);
    uint8_t *pci_conf = pdev->config;
    uint16_t cap_offset = PCI_CONFIG_SPACE_SIZE;
    int pm_cap_offset;

    pci_config_set_interrupt_pin(pci_conf, 1);

    pcie_endpoint_cap_init(pdev, 0x80);

    if (pdev->cap_present & QEMU_PCIE_CAP_ATS) {
        pcie_ats_init(pdev, cap_offset, 1);
        cap_offset += PCI_EXT_CAP_ATS_SIZEOF;

        if (pdev->cap_present & QEMU_PCIE_CAP_PRI) {
            pcie_pri_init(pdev, cap_offset, 2048);
            cap_offset += PCI_EXT_CAP_PRI_SIZEOF;
        }
    }

    memory_region_init_io(&pcie_ats->mmio, OBJECT(pcie_ats),
                          &pcie_ats_mmio_ops, pcie_ats, "pcie-ats-mmio",
                          0x1000);

    pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64,
                     &pcie_ats->mmio);

    pcie_ats->dma_bh = qemu_bh_new(pcie_ats_dma_bh, pcie_ats);

    /* Add PCI Power Management capability */
    pm_cap_offset = pci_add_capability(pdev, PCI_CAP_ID_PM, 0, PCI_PM_SIZEOF, errp);
    if (pm_cap_offset < 0) {
        return;
    }
    pci_set_word(pdev->config + pm_cap_offset + PCI_PM_PMC,
                 PCI_PM_CAP_D1 | PCI_PM_CAP_D2 | PCI_PM_CAP_DSI);

    /* Initialize power management - device starts in D0 state */
    pcie_ats->power_state = PCIE_ATS_POWER_D0;
    pci_set_power(pdev, true);
}

static void pcie_ats_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *pdc = PCI_DEVICE_CLASS(class);
    AcpiDevAmlIfClass *adevc = ACPI_DEV_AML_IF_CLASS(class);

    pdc->realize = pcie_ats_realize;
    pdc->vendor_id = PCI_VENDOR_ID_QEMU;
    pdc->device_id = 0x11e9;
    pdc->revision = 2;
    pdc->class_id = PCI_CLASS_OTHERS;

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    dc->desc = "PCI Express ATS/PRI Test Device";

    adevc->build_dev_aml = build_pcie_ats_aml;
}

static void pcie_ats_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_PCIE_DEVICE },
        { TYPE_ACPI_DEV_AML_IF },
        { },
    };

    static const TypeInfo pcie_ats_info = {
        .name          = TYPE_PCIE_ATS_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(PCIeATSState),
        .class_init    = pcie_ats_class_init,
        .interfaces    = interfaces,
    };

    type_register_static(&pcie_ats_info);
}

type_init(pcie_ats_register_types)
