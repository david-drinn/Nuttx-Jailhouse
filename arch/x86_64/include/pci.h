#ifndef __ARCH_X86_64_INCLUDE_INTEL64_PCI_H
#define __ARCH_X86_64_INCLUDE_INTEL64_PCI_H

#define PCI_CFG_VENDOR_ID	0x000
#define PCI_CFG_DEVICE_ID	0x002
#define PCI_CFG_COMMAND		0x004
# define PCI_CMD_IO		(1 << 0)
# define PCI_CMD_MEM		(1 << 1)
# define PCI_CMD_MASTER		(1 << 2)
# define PCI_CMD_INTX_OFF	(1 << 10)
#define PCI_CFG_STATUS		0x006
# define PCI_STS_INT		(1 << 3)
# define PCI_STS_CAPS		(1 << 4)
#define PCI_CFG_BAR		0x010
# define PCI_BAR_64BIT		0x4
#define PCI_CFG_CAP_PTR		0x034

#define PCI_ID_ANY		0xffff

#define PCI_DEV_CLASS_OTHER	0xff

#define PCI_CAP_MSI		0x05
#define PCI_CAP_MSIX		0x11

#define MSIX_CTRL_ENABLE	0x8000
#define MSIX_CTRL_FMASK		0x4000

uint32_t pci_read_config(uint16_t bdf, unsigned int addr, unsigned int size);
void pci_write_config(uint16_t bdf, unsigned int addr, uint32_t value,
		      unsigned int size);
int pci_find_device(uint16_t vendor, uint16_t device, uint16_t start_bdf);
int pci_find_cap(uint16_t bdf, uint16_t cap);
void pci_msi_set_vector(uint16_t bdf, unsigned int vector);
void pci_msix_set_vector(uint16_t bdf, unsigned int vector, uint32_t index);

#endif /* __ARCH_X86_64_INCLUDE_INTEL64_PCI_H */
