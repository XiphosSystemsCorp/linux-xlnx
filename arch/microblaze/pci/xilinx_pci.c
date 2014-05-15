/*
 * PCI support for Xilinx plbv46_pci soft-core which can be used on
 * Xilinx Virtex ML410 / ML510 boards.
 *
 * Copyright 2009 Roderick Colenbrander
 * Copyright 2009 Secret Lab Technologies Ltd.
 *
 * The pci bridge fixup code was copied from ppc4xx_pci.c and was written
 * by Benjamin Herrenschmidt.
 * Copyright 2007 Ben. Herrenschmidt <benh@kernel.crashing.org>, IBM Corp.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/ioport.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/io.h>

#define XPLB_ISR 0x00
#define XPLB_IPR 0x04
#define XPLB_IER 0x08
#define XPLB_IID 0x18
#define XPLB_GIE 0x1c
#define XPLB_BIR 0x20
#define XPLB_BIE 0x28
#define XPLB_PCI_ADDR 0x10c
#define XPLB_PCI_DATA 0x110
#define XPLB_PCI_BUS  0x114

#define XPLB_DIER_BIRE 4
#define XPLB_DIER_DPTOE 2
#define XPLB_DIER_TERRE 1

#define PCI_HOST_ENABLE_CMD (PCI_COMMAND_SERR | PCI_COMMAND_PARITY | \
				PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY)

static struct of_device_id xilinx_pci_match[] = {
	{ .compatible = "xlnx,plbv46-pci-1.03.a", },
	{ .compatible = "xlnx,plbv46-pci-1.04.a", },
	{}
};

static const char *xilinx_pci_interrupt_names[] = {
  "PLB Master Read SERR",
  "PLB Master Read PERR",
  "PLB Master Read Target Abort",
  "PLB Master Write SERR",
  "PLB Master Write Target Abort",
  "PLB Master Write Master Abort",
  "PLB Master Write Retry",
  "PLB Master Write Retry Disconnect",
  "PLB Master Write Retry Timeout",
  "PLB Master Prefetch Timeout",
  "PCI Initiator Read SERR",
  "PCI Initiator Write SERR",
  "PCI Master Write Rearb. Timeout",
  "PLB Master Read Rearb Timeout",
  "PLB Write Slave BAR Overun",
  "PLB Read Slave BAR Overrun",
};

static irqreturn_t xilinx_pci_interrupt(int irq, void *dev_id)
{
  u32 id;
  u32 bir;
  u32 isr;
  struct pci_controller *hose = dev_id;
  int i;

  printk(KERN_DEBUG "xilinx_pci: interrupt %d\n", irq);

  isr = in_be32(hose->bridge_addr + XPLB_ISR);
  out_be32(hose->bridge_addr + XPLB_ISR, isr);
  id = in_be32(hose->bridge_addr + XPLB_IID);
  bir = in_be32(hose->bridge_addr + XPLB_BIR);
  out_be32(hose->bridge_addr + XPLB_BIR, bir);

  printk(KERN_DEBUG "xilinx_pci: ID=0x%02x ISR=0x%02x BIR=0x%04x",
      id, isr, bir);

  i = 0;
  while (bir) {
    if (bir & 1) {
      if (i < ARRAY_SIZE(xilinx_pci_interrupt_names))
        printk(" [%s]", xilinx_pci_interrupt_names[i]);
      else
        printk(" [#%d]", i);

    }
    bir >>= 1;
    i ++;
  }
  printk("\n");

  return IRQ_HANDLED;
}


/**
 * xilinx_pci_fixup_bridge - Block Xilinx PHB configuration.
 */
static void xilinx_pci_fixup_bridge(struct pci_dev *dev)
{
	struct pci_controller *hose;
	int i;
  int irq;
  int rc;
  struct resource r_irq;


	if (dev->devfn || dev->bus->self)
		return;

	hose = pci_bus_to_host(dev->bus);
	if (!hose)
		return;

	if (!of_match_node(xilinx_pci_match, hose->dn))
		return;

	/* Hide the PCI host BARs from the kernel as their content doesn't
	 * fit well in the resource management
	 */
	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		dev->resource[i].start = 0;
		dev->resource[i].end = 0;
		dev->resource[i].flags = 0;
	}

	dev_info(&dev->dev, "Hiding Xilinx plb-pci host bridge resources %s\n",
		 pci_name(dev));
       /* XXX: Xiphos Q6-Specific */
       /* Configure the PBAR0 so that PLB RAM is accessible from the PCI bus */
       pci_write_config_dword(dev, 0x10, 0x80000000);
       dev_info(&dev->dev, "%s: Configuring PBAR0 to 0x80000000\n",
           pci_name(dev));

       dev_info(&dev->dev, "Finding interrupt");

       /* Find interrupt */
       rc = of_irq_to_resource(hose->dn, 0, &r_irq);
       if (!rc) {
         dev_err(&dev->dev, "No IRQ found.\n");
         irq = -1;
       } else {
         irq = r_irq.start;
         dev_err(&dev->dev, "IRQ %d\n", irq);
       }

  /* Setup interrupt */
  rc = request_irq(irq, xilinx_pci_interrupt, 0, "xilinx-pci", hose);
  if (rc <0) {
    dev_warn(&dev->dev, "could not request IRQ\n");
  }
  dev_info(&dev->dev, "IRQ requested\n");

  /* Enable interrupts */
  out_be32(hose->bridge_addr + XPLB_IER, XPLB_DIER_BIRE | XPLB_DIER_DPTOE | XPLB_DIER_TERRE);
  out_be32(hose->bridge_addr + XPLB_BIE, 0x0000ffff);
  out_be32(hose->bridge_addr + XPLB_GIE, 0x80000000);
}
DECLARE_PCI_FIXUP_HEADER(PCI_ANY_ID, PCI_ANY_ID, xilinx_pci_fixup_bridge);

void xilinx_early_pci_dump(struct pci_controller *hose)
{
  unsigned int dev, bus = 0;
  int offset;
  u32 val_dw;
  u16 val_w0, val_w2;
  u8 val_b0, val_b1, val_b2, val_b3;

  printk(KERN_INFO "=========================================\n");
  for (dev = 0; dev <= 32; dev ++) {
    if (dev > 0)
      printk(KERN_INFO
        "-----------------------------------------\n");
    printk(KERN_INFO "Device %02x:%02x:\n", bus, dev);

    for (offset = 0; offset < 64; offset += 4) {

      early_read_config_dword(hose, bus,
        PCI_DEVFN(dev, 0), offset, &val_dw);

      early_read_config_word(hose, bus,
        PCI_DEVFN(dev, 0), offset, &val_w0);
      early_read_config_word(hose, bus,
        PCI_DEVFN(dev, 0), offset + 2, &val_w2);

      early_read_config_byte(hose, bus,
        PCI_DEVFN(dev, 0), offset, &val_b0);
      early_read_config_byte(hose, bus,
        PCI_DEVFN(dev, 0), offset + 1, &val_b1);
      early_read_config_byte(hose, bus,
        PCI_DEVFN(dev, 0), offset + 2, &val_b2);
      early_read_config_byte(hose, bus,
        PCI_DEVFN(dev, 0), offset + 3, &val_b3);

      printk(KERN_INFO
        "  %02x | %08x %04x:%04x %02x:%02x:%02x:%02x\n",
          offset,
          val_dw,
          val_w2, val_w0,
          val_b3, val_b2, val_b1, val_b0);
    }
  }
  printk(KERN_INFO "=========================================\n");
}

#ifdef DEBUG
/**
 * xilinx_pci_exclude_device - Don't do config access for non-root bus
 *
 * This is a hack.  Config access to any bus other than bus 0 does not
 * currently work on the ML510 so we prevent it here.
 */
static int
xilinx_pci_exclude_device(struct pci_controller *hose, u_char bus, u8 devfn)
{
	return (bus != 0);
}

/**
 * xilinx_early_pci_scan - List pci config space for available devices
 *
 * List pci devices in very early phase.
 */
static void __init xilinx_early_pci_scan(struct pci_controller *hose)
{
	u32 bus = 0;
	u32 val, dev, func, offset;

	/* Currently we have only 2 device connected - up-to 32 devices */
	for (dev = 0; dev < 2; dev++) {
		/* List only first function number - up-to 8 functions */
		for (func = 0; func < 1; func++) {
			pr_info("%02x:%02x:%02x", bus, dev, func);
			/* read the first 64 standardized bytes */
			/* Up-to 192 bytes can be list of capabilities */
			for (offset = 0; offset < 64; offset += 4) {
				early_read_config_dword(hose, bus,
					PCI_DEVFN(dev, func), offset, &val);
				if (offset == 0 && val == 0xFFFFFFFF) {
					pr_cont("\nABSENT");
					break;
				}
				if (!(offset % 0x10))
					pr_cont("\n%04x:    ", offset);

				pr_cont("%08x  ", val);
			}
			pr_info("\n");
		}
	}
}
#else
static void __init xilinx_early_pci_scan(struct pci_controller *hose)
{
}
#endif

/**
 * xilinx_pci_init - Find and register a Xilinx PCI host bridge
 */
void __init xilinx_pci_init(void)
{
	struct pci_controller *hose;
	struct resource r;
	void __iomem *pci_reg;
	struct device_node *idsel_gpio_node;
	struct device_node *pci_node;

	pci_node = of_find_matching_node(NULL, xilinx_pci_match);
	if (!pci_node)
		return;

	if (of_address_to_resource(pci_node, 0, &r)) {
		pr_err("xilinx-pci: cannot resolve base address\n");
		return;
	}

	hose = pcibios_alloc_controller(pci_node);
	if (!hose) {
		pr_err("xilinx-pci: pcibios_alloc_controller() failed\n");
		return;
	}

	/* Check if we need to do the GPIO IDSEL hack */
	idsel_gpio_node = of_parse_phandle(pci_node, "idsel-gpio", 0);
	if (idsel_gpio_node) {
		struct resource r;

		if (of_address_to_resource(idsel_gpio_node, 0, &r)) {
			pr_warn("xilinx-pci: "
					"cannot get memory resource for idsel-gpio\n");
		} else {
			hose->idsel_gpio =
				ioremap(r.start, r.end - r.start + 1);
			if (hose->idsel_gpio)
				pr_warn("xilinx-pci: IDSEL GPIO at 0x%08x\n",
						r.start);
		}
	} else {
		pr_warn("xilinx-pci: no idsel-gpio\n");
	}

	/* Setup config space */
	setup_indirect_pci(hose, r.start + XPLB_PCI_ADDR,
			r.start + XPLB_PCI_DATA,
			INDIRECT_TYPE_SET_CFG_TYPE);
	hose->bridge_addr = ioremap(r.start, PAGE_SIZE);

	/* According to the xilinx plbv46_pci documentation the soft-core starts
	 * a self-init when the bus master enable bit is set. Without this bit
	 * set the pci bus can't be scanned.
	 */
	early_write_config_word(hose, 0, 0, PCI_COMMAND, PCI_HOST_ENABLE_CMD);

	/* Set the max latency timer to 255 */
	early_write_config_byte(hose, 0, 0, PCI_LATENCY_TIMER, 0xff);

	/* Set the max bus number to 255, and bus/subbus no's to 0 */
	pci_reg = of_iomap(pci_node, 0);
	out_be32(pci_reg + XPLB_PCI_BUS, 0x000000ff);
	iounmap(pci_reg);

	/* Register the host bridge with the linux kernel! */
	pci_process_bridge_OF_ranges(hose, pci_node,
			INDIRECT_TYPE_SET_CFG_TYPE);

	pr_info("xilinx-pci: Registered PCI host bridge\n");
	xilinx_early_pci_scan(hose);
}
