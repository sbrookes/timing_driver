#ifndef _TIMING_KERNEL_DEFS_H
#define _TIMING_KERNEL_DEFS_H

/* 

   Header file whose purpose will be to contain
   definitions pertinent to the driver written
   for ADLINK Timing Card PCIe-7300A

   AUTHOR: Scott Brookes
   Date: Created 2.1.13

 */

#include <linux/cdev.h>
#include <linux/pci.h>

/*
  Vendor and device ID used by the PCI protocol
  to register the card with the system. These
  are defined in the device's documentation.

  in this case, it is the ID of the PCI bus master chip,
  because it appears that AdLink did not use their
  own vendor or Device IDs. For this reason, the driver
  will have to check the sub_vendor and sub_device IDs
  to confirm the card
*/
#define TIMING_VENDOR_ID  0x10b5              
#define TIMING_DEVICE_ID  0x9080

/* card specific IDs */ 
#define ADLINK_VENDOR_ID  0x144a
#define ADLINK_7300A_ID   0x7300

/* offsets in configuration register of */
/*         device as per PCI standard   */
#define SUBVENDOR_ID_OFF  0x2c
#define SUBDEVICE_ID_OFF  0x2e
#define IRQ_LINE_OFF      0x3c

/* for the Registration of the driver */
#define FIRST_MINOR      0

/* 9 char drivers... one for each of the */
/* 8 IO Ports on the card, and one for   */
/* access to the Bus Master   LCR (local */
/* configuration registers)              */
#define TIMING_DEV_COUNT    13
#define TIMING_IOPORT_COUNT  8
#define TIMING_8254_COUNT    4

/* 32 bit ports as per manual */
#define TIMING_IOPORT_SIZE 0x4

/* MAX FIFO len for the card * 32 */
#define MAX_DEPTH 65536

/*
  The Card's manual states that the base address is 
      stored at offset 0x18 of the PCI Configuration 
      register. According to the PCI Standard, that
      offset holds the address of BAR 2

  Control of the LCR (local configuration registers)
      is detailed in the documentation for the PLX
      9080. The timing card manual states
      that the base address of these registers is 
      stored at offset 0x14 of the PCI Configuration
      register. Accordign to the PCI Standard, that
      offset holds the address of BAR 1
 */
#define TIMING_BAR  2
#define PLX9080_BAR 1

/* regiond on card identifiers */
#define PLX9080_ID    9
#define TIMER8254_ID  8
#define PCI7300_ID    7

/* ioctl commands  -- note: not picked carfully */
#define CHANGE_PLX_OFFSET 0x34d0 /* arbitrary identifier */

/*
  Structure internal to the driver to manage data
       for a given device
 */
typedef struct _sdarn_timing_driver_data {

  struct pci_driver *driver;      /* kernel PCI  driver struct */
  struct cdev cdev;               /* kernel char driver struct */
  void __iomem *base;             /* base address */
  unsigned long len;              /* resource length */
  struct file_operations *fops;   /* char driver file ops */
  dev_t num;                      /* device number */
  int component;                  /* component     */
  unsigned int offset;            /* offset from base (for PLX9080) */

} timing_dev_data;

/* module init and exit functions */
static int  __init timing_dev_init(void);
static void __exit timing_dev_exit(void);

/* for installation/removal of PCI device */
static int timing_dev_probe(struct pci_dev *dev, 
			    const struct pci_device_id *id);
static void timing_dev_remove(struct pci_dev *dev);

/* char device operations */
static int timing_dev_open(struct inode *inode, 
			   struct file  *filp );
static int timing_dev_release(struct inode *inode,
			      struct file  *filp );

void configure_for_dma(void);

int dma_init_kthread(void *data);
static ssize_t dma_transfer(struct file *filp, const char __user *buf,
			    size_t count, loff_t *f_pos);

static ssize_t timing_read(struct file *filp, char __user *buf,
			   size_t count, loff_t *f_pos);
static ssize_t timing_write(struct file *filp, const char __user *buf,
			    size_t count, loff_t *f_pos);

static ssize_t chip_8254_write(struct file *filp, const char __user *buf,
			       size_t count, loff_t *f_pos);

long timing_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#endif
