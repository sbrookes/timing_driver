#ifndef DEF_GUARD_PLX_9080_H_
#define DEF_GUARD_PLX_9080_H_

/* 

   The PLX 9080 is a PCI bridge chip used by
   PCI card manufacterers to bridge their 
   hardware to the system's PCI bus. It also 
   manages DMA. Full documentation is available,
   but these are the appropriate register 
   offsets.

   

 */

/* PLX 9080 PCI CONFIGURATION REGISTERS */
#define PLX9080_PCIIDR			0x00
#define PLX9080_PCIICR			0x04
#define PLX9080_PCISR			0x06
#define PLX9080_PCIREV			0x08
#define PLX9080_PCICCR			0x09
#define PLX9080_PCICLSR			0x0c
#define PLX9080_PCILTR			0x0d
#define PLX9080_PCIHTR			0x0e
#define PLX9080_PCIBISTR		0x0f
#define PLX9080_PCIBAR0			0x10
#define PLX9080_PCIBAR1			0x14
#define PLX9080_PCIBAR2			0x18
#define PLX9080_PCIBAR3			0x1c
#define PLX9080_PCIBAR4			0x20
#define PLX9080_PCIBAR5			0x24
#define PLX9080_PCICIS			0x28
#define PLX9080_PCISVID			0x2c
#define PLX9080_PCISID			0x2e
#define PLX9080_PCIERBAR		0x30
#define PLX9080_CAP_PTR			0x34
#define PLX9080_PCIILR			0x3c
#define PLX9080_PCIIPR			0x3d
#define PLX9080_PCIMGR			0x3e
#define PLX9080_PCIMLR			0x3f
#define PLX9080_PMCAPID			0x40
#define PLX9080_PMNEXT			0x41
#define PLX9080_PMC			0x42
#define PLX9080_PMCSR			0x44
#define PLX9080_PMDATA			0x47
#define PLX9080_HS_CNTL			0x48
#define PLX9080_HS_NEXT			0x49
#define PLX9080_HS_CSR			0x4a
#define PLX9080_PVPDID			0x4c
#define PLX9080_PVPD_NEXT		0x4d
#define PLX9080_PVPDAD			0x4e
#define PLX9080_PVPDATA			0x50

/* PLX 9080 LOCAL CONFGURATION REGISTERS */
/*     offset from PCI base addr         */
#define PLX9080_LAS0RR			0x00
#define PLX9080_LAS0BA			0x04
#define PLX9080_MARBR			0x08
#define PLX9080_BIGEND			0x0c
#define PLX9080_LMISC1			0x0d
#define PLX9080_PROT_AREA		0x0e
#define PLX9080_LIMSC2			0x0f
#define PLX9080_EROMRR			0x10
#define PLX9080_EROMBA			0x14
#define PLX9080_LBRD0			0x18
#define PLX9080_DMRR			0x1c			
#define PLX9080_DMLBAM			0x20
#define PLX9080_DMLBAI			0x24
#define PLX9080_DMPBAM			0x28
#define PLX9080_DMCFGA			0x2c
#define PLX9080_LAS1RR			0xf0
#define PLX9080_LAS1BA			0xf4
#define PLX9080_LBRD1			0xf8
#define PLX9080_DMDAC			0xfc
#define PLX9080_PCIARB			0x100
#define PLX9080_PABTADR			0x104

/* PLX 9080 RUNTIME REGISTERS            */
/*     offset from PCI base addr         */
#define PLX9080_MBOX0			0x40
#define PLX9080_MBOX1			0x44
#define PLX9080_MBOX2			0x48
#define PLX9080_MBOX3			0x4c
#define PLX9080_MBOX4			0x50
#define PLX9080_MBOX5			0x54
#define PLX9080_MBOX6			0x58
#define PLX9080_MBOX7			0x5c
#define PLX9080_P2LDBELL		0x60
#define PLX9080_L2PDBELL		0x64
#define PLX9080_INTCSR			0x68
#define PLX9080_CNTRL			0x6c
#define PLX9080_PCIHIDR			0x70
#define PLX9080_PCIHREV			0x74

/* PLX 9080 DMA REGISTERS                */
/*     offset from PCI base addr         */
#define PLX9080_DMAMODE0		0x80
#define PLX9080_DMAPADR0		0x84
#define PLX9080_DMALADR0		0x88
#define PLX9080_DMASIZ0			0x8c
#define PLX9080_DMADPR0			0x90
#define PLX9080_DMAMODE1		0x94
#define PLX9080_DMAPADR1		0x98
#define PLX9080_DMALADR1		0x9c
#define PLX9080_DMASIZ1			0xa0
#define PLX9080_DMADPR1			0xa4
#define PLX9080_DMACSR0			0xa8
#define PLX9080_DMACSR1			0xa9
#define PLX9080_DMAARB			0xac
#define PLX9080_DMATHR			0xb0
#define PLX9080_DMADAC0			0xb4
#define PLX9080_DMADAC1			0xb8

/* PLX 9080 MESSAGING QUEUE REGISTERS    */
/*     offset from PCI base addr         */
#define PLX9080_OPQIS			0x30
#define PLX9080_OPQIM			0x34
#define PLX9080_IQP			0x40
#define PLX9080_OQP			0x44
#define PLX9080_MQCR			0xc0
#define PLX9080_QBAR			0xc4
#define PLX9080_IFHPR			0xc8
#define PLX9080_IFTPR			0xcc
#define PLX9080_IPHPR			0xd0
#define PLX9080_IPTPR			0xd4
#define PLX9080_OFHPR			0xd8
#define PLX9080_OFTPR			0xdc
#define PLX9080_OPHPR			0xe0
#define PLX9080_OPTPR			0xe4
#define PLX9080_QSR			0xe8

#endif

