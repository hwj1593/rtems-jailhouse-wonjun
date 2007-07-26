/*
 *
 *  PCI defines and function prototypes
 *  Copyright 1994, Drew Eckhardt
 *  Copyright 1997, 1998 Martin Mares <mj@atrey.karlin.mff.cuni.cz>
 *
 *  For more information, please consult the following manuals (look at
 *  http://www.pcisig.com/ for how to get them):
 *
 *  PCI BIOS Specification
 *  PCI Local Bus Specification
 *  PCI to PCI Bridge Specification
 *  PCI System Design Guide
 *
 * $Id$
 */

#ifndef _RTEMS_PCI_H
#define _RTEMS_PCI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Under PCI, each device has 256 bytes of configuration address space,
 * of which the first 64 bytes are standardized as follows:
 */
#define PCI_VENDOR_ID      0x00  /* 16 bits */
#define PCI_DEVICE_ID      0x02  /* 16 bits */

#define PCI_COMMAND             0x04    /* 16 bits */
#define PCI_COMMAND_IO          0x0001  /* Enable response in I/O space */
#define PCI_COMMAND_MEMORY      0x0002  /* Enable response in Memory space */
#define PCI_COMMAND_MASTER      0x0004  /* Enable bus mastering */
#define PCI_COMMAND_SPECIAL     0x0008  /* Enable response to special cycles */
#define PCI_COMMAND_INVALIDATE  0x0010  /* Use memory write and invalidate */
#define PCI_COMMAND_VGA_PALETTE 0x0020  /* Enable palette snooping */
#define PCI_COMMAND_PARITY      0x0040  /* Enable parity checking */
#define PCI_COMMAND_WAIT        0x0080  /* Enable address/data stepping */
#define PCI_COMMAND_SERR        0x0100  /* Enable SERR */
#define PCI_COMMAND_FAST_BACK   0x0200  /* Enable back-to-back writes */

#define PCI_STATUS                  0x06   /* 16 bits */
#define PCI_STATUS_66MHZ            0x0020 /* Support 66 Mhz PCI 2.1 bus */
#define PCI_STATUS_UDF              0x0040 /* Support User Definable Features */
#define PCI_STATUS_FAST_BACK        0x0080 /* Accept fast-back to back */
#define PCI_STATUS_PARITY           0x0100 /* Detected parity error */
#define PCI_STATUS_DEVSEL_MASK      0x0600 /* DEVSEL timing */
#define PCI_STATUS_DEVSEL_FAST      0x0000
#define PCI_STATUS_DEVSEL_MEDIUM    0x0200
#define PCI_STATUS_DEVSEL_SLOW      0x0400
#define PCI_STATUS_SIG_TARGET_ABORT 0x0800 /* Set on target abort */
#define PCI_STATUS_REC_TARGET_ABORT 0x1000 /* Master ack of " */
#define PCI_STATUS_REC_MASTER_ABORT 0x2000 /* Set on master abort */
#define PCI_STATUS_SIG_SYSTEM_ERROR 0x4000 /* Set when we drive SERR */
#define PCI_STATUS_DETECTED_PARITY  0x8000 /* Set on parity error */

#define PCI_CLASS_REVISION  0x08  /* High 24 bits are class, low 8 revision */
#define PCI_REVISION_ID     0x08  /* Revision ID */
#define PCI_CLASS_PROG      0x09  /* Reg. Level Programming Interface */
#define PCI_CLASS_DEVICE    0x0a  /* Device class */
#define PCI_CACHE_LINE_SIZE  0x0c /* 8 bits */
#define PCI_LATENCY_TIMER    0x0d /* 8 bits */

#define PCI_HEADER_TYPE         0x0e /* 8 bits */
#define PCI_HEADER_TYPE_NORMAL  0
#define PCI_HEADER_TYPE_BRIDGE  1
#define PCI_HEADER_TYPE_CARDBUS 2

#define PCI_BIST            0x0f  /* 8 bits */
#define PCI_BIST_CODE_MASK  0x0f  /* Return result */
#define PCI_BIST_START      0x40  /* 1 to start BIST, 2 secs or less */
#define PCI_BIST_CAPABLE    0x80  /* 1 if BIST capable */

/*
 * Base addresses specify locations in memory or I/O space.
 * Decoded size can be determined by writing a value of
 * 0xffffffff to the register, and reading it back.  Only
 * 1 bits are decoded.
 */
#define PCI_BASE_ADDRESS_0  0x10  /* 32 bits */
#define PCI_BASE_ADDRESS_1  0x14  /* 32 bits [htype 0,1 only] */
#define PCI_BASE_ADDRESS_2  0x18  /* 32 bits [htype 0 only] */
#define PCI_BASE_ADDRESS_3  0x1c  /* 32 bits */
#define PCI_BASE_ADDRESS_4  0x20  /* 32 bits */
#define PCI_BASE_ADDRESS_5  0x24  /* 32 bits */
#define PCI_BASE_ADDRESS_SPACE         0x01  /* 0 = memory, 1 = I/O */
#define PCI_BASE_ADDRESS_SPACE_IO      0x01
#define PCI_BASE_ADDRESS_SPACE_MEMORY  0x00
#define PCI_BASE_ADDRESS_MEM_TYPE_MASK 0x06
#define PCI_BASE_ADDRESS_MEM_TYPE_32   0x00  /* 32 bit address */
#define PCI_BASE_ADDRESS_MEM_TYPE_1M   0x02  /* Below 1M */
#define PCI_BASE_ADDRESS_MEM_TYPE_64   0x04  /* 64 bit address */
#define PCI_BASE_ADDRESS_MEM_PREFETCH  0x08  /* prefetchable? */
#define PCI_BASE_ADDRESS_MEM_MASK      (~0x0fUL)
#define PCI_BASE_ADDRESS_IO_MASK      (~0x03UL)
/* bit 1 is reserved if address_space = 1 */

/* Header type 0 (normal devices) */
#define PCI_CARDBUS_CIS         0x28
#define PCI_SUBSYSTEM_VENDOR_ID 0x2c
#define PCI_SUBSYSTEM_ID        0x2e
#define PCI_ROM_ADDRESS         0x30/* Bits 31..11 address, 10..1 reserved */
#define PCI_ROM_ADDRESS_ENABLE  0x01
#define PCI_ROM_ADDRESS_MASK    (~0x7ffUL)

/* 0x34-0x3b are reserved */
#define PCI_INTERRUPT_LINE  0x3c  /* 8 bits */
#define PCI_INTERRUPT_PIN   0x3d  /* 8 bits */
#define PCI_MIN_GNT         0x3e  /* 8 bits */
#define PCI_MAX_LAT         0x3f  /* 8 bits */

/* Header type 1 (PCI-to-PCI bridges) */
#define PCI_PRIMARY_BUS        0x18  /* Primary bus number */
#define PCI_SECONDARY_BUS      0x19  /* Secondary bus number */
#define PCI_SUBORDINATE_BUS    0x1a  /* Highest bus number behind the bridge */
#define PCI_SEC_LATENCY_TIMER  0x1b  /* Latency timer for secondary interface */
#define PCI_IO_BASE            0x1c  /* I/O range behind the bridge */
#define PCI_IO_LIMIT           0x1d
#define PCI_IO_RANGE_TYPE_MASK 0x0f  /* I/O bridging type */
#define PCI_IO_RANGE_TYPE_16   0x00
#define PCI_IO_RANGE_TYPE_32   0x01
#define PCI_IO_RANGE_MASK      ~0x0f

#define PCI_SEC_STATUS 0x1e  /* Secondary status register, only bit 14 used */

#define PCI_MEMORY_BASE            0x20  /* Memory range behind */
#define PCI_MEMORY_LIMIT           0x22
#define PCI_MEMORY_RANGE_TYPE_MASK 0x0f
#define PCI_MEMORY_RANGE_MASK      ~0x0f
#define PCI_PREF_MEMORY_BASE       0x24  /* Prefetchable memory range behind */
#define PCI_PREF_MEMORY_LIMIT      0x26
#define PCI_PREF_RANGE_TYPE_MASK   0x0f
#define PCI_PREF_RANGE_TYPE_32     0x00
#define PCI_PREF_RANGE_TYPE_64     0x01
#define PCI_PREF_RANGE_MASK        ~0x0f
#define PCI_PREF_BASE_UPPER32      0x28 /* Upper half of prefetchable memory */
#define PCI_PREF_LIMIT_UPPER32     0x2c
#define PCI_IO_BASE_UPPER16        0x30  /* Upper half of I/O addresses */
#define PCI_IO_LIMIT_UPPER16       0x32
/* 0x34-0x3b is reserved */
#define PCI_ROM_ADDRESS1  0x38  /* Same as PCI_ROM_ADDRESS, but for htype 1 */
/* 0x3c-0x3d are same as for htype 0 */
#define PCI_BRIDGE_CONTROL     0x3e
#define PCI_BRIDGE_CTL_PARITY  0x01  /* Enable parity detection on secondary interface */
#define PCI_BRIDGE_CTL_SERR         0x02  /* The same for SERR forwarding */
#define PCI_BRIDGE_CTL_NO_ISA       0x04  /* Disable bridging of ISA ports */
#define PCI_BRIDGE_CTL_VGA          0x08  /* Forward VGA addresses */
#define PCI_BRIDGE_CTL_MASTER_ABORT 0x20  /* Report master aborts */
#define PCI_BRIDGE_CTL_BUS_RESET    0x40  /* Secondary bus reset */
#define PCI_BRIDGE_CTL_FAST_BACK    0x80  /* Fast Back2Back enabled */
                                          /*     on secondary interface */

/* Header type 2 (CardBus bridges) */
/* 0x14-0x15 reserved */
#define PCI_CB_SEC_STATUS             0x16  /* Secondary status */
#define PCI_CB_PRIMARY_BUS            0x18  /* PCI bus number */
#define PCI_CB_CARD_BUS               0x19  /* CardBus bus number */
#define PCI_CB_SUBORDINATE_BUS        0x1a  /* Subordinate bus number */
#define PCI_CB_LATENCY_TIMER          0x1b  /* CardBus latency timer */
#define PCI_CB_MEMORY_BASE_0          0x1c
#define PCI_CB_MEMORY_LIMIT_0         0x20
#define PCI_CB_MEMORY_BASE_1          0x24
#define PCI_CB_MEMORY_LIMIT_1         0x28
#define PCI_CB_IO_BASE_0              0x2c
#define PCI_CB_IO_BASE_0_HI           0x2e
#define PCI_CB_IO_LIMIT_0             0x30
#define PCI_CB_IO_LIMIT_0_HI          0x32
#define PCI_CB_IO_BASE_1              0x34
#define PCI_CB_IO_BASE_1_HI           0x36
#define PCI_CB_IO_LIMIT_1             0x38
#define PCI_CB_IO_LIMIT_1_HI          0x3a
#define PCI_CB_IO_RANGE_MASK          ~0x03
/* 0x3c-0x3d are same as for htype 0 */

#define PCI_CB_BRIDGE_CONTROL           0x3e
#define PCI_CB_BRIDGE_CTL_PARITY        0x01  /* Similar to standard bridge */
                                              /*    control register */
#define PCI_CB_BRIDGE_CTL_SERR          0x02
#define PCI_CB_BRIDGE_CTL_ISA           0x04
#define PCI_CB_BRIDGE_CTL_VGA           0x08
#define PCI_CB_BRIDGE_CTL_MASTER_ABORT  0x20
#define PCI_CB_BRIDGE_CTL_CB_RESET      0x40  /* CardBus reset */
#define PCI_CB_BRIDGE_CTL_16BIT_INT     0x80  /* Enable interrupt for */
                                              /*    16-bit cards */
#define PCI_CB_BRIDGE_CTL_PREFETCH_MEM0 0x100 /* Prefetch enable for */
                                              /*    both memory regions */
#define PCI_CB_BRIDGE_CTL_PREFETCH_MEM1 0x200
#define PCI_CB_BRIDGE_CTL_POST_WRITES   0x400

#define PCI_CB_SUBSYSTEM_VENDOR_ID      0x40
#define PCI_CB_SUBSYSTEM_ID             0x42

/* 16-bit PC Card legacy mode base address (ExCa) */
#define PCI_CB_LEGACY_MODE_BASE  0x44

/* 0x48-0x7f reserved */

/* Device classes and subclasses */

#define PCI_CLASS_NOT_DEFINED      0x0000
#define PCI_CLASS_NOT_DEFINED_VGA  0x0001

#define PCI_BASE_CLASS_STORAGE    0x01
#define PCI_CLASS_STORAGE_SCSI    0x0100
#define PCI_CLASS_STORAGE_IDE     0x0101
#define PCI_CLASS_STORAGE_FLOPPY  0x0102
#define PCI_CLASS_STORAGE_IPI     0x0103
#define PCI_CLASS_STORAGE_RAID    0x0104
#define PCI_CLASS_STORAGE_OTHER   0x0180

#define PCI_BASE_CLASS_NETWORK        0x02
#define PCI_CLASS_NETWORK_ETHERNET    0x0200
#define PCI_CLASS_NETWORK_TOKEN_RING  0x0201
#define PCI_CLASS_NETWORK_FDDI        0x0202
#define PCI_CLASS_NETWORK_ATM         0x0203
#define PCI_CLASS_NETWORK_OTHER       0x0280

#define PCI_BASE_CLASS_DISPLAY    0x03
#define PCI_CLASS_DISPLAY_VGA     0x0300
#define PCI_CLASS_DISPLAY_XGA     0x0301
#define PCI_CLASS_DISPLAY_OTHER   0x0380

#define PCI_BASE_CLASS_MULTIMEDIA   0x04
#define PCI_CLASS_MULTIMEDIA_VIDEO  0x0400
#define PCI_CLASS_MULTIMEDIA_AUDIO  0x0401
#define PCI_CLASS_MULTIMEDIA_OTHER  0x0480

#define PCI_BASE_CLASS_MEMORY  0x05
#define PCI_CLASS_MEMORY_RAM   0x0500
#define PCI_CLASS_MEMORY_FLASH 0x0501
#define PCI_CLASS_MEMORY_OTHER 0x0580

#define PCI_BASE_CLASS_BRIDGE    0x06
#define PCI_CLASS_BRIDGE_HOST    0x0600
#define PCI_CLASS_BRIDGE_ISA     0x0601
#define PCI_CLASS_BRIDGE_EISA    0x0602
#define PCI_CLASS_BRIDGE_MC      0x0603
#define PCI_CLASS_BRIDGE_PCI     0x0604
#define PCI_CLASS_BRIDGE_PCMCIA  0x0605
#define PCI_CLASS_BRIDGE_NUBUS   0x0606
#define PCI_CLASS_BRIDGE_CARDBUS 0x0607
#define PCI_CLASS_BRIDGE_OTHER   0x0680

#define PCI_BASE_CLASS_COMMUNICATION     0x07
#define PCI_CLASS_COMMUNICATION_SERIAL   0x0700
#define PCI_CLASS_COMMUNICATION_PARALLEL 0x0701
#define PCI_CLASS_COMMUNICATION_OTHER    0x0780

#define PCI_BASE_CLASS_SYSTEM  0x08
#define PCI_CLASS_SYSTEM_PIC   0x0800
#define PCI_CLASS_SYSTEM_DMA   0x0801
#define PCI_CLASS_SYSTEM_TIMER 0x0802
#define PCI_CLASS_SYSTEM_RTC   0x0803
#define PCI_CLASS_SYSTEM_OTHER 0x0880

#define PCI_BASE_CLASS_INPUT     0x09
#define PCI_CLASS_INPUT_KEYBOARD 0x0900
#define PCI_CLASS_INPUT_PEN      0x0901
#define PCI_CLASS_INPUT_MOUSE    0x0902
#define PCI_CLASS_INPUT_OTHER    0x0980

#define PCI_BASE_CLASS_DOCKING     0x0a
#define PCI_CLASS_DOCKING_GENERIC  0x0a00
#define PCI_CLASS_DOCKING_OTHER    0x0a01

#define PCI_BASE_CLASS_PROCESSOR    0x0b
#define PCI_CLASS_PROCESSOR_386     0x0b00
#define PCI_CLASS_PROCESSOR_486     0x0b01
#define PCI_CLASS_PROCESSOR_PENTIUM 0x0b02
#define PCI_CLASS_PROCESSOR_ALPHA   0x0b10
#define PCI_CLASS_PROCESSOR_POWERPC 0x0b20
#define PCI_CLASS_PROCESSOR_CO      0x0b40

#define PCI_BASE_CLASS_SERIAL     0x0c
#define PCI_CLASS_SERIAL_FIREWIRE 0x0c00
#define PCI_CLASS_SERIAL_ACCESS   0x0c01
#define PCI_CLASS_SERIAL_SSA      0x0c02
#define PCI_CLASS_SERIAL_USB      0x0c03
#define PCI_CLASS_SERIAL_FIBER    0x0c04

#define PCI_CLASS_OTHERS    0xff

/*
 * Vendor and card ID's: sort these numerically according to vendor
 * (and according to card ID within vendor). Send all updates to
 * <linux-pcisupport@cck.uni-kl.de>.
 */
#define PCI_VENDOR_ID_COMPAQ                       0x0e11
#define PCI_DEVICE_ID_COMPAQ_1280                  0x3033
#define PCI_DEVICE_ID_COMPAQ_TRIFLEX               0x4000
#define PCI_DEVICE_ID_COMPAQ_SMART2P               0xae10
#define PCI_DEVICE_ID_COMPAQ_NETEL100              0xae32
#define PCI_DEVICE_ID_COMPAQ_NETEL10               0xae34
#define PCI_DEVICE_ID_COMPAQ_NETFLEX3I             0xae35
#define PCI_DEVICE_ID_COMPAQ_NETEL100D             0xae40
#define PCI_DEVICE_ID_COMPAQ_NETEL100PI            0xae43
#define PCI_DEVICE_ID_COMPAQ_NETEL100I             0xb011
#define PCI_DEVICE_ID_COMPAQ_THUNDER               0xf130
#define PCI_DEVICE_ID_COMPAQ_NETFLEX3B             0xf150

#define PCI_VENDOR_ID_NCR                          0x1000
#define PCI_DEVICE_ID_NCR_53C810                   0x0001
#define PCI_DEVICE_ID_NCR_53C820                   0x0002
#define PCI_DEVICE_ID_NCR_53C825                   0x0003
#define PCI_DEVICE_ID_NCR_53C815                   0x0004
#define PCI_DEVICE_ID_NCR_53C860                   0x0006
#define PCI_DEVICE_ID_NCR_53C896                   0x000b
#define PCI_DEVICE_ID_NCR_53C895                   0x000c
#define PCI_DEVICE_ID_NCR_53C885                   0x000d
#define PCI_DEVICE_ID_NCR_53C875                   0x000f
#define PCI_DEVICE_ID_NCR_53C875J                  0x008f

#define PCI_VENDOR_ID_ATI                          0x1002
#define PCI_DEVICE_ID_ATI_68800                    0x4158
#define PCI_DEVICE_ID_ATI_215CT222                 0x4354
#define PCI_DEVICE_ID_ATI_210888CX                 0x4358
#define PCI_DEVICE_ID_ATI_215GB                    0x4742
#define PCI_DEVICE_ID_ATI_215GD                    0x4744
#define PCI_DEVICE_ID_ATI_215GI                    0x4749
#define PCI_DEVICE_ID_ATI_215GP                    0x4750
#define PCI_DEVICE_ID_ATI_215GQ                    0x4751
#define PCI_DEVICE_ID_ATI_215GT                    0x4754
#define PCI_DEVICE_ID_ATI_215GTB                   0x4755
#define PCI_DEVICE_ID_ATI_210888GX                 0x4758
#define PCI_DEVICE_ID_ATI_215LG                    0x4c47
#define PCI_DEVICE_ID_ATI_264LT                    0x4c54
#define PCI_DEVICE_ID_ATI_264VT                    0x5654

#define PCI_VENDOR_ID_VLSI                         0x1004
#define PCI_DEVICE_ID_VLSI_82C592                  0x0005
#define PCI_DEVICE_ID_VLSI_82C593                  0x0006
#define PCI_DEVICE_ID_VLSI_82C594                  0x0007
#define PCI_DEVICE_ID_VLSI_82C597                  0x0009
#define PCI_DEVICE_ID_VLSI_82C541                  0x000c
#define PCI_DEVICE_ID_VLSI_82C543                  0x000d
#define PCI_DEVICE_ID_VLSI_82C532                  0x0101
#define PCI_DEVICE_ID_VLSI_82C534                  0x0102
#define PCI_DEVICE_ID_VLSI_82C535                  0x0104
#define PCI_DEVICE_ID_VLSI_82C147                  0x0105
#define PCI_DEVICE_ID_VLSI_VAS96011                0x0702

#define PCI_VENDOR_ID_ADL                          0x1005
#define PCI_DEVICE_ID_ADL_2301                     0x2301

#define PCI_VENDOR_ID_NS                           0x100b
#define PCI_DEVICE_ID_NS_87415                     0x0002
#define PCI_DEVICE_ID_NS_87410                     0xd001

#define PCI_VENDOR_ID_TSENG                        0x100c
#define PCI_DEVICE_ID_TSENG_W32P_2                 0x3202
#define PCI_DEVICE_ID_TSENG_W32P_b                 0x3205
#define PCI_DEVICE_ID_TSENG_W32P_c                 0x3206
#define PCI_DEVICE_ID_TSENG_W32P_d                 0x3207
#define PCI_DEVICE_ID_TSENG_ET6000                 0x3208

#define PCI_VENDOR_ID_WEITEK                       0x100e
#define PCI_DEVICE_ID_WEITEK_P9000                 0x9001
#define PCI_DEVICE_ID_WEITEK_P9100                 0x9100

#define PCI_VENDOR_ID_DEC                          0x1011
#define PCI_DEVICE_ID_DEC_BRD                      0x0001
#define PCI_DEVICE_ID_DEC_TULIP                    0x0002
#define PCI_DEVICE_ID_DEC_TGA                      0x0004
#define PCI_DEVICE_ID_DEC_TULIP_FAST               0x0009
#define PCI_DEVICE_ID_DEC_TGA2                     0x000D
#define PCI_DEVICE_ID_DEC_FDDI                     0x000F
#define PCI_DEVICE_ID_DEC_TULIP_PLUS               0x0014
#define PCI_DEVICE_ID_DEC_21142                    0x0019
#define PCI_DEVICE_ID_DEC_21052                    0x0021
#define PCI_DEVICE_ID_DEC_21150                    0x0022
#define PCI_DEVICE_ID_DEC_21152                    0x0024

#define PCI_VENDOR_ID_CIRRUS                       0x1013
#define PCI_DEVICE_ID_CIRRUS_7548                  0x0038
#define PCI_DEVICE_ID_CIRRUS_5430                  0x00a0
#define PCI_DEVICE_ID_CIRRUS_5434_4                0x00a4
#define PCI_DEVICE_ID_CIRRUS_5434_8                0x00a8
#define PCI_DEVICE_ID_CIRRUS_5436                  0x00ac
#define PCI_DEVICE_ID_CIRRUS_5446                  0x00b8
#define PCI_DEVICE_ID_CIRRUS_5480                  0x00bc
#define PCI_DEVICE_ID_CIRRUS_5464                  0x00d4
#define PCI_DEVICE_ID_CIRRUS_5465                  0x00d6
#define PCI_DEVICE_ID_CIRRUS_6729                  0x1100
#define PCI_DEVICE_ID_CIRRUS_6832                  0x1110
#define PCI_DEVICE_ID_CIRRUS_7542                  0x1200
#define PCI_DEVICE_ID_CIRRUS_7543                  0x1202
#define PCI_DEVICE_ID_CIRRUS_7541                  0x1204

#define PCI_VENDOR_ID_IBM                          0x1014
#define PCI_DEVICE_ID_IBM_FIRE_CORAL               0x000a
#define PCI_DEVICE_ID_IBM_TR                       0x0018
#define PCI_DEVICE_ID_IBM_82G2675                  0x001d
#define PCI_DEVICE_ID_IBM_MCA                      0x0020
#define PCI_DEVICE_ID_IBM_82351                    0x0022
#define PCI_DEVICE_ID_IBM_SERVERAID                0x002e
#define PCI_DEVICE_ID_IBM_TR_WAKE                  0x003e
#define PCI_DEVICE_ID_IBM_MPIC                     0x0046
#define PCI_DEVICE_ID_IBM_3780IDSP                 0x007d
#define PCI_DEVICE_ID_IBM_MPIC_2                   0xffff

#define PCI_VENDOR_ID_WD                           0x101c
#define PCI_DEVICE_ID_WD_7197                      0x3296

#define PCI_VENDOR_ID_AMD                          0x1022
#define PCI_DEVICE_ID_AMD_LANCE                    0x2000
#define PCI_DEVICE_ID_AMD_SCSI                     0x2020

#define PCI_VENDOR_ID_TRIDENT                      0x1023
#define PCI_DEVICE_ID_TRIDENT_9397                 0x9397
#define PCI_DEVICE_ID_TRIDENT_9420                 0x9420
#define PCI_DEVICE_ID_TRIDENT_9440                 0x9440
#define PCI_DEVICE_ID_TRIDENT_9660                 0x9660
#define PCI_DEVICE_ID_TRIDENT_9750                 0x9750

#define PCI_VENDOR_ID_AI                           0x1025
#define PCI_DEVICE_ID_AI_M1435                     0x1435

#define PCI_VENDOR_ID_MATROX                       0x102B
#define PCI_DEVICE_ID_MATROX_MGA_2                 0x0518
#define PCI_DEVICE_ID_MATROX_MIL                   0x0519
#define PCI_DEVICE_ID_MATROX_MYS                   0x051A
#define PCI_DEVICE_ID_MATROX_MIL_2                 0x051b
#define PCI_DEVICE_ID_MATROX_MIL_2_AGP             0x051f
#define PCI_DEVICE_ID_MATROX_MGA_IMP               0x0d10

#define PCI_VENDOR_ID_CT                           0x102c
#define PCI_DEVICE_ID_CT_65545                     0x00d8
#define PCI_DEVICE_ID_CT_65548                     0x00dc
#define PCI_DEVICE_ID_CT_65550                     0x00e0
#define PCI_DEVICE_ID_CT_65554                     0x00e4
#define PCI_DEVICE_ID_CT_65555                     0x00e5

#define PCI_VENDOR_ID_MIRO                         0x1031
#define PCI_DEVICE_ID_MIRO_36050                   0x5601

#define PCI_VENDOR_ID_NEC                          0x1033
#define PCI_DEVICE_ID_NEC_PCX2                     0x0046

#define PCI_VENDOR_ID_FD                           0x1036
#define PCI_DEVICE_ID_FD_36C70                     0x0000

#define PCI_VENDOR_ID_SI                           0x1039
#define PCI_DEVICE_ID_SI_5591_AGP                  0x0001
#define PCI_DEVICE_ID_SI_6202                      0x0002
#define PCI_DEVICE_ID_SI_503                       0x0008
#define PCI_DEVICE_ID_SI_ACPI                      0x0009
#define PCI_DEVICE_ID_SI_5597_VGA                  0x0200
#define PCI_DEVICE_ID_SI_6205                      0x0205
#define PCI_DEVICE_ID_SI_501                       0x0406
#define PCI_DEVICE_ID_SI_496                       0x0496
#define PCI_DEVICE_ID_SI_601                       0x0601
#define PCI_DEVICE_ID_SI_5107                      0x5107
#define PCI_DEVICE_ID_SI_5511                      0x5511
#define PCI_DEVICE_ID_SI_5513                      0x5513
#define PCI_DEVICE_ID_SI_5571                      0x5571
#define PCI_DEVICE_ID_SI_5591                      0x5591
#define PCI_DEVICE_ID_SI_5597                      0x5597
#define PCI_DEVICE_ID_SI_7001                      0x7001

#define PCI_VENDOR_ID_HP                           0x103c
#define PCI_DEVICE_ID_HP_J2585A                    0x1030
#define PCI_DEVICE_ID_HP_J2585B                    0x1031

#define PCI_VENDOR_ID_PCTECH                       0x1042
#define PCI_DEVICE_ID_PCTECH_RZ1000                0x1000
#define PCI_DEVICE_ID_PCTECH_RZ1001                0x1001
#define PCI_DEVICE_ID_PCTECH_SAMURAI_0             0x3000
#define PCI_DEVICE_ID_PCTECH_SAMURAI_1             0x3010
#define PCI_DEVICE_ID_PCTECH_SAMURAI_IDE           0x3020

#define PCI_VENDOR_ID_DPT                          0x1044
#define PCI_DEVICE_ID_DPT                          0xa400

#define PCI_VENDOR_ID_OPTI                         0x1045
#define PCI_DEVICE_ID_OPTI_92C178                  0xc178
#define PCI_DEVICE_ID_OPTI_82C557                  0xc557
#define PCI_DEVICE_ID_OPTI_82C558                  0xc558
#define PCI_DEVICE_ID_OPTI_82C621                  0xc621
#define PCI_DEVICE_ID_OPTI_82C700                  0xc700
#define PCI_DEVICE_ID_OPTI_82C701                  0xc701
#define PCI_DEVICE_ID_OPTI_82C814                  0xc814
#define PCI_DEVICE_ID_OPTI_82C822                  0xc822
#define PCI_DEVICE_ID_OPTI_82C825                  0xd568

#define PCI_VENDOR_ID_SGS                          0x104a
#define PCI_DEVICE_ID_SGS_2000                     0x0008
#define PCI_DEVICE_ID_SGS_1764                     0x0009

#define PCI_VENDOR_ID_BUSLOGIC                     0x104B
#define PCI_DEVICE_ID_BUSLOGIC_MULTIMASTER_NC      0x0140
#define PCI_DEVICE_ID_BUSLOGIC_MULTIMASTER         0x1040
#define PCI_DEVICE_ID_BUSLOGIC_FLASHPOINT          0x8130

#define PCI_VENDOR_ID_TI                           0x104c
#define PCI_DEVICE_ID_TI_TVP4010                   0x3d04
#define PCI_DEVICE_ID_TI_TVP4020                   0x3d07
#define PCI_DEVICE_ID_TI_PCI1130                   0xac12
#define PCI_DEVICE_ID_TI_PCI1031                   0xac13
#define PCI_DEVICE_ID_TI_PCI1131                   0xac15
#define PCI_DEVICE_ID_TI_PCI1250                   0xac16
#define PCI_DEVICE_ID_TI_PCI1220                   0xac17

#define PCI_VENDOR_ID_OAK                          0x104e
#define PCI_DEVICE_ID_OAK_OTI107                   0x0107

#define PCI_VENDOR_ID_WINBOND2                     0x1050
#define PCI_DEVICE_ID_WINBOND2_89C940              0x0940

#define PCI_VENDOR_ID_MOTOROLA                     0x1057
#define PCI_DEVICE_ID_MOTOROLA_MPC105              0x0001
#define PCI_DEVICE_ID_MOTOROLA_MPC106              0x0002
#define PCI_DEVICE_ID_MOTOROLA_RAVEN               0x4801
#define PCI_DEVICE_ID_MOTOROLA_HAWK                0x4803

#define PCI_VENDOR_ID_PROMISE                      0x105a
#define PCI_DEVICE_ID_PROMISE_20246                0x4d33
#define PCI_DEVICE_ID_PROMISE_5300                 0x5300

#define PCI_VENDOR_ID_N9                           0x105d
#define PCI_DEVICE_ID_N9_I128                      0x2309
#define PCI_DEVICE_ID_N9_I128_2                    0x2339
#define PCI_DEVICE_ID_N9_I128_T2R                  0x493d

#define PCI_VENDOR_ID_UMC                          0x1060
#define PCI_DEVICE_ID_UMC_UM8673F                  0x0101
#define PCI_DEVICE_ID_UMC_UM8891A                  0x0891
#define PCI_DEVICE_ID_UMC_UM8886BF                 0x673a
#define PCI_DEVICE_ID_UMC_UM8886A                  0x886a
#define PCI_DEVICE_ID_UMC_UM8881F                  0x8881
#define PCI_DEVICE_ID_UMC_UM8886F                  0x8886
#define PCI_DEVICE_ID_UMC_UM9017F                  0x9017
#define PCI_DEVICE_ID_UMC_UM8886N                  0xe886
#define PCI_DEVICE_ID_UMC_UM8891N                  0xe891

#define PCI_VENDOR_ID_X                            0x1061
#define PCI_DEVICE_ID_X_AGX016                     0x0001

#define PCI_VENDOR_ID_PICOP                        0x1066
#define PCI_DEVICE_ID_PICOP_PT86C52X               0x0001
#define PCI_DEVICE_ID_PICOP_PT80C524               0x8002

#define PCI_VENDOR_ID_APPLE                        0x106b
#define PCI_DEVICE_ID_APPLE_BANDIT                 0x0001
#define PCI_DEVICE_ID_APPLE_GC                     0x0002
#define PCI_DEVICE_ID_APPLE_HYDRA                  0x000e

#define PCI_VENDOR_ID_NEXGEN                       0x1074
#define PCI_DEVICE_ID_NEXGEN_82C501                0x4e78

#define PCI_VENDOR_ID_QLOGIC                       0x1077
#define PCI_DEVICE_ID_QLOGIC_ISP1020               0x1020
#define PCI_DEVICE_ID_QLOGIC_ISP1022               0x1022

#define PCI_VENDOR_ID_CYRIX                        0x1078
#define PCI_DEVICE_ID_CYRIX_5510                   0x0000
#define PCI_DEVICE_ID_CYRIX_PCI_MASTER             0x0001
#define PCI_DEVICE_ID_CYRIX_5520                   0x0002
#define PCI_DEVICE_ID_CYRIX_5530_LEGACY            0x0100
#define PCI_DEVICE_ID_CYRIX_5530_SMI               0x0101
#define PCI_DEVICE_ID_CYRIX_5530_IDE               0x0102
#define PCI_DEVICE_ID_CYRIX_5530_AUDIO             0x0103
#define PCI_DEVICE_ID_CYRIX_5530_VIDEO             0x0104

#define PCI_VENDOR_ID_LEADTEK                      0x107d
#define PCI_DEVICE_ID_LEADTEK_805                  0x0000

#define PCI_VENDOR_ID_CONTAQ                       0x1080
#define PCI_DEVICE_ID_CONTAQ_82C599                0x0600
#define PCI_DEVICE_ID_CONTAQ_82C693                0xc693

#define PCI_VENDOR_ID_FOREX                        0x1083

#define PCI_VENDOR_ID_OLICOM                       0x108d
#define PCI_DEVICE_ID_OLICOM_OC3136                0x0001
#define PCI_DEVICE_ID_OLICOM_OC2315                0x0011
#define PCI_DEVICE_ID_OLICOM_OC2325                0x0012
#define PCI_DEVICE_ID_OLICOM_OC2183                0x0013
#define PCI_DEVICE_ID_OLICOM_OC2326                0x0014
#define PCI_DEVICE_ID_OLICOM_OC6151                0x0021

#define PCI_VENDOR_ID_SUN                          0x108e
#define PCI_DEVICE_ID_SUN_EBUS                     0x1000
#define PCI_DEVICE_ID_SUN_HAPPYMEAL                0x1001
#define PCI_DEVICE_ID_SUN_SIMBA                    0x5000
#define PCI_DEVICE_ID_SUN_PBM                      0x8000
#define PCI_DEVICE_ID_SUN_SABRE                    0xa000

#define PCI_VENDOR_ID_CMD                          0x1095
#define PCI_DEVICE_ID_CMD_640                      0x0640
#define PCI_DEVICE_ID_CMD_643                      0x0643
#define PCI_DEVICE_ID_CMD_646                      0x0646
#define PCI_DEVICE_ID_CMD_647                      0x0647
#define PCI_DEVICE_ID_CMD_670                      0x0670

#define PCI_VENDOR_ID_VISION                       0x1098
#define PCI_DEVICE_ID_VISION_QD8500                0x0001
#define PCI_DEVICE_ID_VISION_QD8580                0x0002

#define PCI_VENDOR_ID_BROOKTREE                    0x109e
#define PCI_DEVICE_ID_BROOKTREE_848                0x0350
#define PCI_DEVICE_ID_BROOKTREE_849A               0x0351
#define PCI_DEVICE_ID_BROOKTREE_8474               0x8474

#define PCI_VENDOR_ID_SIERRA                       0x10a8
#define PCI_DEVICE_ID_SIERRA_STB                   0x0000

#define PCI_VENDOR_ID_ACC                          0x10aa
#define PCI_DEVICE_ID_ACC_2056                     0x0000

#define PCI_VENDOR_ID_WINBOND                      0x10ad
#define PCI_DEVICE_ID_WINBOND_83769                0x0001
#define PCI_DEVICE_ID_WINBOND_82C105               0x0105
#define PCI_DEVICE_ID_WINBOND_83C553               0x0565

#define PCI_VENDOR_ID_DATABOOK                     0x10b3
#define PCI_DEVICE_ID_DATABOOK_87144               0xb106

#define PCI_VENDOR_ID_PLX                          0x10b5
#define PCI_DEVICE_ID_PLX_9050                     0x9050
#define PCI_DEVICE_ID_PLX_9060                     0x9060
#define PCI_DEVICE_ID_PLX_9060ES                   0x906E
#define PCI_DEVICE_ID_PLX_9060SD                   0x906D
#define PCI_DEVICE_ID_PLX_9080                     0x9080

#define PCI_VENDOR_ID_MADGE                        0x10b6
#define PCI_DEVICE_ID_MADGE_MK2                    0x0002
#define PCI_DEVICE_ID_MADGE_C155S                  0x1001

#define PCI_VENDOR_ID_3COM                         0x10b7
#define PCI_DEVICE_ID_3COM_3C339                   0x3390
#define PCI_DEVICE_ID_3COM_3C590                   0x5900
#define PCI_DEVICE_ID_3COM_3C595TX                 0x5950
#define PCI_DEVICE_ID_3COM_3C595T4                 0x5951
#define PCI_DEVICE_ID_3COM_3C595MII                0x5952
#define PCI_DEVICE_ID_3COM_3C900TPO                0x9000
#define PCI_DEVICE_ID_3COM_3C900COMBO              0x9001
#define PCI_DEVICE_ID_3COM_3C905TX                 0x9050
#define PCI_DEVICE_ID_3COM_3C905T4                 0x9051
#define PCI_DEVICE_ID_3COM_3C905B_TX               0x9055

#define PCI_VENDOR_ID_SMC                          0x10b8
#define PCI_DEVICE_ID_SMC_EPIC100                  0x0005

#define PCI_VENDOR_ID_AL                           0x10b9
#define PCI_DEVICE_ID_AL_M1445                     0x1445
#define PCI_DEVICE_ID_AL_M1449                     0x1449
#define PCI_DEVICE_ID_AL_M1451                     0x1451
#define PCI_DEVICE_ID_AL_M1461                     0x1461
#define PCI_DEVICE_ID_AL_M1489                     0x1489
#define PCI_DEVICE_ID_AL_M1511                     0x1511
#define PCI_DEVICE_ID_AL_M1513                     0x1513
#define PCI_DEVICE_ID_AL_M1521                     0x1521
#define PCI_DEVICE_ID_AL_M1523                     0x1523
#define PCI_DEVICE_ID_AL_M1531                     0x1531
#define PCI_DEVICE_ID_AL_M1533                     0x1533
#define PCI_DEVICE_ID_AL_M3307                     0x3307
#define PCI_DEVICE_ID_AL_M4803                     0x5215
#define PCI_DEVICE_ID_AL_M5219                     0x5219
#define PCI_DEVICE_ID_AL_M5229                     0x5229
#define PCI_DEVICE_ID_AL_M5237                     0x5237
#define PCI_DEVICE_ID_AL_M7101                     0x7101

#define PCI_VENDOR_ID_MITSUBISHI                   0x10ba

#define PCI_VENDOR_ID_SURECOM                      0x10bd
#define PCI_DEVICE_ID_SURECOM_NE34                 0x0e34

#define PCI_VENDOR_ID_NEOMAGIC                     0x10c8
#define PCI_DEVICE_ID_NEOMAGIC_MAGICGRAPH_NM2070   0x0001
#define PCI_DEVICE_ID_NEOMAGIC_MAGICGRAPH_128V     0x0002
#define PCI_DEVICE_ID_NEOMAGIC_MAGICGRAPH_128ZV    0x0003
#define PCI_DEVICE_ID_NEOMAGIC_MAGICGRAPH_NM2160   0x0004

#define PCI_VENDOR_ID_ASP                          0x10cd
#define PCI_DEVICE_ID_ASP_ABP940                   0x1200
#define PCI_DEVICE_ID_ASP_ABP940U                  0x1300
#define PCI_DEVICE_ID_ASP_ABP940UW                 0x2300

#define PCI_VENDOR_ID_MACRONIX                     0x10d9
#define PCI_DEVICE_ID_MACRONIX_MX98713             0x0512
#define PCI_DEVICE_ID_MACRONIX_MX987x5             0x0531

#define PCI_VENDOR_ID_CERN                         0x10dc
#define PCI_DEVICE_ID_CERN_SPSB_PMC                0x0001
#define PCI_DEVICE_ID_CERN_SPSB_PCI                0x0002
#define PCI_DEVICE_ID_CERN_HIPPI_DST               0x0021
#define PCI_DEVICE_ID_CERN_HIPPI_SRC               0x0022

#define PCI_VENDOR_ID_NVIDIA                       0x10de

#define PCI_VENDOR_ID_IMS                          0x10e0
#define PCI_DEVICE_ID_IMS_8849                     0x8849

#define PCI_VENDOR_ID_TEKRAM2                      0x10e1
#define PCI_DEVICE_ID_TEKRAM2_690c                 0x690c

#define PCI_VENDOR_ID_TUNDRA                       0x10e3
#define PCI_DEVICE_ID_TUNDRA_CA91C042              0x0000

#define PCI_VENDOR_ID_AMCC                         0x10e8
#define PCI_DEVICE_ID_AMCC_MYRINET                 0x8043
#define PCI_DEVICE_ID_AMCC_PARASTATION             0x8062
#define PCI_DEVICE_ID_AMCC_S5933                   0x807d
#define PCI_DEVICE_ID_AMCC_S5933_HEPC3             0x809c

#define PCI_VENDOR_ID_INTERG                       0x10ea
#define PCI_DEVICE_ID_INTERG_1680                  0x1680
#define PCI_DEVICE_ID_INTERG_1682                  0x1682

#define PCI_VENDOR_ID_REALTEK                      0x10ec
#define PCI_DEVICE_ID_REALTEK_8029                 0x8029
#define PCI_DEVICE_ID_REALTEK_8129                 0x8129
#define PCI_DEVICE_ID_REALTEK_8139                 0x8139

#define PCI_VENDOR_ID_TRUEVISION                   0x10fa
#define PCI_DEVICE_ID_TRUEVISION_T1000             0x000c

#define PCI_VENDOR_ID_INIT                         0x1101
#define PCI_DEVICE_ID_INIT_320P                    0x9100
#define PCI_DEVICE_ID_INIT_360P                    0x9500

#define PCI_VENDOR_ID_TTI                          0x1103
#define PCI_DEVICE_ID_TTI_HPT343                   0x0003

#define PCI_VENDOR_ID_VIA                          0x1106
#define PCI_DEVICE_ID_VIA_82C505                   0x0505
#define PCI_DEVICE_ID_VIA_82C561                   0x0561
#define PCI_DEVICE_ID_VIA_82C586_1                 0x0571
#define PCI_DEVICE_ID_VIA_82C576                   0x0576
#define PCI_DEVICE_ID_VIA_82C585                   0x0585
#define PCI_DEVICE_ID_VIA_82C586_0                 0x0586
#define PCI_DEVICE_ID_VIA_82C595                   0x0595
#define PCI_DEVICE_ID_VIA_82C597_0                 0x0597
#define PCI_DEVICE_ID_VIA_82C926                   0x0926
#define PCI_DEVICE_ID_VIA_82C416                   0x1571
#define PCI_DEVICE_ID_VIA_82C595_97                0x1595
#define PCI_DEVICE_ID_VIA_82C586_2                 0x3038
#define PCI_DEVICE_ID_VIA_82C586_3                 0x3040
#define PCI_DEVICE_ID_VIA_86C100A                  0x6100
#define PCI_DEVICE_ID_VIA_82C597_1                 0x8597

#define PCI_VENDOR_ID_VORTEX                       0x1119
#define PCI_DEVICE_ID_VORTEX_GDT60x0               0x0000
#define PCI_DEVICE_ID_VORTEX_GDT6000B              0x0001
#define PCI_DEVICE_ID_VORTEX_GDT6x10               0x0002
#define PCI_DEVICE_ID_VORTEX_GDT6x20               0x0003
#define PCI_DEVICE_ID_VORTEX_GDT6530               0x0004
#define PCI_DEVICE_ID_VORTEX_GDT6550               0x0005
#define PCI_DEVICE_ID_VORTEX_GDT6x17               0x0006
#define PCI_DEVICE_ID_VORTEX_GDT6x27               0x0007
#define PCI_DEVICE_ID_VORTEX_GDT6537               0x0008
#define PCI_DEVICE_ID_VORTEX_GDT6557               0x0009
#define PCI_DEVICE_ID_VORTEX_GDT6x15               0x000a
#define PCI_DEVICE_ID_VORTEX_GDT6x25               0x000b
#define PCI_DEVICE_ID_VORTEX_GDT6535               0x000c
#define PCI_DEVICE_ID_VORTEX_GDT6555               0x000d
#define PCI_DEVICE_ID_VORTEX_GDT6x17RP             0x0100
#define PCI_DEVICE_ID_VORTEX_GDT6x27RP             0x0101
#define PCI_DEVICE_ID_VORTEX_GDT6537RP             0x0102
#define PCI_DEVICE_ID_VORTEX_GDT6557RP             0x0103
#define PCI_DEVICE_ID_VORTEX_GDT6x11RP             0x0104
#define PCI_DEVICE_ID_VORTEX_GDT6x21RP             0x0105
#define PCI_DEVICE_ID_VORTEX_GDT6x17RP1            0x0110
#define PCI_DEVICE_ID_VORTEX_GDT6x27RP1            0x0111
#define PCI_DEVICE_ID_VORTEX_GDT6537RP1            0x0112
#define PCI_DEVICE_ID_VORTEX_GDT6557RP1            0x0113
#define PCI_DEVICE_ID_VORTEX_GDT6x11RP1            0x0114
#define PCI_DEVICE_ID_VORTEX_GDT6x21RP1            0x0115
#define PCI_DEVICE_ID_VORTEX_GDT6x17RP2            0x0120
#define PCI_DEVICE_ID_VORTEX_GDT6x27RP2            0x0121
#define PCI_DEVICE_ID_VORTEX_GDT6537RP2            0x0122
#define PCI_DEVICE_ID_VORTEX_GDT6557RP2            0x0123
#define PCI_DEVICE_ID_VORTEX_GDT6x11RP2            0x0124
#define PCI_DEVICE_ID_VORTEX_GDT6x21RP2            0x0125

#define PCI_VENDOR_ID_EF                           0x111a
#define PCI_DEVICE_ID_EF_ATM_FPGA                  0x0000
#define PCI_DEVICE_ID_EF_ATM_ASIC                  0x0002

#define PCI_VENDOR_ID_FORE                         0x1127
#define PCI_DEVICE_ID_FORE_PCA200PC                0x0210
#define PCI_DEVICE_ID_FORE_PCA200E                 0x0300

#define PCI_VENDOR_ID_IMAGINGTECH                  0x112f
#define PCI_DEVICE_ID_IMAGINGTECH_ICPCI            0x0000

#define PCI_VENDOR_ID_PHILIPS                      0x1131
#define PCI_DEVICE_ID_PHILIPS_SAA7145              0x7145
#define PCI_DEVICE_ID_PHILIPS_SAA7146              0x7146

#define PCI_VENDOR_ID_CYCLONE                      0x113c
#define PCI_DEVICE_ID_CYCLONE_SDK                  0x0001

#define PCI_VENDOR_ID_ALLIANCE                     0x1142
#define PCI_DEVICE_ID_ALLIANCE_PROMOTIO            0x3210
#define PCI_DEVICE_ID_ALLIANCE_PROVIDEO            0x6422
#define PCI_DEVICE_ID_ALLIANCE_AT24                0x6424
#define PCI_DEVICE_ID_ALLIANCE_AT3D                0x643d

#define PCI_VENDOR_ID_SK                           0x1148
#define PCI_DEVICE_ID_SK_FP                        0x4000
#define PCI_DEVICE_ID_SK_TR                        0x4200
#define PCI_DEVICE_ID_SK_GE                        0x4300

#define PCI_VENDOR_ID_VMIC                         0x114a
#define PCI_DEVICE_ID_VMIC_VME                     0x7587

#define PCI_VENDOR_ID_DIGI                         0x114f
#define PCI_DEVICE_ID_DIGI_EPC                     0x0002
#define PCI_DEVICE_ID_DIGI_RIGHTSWITCH             0x0003
#define PCI_DEVICE_ID_DIGI_XEM                     0x0004
#define PCI_DEVICE_ID_DIGI_XR                      0x0005
#define PCI_DEVICE_ID_DIGI_CX                      0x0006
#define PCI_DEVICE_ID_DIGI_XRJ                     0x0009
#define PCI_DEVICE_ID_DIGI_EPCJ                    0x000a
#define PCI_DEVICE_ID_DIGI_XR_920                  0x0027

#define PCI_VENDOR_ID_MUTECH                       0x1159
#define PCI_DEVICE_ID_MUTECH_MV1000                0x0001

#define PCI_VENDOR_ID_RENDITION                    0x1163
#define PCI_DEVICE_ID_RENDITION_VERITE             0x0001
#define PCI_DEVICE_ID_RENDITION_VERITE2100         0x2000

#define PCI_VENDOR_ID_TOSHIBA                      0x1179
#define PCI_DEVICE_ID_TOSHIBA_601                  0x0601
#define PCI_DEVICE_ID_TOSHIBA_TOPIC95              0x060a
#define PCI_DEVICE_ID_TOSHIBA_TOPIC97              0x060f

#define PCI_VENDOR_ID_RICOH                        0x1180
#define PCI_DEVICE_ID_RICOH_RL5C465                0x0465
#define PCI_DEVICE_ID_RICOH_RL5C466                0x0466
#define PCI_DEVICE_ID_RICOH_RL5C475                0x0475
#define PCI_DEVICE_ID_RICOH_RL5C478                0x0478

#define PCI_VENDOR_ID_ARTOP                        0x1191
#define PCI_DEVICE_ID_ARTOP_ATP8400                0x0004
#define PCI_DEVICE_ID_ARTOP_ATP850UF               0x0005

#define PCI_VENDOR_ID_ZEITNET                      0x1193
#define PCI_DEVICE_ID_ZEITNET_1221                 0x0001
#define PCI_DEVICE_ID_ZEITNET_1225                 0x0002

#define PCI_VENDOR_ID_OMEGA                        0x119b
#define PCI_DEVICE_ID_OMEGA_82C092G                0x1221

#define PCI_VENDOR_ID_LITEON                       0x11ad
#define PCI_DEVICE_ID_LITEON_LNE100TX              0x0002

#define PCI_VENDOR_ID_NP                           0x11bc
#define PCI_DEVICE_ID_NP_PCI_FDDI                  0x0001

#define PCI_VENDOR_ID_ATT                          0x11c1
#define PCI_DEVICE_ID_ATT_L56XMF                   0x0440

#define PCI_VENDOR_ID_SPECIALIX                    0x11cb
#define PCI_DEVICE_ID_SPECIALIX_IO8                0x2000
#define PCI_DEVICE_ID_SPECIALIX_XIO                0x4000
#define PCI_DEVICE_ID_SPECIALIX_RIO                0x8000

#define PCI_VENDOR_ID_AURAVISION                   0x11d1
#define PCI_DEVICE_ID_AURAVISION_VXP524            0x01f7

#define PCI_VENDOR_ID_IKON                         0x11d5
#define PCI_DEVICE_ID_IKON_10115                   0x0115
#define PCI_DEVICE_ID_IKON_10117                   0x0117

#define PCI_VENDOR_ID_ZORAN                        0x11de
#define PCI_DEVICE_ID_ZORAN_36057                  0x6057
#define PCI_DEVICE_ID_ZORAN_36120                  0x6120

#define PCI_VENDOR_ID_KINETIC                      0x11f4
#define PCI_DEVICE_ID_KINETIC_2915                 0x2915

#define PCI_VENDOR_ID_COMPEX                       0x11f6
#define PCI_DEVICE_ID_COMPEX_ENET100VG4            0x0112
#define PCI_DEVICE_ID_COMPEX_RL2000                0x1401

#define PCI_VENDOR_ID_RP                           0x11fe
#define PCI_DEVICE_ID_RP32INTF                     0x0001
#define PCI_DEVICE_ID_RP8INTF                      0x0002
#define PCI_DEVICE_ID_RP16INTF                     0x0003
#define PCI_DEVICE_ID_RP4QUAD                      0x0004
#define PCI_DEVICE_ID_RP8OCTA                      0x0005
#define PCI_DEVICE_ID_RP8J                         0x0006
#define PCI_DEVICE_ID_RPP4                         0x000A
#define PCI_DEVICE_ID_RPP8                         0x000B
#define PCI_DEVICE_ID_RP8M                         0x000C

#define PCI_VENDOR_ID_CYCLADES                     0x120e
#define PCI_DEVICE_ID_CYCLOM_Y_Lo                  0x0100
#define PCI_DEVICE_ID_CYCLOM_Y_Hi                  0x0101
#define PCI_DEVICE_ID_CYCLOM_Z_Lo                  0x0200
#define PCI_DEVICE_ID_CYCLOM_Z_Hi                  0x0201

#define PCI_VENDOR_ID_ESSENTIAL                    0x120f
#define PCI_DEVICE_ID_ESSENTIAL_ROADRUNNER         0x0001

#define PCI_VENDOR_ID_O2                           0x1217
#define PCI_DEVICE_ID_O2_6729                      0x6729
#define PCI_DEVICE_ID_O2_6730                      0x673a
#define PCI_DEVICE_ID_O2_6832                      0x6832
#define PCI_DEVICE_ID_O2_6836                      0x6836

#define PCI_VENDOR_ID_3DFX                         0x121a
#define PCI_DEVICE_ID_3DFX_VOODOO                  0x0001
#define PCI_DEVICE_ID_3DFX_VOODOO2                 0x0002

#define PCI_VENDOR_ID_SIGMADES                     0x1236
#define PCI_DEVICE_ID_SIGMADES_6425                0x6401

#define PCI_VENDOR_ID_CCUBE                        0x123f

#define PCI_VENDOR_ID_DIPIX                        0x1246

#define PCI_VENDOR_ID_STALLION                     0x124d
#define PCI_DEVICE_ID_STALLION_ECHPCI832           0x0000
#define PCI_DEVICE_ID_STALLION_ECHPCI864           0x0002
#define PCI_DEVICE_ID_STALLION_EIOPCI              0x0003

#define PCI_VENDOR_ID_OPTIBASE                     0x1255
#define PCI_DEVICE_ID_OPTIBASE_FORGE               0x1110
#define PCI_DEVICE_ID_OPTIBASE_FUSION              0x1210
#define PCI_DEVICE_ID_OPTIBASE_VPLEX               0x2110
#define PCI_DEVICE_ID_OPTIBASE_VPLEXCC             0x2120
#define PCI_DEVICE_ID_OPTIBASE_VQUEST              0x2130

#define PCI_VENDOR_ID_SATSAGEM                     0x1267
#define PCI_DEVICE_ID_SATSAGEM_PCR2101             0x5352
#define PCI_DEVICE_ID_SATSAGEM_TELSATTURBO         0x5a4b

#define PCI_VENDOR_ID_HUGHES                       0x1273
#define PCI_DEVICE_ID_HUGHES_DIRECPC               0x0002

#define PCI_VENDOR_ID_ENSONIQ                      0x1274
#define PCI_DEVICE_ID_ENSONIQ_AUDIOPCI             0x5000

#define PCI_VENDOR_ID_ALTEON                       0x12ae
#define PCI_DEVICE_ID_ALTEON_ACENIC                0x0001

#define PCI_VENDOR_ID_PICTUREL                     0x12c5
#define PCI_DEVICE_ID_PICTUREL_PCIVST              0x0081

#define PCI_VENDOR_ID_NVIDIA_SGS                   0x12d2
#define PCI_DEVICE_ID_NVIDIA_SGS_RIVA128           0x0018

#define PCI_VENDOR_ID_CBOARDS                      0x1307
#define PCI_DEVICE_ID_CBOARDS_DAS1602_16           0x0001

#define PCI_VENDOR_ID_SYMPHONY                     0x1c1c
#define PCI_DEVICE_ID_SYMPHONY_101                 0x0001

#define PCI_VENDOR_ID_TEKRAM                       0x1de1
#define PCI_DEVICE_ID_TEKRAM_DC290                 0xdc29

#define PCI_VENDOR_ID_3DLABS                       0x3d3d
#define PCI_DEVICE_ID_3DLABS_300SX                 0x0001
#define PCI_DEVICE_ID_3DLABS_500TX                 0x0002
#define PCI_DEVICE_ID_3DLABS_DELTA                 0x0003
#define PCI_DEVICE_ID_3DLABS_PERMEDIA              0x0004
#define PCI_DEVICE_ID_3DLABS_MX                    0x0006

#define PCI_VENDOR_ID_AVANCE                       0x4005
#define PCI_DEVICE_ID_AVANCE_ALG2064               0x2064
#define PCI_DEVICE_ID_AVANCE_2302                  0x2302

#define PCI_VENDOR_ID_NETVIN                       0x4a14
#define PCI_DEVICE_ID_NETVIN_NV5000SC              0x5000

#define PCI_VENDOR_ID_S3                           0x5333
#define PCI_DEVICE_ID_S3_PLATO_PXS                 0x0551
#define PCI_DEVICE_ID_S3_ViRGE                     0x5631
#define PCI_DEVICE_ID_S3_TRIO                      0x8811
#define PCI_DEVICE_ID_S3_AURORA64VP                0x8812
#define PCI_DEVICE_ID_S3_TRIO64UVP                 0x8814
#define PCI_DEVICE_ID_S3_ViRGE_VX                  0x883d
#define PCI_DEVICE_ID_S3_868                       0x8880
#define PCI_DEVICE_ID_S3_928                       0x88b0
#define PCI_DEVICE_ID_S3_864_1                     0x88c0
#define PCI_DEVICE_ID_S3_864_2                     0x88c1
#define PCI_DEVICE_ID_S3_964_1                     0x88d0
#define PCI_DEVICE_ID_S3_964_2                     0x88d1
#define PCI_DEVICE_ID_S3_968                       0x88f0
#define PCI_DEVICE_ID_S3_TRIO64V2                  0x8901
#define PCI_DEVICE_ID_S3_PLATO_PXG                 0x8902
#define PCI_DEVICE_ID_S3_ViRGE_DXGX                0x8a01
#define PCI_DEVICE_ID_S3_ViRGE_GX2                 0x8a10
#define PCI_DEVICE_ID_S3_ViRGE_MX                  0x8c01
#define PCI_DEVICE_ID_S3_ViRGE_MXP                 0x8c02
#define PCI_DEVICE_ID_S3_ViRGE_MXPMV               0x8c03
#define PCI_DEVICE_ID_S3_SONICVIBES                0xca00

#define PCI_VENDOR_ID_INTEL                        0x8086
#define PCI_DEVICE_ID_INTEL_82375                  0x0482
#define PCI_DEVICE_ID_INTEL_82424                  0x0483
#define PCI_DEVICE_ID_INTEL_82378                  0x0484
#define PCI_DEVICE_ID_INTEL_82430                  0x0486
#define PCI_DEVICE_ID_INTEL_82434                  0x04a3
#define PCI_DEVICE_ID_INTEL_82092AA_0              0x1221
#define PCI_DEVICE_ID_INTEL_82092AA_1              0x1222
#define PCI_DEVICE_ID_INTEL_7116                   0x1223
#define PCI_DEVICE_ID_INTEL_82596                  0x1226
#define PCI_DEVICE_ID_INTEL_82865                  0x1227
#define PCI_DEVICE_ID_INTEL_82557                  0x1229
#define PCI_DEVICE_ID_INTEL_82437                  0x122d
#define PCI_DEVICE_ID_INTEL_82371FB_0              0x122e
#define PCI_DEVICE_ID_INTEL_82371FB_1              0x1230
#define PCI_DEVICE_ID_INTEL_82371MX                0x1234
#define PCI_DEVICE_ID_INTEL_82437MX                0x1235
#define PCI_DEVICE_ID_INTEL_82441                  0x1237
#define PCI_DEVICE_ID_INTEL_82380FB                0x124b
#define PCI_DEVICE_ID_INTEL_82439                  0x1250
#define PCI_DEVICE_ID_INTEL_82371SB_0              0x7000
#define PCI_DEVICE_ID_INTEL_82371SB_1              0x7010
#define PCI_DEVICE_ID_INTEL_82371SB_2              0x7020
#define PCI_DEVICE_ID_INTEL_82437VX                0x7030
#define PCI_DEVICE_ID_INTEL_82439TX                0x7100
#define PCI_DEVICE_ID_INTEL_82371AB_0              0x7110
#define PCI_DEVICE_ID_INTEL_82371AB                0x7111
#define PCI_DEVICE_ID_INTEL_82371AB_2              0x7112
#define PCI_DEVICE_ID_INTEL_82371AB_3              0x7113
#define PCI_DEVICE_ID_INTEL_82443LX_0              0x7180
#define PCI_DEVICE_ID_INTEL_82443LX_1              0x7181
#define PCI_DEVICE_ID_INTEL_82443BX_0              0x7190
#define PCI_DEVICE_ID_INTEL_82443BX_1              0x7191
#define PCI_DEVICE_ID_INTEL_82443BX_2              0x7192
#define PCI_DEVICE_ID_INTEL_P6                     0x84c4
#define PCI_DEVICE_ID_INTEL_82450GX                0x84c5

#define PCI_VENDOR_ID_KTI                          0x8e2e
#define PCI_DEVICE_ID_KTI_ET32P2                   0x3000

#define PCI_VENDOR_ID_ADAPTEC                      0x9004
#define PCI_DEVICE_ID_ADAPTEC_7810                 0x1078
#define PCI_DEVICE_ID_ADAPTEC_7850                 0x5078
#define PCI_DEVICE_ID_ADAPTEC_7855                 0x5578
#define PCI_DEVICE_ID_ADAPTEC_5800                 0x5800
#define PCI_DEVICE_ID_ADAPTEC_1480A                0x6075
#define PCI_DEVICE_ID_ADAPTEC_7860                 0x6078
#define PCI_DEVICE_ID_ADAPTEC_7861                 0x6178
#define PCI_DEVICE_ID_ADAPTEC_7870                 0x7078
#define PCI_DEVICE_ID_ADAPTEC_7871                 0x7178
#define PCI_DEVICE_ID_ADAPTEC_7872                 0x7278
#define PCI_DEVICE_ID_ADAPTEC_7873                 0x7378
#define PCI_DEVICE_ID_ADAPTEC_7874                 0x7478
#define PCI_DEVICE_ID_ADAPTEC_7895                 0x7895
#define PCI_DEVICE_ID_ADAPTEC_7880                 0x8078
#define PCI_DEVICE_ID_ADAPTEC_7881                 0x8178
#define PCI_DEVICE_ID_ADAPTEC_7882                 0x8278
#define PCI_DEVICE_ID_ADAPTEC_7883                 0x8378
#define PCI_DEVICE_ID_ADAPTEC_7884                 0x8478
#define PCI_DEVICE_ID_ADAPTEC_1030                 0x8b78

#define PCI_VENDOR_ID_ADAPTEC2                     0x9005
#define PCI_DEVICE_ID_ADAPTEC2_2940U2              0x0010
#define PCI_DEVICE_ID_ADAPTEC2_7890                0x001f
#define PCI_DEVICE_ID_ADAPTEC2_3940U2              0x0050
#define PCI_DEVICE_ID_ADAPTEC2_7896                0x005f

#define PCI_VENDOR_ID_ATRONICS                     0x907f
#define PCI_DEVICE_ID_ATRONICS_2015                0x2015

#define PCI_VENDOR_ID_HOLTEK                       0x9412
#define PCI_DEVICE_ID_HOLTEK_6565                  0x6565

#define PCI_VENDOR_ID_TIGERJET                     0xe159
#define PCI_DEVICE_ID_TIGERJET_300                 0x0001

#define PCI_VENDOR_ID_ARK                          0xedd8
#define PCI_DEVICE_ID_ARK_STING                    0xa091
#define PCI_DEVICE_ID_ARK_STINGARK                 0xa099
#define PCI_DEVICE_ID_ARK_2000MT                   0xa0a1
/*
 * The PCI interface treats multi-function devices as independent
 * devices.  The slot/function address of each device is encoded
 * in a single byte as follows:
 *
 *  7:3 = slot
 *  2:0 = function
 */
#define PCI_DEVFN(_slot,_func) ((((_slot) & 0x1f) << 3) | ((_func) & 0x07))
#define PCI_SLOT(_devfn)      (((_devfn) >> 3) & 0x1f)
#define PCI_FUNC(_devfn)      ((_devfn) & 0x07)

/*
 * Error values that may be returned by the PCI bios.
 */
#define PCIBIOS_SUCCESSFUL          0x00
#define PCIBIOS_FUNC_NOT_SUPPORTED  0x81
#define PCIBIOS_BAD_VENDOR_ID       0x83
#define PCIBIOS_DEVICE_NOT_FOUND    0x86
#define PCIBIOS_BAD_REGISTER_NUMBER 0x87
#define PCIBIOS_SET_FAILED          0x88
#define PCIBIOS_BUFFER_TOO_SMALL    0x89

/* T. Straumann, 7/31/2001: increased to 32 - PMC slots are not
 * scanned on mvme2306 otherwise
 */
#define PCI_MAX_DEVICES      32
#define PCI_MAX_FUNCTIONS    8

typedef struct  {
  int (*read_config_byte)(unsigned char, unsigned char,  unsigned char,
                          unsigned char, uint8_t *);
  int (*read_config_word)(unsigned char, unsigned char,  unsigned char,
                          unsigned char, uint16_t *);
  int (*read_config_dword)(unsigned char, unsigned char,  unsigned char,
                           unsigned char, uint32_t *);
  int (*write_config_byte)(unsigned char, unsigned char,  unsigned char,
                           unsigned char, uint8_t);
  int (*write_config_word)(unsigned char, unsigned char,  unsigned char,
                           unsigned char, uint16_t);
  int (*write_config_dword)(unsigned char, unsigned char,  unsigned char,
                            unsigned char, uint32_t);
} pci_config_access_functions;

/* Error codes for pci_initialize */
#define PCIB_ERR_SUCCESS       (0)
#define PCIB_ERR_UNINITIALIZED (-1)  /* PCI BIOS is not initilized */
#define PCIB_ERR_NOTPRESENT    (-2)  /* PCI BIOS not present */
#define PCIB_ERR_NOFUNC        (-3)  /* Function not supported */
#define PCIB_ERR_BADVENDOR     (-4)  /* Bad Vendor ID */
#define PCIB_ERR_DEVNOTFOUND   (-5)  /* Device not found */
#define PCIB_ERR_BADREG        (-6)  /* Bad register number */

extern int pci_initialize();

typedef struct {
  volatile unsigned char*            pci_config_addr;
  volatile unsigned char*            pci_config_data;
  const pci_config_access_functions* pci_functions;
} pci_config;

extern pci_config BSP_pci_configuration;

extern inline int
pci_read_config_byte(
  unsigned char bus,
  unsigned char slot,
  unsigned char function,
  unsigned char where,
  uint8_t      *val)
{
  return BSP_pci_configuration.pci_functions->read_config_byte(
            bus, slot, function, where, val);
}

extern inline int
pci_read_config_word(
  unsigned char bus,
  unsigned char slot,
  unsigned char function,
  unsigned char where,
  uint16_t     *val)
{
  return BSP_pci_configuration.pci_functions->read_config_word(
             bus, slot, function, where, val);
}

extern inline int
pci_read_config_dword(
  unsigned char bus,
  unsigned char slot,
  unsigned char function,
  unsigned char where,
  uint32_t     *val)
{
  return BSP_pci_configuration.pci_functions->read_config_dword(bus, slot, function, where, val);
}

extern inline int
pci_write_config_byte(
  unsigned char bus, 
  unsigned char slot, 
  unsigned char function,
  unsigned char where, 
  uint8_t       val)
{
  return BSP_pci_configuration.pci_functions->write_config_byte(
             bus, slot, function, where, val);
}

extern inline int
pci_write_config_word(
  unsigned char bus,
  unsigned char slot,
  unsigned char function,
  unsigned char where,
  uint16_t      val)
{
  return BSP_pci_configuration.pci_functions->write_config_word(
              bus, slot, function, where, val);
}

extern inline int
pci_write_config_dword(
  unsigned char bus,
  unsigned char slot,
  unsigned char function,
  unsigned char where,
  uint32_t      val)
{
  return BSP_pci_configuration.pci_functions->write_config_dword(
      bus, slot, function, where, val);
}

/* scan for a specific device */
/* find a particular PCI device
 * (currently, only bus0 is scanned for device/fun0)
 *
 * RETURNS: zero on success, bus/dev/fun in *pbus / *pdev / *pfun
 */
int
pci_find_device(
  unsigned short vendorid,
  unsigned short deviceid,
  int instance,
  int *pbus,
  int *pdev,
  int *pfun
);

/*
 * Return the number of PCI busses in the system
 */
extern unsigned char pci_bus_count();

#ifdef __cplusplus
}
#endif

#endif /* _RTEMS_PCI_H */
