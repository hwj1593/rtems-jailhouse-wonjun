/*
 *  COPYRIGHT (c) 1989-2012.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/printer.h>
#include <rtems/test.h>
#include <rtems/pci.h>
#include <time.h>
#include <sys/time.h>
//#include <pcibios.h>        // PCI test
#include <bsp/irq.h>            // IOAPIC test
#include <errno.h>

#include <bsp.h> /* for device driver prototypes */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
/* forward declarations to avoid warnings */
rtems_task Init(rtems_task_argument argument);
//void BSP_irq_count_dump(FILE *);
const char rtems_test_name[] = "HELLO WORLD";
rtems_printer rtems_test_printer;

typedef unsigned short     u16;
typedef unsigned int       u32;
typedef unsigned long long u64;


struct desc_table_reg {
        u16 limit;
        u64 base;
} __attribute__((packed));

static inline unsigned int jh_inl(unsigned short port)
{
        unsigned int v;
        asm volatile("inl %1,%0" : "=a" (v) : "dN" (port));
        return v;
}

static void init_apic(void)  
{
  unsigned int lapic_reg = READ_MSR_LO(0x1b);     // & 0xFFFFF000);
  
  printf("LAPIC-Addr  0x%x\n", lapic_reg );
//  write_msr(0x1b, 0xfee00c00, 0);               // Qemu machine q35: Unhandled MSR write
//  lapic_reg = READ_MSR_LO(0x1b);
//  printf("LAPIC-Addr2 0x%x\n", lapic_reg );

  lapic_reg = READ_MSR_LO(0x802);
  printf("LAPIC-ID:   0x%x\n", lapic_reg );
  lapic_reg = READ_MSR_LO(0x803);
  printf("LAPIC-VER:  0x%x\n", lapic_reg );
  
  lapic_reg = READ_MSR_LO(0x830);                   // ICR Command 
  printf("ICR:   0x%x\n", lapic_reg );
//write_msr(0x830, 0x0004c003, 0xffffffff);        // self , assert 
}

static uint32_t pci_form_request(uint8_t bus, uint8_t dev, uint8_t func, uint8_t regnum)
{
  u32 address;
  address = ( ((bus << PCI_ADDR_BUS_SHIFT) & PCI_ADDR_BUS_MASK) |
  ((dev << PCI_ADDR_DEV_SHIFT) & PCI_ADDR_DEV_MASK) |
  ((func << PCI_ADDR_FUNC_SHIFT) & PCI_ADDR_FUNC_MASK) |
  (regnum & PCI_ADDR_REGNUM_MASK ) | PCI_ADDR_ENABLE);
   return address;
}

static uint32_t pci_read_data_reg32(uint8_t bus, uint8_t dev, uint8_t func, uint8_t regnum)
{
  uint32_t data_val, addr_val = pci_form_request(bus,dev,func,regnum);
  
  outport_long(PCI_REG_ADDR_PORT, addr_val);
  inport_long(PCI_REG_DATA_PORT, data_val);
  return data_val;
}

static void pci_find_devs(void)
{
  uint32_t bus, devid, stscmd, classrv, type, bar0, bar1, bar2, irq, caps;
  int dev, fun, rc;
 
  printk("Dev/Fn Ven:DevID StatCmd    ClasssRev  BAR0       BAR1       BAR2       IRQ        CAPS       Type\n ");    //   8086:1209 

  for (bus = 0; bus < 0x3; bus++) {
  printk("bus: 0x%x\n ", bus);
    for (dev = 0; dev <  PCI_MAX_DEVICES; dev++) {
      for (fun = 0; fun < PCI_MAX_FUNCTIONS; fun++) {
  pci_read_config_dword(bus, dev, fun, PCI_VENDOR_ID, &devid);
        stscmd  = pci_read_data_reg32(bus, dev, fun, PCI_COMMAND);
        classrv = pci_read_data_reg32(bus, dev, fun, PCI_CLASS_REVISION);
        type    = pci_read_data_reg32(bus, dev, fun, PCI_HEADER_TYPE);
        bar0    = pci_read_data_reg32(bus, dev, fun, PCI_BASE_ADDRESS_0);
        bar1    = pci_read_data_reg32(bus, dev, fun, PCI_BASE_ADDRESS_1);
        bar2    = pci_read_data_reg32(bus, dev, fun, PCI_BASE_ADDRESS_2);
        irq     = pci_read_data_reg32(bus, dev, fun, PCI_INTERRUPT_LINE);
        caps    = pci_read_data_reg32(bus, dev, fun, PCI_CAPABILITY_LIST_POINTER);
        if (devid != PCI_INVALID_VENDORDEVICEID)
      printk("%i/%i   %x:%x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%x\n ",
             dev, fun, devid&0xffff, devid>>16, stscmd, classrv, bar0, bar1, bar2, irq, caps, type);
      }
    }
  }

  rc = pci_find_device(0x8086, 0x1229, 0, (int *) &bus, &dev, &fun);
  printk("pci_find_device(8086:1229): rc=%i dev: %x/%x/%x\n", rc, bus, dev, fun);

//  read BAR0 size: forbidden by Jailhouse filter!
//  pci_write_config_dword(0, 5, 0, PCI_COMMAND, 0xffffffff);
//  pci_read_config_dword(0, 5, 0, PCI_COMMAND, &stscmd);
//  printk("wr stat/cmd ...    = 0x%x\n ", stscmd);
//  pci_write_data_reg32(0, 5, 0, PCI_COMMAND, 0x2900003 );   // restore old
}

//==================================================================================

static void init_ioapic(void)  
{
  printk(" IOAPIC ID   =        0x%x\n", ioapic_read(0));
//ioapic_write(0, 0x8000);                                 // ID, Scratchpad bit15 (not allowed)
  printk(" IOAPIC VER  =        0x%x\n", ioapic_read(1));

  printk(" IOAPIC RED10LO  =    0x%x\n", ioapic_read(IOAPIC_REDIR_IDXLO(10))); // was 0x1803A
  printk(" IOAPIC RED10HI  =    0x%x\n", ioapic_read(IOAPIC_REDIR_IDXHI(10))); // was 0
  printk(" IOAPIC RED11LO  =    0x%x\n", ioapic_read(IOAPIC_REDIR_IDXLO(11)));
  printk(" IOAPIC RED11HI  =    0x%x\n", ioapic_read(IOAPIC_REDIR_IDXHI(11)));

//printk("IRR0..127: %x %x %x %x\n",READ_MSR_LO(0x820),READ_MSR_LO(0x821),READ_MSR_LO(0x822),READ_MSR_LO(0x823));
}


rtems_task Init(
  rtems_task_argument ignored
)
{
  unsigned int *physAddress, *mappedAddress, val;
  volatile int n, rc;


  rtems_print_printer_printf(&rtems_test_printer);
  rtems_test_begin();


  printf( "Hello World\n" );


  init_apic();
  printf( "\nHello World Arg: %u Self-TID: 0x%x\n", (unsigned int)ignored, (unsigned int)rtems_task_self());

  rc = init_paging();
  if (rc != 0) printf( "ERR init_paging()\n");

  pci_find_devs();


  usleep(1000000); 
  printf( "IRQ counts:\n" );
  BSP_irq_count_dump(stdout);

  rtems_test_end();
  exit( 0 );
}


/* file descriptors needed for networking */
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 50


/* NOTICE: the clock driver is explicitly disabled */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_MAXIMUM_TASKS            1010
#define CONFIGURE_USE_DEVFS_AS_BASE_FILESYSTEM

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
