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

#define RTEMS_BSP_NETWORK_DRIVER_NAME    "fx1"
#define RTEMS_BSP_NETWORK_DRIVER_ATTACH  rtems_fxp_attach


#include <bsp.h> /* for device driver prototypes */
#include <rtems/rtems_bsdnet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
extern int rtems_fxp_attach(struct rtems_bsdnet_ifconfig* , int );

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
// #include "system.h"        // tasks
// #include "tmacros.h"
// #include "pritime.h"
#include <time.h>
#include <sys/time.h>
//#include <pcibios.h>          // PCI test  pcibios.h was removed...
#include <bsp/irq.h>            // IOAPIC test

/* forward declarations to avoid warnings */
rtems_task Init(rtems_task_argument argument);

const char rtems_test_name[] = "HELLO WORLD";
rtems_printer rtems_test_printer;

void BSP_irq_count_dump(FILE *);

typedef unsigned short     u16;
typedef unsigned int       u32;
typedef unsigned long long u64;

// static u32 idt[NUM_IDT_DESC * 4];

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
//==================================================================================
#define NTASKS 1000
static rtems_id init_tsk, task_id[NTASKS];

rtems_task tsk_fun(rtems_task_argument my_num);

static void subtract_em(
  struct timespec *start,
  struct timespec *stop,
  struct timespec *t )
{
  t->tv_sec = 0;
  t->tv_nsec = 0;
  _Timespec_Subtract( start, stop, t );
}

rtems_task tsk_fun(rtems_task_argument my_num)
{
  rtems_event_set out;

//  printf("tsk_fun: %d Self: 0x%x\n", (unsigned int)my_num, (unsigned int)rtems_task_self()); fflush(stdout);

  while (1) {
//  printf( "tsk_fun %d: rx..\n", (int)my_num); fflush(stdout);
    rtems_event_receive(1, RTEMS_WAIT | RTEMS_EVENT_ANY, 0, &out);
//  printf( "tsk_fun %d: gotit, sendto 0x%x\n", (int)my_num, my_num == NTASKS-1 ? init_tsk : task_id[my_num + 1] ); fflush(stdout);
    rtems_event_send(my_num == NTASKS-1 ? init_tsk : task_id[my_num + 1], 1);
  }
}

static void pingpong(void)  
{
  rtems_status_code  result;
  rtems_event_set    out;
  int n, i, loops = 10000;                 // task pingpong loops
  struct timespec start, stop, diff;
  uint64_t swperf;

  init_tsk = rtems_task_self();

  for (i = 0; i < NTASKS; i++) {
    result = rtems_task_create(rtems_build_name('t', '0', '0', '1'),
                               10,
                               RTEMS_MINIMUM_STACK_SIZE,
                               RTEMS_DEFAULT_ATTRIBUTES,
                               RTEMS_LOCAL,
                               &task_id[i]);
    if (result != RTEMS_SUCCESSFUL) printf( "ERR task_create\n" );

    result = rtems_task_start(task_id[i], tsk_fun, (rtems_task_argument) i);
    if (result != RTEMS_SUCCESSFUL) printf( "ERR task_start\n" );
  }
  printf("Pingpong: %d tasks started\n", NTASKS); fflush(stdout);

  for (n = 0; n < 10; n++) {
    rtems_clock_get_uptime( &start );
    for (i = 0; i < loops; i++) {
//    printf( "Parent evsend..\n" ); fflush(stdout);

      rtems_event_send(task_id[0], 1);
      if (result != RTEMS_SUCCESSFUL) printf( "ERR event_send\n");

      rtems_event_receive(1, RTEMS_WAIT | RTEMS_EVENT_ANY, 0, &out);
//    printf( "Parent: got event\n" ); fflush(stdout);
    }
  
    rtems_clock_get_uptime( &stop );
    subtract_em( &start, &stop, &diff );
    swperf = ((NTASKS+1) * loops * 1000000000ULL) / (diff.tv_sec * 1000000000ULL + diff.tv_nsec);

    printf( "sec:nsec %ld:%ld  %ld:%ld -> %ld:%ld %u Sw/sec\n",
      start.tv_sec, start.tv_nsec, stop.tv_sec, stop.tv_nsec,
      diff.tv_sec, diff.tv_nsec, (unsigned)swperf);
  }
  printf( "Pingpong test finished: %d tasks, %d loops\n\n", NTASKS, loops ); 
  usleep(1000000); 
}
//==================================================================================

#define PCI_REG_ADDR_PORT               0xcf8
#define PCI_REG_DATA_PORT               0xcfc
/*----------------- Address register's fields -------------------- Ivan pci-test.h */
/* Bits 31: Enable bit*/
#define PCI_ADDR_ENABLE 0x80000000
/* Bits 23-16: Bus number */
#define PCI_ADDR_BUS_SHIFT 16
#define PCI_ADDR_BUS_MASK (0xFF << PCI_ADDR_BUS_SHIFT)
/* Bits 15-11: Device number */
#define PCI_ADDR_DEV_SHIFT 11
#define PCI_ADDR_DEV_MASK (0x1F << PCI_ADDR_DEV_SHIFT)
/* Bits 10-8: Function number */
#define PCI_ADDR_FUNC_SHIFT 8
#define PCI_ADDR_FUNC_MASK (0x7 << PCI_ADDR_FUNC_SHIFT)
/* Bits 7-0: Register number. Access to registers must be 4-byte aligned  !!! */
#define PCI_ADDR_REGNUM_MASK (0x3F << 2)

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
#if 0
static void pci_write_data_reg32(uint8_t bus, uint8_t dev, uint8_t func, uint8_t regnum, u32 val)
{
  u32 addr_val = pci_form_request(bus,dev,func,regnum);
  
  outport_long(PCI_REG_ADDR_PORT, addr_val);
  outport_long(PCI_REG_DATA_PORT, val);
  return;
}
static void pci_write_data_reg8(uint8_t bus, uint8_t dev, uint8_t func, uint8_t regnum, uint8_t val)
{
  u32 addr_val = pci_form_request(bus,dev,func,regnum);
  
  outport_long(PCI_REG_ADDR_PORT, addr_val);
  outport_byte(PCI_REG_DATA_PORT, val);
  return;
}
#endif

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

static struct rtems_bsdnet_ifconfig netdriver_config = {
  RTEMS_BSP_NETWORK_DRIVER_NAME,
  RTEMS_BSP_NETWORK_DRIVER_ATTACH
};
struct rtems_bsdnet_config rtems_bsdnet_config = {
  &netdriver_config,
  rtems_bsdnet_do_bootp,
};

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

//==================================================================================
static void udp_ping(void)
{
    int s;
    struct sockaddr_in myAddr, farAddr;
    char cbuf[200];
    int i;

        s = socket(AF_INET, SOCK_DGRAM , 0);
        if (s < 0) {
                printf("Can't create client socket: %s\n", strerror(errno));
        return;
    }
    memset(&myAddr, 0, sizeof myAddr);
        myAddr.sin_family = AF_INET;
        myAddr.sin_port = htons(0);
        myAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(s, (struct sockaddr *)&myAddr, sizeof myAddr) < 0) {
                printf("Can't bind socket: %s\n", strerror(errno));
        goto close;
    }
    memset(&farAddr, 0, sizeof farAddr);
        farAddr.sin_family = AF_INET;
        farAddr.sin_port = htons(3344);
        farAddr.sin_addr.s_addr = htonl(0x0a000202);   // Qemu Srv: 10.0.2.2  self: 10 .0 .2 .15

	printf("Connect to server.\n");
        if (connect(s, (struct sockaddr *)&farAddr, sizeof farAddr) < 0) {
                printf("Can't connect to server: %s\n", strerror(errno));
        goto close;
    }


    i = sprintf(cbuf, "HI..............\n");
    i++;    /* Send the '\0', too */
    printf("Write %d-byte message to server.\n", i);
    if (write(s, cbuf, i) < 0) {
                printf("Can't write to server: %s\n", strerror(errno));
        goto close;
    }
    goto close;                               // don't wait for reply

    if ((i = read(s, cbuf, sizeof cbuf)) < 0) {
                printf("Can't read from server: %s\n", strerror(errno));
        goto close;
    }
    printf("Read %d from server: %.*s\n", i, i, cbuf);

    
  close:
    printf("Client closing connection.\n");
    if (close(s) < 0)
        printf("Can't close client task socket: %s\n", strerror(errno));
}

//==================================================================================

rtems_task Init(
  rtems_task_argument ignored
)
{
	unsigned int *physAddress, *mappedAddress, val;
  volatile int n, rc;
  
#if 0
  rtems_time_of_day time; rtems_status_code  status;
  time.year   = 2014;  time.month  = 04; time.day    = 1;  time.hour   = 19;
  time.minute = 00;  time.second = 0;  time.ticks  = 0;
  status = rtems_clock_set( &time );
  if (status != RTEMS_SUCCESSFUL) printf( "ERR clock_set\n" );
  while (1) {
    sleep(10);
    status = rtems_clock_get_tod( &time );
    if (status != RTEMS_SUCCESSFUL) printf( "ERR clock_get_tod\n" );
    printf( "%ld.%ld %ld:%ld.%ldsec\n", time.day , time.month, time.hour, time.minute, time.second);
  }
#endif

  init_apic();
  printf( "\nHello World Arg: %u Self-TID: 0x%x\n", (unsigned int)ignored, (unsigned int)rtems_task_self());

  pingpong();
  
  rc = init_paging();
  if (rc != 0) printf( "ERR init_paging()\n");

  init_ioapic();

  pci_find_devs();
  
  printf( "\nInit network:\n" );
  rc = rtems_bsdnet_initialize_network();
  if (rc != 0) printf( "ERR init_network()\n");

  udp_ping();

  physAddress = (unsigned *) 0x80040000;            // PCI e100 slot5 BAR0 
  n = _CPU_map_phys_address((void **) &mappedAddress, (void *)physAddress, 0x1000, PTE_CACHE_DISABLE);  // 4k
  if (n != 0) printf( "ERR MAPPING PHYS\n" );

  printk("\n mappedAddress = 0x%p\n ", mappedAddress);
  n=_CPU_display_memory_attribute();
  
  printf( "*** END OF HELLO WORLD TEST ***\n" );
  usleep(1000000); 

// Ports 0xb008...0xb00f   PM-timer QEMU
// qemu-vm: IOAPIC: 0xfec00000   HPET: 0xfed00000

  printf( "IRQ counts:\n" );
  BSP_irq_count_dump(stdout);

  rtems_print_printer_printf(&rtems_test_printer);
  rtems_test_begin();
  printf( "Hello World\n" );
  rtems_test_end();

  exit( 0 );
}

/* file descriptors needed for networking */
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 50

/* NOTICE: the clock driver is explicitly disabled */
/* #define CONFIGURE_APPLICATION_DOES_NOT_NEED_CLOCK_DRIVER  enable clockdrv.. CONFIGURE_APPLICATION_NEEDS_TIMER_DRIVER ? */ 
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER


#define CONFIGURE_MAXIMUM_TASKS            (NTASKS + 10)
#define CONFIGURE_USE_DEVFS_AS_BASE_FILESYSTEM

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
