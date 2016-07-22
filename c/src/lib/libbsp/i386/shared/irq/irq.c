/*
 *  This file contains the implementation of the function described in irq.h
 */

/*
 *  Copyright (c) 2009 embedded brains GmbH
 *  Copyright (C) 1998 valette@crf.canon.fr
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <bsp.h>
#include <bsp/irq.h>
#include <bsp/irq-generic.h>

#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>


#include "elcr.h"

/*
 * pointer to the mask representing the additionnal irq vectors
 * that must be disabled when a particular entry is activated.
 * They will be dynamically computed from teh prioruty table given
 * in BSP_rtems_irq_mngt_set();
 * CAUTION : this table is accessed directly by interrupt routine
 *       prologue.
 */
static rtems_i8259_masks irq_mask_or_tbl[BSP_IRQ_LINES_NUMBER];

/*
 * Stats of interrupts dispatched.
 */
static uint32_t irq_count[BSP_IRQ_VECTOR_NUMBER] = {0};
static uint32_t spurious_count;

/*
 * Edge or level trigger interrupts.
 */
static enum intr_trigger irq_trigger[BSP_IRQ_LINES_NUMBER];

/*-------------------------------------------------------------------------+
| Cache for 1st and 2nd PIC IRQ line's mssk (enabled or disabled) register.
+--------------------------------------------------------------------------*/
/*
 * lower byte is interrupt mask on the master PIC.
 * while upper bits are interrupt on the slave PIC.
 * This cache is initialized in ldseg.s
 */
static rtems_i8259_masks i8259a_cache = 0xFFFB;

/*
 * Print the stats.
 */
uint32_t BSP_irq_count_dump(FILE *f)
{
  uint32_t tot = 0;
  int      i;
 if ( !f )
   f = stdout;
 fprintf(f,"SPURIOUS: %9"PRIu32"\n", spurious_count);
 for ( i = 0; i < BSP_IRQ_VECTOR_NUMBER; i++ ) {
   char type = '-';
   if (i < BSP_IRQ_LINES_NUMBER)
     type = irq_trigger[i] == INTR_TRIGGER_EDGE ? 'E' : 'L';
   tot += irq_count[i];
   if (irq_count[i])
    fprintf(f,"IRQ %2u: %c %9"PRIu32"\n", i, type, irq_count[i]);
 }
 return tot;
}

/*
 * Is the IRQ valid?
 */
static inline bool BSP_i8259a_irq_valid(const rtems_irq_number irqLine)
{
  return ((int)irqLine >= BSP_IRQ_VECTOR_LOWEST_OFFSET) &&
    ((int)irqLine <= BSP_IRQ_MAX_ON_i8259A);
}

/*
 * Read the IRR register. The default.
 */
static inline uint8_t BSP_i8259a_irq_int_request_reg(uint32_t ioport)
{
  uint8_t isr;
  inport_byte(ioport, isr);
  return isr;
}

/*
 * Read the ISR register. Keep the default of the IRR.
 */
static inline uint8_t BSP_i8259a_irq_in_service_reg(uint32_t ioport)
{
  uint8_t isr;
  outport_byte(ioport, PIC_OCW3_SEL | PIC_OCW3_RR | PIC_OCW3_RIS);
  inport_byte(ioport, isr);
  outport_byte(ioport, PIC_OCW3_SEL | PIC_OCW3_RR);
  return isr;
}

/*-------------------------------------------------------------------------+
|         Function:  BSP_irq_disable_at_i8259a
|      Description: Mask IRQ line in appropriate PIC chip.
| Global Variables: i8259a_cache
|        Arguments: vector_offset - number of IRQ line to mask.
|          Returns: 0 is OK.
+--------------------------------------------------------------------------*/
static int BSP_irq_disable_at_i8259a(const rtems_irq_number irqLine)
{
  unsigned short        mask;
  rtems_interrupt_level level;

  rtems_interrupt_disable(level);

  mask = 1 << irqLine;
  i8259a_cache |= mask;

  if (irqLine < 8)
  {
//    outport_byte(PIC_MASTER_IMR_IO_PORT, i8259a_cache & 0xff);      // Jailhouse
  }
  else
  {
    //    outport_byte(PIC_SLAVE_IMR_IO_PORT, ((i8259a_cache & 0xff00) >> 8));  // Jailhouse
  }

  rtems_interrupt_enable(level);

  return 0;
}

/*-------------------------------------------------------------------------+
|         Function:  BSP_irq_enable_at_i8259a
|      Description: Unmask IRQ line in appropriate PIC chip.
| Global Variables: i8259a_cache
|        Arguments: irqLine - number of IRQ line to mask.
|          Returns: Nothing.
+--------------------------------------------------------------------------*/
static int BSP_irq_enable_at_i8259a(const rtems_irq_number irqLine)
{
  unsigned short        mask;
  rtems_interrupt_level level;
  uint8_t               isr;
  uint8_t               irr;

  rtems_interrupt_disable(level);

  mask = 1 << irqLine;
  i8259a_cache &= ~mask;

  if (irqLine < 8)
  {
    //isr = BSP_i8259a_irq_in_service_reg(PIC_MASTER_COMMAND_IO_PORT);    // Jailhouse
    //irr = BSP_i8259a_irq_int_request_reg(PIC_MASTER_COMMAND_IO_PORT);
    //outport_byte(PIC_MASTER_IMR_IO_PORT, i8259a_cache & 0xff);
  }
  else
  {
    //isr = BSP_i8259a_irq_in_service_reg(PIC_SLAVE_COMMAND_IO_PORT);     // Jailhouse
    //irr = BSP_i8259a_irq_int_request_reg(PIC_SLAVE_COMMAND_IO_PORT);
    //outport_byte(PIC_SLAVE_IMR_IO_PORT, (i8259a_cache >> 8) & 0xff);
  }

  if (((isr ^ irr) & mask) != 0)
    printk("i386: isr=%x irr=%x\n", isr, irr);

  rtems_interrupt_enable(level);

  return 0;
} /* mask_irq */

/*-------------------------------------------------------------------------+
|         Function: BSP_irq_ack_at_i8259a
|      Description: Signal generic End Of Interrupt (EOI) to appropriate PIC.
| Global Variables: None.
|        Arguments: irqLine - number of IRQ line to acknowledge.
|          Returns: Nothing.
+--------------------------------------------------------------------------*/
static int BSP_irq_ack_at_i8259a(const rtems_irq_number irqLine)
{
  uint8_t slave_isr = 0;
  return 0;          // Jailhouse

  if (irqLine >= 8) {
  // outport_byte(PIC_SLAVE_COMMAND_IO_PORT, PIC_EOI);  // Jailhouse: not used
  // slave_isr = BSP_i8259a_irq_in_service_reg(PIC_SLAVE_COMMAND_IO_PORT);
  }

  /*
   * Only issue the EOI to the master if there are no more interrupts in
   * service for the slave. i8259a data sheet page 18, The Special Fully Nested
   * Mode, b.
   */
  //if (slave_isr == 0)
   // outport_byte(PIC_MASTER_COMMAND_IO_PORT, PIC_EOI);

  return 0;

} /* ackIRQ */

/*
 * ------------------------ RTEMS Irq helper functions ----------------
 */

static rtems_irq_prio irqPrioTable[BSP_IRQ_LINES_NUMBER]={
  /*
   * actual priorities for each interrupt source:
   *  0   means that only current interrupt is masked
   *  255 means all other interrupts are masked
   * The second entry has a priority of 255 because
   * it is the slave pic entry and is should always remain
   * unmasked.
   */
  0,0,
  255,
  0, 0, 0, 0,  0,  0,  0,  0,  0,  0,  0,  0
};

static void compute_i8259_masks_from_prio (void)
{
  rtems_interrupt_level level;
  unsigned int i;
  unsigned int j;

  rtems_interrupt_disable(level);

  /*
   * Always mask at least current interrupt to prevent re-entrance
   */
  for (i=0; i < BSP_IRQ_LINES_NUMBER; i++) {
    * ((unsigned short*) &irq_mask_or_tbl[i]) = (1 << i);
    for (j = 0; j < BSP_IRQ_LINES_NUMBER; j++) {
      /*
       * Mask interrupts at i8259 level that have a lower priority
       */
      if (irqPrioTable [i] > irqPrioTable [j]) {
  * ((unsigned short*) &irq_mask_or_tbl[i]) |= (1 << j);
      }
    }
  }

  rtems_interrupt_enable(level);
}

static inline bool bsp_interrupt_vector_is_valid(rtems_vector_number vector)
{
  return BSP_i8259a_irq_valid((const rtems_irq_number) vector);
}

rtems_status_code bsp_interrupt_vector_enable(rtems_vector_number vector)
{
  if (bsp_interrupt_vector_is_valid(vector))
    BSP_irq_enable_at_i8259a(vector);
    
  if (vector == BSP_ETH_FXP_IRQ) {                  /* Jailhouse IOAPIC irq */
    printk("IOAPIC: enable vector %d\n", vector);
    ioapic_write(IOAPIC_REDIR_IDXLO(10), 0x0802a);
    ioapic_write(IOAPIC_REDIR_IDXLO(11), 0x0802a);
  }

  return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_interrupt_vector_disable(rtems_vector_number vector)
{
  if (bsp_interrupt_vector_is_valid(vector))
    BSP_irq_disable_at_i8259a(vector);
  return RTEMS_SUCCESSFUL;
}

rtems_status_code bsp_interrupt_facility_initialize(void)
{
  int i;

  /*
   * set up internal tables used by rtems interrupt prologue
   */
  compute_i8259_masks_from_prio();

  /*
   * must enable slave pic anyway
   */
  BSP_irq_enable_at_i8259a(2);

  /* Jailhouse IOAPIC init for pin 10/11 (unstable for Qemu PCI e100 device) */
  printk("IOAPIC: init irq-pin 10/11\n");         
  ioapic_write(IOAPIC_REDIR_IDXHI(10), 0x03000000);
  ioapic_write(IOAPIC_REDIR_IDXLO(10), 0x1802a);   /* masked, level, vec. 0x2D */
  ioapic_write(IOAPIC_REDIR_IDXHI(11), 0x03000000);
  ioapic_write(IOAPIC_REDIR_IDXLO(11), 0x1802a); 
  
  /*
   * Probe the ELCR.
   */
  elcr_probe();

  for (i = 0; i < BSP_IRQ_LINES_NUMBER; i++)
    irq_trigger[i] = elcr_read_trigger(i);

  return RTEMS_SUCCESSFUL;
}
/* =======================  IOAPIC Jailhouse ================================== */

volatile uint32_t *ioapic_va = NULL;

uint32_t ioapic_read(uint32_t regidx)
{
  volatile uint32_t regval, idx = regidx; /* force register indirect mov instr. */

  if (!ioapic_va) {
    if (_CPU_map_phys_address((void **) &ioapic_va, (void *)IOAPIC_BASE_ADDR, 0x1000, PTE_CACHE_DISABLE)) // 4k
      printk( "Error mapping IOAPIC\n");
  }
  *ioapic_va = idx;                                          /* write index reg. */
  regval     = *(ioapic_va + 4);
  return regval;
}

void ioapic_write(uint32_t regidx, uint32_t regval)
{
  volatile uint32_t idx = regidx, val = regval;

  if (!ioapic_va) {
    printk( "Error IOAPIC not mapped\n");
    return;
  }

  *ioapic_va       = idx;                                    /* write index reg. */
  *(ioapic_va + 4) = val;
}
/* =======================  IOAPIC Jailhouse =================================== */


/*
 * Global so the asm handler can call it.
 */
void BSP_dispatch_isr(int vector);

void BSP_dispatch_isr(int vector)
{
  uint32_t apic_esr;
  uint16_t old_imr = 0;
  
  if (vector != BSP_PERIODIC_TIMER)              /* Jailhouse  */
    printk("IRQ %d\n", vector);

  if (vector < BSP_IRQ_VECTOR_NUMBER) {
    /*
     * Hardware?
     */
    if (vector <= BSP_IRQ_MAX_ON_i8259A) {
      /*
       * See if this is a spurious interrupt.
       */
      if ((vector == 7 || vector == 15)) {
        /*
         * Only check it there no handler for 7 or 15.
         */
        if (bsp_interrupt_handler_is_empty(vector)) {
          /*
           * Read the ISR register to see if IRQ 7/15 is really pending.
           */
          uint8_t isr = BSP_i8259a_irq_in_service_reg(PIC_MASTER_COMMAND_IO_PORT);
          if ((isr & (1 << 7)) == 0) {
            ++spurious_count;
            return;
          }
        }
      }

      /*
       * Save the current cached value for the IMR. It will have the bit for this
       * vector clear.
       */
      if (vector <= BSP_IRQ_MAX_ON_i8259A) {
        old_imr = i8259a_cache;
        i8259a_cache |= irq_mask_or_tbl[vector];
        outport_byte(PIC_MASTER_IMR_IO_PORT, i8259a_cache & 0xff);
        outport_byte(PIC_SLAVE_IMR_IO_PORT, (i8259a_cache >> 8) & 0xff);
      }

      /*
       * Do not use auto-EOI as some slave PIC do not work correctly.
       */
      BSP_irq_ack_at_i8259a(vector);
    }

    /*
     * Count the interrupt.
     */
    irq_count[vector]++;

    /*
     * Allow nesting.
     */
    __asm__ __volatile__("sti");

    bsp_interrupt_handler_dispatch(vector);

    /*
   * Jailhouse: check X2APIC error register
   */
    write_msr(X2APIC_ESR, 0, 0);                   /* write zero before read! */
    apic_esr = READ_MSR_LO(X2APIC_ESR);
    if (apic_esr)
      printk("APIC-ESR 0x%x\n", apic_esr);

  // write_msr(X2APIC_EOI, APIC_EOI_ACK, 0);

    /*
     * Disallow nesting.
     */
    __asm__ __volatile__("cli");

    if (vector <= BSP_IRQ_MAX_ON_i8259A) {
      /*
       * Put the mask back but keep this vector masked if the trigger type is
       * level. The driver or a thread level interrupt server needs to enable it
       * again.
       */
      if (vector <= BSP_IRQ_MAX_ON_i8259A) {
        if (irq_trigger[vector] == INTR_TRIGGER_LEVEL)
          old_imr |= 1 << vector;
        i8259a_cache = old_imr;
        outport_byte(PIC_MASTER_IMR_IO_PORT, i8259a_cache & 0xff);
        outport_byte(PIC_SLAVE_IMR_IO_PORT, (i8259a_cache >> 8) & 0xff);
      }
    }
  }
}
