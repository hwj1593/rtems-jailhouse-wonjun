/**
 *  @file
 *
 *  Clock Tick Device Driver
 *
 *  History:
 *    + Original driver was go32 clock by Joel Sherrill
 *    + go32 clock driver hardware code was inserted into new
 *      boilerplate when the pc386 BSP by:
 *        Pedro Miguel Da Cruz Neto Romano <pmcnr@camoes.rnl.ist.utl.pt>
 *        Jose Rufino <ruf@asterix.ist.utl.pt>
 *    + Reworked by Joel Sherrill to use clock driver template.
 *      This removes all boilerplate and leave original hardware
 *      code I developed for the go32 BSP.
 */

/*
 *  COPYRIGHT (c) 1989-2012.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */
#define __JAILHOUSE 1

#include <bsp.h>
#include <bsp/irq-generic.h>
#include <bspopts.h>
#include <libcpu/cpuModel.h>
#include <assert.h>
#include <rtems/timecounter.h>
#include <libcpu/cpu.h>                        /* Jailhouse X2APIC defines, */

#define CLOCK_VECTOR 0

volatile uint32_t pc386_microseconds_per_isr;
volatile uint32_t pc386_isrs_per_tick;
uint32_t pc386_clock_click_count;

/* forward declaration */
void Clock_isr(void *param);
static void clockOff(void);
static void Clock_isr_handler(void *param);
unsigned int rd_pmtmr(void);

/*
 * Roughly the number of cycles per second. Note that these
 * will be wildly inaccurate if the chip speed changes due to power saving
 * or thermal modes.
 *t
 * NOTE: These are only used when the TSC method is used.
 */
//uint64_t pc586_tsc_per_tick;      /* It was removed. what would be aternative for this? */
//uint64_t pc586_tsc_at_tick;       /* It was removed. what would be aternative for this? */

static uint64_t pc586_tsc_frequency;
uint64_t pc586_apictmr_per_tick;      /* Jailhouse: APIC Timers per tick, 32bit */
uint64_t tsc_per_apictmr;             /* Jailhouse: TSC/APIC Timer ratio */

static struct timecounter pc386_tc;

/* this driver may need to count ISRs per tick */
#define CLOCK_DRIVER_ISRS_PER_TICK       1
#define CLOCK_DRIVER_ISRS_PER_TICK_VALUE pc386_isrs_per_tick

extern volatile uint32_t Clock_driver_ticks;

#define READ_8254( _lsb, _msb )                               \
  do { outport_byte(TIMER_MODE, TIMER_SEL0|TIMER_LATCH);      \
     inport_byte(TIMER_CNTR0, _lsb);                          \
     inport_byte(TIMER_CNTR0, _msb);                          \
  } while (0)

/* Copy of Jailhouse Comm-region/PM-Timer port (remapping needed after paging enabled) */
uint16_t jailhs_pmtmr_port = 0;

/* Read PM-Timer using stored port number */
unsigned int rd_pmtmr(void)
{
        unsigned int v;

        asm volatile("inl %1,%0" : "=a" (v) : "dN" (jailhs_pmtmr_port));
        return v;
}

#ifdef RTEMS_SMP
#define Clock_driver_support_at_tick() \
  _SMP_Send_message_broadcast(SMP_MESSAGE_CLOCK_TICK)
#endif

#define Clock_driver_support_install_isr( _new, _old ) \
  do { \
    _old = NULL; \
  } while(0)

static uint32_t pc386_get_timecount_tsc(struct timecounter *tc)
{
  return (uint32_t)rdtsc();
}

static uint32_t pc386_get_timecount_i8254(struct timecounter *tc)
{
  uint32_t                 irqs;
  uint8_t                  lsb, msb;
  rtems_interrupt_level    level;

  /*
   * Fetch all the data in an interrupt critical section.
   */
  rtems_interrupt_disable(level);
    READ_8254(lsb, msb);
    irqs = Clock_driver_ticks;
  rtems_interrupt_enable(level);

  return (irqs + 1) * pc386_microseconds_per_isr - ((msb << 8) | lsb);
}

/*
 * Calibrate CPU cycles per tick. Interrupts should be disabled.
 */
static void calibrate_tsc(void)
{
  uint64_t              begin_time;
  uint8_t               then_lsb, then_msb, now_lsb, now_msb;
  uint32_t              i;

  /*
   * We just reset the timer, so we know we're at the beginning of a tick.
   */

  /*
   * Count cycles. Watching the timer introduces a several microsecond
   * uncertaintity, so let it cook for a while and divide by the number of
   * ticks actually executed.
   */

  begin_time = rdtsc();

  for (i = rtems_clock_get_ticks_per_second() * pc386_isrs_per_tick;
       i != 0; --i ) {
    /* We know we've just completed a tick when timer goes from low to high */
    then_lsb = then_msb = 0xff;
    do {
      READ_8254(now_lsb, now_msb);
      if ((then_msb < now_msb) ||
          ((then_msb == now_msb) && (then_lsb < now_lsb)))
        break;
      then_lsb = now_lsb;
      then_msb = now_msb;
    } while (1);
  }

  pc586_tsc_frequency = rdtsc() - begin_time;

#if 0
  printk( "CPU clock at %u MHz\n", (uint32_t)(pc586_tsc_frequency / 1000000));
#endif
}

/*
 * Calibrate APIC Timer / TSC cycles per tick using PM Timer.
 */
static void calibrate_pmtimer(void)
{
  uint32_t              start_pmt, end_pmt, end_apic, rtems_hz;
  uint64_t              start_tsc, end_tsc;
  int                   overflow;

  start_pmt = rd_pmtmr();                         /* timer stuck check */
  rtems_hz = rtems_clock_get_ticks_per_second() * pc386_isrs_per_tick;
  printk("Calibrate for %d Hz.. ", rtems_hz);

  if (start_pmt == rd_pmtmr()) {                  /* timer not functional */
    printk("PM Timer stuck\n");
    return; 
  }

  do {                                            /* try forever ? */
    overflow  = 0;
    start_pmt = rd_pmtmr();
    start_tsc = rdtsc();
    write_msr(X2APIC_TMICT, 0xffffffff, 0);

    do {
      end_pmt=rd_pmtmr();

      if (end_pmt < start_pmt) {                   /* overflow */
  write_msr(X2APIC_TMICT, 0, 0);             // stop timer
        printk("Overflow! startPMT %u endPMT %u\n", start_pmt, end_pmt);
        overflow = 1;
        break;
      }
    } while ((end_pmt - start_pmt) <  PM_TIMER_HZ / rtems_hz);

  } while (overflow);

  end_tsc  = rdtsc();
  end_apic = READ_MSR_LO(X2APIC_TMCCT);
  write_msr(X2APIC_TMICT, 0, 0);                    // stop timer
  
  pc586_tsc_per_tick     = end_tsc - start_tsc;
  pc586_apictmr_per_tick = 0xffffffff - end_apic;
  tsc_per_apictmr        = pc586_tsc_per_tick/pc586_apictmr_per_tick;
  
  printk("%u PMTimer %u TSC/tick %u TSC/PM\n", (uint32_t)pc586_apictmr_per_tick, (uint32_t) pc586_tsc_per_tick,
                                               (uint32_t) tsc_per_apictmr);
}
static void clockOn(void)
{
  pc386_isrs_per_tick        = 1;
  pc386_microseconds_per_isr = rtems_configuration_get_microseconds_per_tick();

  while (US_TO_TICK(pc386_microseconds_per_isr) > 65535) {
    pc386_isrs_per_tick  *= 10;
    pc386_microseconds_per_isr /= 10;
  }
  pc386_clock_click_count = US_TO_TICK(pc386_microseconds_per_isr);

  //bsp_interrupt_vector_enable( BSP_PERIODIC_TIMER - BSP_IRQ_VECTOR_BASE );

  //#if 0
    printk( "configured usecs per tick=%d \n",
      rtems_configuration_get_microseconds_per_tick() );
    printk( "Microseconds per ISR =%d\n", pc386_microseconds_per_isr );
    printk( "final ISRs per tick=%d\n", pc386_isrs_per_tick );
    printk( "final timer counts=%d\n", pc386_clock_click_count );
 // #endif

   /* save PM-Timer Port from Jailhouse communication region */
  jailhs_pmtmr_port = JAILHOUSE_COMM_REGION->pm_timer_address;
  printk( "CommReg PM-TMR @ 0x%x\n", jailhs_pmtmr_port);

  write_msr(X2APIC_SPIV, 0x1ff, 0);                 // set APIC SW-enable
  write_msr(X2APIC_TDCR, 3, 0);                     // Divide Config, devide by 16
  calibrate_pmtimer();
  calibrate_pmtimer();
  calibrate_pmtimer();

  write_msr(X2APIC_TMICT, 0, 0);                    // stop
  write_msr(X2APIC_LVTT, BSP_PERIODIC_TIMER + BSP_IRQ_VECTOR_BASE, 0); // set vector 0x20
  write_msr(X2APIC_TMICT, pc586_apictmr_per_tick, 0);                  // start timer
  //outport_byte(TIMER_MODE, TIMER_SEL0|TIMER_16BIT|TIMER_RATEGEN);
  //outport_byte(TIMER_CNTR0, pc386_clock_click_count >> 0 & 0xff);
  //outport_byte(TIMER_CNTR0, pc386_clock_click_count >> 8 & 0xff);

  /*
   * Now calibrate cycles per tick. Do this every time we
   * turn the clock on in case the CPU clock speed has changed.
   */
//  if ( x86_has_tsc() )            /* Jailhouse: done already */
//    calibrate_tsc();
}

static void clockOff(void)
{
  /* reset timer mode to standard (BIOS) value */
  //outport_byte(TIMER_MODE, TIMER_SEL0 | TIMER_16BIT | TIMER_RATEGEN);
  //outport_byte(TIMER_CNTR0, 0);
  //outport_byte(TIMER_CNTR0, 0);
  write_msr(X2APIC_TMICT, 0, 0);                    // stop
} /* Clock_exit */

// uint32_t tscdiff_min = 0xffffffff, tscdiff_max = 0;


bool Clock_isr_enabled = false;
static void Clock_isr_handler(void *param)
{
  uint64_t last_tsc, tsc_diff;
  
  last_tsc = pc586_tsc_at_tick;         /* save last TSC  pc586 would be changed*/
  
//  printk("E: %d TSC: %u nssincelast: %u\n", Clock_isr_enabled, pc586_tsc_at_tick, bsp_clock_nanoseconds_since_last_tick_tsc());

  if ( Clock_isr_enabled )
    Clock_isr( param );

  tsc_diff = rdtsc() - (last_tsc + pc586_tsc_per_tick);   /* latency compensation */
  
//  if (tsc_diff < tscdiff_min) tscdiff_min = tsc_diff;
//  if (Clock_driver_ticks>10)   if (tsc_diff > tscdiff_max) tscdiff_max = tsc_diff;

  if (tsc_diff > pc586_tsc_per_tick) tsc_diff = 100000;    /* don't believe */

//if ((Clock_driver_ticks % 2000) == 1990) printk("TSCd %u Min %u Max %u ICT %u\n", (uint32_t)tsc_diff,
//                   tscdiff_min, tscdiff_max, (uint32_t)(pc586_apictmr_per_tick - tsc_diff/tsc_per_apictmr));

  /* restart APIC timer, account for some fixed overhead */
  write_msr(X2APIC_TMICT, pc586_apictmr_per_tick - tsc_diff/tsc_per_apictmr - 868, 0);
  
}

void Clock_driver_install_handler(void)
{
  rtems_status_code status;

  status = rtems_interrupt_handler_install(
    BSP_PERIODIC_TIMER,
    "ckinit",
    RTEMS_INTERRUPT_UNIQUE,
    Clock_isr_handler,
    NULL
  );
  assert(status == RTEMS_SUCCESSFUL);
  clockOn();
}

#define Clock_driver_support_set_interrupt_affinity(online_processors) \
  do { \
    /* FIXME: Is there a way to do this on x86? */ \
    (void) online_processors; \
  } while (0)

void Clock_driver_support_initialize_hardware(void)
{
  bool use_tsc = false;
  bool use_8254 = false;

  #if (CLOCK_DRIVER_USE_TSC == 1)
    use_tsc = true;
  #endif

  #if (CLOCK_DRIVER_USE_8254 == 1)
    use_8254 = true;
  #endif

  if ( !use_tsc && !use_8254 ) {
    if ( x86_has_tsc() ) use_tsc  = true;
    else                 use_8254 = true;
  }

  if ( use_8254 ) {
    /* printk( "Use 8254\n" ); */
    pc386_tc.tc_get_timecount = pc386_get_timecount_i8254;
    pc386_tc.tc_counter_mask = 0xffffffff;
    pc386_tc.tc_frequency = TIMER_TICK;
  } else {
    /* printk( "Use TSC\n" ); */
    pc386_tc.tc_get_timecount = pc386_get_timecount_tsc;
    pc386_tc.tc_counter_mask = 0xffffffff;
    pc386_tc.tc_frequency = pc586_tsc_frequency;
  }

  pc386_tc.tc_quality = RTEMS_TIMECOUNTER_QUALITY_CLOCK_DRIVER;
  rtems_timecounter_install(&pc386_tc);
  Clock_isr_enabled = true;
}
// re-define ?!
#define Clock_driver_support_shutdown_hardware() \
  do { \
    rtems_status_code status; \
    clockOff(); \
    status = rtems_interrupt_handler_remove(  \
      BSP_PERIODIC_TIMER, \
      Clock_isr_handler,  \
      NULL  \
    );  \
    assert(status == RTEMS_SUCCESSFUL); \
  } while (0)

#include "../../../shared/clockdrv_shell.h"
