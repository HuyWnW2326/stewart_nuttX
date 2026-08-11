/****************************************************************************
 * boards/arm/stm32/stm32f411e-disco/src/stm32_pwmcapture.c
 *
 * Reads 3 PWM channels sent by PX4 (MicroAir) on TIM1 CH1/CH2/CH3
 * (PA8/PA9/PA10). Uses plain Input Capture Mode (not STM32's dedicated
 * "PWM Input Mode", which only measures one signal per timer) with
 * polarity toggled in the ISR to catch rising and falling edges on the
 * same channel — this is the approach already confirmed for this project.
 *
 * Exposes /dev/pwmcap0, /dev/pwmcap1, /dev/pwmcap2 as ioctl-only char
 * devices, following the same pattern as stm32_steppulse.c's
 * /dev/step0..2.
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"
#include "hardware/stm32_tim.h"

#include "stm32f411e-disco.h"
#include "pwm_capture_ioctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin assignment: TIM1_CH1/CH2/CH3 on PA8/PA9/PA10 (AF1).
 * GPIO_TIM1_CH1IN/CH2IN/CH3IN and STM32_APB2_TIM1_CLKIN are defined in
 * boards/stm32f411e-disco/include/board.h (next to the TIM3 pulse-output
 * block), not here — keep board wiring/clocking centralized in board.h.
 */
  
#define PWMCAP_NCHANNELS  3

#define PWMCAP_TIMER_TICK_HZ   1000000UL
#define PWMCAP_PSC             ((STM32_APB2_TIM1_CLKIN / PWMCAP_TIMER_TICK_HZ) - 1)
#define PWMCAP_ARR              0xFFFF

/* If no new edge arrives within this many ms, report the channel stale
 * (PX4 link lost / signal absent) rather than reporting a frozen value.
 */

#define PWMCAP_TIMEOUT_MS      50

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum pwmcap_state_e
{
  PWMCAP_WAIT_RISE = 0,
  PWMCAP_WAIT_FALL
};

struct pwmcap_chan_s
{
  uint8_t  ccr_offset;        /* STM32_GTIM_CCR1..3_OFFSET for this channel */
  uint8_t  ccmr_shift;        /* bit shift of CCxS/ICxF within its CCMR reg */
  bool     use_ccmr2;         /* CH3 lives in CCMR2, CH1/CH2 in CCMR1       */
  uint8_t  ccer_ccxe_shift;   /* CCxE bit position in CCER                  */
  uint8_t  ccer_ccxp_shift;   /* CCxP bit position in CCER                  */
  uint8_t  dier_ccxie_bit;    /* CCxIE bit position in DIER                 */
  uint8_t  sr_ccxif_bit;      /* CCxIF bit position in SR                   */

  enum pwmcap_state_e state;
  uint16_t last_rise_tick;
  uint32_t pulse_width_us;
  uint32_t period_us;
  bool     valid;
  clock_t  last_edge_systick;
};

struct pwmcap_dev_s
{
  int minor;                  /* 0, 1, or 2 -> indexes g_pwmcap_chan[] */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pwmcap_chan_s g_pwmcap_chan[PWMCAP_NCHANNELS] =
{
  /* CH1: CCMR1 bits [1:0]=CCxS, [3:2]=ICxF low bits ... ; CCER bit0=CC1E,
   * bit1=CC1P; DIER bit1=CC1IE; SR bit1=CC1IF
   */
  {
    .ccr_offset      = STM32_GTIM_CCR1_OFFSET,
    .ccmr_shift      = 0,
    .use_ccmr2       = false,
    .ccer_ccxe_shift = 0,
    .ccer_ccxp_shift = 1,
    .dier_ccxie_bit  = 1,
    .sr_ccxif_bit    = 1,
  },
  /* CH2: CCMR1 bits [9:8]=CCxS; CCER bit4=CC2E, bit5=CC2P; DIER bit2;
   * SR bit2
   */
  {
    .ccr_offset      = STM32_GTIM_CCR2_OFFSET,
    .ccmr_shift       = 8,
    .use_ccmr2       = false,
    .ccer_ccxe_shift = 4,
    .ccer_ccxp_shift = 5,
    .dier_ccxie_bit  = 2,
    .sr_ccxif_bit    = 2,
  },
  /* CH3: CCMR2 bits [1:0]=CCxS; CCER bit8=CC3E, bit9=CC3P; DIER bit3;
   * SR bit3
   */
  {
    .ccr_offset      = STM32_GTIM_CCR3_OFFSET,
    .ccmr_shift      = 0,
    .use_ccmr2       = true,
    .ccer_ccxe_shift = 8,
    .ccer_ccxp_shift = 9,
    .dier_ccxie_bit  = 3,
    .sr_ccxif_bit    = 3,
  },
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  pwmcap_open(FAR struct file *filep);
static int  pwmcap_close(FAR struct file *filep);
static int  pwmcap_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  pwmcap_interrupt(int irq, FAR void *context, FAR void *arg);

static const struct file_operations g_pwmcap_fops =
{
  pwmcap_open,   /* open */
  pwmcap_close,  /* close */
  NULL,          /* read */
  NULL,          /* write */
  NULL,          /* seek */
  pwmcap_ioctl,  /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t pwmcap_getreg(uint32_t offset)
{
  return getreg32(STM32_TIM1_BASE + offset);
}

static inline void pwmcap_putreg(uint32_t offset, uint32_t val)
{
  putreg32(val, STM32_TIM1_BASE + offset);
}

static inline void pwmcap_modreg(uint32_t offset, uint32_t clr, uint32_t set)
{
  modifyreg32(STM32_TIM1_BASE + offset, clr, set);
}

static int pwmcap_open(FAR struct file *filep)
{
  return OK;
}

static int pwmcap_close(FAR struct file *filep)
{
  return OK;
}

static int pwmcap_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct pwmcap_dev_s *dev = inode->i_private;
  FAR struct pwmcap_chan_s *ch;
  irqstate_t flags;

  DEBUGASSERT(dev != NULL && dev->minor < PWMCAP_NCHANNELS);
  ch = &g_pwmcap_chan[dev->minor];

  switch (cmd)
    {
      case PWMCAPIOC_GET:
        {
          FAR struct pwmcap_result_s *res =
              (FAR struct pwmcap_result_s *)((uintptr_t)arg);

          if (res == NULL)
            {
              return -EINVAL;
            }

          flags = enter_critical_section();

          res->pulse_width_us = ch->pulse_width_us;
          res->period_us      = ch->period_us;
          res->valid          = ch->valid;
          res->stale          = ch->valid &&
              ((clock_systime_ticks() - ch->last_edge_systick) >
               MSEC2TICK(PWMCAP_TIMEOUT_MS));

          leave_critical_section(flags);

          return OK;
        }

      case PWMCAPIOC_RESET:
        {
          flags = enter_critical_section();
          ch->valid = false;
          ch->pulse_width_us = 0;
          ch->period_us = 0;
          leave_critical_section(flags);
          return OK;
        }

      default:
        return -ENOTTY;
    }
}

/* Shared handler for TIM1 capture/compare interrupt (covers CH1/2/3) */

static int pwmcap_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t sr = pwmcap_getreg(STM32_GTIM_SR_OFFSET);
  int i;

  for (i = 0; i < PWMCAP_NCHANNELS; i++)
    {
      FAR struct pwmcap_chan_s *ch = &g_pwmcap_chan[i];

      if ((sr & (1 << ch->sr_ccxif_bit)) == 0)
        {
          continue;
        }

      /* Clear the CCxIF flag for this channel (write 0, RC_W0) */

      pwmcap_modreg(STM32_GTIM_SR_OFFSET, (1 << ch->sr_ccxif_bit), 0);

      uint16_t tick = (uint16_t)pwmcap_getreg(ch->ccr_offset);

      if (ch->state == PWMCAP_WAIT_RISE)
        {
          /* This capture is a rising edge: close out the period, then
           * flip polarity to catch the falling edge next.
           */

          if (ch->valid || ch->period_us != 0 || ch->last_rise_tick != 0)
            {
              ch->period_us = (uint16_t)(tick - ch->last_rise_tick);
            }

          ch->last_rise_tick = tick;
          ch->last_edge_systick = clock_systime_ticks();

          pwmcap_modreg(STM32_GTIM_CCER_OFFSET,
                        (1 << ch->ccer_ccxp_shift),
                        (1 << ch->ccer_ccxp_shift));   /* CCxP = 1: falling */

          ch->state = PWMCAP_WAIT_FALL;
        }
      else
        {
          /* Falling edge: pulse width = fall_tick - rise_tick */

          ch->pulse_width_us = (uint16_t)(tick - ch->last_rise_tick);
          ch->valid = true;
          ch->last_edge_systick = clock_systime_ticks();

          pwmcap_modreg(STM32_GTIM_CCER_OFFSET,
                        (1 << ch->ccer_ccxp_shift),
                        0);                            /* CCxP = 0: rising */

          ch->state = PWMCAP_WAIT_RISE;
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwmcapture_initialize
 *
 * Configures TIM1 CH1/CH2/CH3 (PA8/PA9/PA10) as PWM input-capture
 * channels and registers /dev/pwmcap0, /dev/pwmcap1, /dev/pwmcap2.
 * Call once from stm32_bringup.c, alongside stm32_steppulse_initialize()
 * and stm32_sensorbtn_initialize().
 ****************************************************************************/

int stm32_pwmcapture_initialize(void)
{
  static struct pwmcap_dev_s g_dev0 = { .minor = 0 };
  static struct pwmcap_dev_s g_dev1 = { .minor = 1 };
  static struct pwmcap_dev_s g_dev2 = { .minor = 2 };
  int ret;
  int i;

  /* 1. Configure GPIOs as TIM1 alternate function inputs (pin macros
   *    come from board.h)
   */

  stm32_configgpio(GPIO_TIM1_CH1IN);
  stm32_configgpio(GPIO_TIM1_CH2IN);
  stm32_configgpio(GPIO_TIM1_CH3IN);

  /* 2. Enable TIM1 clock */

  modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);

  /* 3. Base timer config: free-running up-counter, 1us tick, max ARR */

  pwmcap_putreg(STM32_GTIM_CR1_OFFSET, 0);
  pwmcap_putreg(STM32_GTIM_PSC_OFFSET, PWMCAP_PSC);
  pwmcap_putreg(STM32_GTIM_ARR_OFFSET, PWMCAP_ARR);

  /* 4. Configure CH1/CH2 in CCMR1: CCxS = 01 (IC1/2 mapped to TI1/2),
   *    ICxF = 0011 (small digital filter, ~8 samples) to reject glitches.
   */

  pwmcap_putreg(STM32_GTIM_CCMR1_OFFSET,
                (1 << (0 + 0)) | (0x3 << (0 + 4)) |   /* CH1: CC1S, IC1F */
                (1 << (8 + 0)) | (0x3 << (8 + 4)));   /* CH2: CC2S, IC2F */

  /* 5. Configure CH3 in CCMR2: CCxS = 01, ICxF = 0011 */

  pwmcap_putreg(STM32_GTIM_CCMR2_OFFSET,
                (1 << (0 + 0)) | (0x3 << (0 + 4)));   /* CH3: CC3S, IC3F */

  /* 6. Enable capture on all 3 channels, all starting on rising edge
   *    (CCxP = 0), and clear any stale flags before enabling interrupts.
   */

  pwmcap_putreg(STM32_GTIM_CCER_OFFSET,
                (1 << g_pwmcap_chan[0].ccer_ccxe_shift) |
                (1 << g_pwmcap_chan[1].ccer_ccxe_shift) |
                (1 << g_pwmcap_chan[2].ccer_ccxe_shift));

  pwmcap_putreg(STM32_GTIM_SR_OFFSET, 0);

  for (i = 0; i < PWMCAP_NCHANNELS; i++)
    {
      g_pwmcap_chan[i].state           = PWMCAP_WAIT_RISE;
      g_pwmcap_chan[i].last_rise_tick  = 0;
      g_pwmcap_chan[i].pulse_width_us  = 0;
      g_pwmcap_chan[i].period_us       = 0;
      g_pwmcap_chan[i].valid           = false;
      g_pwmcap_chan[i].last_edge_systick = 0;
    }

  /* 7. Attach and enable the shared TIM1 capture/compare interrupt */

  ret = irq_attach(STM32_IRQ_TIM1CC, pwmcap_interrupt, NULL);
  if (ret < 0)
    {
      tmrerr("ERROR: irq_attach(TIM1CC) failed: %d\n", ret);
      return ret;
    }

  pwmcap_putreg(STM32_GTIM_DIER_OFFSET,
                (1 << g_pwmcap_chan[0].dier_ccxie_bit) |
                (1 << g_pwmcap_chan[1].dier_ccxie_bit) |
                (1 << g_pwmcap_chan[2].dier_ccxie_bit));

  up_enable_irq(STM32_IRQ_TIM1CC);

  /* 8. Start the counter */

  pwmcap_modreg(STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);

  /* 9. Register the 3 char devices */

  ret = register_driver("/dev/pwmcap0", &g_pwmcap_fops, 0666, &g_dev0);
  if (ret < 0)
    {
      return ret;
    }

  ret = register_driver("/dev/pwmcap1", &g_pwmcap_fops, 0666, &g_dev1);
  if (ret < 0)
    {
      return ret;
    }

  ret = register_driver("/dev/pwmcap2", &g_pwmcap_fops, 0666, &g_dev2);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}