/****************************************************************************
 * boards/arm/stm32/stm32f411e-disco/src/stm32_steppulse.c
 *
 * Step/Direction pulse generator cho 3 truc AC servo, dung TIM3 lam
 * master phat xung PWM lien tuc, cascade qua Internal Trigger toi
 * cac slave timer (TIM2/TIM4/TIM5) de dem xung hoan toan bang phan
 * cung - CPU chi nhan 1 ngat/lan di chuyen thay vi hang tram nghin
 * ngat/giay. Ho tro doi target giua chung khong can doi lenh cu
 * hoan thanh, va khoa chieu theo tung limit switch rieng.
 ****************************************************************************/
#include "chip.h"
#include "hardware/stm32_tim.h"

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "stm32_tim.h"
#include "stm32_gpio.h"
#include <arch/board/board.h>

#include "step_ioctl.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NUM_STEP_CHANNELS  3

/* SMCR khong co bitfield san trong NuttX (stm32_tim.h chi co dia chi
 * base) - tu dinh nghia. Vi tri bit SMS/TS trong SMCR la chuan chung
 * cho moi general-purpose timer STM32 (TIM2-TIM5), khong doi theo chip.
 */
#define STEP_SMCR_SMS_SHIFT     0
#define STEP_SMCR_SMS_EXTCLK1   (7 << STEP_SMCR_SMS_SHIFT)   /* External Clock Mode 1 */

#define STEP_SMCR_TS_SHIFT      4
#define STEP_SMCR_TS_ITR0       (0 << STEP_SMCR_TS_SHIFT)
#define STEP_SMCR_TS_ITR1       (1 << STEP_SMCR_TS_SHIFT)
#define STEP_SMCR_TS_ITR2       (2 << STEP_SMCR_TS_SHIFT)
#define STEP_SMCR_TS_ITR3       (3 << STEP_SMCR_TS_SHIFT)

/* TODO: xac nhan tu RM0383 bang "TIMx internal trigger connection"
 * xem TIM3 TRGO noi vao ITRx nao cua tung slave, roi thay dung gia
 * tri STEP_SMCR_TS_ITRx vao day.
 */
#define TIM3_ITR_ON_TIM2   STEP_SMCR_TS_ITR2   /* TODO verify */
#define TIM3_ITR_ON_TIM4   STEP_SMCR_TS_ITR2   /* TODO verify */
#define TIM3_ITR_ON_TIM5   STEP_SMCR_TS_ITR1   /* TODO verify */


/****************************************************************************
 * Private Types
 ****************************************************************************/

enum step_mode_e
{
  STEP_MODE_IDLE = 0,
  STEP_MODE_HOMING,
  STEP_MODE_TARGETED
};

struct step_channel_s
{
  FAR struct stm32_tim_dev_s *master;        /* luon la TIM3 */
  uint8_t   timer_channel;                    /* 1/2/3 tren TIM3 */
  uint32_t  dir_pin;

  uint32_t  slave_base;                       /* dia chi goc thanh ghi slave timer */
  bool      slave_is_16bit;                   /* true chi voi TIM4 */

  volatile enum step_mode_e mode;
  volatile bool     busy;
  volatile bool     dir_up;
  volatile uint32_t overflow_remaining;       /* chi dung khi slave_is_16bit */
  volatile uint32_t last_cycle_arr;     

  volatile bool     limit_up_active;
  volatile bool     limit_down_active;
};

static struct step_channel_s g_step_ch[NUM_STEP_CHANNELS];
static uint32_t g_tim3_current_period = 0;   /* period dang nap tren TIM3 */

/****************************************************************************
 * Private Functions - thanh ghi slave timer (offset chuan cho moi dong timer)
 ****************************************************************************/

static inline void step_slave_write_cnt(FAR struct step_channel_s *ch, uint32_t v)
{
  putreg32(v, ch->slave_base + STM32_GTIM_CNT_OFFSET);
}

static inline void step_slave_write_arr(FAR struct step_channel_s *ch, uint32_t v)
{
  putreg32(v, ch->slave_base + STM32_GTIM_ARR_OFFSET);
}

static uint32_t tim3_ccxe_bit(uint8_t timer_channel)
{
  switch (timer_channel)
    {
      case 1:  return GTIM_CCER_CC1E;
      case 2:  return GTIM_CCER_CC2E;
      default: return GTIM_CCER_CC3E;
    }
}

static void step_tim3_channel_enable(uint8_t timer_channel, bool enable)
{
  uint32_t bit = tim3_ccxe_bit(timer_channel);
  uint32_t regval = getreg32(STM32_TIM3_CCER);

  if (enable)
    {
      regval |= bit;
    }
  else
    {
      regval &= ~bit;
    }

  putreg32(regval, STM32_TIM3_CCER);
}

/****************************************************************************
 * Private Functions - ISR cho tung slave timer (1 ngat / 1 lan di chuyen)
 ****************************************************************************/

static int step_slave_isr_common(int idx)
{
  FAR struct step_channel_s *ch = &g_step_ch[idx];

  /* Ack ngat Update cua slave timer nay */
  putreg32(0, ch->slave_base + STM32_GTIM_SR_OFFSET);

  if (ch->mode != STEP_MODE_TARGETED)
    {
      /* Dang o mode HOMING - slave timer khong tham gia dinh doan
       * dung o day, bo qua (khong nen xay ra neu init dung).
       */
      return OK;
    }

  if (ch->slave_is_16bit && ch->overflow_remaining > 0)
    {
      ch->overflow_remaining--;
      step_slave_write_cnt(ch, 0);

      if (ch->overflow_remaining == 0)
        {
          /* Vua het cac vong day - chuyen ARR sang vong le (phan du)
           * cho lan dem tiep theo.
           */
          step_slave_write_arr(ch, ch->last_cycle_arr);
        }

      return OK;
    }

  /* Du so xung muc tieu - tat kenh xung tren TIM3 */
  step_tim3_channel_enable(ch->timer_channel, false);
  ch->busy = false;
  ch->mode = STEP_MODE_IDLE;

  return OK;
}

static int step_tim2_isr(int irq, FAR void *context, FAR void *arg)
{
  return step_slave_isr_common(0);
}

static int step_tim4_isr(int irq, FAR void *context, FAR void *arg)
{
  return step_slave_isr_common(1);
}

static int step_tim5_isr(int irq, FAR void *context, FAR void *arg)
{
  return step_slave_isr_common(2);
}

/****************************************************************************
 * Private Functions - ap dung lenh di chuyen (dung chung cho MOVE/HOME)
 ****************************************************************************/

static int step_apply_move(int idx, bool dir_up, uint32_t pulses,
                            uint32_t freq_hz, bool homing)
{
  FAR struct step_channel_s *ch = &g_step_ch[idx];
  irqstate_t flags;
  uint32_t period;

  if (!homing && ((dir_up && ch->limit_up_active) || (!dir_up && ch->limit_down_active)))
    {
      /* Dang cham dung chieu muon di - tu choi ngay, khong dc phep
       * "de tram con dong luc" - phai la nguoi goi tu chinh lenh doi
       * chieu neu muon nha limit.
       */
      return -EACCES;
    }

  flags = enter_critical_section();

  if (ch->busy && ch->dir_up != dir_up)
    {
      /* Doi chieu giua chung - phai tat xung truoc khi doi DIR, driver
       * AC servo can DIR on dinh truoc khi co xung PUL tiep theo.
       */
      step_tim3_channel_enable(ch->timer_channel, false);
    }

  stm32_gpiowrite(ch->dir_pin, !dir_up);
  ch->dir_up = dir_up;

  period = STM32_APB1_TIM3_CLKIN / freq_hz;
  if (period != g_tim3_current_period)
    {
      STM32_TIM_SETPERIOD(ch->master, period);
      g_tim3_current_period = period;
    }

  STM32_TIM_SETCOMPARE(ch->master, ch->timer_channel, period / 2);

  step_slave_write_cnt(ch, 0);

  if (homing)
    {
      /* Khong biet truoc can bao nhieu xung - nap ARR max, viec dung
       * lai hoan toan phu thuoc stm32_steppulse_notify_limit() khi
       * cham cong tac gioi han, khong phu thuoc slave timer.
       */
      ch->mode = STEP_MODE_HOMING;
      ch->overflow_remaining = 0;
      step_slave_write_arr(ch, ch->slave_is_16bit ? 0xFFFF : 0xFFFFFFFF);
    }
  else
    {
      ch->mode = STEP_MODE_TARGETED;

      if (ch->slave_is_16bit && pulses > 0xFFFF)
        {
          uint32_t full_cycles = pulses / 0x10000;
          uint32_t remainder   = pulses % 0x10000;

          if (remainder == 0)
            {
              /* Chia het cho 0x10000 - vong cuoi cung la 1 vong day du,
               * khong can vong le rieng.
               */
              ch->overflow_remaining = full_cycles - 1;
              ch->last_cycle_arr     = 0xFFFF;
            }
          else
            {
              ch->overflow_remaining = full_cycles;
              ch->last_cycle_arr     = remainder - 1;  /* ARR = so xung - 1 */
            }

          /* Vong dau tien luon la vong day (0x10000 xung) neu con vong
           * day phia truoc; neu overflow_remaining=0 ngay tu dau (nghia
           * la chi co 1 vong le) thi nap thang ARR vong le luon.
           */
          if (ch->overflow_remaining > 0)
            {
              step_slave_write_arr(ch, 0xFFFF);
            }
          else
            {
              step_slave_write_arr(ch, ch->last_cycle_arr);
            }
        }
      else
        {
          ch->overflow_remaining = 0;
          step_slave_write_arr(ch, pulses);
        }
    }

  ch->busy = true;
  step_tim3_channel_enable(ch->timer_channel, true);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Private Functions - file ops
 ****************************************************************************/

static int step_open(FAR struct file *filep)
{
  return OK;
}

static int step_close(FAR struct file *filep)
{
  return OK;
}

static int step_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  int idx = (int)(intptr_t)inode->i_private;

  switch (cmd)
    {
      case STEPIOC_MOVE:
        {
          FAR struct step_move_s *mv = (FAR struct step_move_s *)arg;

          if (mv->pulses == 0)
            {
              return OK;
            }

          return step_apply_move(idx, mv->dir_up, mv->pulses,
                                  mv->freq_hz, false);
        }

      case STEPIOC_HOME:
        {
          FAR struct step_home_s *hm = (FAR struct step_home_s *)arg;

          return step_apply_move(idx, hm->dir_up, 0, hm->freq_hz, true);
        }

      case STEPIOC_STATUS:
        {
          FAR bool *is_busy = (FAR bool *)arg;
          *is_busy = g_step_ch[idx].busy;
          return OK;
        }

      case STEPIOC_ESTOP:
        {
          int j;

          for (j = 0; j < NUM_STEP_CHANNELS; j++)
            {
              step_tim3_channel_enable(g_step_ch[j].timer_channel, false);
              g_step_ch[j].busy = false;
              g_step_ch[j].mode = STEP_MODE_IDLE;
            }

          return OK;
        }

      default:
        return -ENOTTY;
    }
}

static const struct file_operations g_step_fops =
{
  step_open,
  step_close,
  NULL,
  NULL,
  NULL,
  step_ioctl,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_steppulse_notify_limit(int motor_id, bool is_up, bool active)
{
  FAR struct step_channel_s *ch = &g_step_ch[motor_id];

  if (is_up)
    {
      ch->limit_up_active = active;
    }
  else
    {
      ch->limit_down_active = active;
    }

  if (active && ch->busy && ch->dir_up == is_up)
    {
      /* Dang chay dung chieu vua cham limit - cat xung NGAY, khong
       * qua task, khong doi ISR nay tra ve roi task moi xu ly.
       */
      step_tim3_channel_enable(ch->timer_channel, false);
      ch->busy = false;
      ch->mode = STEP_MODE_IDLE;
    }
}

int stm32_steppulse_initialize(void)
{
  FAR struct stm32_tim_dev_s *tim3;
  FAR struct stm32_tim_dev_s *tim2;
  FAR struct stm32_tim_dev_s *tim4;
  FAR struct stm32_tim_dev_s *tim5;
  char devname[16];
  int i;
  int ret;

  stm32_configgpio(GPIO_TIM3_CH1OUT);
  stm32_configgpio(GPIO_TIM3_CH2OUT);
  stm32_configgpio(GPIO_TIM3_CH3OUT);

  stm32_configgpio(GPIO_MOTOR1_DIR);
  stm32_configgpio(GPIO_MOTOR2_DIR);
  stm32_configgpio(GPIO_MOTOR3_DIR);

  tim3 = stm32_tim_init(3);
  tim2 = stm32_tim_init(2);
  tim4 = stm32_tim_init(4);
  tim5 = stm32_tim_init(5);

  if (!tim3 || !tim2 || !tim4 || !tim5)
    {
      return -ENODEV;
    }

  /* --- TIM3: master, phat PWM lien tuc 3 kenh --- */

  STM32_TIM_SETPERIOD(tim3, 1000);   /* gia tri hop le tam, MOVE se ghi de */
  STM32_TIM_SETMODE(tim3, STM32_TIM_MODE_UP);

  STM32_TIM_SETCHANNEL(tim3, 1, STM32_TIM_CH_OUTPWM);
  STM32_TIM_SETCHANNEL(tim3, 2, STM32_TIM_CH_OUTPWM);
  STM32_TIM_SETCHANNEL(tim3, 3, STM32_TIM_CH_OUTPWM);

  /* CCxE tat het luc khoi dong - khong kenh nao phat xung cho toi
   * khi co lenh MOVE/HOME dau tien.
   */
  putreg32(getreg32(STM32_TIM3_CCER) &
           ~(GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E),
           STM32_TIM3_CCER);

  /* TRGO = Update Event, de 3 slave timer dung lam nguon dem chung */
  putreg32((getreg32(STM32_TIM3_CR2) & ~GTIM_CR2_MMS_MASK) |
           GTIM_CR2_MMS_UPDATE, STM32_TIM3_CR2);

  STM32_TIM_ENABLE(tim3);

  /* --- TIM2/TIM4/TIM5: slave, dem xung tu TRGO cua TIM3 --- */

  g_step_ch[0].master = tim3;
  g_step_ch[0].timer_channel = 1;
  g_step_ch[0].dir_pin = GPIO_MOTOR1_DIR;
  g_step_ch[0].slave_base = STM32_TIM2_BASE;
  g_step_ch[0].slave_is_16bit = false;

  g_step_ch[1].master = tim3;
  g_step_ch[1].timer_channel = 2;
  g_step_ch[1].dir_pin = GPIO_MOTOR2_DIR;
  g_step_ch[1].slave_base = STM32_TIM4_BASE;
  g_step_ch[1].slave_is_16bit = true;

  g_step_ch[2].master = tim3;
  g_step_ch[2].timer_channel = 3;
  g_step_ch[2].dir_pin = GPIO_MOTOR3_DIR;
  g_step_ch[2].slave_base = STM32_TIM5_BASE;
  g_step_ch[2].slave_is_16bit = false;

  /* TIM2 (kenh 0): SMS = External Clock Mode 1, TS = ITR cua TIM3 */
  putreg32((TIM3_ITR_ON_TIM2) | STEP_SMCR_SMS_EXTCLK1, STM32_TIM2_SMCR);
  putreg32(0, STM32_TIM2_ARR);
  STM32_TIM_ENABLE(tim2);

  putreg32((TIM3_ITR_ON_TIM4) | STEP_SMCR_SMS_EXTCLK1, STM32_TIM4_SMCR);
  putreg32(0, STM32_TIM4_ARR);
  STM32_TIM_ENABLE(tim4);

  putreg32((TIM3_ITR_ON_TIM5) | STEP_SMCR_SMS_EXTCLK1, STM32_TIM5_SMCR);
  putreg32(0, STM32_TIM5_ARR);
  STM32_TIM_ENABLE(tim5);

  ret = STM32_TIM_SETISR(tim2, step_tim2_isr, NULL, 0);
  if (ret < 0) return ret;
  STM32_TIM_ENABLEINT(tim2, GTIM_DIER_UIE);

  ret = STM32_TIM_SETISR(tim4, step_tim4_isr, NULL, 0);
  if (ret < 0) return ret;
  STM32_TIM_ENABLEINT(tim4, GTIM_DIER_UIE);

  ret = STM32_TIM_SETISR(tim5, step_tim5_isr, NULL, 0);
  if (ret < 0) return ret;
  STM32_TIM_ENABLEINT(tim5, GTIM_DIER_UIE);

  for (i = 0; i < NUM_STEP_CHANNELS; i++)
    {
      snprintf(devname, sizeof(devname), "/dev/step%d", i);
      ret = register_driver(devname, &g_step_fops, 0666,
                             (FAR void *)(intptr_t)i);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}