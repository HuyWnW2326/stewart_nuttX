/****************************************************************************
 * common/motor_pos/motor_pos.c
 *
 * Luu feedback encoder va tinh so xung can di tu moc zero da hieu chuan.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include <nuttx/clock.h>

#include "motor_pos.h"

struct motor_pos_state_s
{
  int32_t encode_value;
  int32_t turn;
  int32_t rev;
  bool    valid;
  clock_t last_update_tick;
};

static struct motor_pos_state_s g_state[MOTOR_POS_COUNT];
static pthread_mutex_t          g_lock = PTHREAD_MUTEX_INITIALIZER;
static int32_t g_zero_pulse[MOTOR_POS_COUNT];
static bool    g_zero_captured[MOTOR_POS_COUNT];


/****************************************************************************
 * Name: motor_id_valid
 *
 * Description:
 *   Guard chung cho tat ca ham public - tranh out-of-bounds access
 *   neu caller truyen sai motor_id.
 ****************************************************************************/

static bool motor_id_valid(int motor_id)
{
  return motor_id >= 0 && motor_id < MOTOR_POS_COUNT;
}

static int32_t motor_pos_decode(int32_t encode_value, int32_t turn)
{
  return encode_value + (((turn & 1) + (encode_value < 0)) << 16);
}


/* Quy doi feedback encoder nhieu vong sang don vi xung motor. */
static int32_t motor_pos_current_pulse(int32_t encode_value, int32_t turn,
                                        int32_t rev)
{
  int32_t pos_cur = motor_pos_decode(encode_value, turn);
  return (int32_t)((float)pos_cur / MOTOR_POS_ENCODER_RESOLUTION
                    * MOTOR_POS_PULSE + (float)rev * MOTOR_POS_PULSE);
}

/****************************************************************************
 * Name: motor_pos_control
 *
 * Description:
 *   Quy doi goc dat sang xung va tru vi tri hien tai da dua ve moc zero:
 *   target = deg * gear_ratio * PPR / 360.
 *
 * Returned Value:
 *   So xung co dau can di; dau duong la tang goc.
 ****************************************************************************/

static int32_t motor_pos_control(float pos_des_deg, int32_t pos_cur_pulse,
                                  int32_t zero_pulse)
{
  float pos_des_pulse;

  pos_des_pulse = pos_des_deg * MOTOR_POS_GEAR_RATIO * MOTOR_POS_PULSE
                  / 360.0f;

  return (int32_t)(pos_des_pulse - ((float)pos_cur_pulse
                    - (float)zero_pulse));
}

void motor_pos_init(void)
{
  pthread_mutex_lock(&g_lock);
  memset(g_state, 0, sizeof(g_state));
  memset(g_zero_pulse, 0, sizeof(g_zero_pulse));
  memset(g_zero_captured, 0, sizeof(g_zero_captured));
  pthread_mutex_unlock(&g_lock);
}

void motor_pos_update(int motor_id, int32_t encode_value, int32_t turn,
                       int32_t rev)
{
  if (!motor_id_valid(motor_id))
    {
      return;
    }

  pthread_mutex_lock(&g_lock);

  g_state[motor_id].encode_value     = encode_value;
  g_state[motor_id].turn             = turn;
  g_state[motor_id].rev              = rev;
  g_state[motor_id].valid            = true;
  g_state[motor_id].last_update_tick = clock_systime_ticks();

  pthread_mutex_unlock(&g_lock);    
}

bool motor_pos_is_fresh(int motor_id, uint32_t max_age_ms)
{
  bool    valid;
  clock_t last_tick;
  clock_t now;

  if (!motor_id_valid(motor_id))
    {
      return false;
    }

  pthread_mutex_lock(&g_lock);
  valid     = g_state[motor_id].valid;
  last_tick = g_state[motor_id].last_update_tick;
  pthread_mutex_unlock(&g_lock);

  if (!valid)
    {
      return false;
    }

  now = clock_systime_ticks();
  return (now - last_tick) < MSEC2TICK(max_age_ms);
}

/****************************************************************************
 * Name: motor_pos_get_pulses
 *
 * Description:
 *   Tinh sai lech xung tu feedback moi nhat va moc zero cua mot truc.
 *
 * Returned Value:
 *   OK khi co du lieu hop le; -EINVAL neu tham so sai; -EAGAIN neu chua
 *   co feedback hoac chua chot moc zero.
 ****************************************************************************/

int motor_pos_get_pulses(int motor_id, float pos_des_deg,
                          int32_t *out_pulses)
{
  int32_t encode_value;
  int32_t turn;
  int32_t rev;
  int32_t zero_pulse;
  int32_t pos_cur_pulse;
  bool    valid;
  bool    captured;

  if (!motor_id_valid(motor_id) || out_pulses == NULL)
    {
      return -EINVAL;
    }

  pthread_mutex_lock(&g_lock);
  encode_value = g_state[motor_id].encode_value;
  turn         = g_state[motor_id].turn;
  rev          = g_state[motor_id].rev;
  valid        = g_state[motor_id].valid;
  zero_pulse   = g_zero_pulse[motor_id];
  captured     = g_zero_captured[motor_id];
  pthread_mutex_unlock(&g_lock);

  if (!valid)
    {
      return -EAGAIN;
    }

  if (!captured)
    {
      /* Chua co moc 0 do (chua chay xong homing cho truc nay) -
       * KHONG duoc tinh pulse voi zero_pulse=0 mac dinh, se sai
       * hoan toan vi tri thuc te. Bao loi de caller (motion_task)
       * tu quyet dinh xu ly (vi du cho homing xong).
       */

      return -EAGAIN;
    }

  pos_cur_pulse = motor_pos_current_pulse(encode_value, turn, rev);

  *out_pulses = motor_pos_control(pos_des_deg, pos_cur_pulse, zero_pulse);

  return OK;
}

bool motor_pos_zero_captured(int motor_id)
{
  bool captured;

  if (!motor_id_valid(motor_id))
    {
      return false;
    }

  pthread_mutex_lock(&g_lock);
  captured = g_zero_captured[motor_id];
  pthread_mutex_unlock(&g_lock);

  return captured;
}

clock_t motor_pos_get_update_tick(int motor_id)
{
  clock_t tick;

  if (!motor_id_valid(motor_id))
    {
      return 0;
    }

  pthread_mutex_lock(&g_lock);
  tick = g_state[motor_id].last_update_tick;
  pthread_mutex_unlock(&g_lock);

  return tick;
}

/****************************************************************************
 * Name: motor_pos_capture_zero
 *
 * Description:
 *   Chot feedback hien tai lam moc limit duoi va cong margin quy doi theo
 *   margin_pulse = margin_deg * gear_ratio * PPR / 360 de tao moc 0 do.
 *
 * Returned Value:
 *   OK khi thanh cong; -EAGAIN neu chua co feedback hop le.
 ****************************************************************************/

int motor_pos_capture_zero(int motor_id, float margin_deg)
{
  int32_t encode_value;
  int32_t turn;
  int32_t rev;
  int32_t pos_cur;
  int32_t margin_pulse;
  bool    valid;

  pthread_mutex_lock(&g_lock);
  encode_value = g_state[motor_id].encode_value;
  turn         = g_state[motor_id].turn;
  rev          = g_state[motor_id].rev;
  valid        = g_state[motor_id].valid;
  pthread_mutex_unlock(&g_lock);

  if (!valid)
    {
      return -EAGAIN;
    }

  pos_cur = motor_pos_decode(encode_value, turn);

  margin_pulse = (int32_t)(margin_deg * MOTOR_POS_GEAR_RATIO
                            * MOTOR_POS_PULSE / 360.0f);

  g_zero_pulse[motor_id] = (int32_t)((float)pos_cur
                            / MOTOR_POS_ENCODER_RESOLUTION * MOTOR_POS_PULSE
                            + (float)rev * MOTOR_POS_PULSE)
                            + margin_pulse;

  g_zero_captured[motor_id] = true;

  return OK;
}
