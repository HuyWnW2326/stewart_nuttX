/****************************************************************************
 * common/motor_pos/motor_pos.c  (cap nhat)
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
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

static int32_t motor_pos_decode(int32_t encode_value, int32_t turn)
{
  return encode_value + (((turn & 1) + (encode_value < 0)) << 16);
}

static int32_t motor_pos_control(float pos_des_deg, int32_t pos_cur,
                                  int32_t rev)
{
  float pos_des_pulse;
  float pos_cur_pulse;

  pos_des_pulse = pos_des_deg * MOTOR_POS_GEAR_RATIO * MOTOR_POS_PULSE
                  / 360.0f;

  pos_cur_pulse = (float)pos_cur / MOTOR_POS_ENCODER_RESOLUTION
                  * MOTOR_POS_PULSE + (float)rev * MOTOR_POS_PULSE;

  return (int32_t)(pos_des_pulse - pos_cur_pulse);
}

void motor_pos_init(void)
{
  pthread_mutex_lock(&g_lock);
  memset(g_state, 0, sizeof(g_state));
  pthread_mutex_unlock(&g_lock);
}

void motor_pos_update(int motor_id, int32_t encode_value, int32_t turn,
                       int32_t rev)
{
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

int motor_pos_get_pulses(int motor_id, float pos_des_deg,
                          int32_t *out_pulses)
{
  int32_t encode_value;
  int32_t turn;
  int32_t rev;
  int32_t pos_cur;
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

  *out_pulses = motor_pos_control(pos_des_deg, pos_cur, rev);

  return OK;
}