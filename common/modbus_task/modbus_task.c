/****************************************************************************
 * common/modbus_task/modbus_task.c
 *
 * Doc feedback vi tri tu ba Modbus slave va cap nhat motor_pos.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sched.h>

#include "modbus/mb.h"
#include "modbus/mb_m.h"
#include "modbus/mbport.h"

#include "motor_pos.h"
#include "modbus_task.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MBMASTER_PORT              1
#define MBMASTER_DEVICE            "/dev/ttyS1"
#define MBMASTER_BAUD              ((speed_t)115200)
#define MBMASTER_PARITY            MB_PAR_EVEN

#define MB_MOTOR_COUNT              3

#define MB_POS_START_ADDRESS        31
#define MB_POS_REGISTER_COUNT       6    /* doc lien tiep thanh ghi 31..36 */

/* Timeout GIANH QUYEN bus, don vi microsecond (xac nhan tu source
 * xMBMasterRunResTake). Khong dung -1 (cho vo han) - luon huu han
 * de phong truong hop bat thuong khong bao gio bi treo vinh vien.
 */
#define MB_BUS_ACQUIRE_TIMEOUT_US   500000UL   /* 500ms */

#define MB_STACK_STARTUP_DELAY_US   20000UL

#define MB_POLLTHREAD_PRIORITY   80
#define MB_REQTHREAD_PRIORITY    80

#define MB_DEBUG_MOTOR_ID    1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mb_input_data_s
{
  int16_t regs[MB_POS_REGISTER_COUNT];   /* raw: reg31..reg36 */
  bool    valid;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_t        g_mb_pollthread;
static volatile bool    g_mb_running;
static bool             g_mb_pollthread_created;

static pthread_t        g_mb_reqthread;
static volatile bool    g_mb_reqthread_running;

static volatile bool    g_input_callback_seen;
static struct mb_input_data_s g_input_data;

/* slave ID 1/2/3 tuong ung motor_id 0/1/2 (khop /dev/step0-2) */
static const uint8_t g_slave_ids[MB_MOTOR_COUNT] = {1, 2, 3};

extern void vMBMasterPortTimerPoll(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mb_pollthread
 *
 * Description:
 *   Dong co chay nen cua FreeModbus - BAT BUOC phai song suot thoi
 *   gian hoat dong, khong lien quan truc tiep den logic doc 3 slave.
 ****************************************************************************/

static FAR void *mb_pollthread(FAR void *arg)
{
  eMBErrorCode error;

  (void)arg;

  printf("[MB] Poll thread started\n");
  fflush(stdout);

  while (g_mb_running)
    {
      error = eMBMasterPoll();

      if (error != MB_ENOERR)
        {
          if (g_mb_running)
            {
              printf("[MB] ERROR: eMBMasterPoll returned %d\n", (int)error);
              fflush(stdout);
            }

          break;
        }

      vMBMasterPortTimerPoll();
    }

  printf("[MB] Poll thread stopped\n");
  fflush(stdout);

  return NULL;
}

/****************************************************************************
 * Name: modbus_engine_initialize
 *
 * Description:
 *   Khoi tao FreeModbus master va tao poll thread nen voi priority thap
 *   hon cac task dieu khien va an toan.
 *
 * Returned Value:
 *   OK khi thanh cong; ma loi am neu khoi tao stack hoac thread that bai.
 ****************************************************************************/

static int modbus_engine_initialize(void)
{
  eMBErrorCode error;
  int ret;
  struct sched_param param;

  printf("[MB] Initializing FreeModbus master (%s, 115200 8E1)\n",
         MBMASTER_DEVICE);
  fflush(stdout);

  error = eMBMasterInit(MB_RTU, MBMASTER_PORT, MBMASTER_BAUD,
                        MBMASTER_PARITY);
  if (error != MB_ENOERR)
    {
      printf("[MB] ERROR: eMBMasterInit failed: %d\n", (int)error);
      fflush(stdout);
      return -ENODEV;
    }

  error = eMBMasterEnable();
  if (error != MB_ENOERR)
    {
      printf("[MB] ERROR: eMBMasterEnable failed: %d\n", (int)error);
      fflush(stdout);
      eMBMasterClose();
      return -ENODEV;
    }

  g_mb_running = true;

  ret = pthread_create(&g_mb_pollthread, NULL, mb_pollthread, NULL);
  if (ret != OK)
    {
      printf("[MB] ERROR: poll thread creation failed: %d\n", ret);
      fflush(stdout);
      g_mb_running = false;
      eMBMasterDisable();
      eMBMasterClose();
      return -ret;
    }

  g_mb_pollthread_created = true;

  /* Poll thread chay thap hon cac task dieu khien motor va an toan. */

  param.sched_priority = MB_POLLTHREAD_PRIORITY;
  ret = pthread_setschedparam(g_mb_pollthread, SCHED_FIFO, &param);
  if (ret != 0)
    {
      printf("[MB] WARNING: setschedparam pollthread failed: %d\n", ret);
      fflush(stdout);
    }

  usleep(MB_STACK_STARTUP_DELAY_US);

  printf("[MB] Engine initialization complete\n");
  fflush(stdout);

  return OK;
}

/****************************************************************************
 * Name: modbus_poll_one_slave
 *
 * Description:
 *   Doc sau input register lien tiep tu dia chi 31. Reg31 la gia tri
 *   encoder, reg32 la bo dem nua vong va reg36 la so vong day du. Chi
 *   goi motor_pos_update() khi doc thanh cong; loi/timeout bi bo qua,
 *   khong retry tai cho vi retry se lam
 *   2 slave con lai bi doi lau hon).
 ****************************************************************************/

static void modbus_poll_one_slave(int motor_id)
{
  uint8_t slave_id = g_slave_ids[motor_id];
  eMBMasterReqErrCode error;

  g_input_callback_seen = false;
  g_input_data.valid = false;

  error = eMBMasterReqReadInputRegister(slave_id, MB_POS_START_ADDRESS,
                                        MB_POS_REGISTER_COUNT,
                                        MB_BUS_ACQUIRE_TIMEOUT_US);

  if (error != MB_MRE_NO_ERR)
  {
    if (motor_id == MB_DEBUG_MOTOR_ID)
      {
        static int dbg_counter = 0;
        if (++dbg_counter % 20 == 0)   /* giam tan suat in, tranh flood UART */
          {
            printf("[MB dbg] motor=%d reg31=%d reg32=%d reg35=%d -> motor_pos_update\n",
                  motor_id, g_input_data.regs[0], g_input_data.regs[1],
                  g_input_data.regs[4]);
            fflush(stdout);
          }
      }
      
    printf("[MB] slave=%u loi/timeout, ma loi=%d\n",
            (unsigned int)slave_id, (int)error);
    fflush(stdout);
    return;
  }

  if (!g_input_callback_seen || !g_input_data.valid)
    {
      printf("[MB] slave=%u khong nhan duoc callback hop le\n",
             (unsigned int)slave_id);
      fflush(stdout);
      return;
    }

  printf("[MB] slave=%u reg31=%d reg32=%d reg33=%d reg34=%d reg35=%d reg36=%d\n",
         (unsigned int)slave_id,
         g_input_data.regs[0], g_input_data.regs[1], g_input_data.regs[2], g_input_data.regs[3], g_input_data.regs[4], g_input_data.regs[5]);
  fflush(stdout);

  motor_pos_update(motor_id,
                   (int32_t)g_input_data.regs[0],
                   (int32_t)g_input_data.regs[1],
                   (int32_t)g_input_data.regs[5]);
}

/****************************************************************************
 * Name: mb_reqthread
 *
 * Description:
 *   modbus_task thuc su - hoi tuan tu 3 slave lien tuc, best-effort,
 *   khong dong bo cung voi motion_task.
 ****************************************************************************/

static FAR void *mb_reqthread(FAR void *arg)
{
  int i;

  (void)arg;

  printf("[MB] Request thread started\n");
  fflush(stdout);

  while (g_mb_reqthread_running)
    {
      for (i = 0; i < MB_MOTOR_COUNT; i++)
        {
          modbus_poll_one_slave(i);
        }
    }

  printf("[MB] Request thread stopped\n");
  fflush(stdout);

  return NULL;
}

/****************************************************************************
 * Name: eMBMasterRegInputCB
 *
 * Description:
 *   Nhan MB_POS_REGISTER_COUNT input register tu FreeModbus theo thu tu
 *   dia chi 31..36 va giai ma moi gia tri big-endian thanh int16_t.
 *
 * Returned Value:
 *   MB_ENOERR khi du lieu hop le; MB_EINVAL hoac MB_ENOREG neu callback
 *   khong cung cap buffer va so thanh ghi can thiet.
 ****************************************************************************/

eMBErrorCode eMBMasterRegInputCB(FAR uint8_t *buffer, uint16_t address,
                                 uint16_t nregs)
{
  uint16_t i;

  if (buffer == NULL)
    {
      return MB_EINVAL;
    }

  if (nregs < MB_POS_REGISTER_COUNT)
    {
      return MB_ENOREG;
    }

  for (i = 0; i < MB_POS_REGISTER_COUNT; i++)
    {
      uint16_t raw = ((uint16_t)buffer[2u * i] << 8) |
                     ((uint16_t)buffer[2u * i + 1u]);

      g_input_data.regs[i] = (int16_t)raw;
    }

  g_input_data.valid = true;
  g_input_callback_seen = true;

  return MB_ENOERR;
}

/****************************************************************************
 * Name: modbus_task_start
 *
 * Description:
 *   Khoi tao Modbus engine va tao request thread doc tuan tu ba slave.
 *
 * Returned Value:
 *   OK khi thanh cong; ma loi am neu khoi tao hoac tao thread that bai.
 ****************************************************************************/

int modbus_task_start(void)
{
  int ret;
  struct sched_param param;

  ret = modbus_engine_initialize();
  if (ret != OK)
    {
      return ret;
    }

  g_mb_reqthread_running = true;

  ret = pthread_create(&g_mb_reqthread, NULL, mb_reqthread, NULL);
  if (ret != OK)
    {
      printf("[MB] ERROR: request thread creation failed: %d\n", ret);
      fflush(stdout);
      g_mb_reqthread_running = false;
      return -ret;
    }

  param.sched_priority = MB_REQTHREAD_PRIORITY;
  ret = pthread_setschedparam(g_mb_reqthread, SCHED_FIFO, &param);
  if (ret != 0)
    {
      printf("[MB] WARNING: setschedparam reqthread failed: %d\n", ret);
      fflush(stdout);
    }

  return OK;
}
