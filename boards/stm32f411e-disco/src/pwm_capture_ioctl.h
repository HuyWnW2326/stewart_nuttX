/****************************************************************************
 * common/pwm_capture_ioctl.h
 *
 * Dinh nghia ioctl va ket qua do cho /dev/pwmcap0..2.
 ****************************************************************************/

#ifndef __COMMON_PWM_CAPTURE_IOCTL_H
#define __COMMON_PWM_CAPTURE_IOCTL_H

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <stdint.h>
#include <stdbool.h>

/* ioctl commands ***********************************************************/

/* Plain fixed command numbers for this board-specific char device — not
 * routed through NuttX's shared _IOC()/reserved-base scheme, since this
 * driver is local to this board and doesn't need a globally-reserved
 * ioctl base.
 */

#define PWMCAPIOC_GET     0x1001  /* arg: FAR struct pwmcap_result_s * */
#define PWMCAPIOC_RESET   0x1002  /* arg: none - clears valid/stale state */

struct pwmcap_result_s
{
  uint32_t pulse_width_us;  /* last measured high-time, in microseconds   */
  uint32_t period_us;       /* last measured full period, in microseconds
                              * (rising-to-rising); 0 if not yet measured  */
  bool     valid;           /* true if a full pulse has been captured     */
  bool     stale;           /* true if no new edge for > PWMCAP_TIMEOUT_MS */
};

#endif /* __COMMON_PWM_CAPTURE_IOCTL_H */
