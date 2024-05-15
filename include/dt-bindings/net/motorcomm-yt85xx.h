/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _DT_BINDINGS_MOTORCOMM_YT85XX_H
#define _DT_BINDINGS_MOTORCOMM_YT85XX_H

/* PHY LED Modes */

#define YT85XX_LED_2_FORCE_EN	(1 << 8)
#define YT85XX_LED_2_OFF	(0 << 6)
#define YT85XX_LED_2_ON		(1 << 6)
#define YT85XX_LED_2_BLK_2	(2 << 6)
#define YT85XX_LED_2_BLK_1	(3 << 6)
#define YT85XX_LED_1_FORCE_EN	(1 << 5)
#define YT85XX_LED_1_OFF	(0 << 3)
#define YT85XX_LED_1_ON		(1 << 3)
#define YT85XX_LED_1_BLK_2	(2 << 3)
#define YT85XX_LED_1_BLK_1	(3 << 3)
#define YT85XX_LED_0_FORCE_EN	(1 << 2)
#define YT85XX_LED_0_OFF	(0 << 0)
#define YT85XX_LED_0_ON		(1 << 0)
#define YT85XX_LED_0_BLK_2	(2 << 0)
#define YT85XX_LED_0_BLK_1	(3 << 0)

#endif
