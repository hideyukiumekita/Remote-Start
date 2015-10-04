/******************************************************************************
 *  app.h (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Definition of the Task main_task
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *  #001 リモートスタート
 *  #002 タッチセンサスタート
 *  #003 尻尾制御
 *****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "ev3api.h"

/*
 *  各タスクの優先度の定義
 */
#define MAIN_PRIORITY    TMIN_APP_TPRI + 1  /* メインタスクの優先度 */
#define BT_PRIORITY      TMIN_APP_TPRI + 2                                      //#001
#define TRACER_PRIORITY  TMIN_APP_TPRI + 3


/*
 *  ターゲットに依存する可能性のある定数の定義
 */
#ifndef STACK_SIZE
#define STACK_SIZE      4096        /* タスクのスタックサイズ */
#endif /* STACK_SIZE */


/* 下記のマクロは個体/環境に合わせて変更する必要があります */                   //#001
#define CMD_START         '1'    /* リモートスタートコマンド */                 //#001
#define TAIL_ANGLE_STAND_UP  88  /* 完全停止時の角度[度] */                     //#003
#define TAIL_ANGLE_DRIVE      3  /* バランス走行時の角度[度] */                 //#003
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例係数 */             //#003
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */        //#003

/*
 *  関数のプロトタイプ宣言
 */
#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void tracer_task(intptr_t exinf);
extern void bt_task(intptr_t exinf);                                            //#001
extern void ev3_cyc_tracer(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif
