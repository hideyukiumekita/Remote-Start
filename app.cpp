/******************************************************************************
 *  app.cpp (for LEGO Mindstorms EV3)
 *  Created on: 2015/01/25
 *  Implementation of the Task main_task
 *  Author: Kazuhiro.Kawachi
 *  Copyright (c) 2015 Embedded Technology Software Design Robot Contest
 *  Revision
 *  #001 Bluetooth対応
 *  #002 タッチセンサ対応
 *  #003 尻尾制御
 *****************************************************************************/

#include "ev3api.h"                                                             //#001
#include "app.h"
#include "LineTracer.h"
#include "Clock.h"                                                              //#001
#include "TouchSensor.h"                                                        //#002

using namespace ev3api;                                                         //#001

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

// using宣言
using ev3api::ColorSensor;
using ev3api::GyroSensor;
using ev3api::Motor;

// Device objects
// オブジェクトを静的に確保する
ColorSensor gColorSensor(PORT_3);
GyroSensor  gGyroSensor(PORT_4);
Motor       gLeftWheel(PORT_C);
Motor       gRightWheel(PORT_B);

// オブジェクトの定義
static LineMonitor     *gLineMonitor;
static Balancer        *gBalancer;
static BalancingWalker *gBalancingWalker;
static LineTracer      *gLineTracer;

// オブジェクトへのポインタ定義
Clock*        clock;                                                            //#001
TouchSensor*  touchSensor;                                                      //#002
Motor*        tailMotor;                                                        //#003

/**
 * EV3システム生成
 */
static void user_system_create() {
    // オブジェクトの作成
    gBalancer        = new Balancer();
    gBalancingWalker = new BalancingWalker(gGyroSensor,
                                           gLeftWheel,
                                           gRightWheel,
                                           gBalancer);
    gLineMonitor     = new LineMonitor(gColorSensor);
    gLineTracer      = new LineTracer(gLineMonitor, gBalancingWalker);
	clock            = new Clock();                                         //#001
	touchSensor      = new TouchSensor(PORT_1);                             //#002
	tailMotor        = new Motor(PORT_A);                                   //#003

    // 初期化完了通知
    ev3_led_set_color(LED_ORANGE);
}

/**
 * EV3システム破棄
 */
static void user_system_destroy() {
    gLeftWheel.reset();
    gRightWheel.reset();

    delete gLineTracer;
    delete gLineMonitor;
    delete gBalancingWalker;
    delete gBalancer;
}

/**
 * tail_control
 */
static void tail_control(int32_t angle)                                         //#003
{                                                                               //#003
	// 比例制御
	float pwm = (float)(angle - tailMotor->getCount()) * P_GAIN;            //#003
	
	// PWM出力飽和処理
	if (pwm > PWM_ABS_MAX)                                                  //#003
	{                                                                       //#003
		pwm = PWM_ABS_MAX;                                              //#003
	}                                                                       //#003
	else if (pwm < -PWM_ABS_MAX)                                            //#003
	{                                                                       //#003
		pwm = -PWM_ABS_MAX;                                             //#003
	}                                                                       //#003
	tailMotor->setPWM(pwm);                                                 //#003
	
}                                                                               //#003

/*                                                                              //#001
 *  Bluetooth                                                                   //#001
 */
 // Bluetoothコマンド 1:リモートスタート
static int32_t   bt_cmd = 0;

// Bluetoothファイルハンドル
static FILE     *bt = NULL;

/*                                                                              //#003
 *  tail                                                                        //#003
 */                                                                             //#003
static void tail_control(int32_t angle);                                        //#003

/**
 * トレース実行タイミング
 */
void ev3_cyc_tracer(intptr_t exinf) {
    act_tsk(TRACER_TASK);
}

/**
 * メインタスク
 */
void main_task(intptr_t unused)
{
	// センサやモータの初期化処理
	user_system_create();
	
	// 尻尾モーターのリセット
	tailMotor->reset();                                                     //#003
	
	// 周期ハンドラ開始
	ev3_sta_cyc(EV3_CYC_TRACER);
	
	// バックボタンが押されるまで待つ
	slp_tsk();
	
	// 周期ハンドラ停止
	ev3_stp_cyc(EV3_CYC_TRACER);
	
	// 終了処理
	user_system_destroy();
	
	ext_tsk();
	
}


/**
 * BTタスク
 */
void bt_task(intptr_t unused)                                                   //#001
{                                                                               //#001
	while(1){                                                               //#001
    	// 受信
        uint8_t c = fgetc(bt);                                                  //#001
        switch(c){                                                              //#001
        case '1':                                                               //#001
            bt_cmd = 1;                                                         //#001
            break;                                                              //#001
        default:                                                                //#001
            break;                                                              //#001
        }                                                                       //#001
        // エコーバック
        fputc(c, bt);                                                           //#001
    }                                                                           //#001
	ext_tsk();                                                              //#001
}                                                                               //#001


/**
 * ライントレースタスク
 */
void tracer_task(intptr_t exinf)
{
	// Open Bluetooth file
	bt = ev3_serial_open_file(EV3_SERIAL_BT);                               //#001
	assert(bt != NULL);                                                     //#001
	
	// Bluetooth通信タスクの起動
	act_tsk(BT_TASK);                                                       //#001
	
	// スタート待機
	while(1)                                                                //#001
	{                                                                       //#001
		// 完全停止用角度に制御
		tail_control(TAIL_ANGLE_STAND_UP);                              //#003
		
		if(bt_cmd == 1)                                                 //#001
		{                                                               //#001
			break;                                                  //#001
		}                                                               //#001
		if(touchSensor->isPressed())                                    //#002
		{                                                               //#002
			break;                                                  //#002
		}                                                               //#002
		clock->sleep(10);                                               //#001
	}                                                                       //#001
	
	while(1)                                                                //#003
	{                                                                       //#003
		if (ev3_button_is_pressed(BACK_BUTTON))
		{
			// バックボタン押下
			wup_tsk(MAIN_TASK);
		}
		// バランス走行用角度に制御
		tail_control(TAIL_ANGLE_DRIVE);                                 //#003
		
		// 倒立走行
		gLineTracer->run();
		
		// 4msec周期起動
		clock->sleep(4);                                                //#003
	}
	ter_tsk(BT_TASK);                                                       //#001
	fclose(bt);                                                             //#001
	ext_tsk();
}
}
