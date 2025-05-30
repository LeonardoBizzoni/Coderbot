#ifndef CODERBOT_H
#define CODERBOT_H

#include <pigpio.h>
#include <libcoderbot/include/cbdef.h>
#include <libcoderbot/include/motor.h>
#include <libcoderbot/include/encoder.h>

#define InitialDutyCycle 0.2

// distanza tra le ruote in mm
#define BASELINE_MM 120.0

// in mm/s
#define TargetSpeed 70.0

#define MillimeterFromTicks_Left  0.1306443689
#define MillimeterFromTicks_Right 0.1330557259
#define Duty_CycleFromTicks_Left  0.5469512
#define Duty_CycleFromTicks_Right 0.1771385
#define Kp_Left  0.005
#define Kp_Right 0.005
#define Ki_Left  0.0005
#define Ki_Right 0.0005

fn void cb_stop(void);
fn void cb_encoder_callback_isrA(i32 gpio, i32 level, u32 tick, void *_enc);
fn void cb_encoder_callback_isrB(i32 gpio, i32 level, u32 tick, void *_enc);

#endif
