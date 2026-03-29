#ifndef CURVE_MOVE_H
#define CURVE_MOVE_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os2.h"
#include "interfaces/motor_if.h"
#include "math.h"

// 轨迹类型
typedef enum
{
    LINEAR,
    TRAPEZOID,
    S_CURVE
} TrajectoryType;
typedef struct
{
    Motor_PosCtrl_t* pid;
    float targetPos;
    float totalTime;
    TrajectoryType curve;
} TrajectoryParam_t;

// 单个电机运动任务的参数
typedef struct
{
    bool is_active;       // 此任务槽是否正在使用
    Motor_PosCtrl_t* pid; // 电机位置控制器指针
    float startPos;       // 起始位置
    float targetPos;      // 目标位置
    float totalTime;      // 总运动时间 (ms)
    TrajectoryType curve; // 运动曲线类型
    uint32_t currentTime; // 当前已运动时间 (ms)
} MotorMovement_t;


void CurveMove_Init(void);
int Move_Along_Curve(Motor_PosCtrl_t* pos_dji, float targetPos, float time, TrajectoryType curve);

#endif // CURVE_MOVE_H
