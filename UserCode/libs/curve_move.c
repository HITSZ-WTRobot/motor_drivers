#include "curve_move.h"

/*
使用说明：
本库实现了多电机的曲线运动控制功能，支持线性、梯形和S型曲线运动。
通过调用 Move_Along_Curve 函数，可以为指定电机分配一个运动任务，
电机会在 CurveMoveManagerTask 任务中按照指定曲线类型和时间完成位置移动。

注意事项：
1. 同一个电机重复调用指派任务时请确保上一个任务已结束；否则新任务会覆盖旧任务。

示例代码：
```c
// 初始化曲线运动模块
CurveMove_Init();
// 为电机分配一个S型曲线运动任务
Move_Along_Curve(&motor1_posCtrl, 1000.0f, 2000.0f, S_CURVE);
// 为另一个电机分配一个梯形曲线运动任务
Move_Along_Curve(&motor2_posCtrl, 500.0f, 1500.0f, TRAPEZOID);
```

曲线类型：
- LINEAR: 线性运动
- TRAPEZOID: 梯形加速运动
- S_CURVE: S型曲线运动

*/

#define MAX_CONCURRENT_MOTORS 10 // 可同时运动的最大电机数
#define MANAGER_TASK_INTERVAL 10 // 管理任务的运行间隔 (ms)


// 电机运动任务池
static MotorMovement_t motor_movements[MAX_CONCURRENT_MOTORS];

// 曲线位置计算函数

float Trajectory_GetPos(uint32_t t, float targetPos, float totalTime, TrajectoryType curve)
{
    float pos = 0.0f;
    switch (curve)
    {
    case LINEAR:
        pos = (targetPos / totalTime) * t;
        break;
    case TRAPEZOID:
        {
            if (totalTime <= 0)
                return 0;
            float accelTime  = totalTime / 4.0f;
            float cruiseTime = totalTime / 2.0f;
            float decelTime  = totalTime / 4.0f;
            float max_vel    = (targetPos * 2.0f) / (totalTime + cruiseTime); // Vmax = 2*S / (t_total + t_cruise)

            if (t < accelTime)
            {
                // 加速段
                float a = max_vel / accelTime;
                pos     = 0.5f * a * t * t;
            }
            else if (t < (accelTime + cruiseTime))
            {
                // 匀速段
                float accelDist = max_vel * accelTime / 2.0f;
                pos             = accelDist + max_vel * (t - accelTime);
            }
            else if (t <= totalTime)
            {
                // 减速段
                float accelDist  = max_vel * accelTime / 2.0f;
                float cruiseDist = max_vel * cruiseTime;
                float dt         = t - (accelTime + cruiseTime);
                float a          = max_vel / decelTime;
                pos              = accelDist + cruiseDist + max_vel * dt - 0.5f * a * dt * dt;
            }
            else
            {
                pos = targetPos;
            }
            break;
        }
    case S_CURVE:
        {
            if (totalTime <= 0)
                return 0;
            // 确保 t 不超过 totalTime，避免 sin 函数参数问题
            if (t > totalTime)
                t = totalTime;
            pos = targetPos * ((float)t / totalTime - (1.0f / (2.0f * M_PI)) * sin(2.0f * M_PI * t / totalTime));
            break;
        }
    default:
        pos = 0.0f;
        break;
    }
    return pos;
}


// 运动管理任务
void CurveMoveManagerTask(void* argument)
{
    for (;;)
    {
        for (int i = 0; i < MAX_CONCURRENT_MOTORS; i++)
        {
            if (motor_movements[i].is_active)
            {
                MotorMovement_t* move = &motor_movements[i];

                if (move->currentTime < move->totalTime)
                {
                    // 计算当前时刻的参考位置
                    float pos_ref = Trajectory_GetPos(move->currentTime, move->targetPos - move->startPos,
                                                      move->totalTime, move->curve);
                    // 更新电机目标位置
                    Motor_PosCtrl_SetRef(move->pid, move->startPos + pos_ref);
                    move->currentTime += MANAGER_TASK_INTERVAL;
                }
                else
                {
                    // 运动结束
                    Motor_PosCtrl_SetRef(move->pid, move->targetPos); // 确保到达最终位置
                    move->is_active = false;                          // 释放任务槽
                }
            }
        }
        osDelay(MANAGER_TASK_INTERVAL);
    }
}

// 初始化函数，创建并启动管理任务
void CurveMove_Init(void)
{
    // 清空任务池
    memset(motor_movements, 0, sizeof(motor_movements));

    // 定义任务属性
    const osThreadAttr_t managerTaskAttr = {
        .name       = "CurveMoveManager",
        .priority   = osPriorityNormal,
        .stack_size = 1024 // 栈大小可能需要根据实际情况调整
    };
    // 创建唯一的管理任务
    osThreadNew(CurveMoveManagerTask, NULL, &managerTaskAttr);
}

// 将电机运动请求添加/更新到任务池
int Move_Along_Curve(Motor_PosCtrl_t* pid, float targetPos, float time, TrajectoryType curve)
{
    int task_index = -1;
    int free_index = -1;

    // 寻找是否已有该电机的任务，或者寻找一个空闲槽
    for (int i = 0; i < MAX_CONCURRENT_MOTORS; i++)
    {
        if (motor_movements[i].is_active && motor_movements[i].pid == pid)
        {
            task_index = i; // 找到该电机的现有任务，将进行覆盖更新
            break;
        }
        if (!motor_movements[i].is_active && free_index == -1)
        {
            free_index = i; // 找到第一个空闲槽
        }
    }

    if (task_index == -1)
    {
        task_index = free_index; // 没有现有任务，使用找到的空闲槽
    }

    if (task_index == -1)
    {
        // 任务池已满，且没有可覆盖的旧任务
        return -1;
    }

    // 填充或更新任务参数
    MotorMovement_t* move = &motor_movements[task_index];
    move->pid             = pid;
    move->startPos        = pid->position_pid.fdb; // 使用当前反馈位置作为起点
    move->targetPos       = targetPos;
    move->totalTime       = time;
    move->curve           = curve;
    move->currentTime     = 0;
    move->is_active       = true; // 激活任务

    return task_index;
}
