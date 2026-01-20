/**
 * @file    s_curve_traj_follower.h
 * @author  syhanjin
 * @date    2025-12-15
 * @brief   motor trajectory follower using an s-curve.
 */
#ifndef S_CURVE_TRAJ_FOLLOWER_H
#define S_CURVE_TRAJ_FOLLOWER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "libs/s_curve.h"
#include "libs/pid_pd.h"
#include "interfaces/motor_if.h"

/**
 * deg/s 转为 rpm
 * @param __DEG_PER_SEC__ deg/s
 */
#define DPS2RPM(__DEG_PER_SEC__) ((__DEG_PER_SEC__) / 360.0f * 60.0f)

typedef struct
{
    bool running;

    float            update_interval; ///< 更新间隔
    SCurve_t         s;
    PD_t             pd;
    Motor_VelCtrl_t* ctrl;
    float            v_max; ///< 最大速度
    float            a_max; ///< 最大加速度
    float            j_max; ///< 最大加加速度

    float now;

#ifdef DEBUG
    float current_target; ///< 曲线当前目标位置
#endif
} SCurveTrajFollower_Axis_t;

/**
 * 单电机速度曲线规划配置
 *
 * @attention pd 里的 max_output 的单位是 deg/s
 */
typedef struct
{
    float            update_interval; ///< 更新间隔 (s)
    PD_Config_t      error_pd;        ///< 位置误差PD控制器参数
    Motor_VelCtrl_t* motor_vel_ctrl;  ///< 电机控制对象
    float            v_max;           ///< 最大速度
    float            a_max;           ///< 最大加速度
    float            j_max;           ///< 最大加加速度
} SCurveTrajFollower_AxisConfig_t;

typedef struct
{
    Motor_VelCtrl_t* ctrl;
    PD_t             pd;
} SCurveTrajFollower_GroupItem_t;

typedef struct
{
    Motor_VelCtrl_t* ctrl;
    PD_Config_t      error_pd;
} SCurveTrajFollower_GroupItem_Config_t;

typedef struct
{
    bool running;

    float    update_interval; ///< 更新间隔
    SCurve_t s;
    float    v_max; ///< 最大速度
    float    a_max; ///< 最大加速度
    float    j_max; ///< 最大加加速度

    SCurveTrajFollower_GroupItem_t* items;
    size_t                          item_count;

    float now;

#ifdef DEBUG
    float current_target; ///< 曲线当前目标位置
#endif
} SCurveTrajFollower_Group_t;

/**
 * 多电机协同速度曲线规划配置
 *
 * @attention pd 里的 max_output 的单位是 deg/s
 */
typedef struct
{
    float update_interval; ///< 更新间隔 (s)

    SCurveTrajFollower_GroupItem_Config_t* item_configs;
    SCurveTrajFollower_GroupItem_t*        items;
    size_t                                 item_count;

    float v_max; ///< 最大速度
    float a_max; ///< 最大加速度
    float j_max; ///< 最大加加速度
} SCurveTrajFollower_GroupConfig_t;

/**
 * 判断是否执行完毕
 * @param __follower__ follower
 */
#define SCurveTraj_isFinished(__follower__)                                                        \
    ((__follower__)->running && (__follower__)->now >= (__follower__)->s.total_time)

void SCurveTraj_Axis_Init(SCurveTrajFollower_Axis_t*             follower,
                          const SCurveTrajFollower_AxisConfig_t* config);

void SCurveTraj_Axis_Update(SCurveTrajFollower_Axis_t* follower);

void SCurveTraj_Axis_ResetAll(SCurveTrajFollower_Axis_t* follower);
void SCurveTraj_Axis_Stop(SCurveTrajFollower_Axis_t* follower);

SCurve_Result_t SCurveTraj_Axis_SetTarget(SCurveTrajFollower_Axis_t* follower, float target);

float SCurveTraj_Axis_EstimateDuration(const SCurveTrajFollower_Axis_t* follower, float target);

void SCurveTraj_Group_Init(SCurveTrajFollower_Group_t*             follower,
                           const SCurveTrajFollower_GroupConfig_t* config);

void SCurveTraj_Group_Update(SCurveTrajFollower_Group_t* follower);

void SCurveTraj_Group_ResetAll(SCurveTrajFollower_Group_t* follower);
void SCurveTraj_Group_Stop(SCurveTrajFollower_Group_t* follower);

SCurve_Result_t SCurveTraj_Group_SetTarget(SCurveTrajFollower_Group_t* follower, float target);

float SCurveTraj_Group_EstimateDuration(const SCurveTrajFollower_Group_t* follower, float target);

#ifdef __cplusplus
}
#endif

#endif // S_CURVE_TRAJ_FOLLOWER_H
