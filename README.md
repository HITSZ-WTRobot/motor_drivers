# motor drivers

!IMPORTANT 本仓库已经停止维护，请移步 https://github.com/HITSZ-WTRobot-Packages/MotorDrivers

本仓库为基于 STM32 HAL 库 + FreeRTOS CMSISv2 的统一电机驱动接口

## 支持的电机类型

- [x] DJI
    - [x] M3508-C620
    - [x] M2006-C610
- [x] TB6612 + 编码器（STM32 定时器）
- [x] VESC 电调 + 各种电机
- [x] 达妙电机

## 使用说明

本仓库的目的是为不同的电机提供一套统一的封装，方便上层调用（没错就是手搓 `vtable` =.=）

### 项目结构

```text
UserCode/
├── bsp/
├── drivers/                # 对应电机的驱动
    ├── DJI.[hc]
    ├── ...
├── libs/
    ├── pid_motor.[hc]      # PID 计算库
├── interface/
    ├── motor_if.[hc]       # 统一电机驱动接口
```

### 基本思路

本驱动库采用 *电机对象*（如 `DJI_t`） 与 *控制对象*（如 `Motor_VelCtrl_t`）分离的设计思路，二者分开初始化和维护。

同一个 *电机对象* 可以对应多个 *控制对象*，用于实现控制切换。
**用户应当仔细维护 *控制对象* 的启用，确保同一时刻只有一个 *控制对象* 在控制 *电机对象***

### 用法

可以通过 submodule 功能将本仓库引用到你自己的项目（需要是 STM32CubeMX 的 CMake 工具链），具体操作为

```shell
git submodule add git@github.com:HITSZ-WTRobot/motor_drivers.git Modules/motor_drivers
```

然后在项目的 CMakeLists.txt 里添加 subdirectory，并将模块添加到项目

```cmake
set(ENABLE_BSP ON) # 启用 bsp
set(ENABLE_LIBS ON) # 启用 lib
set(ENABLE_CONTROLLERS ON) # 启用 controller

set(ENABLE_DJI_DRIVER ON)
set(ENABLE_TB6612_DRIVER OFF)
set(ENABLE_VESC_DRIVER OFF)
set(ENABLE_DM_DRIVER OFF)

add_subdirectory(Modules/motor_drivers/UserCode)

target_link_libraries(motor_drivers PRIVATE stm32cubemx)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE motor_drivers)
```

#### motor_if

1. 参考下一节定义 *电机对象* 并初始化

2. 根据需求定义 速度环控制对象 (`Motor_VelCtrl_t`) 或 位置环控制对象 (`Motor_PosCtrl_t`)

3. 初始化 *控制对象*
    - 位置控制配置

      ```c
      typedef struct
      {
          MotorType_t motor_type; ///< 受控电机类型
          void*            motor; ///< 受控电机
          MotorPID_Config_t velocity_pid; ///< 内环配置
          MotorPID_Config_t position_pid; ///< 外环配置
          uint32_t          pos_vel_freq_ratio; ///< 内外环频率比
      
          float    error_threshold;  ///< 允许的误差范围
          uint32_t settle_count_max; ///< 在误差内多少周期认为就位
      } Motor_PosCtrlConfig_t;
      ```

      调用以下函数初始化
      ```c
      void Motor_PosCtrl_Init(Motor_PosCtrl_t* hctrl, const Motor_PosCtrlConfig_t* config);
      ```

    - 速度控制配置

      ```c
      typedef struct
      {
          MotorType_t motor_type; //< 受控电机类型
          void*            motor; //< 受控电机
          MotorPID_Config_t pid;
      } Motor_VelCtrlConfig_t;
      ```

      调用以下函数初始化
      ```c
      void Motor_VelCtrl_Init(Motor_VelCtrl_t* hctrl, const Motor_VelCtrlConfig_t* config);
      ```

4. 在定时器中断中更新 *控制对象* 和 *电机对象*

    ```c
    void Motor_PosCtrlUpdate(Motor_PosCtrl_t* hctrl);
    void Motor_VelCtrlUpdate(Motor_VelCtrl_t* hctrl);
    ```

#### 各种电机

##### DJI 大疆电机

大疆电机的初始化结构体如下

```c
typedef struct
{
    bool               auto_zero;
    bool               reverse; ///< 是否反转
    DJI_MotorType_t    motor_type;
    CAN_HandleTypeDef* hcan;
    uint8_t            id1;            ///< 电机编号 1~8
    float              reduction_rate; ///< 外接减速比
} DJI_Config_t;
```

其中外接减速比是指在输出轴上额外连接的减速比，大疆电机本身的减速比会在在驱动内部自动处理。

> 大疆电机需要维护 CAN 配置。
>
> 首先初始化 CAN 滤波器配置，如果使用 CAN2，则必须启用 CAN1，且 CAN2 的滤波器编号从 14 起
>
> ```c
> void DJI_CAN_FilterInit(CAN_HandleTypeDef* hcan, uint32_t filter_bank);
> ```
>
> 请使用 `bsp/can_driver` 里的
>
> ```c
> void CAN_RegisterCallback(CAN_HandleTypeDef*        hcan,
>                           const uint32_t            filter_match_index,
>                           CAN_FifoReceiveCallback_t callback)
> ```
>
> 函数来注册 CAN 回调，其中 `hcan` 和 `filter_match_index` 分别为 `DJI_CAN_FilterInit` 的 `hcan` 和 `filter_bank`，
> `callback` 为 `DJI_CAN_BaseReceiveCallback`。
>
> 之后使用
>
> ```c
> HAL_CAN_RegisterCallback(&hcanX, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CAN_Fifo0ReceiveCallback);
> ```
>
> 来将 `bsp/can_driver` 的主回调函数注册为 STM32 CAN 的回调函数。

另外，需要在定时器中断回调结尾发送控制指令，调用

```c
DJI_SendSetIqCommand(&hcanX, IQ_CMD_GROUP_1_4);
DJI_SendSetIqCommand(&hcanX, IQ_CMD_GROUP_5_8);
```

请注意：一条 CAN 线上不要挂载超过 **7** 个大疆电机，挂载 6 个最佳

##### DM 达妙电机

TODO:

##### TB6612 直流有刷电机

正经人不会用这个，不写

##### VESC 电调控制的电机

VESC 具有它自己的上位机程序 vesctool，大部分配置请在上位机完成。驱动需要的配置如下：

```c
typedef struct
{
    bool               auto_zero; ///< 自动重置零点
    CAN_HandleTypeDef* hcan;
    uint8_t            id;         ///< 控制器 id，0xFF 代表广播
    uint8_t            electrodes; ///< 电极数
} VESC_Config_t;
```

注意是 *电极数* 不是电极对数

## 许可协议（License）

本项目自 2025-10-06 起采用 **GNU 通用公共许可证 第3版（GPLv3）** 进行授权。

- 除非另有说明，本项目的所有源代码、文档及其他资源均受 GPLv3 协议约束；
- 任何人在遵守 GPLv3 条款的前提下，可自由复制、修改、分发本项目；
- 本项目不提供任何形式的担保，包括但不限于适销性或特定用途适用性。

在此日期之前的代码可能未受该协议约束，权利归原作者所有。  
完整协议请见项目根目录的 [`LICENSE`](./LICENSE) 文件，或访问 [GNU 官网](https://www.gnu.org/licenses/gpl-3.0.txt)。
