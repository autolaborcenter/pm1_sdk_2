# 关于方向盘驱动的笔记

## 接口

### 同步周期

- 同步到方向盘的周期：在每次收到来自方向盘的状态改变事件时执行用户代码
  - 阻塞获取方向盘状态
  - 异步传递机器人状态

- 同步到机器人的周期：在每次收到来自机器人的最新状态反馈时执行用户代码
  - 异步获取方向盘状态
  - 异步传递机器人状态
  - 需要一个独立线程以执行方向盘上位机逻辑

### 接口设计及实现

1. 阻塞获取方向盘状态
   - 状态可表达方向盘不存在
   - 并更新基于方向盘状态的力反馈
   - 并更新补充回中力
2. 传递机器人状态并获取方向盘状态
   - 非阻塞
   - 方向盘不存在返回静止为目标状态
   - 并更新基于机器人状态的力反馈
   - 并更新回中力
3. 轨迹预测
   - TODO

## 力反馈

### 设计

1. 回正
   - 大小只与机器人线速度为线性关系，与方向盘位置无关
   - 方向只取决于方向盘方向
2. 手感标记
   - TODO
3. 阻尼
   - TODO

### 实现

- 内置自动回正
  - 响应快，不需要上位机参与
  - 死区太大，效果不好

  ```c++
  // _event = open(/dev/input/eventX, O_RDWR);
  // value: uint16_t ∈ [0, 0xffff]
  input_event msg{.type = EV_FF, .code = FF_AUTOCENTER, .value = value};
  auto _ = write(_event, &msg, sizeof(input_event));
  ```

- 设置固定力矩
  - TODO
  
  ```c++
  ff_effect effect{
      .type = FF_CONSTANT,
      .id = -1,
      .direction = 0xc000,
      .trigger{},
      .replay{.length = 0xffff, .delay = 0},
      .u{.constant{
          .level = 10000,
          .envelope{
              .attack_length = 0,
              .attack_level = 0,
              .fade_length = 0,
              .fade_level = 0,
          }}},
  };
  
  if (ioctl(event, EVIOCSFF, &effect) < 0) {
      fprintf(stderr, "ERROR: uploading effect failed (%s) [%s:%d]\n", strerror(errno), __FILE__,   __LINE__);
      return 1;
  }
  
  input_event msg{
      .time{},
      .type = EV_FF,
      .code = static_cast<uint16_t>(effect.id),
      .value = 1,
  };
  if (write(event, &msg, sizeof(msg)) != sizeof(msg)) {
      std::cerr << "failed to start effect: " << strerror(errno) << std::endl;
      return 1;
  }
  ```
