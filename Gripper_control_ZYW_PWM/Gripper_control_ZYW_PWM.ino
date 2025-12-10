#include <Arduino.h>
// 使用 Servo 库来输出微秒级 PWM（用于 RC/ESC 风格的 PWM 驱动）
#include <Servo.h>

// 定义遥控接收机通道引脚
const int CH5_PIN = 13;  // 高低切换触发
const int CH6_PIN = 12;  //低: 停止，中: 正转，高: 反转）
const int CH7_PIN = 4;   // 仅保留两档: 低(LOW) 和 高(HIGH)，低位对应 CH7_LOW_DURATION_MS

// 原 L298N 电机驱动器引脚（已弃用，保留注释以便参考）
// const int IN1 = 2;     
// const int IN2 = 7;      
// const int ENA = 9;     

// 新的 PWM 驱动输出引脚（D3），使用微秒脉宽：1000=反转, 1500=停止, 2000=正转
const int PWM_OUT_PIN = 3;
Servo pwmDriver;

// PWM阈值（典型RC PWM范围: 1000-2000us）
//ch6和ch7三档
const int PWM_LOW_THRESHOLD = 1200;   // 低于此为低信号
const int PWM_HIGH_THRESHOLD = 1800;  // 高于此为高信号
// Ch5二挡
const int PWM_CH5_THRESHOLD = 1500;   //低于1500为低，高于1500为高

// CH7 两档对应的转动时长（毫秒），在此处统一定义
// 注意：将“中档”重命名为 LOW（低位）以匹配你的命名约定
const unsigned long CH7_LOW_DURATION_MS = 5800UL;   // 低位时长（例如 5.8s）
const unsigned long CH7_HIGH_DURATION_MS = 21000UL; // 高位时长（例如 21s）

// 电机状态枚举
enum MotorState { STOPPED, FORWARD, REVERSE };

// 设置变量
MotorState currentState = STOPPED;  // 当前电机状态
MotorState desiredDirection = STOPPED;  // 由Ch6决定的期望方向
unsigned long startTime = 0;        // 启动时间
unsigned long timerDuration = 0;    // 转动时长，根据Ch7动态设置
bool isRunning = false;             // 电机是否正在运行
int lastCh5Value = 0;               // 上次Ch5 PWM值，用于检测切换
bool lastCh5High = false;           // 上次Ch5是否为高（true: 高，false: 低）
const int DEBOUNCE_TIME = 20;       // 去抖动时间（ms），确保快速响应但稳定

void setup() {
  // 初始化串口用于调试
  Serial.begin(115200);  // 使用更高波特率以提高响应速度

  // 设置输入引脚用于RC通道
  pinMode(CH5_PIN, INPUT);
  pinMode(CH6_PIN, INPUT);
  pinMode(CH7_PIN, INPUT);

  // 设置输出引脚用于L298N
  // L298N 引脚注释（已迁移至 PWM 驱动）
  // pinMode(IN1, OUTPUT);
  // pinMode(IN2, OUTPUT);
  // pinMode(ENA, OUTPUT);

  // 初始化 PWM 驱动（采用微秒脉宽，模拟 RC/ESC）
  pwmDriver.attach(PWM_OUT_PIN);
  // 先写入中立脉宽 1500us，确保电机/驱动器为停止状态
  pwmDriver.writeMicroseconds(1500);

  // 初始停止电机
  stopMotor();
  Serial.println("系统初始化完成，等待遥控信号...");
}

void loop() {
  // 读取通道PWM值
  int ch5Value = pulseIn(CH5_PIN, HIGH, 20000);  // 超时20ms:使用较短超时以提高响应速度
  int ch6Value = pulseIn(CH6_PIN, HIGH, 20000);
  int ch7Value = pulseIn(CH7_PIN, HIGH, 20000);

  // Ch5：仅当值显著变化时更新，并短暂延迟重新读取以确认
  if (abs(ch5Value - lastCh5Value) > 300) {  // 变化超过300us视为潜在切换
    delay(DEBOUNCE_TIME);  // 短暂等待信号稳定
    ch5Value = pulseIn(CH5_PIN, HIGH, 20000);  // 重新读取
    lastCh5Value = ch5Value;
    Serial.print("Ch5信号变化检测: "); Serial.println(ch5Value);
  }

  // Ch6类似处理
  static int lastCh6Value = 0;
  if (abs(ch6Value - lastCh6Value) > 100) {
    delay(DEBOUNCE_TIME);
    ch6Value = pulseIn(CH6_PIN, HIGH, 20000);
    lastCh6Value = ch6Value;
    Serial.print("Ch6信号变化检测: "); Serial.println(ch6Value);
  }

  // Ch7类似处理
  static int lastCh7Value = 0;
  if (abs(ch7Value - lastCh7Value) > 100) {
    delay(DEBOUNCE_TIME);
    ch7Value = pulseIn(CH7_PIN, HIGH, 20000);
    lastCh7Value = ch7Value;
    Serial.print("Ch7信号变化检测: "); Serial.println(ch7Value);
  }

  // 输出实时通道值
  Serial.print("当前Ch5 PWM值: "); Serial.println(ch5Value);
  Serial.print("当前Ch6 PWM值: "); Serial.println(ch6Value);
  Serial.print("当前Ch7 PWM值: "); Serial.println(ch7Value);

  // 确定Ch6方向
  if (ch6Value < PWM_LOW_THRESHOLD && ch6Value > 800) {
    desiredDirection = STOPPED;
  } else if (ch6Value >= PWM_LOW_THRESHOLD && ch6Value <= PWM_HIGH_THRESHOLD) {
    desiredDirection = FORWARD;
  } else if (ch6Value > PWM_HIGH_THRESHOLD && ch6Value < 2200) {
    desiredDirection = REVERSE;
  } else {
    desiredDirection = STOPPED;  // 无效信号默认为停止
  }
  Serial.print("当前Ch6方向: ");
  switch (desiredDirection) {
    case STOPPED: Serial.println("停止"); break;
    case FORWARD: Serial.println("正转"); break;
    case REVERSE: Serial.println("反转"); break;
  }

  // 检测Ch5是否切换（高低之间转换）
  bool currentCh5High = (ch5Value > PWM_CH5_THRESHOLD);
  bool ch5Switched = (currentCh5High != lastCh5High);
  if (ch5Switched) {
    lastCh5High = currentCh5High;
    Serial.println("Ch5切换触发!");
  }

  // 控制逻辑
  if (desiredDirection == STOPPED) {
    // 如果Ch6为低，强制停止，无论当前状态
    if (isRunning) {
      stopMotor();
      isRunning = false;
      Serial.println("因Ch6为停止，电机强制停止");
    }
  } else {
    // Ch6为正转或反转
    if (ch5Switched && !isRunning) {
      // 确定Ch7转动时长（仅支持两档：低/高，阈值与 CH5 相同）
      if (ch7Value > 800 && ch7Value < 2200) {
        // 合法信号范围内，根据与 CH5 相同的阈值判断低/高
        if (ch7Value < PWM_CH5_THRESHOLD) {
          // 低位 -> 使用 CH7_LOW_DURATION_MS
          timerDuration = CH7_LOW_DURATION_MS;
        } else {
          // 高位 -> 使用 CH7_HIGH_DURATION_MS
          timerDuration = CH7_HIGH_DURATION_MS;
        }
      } else {
        // 无效信号（超出范围）默认为0s
        timerDuration = 0;
      }
      Serial.print("设置转动时长: "); Serial.print(timerDuration / 1000.0); Serial.println(" 秒");

      // 如果时长大于0，启动转动；否则不启动
      if (timerDuration > 0) {
        startMotor(desiredDirection);
        isRunning = true;
        startTime = millis();
        Serial.print("启动电机: "); Serial.println(desiredDirection == FORWARD ? "正转" : "反转");
      } else {
        Serial.println("转动时长为0，不启动电机");
      }
    }
  }

  // 检查正在运行时的停止条件
  if (isRunning) {
    if (millis() - startTime >= timerDuration) {
      stopMotor();
      isRunning = false;
      Serial.println("时间到，电机停止");
    }
  }

  // 输出转动状态和时间
  Serial.print("当前转动状态: ");
  switch (currentState) {
    case STOPPED: Serial.println("停止"); break;
    case FORWARD: Serial.println("正转"); break;
    case REVERSE: Serial.println("反转"); break;
  }
  Serial.print("电机是否运行: "); Serial.println(isRunning ? "是" : "否");
  if (isRunning) {
    unsigned long elapsed = millis() - startTime;
    Serial.print("已转动时间: "); Serial.print(elapsed / 1000); Serial.print(" 秒 (剩余: "); Serial.print((timerDuration - elapsed) / 1000); Serial.println(" 秒)");
  } else {
    Serial.println("未在转动");
  }

  // 最小延迟，确保循环快速（~50ms一循环，但因pulseIn可能更长）
  delay(10);  // 轻微延迟：避免CPU过载，但保持响应快
}

// 启动电机函数
void startMotor(MotorState state) {
  // 使用 RC/ESC 风格 PWM 输出：2000us = 正转，1000us = 反转
  if (state == FORWARD) {
    pwmDriver.writeMicroseconds(2000);
    Serial.println("正转指示: PWM 2000us (正转)");
  } else if (state == REVERSE) {
    pwmDriver.writeMicroseconds(1000);
    Serial.println("反转指示: PWM 1000us (反转)");
  }
  currentState = state;
}

// 停止电机函数
void stopMotor() {
  // 对 RC/ESC 风格的驱动，1500us 为中立/停止
  pwmDriver.writeMicroseconds(1500);
  currentState = STOPPED;
  Serial.println("电机停止: PWM 1500us (中立)");
}