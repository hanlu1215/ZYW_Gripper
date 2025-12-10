#include <Arduino.h>
// 使用 Servo 库来输出微秒级 PWM（用于 RC/ESC 风格的 PWM 驱动）
#include <Servo.h>
#include <SoftwareSerial.h>

// CRSF 串口常量
const uint32_t CRSF_BAUD = 420000;          // CRSF 默认波特率
const uint8_t CRSF_ADDRESS = 0xC8;          // CRSF 接收机地址
const uint8_t CRSF_FRAME_RC_CHANNELS = 0x16;  // RC 通道帧类型
const size_t CRSF_MAX_FRAME_SIZE = 64;      // 保护性上限

// CRSF 通道索引（0 基）
const uint8_t CH5_INDEX = 4;  // 高低切换触发
const uint8_t CH6_INDEX = 5;  // 低: 停止，中: 正转，高: 反转
const uint8_t CH7_INDEX = 6;  // 低/高 两档决定时长

// 新的 PWM 驱动输出引脚（D3），使用微秒脉宽：1000=反转, 1500=停止, 2000=正转
const int PWM_OUT_PIN = 3;
Servo pwmDriver;

// 软件串口用于调试输出（RX=12, TX=13）
SoftwareSerial DebugSerial(12, 13);

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

// CRSF 通道值（微秒），初始化为中立 1500us
int rcChannelUs[16] = {
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
};

// 计算 CRSF CRC8 (poly 0xD5)
uint8_t crc8Crsf(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// 将 RC payload 解码为 16 个通道的微秒值
void decodeCrsfRcChannels(const uint8_t *payload, size_t payloadLen) {
  if (payloadLen < 22) return;  // RC 通道 payload 固定 22 字节

  for (uint8_t ch = 0; ch < 16; ch++) {
    const uint32_t bitStart = ch * 11;
    const uint32_t byteIndex = bitStart / 8;
    const uint8_t bitOffset = bitStart % 8;

    // 保护性取值，避免越界读取
    const uint32_t b0 = payload[byteIndex];
    const uint32_t b1 = (byteIndex + 1 < payloadLen) ? payload[byteIndex + 1] : 0;
    const uint32_t b2 = (byteIndex + 2 < payloadLen) ? payload[byteIndex + 2] : 0;
    uint32_t raw = b0 | (b1 << 8) | (b2 << 16);
    raw = (raw >> bitOffset) & 0x7FF;  // 11 位值 0-2047

    // CRSF 典型范围 172-1811，对应 RC 1000-2000us
    int us = map(raw, 172, 1811, 988, 2012);
    rcChannelUs[ch] = constrain(us, 800, 2200);
  }
}

// 读取串口并解析 CRSF RC 通道帧
bool readCrsfFrame() {
  static uint8_t buffer[CRSF_MAX_FRAME_SIZE];
  static uint8_t idx = 0;
  static uint8_t expectedLength = 0;
  bool updated = false;

  while (Serial.available()) {
    uint8_t b = Serial.read();

    // 寻找帧起始地址
    if (idx == 0) {
      if (b != CRSF_ADDRESS) {
        continue;
      }
      buffer[idx++] = b;
      continue;
    }

    buffer[idx++] = b;

    if (idx == 2) {
      expectedLength = buffer[1];
      if (expectedLength < 2 || expectedLength + 2 > CRSF_MAX_FRAME_SIZE) {
        idx = 0;  // 长度非法，丢弃
      }
      continue;
    }

    // 完整帧：地址(1) + 长度(1) + 长度值字节
    const uint8_t totalNeeded = expectedLength + 2;
    if (expectedLength > 0 && idx == totalNeeded) {
      const uint8_t type = buffer[2];
      const uint8_t crcIndex = totalNeeded - 1;
      const uint8_t computedCrc = crc8Crsf(buffer + 2, expectedLength - 1);  // type+payload
      const uint8_t receivedCrc = buffer[crcIndex];

      if (computedCrc == receivedCrc && type == CRSF_FRAME_RC_CHANNELS) {
        const size_t payloadLen = expectedLength - 2;  // 去掉 type 和 crc
        decodeCrsfRcChannels(buffer + 3, payloadLen);
        updated = true;
      }

      idx = 0;  // 重置准备下一帧
    }

    // 防御性重置
    if (idx >= CRSF_MAX_FRAME_SIZE) {
      idx = 0;
    }
  }

  return updated;
}

// 电机状态枚举
enum MotorState { STOPPED, FORWARD, REVERSE };

// 前置声明
void startMotor(MotorState state);
void stopMotor();

// 设置变量
MotorState currentState = STOPPED;  // 当前电机状态
MotorState desiredDirection = STOPPED;  // 由Ch6决定的期望方向
unsigned long startTime = 0;        // 启动时间
unsigned long timerDuration = 0;    // 转动时长，根据Ch7动态设置
bool isRunning = false;             // 电机是否正在运行
int lastCh5Value = 0;               // 上次Ch5 PWM值，用于检测切换
bool lastCh5High = false;           // 上次Ch5是否为高（true: 高，false: 低）

void setup() {
  // 初始化默认串口用于 CRSF
  Serial.begin(CRSF_BAUD);

  // 初始化软件串口用于调试（仅输出）
  DebugSerial.begin(115200);
  DebugSerial.println("Debug serial started on D12/D13");

  // 初始化 PWM 驱动（采用微秒脉宽，模拟 RC/ESC）
  pwmDriver.attach(PWM_OUT_PIN);
  // 先写入中立脉宽 1500us，确保电机/驱动器为停止状态
  pwmDriver.writeMicroseconds(1500);

  // 初始停止电机
  stopMotor();
}

void loop() {
  // 从 CRSF 串口读取并更新通道值
  readCrsfFrame();

  int ch5Value = rcChannelUs[CH5_INDEX];
  int ch6Value = rcChannelUs[CH6_INDEX];
  int ch7Value = rcChannelUs[CH7_INDEX];

  // Ch5：显著变化才更新记录
  if (abs(ch5Value - lastCh5Value) > 50) {
    lastCh5Value = ch5Value;
    DebugSerial.print("Ch5:"); DebugSerial.println(ch5Value);
  }

  // Ch6 变化检测
  static int lastCh6Value = 0;
  if (abs(ch6Value - lastCh6Value) > 30) {
    lastCh6Value = ch6Value;
    DebugSerial.print("Ch6:"); DebugSerial.println(ch6Value);
  }

  // Ch7 变化检测
  static int lastCh7Value = 0;
  if (abs(ch7Value - lastCh7Value) > 30) {
    lastCh7Value = ch7Value;
    DebugSerial.print("Ch7:"); DebugSerial.println(ch7Value);
  }

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
  // 方向仅内部使用，不输出串口

  // 检测Ch5是否切换（高低之间转换）
  bool currentCh5High = (ch5Value > PWM_CH5_THRESHOLD);
  bool ch5Switched = (currentCh5High != lastCh5High);
  if (ch5Switched) {
    lastCh5High = currentCh5High;
    DebugSerial.println("Ch5 toggled");
  }

  // 控制逻辑
  if (desiredDirection == STOPPED) {
    // 如果Ch6为低，强制停止，无论当前状态
    if (isRunning) {
      stopMotor();
      isRunning = false;
      DebugSerial.println("Ch6 stop, motor forced stop");
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
      // 如果时长大于0，启动转动；否则不启动
      if (timerDuration > 0) {
        startMotor(desiredDirection);
        isRunning = true;
        startTime = millis();
        DebugSerial.print("Start motor dir=");
        DebugSerial.print(desiredDirection == FORWARD ? "F" : "R");
        DebugSerial.print(" duration(ms)=");
        DebugSerial.println(timerDuration);
      }
    }
  }

  // 检查正在运行时的停止条件
  if (isRunning) {
    if (millis() - startTime >= timerDuration) {
      stopMotor();
      isRunning = false;
      DebugSerial.println("Timer done, stop motor");
    }
  }

  // 输出转动状态和时间
  // 状态仅内部使用，不输出串口

  // 最小延迟，确保循环快速
  delay(10);  // 轻微延迟：避免CPU过载，但保持响应快
}

// 启动电机函数
void startMotor(MotorState state) {
  // 使用 RC/ESC 风格 PWM 输出：2000us = 正转，1000us = 反转
  if (state == FORWARD) {
    pwmDriver.writeMicroseconds(2000);
  } else if (state == REVERSE) {
    pwmDriver.writeMicroseconds(1000);
  }
  currentState = state;
}

// 停止电机函数
void stopMotor() {
  // 对 RC/ESC 风格的驱动，1500us 为中立/停止
  pwmDriver.writeMicroseconds(1500);
  currentState = STOPPED;
}