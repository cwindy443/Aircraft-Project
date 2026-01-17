#include <math.h>
#include <stdio.h>

// 1. 定义控制参数和结构体
typedef struct {
  double pitch_stick; // 驾驶杆纵向输入 [-1, 1]
  double roll_stick;  // 驾驶杆横向输入 [-1, 1]
  double yaw_pedal;   // 脚蹬输入 [-1, 1]
} PilotInput;

typedef struct {
  double alpha;      // 迎角 (Angle of Attack)
  double pitch_rate; // 俯仰速率
  double roll_rate;  // 翻滚速率
  double yaw_rate;   // 偏航速率
} AircraftState;

typedef struct {
  double canard_l, canard_r; // 左右鸭翼
  double elevon_l, elevon_r; // 左右襟副翼
  double rudder;             // 方向舵
} ActuatorOutput;

// 2. 核心飞控算法逻辑
void flight_control_law(PilotInput pilot, AircraftState state,
                        ActuatorOutput *output) {

  // --- A. 俯仰控制 (Pitch) & 静不稳定补偿 ---
  // 由于歼-10C是静不稳定的，飞控需要实时产生反向力矩来维持平衡
  // 补偿公式简写为：控制量 = 飞行员意图 + (K * 俯仰速率反馈)
  double pitch_cmd = (pilot.pitch_stick * 25.0) - (state.pitch_rate * 12.5);

  // 鸭翼向上偏转产生正升力，主翼后缘辅助
  output->canard_l = pitch_cmd;
  output->canard_r = pitch_cmd;

  // --- B. 翻滚控制 (Roll) ---
  // 通过左右襟副翼的差动实现
  double roll_cmd = pilot.roll_stick * 20.0;
  double roll_stab = state.roll_rate * 5.0; // 翻滚增稳

  double roll_final = roll_cmd - roll_stab;

  // --- C. 控制面耦合混合 (Mixing) ---
  // 襟副翼同时负责俯仰和翻滚（Elevon = Elevator + Aileron）
  output->elevon_l = pitch_cmd * 0.6 - roll_final;
  output->elevon_r = pitch_cmd * 0.6 + roll_final;

  // --- D. 偏航控制 (Yaw) ---
  double yaw_cmd = (pilot.yaw_pedal * 15.0) - (state.yaw_rate * 8.0);
  output->rudder = yaw_cmd;

  // --- E. 安全限幅 (Hard Limiting) ---
  if (output->canard_l > 30.0)
    output->canard_l = 30.0;
  if (output->canard_l < -50.0)
    output->canard_l = -50.0;
  // (其他舵面限幅省略...)
}

// 3. 主循环模拟
int main() {
  PilotInput my_pilot = {0.8, 0.2, 0.0}; // 模拟飞行员猛下拉杆并稍向右压杆
  AircraftState j10c_sensor = {5.0, 0.2, 0.1,
                               0.0}; // 传感器反馈：迎角5度，已有俯仰速率0.2
  ActuatorOutput servo_cmds;

  printf("--- 歼-10C 飞控系统实时计算启动 ---\n");

  // 在真实飞机中，此函数每秒运行 100-400 次 (100Hz-400Hz)
  // for (int i = 1; i <= 300; i++)
  flight_control_law(my_pilot, j10c_sensor, &servo_cmds);

  printf("鸭翼偏角: %.2f°\n", servo_cmds.canard_l);
  printf("左/右襟副翼: %.2f° / %.2f°\n", servo_cmds.elevon_l,
         servo_cmds.elevon_r);
  printf("方向舵偏角: %.2f°\n", servo_cmds.rudder);

  return 0;
}
