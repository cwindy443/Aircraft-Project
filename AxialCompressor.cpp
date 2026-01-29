#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// Struct to hold gas properties
struct GasProperties {
  double k; // Heat capacity ratio
  double R; // Specific gas constant (J/kg·K)
};

// Struct to hold inlet conditions
struct InletConditions {
  double T; // Temperature (K)
  double p; // Pressure (Pa)
  double c; // Velocity (m/s)
};

// Struct to hold efficiency parameters
struct EfficiencyParams {
  double ad;   // Adiabatic efficiency
  double mech; // Mechanical efficiency
  double leak; // Leakage efficiency
};

// Struct to hold geometry parameters
struct GeometryParams {
  // Can be extended with blade geometry details
};

// Struct to hold single stage results
struct StageResult {
  int stage;
  double inlet_T;
  double outlet_T;
  double inlet_p;
  double outlet_p;
  double inlet_velocity;
  double outlet_velocity;
  double density;
  double power;
  double reaction;
  double pressure_ratio;
  double temperature_rise;
};

// Struct to hold overall calculation results
struct CompressorResults {
  int stages;
  double total_power;
  double total_efficiency;
  double outlet_T;
  double outlet_p;
  double outlet_velocity;
  std::vector<StageResult> stage_results;
};

class AxialCompressor {
private:
  GasProperties gas;
  InletConditions inlet;
  double outlet_pressure;
  double mass_flow;
  int stages;
  EfficiencyParams efficiency;
  GeometryParams geometry;
  double u; // Circumferential velocity (m/s)
  std::vector<double> reaction_coeff;

public:
  // Constructor
  AxialCompressor() : outlet_pressure(0), mass_flow(0), stages(0), u(0) {}

  // Setters
  void setGasProperties(double k, double R) {
    gas.k = k;
    gas.R = R;
  }

  void setInletConditions(double T, double p, double c) {
    inlet.T = T;
    inlet.p = p;
    inlet.c = c;
  }

  void setOutletPressure(double p) { outlet_pressure = p; }

  void setMassFlow(double flow) { mass_flow = flow; }

  void setStages(int n) { stages = n; }

  void setEfficiency(double ad, double mech, double leak) {
    efficiency.ad = ad;
    efficiency.mech = mech;
    efficiency.leak = leak;
  }

  void setCircumferentialVelocity(double velocity) { u = velocity; }

  void setReactionCoefficients(const std::vector<double> &coeffs) {
    reaction_coeff = coeffs;
  }

  // Validate required parameters
  void validateInputs() {
    if (gas.k == 0) {
      throw std::runtime_error("缺少必要参数: gas.k");
    }
    if (gas.R == 0) {
      throw std::runtime_error("缺少必要参数: gas.R");
    }
    if (inlet.T == 0) {
      throw std::runtime_error("缺少必要参数: inlet.T");
    }
    if (inlet.p == 0) {
      throw std::runtime_error("缺少必要参数: inlet.p");
    }
    if (inlet.c == 0) {
      throw std::runtime_error("缺少必要参数: inlet.c");
    }
    if (outlet_pressure == 0) {
      throw std::runtime_error("缺少必要参数: outlet_pressure");
    }
    if (mass_flow == 0) {
      throw std::runtime_error("缺少必要参数: mass_flow");
    }
    if (stages == 0) {
      throw std::runtime_error("缺少必要参数: stages");
    }
    if (efficiency.ad == 0) {
      throw std::runtime_error("缺少必要参数: efficiency.ad");
    }
    if (efficiency.mech == 0) {
      throw std::runtime_error("缺少必要参数: efficiency.mech");
    }
    if (efficiency.leak == 0) {
      throw std::runtime_error("缺少必要参数: efficiency.leak");
    }
    if (u == 0) {
      throw std::runtime_error("缺少必要参数: u");
    }
  }

  // Main calculation method
  CompressorResults calculate() {
    validateInputs();

    double k = gas.k;
    double R = gas.R;
    double Cp = k * R / (k - 1);

    // 1. Calculate total adiabatic energy head
    double p_ratio = outlet_pressure / inlet.p;
    double H_ad_total =
        (k / (k - 1)) * R * inlet.T * (std::pow(p_ratio, (k - 1) / k) - 1);

    // 2. Calculate reheat coefficient
    double a = 1 + (efficiency.ad / efficiency.ad - 1) * (1 - 1.0 / stages);

    // 3. Distribute energy head per stage
    double H_ad_stage = a * H_ad_total / stages;

    // Initialize stage parameters
    double T_in = inlet.T;
    double p_in = inlet.p;
    double c_in = inlet.c;
    double rho_in = p_in / (R * T_in);

    // Calculate initial flow area
    double A_flow = mass_flow / (rho_in * c_in);

    std::vector<StageResult> stage_results;
    double total_power = 0;

    // Loop through each stage
    for (int i = 0; i < stages; i++) {
      // 4. Calculate reaction degree
      double Omega;
      if (!reaction_coeff.empty() && reaction_coeff.size() > i) {
        Omega = reaction_coeff[i];
      } else {
        if (stages > 1) {
          Omega = 0.5 + 0.1 * (i) / (stages - 1);
        } else {
          Omega = 0.5;
        }
      }

      // 5. Calculate stage adiabatic temperature rise
      double Delta_T_ad = H_ad_stage / Cp;

      // 6. Calculate actual temperature rise
      double total_efficiency =
          efficiency.ad * efficiency.mech * efficiency.leak;
      double Delta_T = Delta_T_ad / total_efficiency;

      // 7. Calculate outlet temperature
      double T_out = T_in + Delta_T;

      // 8. Calculate outlet pressure
      double n = k / (k - efficiency.ad * (k - 1));
      double p_out = p_in * std::pow(T_out / T_in, n / (n - 1));

      // 9. Calculate outlet density
      double rho_out = p_out / (R * T_out);

      // 10. Calculate outlet velocity
      double c_out = mass_flow / (rho_out * A_flow);

      // 11. Calculate stage power
      double power_stage = mass_flow * Cp * Delta_T;
      total_power += power_stage;

      // 12. Store stage results
      StageResult result;
      result.stage = i + 1;
      result.inlet_T = T_in;
      result.outlet_T = T_out;
      result.inlet_p = p_in / 1000;   // Convert to kPa
      result.outlet_p = p_out / 1000; // Convert to kPa
      result.inlet_velocity = c_in;
      result.outlet_velocity = c_out;
      result.density = rho_out;
      result.power = power_stage;
      result.reaction = Omega;
      result.pressure_ratio = p_out / p_in;
      result.temperature_rise = Delta_T;

      stage_results.push_back(result);

      // Update inlet parameters for next stage
      T_in = T_out;
      p_in = p_out;
      c_in = c_out;
      rho_in = rho_out;

      // Update flow area
      A_flow = mass_flow / (rho_in * c_in);
    }

    // 13. Calculate total efficiency
    double overall_efficiency = H_ad_total / (total_power / mass_flow);

    // 14. Compile results
    CompressorResults results;
    results.stages = stages;
    results.total_power = total_power;
    results.total_efficiency = overall_efficiency;
    results.outlet_T = T_in;
    results.outlet_p = p_in / 1000;
    results.outlet_velocity = c_in;
    results.stage_results = stage_results;

    return results;
  }

  // Display stage results
  void displayStageResults(const CompressorResults &results) {
    std::cout << "\n=== 轴流压缩机逐级计算结果 ===" << std::endl;
    std::cout << "级数: " << results.stages << std::endl;
    std::cout << "总压比: " << std::fixed << std::setprecision(2)
              << (results.outlet_p * 1000 / inlet.p) << std::endl;
    std::cout << "总温升: " << (results.outlet_T - inlet.T) << " K"
              << std::endl;
    std::cout << "总耗功: " << (results.total_power / 1000) << " kW"
              << std::endl
              << std::endl;

    for (const auto &s : results.stage_results) {
      std::cout << "--- 第" << s.stage << "级 ---" << std::endl;
      std::cout << "  进口温度: " << std::setprecision(2) << s.inlet_T << " K"
                << std::endl;
      std::cout << "  出口温度: " << s.outlet_T << " K" << std::endl;
      std::cout << "  进口压力: " << s.inlet_p << " kPa" << std::endl;
      std::cout << "  出口压力: " << s.outlet_p << " kPa" << std::endl;
      std::cout << "  压比: " << std::setprecision(3) << s.pressure_ratio
                << std::endl;
      std::cout << "  温升: " << std::setprecision(2) << s.temperature_rise
                << " K" << std::endl;
      std::cout << "  进口速度: " << s.inlet_velocity << " m/s" << std::endl;
      std::cout << "  出口速度: " << s.outlet_velocity << " m/s" << std::endl;
      std::cout << "  反动度: " << std::setprecision(3) << s.reaction
                << std::endl;
      std::cout << "  级功率: " << std::setprecision(2) << (s.power / 1000)
                << " kW" << std::endl
                << std::endl;
    }
  }
};

// Example usage
int main() {
  try {
    AxialCompressor compressor;

    // Set gas properties (example: air at sea level)
    compressor.setGasProperties(1.4, 287.0);

    // Set inlet conditions
    compressor.setInletConditions(288.15, 101325.0, 100.0);

    // Set outlet pressure
    compressor.setOutletPressure(304000.0);

    // Set mass flow rate
    compressor.setMassFlow(50.0);

    // Set number of stages
    compressor.setStages(5);

    // Set efficiency parameters
    compressor.setEfficiency(0.88, 0.98, 0.95);

    // Set circumferential velocity
    compressor.setCircumferentialVelocity(200.0);

    // Calculate
    CompressorResults results = compressor.calculate();

    // Display results
    compressor.displayStageResults(results);

    std::cout << "\nFinal Results:" << std::endl;
    std::cout << "Total Power: " << results.total_power / 1000 << " kW"
              << std::endl;
    std::cout << "Total Efficiency: " << results.total_efficiency << std::endl;

  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
