#include <casadi/casadi.hpp>

#include <cmath>
#include <iostream>
#include <string>

int main()
{
  using casadi::DM;
  using casadi::DMDict;
  using casadi::Dict;
  using casadi::Function;
  using casadi::SX;
  using casadi::SXDict;

  if (!casadi::has_nlpsol("ipopt")) {
    std::cerr << "CASADI_CPP_SMOKE_FAILED ipopt plugin unavailable\n";
    return 2;
  }

  const SX x = SX::sym("x");
  const SX error = x - 3.0;
  const SXDict nlp{{"x", x}, {"f", error * error}, {"g", x}};
  Dict options;
  options["print_time"] = false;
  options["ipopt.print_level"] = 0;
  options["ipopt.sb"] = "yes";
  options["ipopt.max_iter"] = 50;

  const Function solver = casadi::nlpsol("solver", "ipopt", nlp, options);
  const DMDict arguments{
    {"x0", DM(0.0)},
    {"lbx", DM(-10.0)},
    {"ubx", DM(10.0)},
    {"lbg", DM(0.0)},
    {"ubg", DM(casadi::inf)},
  };
  const DMDict result = solver(arguments);
  const double solution = static_cast<double>(result.at("x"));
  const std::string return_status = solver.stats().at("return_status").to_string();

  if (!std::isfinite(solution) || std::abs(solution - 3.0) > 1e-6) {
    std::cerr << "CASADI_CPP_SMOKE_FAILED solution=" << solution
              << " status=" << return_status << '\n';
    return 3;
  }

  std::cout << "CASADI_CPP_SMOKE_OK version=" << casadi::CasadiMeta::version()
            << " solution=" << solution << " status=" << return_status << '\n';
  return 0;
}
