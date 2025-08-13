#include <casadi/casadi.hpp>
#include "mpc_casadi.h"

Eigen::VectorXd MPCCasadi::getMPCCasadiCommand(const Eigen::VectorXd &current_state, const Eigen::VectorXd ref_state)
{
  using namespace casadi;

  const int N = params_.MPC_params.N;
  const int nx = params_.sys_params.state_dim;
  const int nu = params_.sys_params.ctrl_input_dim;

  // Convert Eigen to CasADi DM
  std::function<casadi::DM(const Eigen::MatrixXd &)> eig2dm = [](const Eigen::MatrixXd &M)
  {
    std::vector<double> data(M.data(), M.data() + M.size());
    DM dm(data);
    return reshape(dm, M.rows(), M.cols());
  };
  auto vec2dm = [](const Eigen::VectorXd &v)
  {
    std::vector<double> data(v.data(), v.data() + v.size());
    DM dm(data);
    return reshape(dm, v.size(), 1);
  };

  // Matrices
  MX Q = eig2dm(params_.MPC_params.Q_matrix);
  MX R = eig2dm(params_.MPC_params.R_matrix);
  MX P = eig2dm(params_.MPC_params.P_matrix);
  MX A = eig2dm(params_.sys_params.A_matirx);
  MX B = eig2dm(params_.sys_params.B_matrix);

  MX xref = vec2dm(ref_state);
  MX x0 = vec2dm(current_state);

  // Variables
  MX X = MX::sym("X", nx, N + 1);
  MX U = MX::sym("U", nu, N);

  // Decision vector Z = [X0 ... XN, U0 ... U_{N-1}]
  std::vector<MX> opt_vars;
  opt_vars.reserve((N + 1) * nx + N * nu);
  for (int k = 0; k <= N; ++k)
    opt_vars.push_back(X(Slice(), k));
  for (int k = 0; k < N; ++k)
    opt_vars.push_back(U(Slice(), k));
  MX Z = vertcat(opt_vars);

  // Cost function
  MX cost = MX::zeros();
  for (int k = 0; k < N; ++k)
  {
    MX xk = X(Slice(), k);
    MX uk = U(Slice(), k);
    cost += MX::mtimes({(xk - xref).T(), Q, (xk - xref)}) +
            MX::mtimes({uk.T(), R, uk});
  }
  cost += MX::mtimes({(X(Slice(), N) - xref).T(), P, (X(Slice(), N) - xref)});

  // Constraints
  std::vector<MX> g;

  // (1) Initial condition
  g.push_back(X(Slice(), 0) - x0); // nx constraints

  // (2) Dynamics constraints
  for (int k = 0; k < N; ++k)
  {
    MX x_next = MX::mtimes(A, X(Slice(), k)) + MX::mtimes(B, U(Slice(), k));
    g.push_back(X(Slice(), k + 1) - x_next); // nx constraints each
  }

  // (3) Obstacle avoidance via SDF (placeholder)
  // In IPOPT version: dist >= 0. Here, we make it a >= 0 inequality.
  for (int k = 1; k <= N; ++k)
  {
    MX pos = X(Slice(0, 3), k); // first 3 elements: position
    // Replace with your own symbolic SDF function:
    // MX sdf_val = voxel_map_ptr_->getCasadiSDF()(pos);
    MX sdf_val = MX::sym("sdf_val" + std::to_string(k)); // placeholder
    g.push_back(sdf_val);                                // 1 constraint
  }

  // (4) Terminal Lyapunov constraint: V(xN) - threshold <= 0
  double lyap_threshold = dsm_ptr_->getLyapunovThreshold(current_state, ref_state);
  MX lyap_val = MX::mtimes({(X(Slice(), N) - xref).T(), P, (X(Slice(), N) - xref)});
  g.push_back(lyap_val - lyap_threshold); // 1 constraint

  MX g_all = vertcat(g);

  // NLP dict
  MXDict nlp = {{"x", Z}, {"f", cost}, {"g", g_all}};

  // Solver options
  Dict opts;
  opts["ipopt.print_level"] = 0;
  opts["print_time"] = false;
  Function solver = nlpsol("solver", "ipopt", nlp, opts);

  // Bounds on Z
  std::vector<double> lbz(Z.size1(), -1e20), ubz(Z.size1(), 1e20);
  {
    const auto &lb_eig = params_.MPC_params.decision_var_lower_bound;
    const auto &ub_eig = params_.MPC_params.decision_var_upper_bound;
    for (int i = 0; i < lb_eig.size(); ++i)
    {
      lbz[i] = lb_eig[i];
      ubz[i] = ub_eig[i];
    }
  }

  // Bounds on g
  std::vector<double> lbg(g_all.size1(), 0.0), ubg(g_all.size1(), 0.0);

  int constr_idx = 0;
  // Initial condition: equality
  for (int i = 0; i < nx; ++i)
  {
    lbg[constr_idx] = 0.0;
    ubg[constr_idx] = 0.0;
    constr_idx++;
  }
  // Dynamics constraints: equality
  for (int k = 0; k < N; ++k)
  {
    for (int i = 0; i < nx; ++i)
    {
      lbg[constr_idx] = 0.0;
      ubg[constr_idx] = 0.0;
      constr_idx++;
    }
  }
  // SDF constraints: inequality dist >= 0.1
  for (int k = 1; k <= N; ++k)
  {
    lbg[constr_idx] = 0.1; // min distance
    ubg[constr_idx] = 1e20;
    constr_idx++;
  }
  // Lyapunov: <= 0
  lbg[constr_idx] = -1e20;
  ubg[constr_idx] = 0.0;
  constr_idx++;

  // Initial guess
  std::vector<double> z0(Z.size1(), 0.0);
  {
    const auto &init_eig = params_.MPC_params.decision_var_init;
    for (int i = 0; i < init_eig.size(); ++i)
    {
      z0[i] = init_eig[i];
    }
  }

  // Solve NLP
  casadi::DM x0_dm = casadi::DM(z0);
  casadi::DM lbx_dm = casadi::DM(lbz);
  casadi::DM ubx_dm = casadi::DM(ubz);
  casadi::DM lbg_dm = casadi::DM(lbg);
  casadi::DM ubg_dm = casadi::DM(ubg);

  casadi::DMDict arg;
  arg["x0"] = x0_dm;
  arg["lbx"] = lbx_dm;
  arg["ubx"] = ubx_dm;
  arg["lbg"] = lbg_dm;
  arg["ubg"] = ubg_dm;

  casadi::DMDict sol = solver(arg);
  DM sol_z = sol.at("x");
  std::vector<double> sol_vec = std::vector<double>(sol_z->begin(), sol_z->end());

  // Extract first control input u0
  Eigen::VectorXd cmd(nu);
  int u0_index = (N + 1) * nx; // start of U0 in Z
  for (int i = 0; i < nu; ++i)
  {
    cmd(i) = sol_vec[u0_index + i];
  }

  return cmd;
}
