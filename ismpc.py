import numpy as np
import casadi as cs

class Ismpc:
  def __init__(self, initial, footstep_planner, params):
    # parameters
    self.params = params
    self.N = params['N']
    self.delta = params['world_time_step']
    self.h = params['h']
    self.eta = params['eta']
    self.foot_size = params['foot_size']
    self.initial = initial
    self.footstep_planner = footstep_planner
    self.sigma = lambda t, t0, t1: np.clip((t - t0) / (t1 - t0), 0, 1) # piecewise linear sigmoidal function

    self.k_1 = params['k_1']
    self.k_2 = params['k_2']
    self.k_i = params['k_i']
    self.alpha = params['alpha']
    self.use_cp = params.get('use_cp', True)
    self.use_open_loop = params.get('open_loop', False)

    # lip model matrices
    self.A_lip = np.array([[0, 1, 0], [self.eta**2, 0, -self.eta**2], [0, 0, 0]]) # per aggiungere il lag g_p basta cambiare il terzo 0 del terzo vettore
    self.B_lip = np.array([[0], [0], [1]])

    # dynamics
    self.f = lambda x, u: cs.vertcat(
      self.A_lip @ x[0:3] + self.B_lip @ u[0],
      self.A_lip @ x[3:6] + self.B_lip @ u[1],
      self.A_lip @ x[6:9] + self.B_lip @ u[2] + np.array([0, - params['g'], 0]),
    )
    self.xi_error_int = np.zeros(3)
    self._xi_ref_prev = None   # stores X[:,1] from previous solve for xi_ref alignment
    self.x_mpc = None          # internal LIP state; None until first solve
    self.u = np.zeros(3)       # previous optimal control input

    # optimization problem
    self.opt = cs.Opti('conic')
    p_opts = {"expand": True}
    s_opts = {"max_iter": 1000, "verbose": False}
    self.opt.solver("osqp", p_opts, s_opts)

    self.U = self.opt.variable(3, self.N)
    self.X = self.opt.variable(9, self.N + 1)

    self.x0_param = self.opt.parameter(9)
    self.zmp_x_mid_param = self.opt.parameter(self.N)
    self.zmp_y_mid_param = self.opt.parameter(self.N)
    self.zmp_z_mid_param = self.opt.parameter(self.N)

    for i in range(self.N):
      self.opt.subject_to(self.X[:, i + 1] == self.X[:, i] + self.delta * self.f(self.X[:, i], self.U[:, i]))

    cost = cs.sumsqr(self.U) + \
           100 * cs.sumsqr(self.X[2, 1:].T - self.zmp_x_mid_param) + \
           100 * cs.sumsqr(self.X[5, 1:].T - self.zmp_y_mid_param) + \
           100 * cs.sumsqr(self.X[8, 1:].T - self.zmp_z_mid_param)

    self.opt.minimize(cost)

    # zmp constraints
    self.opt.subject_to(self.X[2, 1:].T <= self.zmp_x_mid_param + self.foot_size / 2.)
    self.opt.subject_to(self.X[2, 1:].T >= self.zmp_x_mid_param - self.foot_size / 2.)
    self.opt.subject_to(self.X[5, 1:].T <= self.zmp_y_mid_param + self.foot_size / 2.)
    self.opt.subject_to(self.X[5, 1:].T >= self.zmp_y_mid_param - self.foot_size / 2.)
    self.opt.subject_to(self.X[8, 1:].T <= self.zmp_z_mid_param + self.foot_size / 2.)
    self.opt.subject_to(self.X[8, 1:].T >= self.zmp_z_mid_param - self.foot_size / 2.)

    # initial state constraint
    self.opt.subject_to(self.X[:, 0] == self.x0_param)

    # stability constraint with periodic tail (pure CP periodicity, no ZMP subtraction)
    self.opt.subject_to(self.X[1, 0     ] + self.eta * self.X[0, 0     ] == \
                        self.X[1, self.N] + self.eta * self.X[0, self.N])
    self.opt.subject_to(self.X[4, 0     ] + self.eta * self.X[3, 0     ] == \
                        self.X[4, self.N] + self.eta * self.X[3, self.N])
    self.opt.subject_to(self.X[7, 0     ] + self.eta * self.X[6, 0     ] == \
                        self.X[7, self.N] + self.eta * self.X[6, self.N])

    # state
    self.x = np.zeros(9)
    self.lip_state = {'com': {'pos': np.zeros(3), 'vel': np.zeros(3), 'acc': np.zeros(3)},
                      'zmp': {'pos': np.zeros(3), 'vel': np.zeros(3)}}

  def solve(self, current, t):
    self.x = np.array([current['com']['pos'][0], current['com']['vel'][0], current['zmp']['pos'][0],
                       current['com']['pos'][1], current['com']['vel'][1], current['zmp']['pos'][1],
                       current['com']['pos'][2], current['com']['vel'][2], current['zmp']['pos'][2]])
    
    mc_x, mc_y, mc_z = self.generate_moving_constraint(t)

  

    if self.use_open_loop:
      # propagate internal LIP state (keeps QP initial condition kinematically consistent)
      if self.x_mpc is None:
        self.x_mpc = self.x.copy()
      else:
        b = self.B_lip.flatten()  # [0, 0, 1]
        g_vec = np.array([0., -self.params['g'], 0.])
        dx = np.concatenate([
          self.A_lip @ self.x_mpc[0:3] + b * self.u[0],
          self.A_lip @ self.x_mpc[3:6] + b * self.u[1],
          self.A_lip @ self.x_mpc[6:9] + b * self.u[2] + g_vec,
        ])
        self.x_mpc = self.x_mpc + self.delta * dx

    else:
      self.x_mpc = self.x.copy()
    

    # solve optimization problem
    self.opt.set_value(self.x0_param, self.x_mpc)
    self.opt.set_value(self.zmp_x_mid_param, mc_x)
    self.opt.set_value(self.zmp_y_mid_param, mc_y)
    self.opt.set_value(self.zmp_z_mid_param, mc_z)

    sol = self.opt.solve()

    self.u = sol.value(self.U[:,0])

    if self.use_cp:
      p_ref = sol.value(self.X[[2, 5, 8], 1])   # Desired ZMP FEEDFORWARD prodotta da MPC

      p_meas  = current['zmp']['pos']

      dt = self.params['world_time_step']
      xi_meas = self.compute_cp(current['com']['pos'], current['com']['vel'])
      x_next = sol.value(self.X[:, 1])
      if self._xi_ref_prev is not None:
        xi_ref = self.compute_cp(self._xi_ref_prev[[0, 3, 6]], self._xi_ref_prev[[1, 4, 7]])
      else:
        xi_ref = xi_meas
      self._xi_ref_prev = x_next
      self.xi_error_int += (xi_meas - xi_ref) * dt

      p_cmd = (
          p_ref
          - self.k_1 * (xi_meas - xi_ref)
          - self.k_2 * (p_meas - p_ref)
          - self.k_i * self.xi_error_int
      )
    else:
      # Plain MPC: command the ZMP planned by the MPC, no capture-point feedback.
      p_cmd = sol.value(self.X[[2, 5, 8], 1])

    self.opt.set_initial(self.U, sol.value(self.U))
    self.opt.set_initial(self.X, sol.value(self.X))

    # create output LIP state da MPC (use internal propagated state, not raw sensor)
    self.lip_state['com']['pos'] = np.array([self.x_mpc[0], self.x_mpc[3], self.x_mpc[6]])
    self.lip_state['com']['vel'] = np.array([self.x_mpc[1], self.x_mpc[4], self.x_mpc[7]])
    self.lip_state['zmp']['pos'] = np.array([self.x_mpc[2], self.x_mpc[5], self.x_mpc[8]])

    self.lip_state['zmp']['vel'] = self.u
    self.lip_state['com']['acc'] = self.eta**2 * (self.lip_state['com']['pos'] - self.lip_state['zmp']['pos']) + np.hstack([0, 0, - self.params['g']])

    contact = self.footstep_planner.get_phase_at_time(t)
    if contact == 'ss':
      contact = self.footstep_planner.plan[self.footstep_planner.get_step_index_at_time(t)]['foot_id']

    xi_error_int = self.xi_error_int

    return self.lip_state, contact, p_cmd, xi_error_int
  
  def generate_moving_constraint(self, t):
    mc_x = np.full(self.N, (self.initial['lfoot']['pos'][3] + self.initial['rfoot']['pos'][3]) / 2.)
    mc_y = np.full(self.N, (self.initial['lfoot']['pos'][4] + self.initial['rfoot']['pos'][4]) / 2.)
    time_array = np.array(range(t, t + self.N))
    for j in range(len(self.footstep_planner.plan) - 1):
      fs_start_time = self.footstep_planner.get_start_time(j)
      ds_start_time = fs_start_time + self.footstep_planner.plan[j]['ss_duration']
      fs_end_time = ds_start_time + self.footstep_planner.plan[j]['ds_duration']
      fs_current_pos = self.footstep_planner.plan[j]['pos'] if j > 0 else np.array([mc_x[0], mc_y[0]])
      fs_target_pos = self.footstep_planner.plan[j + 1]['pos']
      mc_x += self.sigma(time_array, ds_start_time, fs_end_time) * (fs_target_pos[0] - fs_current_pos[0])
      mc_y += self.sigma(time_array, ds_start_time, fs_end_time) * (fs_target_pos[1] - fs_current_pos[1])

    # final step: the loop above leaves the constraint on the last footstep (a
    # single foot). During the last double support both last feet are planted, so
    # re-center the ZMP on the support-polygon center (midpoint of the last two
    # footsteps) for a stable two-foot rest instead of resting on one foot.
    last = len(self.footstep_planner.plan) - 1
    if last >= 1:
      ds_start_time = self.footstep_planner.get_start_time(last) + self.footstep_planner.plan[last]['ss_duration']
      fs_end_time = ds_start_time + self.footstep_planner.plan[last]['ds_duration']
      last_foot_pos = self.footstep_planner.plan[last]['pos']
      center_pos = (last_foot_pos + self.footstep_planner.plan[last - 1]['pos']) / 2.
      mc_x += self.sigma(time_array, ds_start_time, fs_end_time) * (center_pos[0] - last_foot_pos[0])
      mc_y += self.sigma(time_array, ds_start_time, fs_end_time) * (center_pos[1] - last_foot_pos[1])

    return mc_x, mc_y, np.zeros(self.N)
  
  def compute_cp(self, com_pos, com_vel):
    return com_pos + com_vel / self.eta