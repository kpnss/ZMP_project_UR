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

    # nuove variabili
    self.k_1 = params['k_1']
    self.k_2 = params['k_2']
    self.alpha = params['alpha']
    self.beta = params['beta']
    self.eta = params['eta']


    # lip model matrices
    self.A_lip = np.array([[0, 1, 0], [self.eta**2, 0, -self.eta**2], [0, 0, 0]]) # per aggiungere il lag g_p basta cambiare il terzo 0 del terzo vettore
    self.B_lip = np.array([[0], [0], [1]])

    # dynamics
    self.f = lambda x, u: cs.vertcat(
      self.A_lip @ x[0:3] + self.B_lip @ u[0],
      self.A_lip @ x[3:6] + self.B_lip @ u[1],
      self.A_lip @ x[6:9] + self.B_lip @ u[2] + np.array([0, - params['g'], 0]),
    )

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

    # stability constraint with periodic tail
    self.opt.subject_to(self.X[1, 0     ] + self.eta * (self.X[0, 0     ] - self.X[2, 0     ]) == \
                        self.X[1, self.N] + self.eta * (self.X[0, self.N] - self.X[2, self.N]))
    self.opt.subject_to(self.X[4, 0     ] + self.eta * (self.X[3, 0     ] - self.X[5, 0     ]) == \
                        self.X[4, self.N] + self.eta * (self.X[3, self.N] - self.X[5, self.N]))
    self.opt.subject_to(self.X[7, 0     ] + self.eta * (self.X[6, 0     ] - self.X[8, 0     ]) == \
                        self.X[7, self.N] + self.eta * (self.X[6, self.N] - self.X[8, self.N]))

    # state
    self.x = np.zeros(9)
    self.lip_state = {'com': {'pos': np.zeros(3), 'vel': np.zeros(3), 'acc': np.zeros(3)},
                      'zmp': {'pos': np.zeros(3), 'vel': np.zeros(3)}}
    
    self.x_mpc = np.array([
        initial['com']['pos'][0], initial['com']['vel'][0], initial['zmp']['pos'][0],
        initial['com']['pos'][1], initial['com']['vel'][1], initial['zmp']['pos'][1],
        initial['com']['pos'][2], initial['com']['vel'][2], initial['zmp']['pos'][2],
    ])
  # Se si vuole avere mpc nel loop usare il solve sotto, altrimenti questo qua (commentare e decomenntare di conseguenza)
  
  def solve(self, current, t):

    x_meas = np.array([
        current['com']['pos'][0], current['com']['vel'][0], current['zmp']['pos'][0],
        current['com']['pos'][1], current['com']['vel'][1], current['zmp']['pos'][1],
        current['com']['pos'][2], current['com']['vel'][2], current['zmp']['pos'][2]
    ])

    mc_x, mc_y, mc_z = self.generate_moving_constraint(t)

    # MPC: sempre inizializzato sullo stato misurato
    self.opt.set_value(self.x0_param, x_meas)
    self.opt.set_value(self.zmp_x_mid_param, mc_x)
    self.opt.set_value(self.zmp_y_mid_param, mc_y)
    self.opt.set_value(self.zmp_z_mid_param, mc_z)

    sol = self.opt.solve()

    # feedforward ZMP da MPC
    p_ref = sol.value(self.X[[2, 5, 8], 1])

    # CP reference coerente con MPC
    x_pred_1 = sol.value(self.X[:, 1])
    xi_ref = self.compute_cp(
        x_meas[[0, 3, 6]],
        x_meas[[1, 4, 7]]
    )

    # misure reali
    p_meas  = current['zmp']['pos']
    xi_meas = self.compute_cp(current['com']['pos'], current['com']['vel'])

    # balance control
    p_cmd = (
        p_ref
        - self.k_1 * (xi_meas - xi_ref)
        - self.k_2 * (p_meas  - p_ref)
    )

    # output lip_state (qui ha senso usare x_meas)
    self.lip_state['com']['pos'] = x_meas[[0, 3, 6]]
    self.lip_state['com']['vel'] = x_meas[[1, 4, 7]]
    self.lip_state['zmp']['pos'] = x_meas[[2, 5, 8]]
    self.lip_state['zmp']['vel'] = sol.value(self.U[:, 0])
    self.lip_state['com']['acc'] = self.eta**2 * (self.lip_state['com']['pos'] - self.lip_state['zmp']['pos']) \
                                   + np.array([0, 0, -self.params['g']])

    contact = self.footstep_planner.get_phase_at_time(t)
    if contact == 'ss':
        contact = self.footstep_planner.plan[self.footstep_planner.get_step_index_at_time(t)]['foot_id']

    return self.lip_state, contact, p_cmd

  '''
  def solve(self, current, t):
    self.x = np.array([current['com']['pos'][0], current['com']['vel'][0], current['zmp']['pos'][0],
                       current['com']['pos'][1], current['com']['vel'][1], current['zmp']['pos'][1],
                       current['com']['pos'][2], current['com']['vel'][2], current['zmp']['pos'][2]])
    
    mc_x, mc_y, mc_z = self.generate_moving_constraint(t)

    # solve optimization problem
    self.opt.set_value(self.x0_param, self.x)
    self.opt.set_value(self.zmp_x_mid_param, mc_x)
    self.opt.set_value(self.zmp_y_mid_param, mc_y)
    self.opt.set_value(self.zmp_z_mid_param, mc_z)

    sol = self.opt.solve()
    self.x_pred = sol.value(self.X[:,1]) # rimosso perchè non uso piu mpc
    self.u = sol.value(self.U[:,0])

    p_ref = sol.value(self.X[[2, 5, 8], 1])   # Desired ZMP FEEDFORWARD

    xi_ref = self.compute_cp(
        self.x[[0, 3, 6]],   # COM pos predetta MPC
        self.x[[1, 4, 7]]    # COM vel predetta MPC
    ) # FEEDBACK

    p_meas  = current['zmp']['pos']
    xi_meas = self.compute_cp(
        current['com']['pos'],
        current['com']['vel']
    ) # FEEDBACK

    p_cmd = (
        p_ref
        - self.k_1 * (xi_meas - xi_ref)
        - self.k_2 * (p_meas  - p_ref)
    )

    self.opt.set_initial(self.U, sol.value(self.U))
    self.opt.set_initial(self.X, sol.value(self.X))

    # create output LIP state da MPC
    self.lip_state['com']['pos'] = np.array([self.x[0], self.x[3], self.x[6]])
    self.lip_state['com']['vel'] = np.array([self.x[1], self.x[4], self.x[7]])
    self.lip_state['zmp']['pos'] = np.array([self.x[2], self.x[5], self.x[8]])
    self.lip_state['zmp']['vel'] = self.u
    self.lip_state['com']['acc'] = self.eta**2 * (self.lip_state['com']['pos'] - self.lip_state['zmp']['pos']) + np.hstack([0, 0, - self.params['g']])

    contact = self.footstep_planner.get_phase_at_time(t)
    if contact == 'ss':
      contact = self.footstep_planner.plan[self.footstep_planner.get_step_index_at_time(t)]['foot_id']

    return self.lip_state, contact, p_cmd
  '''
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

    return mc_x, mc_y, np.zeros(self.N)
    
  def compute_cp(self, com_pos, com_vel):
    return com_pos + com_vel / self.eta