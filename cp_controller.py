import numpy as np

class CPController:
    def __init__(self, initial, footstep_planner, params):
        self.params = params
        self.delta = params['world_time_step']
        self.eta = params['eta']
        self.use_cp = params.get('use_cp', True)
        self.use_lag = params.get('use_lag', False)   # <-- add this
        self.footstep_planner = footstep_planner

        alpha = params['alpha']
        gamma = params['gamma']

        if self.use_lag:
            # Full 3rd-order CPI-ZMP system WITH ZMP lag (Balance_control.pdf eq. 20-22).
            # Poles {alpha, beta, gamma} assigned via pole placement; 'beta' and 'g_p'
            # now enter, unlike the lagless case below.
            self.g_p = params['g_p']
            beta = params['beta']
            self.k_1 = -(alpha*beta + beta*gamma + gamma*alpha
                        - self.eta*(alpha + beta + gamma - self.eta)) / (self.eta * self.g_p)
            self.k_2 = -(alpha + beta + gamma + self.g_p - self.eta) / self.g_p
            self.k_I = (alpha * beta * gamma) / (self.eta * self.g_p)

            self.A_lip = np.array([[0, 1, 0],
                                    [self.eta**2, 0, -self.eta**2],
                                    [0, 0, -self.g_p]])
            self.B_lip = np.array([[0], [0], [self.g_p]])
        else:
            # Lagless plant (g_p -> inf): commanded ZMP realized within one control step.
            # Collapses to the 2nd-order system in [e_xi, integral(e_xi)]; 'beta' and
            # 'g_p' don't enter.
            self.k_1 = (alpha + gamma) / self.eta - 1.0
            self.k_2 = 0.0
            self.k_I = -(alpha * gamma) / self.eta

            self.A_lip = np.array([[0, 1, 0], [self.eta**2, 0, -self.eta**2], [0, 0, 0]])
            self.B_lip = np.array([[0], [0], [1]])

        self.cp_error_integral = np.zeros(2)

        self.lip_state = {
            'com': {'pos': initial['com']['pos'].copy(), 'vel': initial['com']['vel'].copy(), 'acc': np.zeros(3)},
            'zmp': {'pos': initial['zmp']['pos'].copy(), 'vel': np.zeros(3)}
        }
    def solve(self, current, t):
        # 1. Recupero informazioni sul passo corrente e successivo
        step_index = self.footstep_planner.get_step_index_at_time(t)
        if step_index is None:
            step_index = len(self.footstep_planner.plan) - 1
            
        step = self.footstep_planner.plan[step_index]
        start_time = self.footstep_planner.get_start_time(step_index)
        time_in_step = t - start_time
        
        ss_dur = step['ss_duration']
        ds_dur = step['ds_duration']
        
        p_current = np.array([step['pos'][0], step['pos'][1], 0.0])
        if step_index + 1 < len(self.footstep_planner.plan):
            p_next = np.array([self.footstep_planner.plan[step_index + 1]['pos'][0],
                               self.footstep_planner.plan[step_index + 1]['pos'][1], 0.0])
        elif step_index >= 1:
            # last step: there is no next footstep, so during the final double
            # support, drive the ZMP/CP to the center of the two planted feet
            prev_pos = self.footstep_planner.plan[step_index - 1]['pos']
            p_next = np.array([(step['pos'][0] + prev_pos[0]) / 2.,
                               (step['pos'][1] + prev_pos[1]) / 2., 0.0])
        else:
            p_next = p_current

        # 2. Traiettorie ideali (ZMP e Capture Point)
        if time_in_step < ss_dur:
            p_ref = p_current
        else:
            # Transizione fluida durante il doppio appoggio
            phase = (time_in_step - ss_dur) / max(ds_dur, 1)
            phase = np.clip(phase, 0, 1)
            phase_smooth = phase * phase * (3 - 2 * phase) 
            p_ref = p_current + phase_smooth * (p_next - p_current)
            
        # Traiettoria esponenziale esatta del Capture Point Ideale
        t_rem_sec = (ss_dur + ds_dur - time_in_step) * self.delta
        xi_ref = p_ref + np.exp(-self.eta * t_rem_sec) * (p_next - p_ref)

        # 3. Controllo feedback con integrazione (Equazione 21)
        xi_meas = current['com']['pos'] + current['com']['vel'] / self.eta
        p_meas  = current['zmp']['pos']

        cp_error = xi_meas[0:2] - xi_ref[0:2]
        
        # Accumulo dell'errore con Anti-Windup a 5cm
        self.cp_error_integral += cp_error * self.delta
        self.cp_error_integral = np.clip(self.cp_error_integral, -0.05, 0.05)

        # Legge di controllo ZMP completa (Eq. 21)
        p_cmd = np.zeros(3)
        p_cmd = np.zeros(3)
        if self.use_cp:
            fb = - self.k_1 * cp_error - self.k_2 * (p_meas[0:2] - p_ref[0:2])
            if self.use_lag:
                fb -= self.k_I * self.cp_error_integral
            p_cmd[0:2] = p_ref[0:2] + fb
        else:
            # Plain ZMP tracking: follow the planned ZMP trajectory, no CP feedback.
            p_cmd[0:2] = p_ref[0:2]
        
        # Keep the commanded ZMP inside the support polygon (Balance_control.pdf,
        # Sec. III-B). Single support: within the support foot (+-foot_size/2).
        # Double support: anywhere in the bounding box of the two feet, plus margin.
        half = self.params['foot_size'] / 2.0
        if time_in_step < ss_dur:
            lo = p_current - half
            hi = p_current + half
        else:
            lo = np.minimum(p_current, p_next) - half
            hi = np.maximum(p_current, p_next) + half
        p_cmd[0] = np.clip(p_cmd[0], lo[0], hi[0])
        p_cmd[1] = np.clip(p_cmd[1], lo[1], hi[1])

        # 4. Evoluzione del modello di riferimento
        com_pos_ref = self.lip_state['com']['pos']
        
        com_vel_ref = np.zeros(3)
        com_vel_ref[0:2] = self.eta * (xi_ref[0:2] - com_pos_ref[0:2])
        
        com_pos_ref_new = com_pos_ref + com_vel_ref * self.delta
        com_pos_ref_new[2] = self.params['h'] 
        
        # COM acceleration feedforward MUST use the commanded ZMP p_cmd (not p_ref):
        # inverse_dynamics consumes only desired['com']['acc'] (it ignores
        # desired['zmp']), so this term is the ONLY path by which the CP feedback
        # correction reaches the robot. Using p_ref here would discard the feedback.
        # For constant height (com_z = h, zmp_z = 0, eta^2 = g/h) the z term reduces to eta^2*h - g = 0
        com_acc_ref = (self.eta**2) * (com_pos_ref_new - p_cmd) + np.array([0., 0., -self.params['g']])

        prev_zmp = self.lip_state['zmp']['pos'].copy()
        if self.use_lag:
            # First-order lag toward the commanded ZMP: p_dot = g_p*(p_cmd - p)
            zmp_vel = self.g_p * (p_cmd - prev_zmp)
            self.lip_state['zmp']['pos'] = prev_zmp + zmp_vel * self.delta
            self.lip_state['zmp']['vel'] = zmp_vel
        else:
            self.lip_state['zmp']['pos'] = p_cmd
            self.lip_state['zmp']['vel'] = (p_cmd - prev_zmp) / self.delta
        self.lip_state['com']['pos'] = com_pos_ref_new
        self.lip_state['com']['vel'] = com_vel_ref
        self.lip_state['com']['acc'] = com_acc_ref

        contact = self.footstep_planner.get_phase_at_time(t)
        if contact == 'ss':
            contact = step['foot_id']

        return self.lip_state, contact, p_cmd, self.cp_error_integral
