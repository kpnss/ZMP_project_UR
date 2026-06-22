import numpy as np

class CPController:
    def __init__(self, initial, footstep_planner, params):
        self.params = params
        self.delta = params['world_time_step']
        self.eta = params['eta']
        self.use_cp = params.get('use_cp', True)
        self.footstep_planner = footstep_planner
        
        # Closed-loop poles for the CP-error + CP-integral dynamics (both < 0):
        #   alpha -> capture-point error pole, gamma -> integrator pole.
        alpha = params['alpha']
        gamma = params['gamma']

        # The commanded ZMP is realized within one control step (simulation.py sets
        # desired['zmp']['vel'] = (p_cmd - zmp)/dt), so the plant has NO ZMP lag
        # (g_p -> inf). This collapses Morisawa's 3rd-order system (Balance_control.pdf
        # eq. 20) to a 2nd-order one in [e_xi, integral(e_xi)]; 'beta' and 'g_p' no
        # longer enter. Placing the two remaining poles at {alpha, gamma} on the
        # lagless plant gives:
        #   k_1 = (alpha + gamma) / eta - 1
        #   k_2 = 0                          (no ZMP-lag state left to feed back)
        #   k_I = -alpha * gamma / eta
        self.k_1 = (alpha + gamma) / self.eta - 1.0
        self.k_2 = 0.0
        self.k_I = -(alpha * gamma) / self.eta
        self.cp_error_integral = np.zeros(2)
        
        self.A_lip = np.array([[0, 1, 0], [self.eta**2, 0, -self.eta**2], [0, 0, 0]])
        self.B_lip = np.array([[0], [0], [1]])
        
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
        if self.use_cp:
            p_cmd[0:2] = (
                p_ref[0:2]
                - self.k_1 * cp_error
                - self.k_2 * (p_meas[0:2] - p_ref[0:2])
                # - self.k_I * self.cp_error_integral
            )
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
        com_acc_ref = np.zeros(3)
        com_acc_ref[0:2] = (self.eta**2) * (com_pos_ref_new[0:2] - p_cmd[0:2])

        prev_zmp = self.lip_state['zmp']['pos'].copy()
        self.lip_state['zmp']['pos'] = p_cmd
        self.lip_state['zmp']['vel'] = (p_cmd - prev_zmp) / self.delta
        self.lip_state['com']['pos'] = com_pos_ref_new
        self.lip_state['com']['vel'] = com_vel_ref
        self.lip_state['com']['acc'] = com_acc_ref

        contact = self.footstep_planner.get_phase_at_time(t)
        if contact == 'ss':
            contact = step['foot_id']

        return self.lip_state, contact, p_cmd, self.cp_error_integral
