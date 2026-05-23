import numpy as np

class CPController:
    def __init__(self, initial, footstep_planner, params):
        self.params = params
        self.delta = params['world_time_step']
        self.eta = params['eta']
        self.footstep_planner = footstep_planner
        
        alpha = params['alpha']
        beta = params['beta']
        gamma = params['gamma'] 
        g_p = params['g_p']
        
        self.k_1 = -((alpha - self.eta) * (beta - self.eta)) / (self.eta * g_p)
        self.k_2 = -(alpha + beta - self.eta + g_p) / g_p
        
        
        self.k_I = (alpha * beta * gamma) / (self.eta * g_p) # Guadagno integrale
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
        p_cmd[0:2] = (
            p_ref[0:2] 
            - self.k_1 * cp_error 
            - self.k_2 * (p_meas[0:2] - p_ref[0:2]) 
            - self.k_I * self.cp_error_integral
        )
        
        # Clipping rigoroso del ZMP per non far esplodere la Dinamica Inversa
        limit = 0.06
        p_cmd[0] = np.clip(p_cmd[0], p_ref[0] - limit, p_ref[0] + limit)
        p_cmd[1] = np.clip(p_cmd[1], p_ref[1] - limit, p_ref[1] + limit)

        # 4. Evoluzione del modello di riferimento
        com_pos_ref = self.lip_state['com']['pos']
        
        com_vel_ref = np.zeros(3)
        com_vel_ref[0:2] = self.eta * (xi_ref[0:2] - com_pos_ref[0:2])
        
        com_pos_ref_new = com_pos_ref + com_vel_ref * self.delta
        com_pos_ref_new[2] = self.params['h'] 
        
        com_acc_ref = np.zeros(3)
        com_acc_ref[0:2] = (self.eta**2) * (com_pos_ref_new[0:2] - p_ref[0:2])

        self.lip_state['zmp']['pos'] = p_cmd
        self.lip_state['zmp']['vel'] = (p_cmd - self.lip_state['zmp']['pos']) / self.delta
        self.lip_state['com']['pos'] = com_pos_ref_new
        self.lip_state['com']['vel'] = com_vel_ref
        self.lip_state['com']['acc'] = com_acc_ref

        contact = self.footstep_planner.get_phase_at_time(t)
        if contact == 'ss':
            contact = step['foot_id']

        return self.lip_state, contact, p_cmd, self.cp_error_integral
