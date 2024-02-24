import numpy as np
from minimal_working_example_no_ros.ControlTools.ControlToolbox.interfaces import DataSource
from datetime import datetime
class PID(DataSource):
    def __init__(self, name, pid_config, dT):
        self._name = name
        self._signal_names = {'P', 'I', 'D', 'error', 'output'}
        self._signals = {key: [] for key in self._signal_names}
        self.terms = np.zeros((3), dtype=np.float32)
        self.Kp = pid_config['Kp']
        self.Ki = pid_config['Ki']
        self.Kd = pid_config['Kd']
        self.deltaT = dT
        self.prev_error = 0

    def __call__(self, error):
        P = self.pTerm(error)
        I = self.iTerm(error)
        D = self.dTerm(error)
        u = P + I + D
        current_signals = {'P': P, 'I': I, 'D': D, 'error': error, 'output': u}
        self.update_signals(current_signals)
        self.update_signals_dict()
        return u

    def pTerm(self, error):
        self.terms[0] = self.Kp * error
        return self.terms[0]

    def iTerm(self, error):
        self.terms[1] = self.terms[1] + self.Ki * self.deltaT * error
        return self.terms[1]
        ##TODO ADD ANTI-WINDUP TO iTerm

    def dTerm(self, error):
        self.terms[2] = (error - self.prev_error) / self.deltaT
        self.terms[2] = self.terms[2] * self.Kd
        self.prev_error = error
        return self.terms[2]

    @property
    def name(self):
        return self._name

    @property
    def signals(self):
        return self._signals

    @property
    def signal_names(self):
        return self._signal_names

    def update_signals(self, current_signals):
        current_time = datetime.now()
        for key in current_signals.keys():
            self._signals[key].append((current_time, current_signals[key]))

