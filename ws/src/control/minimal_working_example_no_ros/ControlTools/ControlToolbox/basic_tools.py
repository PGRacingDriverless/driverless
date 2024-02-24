from minimal_working_example_no_ros.ControlTools.ControlToolbox.interfaces import DataSource
from datetime import datetime
class Saturator(DataSource):
    def __init__(self, name, min=-1.0, max=1.0):
        self._name=name
        self._signal_names = {'input',
                              'output'}
        self._signals = {key: [] for key in self._signal_names}
        self.min=min
        self.max=max
    def __call__(self, x):
        out = min(max(x, self.min), self.max)
        current_signals = {'input': x, 'output': out}
        self.update_signals(current_signals)
        self.update_signals_dict()
        return out
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


class SimpleLowPass:
    def __init__(self, weight):
        self.weight=weight
        self.prev_value=0

    def __call__(self, value):
        return self.weight*self.prev_value + (1 - self.weight)*value

