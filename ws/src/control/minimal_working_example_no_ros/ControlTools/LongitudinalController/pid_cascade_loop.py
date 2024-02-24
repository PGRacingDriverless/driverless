from minimal_working_example_no_ros.ControlTools.ControlToolbox.basic_controllers import PID
from minimal_working_example_no_ros.ControlTools.ControlToolbox.interfaces import DataSource
from datetime import datetime
class DoubleCascadeLoop(DataSource):
    # TODO add visual representation/diagram of this system to README/documentation
    def __init__(self, name, loop_config, dT):
        self._name = name
        self._signal_names = {'velocity_setpoint',
                              'velocity_measured',
                              'acceleration_setpoint',
                              'acceleration_measured'}
        self._signals = {key: [] for key in self._signal_names}
        self.pid_velocity = PID('outer_pid',loop_config['pid_velocity'], dT)
        self.pid_acceleration = PID('inner_pid', loop_config['pid_acceleration'], dT)
        self.pids = [self.pid_velocity, self.pid_acceleration]

    def __call__(self, velocity_setpoint, velocity_measured, acceleration_measured):
        setpoints = [velocity_setpoint]
        measurements = [velocity_measured, acceleration_measured]
        for i, controller in enumerate(self.pids):
            measurement = measurements[i]
            setpoint = setpoints[-1]
            error = setpoint - measurement
            controller_output = controller(error)
            setpoints.append(controller_output)
        u = controller_output
        current_signals = {'velocity_setpoint': setpoints[0], 'velocity_measured': velocity_measured,
                           'acceleration_setpoint': setpoints[1], 'acceleration_measured': acceleration_measured}
        self.update_signals(current_signals)
        self.update_signals_dict()
        return u

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