import sys, os

# add FSDS to module search path
fsds_lib_path = os.path.join(os.getcwd(), "Formula-Student-Driverless-Simulator", "python")
sys.path.insert(0, fsds_lib_path)

import pickle
from oclock import Timer
import fsds
from minimal_working_example_no_ros.ControlTools.LongitudinalController.pid_cascade_loop import DoubleCascadeLoop
from minimal_working_example_no_ros.ControlTools.ControlToolbox.basic_tools import Saturator, SimpleLowPass
from minimal_working_example_no_ros.controller_parameters import longitudinal_double_cascade_config
from minimal_working_example_no_ros.ControlTools.ControlToolbox.interfaces import DataSource
from minimal_working_example_no_ros.ControlTools.LateralController.pure_pursuit_controller import get_reference_point, orientation_to_heading
from minimal_working_example_no_ros.vehicle_parameters import FSDS_car_params
from minimal_working_example_no_ros.ControlTools.LateralController.pure_pursuit_controller import PurePursuit
if __name__ == "__main__":
    # set reference velocity
    velocity_ref = 4
    # create controller
    controller = DoubleCascadeLoop('control_loop', longitudinal_double_cascade_config, longitudinal_double_cascade_config['control_interval'])
    # create output saturator
    saturator = Saturator('control_saturator', min=-1.0, max=1.0)
    # pure pursuit controller
    lateral_controller = PurePursuit(K_dd=1, wheelbase=FSDS_car_params['wheelbase'],
                                     min_lookahead=2, max_lookahead=25, k=100)
    # create simple low pass filter
    lowpass = SimpleLowPass(weight=1)
    # config FSDS client
    client = fsds.FSDSClient()
    client.confirmConnection()

    client.enableApiControl(True)
    client.reset()
    car_controls = fsds.CarControls()
    # initialize timer
    timer = Timer(interval=longitudinal_double_cascade_config['control_interval'])

    # previous velocity memory for acceleration calculation
    velocity_prev=0

    while timer.total_time < 1000:
        # get car state
        state = client.getCarState()
        referee = client.getRefereeState()
        velocity = state.speed
        global_position = state.kinematics_estimated.position
        global_position = global_position.to_numpy_array()
        # get cones positions as reference path
        cones_positions = [((d['x'] - referee.initial_position.x)/100, -(d['y'] -  referee.initial_position.y)/100) for d in referee.cones if d.get('color', None) == 0]
        #cones_positions.insert(0, (0-0.75, 0))
        #cones_positions = [(0-0.75, 0), (60, 0), (60, 60)]
        acceleration = lowpass((velocity - velocity_prev) / longitudinal_double_cascade_config['control_interval'])
        # calculate heading
        vehicle_heading = orientation_to_heading(state.kinematics_estimated.orientation)
        steering_angle = lateral_controller(cones_positions, global_position, vehicle_heading, velocity)
        # calculate throttle/break
        u = float(controller(velocity_ref, velocity, acceleration))
        # set valid controls
        car_controls.throttle = max(saturator(u), 0.0)
        car_controls.brake = -min(saturator(u), 0.0)
        car_controls.steering = steering_angle
        client.setCarControls(car_controls)
        # store previous velocity
        velocity_prev=velocity

        # watchdog
        timer.checkpt()
    with open('../logs/longitudinal_pid_test.pkl', "wb") as pickle_file:
        pickle.dump(DataSource.signals_dict, pickle_file)
    # Places the vehicle back at it's original position
    client.reset()