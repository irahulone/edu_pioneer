
skid_steering_expressions = [
        (lambda gain, lx, az: gain * (-lx + az)),
        (lambda gain, lx, az: gain * (-lx - az))
]

tank_steering_expressions = (lambda gain, lx: gain * (-lx))

def skid_steering_evaluation(linear, angular, gain):
    return [skid_steering_expressions[0](gain, linear['x'], angular['z']), skid_steering_expressions[1](gain, linear['x'], angular['z'])]

def tank_steering_evaluation(linear, gain):
    return [tank_steering_expressions(linear['x'], gain), tank_steering_expressions(linear['y'], gain)]
