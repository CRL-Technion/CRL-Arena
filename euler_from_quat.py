import numpy

def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler pitch, yaw, roll.

    The intake is switched up a bit to accommodate for Motive's unexpected behavior.

    quat = {"x": 0, "y": 0, "z": 0, "w": 0}
    """
    #if passing in a Rotation object for quat
    # x = quat.w
    # y = quat.x
    # z = quat.y
    # w = quat.z


    x = float(quat['x'])
    y = float(quat['y'])
    z = float(quat['z'])
    w = float(quat['w'])

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = numpy.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = numpy.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = numpy.arctan2(siny_cosp, cosy_cosp)


    return pitch, yaw, roll
