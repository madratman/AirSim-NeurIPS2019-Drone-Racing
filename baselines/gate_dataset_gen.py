import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pytransform3d.rotations as pr
import pytransform3d.transformations as pt
from pytransform3d.transform_manager import TransformManager

random_state = np.random.RandomState(0)

ee2robot = pt.transform_from_pq(
    np.hstack((np.array([0.4, -0.3, 0.5]), pr.random_quaternion(random_state))))
cam2robot = pt.transform_from_pq(
    np.hstack((np.array([0.0, 0.0, 0.8]), pr.q_id)))
object2cam = pt.transform_from(
    pr.matrix_from_euler_xyz(np.array([0.0, 0.0, 0.5])),
                             np.array([0.5, 0.1, 0.1]))

tm = TransformManager()
tm.add_transform("end-effector", "robot", ee2robot)
tm.add_transform("camera", "robot", cam2robot)
tm.add_transform("object", "camera", object2cam)

ee2object = tm.get_transform("end-effector", "object")

ax = tm.plot_frames_in("robot", s=0.1)
ax.set_xlim((-0.25, 0.75))
ax.set_ylim((-0.5, 0.5))
ax.set_zlim((0.0, 1.0))
plt.show()


def randomQuadPose(x_range, y_range, z_range, yaw_range, pitch_range, roll_range):
    x = randomSample(x_range)
    y = randomSample(y_range)
    z = randomSample(z_range)
    yaw = randomSample(yaw_range)
    pitch = randomSample(pitch_range)
    roll = randomSample(roll_range)
    q = Rotation.from_euler('ZYX', [yaw, pitch, roll])  # capital letters denote intrinsic rotation (lower case would be extrinsic)
    q = q.as_quat()
    t_o_b = Vector3r(x,y,z)
    q_o_b = Quaternionr(q[0], q[1], q[2], q[3])
    return Pose(t_o_b, q_o_b), yaw

def randomSample(value_range):
    return (value_range[1] - value_range[0])*np.random.random() + value_range[0]

def randomGatePose(p_o_b, phi_base, r_range, cam_fov, correction):
    # create translation of gate
    r = randomSample(r_range)
    alpha = cam_fov / 2.0  # alpha is half of fov angle
    theta_range = [-alpha, alpha]
    theta = randomSample(theta_range)
    # need to make projection on geodesic curve! not equal FOV in theta and psi
    alpha_prime = np.arctan(np.cos(np.abs(theta)))
    psi_range = [-alpha_prime, alpha_prime]
    psi_range = [x * correction for x in psi_range]
    psi = randomSample(psi_range) + np.pi / 2.0
    # get relative vector in the base frame coordinates
    body_to_gate_vector = get_vector_from_polar(r, theta, psi)

    # transform relative vector from base frame to the world frame
    t_b_g = convert_t_body_2_world(body_to_gate_vector, p_o_b.orientation)

    # get the gate coord in world coordinates from origin
    t_o_g = p_o_b.position + t_b_g

    # check if gate is at least half outside the ground
    if t_o_g.z_val >= 0.0:
        continue

    # create rotation of gate
    eps = 0  # np.pi/15.0
    phi_rel_range = [-np.pi + eps, 0 - eps]
    phi_rel = randomSample(phi_rel_range)
    phi_quad_ref = get_yaw_base(p_o_b)
    phi_gate = phi_quad_ref + phi_rel
    rot_gate = Rotation.from_euler('ZYX', [phi_gate, 0, 0])
    q = rot_gate.as_quat()
    p_o_g = Pose(t_o_g, Quaternionr(q[0], q[1], q[2], q[3]))

    return p_o_g, r, theta, psi, phi_rel

# follow below convention for polar coordinates
# r: radius
# theta: azimuth (horizontal)
# psi: vertical
def get_vector_from_polar(r, theta, psi):
    x = r * np.cos(theta) * np.sin(psi)
    y = r * np.sin(theta) * np.sin(psi)
    z = r * np.cos(psi)
    return Vector3r(x, y, z)