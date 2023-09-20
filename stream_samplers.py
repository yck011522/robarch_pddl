import pybullet_planning as pp
from compas_fab.robots import Configuration, Robot
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame

from integral_timber_joints.process import RobotClampAssemblyProcess, RoboticMovement, Action
from integral_timber_joints.planning.robot_setup import GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import gantry_base_generator
from integral_timber_joints.planning.stream import _get_sample_bare_arm_ik_fn

from utils import LOGGER

def sample_ik_for_target_frames(client: PyChoreoClient, robot: Robot, frames, options=None):
    debug = options.get('debug', False)
    verbose = options.get('verbose', False)
    gantry_attempts = options.get('gantry_attempts', 50)
    reachable_range = options.get('reachable_range', (0.2, 2.4))

    sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

    found_confs = [None for _ in frames]
    with pp.WorldSaver():
        for i, end_t0cf_frame in enumerate(frames):
            sample_found = False

            # * do rejection sampling only for the robot body and the static obstacles
            gantry_base_gen_fn = gantry_base_generator(client, robot, end_t0cf_frame, reachable_range=reachable_range, scale=1.0, options=options)

            for gantry_iter, base_conf in zip(range(gantry_attempts), gantry_base_gen_fn):
                # * bare-arm IK sampler
                arm_conf_vals = sample_ik_fn(pose_from_frame(end_t0cf_frame, scale=1))
                # * iterate through all 6-axis IK solution
                for arm_conf_val in arm_conf_vals:
                    if arm_conf_val is None:
                        continue
                    full_conf = Configuration(list(base_conf.joint_values) + list(arm_conf_val),
                        gantry_arm_joint_types, gantry_arm_joint_names)

                    # * the collision is checked among:
                    #     1. robot self-collision (if `self_collisions=true`), ignored robot link pairs can be specified in `disabled_collisions`
                    #     2. between (robot links) and (attached objects)
                    #     3. between (robot links, attached objects) and obstacles
                    # ! since we don't specify anything in the attachment, we only need to check 1 and 3
                    if not client.check_collisions(robot, full_conf, options=options):
                        sample_found = True
                        LOGGER.debug('IK sample found after {} gantry iters.'.format(gantry_iter))
                        # if debug:
                            # pp.wait_if_gui('IK Conf found.')
                        break
                if sample_found:
                    break

            # ! return None if one of the movement cannot find an IK solution
            if not sample_found:
                return None
            else:
                found_confs[i] = full_conf

    return (found_confs,)