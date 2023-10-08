from collections import defaultdict
from itertools import product
from termcolor import colored

import pybullet_planning as pp
from compas_fab.robots import Configuration, Robot, AttachedCollisionMesh, CollisionMesh
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame

from integral_timber_joints.process import RobotClampAssemblyProcess, RoboticMovement
from integral_timber_joints.planning.robot_setup import GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import gantry_base_generator
from integral_timber_joints.planning.stream import _get_sample_bare_arm_ik_fn
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.process.action import BeamPlacementWithClampsAction, BeamPlacementWithoutClampsAction, AssembleBeamWithScrewdriversAction, RoboticMovement

from utils import LOGGER

def get_sample_fn_plan_motion_for_beam_assembly(client, robot, process, options=None):
    options = options or {}

    # Precompute and cache all target frames for each beam's assembly action
    gripper_names_from_type = defaultdict(list)
    for gripper in process.grippers:
        if gripper.type_name:
            gripper_names_from_type[gripper.type_name].append(gripper.name)
    for screwdriver in process.screwdrivers:
        if screwdriver.type_name:
            gripper_names_from_type[screwdriver.type_name].append(screwdriver.name)

    toolchanger = process.robot_toolchanger
    flange_from_toolchanger_base = toolchanger.t_t0cf_from_tcf
    target_frames_from_beam_id = defaultdict(list)
    beam_grasps = {}
    for i, beam_id in enumerate(process.assembly.sequence):
        #  Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue

        gripper_type = process.assembly.get_beam_attribute(
            beam_id, "gripper_type")
        assert gripper_type in gripper_names_from_type
        # randomly assign one suitable gripper to each beam just for sampling purposes
        gripper_id = gripper_names_from_type[gripper_type][0]

        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.GROUND_CONTACT:
            action = BeamPlacementWithoutClampsAction(beam_id=beam_id, gripper_id=gripper_id)

        elif process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.CLAMPED:
            # we can leave joint_ids and clamp_ids empty beacuse we don't need ACM here
            action = BeamPlacementWithClampsAction(beam_id=beam_id, joint_ids=[], gripper_id=gripper_id, clamp_ids=[])

        elif (process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.SCREWED_WITH_GRIPPER or \
              process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.SCREWED_WITHOUT_GRIPPER):
            # we can leave joint_ids and clamp_ids empty beacuse we don't need ACM here
            action = AssembleBeamWithScrewdriversAction(beam_id=beam_id, joint_ids=[], gripper_id=gripper_id, screwdriver_ids=[])

        else:
            continue

        # * compute movements from action
        action.create_movements(process)
        for movement in action.movements:
            # * trigger state diff computation
            movement.create_state_diff(process)

            # * skip non-robotic movements for IK computation
            if not isinstance(movement, RoboticMovement):
               continue

            # * skip if there exists a taught conf
            if movement.target_configuration is not None:
                continue

            # * compute IK
            if movement.target_frame is None:
                LOGGER.error(f'Target frame is None in {movement.short_summary}')

            end_t0cf_frame = movement.target_frame.copy()
            end_t0cf_frame.point *= 1e-3
            target_frames_from_beam_id[beam_id].append(end_t0cf_frame)

        # * Cache all grasp transformations
        # ? different gripper might have different grasp for a beam?
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(beam_id)
        beam_gripper_id = process.assembly.get_beam_attribute(beam_id, "gripper_id")
        beam_gripper = process.tool(beam_gripper_id)
        flange_from_beam = flange_from_toolchanger_base * beam_gripper.t_t0cf_from_tcf * t_gripper_tcf_from_beam
        # scale the translation part of the transformation to meter
        for k in range(3):
            flange_from_beam[k,3] *= 1e-3
        beam_grasps[beam_id] = flange_from_beam

    gantry_attempts = options.get('gantry_attempts', 100)
    reachable_range = options.get('reachable_range', (0.2, 2.4))
    sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    cartesian_move_group = GANTRY_ARM_GROUP
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

    flange_link_name = process.ROBOT_END_LINK
    touched_robot_links = []
    attached_object_base_link_name = None
 
    def sample_fn(beam_id: str, gripper_type: str):
        with pp.WorldSaver():
            end_t0cf_frame = target_frames_from_beam_id[beam_id][0]
            sample_found = False

            # Create attachments for the heldbeam
            client.add_attached_collision_mesh(
                AttachedCollisionMesh(CollisionMesh(None, beam_id),
                                      flange_link_name, touch_links=touched_robot_links),
                options={'robot': robot,
                         'attached_child_link_name': attached_object_base_link_name,
                         'parent_link_from_child_link_transformation' : beam_grasps[beam_id],
                         })

            # * do rejection sampling only for the robot body and the static obstacles
            gantry_base_gen_fn = gantry_base_generator(client, robot, end_t0cf_frame, reachable_range=reachable_range, scale=1.0, options=options)

            # cartesian_attempts = 2
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
                        # do Cartesian planning for the rest of the frames
                        trajectory = [full_conf]
                        for i in range(1, len(target_frames_from_beam_id[beam_id])):
                            line_segment = target_frames_from_beam_id[beam_id][i-1:i+1]
                            cart_traj = client.plan_cartesian_motion(robot, line_segment, start_configuration=trajectory[-1], group=cartesian_move_group, options=options)
                            if cart_traj is None:
                                # if any of the Cartesian planning fails, discard the whole sample
                                break
                            trajectory.append(cart_traj.points[-1])
                        # If all target frames are reached, return the trajectory
                        if len(trajectory) == len(target_frames_from_beam_id[beam_id]):
                            sample_found = True
                            from compas_fab.robots import JointTrajectory
                            trajectory = JointTrajectory(trajectory,trajectory[0].joint_names,full_conf,1)
                            LOGGER.debug('Cartesian plan sample found after {} gantry iters.'.format(gantry_iter))
                    if sample_found:
                        break
                if sample_found:
                    break


            # clean up the attached object and ACM
            client.detach_attached_collision_mesh(beam_id, options={})
            # if temp_name in client.extra_disabled_collision_links:
            #     del client.extra_disabled_collision_links[temp_name]

        # ! return None if one of the movement cannot find an IK solution
        if not sample_found:
            LOGGER.debug(colored('Cartesian plan sample NOT found after {} gantry iters.'.format(gantry_iter), 'red'))
        else:
            return (trajectory,)

    return sample_fn

##########################################

def get_test_fn_beam_assembly_collision_check(
        client: PyChoreoClient, 
        robot: Robot, 
        process: RobotClampAssemblyProcess,
        options=None):
    options = options or {}

    toolchanger = process.robot_toolchanger
    flange_from_toolchanger_base = toolchanger.t_t0cf_from_tcf
    beam_assembled_frames = {}
    beam_grasps = {}
    for beam_id in process.assembly.sequence:
        # Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue

        # Cache all beam's frame at asssembled position
        # assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(beam_id, 'assembly_wcf_final').copy()
        f_world_from_beam_final = process.assembly.get_beam_attribute(beam_id, 'assembly_wcf_final').copy()
        f_world_from_beam_final.point *= 1e-3
        beam_assembled_frames[beam_id] = f_world_from_beam_final

        # Cache all grasp transformations
        # ? different gripper might have different grasp for a beam?
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(beam_id)
        beam_gripper_id = process.assembly.get_beam_attribute(beam_id, "gripper_id")
        beam_gripper = process.tool(beam_gripper_id)
        flange_from_beam = flange_from_toolchanger_base * beam_gripper.t_t0cf_from_tcf * t_gripper_tcf_from_beam
        # scale the translation part of the transformation to meter
        for k in range(3):
            flange_from_beam[k,3] *= 1e-3

        beam_grasps[beam_id] = flange_from_beam

    flange_link_name = process.ROBOT_END_LINK
    touched_robot_links = []
    attached_object_base_link_name = None

    def test_fn(traj, heldbeam: str, otherbeam: str):
        # Returns: AssembleBeamNotInCollision
        # return True

        # set the otherbeam to the assembled position
        assemble_beam_not_in_collision = False
        with pp.WorldSaver():
            client.set_object_frame('^{}$'.format(otherbeam), beam_assembled_frames[otherbeam])

            # Create attachments for the heldbeam
            client.add_attached_collision_mesh(
                AttachedCollisionMesh(CollisionMesh(None, heldbeam),
                                      flange_link_name, touch_links=touched_robot_links),
                options={'robot': robot,
                         'attached_child_link_name': attached_object_base_link_name,
                         'parent_link_from_child_link_transformation' : beam_grasps[heldbeam],
                         })

            # if there is a joint between the heldbeam and the otherbeam, disable collision checking between them
            temp_name = '_disable_joint_collision_{}-{}'.format(otherbeam, heldbeam)
            if otherbeam in process.assembly.get_already_built_neighbors(heldbeam):
                otherbeam_bodies = client._get_bodies('^{}'.format(otherbeam))
                heldbeam_bodies = client._get_bodies('^{}'.format(heldbeam))
                for parent_body, child_body in product(otherbeam_bodies, heldbeam_bodies):
                    client.extra_disabled_collision_links[temp_name].add(
                        ((parent_body, None), (child_body, None))
                    )

            for conf in traj.points:
                # check robot and heldbeam collision with the otherbeam using FK
                # ! this checks a lot more than what we need here
                # so there are rooms for acceleration, but the code will get more complicated
                # options['diagnosis'] = True
                assemble_beam_not_in_collision = not client.check_collisions(robot, conf, options=options)
                # options['diagnosis'] = False

                if not assemble_beam_not_in_collision:
                    # early return if collision found
                    break

            # clean up the attached object and ACM
            client.detach_attached_collision_mesh(heldbeam, options={})
            if temp_name in client.extra_disabled_collision_links:
                del client.extra_disabled_collision_links[temp_name]

        return assemble_beam_not_in_collision

    return test_fn