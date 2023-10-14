from collections import defaultdict
from itertools import product
from termcolor import colored

import pybullet_planning as pp
from compas_fab.robots import Configuration, Robot, AttachedCollisionMesh, CollisionMesh, JointTrajectory, JointTrajectoryPoint, Duration
from compas_fab_pychoreo.client import PyChoreoClient
from compas_fab_pychoreo.conversions import pose_from_frame, pose_from_transformation, frame_from_pose
from compas_fab_pychoreo.backend_features.pychoreo_plan_cartesian_motion import plan_cartesian_motion_from_links

from integral_timber_joints.process import RobotClampAssemblyProcess, RoboticMovement
from integral_timber_joints.planning.robot_setup import GANTRY_ARM_GROUP
from integral_timber_joints.planning.state import gantry_base_generator
from integral_timber_joints.planning.stream import _get_sample_bare_arm_ik_fn
from integral_timber_joints.assembly import BeamAssemblyMethod
from integral_timber_joints.process.action import BeamPlacementWithClampsAction, BeamPlacementWithoutClampsAction, AssembleBeamWithScrewdriversAction, RoboticMovement, PlaceClampToStructureAction, PickClampFromStructureAction

from utils import LOGGER

def get_sample_fn_plan_motion_for_beam_assembly_stateless(client, robot, process, options=None):
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
    gantry_attempts = options.get('gantry_attempts', 500) #int(1e8)
    reachable_range = options.get('reachable_range', (0.2, 2.4))

    beam_target_poses = defaultdict(list)
    beam_grasps = {}
    beam_bodies = {}
    beam_gantry_sampler = {}
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

            if movement.target_frame is None:
                LOGGER.error(f'Target frame is None in {movement.short_summary}')

            beam_target_poses[beam_id].append(pose_from_frame(movement.target_frame, scale=1e-3))

        # * Cache all grasp transformations
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(beam_id)
        beam_gripper_id = process.assembly.get_beam_attribute(beam_id, "gripper_id")
        beam_gripper = process.tool(beam_gripper_id)
        flange_from_beam = flange_from_toolchanger_base * beam_gripper.t_t0cf_from_tcf * t_gripper_tcf_from_beam
        # scale the translation part of the transformation to meter
        for k in range(3):
            flange_from_beam[k,3] *= 1e-3
        beam_grasps[beam_id] = pose_from_transformation(flange_from_beam)
        beam_bodies[beam_id] = client._get_bodies('^{}$'.format(beam_id))[0]

        beam_gantry_sampler[beam_id] = gantry_base_generator(client, robot, frame_from_pose(beam_target_poses[beam_id][0]), reachable_range=reachable_range, scale=1.0, options=options)

    arm_sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

    robot_uid = client.get_robot_pybullet_uid(robot)
    flange_link_name = process.ROBOT_END_LINK
    tool_attach_link = pp.link_from_name(robot_uid, flange_link_name)

    cartesian_move_group = GANTRY_ARM_GROUP
    selected_links = [pp.link_from_name(robot_uid, l) for l in robot.get_link_names(group=cartesian_move_group)]
    selected_joint_names = robot.get_configurable_joint_names(group=cartesian_move_group)
    selected_joint_types = robot.get_joint_types_by_names(selected_joint_names)
    tool_link_name = robot.get_end_effector_link_name(group=cartesian_move_group)
    tool_link = pp.link_from_name(robot_uid, tool_link_name)

    static_obstacles = [] 
    for env_name in process.environment_models:
        static_obstacles.extend(client._get_bodies('^{}$'.format(env_name)))

    joint_custom_limits = options.get('joint_custom_limits', {})
    pb_custom_limits = {pp.joint_from_name(robot_uid, jn) : lims \
        for jn, lims in joint_custom_limits.items()}

    collision_distance_threshold = options.get('collision_distance_threshold', 0.0)
    max_distance = options.get('collision_buffer_distance_threshold', 0.0)

    body_name_from_id=client._name_from_body_id
    gantry_arm_joints = pp.joints_from_names(robot_uid, gantry_arm_joint_names)
    gantry_arm_links = pp.get_moving_links(robot_uid, gantry_arm_joints)
    robot_env_collision_fn = pp.get_collision_fn(robot_uid, gantry_arm_joints, obstacles=static_obstacles,
                                    attachments=[], 
                                    self_collisions=True,
                                    disabled_collisions=client.get_self_collision_link_ids(robot), # get disabled self-collision links (srdf)
                                    # extra_disabled_collisions=extra_disabled_collisions,
                                    custom_limits=pb_custom_limits,
                                    body_name_from_id=body_name_from_id,
                                    distance_threshold=collision_distance_threshold, max_distance=max_distance)

    diagnosis = options.get('diagnosis', False)
    # diagnosis = True 

    def traj_sampler_fn(heldbeam: str, gripper_type: str):
        # plan a motion to follow the target frames for inserting beam_id
        # while ensuring there is no collision between
        # - robot self collisions
        # - robot and the heldbeam

        # Create attachments for the heldbeam
        attachment = pp.Attachment(robot_uid, tool_attach_link, beam_grasps[heldbeam], beam_bodies[heldbeam])

        for gantry_iter, base_conf in zip(range(gantry_attempts), beam_gantry_sampler[heldbeam]):
            # * bare-arm IK sampler
            arm_conf_vals = arm_sample_ik_fn(beam_target_poses[heldbeam][0])

            # * iterate through all 6-axis IK solution
            for arm_conf_val in arm_conf_vals:
                if arm_conf_val is None:
                    continue
                start_conf_value = list(base_conf.joint_values) + list(arm_conf_val)

                # check collisions for this starting configuration among
                #     - robot self-collision 
                #     - between (robot links) and obstacles
                if robot_env_collision_fn(start_conf_value, diagnosis=diagnosis):
                    # LOGGER.debug(f'Cartesian plan {heldbeam}: robot env collision.')
                    continue
                    # break

                # check collisions between robot and the attached beam
                attachment.assign()
                if pp.any_link_pair_collision(robot_uid, gantry_arm_links, attachment.child):
                    # LOGGER.debug(f'Cartesian plan {heldbeam}: robot beam collision.')
                    if diagnosis:
                        cr = pp.any_link_pair_collision_info(robot_uid, gantry_arm_links, attachment.child)
                        pp.draw_collision_diagnosis(cr, body_name_from_id=body_name_from_id)
                    continue
                    # break

                path = plan_cartesian_motion_from_links(robot_uid, selected_links, tool_link,
                    beam_target_poses[heldbeam], custom_limits=pb_custom_limits, get_sub_conf=True, options=options)

                if path is None:
                    LOGGER.debug(f'Assembly plan {heldbeam}: no path found.')
                    continue

                # check collisions for each conf in the path
                path_in_collisions = False
                for conf_val in path[1:]:
                    if robot_env_collision_fn(conf_val, diagnosis=diagnosis):
                        path_in_collisions = True
                        break
                    # check collisions between robot and the attached beam
                    attachment.assign()
                    if pp.any_link_pair_collision(robot_uid, gantry_arm_links, attachment.child):
                        path_in_collisions = True
                        break
                    # TODO joint flip check

                if path_in_collisions:
                    LOGGER.debug(f'Assembly plan {heldbeam} path collision.')
                    continue
                else:
                    # convert path to trajectory
                    jt_traj_pts = []
                    for i, arm_conf_value in enumerate(path):
                        jt_traj_pt = JointTrajectoryPoint(joint_values=arm_conf_value, joint_names=gantry_arm_joint_names, joint_types=gantry_arm_joint_types)
                        jt_traj_pt.time_from_start = Duration(i*1,0)
                        jt_traj_pts.append(jt_traj_pt)
                    trajectory = JointTrajectory(trajectory_points=jt_traj_pts,
                        joint_names=jt_traj_pts[0].joint_names, start_configuration=jt_traj_pts[0], fraction=1.0)
                    LOGGER.debug(f'Assembly plan {heldbeam} sample found after {gantry_iter} gantry iters.')

                    # yield (trajectory,)
                    return (trajectory,)
        else:
            # LOGGER.debug(f'Cartesian plan running out of samples {gantry_iter}')
            raise ValueError(f'Assembly plan running out of samples {gantry_iter}')

    return traj_sampler_fn

##########################################

def get_test_fn_beam_assembly_collision_check_stateless(
        client: PyChoreoClient, 
        robot: Robot, 
        process: RobotClampAssemblyProcess,
        options=None):
    # check collisions between 
    # - robot    and the otherbeam
    options = options or {}

    toolchanger = process.robot_toolchanger
    flange_from_toolchanger_base = toolchanger.t_t0cf_from_tcf
    beam_assembled_poses = {}
    beam_grasps = {}
    beam_bodies = {}
    beam_neighbours = {}
    for beam_id in process.assembly.sequence:
        # Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue

        # Cache all beam's frame at asssembled position
        beam_assembled_poses[beam_id] = pose_from_frame(process.assembly.get_beam_attribute(beam_id, 'assembly_wcf_final'), scale=1e-3)

        # Cache all grasp transformations
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(beam_id)
        beam_gripper_id = process.assembly.get_beam_attribute(beam_id, "gripper_id")
        beam_gripper = process.tool(beam_gripper_id)
        flange_from_beam = flange_from_toolchanger_base * beam_gripper.t_t0cf_from_tcf * t_gripper_tcf_from_beam
        # scale the translation part of the transformation to meter
        for k in range(3):
            flange_from_beam[k,3] *= 1e-3

        beam_grasps[beam_id] = pose_from_transformation(flange_from_beam)
        beam_bodies[beam_id] = client._get_bodies('^{}$'.format(beam_id))[0]
        beam_neighbours[beam_id] = process.assembly.get_already_built_neighbors(beam_id)

    robot_uid = client.get_robot_pybullet_uid(robot)
    body_name_from_id=client._name_from_body_id
    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joints = pp.joints_from_names(robot_uid, gantry_arm_joint_names)
    gantry_arm_links = pp.get_moving_links(robot_uid, gantry_arm_joints)

    diagnosis = options.get('diagnosis', False)

    def test_fn(traj, heldbeam: str, otherbeam: str):
        # Returns: AssembleBeamNotInCollision

        assemble_beam_not_in_collision = True
        heldbeam_body = beam_bodies[heldbeam]
        otherbeam_body = beam_bodies[otherbeam]

        # set the otherbeam to the assembled position
        pp.set_pose(otherbeam_body, beam_assembled_poses[otherbeam])
        # attachment = pp.Attachment(robot_uid, tool_attach_link, beam_grasps[heldbeam], heldbeam_body)
        # ignore_beambeam_collisions = otherbeam in beam_neighbours[heldbeam]

        for conf in traj.points:
            client._set_body_configuration(robot_uid, conf)
            # attachment.assign()

            # * check between robot body and the otherbeam
            if pp.any_link_pair_collision(robot_uid, gantry_arm_links, otherbeam_body):
                # LOGGER.debug(f'Assembly {heldbeam} colliding with {otherbeam}')
                if diagnosis:
                # if True:
                    cr = pp.any_link_pair_collision_info(robot_uid, gantry_arm_links, otherbeam_body)
                    pp.draw_collision_diagnosis(cr, body_name_from_id=body_name_from_id)
                assemble_beam_not_in_collision = False
                break

            # * check between the heldbeam and the otherbeam
            # if not ignore_beambeam_collisions and pp.pairwise_collision(attachment.child, otherbeam_body):
            #     assemble_beam_not_in_collision = False
            #     break

        if not assemble_beam_not_in_collision:
            LOGGER.debug('Tested beam assembly IN COLLISION held {} - {} for {}'.format(heldbeam, otherbeam, traj))
        # else:
        #     LOGGER.debug('Tested beam assembly not in collision held {} - {}'.format(heldbeam, otherbeam))

        return assemble_beam_not_in_collision

    return test_fn

##########################################

def get_sample_fn_plan_motion_for_clamp_stateless(client, robot, process, operation: str, options=None):
    options = options or {}

    # Precompute and cache all target frames for each clamp's attach action
    toolchanger = process.robot_toolchanger

    # all clamp use the same grasp transformation from the tool changer
    clamp_grasp = pose_from_transformation(toolchanger.t_t0cf_from_tcf, scale=1e-3)

    gantry_attempts = options.get('gantry_attempts', int(1e8))
    reachable_range = options.get('reachable_range', (0.2, 2.4))

    joint_target_poses = defaultdict(list)
    joint_gantry_sampler = {}
    for i, beam_id in enumerate(process.assembly.sequence):
        #  Skip non clamped elements
        if process.assembly.get_assembly_method(beam_id) != BeamAssemblyMethod.CLAMPED:
            continue

        #  Iterate through the joints
        for neighbor_id in process.assembly.get_already_built_neighbors(beam_id):
            joint_id = (neighbor_id, beam_id)
            joint_clamp_type = process.assembly.get_joint_attribute(joint_id, 'tool_type')
            # any tool_id assignment is fine here as long as the tool_type is correct
            tool_id = process.assembly.get_joint_attribute(joint_id, 'tool_id')

            if operation == 'attach':
                action = PlaceClampToStructureAction(joint_id=joint_id, tool_type=joint_clamp_type, tool_id=tool_id)
            elif operation == 'detach':
                action = PickClampFromStructureAction(joint_id=joint_id, tool_type=joint_clamp_type, tool_id=tool_id)
            else:
                raise ValueError('operation must be either attach or detach')

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

                if movement.target_frame is None:
                    LOGGER.error(f'Target frame is None in {movement.short_summary}')

                joint_target_poses[joint_id].append(pose_from_frame(movement.target_frame, scale=1e-3))

            joint_gantry_sampler[joint_id] = gantry_base_generator(client, robot, frame_from_pose(joint_target_poses[joint_id][0]), reachable_range=reachable_range, scale=1.0, options=options)

    clamp_bodies = {}
    for clamp in process.clamps:
        clamp_bodies[clamp.name] = client._get_bodies('^{}$'.format(clamp.name))[0]

    arm_sample_ik_fn = _get_sample_bare_arm_ik_fn(client, robot)
    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joint_types = robot.get_joint_types_by_names(gantry_arm_joint_names)

    robot_uid = client.get_robot_pybullet_uid(robot)
    flange_link_name = process.ROBOT_END_LINK
    tool_attach_link = pp.link_from_name(robot_uid, flange_link_name)

    cartesian_move_group = GANTRY_ARM_GROUP
    selected_links = [pp.link_from_name(robot_uid, l) for l in robot.get_link_names(group=cartesian_move_group)]
    selected_joint_names = robot.get_configurable_joint_names(group=cartesian_move_group)
    selected_joint_types = robot.get_joint_types_by_names(selected_joint_names)
    tool_link_name = robot.get_end_effector_link_name(group=cartesian_move_group)
    tool_link = pp.link_from_name(robot_uid, tool_link_name)

    static_obstacles = [] 
    for env_name in process.environment_models:
        static_obstacles.extend(client._get_bodies('^{}$'.format(env_name)))

    joint_custom_limits = options.get('joint_custom_limits', {})
    pb_custom_limits = {pp.joint_from_name(robot_uid, jn) : lims \
        for jn, lims in joint_custom_limits.items()}

    collision_distance_threshold = options.get('collision_distance_threshold', 0.0)
    max_distance = options.get('collision_buffer_distance_threshold', 0.0)

    body_name_from_id=client._name_from_body_id
    gantry_arm_joints = pp.joints_from_names(robot_uid, gantry_arm_joint_names)
    gantry_arm_links = pp.get_moving_links(robot_uid, gantry_arm_joints)
    robot_env_collision_fn = pp.get_collision_fn(robot_uid, gantry_arm_joints, obstacles=static_obstacles,
                                    attachments=[], 
                                    self_collisions=True,
                                    disabled_collisions=client.get_self_collision_link_ids(robot), # get disabled self-collision links (srdf)
                                    # extra_disabled_collisions=extra_disabled_collisions,
                                    custom_limits=pb_custom_limits,
                                    body_name_from_id=body_name_from_id,
                                    distance_threshold=collision_distance_threshold, max_distance=max_distance)

    diagnosis = options.get('diagnosis', False)
    # diagnosis = True 


    def sample_fn(heldclamp: str, clamptype: str, beam1: str, beam2: str):
        # :inputs (?heldclamp ?clamptype ?beam1 ?beam2)
        # plan a motion to follow the target frames for inserting beam_id
        # while ensuring there is no collision between
        # - robot self collisions
        # - robot and the heldclamp
        joint_id = (beam1, beam2)

        # Create attachments for the heldbeam
        attachment = pp.Attachment(robot_uid, tool_attach_link, clamp_grasp, clamp_bodies[heldclamp])

        for gantry_iter, base_conf in zip(range(gantry_attempts), joint_gantry_sampler[joint_id]):
            # * bare-arm IK sampler
            arm_conf_vals = arm_sample_ik_fn(joint_target_poses[joint_id][0])

            # * iterate through all 6-axis IK solution
            for arm_conf_val in arm_conf_vals:
                if arm_conf_val is None:
                    continue
                start_conf_value = list(base_conf.joint_values) + list(arm_conf_val)

                # check collisions for this starting configuration among
                #     - robot self-collision 
                #     - between (robot links) and obstacles
                if robot_env_collision_fn(start_conf_value, diagnosis=diagnosis):
                    continue

                # check collisions between robot and the attached beam
                attachment.assign()
                
                if pp.any_link_pair_collision(robot_uid, gantry_arm_links, attachment.child):
                    if diagnosis:
                        cr = pp.any_link_pair_collision_info(robot_uid, gantry_arm_links, attachment.child)
                        pp.draw_collision_diagnosis(cr, body_name_from_id=body_name_from_id)
                    continue

                path = plan_cartesian_motion_from_links(robot_uid, selected_links, tool_link,
                    joint_target_poses[joint_id], custom_limits=pb_custom_limits, get_sub_conf=True, options=options)

                if path is None:
                    LOGGER.debug(f'Clamp {operation} plan {joint_id}: no path found.')
                    continue

                # check collisions for each conf in the path
                path_in_collisions = False
                for conf_val in path[1:]:
                    if robot_env_collision_fn(conf_val, diagnosis=diagnosis):
                        path_in_collisions = True
                        break
                    # check collisions between robot and the attached beam
                    attachment.assign()
                    if pp.any_link_pair_collision(robot_uid, gantry_arm_links, attachment.child):
                        path_in_collisions = True
                        break
                    # TODO joint flip check

                if path_in_collisions:
                    LOGGER.debug(f'Clamp {operation} plan {joint_id} path collision.')
                    continue
                else:
                    # convert path to trajectory
                    jt_traj_pts = []
                    for i, arm_conf_value in enumerate(path):
                        jt_traj_pt = JointTrajectoryPoint(joint_values=arm_conf_value, joint_names=gantry_arm_joint_names, joint_types=gantry_arm_joint_types)
                        jt_traj_pt.time_from_start = Duration(i*1,0)
                        jt_traj_pts.append(jt_traj_pt)
                    trajectory = JointTrajectory(trajectory_points=jt_traj_pts,
                        joint_names=jt_traj_pts[0].joint_names, start_configuration=jt_traj_pts[0], fraction=1.0)
                    LOGGER.debug(f'Clamp {operation} plan {joint_id} sample found after {gantry_iter} gantry iters.')

                    # yield (trajectory,)
                    return (trajectory,)
        else:
            # LOGGER.debug(f'Cartesian plan running out of samples {gantry_iter}')
            raise ValueError(f'Clamp {operation} plan {joint_id} running out of samples {gantry_iter}')

    return sample_fn

##########################################

def get_test_fn_clamp_beam_collision_check_stateless(
        client: PyChoreoClient, 
        robot: Robot, 
        process: RobotClampAssemblyProcess,
        options=None):
    options = options or {}

    toolchanger = process.robot_toolchanger
    flange_from_toolchanger_base = toolchanger.t_t0cf_from_tcf
    # all clamp use the same grasp transformation from the tool changer
    clamp_grasp = pose_from_transformation(toolchanger.t_t0cf_from_tcf, scale=1e-3)

    beam_assembled_poses = {}
    beam_bodies = {}
    beam_neighbours = {}
    for beam_id in process.assembly.sequence:
        # Skip scaffolding elements
        if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
            continue

        # Cache all beam's frame at asssembled position
        beam_assembled_poses[beam_id] = pose_from_frame(process.assembly.get_beam_attribute(beam_id, 'assembly_wcf_final'), scale=1e-3)

        # Cache all grasp transformations
        t_gripper_tcf_from_beam = process.assembly.get_t_gripper_tcf_from_beam(beam_id)
        beam_gripper_id = process.assembly.get_beam_attribute(beam_id, "gripper_id")
        beam_gripper = process.tool(beam_gripper_id)
        flange_from_beam = flange_from_toolchanger_base * beam_gripper.t_t0cf_from_tcf * t_gripper_tcf_from_beam
        # scale the translation part of the transformation to meter
        for k in range(3):
            flange_from_beam[k,3] *= 1e-3

        beam_bodies[beam_id] = client._get_bodies('^{}$'.format(beam_id))[0]
        beam_neighbours[beam_id] = process.assembly.get_already_built_neighbors(beam_id)

    clamp_bodies = {}
    for clamp in process.clamps:
        clamp_bodies[clamp.name] = client._get_bodies('^{}$'.format(clamp.name))[0]

    robot_uid = client.get_robot_pybullet_uid(robot)
    flange_link_name = process.ROBOT_END_LINK
    tool_attach_link = pp.link_from_name(robot_uid, flange_link_name)

    body_name_from_id=client._name_from_body_id
    gantry_arm_joint_names = robot.get_configurable_joint_names(group=GANTRY_ARM_GROUP)
    gantry_arm_joints = pp.joints_from_names(robot_uid, gantry_arm_joint_names)
    gantry_arm_links = pp.get_moving_links(robot_uid, gantry_arm_joints)

    diagnosis = options.get('diagnosis', False)

    def test_fn(heldclamp, beam1, beam2, traj, otherbeam):
        # (?heldclamp ?beam1 ?beam2 ?traj ?otherbeam)
        # Returns: ClampTrajNotInCollisionWithBeam

        clamp_traj_not_in_collision_with_beam = True
        heldclamp_body = clamp_bodies[heldclamp]
        otherbeam_body = beam_bodies[otherbeam]

        # set the otherbeam to the assembled position
        pp.set_pose(otherbeam_body, beam_assembled_poses[otherbeam])
        attachment = pp.Attachment(robot_uid, tool_attach_link, clamp_grasp, heldclamp_body)
        # ignore_beambeam_collisions = otherbeam in beam_neighbours[heldbeam]

        for conf in traj.points:
            client._set_body_configuration(robot_uid, conf)
            attachment.assign()

            # * check between robot body and the otherbeam
            if pp.any_link_pair_collision(robot_uid, gantry_arm_links, otherbeam_body):
                # LOGGER.debug(f'Clamp traj {heldclamp} : robot colliding with {otherbeam}at {(beam1, beam2)}')
                if diagnosis:
                # if True:
                    cr = pp.any_link_pair_collision_info(robot_uid, gantry_arm_links, otherbeam_body)
                    pp.draw_collision_diagnosis(cr, body_name_from_id=body_name_from_id)
                clamp_traj_not_in_collision_with_beam = False
                break

            # * check between the heldclamp and the otherbeam
            if otherbeam not in [beam1, beam2] and pp.pairwise_collision(attachment.child, otherbeam_body):
                # LOGGER.debug(f'Clamp traj {heldclamp} colliding with {otherbeam} at {(beam1, beam2)}')
                if diagnosis:
                # if True:
                    clamp_joints = pp.get_movable_joints(heldclamp_body)
                    clamp_movable_links = pp.get_moving_links(heldclamp_body, clamp_joints)
                    cr = pp.any_link_pair_collision_info(heldclamp_body, clamp_movable_links, otherbeam_body)
                    pp.draw_collision_diagnosis(cr, body_name_from_id=body_name_from_id)

                clamp_traj_not_in_collision_with_beam = False
                break

        if not clamp_traj_not_in_collision_with_beam:
            LOGGER.debug('Tested clamp {} at {} IN COLLISION held {} - for {}'.format(heldclamp, (beam1, beam2), otherbeam, traj))

        return clamp_traj_not_in_collision_with_beam

    return test_fn

    # #############

    # toolchanger = process.robot_toolchanger
    # flange_from_toolchanger_base = toolchanger.t_t0cf_from_tcf
    # beam_assembled_frames = {}
    # for beam_id in process.assembly.sequence:
    #     # Skip scaffolding elements
    #     if process.assembly.get_assembly_method(beam_id) == BeamAssemblyMethod.MANUAL_ASSEMBLY:
    #         continue

    #     # Cache all beam's frame at asssembled position
    #     # assembly_wcf_final = process.get_gripper_t0cp_for_beam_at(beam_id, 'assembly_wcf_final').copy()
    #     f_world_from_beam_final = process.assembly.get_beam_attribute(beam_id, 'assembly_wcf_final').copy()
    #     f_world_from_beam_final.point *= 1e-3
    #     beam_assembled_frames[beam_id] = f_world_from_beam_final

    # # all clamp use the same grasp transformation from the tool changer
    # clamp_grasp = toolchanger.t_t0cf_from_tcf.copy()
    # # scale the translation part of the transformation to meter
    # for k in range(3):
    #     clamp_grasp[k,3] *= 1e-3

    # flange_link_name = process.ROBOT_END_LINK
    # touched_robot_links = []
    # attached_object_base_link_name = None

    # # diagnosis = options.get('diagnosis', False)
    # diagnosis = True

    #     # set the otherbeam to the assembled position
    #     clamp_traj_not_in_collision_with_beam = False
    #     with pp.WorldSaver():
    #         client.set_object_frame('^{}$'.format(otherbeam), beam_assembled_frames[otherbeam])

    #         # Create attachments for the heldbeam
    #         client.add_attached_collision_mesh(
    #             AttachedCollisionMesh(CollisionMesh(None, heldclamp),
    #                                   flange_link_name, touch_links=touched_robot_links),
    #             options={'robot': robot,
    #                      'attached_child_link_name': attached_object_base_link_name,
    #                      'parent_link_from_child_link_transformation' : clamp_grasp,
    #                      })

    #         temp_name = 'clamp_beam_{}-{}'.format(heldclamp, otherbeam)
    #         if otherbeam == beam1 or otherbeam == beam2:
    #             otherbeam_bodies = client._get_bodies('^{}'.format(otherbeam))
    #             heldclamp_bodies = client._get_bodies('^{}'.format(heldclamp))
    #             for parent_body, child_body in product(otherbeam_bodies, heldclamp_bodies):
    #                 client.extra_disabled_collision_links[temp_name].add(
    #                     ((parent_body, None), (child_body, None))
    #                 )

    #         for conf in traj.points:
    #             # check robot and heldbeam collision with the otherbeam using FK
    #             # ! this checks a lot more than what we need here
    #             # so there are rooms for acceleration, but the code will get more complicated
    #             # options['diagnosis'] = True
    #             clamp_traj_not_in_collision_with_beam = not client.check_collisions(robot, conf, options=options)
    #             # options['diagnosis'] = False

    #             if not clamp_traj_not_in_collision_with_beam:
    #                 # early return if collision found
    #                 break

    #         # if not clamp_traj_not_in_collision_with_beam:
    #         #     LOGGER.debug('Testing clamp-beam IN COLLISION for held {} at ({},{}) - {}'.format(heldclamp, beam1, beam2, otherbeam))
    #         #     pp.wait_if_gui()

    #         # clean up the attached object and ACM
    #         client.detach_attached_collision_mesh(heldclamp, options={})
    #         if temp_name in client.extra_disabled_collision_links:
    #             del client.extra_disabled_collision_links[temp_name]

    #     if not clamp_traj_not_in_collision_with_beam:
    #         LOGGER.debug('Testing clamp-beam IN COLLISION for held {} at ({},{}) - {}'.format(heldclamp, beam1, beam2, otherbeam))
    #     else:
    #         LOGGER.debug('Testing clamp-beam not in collision for held {} at ({},{}) - {}'.format(heldclamp, beam1, beam2, otherbeam))

    #     return clamp_traj_not_in_collision_with_beam

    # return test_fn

