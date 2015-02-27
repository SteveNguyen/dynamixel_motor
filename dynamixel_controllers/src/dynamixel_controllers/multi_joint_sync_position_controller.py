# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division


__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'
__credits__ = 'Cara Slutter'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import rospy

from dynamixel_driver.dynamixel_const import *
from dynamixel_controllers.multi_joint_sync_controller import JointController

from dynamixel_msgs.msg import FastJointState as JointState
from dynamixel_msgs.msg import MotorCmdList


class JointPositionController(JointController):

    def __init__(self, dxl_io, controller_namespace, port_namespace):
        JointController.__init__(
            self, dxl_io, controller_namespace, port_namespace)

        # dict {joint_name:name, joint_speed:speed,
        # motor:{id:id,init:init,min:min,max:max}}

        self.joins = rospy.get_param(self.controller_namespace + '/joints')
        self.ids_list = []
        self.motor_name_id = dict()

        self.motor_id = dict()
        self.motor_name_id = dict()
        self.ids_list = dict()
        self.initial_position_raw = dict()
        self.min_angle_raw = dict()
        self.max_angle_raw = dict()
        self.flipped = dict()
        self.joint_state = dict()

        for j in self.joints:

            self.motor_id[j['joint_name']] = j['motor']['id']
            self.motor_name_id[j['motor']['id']] = j['joint_name']

            self.ids_list.append(j['motor']['id'])
            self.initial_position_raw[j['joint_name']] = j['motor']['init']

            self.min_angle_raw[j['joint_name']] = j['motor']['min']

            self.max_angle_raw[j['joint_name']] = j['motor']['max']

            self.flipped[j[
                'joint_name']] = self.min_angle_raw[j['joint_name']] > self.max_angle_raw[j['joint_name']]

            self.joint_state[j['joint_name']] = JointState(
                name=j['joint_name'], motor_ids=[self.motor_id[j['joint_name']]])

    def initialize(self):
        # verify that the expected motor is connected and responding
        available_ids = rospy.get_param(
            'dynamixel/%s/connected_ids' % self.port_namespace, [])

        self.RADIANS_PER_ENCODER_TICK = dict()
        self.ENCODER_TICKS_PER_RADIAN = dict()
        self.ENCODER_RESOLUTION = dict()
        self.MAX_POSITION = dict()
        self.VELOCITY_PER_TICK = dict()
        self.MIN_VELOCITY = dict()
        self.MAX_VELOCITY = dict()

        for motor_id in self.ids_list:

            if not motor_id in available_ids:
                rospy.logwarn(
                    'The specified motor id is not connected and responding.')
                rospy.logwarn('Available ids: %s' % str(available_ids))
                rospy.logwarn('Specified id: %d' % motor_id)
                return False

            self.RADIANS_PER_ENCODER_TICK[self.motor_name_id[motor_id]] = rospy.get_param(
                'dynamixel/%s/%d/radians_per_encoder_tick' % (self.port_namespace, motor_id))
            self.ENCODER_TICKS_PER_RADIAN[self.motor_name_id[motor_id]] = rospy.get_param(
                'dynamixel/%s/%d/encoder_ticks_per_radian' % (self.port_namespace, motor_id))

            if self.flipped[self.motor_name_id[motor_id]]:
                self.min_angle[self.motor_name_id[motor_id]] = (self.initial_position_raw[self.motor_name_id[motor_id]] - self.min_angle_raw[self.motor_name_id[motor_id]]) * \
                    self.RADIANS_PER_ENCODER_TICK[self.motor_name_id[motor_id]]
                self.max_angle[self.motor_name_id[motor_id]] = (
                    self.initial_position_raw[self.motor_name_id[motor_id]] - self.max_angle_raw[self.motor_name_id[motor_id]]) * self.RADIANS_PER_ENCODER_TICK[self.motor_name_id[motor_id]]
            else:
                self.min_angle[self.motor_name_id[motor_id]] = (
                    self.min_angle_raw[self.motor_name_id[motor_id]] - self.initial_position_raw[self.motor_name_id[motor_id]]) * self.RADIANS_PER_ENCODER_TICK[self.motor_name_id[motor_id]]
                self.max_angle[self.motor_name_id[motor_id]] = (
                    self.max_angle_raw[self.motor_name_id[motor_id]] - self.initial_position_raw[self.motor_name_id[motor_id]]) * self.RADIANS_PER_ENCODER_TICK[self.motor_name_id[motor_id]]

            self.ENCODER_RESOLUTION[self.motor_name_id[motor_id]] = rospy.get_param(
                'dynamixel/%s/%d/encoder_resolution' % (self.port_namespace, motor_id))
            self.MAX_POSITION[self.motor_name_id[motor_id]] = self.ENCODER_RESOLUTION[
                self.motor_name_id[motor_id]] - 1
            self.VELOCITY_PER_TICK[self.motor_name_id[motor_id]] = rospy.get_param(
                'dynamixel/%s/%d/radians_second_per_encoder_tick' % (self.port_namespace, motor_id))
            self.MAX_VELOCITY[self.motor_name_id[motor_id]] = rospy.get_param(
                'dynamixel/%s/%d/max_velocity' % (self.port_namespace, motor_id))
            self.MIN_VELOCITY[self.motor_name_id[motor_id]] = self.VELOCITY_PER_TICK[
                self.motor_name_id[motor_id]]

            # if self.compliance_slope[self.motor_name_id[motor_id]] is not None:
            #     self.set_compliance_slope(
            #         self.compliance_slope[self.motor_name_id[motor_id]])
            # if self.compliance_margin[self.motor_name_id[motor_id]] is not None:
            #     self.set_compliance_margin(
            #         self.compliance_margin[self.motor_name_id[motor_id]])
            # if self.compliance_punch[self.motor_name_id[motor_id]] is not None:
            #     self.set_compliance_punch(
            #         self.compliance_punch[self.motor_name_id[motor_id]])
            if self.torque_limit[self.motor_name_id[motor_id]] is not None:
                self.set_torque_limit(
                    self.torque_limit[self.motor_name_id[motor_id]])

            self.joint_max_speed[self.motor_name_id[motor_id]] = rospy.get_param(
                self.controller_namespace + '/joint_max_speed', self.MAX_VELOCITY[self.motor_name_id[motor_id]])

            if self.joint_max_speed[self.motor_name_id[motor_id]] < self.MIN_VELOCITY[self.motor_name_id[motor_id]]:
                self.joint_max_speed[self.motor_name_id[motor_id]] = self.MIN_VELOCITY[
                    self.motor_name_id[motor_id]]
            elif self.joint_max_speed[self.motor_name_id[motor_id]] > self.MAX_VELOCITY[self.motor_name_id[motor_id]]:
                self.joint_max_speed[self.motor_name_id[motor_id]] = self.MAX_VELOCITY[
                    self.motor_name_id[motor_id]]

            if self.joint_speed[self.motor_name_id[motor_id]] < self.MIN_VELOCITY[self.motor_name_id[motor_id]]:
                self.joint_speed[self.motor_name_id[motor_id]] = self.MIN_VELOCITY[
                    self.motor_name_id[motor_id]]
            elif self.joint_speed[self.motor_name_id[motor_id]] > self.joint_max_speed[self.motor_name_id[motor_id]]:
                self.joint_speed[self.motor_name_id[motor_id]] = self.joint_max_speed[
                    self.motor_name_id[motor_id]]

            self.set_speed(self.joint_speed[self.motor_name_id[motor_id]])

        return True

    def pos_rad_to_raw(self, pos_rad, name):
        if pos_rad < self.min_angle[name]:
            pos_rad = self.min_angle[name]
        elif pos_rad > self.max_angle[name]:
            pos_rad = self.max_angle[name]
        return self.rad_to_raw(pos_rad, self.initial_position_raw[name], self.flipped[name], self.ENCODER_TICKS_PER_RADIAN[name])

    def spd_rad_to_raw(self, spd_rad, name):
        if spd_rad < self.MIN_VELOCITY[name]:
            spd_rad = self.MIN_VELOCITY[name]
        elif spd_rad > self.joint_max_speed[name]:
            spd_rad = self.joint_max_speed[name]
        # velocity of 0 means maximum, make sure that doesn't happen
        return max(1, int(round(spd_rad / self.VELOCITY_PER_TICK[name])))

    def set_torque_enable(self, torque_enable, [name]):
        mcv = (self.motor_id[name], torque_enable)
        self.dxl_io.set_multi_torque_enabled([mcv])

    def set_speed(self, speed, name):
        mcv = (self.motor_id[name], self.spd_rad_to_raw(speed))
        self.dxl_io.set_multi_speed([mcv])

    # def set_compliance_slope(self, slope):
    #     if slope < DXL_MIN_COMPLIANCE_SLOPE:
    #         slope = DXL_MIN_COMPLIANCE_SLOPE
    #     elif slope > DXL_MAX_COMPLIANCE_SLOPE:
    #         slope = DXL_MAX_COMPLIANCE_SLOPE
    #     mcv = (self.motor_id, slope, slope)
    #     self.dxl_io.set_multi_compliance_slopes([mcv])

    # def set_compliance_margin(self, margin):
    #     if margin < DXL_MIN_COMPLIANCE_MARGIN:
    #         margin = DXL_MIN_COMPLIANCE_MARGIN
    #     elif margin > DXL_MAX_COMPLIANCE_MARGIN:
    #         margin = DXL_MAX_COMPLIANCE_MARGIN
    #     else:
    #         margin = int(margin)
    #     mcv = (self.motor_id, margin, margin)
    #     self.dxl_io.set_multi_compliance_margins([mcv])

    # def set_compliance_punch(self, punch):
    #     if punch < DXL_MIN_PUNCH:
    #         punch = DXL_MIN_PUNCH
    #     elif punch > DXL_MAX_PUNCH:
    #         punch = DXL_MAX_PUNCH
    #     else:
    #         punch = int(punch)
    #     mcv = (self.motor_id, punch)
    #     self.dxl_io.set_multi_punch([mcv])

    def set_torque_limit(self, max_torque, name):
        if max_torque > 1:
            max_torque = 1.0         # use all torque motor can provide
        elif max_torque < 0:
            max_torque = 0.0       # turn off motor torque
        raw_torque_val = int(DXL_MAX_TORQUE_TICK * max_torque)
        mcv = (self.motor_id[name], raw_torque_val)
        self.dxl_io.set_multi_torque_limit([mcv])

    # def set_acceleration_raw(self, acc):
    #     if acc < 0:
    #         acc = 0
    #     elif acc > 254:
    #         acc = 254
    #     self.dxl_io.set_acceleration(self.motor_id, acc)

    def process_motor_states(self, state_list):
        if self.running:

            for state in state_list:

                # self.joint_state.motor_temps = [state.temperature]
                # self.joint_state.goal_pos = self.raw_to_rad(
                    # state.goal, self.initial_position_raw, self.flipped,
                    # self.RADIANS_PER_ENCODER_TICK)
                self.joint_state[self.motor_name_id[state.id]].current_pos = self.raw_to_rad(
                    state.position, self.initial_position_raw[self.motor_name_id[state.id]], self.flipped[self.motor_name_id[state.id]], self.RADIANS_PER_ENCODER_TICK[self.motor_name_id[state.id]])
                # self.joint_state.error = state.error * \
                    # self.RADIANS_PER_ENCODER_TICK
                self.joint_state[
                    self.motor_name_id[state.id]].error = state.error
                # self.joint_state.velocity = state.speed * \
                    # self.VELOCITY_PER_TICK
                # self.joint_state.load = state.load
                # self.joint_state.is_moving = state.moving
                self.joint_state[self.motor_name_id[state.id]].header.stamp = rospy.Time.from_sec(
                    state.timestamp)

                self.joint_state_pub[self.motor_name_id[state.id]].publish(
                    self.joint_state[self.motor_name_id[state.id]])

    def process_command(self, msg):

        mcv = []
        for cmd in msg:
            angle = cmd.angle
            mcv + = (self.motor_id[cmd.name], self.pos_rad_to_raw(angle, cmd.name))
        self.dxl_io.set_multi_position([mcv])
