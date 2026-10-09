# Software License Agreement (BSD)
#
# @author    Luis Batista <luisfelipewb@gmail.com>
# @copyright (c) 2025, Georgia Institute of Technology, All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Stand-in for the sail's Phidgets stepper, so kingfisher_sail's sail_controller runs unchanged.

Subscribes to ``phidgets_stepper/command`` (phidgets_msgs/StepperCommand) and moves a position
like the stepper does, ramping at ``acceleration``:

* STEP: to ``target``, at up to ``velocity``, slowing down to arrive;
* RUN: at ``velocity``; RUN at 0 ramps down and holds;
* STOP and DISENGAGED: ramp down and hold (the joint can't be freed).

The position goes to ``thrusters/sail/pos``, the sail's JointPositionController. A simulated
sail is calibrated from the start, so ``sail/calibrated`` (latched) is published once as true.

All topics are relative, so run the node inside the vehicle namespace.
"""

from math import copysign, sqrt

from phidgets_msgs.msg import StepperCommand
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64


class SailStepperSim(Node):

    def __init__(self):
        super().__init__('sail_stepper_sim')

        self.declare_parameter('rate', 50.0)
        # rad/s^2; robot.launch.py passes phidgets_stepper's from kingfisher.yaml
        self.declare_parameter('acceleration', 10.0)

        self.dt = 1.0 / self.get_parameter('rate').value
        self.accel = self.get_parameter('acceleration').value

        self.position = 0.0  # the joint's initial_position
        self.velocity = 0.0
        self.command = None

        self.pub = self.create_publisher(Float64, 'thrusters/sail/pos', 10)
        self.calibrated_pub = self.create_publisher(
            Bool, 'sail/calibrated',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.calibrated_pub.publish(Bool(data=True))
        self.create_subscription(
            StepperCommand, 'phidgets_stepper/command', self.command_callback, 10)
        self.create_timer(self.dt, self.update)

    def command_callback(self, msg: StepperCommand):
        self.command = msg

    def goal_velocity(self):
        cmd = self.command
        if cmd is None:
            return 0.0
        if cmd.mode == StepperCommand.CONTROL_MODE_STEP:
            error = cmd.target - self.position
            return copysign(min(abs(cmd.velocity), sqrt(2.0 * self.accel * abs(error))), error)
        if cmd.mode == StepperCommand.CONTROL_MODE_RUN:
            return cmd.velocity
        return 0.0

    def update(self):
        max_dv = self.accel * self.dt
        dv = max(-max_dv, min(max_dv, self.goal_velocity() - self.velocity))
        self.velocity += dv
        step = self.velocity * self.dt

        cmd = self.command
        if cmd is not None and cmd.mode == StepperCommand.CONTROL_MODE_STEP:
            error = cmd.target - self.position
            # Arrive exactly instead of oscillating around the target
            overshoots = step * error > 0.0 and abs(step) >= abs(error)
            if overshoots or (abs(error) < 1e-4 and abs(self.velocity) <= max_dv):
                self.position = cmd.target
                self.velocity = 0.0
                step = 0.0

        self.position += step
        self.pub.publish(Float64(data=self.position))


def main(args=None):
    rclpy.init(args=args)
    node = SailStepperSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
