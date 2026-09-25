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
Turn the sail at a commanded rate by moving its position target (jogging).

Subscribes to ``sail/cmd_rate`` (std_msgs/Float64, rad/s, + counter-clockwise
seen from above) and ``joint_states``, and publishes ``sail/cmd_angle``
(std_msgs/Float64, rad), the sail's position interface:

* while a non-zero rate arrives, the target starts at the measured
  ``sail_joint`` angle and advances at the rate, clamped to ``max_rate``, and
  never leads the measured angle by more than ``max_rate / min_rate``,
* when the rate goes to zero, or no ``sail/cmd_rate`` arrives for
  ``1 / min_rate`` seconds, the target is set once to the measured angle, so
  the sail stops where it is,
* otherwise nothing is published, and other ``sail/cmd_angle`` sources keep
  control.

All topics are relative, so run the node inside the vehicle namespace.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class SailJog(Node):

    def __init__(self):
        super().__init__('sail_jog')

        self.declare_parameter('rate', 20.0)
        self.declare_parameter('min_rate', 5.0)
        self.declare_parameter('max_rate', 1.0)
        self.declare_parameter('joint', 'sail_joint')

        self.period = 1.0 / self.get_parameter('rate').value
        min_rate = self.get_parameter('min_rate').value
        self.timeout = 1.0 / min_rate
        self.max_rate = self.get_parameter('max_rate').value
        self.max_lead = self.max_rate / min_rate
        self.joint = self.get_parameter('joint').value

        self.cmd_rate = 0.0
        self.last_cmd_time = None
        self.angle = None   # measured
        self.target = None  # None when not jogging

        self.pub = self.create_publisher(Float64, 'sail/cmd_angle', 10)
        self.create_subscription(Float64, 'sail/cmd_rate', self.cmd_rate_callback, 1)
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.create_timer(self.period, self.update)

        self.get_logger().info(
            f'max rate {self.max_rate} rad/s, timeout {self.timeout:.2f} s, '
            f'max lead {self.max_lead:.2f} rad')

    def cmd_rate_callback(self, msg: Float64):
        self.last_cmd_time = self.get_clock().now()
        self.cmd_rate = clamp(msg.data, -self.max_rate, self.max_rate)

    def joint_states_callback(self, msg: JointState):
        if self.joint in msg.name:
            self.angle = msg.position[msg.name.index(self.joint)]

    def update(self):
        if self.angle is None:
            return
        fresh = self.last_cmd_time is not None and \
            (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 <= self.timeout

        if fresh and self.cmd_rate != 0.0:
            if self.target is None:
                self.target = self.angle
            self.target = clamp(self.target + self.cmd_rate * self.period,
                                self.angle - self.max_lead, self.angle + self.max_lead)
            self.pub.publish(Float64(data=self.target))
        elif self.target is not None:
            self.target = None
            self.pub.publish(Float64(data=self.angle))


def main(args=None):
    rclpy.init(args=args)
    node = SailJog()
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
