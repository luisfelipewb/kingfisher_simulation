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
Translate normalized Kingfisher drive commands into thruster forces.

Subscribes to ``cmd_drive`` (kingfisher_msgs/Drive, left/right in [-1, 1]) and
publishes ``thrusters/left/thrust`` and ``thrusters/right/thrust``
(std_msgs/Float64, Newtons) for the Gazebo Thruster plugin, mimicking the real
MCU behaviour:

* the normalized command is rate limited to ``max_update_rate`` per second,
* the command is mapped to a force through a linearly interpolated table
  (deadband, non-linear response and saturation of the jet thrusters),
* if no ``cmd_drive`` arrives for ``1 / min_rate`` seconds the target is zeroed.

All topics are relative, so run the node inside the vehicle namespace.
"""

import numpy as np
import rclpy
from kingfisher_msgs.msg import Drive
from rclpy.node import Node
from std_msgs.msg import Float64

# Default force table: command sampled every 0.1 in [-1, 1] -> Newtons.
DEFAULT_FORCE_TABLE = [
    -4.0, -4.0, -4.0, -4.0, -2.0, -1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.5, 1.5, 4.75, 8.25, 16.0, 19.5, 19.5, 19.5,
]


class CmdDriveTranslator(Node):

    def __init__(self):
        super().__init__('cmd_drive_translate')

        self.declare_parameter('rate', 50.0)
        self.declare_parameter('min_rate', 5.0)
        self.declare_parameter('max_update_rate', 1.0)
        self.declare_parameter('force_table', DEFAULT_FORCE_TABLE)

        rate = self.get_parameter('rate').value
        min_rate = self.get_parameter('min_rate').value
        max_update_rate = self.get_parameter('max_update_rate').value
        self.force_table = np.asarray(self.get_parameter('force_table').value, dtype=float)
        if self.force_table.size < 2:
            raise ValueError('force_table needs at least two entries')
        self.cmd_grid = np.linspace(-1.0, 1.0, self.force_table.size)

        self.period = 1.0 / rate
        self.timeout = 1.0 / min_rate
        self.max_cmd_delta = self.period * max_update_rate

        self.target = np.zeros(2)   # [left, right] requested command
        self.current = np.zeros(2)  # [left, right] rate-limited command
        self.last_cmd_time = self.get_clock().now()

        self.pub_left = self.create_publisher(Float64, 'thrusters/left/thrust', 1)
        self.pub_right = self.create_publisher(Float64, 'thrusters/right/thrust', 1)
        self.create_subscription(Drive, 'cmd_drive', self.cmd_drive_callback, 1)
        self.create_timer(self.period, self.update)

        self.get_logger().info(
            f'rate {rate} Hz, timeout {self.timeout:.2f} s, '
            f'max delta {self.max_cmd_delta:.3f}/tick, '
            f'force range [{self.force_table.min()}, {self.force_table.max()}] N')

    def cmd_drive_callback(self, msg: Drive):
        self.last_cmd_time = self.get_clock().now()
        self.target[:] = np.clip([msg.left, msg.right], -1.0, 1.0)

    def force_from_cmd(self, cmd: float) -> float:
        return float(np.interp(cmd, self.cmd_grid, self.force_table))

    def update(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self.target[:] = 0.0

        delta = np.clip(self.target - self.current, -self.max_cmd_delta, self.max_cmd_delta)
        self.current += delta

        self.pub_left.publish(Float64(data=self.force_from_cmd(self.current[0])))
        self.pub_right.publish(Float64(data=self.force_from_cmd(self.current[1])))


def main(args=None):
    rclpy.init(args=args)
    node = CmdDriveTranslator()
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
