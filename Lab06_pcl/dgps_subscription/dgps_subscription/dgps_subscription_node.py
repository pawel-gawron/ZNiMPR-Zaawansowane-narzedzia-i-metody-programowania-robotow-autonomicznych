#!/usr/bin/env python3

# Copyright 2023 Pawel_Gawron
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
try:
    from dgps_subscription.dgps_subscription import DgpsSubscription
except ImportError:
    from dgps_subscription import DgpsSubscription


class DgpsSubscriptionNode(Node):

    def __init__(self):
        super().__init__('dgps_subscription_node')
        self.dgps_subscription = DgpsSubscription()
        self.param_name = self.declare_parameter('param_name', 456).value
        self.dgps_subscription.foo(self.param_name)


def main(args=None):
    rclpy.init(args=args)
    node = DgpsSubscriptionNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
