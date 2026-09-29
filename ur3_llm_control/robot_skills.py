import rclpy
import tf2_ros

from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory


class RobotSkills(Node):

    def __init__(self):
        super().__init__('robot_skills')

        self.valid_objects = [
            'red_cube',
            'yellow_cube',
            'blue_cube'
        ]

        self.valid_zones = [
            'zone_a',
            'zone_b',
            'zone_c'
        ]

        self.latest_joint_state = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.cartesian_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.get_logger().info(
            'Robot Skills initialized'
        )

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def get_current_joint_state(self):

        self.get_logger().info(
            'Waiting for /joint_states...'
        )

        for _ in range(50):

            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )

            if self.latest_joint_state is not None:
                break

        if self.latest_joint_state is None:

            self.get_logger().error(
                'No /joint_states received'
            )

            return None

        self.get_logger().info(
            'Current joint state received'
        )

        return self.latest_joint_state

    def get_tool_pose(self):

        self.get_logger().info(
            'Waiting for TF base_link -> tool0...'
        )

        try:

            for _ in range(50):

                rclpy.spin_once(
                    self,
                    timeout_sec=0.1
                )

                if self.tf_buffer.can_transform(
                    'base_link',
                    'tool0',
                    rclpy.time.Time()
                ):
                    break

            if not self.tf_buffer.can_transform(
                'base_link',
                'tool0',
                rclpy.time.Time()
            ):

                self.get_logger().error(
                    'TF base_link -> tool0 not available'
                )

                return None

            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time()
            )

            return transform.transform

        except Exception as e:

            self.get_logger().error(
                f'Cannot get TF: {e}'
            )

            return None

    def cartesian_move(
        self,
        dx=0.0,
        dy=0.0,
        dz=0.0
    ):

        self.get_logger().info(
            f'Cartesian move: '
            f'dx={dx}, dy={dy}, dz={dz}'
        )

        if not self.cartesian_client.wait_for_service(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                '/compute_cartesian_path unavailable'
            )

            return 'MOVEIT_UNAVAILABLE'

        if not self.execute_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                '/execute_trajectory unavailable'
            )

            return 'MOVEIT_UNAVAILABLE'

        joint_state = self.get_current_joint_state()

        if joint_state is None:
            return 'JOINT_STATE_ERROR'

        current = self.get_tool_pose()

        if current is None:
            return 'TF_ERROR'

        self.get_logger().info(
            'Current tool0: '
            f'x={current.translation.x:.6f}, '
            f'y={current.translation.y:.6f}, '
            f'z={current.translation.z:.6f}'
        )

        distance = (
            dx ** 2 +
            dy ** 2 +
            dz ** 2
        ) ** 0.5

        max_segment = 0.0005

        if distance == 0.0:
            segments = 1
        else:
            segments = max(
                1,
                int(
                    distance / max_segment
                    + 0.999999
                )
            )

        step_dx = dx / segments
        step_dy = dy / segments
        step_dz = dz / segments

        self.get_logger().info(
            f'Cartesian path divided into '
            f'{segments} segments'
        )

        for i in range(segments):

            self.get_logger().info(
                f'Cartesian segment '
                f'{i + 1}/{segments}'
            )

            target = Pose()

            target.position.x = (
                current.translation.x + step_dx
            )

            target.position.y = (
                current.translation.y + step_dy
            )

            target.position.z = (
                current.translation.z + step_dz
            )

            target.orientation = current.rotation

            request = GetCartesianPath.Request()

            request.header.frame_id = 'base_link'

            request.start_state = RobotState()

            request.start_state.joint_state = (
                joint_state
            )

            request.start_state.is_diff = True

            request.group_name = 'ur_manipulator'

            request.link_name = 'tool0'

            request.waypoints.append(target)

            request.max_step = 0.001

            request.jump_threshold = 0.0

            request.prismatic_jump_threshold = 0.0

            request.revolute_jump_threshold = 0.0

            request.avoid_collisions = False

            request.max_velocity_scaling_factor = 0.2

            request.max_acceleration_scaling_factor = 0.2

            self.get_logger().info(
                'Computing Cartesian path...'
            )

            future = (
                self.cartesian_client.call_async(
                    request
                )
            )

            rclpy.spin_until_future_complete(
                self,
                future
            )

            response = future.result()

            if response is None:

                self.get_logger().error(
                    'Cartesian service returned no response'
                )

                return 'FAILED'

            self.get_logger().info(
                f'Segment fraction = '
                f'{response.fraction:.3f}'
            )

            self.get_logger().info(
                f'Cartesian error code = '
                f'{response.error_code.val}'
            )

            if response.fraction < 0.99:

                self.get_logger().error(
                    f'Cartesian segment '
                    f'{i + 1} failed'
                )

                return 'CARTESIAN_FAILED'

            goal = ExecuteTrajectory.Goal()

            goal.trajectory = response.solution

            self.get_logger().info(
                'Executing Cartesian trajectory...'
            )

            send_future = (
                self.execute_client.send_goal_async(
                    goal
                )
            )

            rclpy.spin_until_future_complete(
                self,
                send_future
            )

            goal_handle = send_future.result()

            if goal_handle is None:

                self.get_logger().error(
                    'Could not send execution goal'
                )

                return 'FAILED'

            if not goal_handle.accepted:

                self.get_logger().error(
                    'Execution goal rejected'
                )

                return 'REJECTED'

            self.get_logger().info(
                'Execution goal accepted'
            )

            result_future = (
                goal_handle.get_result_async()
            )

            rclpy.spin_until_future_complete(
                self,
                result_future
            )

            result = (
                result_future.result().result
            )

            error_code = result.error_code.val

            self.get_logger().info(
                f'Execution result code = '
                f'{error_code}'
            )

            if error_code != 1:

                self.get_logger().error(
                    'Cartesian execution failed'
                )

                return 'FAILED'

            self.get_logger().info(
                f'Segment {i + 1} SUCCESS'
            )

            joint_state = (
                self.get_current_joint_state()
            )

            if joint_state is None:
                return 'JOINT_STATE_ERROR'

            current = self.get_tool_pose()

            if current is None:
                return 'TF_ERROR'

        self.get_logger().info(
            'CARTESIAN MOVE SUCCESS'
        )

        return 'SUCCESS'

    def home(self):

        self.get_logger().info(
            'SKILL home()'
        )

        return 'SUCCESS'

    def pick(self, object_name):

        if object_name not in self.valid_objects:

            self.get_logger().error(
                f'INVALID_OBJECT: {object_name}'
            )

            return 'INVALID_OBJECT'

        self.get_logger().info(
            f'SKILL pick({object_name})'
        )

        return 'SUCCESS'

    def place(self, object_name, zone):

        if object_name not in self.valid_objects:

            self.get_logger().error(
                f'INVALID_OBJECT: {object_name}'
            )

            return 'INVALID_OBJECT'

        if zone not in self.valid_zones:

            self.get_logger().error(
                f'INVALID_ZONE: {zone}'
            )

            return 'INVALID_ZONE'

        self.get_logger().info(
            f'SKILL place({object_name}, {zone})'
        )

        return 'SUCCESS'


def main(args=None):

    rclpy.init(args=args)

    node = RobotSkills()

    result = node.cartesian_move(
        dx=0.002,
        dy=0.0,
        dz=0.0
    )

    node.get_logger().info(
        f'CARTESIAN TEST RESULT: {result}'
    )

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
