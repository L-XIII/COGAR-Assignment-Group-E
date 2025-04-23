#!/usr/bin/env python

import rospy
import math
import random
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import Point, Vector3


class ForceSensor:
    """
    The ForceSensor class provides force feedback during manipulation tasks.
    It sends information to SafetyMonitor and GripperController components.
    """

    def __init__(self):
        self.force_threshold = 10.0  # Newtons
        self.current_force = 0.0

        # Publisher for force data
        self.force_publisher = rospy.Publisher("force_data", Float32, queue_size=10)

    def measure_force(self):
        """
        Simulates force measurement during manipulation tasks
        Returns current force value in Newtons
        """
        # In a real system, this would read from actual force sensors
        # For simulation, we generate simulated force readings
        base_force = 2.0  # baseline force
        noise = random.uniform(-1.0, 1.0)
        self.current_force = base_force + noise

        # Publish force data
        self.force_publisher.publish(self.current_force)

        return self.current_force

    def detect_collision(self):
        """
        Detects if the force exceeds the safe threshold
        Returns True if collision is detected, False otherwise
        """
        return self.current_force > self.force_threshold

    def reset(self):
        """
        Reset force sensor readings
        """
        self.current_force = 0.0


class GripperMotors:
    """
    The GripperMotors class controls the physical motors of the gripper.
    It receives control commands from GripperController.
    """

    def __init__(self):
        self.position = 0.0  # 0.0 = fully open, 1.0 = fully closed
        self.max_force = 15.0  # Newtons
        self.status = "idle"  # idle, moving, holding

        # Subscriber for gripper commands
        rospy.Subscriber("gripper_commands", Float32, self.receive_command)

    def receive_command(self, command_msg):
        """
        Receives and processes motor commands
        """
        target_position = command_msg.data
        self.move_to_position(target_position)

    def move_to_position(self, target_position):
        """
        Moves the gripper to the target position
        Returns success of the movement operation
        """
        # In a real system, this would send commands to actual motors
        self.status = "moving"

        # Simulate movement time
        # In a real implementation, this would be handled by a control loop
        rospy.sleep(0.5)  # Simulate movement time

        # Simulate success with 95% probability
        success = random.random() < 0.95

        if success:
            self.position = target_position
            self.status = "holding" if target_position > 0.5 else "idle"
            return True
        else:
            self.status = "error"
            return False

    def get_status(self):
        """
        Returns the current status of the gripper motors
        """
        return {"position": self.position, "status": self.status}


class GripperController:
    """
    The GripperController class coordinates gripper operations.
    It receives information from ForceSensor and GraspCommand from ManipulationSupervisor.
    It sends control signals to GripperMotors.
    """

    def __init__(self, force_sensor=None, gripper_motors=None):
        self.force_sensor = force_sensor
        self.gripper_motors = gripper_motors
        self.grip_strength = 0.7  # Default grip strength (0-1)
        self.gripping_object = False

        # Publishers and subscribers
        self.grip_command_publisher = rospy.Publisher(
            "gripper_commands", Float32, queue_size=10
        )
        rospy.Subscriber("grasp_commands", String, self.receive_grasp_command)
        rospy.Subscriber("force_data", Float32, self.monitor_force)

    def receive_grasp_command(self, command_msg):
        """
        Processes grasp commands from ManipulationSupervisor
        """
        command = command_msg.data

        if command == "grasp":
            self.execute_grasp()
        elif command == "release":
            self.execute_release()
        elif command.startswith("adjust_"):
            # Commands like "adjust_stronger" or "adjust_gentler"
            self._adjust_grip_strength(command)

    def monitor_force(self, force_msg):
        """
        Monitors force data to adjust gripper behavior
        """
        force = force_msg.data

        # If force exceeds safe threshold, loosen grip
        if force > 12.0 and self.gripping_object:
            self.grip_strength = max(0.5, self.grip_strength - 0.1)
            self._send_grip_command(self.grip_strength)

    def execute_grasp(self):
        """
        Executes a grasp operation
        Returns success status
        """
        # Close gripper to grasp object
        success = self._send_grip_command(self.grip_strength)

        if success:
            self.gripping_object = True
            rospy.loginfo("Object grasped successfully")
        else:
            rospy.logwarn("Failed to grasp object")

        return success

    def execute_release(self):
        """
        Executes a release operation
        Returns success status
        """
        # Open gripper to release object
        success = self._send_grip_command(0.0)

        if success:
            self.gripping_object = False
            rospy.loginfo("Object released successfully")
        else:
            rospy.logwarn("Failed to release object")

        return success

    def _send_grip_command(self, position):
        """
        Sends grip command to gripper motors
        """
        # Ensure position is within valid range
        position = max(0.0, min(1.0, position))

        # Send command
        self.grip_command_publisher.publish(position)

        # In a real system, we would wait for feedback
        # For simulation, we assume 90% success rate
        return random.random() < 0.9

    def _adjust_grip_strength(self, command):
        """
        Adjusts grip strength based on command
        """
        if command == "adjust_stronger":
            self.grip_strength = min(1.0, self.grip_strength + 0.1)
        elif command == "adjust_gentler":
            self.grip_strength = max(0.3, self.grip_strength - 0.1)

        # If already gripping, adjust current grip
        if self.gripping_object:
            self._send_grip_command(self.grip_strength)


class JointsMotors:
    """
    The JointsMotors class controls the physical motors of the robot arm joints.
    It receives control commands from ArmController and sends MotorsFeedback to SafetyMonitor.
    """

    def __init__(self):
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 7-DOF arm
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.status = "idle"

        # Publishers for feedback
        self.feedback_publisher = rospy.Publisher(
            "motors_feedback", String, queue_size=10
        )

        # Subscribers for commands
        rospy.Subscriber("joint_commands", String, self.receive_command)

    def receive_command(self, command_msg):
        """
        Receives and processes joint commands
        Format: "position:j1,j2,j3,j4,j5,j6,j7"
        """
        command = command_msg.data

        if command.startswith("position:"):
            position_data = command.replace("position:", "")
            try:
                joint_values = [float(x) for x in position_data.split(",")]
                if len(joint_values) == 7:
                    self.move_to_position(joint_values)
            except ValueError:
                rospy.logwarn("Invalid joint position command format")

    def move_to_position(self, target_positions):
        """
        Moves joints to target positions
        Returns success status
        """
        # In a real system, this would send commands to actual motors
        self.status = "moving"

        # Publish status update
        self.feedback_publisher.publish("status:moving")

        # Simulate movement time (would be handled by control loop in reality)
        rospy.sleep(0.8)  # Simulate movement time

        # Simulate success with 95% probability
        success = random.random() < 0.95

        if success:
            self.joint_positions = target_positions
            self.status = "idle"

            # Simulate some feedback data
            self._generate_feedback_data()

            # Publish success feedback
            self.feedback_publisher.publish("status:completed")
            return True
        else:
            self.status = "error"
            self.feedback_publisher.publish("status:error")
            return False

    def _generate_feedback_data(self):
        """
        Generates and publishes simulated feedback data
        """
        # Generate simulated motor feedback
        for i in range(len(self.joint_positions)):
            # Add small random noise to position
            noise = random.uniform(-0.01, 0.01)
            self.joint_positions[i] += noise

            # Simulate velocities and torques
            self.joint_velocities[i] = random.uniform(-0.05, 0.05)
            self.joint_torques[i] = random.uniform(0.5, 2.0)

        # Publish feedback
        feedback_msg = f"data:{','.join([str(p) for p in self.joint_positions])}"
        self.feedback_publisher.publish(feedback_msg)

    def get_status(self):
        """
        Returns the current status of joints
        """
        return {
            "positions": self.joint_positions,
            "velocities": self.joint_velocities,
            "torques": self.joint_torques,
            "status": self.status,
        }


class ArmController:
    """
    The ArmController class manages arm movements and trajectories.
    It receives Trajectory information from ManipulationSupervisor and safety data from SafetyMonitor.
    It sends control commands to JointsMotors.
    """

    def __init__(self, joints_motors=None):
        self.joints_motors = joints_motors
        self.current_trajectory = None
        self.execution_status = "idle"  # idle, executing, completed, aborted
        self.safety_status = "normal"  # normal, warning, critical

        # Publishers and subscribers
        self.command_publisher = rospy.Publisher(
            "joint_commands", String, queue_size=10
        )
        rospy.Subscriber("trajectory_commands", String, self.receive_trajectory)
        rospy.Subscriber("safety_status", String, self.receive_safety_status)
        rospy.Subscriber("motors_feedback", String, self.receive_motor_feedback)

    def receive_trajectory(self, trajectory_msg):
        """
        Receives trajectory commands from ManipulationSupervisor
        """
        trajectory_data = trajectory_msg.data

        # Parse trajectory data
        # Format: "trajectory:waypoint1|waypoint2|waypoint3"
        # Each waypoint: "x,y,z,roll,pitch,yaw"
        try:
            if trajectory_data.startswith("trajectory:"):
                trajectory_str = trajectory_data.replace("trajectory:", "")
                waypoints = [wp.split(",") for wp in trajectory_str.split("|")]

                # Convert to float values
                self.current_trajectory = [
                    [float(coord) for coord in wp] for wp in waypoints if len(wp) == 6
                ]

                # Start trajectory execution
                self.execute_trajectory()

        except ValueError:
            rospy.logwarn("Invalid trajectory format")
            self.execution_status = "aborted"

    def receive_safety_status(self, status_msg):
        """
        Receives safety status updates from SafetyMonitor
        """
        self.safety_status = status_msg.data

        # If safety critical, abort current trajectory
        if self.safety_status == "critical" and self.execution_status == "executing":
            self.abort_trajectory()

    def receive_motor_feedback(self, feedback_msg):
        """
        Receives feedback from JointsMotors
        """
        feedback = feedback_msg.data

        if feedback.startswith("status:"):
            motor_status = feedback.replace("status:", "")

            # Update execution status based on motor feedback
            if motor_status == "completed" and self.execution_status == "executing":
                self.execution_status = "completed"
                rospy.loginfo("Trajectory execution completed")
            elif motor_status == "error":
                self.execution_status = "aborted"
                rospy.logwarn("Trajectory execution aborted due to motor error")

    def execute_trajectory(self):
        """
        Executes the current trajectory
        """
        if not self.current_trajectory or self.safety_status == "critical":
            self.execution_status = "aborted"
            return False

        self.execution_status = "executing"
        rospy.loginfo("Starting trajectory execution")

        # In a real system, we'd use a control loop
        # For simulation, we'll execute each waypoint sequentially
        for waypoint_idx, waypoint in enumerate(self.current_trajectory):
            # Check safety status before proceeding
            if self.safety_status == "critical":
                self.abort_trajectory()
                return False

            # Convert cartesian waypoint to joint positions
            # In a real system, we'd use inverse kinematics
            # For simulation, we'll use a simplified approach
            joint_positions = self._simplified_inverse_kinematics(waypoint)

            # Send command to joints motors
            command = f"position:{','.join([str(jp) for jp in joint_positions])}"
            self.command_publisher.publish(command)

            # Wait for completion before proceeding to next waypoint
            # In a real system, we'd use feedback to determine completion
            rospy.sleep(1.0)  # Simulate execution time

            # Log progress
            rospy.loginfo(
                f"Waypoint {waypoint_idx + 1}/{len(self.current_trajectory)} executed"
            )

        self.execution_status = "completed"
        return True

    def abort_trajectory(self):
        """
        Aborts the current trajectory execution
        """
        if self.execution_status == "executing":
            rospy.logwarn("Aborting trajectory execution")

            # Stop movement - send command to move to current position
            # In a real system, we might use a smoother approach
            if self.joints_motors:
                current_positions = self.joints_motors.joint_positions
                command = f"position:{','.join([str(jp) for jp in current_positions])}"
                self.command_publisher.publish(command)

            self.execution_status = "aborted"

    def _simplified_inverse_kinematics(self, cartesian_point):
        """
        Simplified inverse kinematics computation for simulation
        In a real system, this would be a proper IK algorithm
        """
        x, y, z, roll, pitch, yaw = cartesian_point

        # This is a very simplified approximation
        # In a real robot, this would be a proper IK calculation
        j1 = math.atan2(y, x)
        r_xy = math.sqrt(x**2 + y**2)
        j2 = math.atan2(z, r_xy)
        j3 = math.sin(roll) * 0.5
        j4 = math.sin(pitch) * 0.5
        j5 = math.sin(yaw) * 0.5
        j6 = r_xy * 0.1
        j7 = z * 0.1

        # Add some noise to simulate realistic IK
        noise = 0.05
        joint_positions = [
            j1 + random.uniform(-noise, noise),
            j2 + random.uniform(-noise, noise),
            j3 + random.uniform(-noise, noise),
            j4 + random.uniform(-noise, noise),
            j5 + random.uniform(-noise, noise),
            j6 + random.uniform(-noise, noise),
            j7 + random.uniform(-noise, noise),
        ]

        return joint_positions


class SafetyMonitor:
    """
    The SafetyMonitor class monitors the safety of manipulation operations.
    It receives MotorsFeedback from JointsMotors and force data from ForceSensor.
    It sends safety information to ManipulationSupervisor and ArmController.
    """

    def __init__(self, force_sensor=None):
        self.force_sensor = force_sensor
        self.safety_status = "normal"  # normal, warning, critical
        self.joint_limits = {
            "position": [-2.0, 2.0],  # radians, for all joints
            "velocity": [-1.0, 1.0],  # radians/s, for all joints
            "torque": [-10.0, 10.0],  # Nm, for all joints
        }

        # Publishers
        self.safety_publisher = rospy.Publisher("safety_status", String, queue_size=10)

        # Subscribers
        rospy.Subscriber("motors_feedback", String, self.process_motors_feedback)
        rospy.Subscriber("force_data", Float32, self.process_force_data)

    def process_motors_feedback(self, feedback_msg):
        """
        Processes feedback from JointsMotors to detect safety issues
        """
        feedback = feedback_msg.data

        if feedback.startswith("data:"):
            joint_data = feedback.replace("data:", "")
            try:
                joint_positions = [float(x) for x in joint_data.split(",")]
                self._check_joint_limits(joint_positions)
            except ValueError:
                rospy.logwarn("Invalid joint data format")

    def process_force_data(self, force_msg):
        """
        Processes force data to detect potential collisions
        """
        force = force_msg.data

        # Update safety status based on force readings
        if force > 12.0:
            self.update_safety_status("warning", f"High force detected: {force}N")
        elif force > 15.0:
            self.update_safety_status("critical", f"Critical force detected: {force}N")
        else:
            # Only reset to normal if current status is warning due to force
            if self.safety_status == "warning":
                self.update_safety_status("normal", "Force returned to normal range")

    def _check_joint_limits(self, joint_positions):
        """
        Checks if joint positions are within safe limits
        """
        min_pos, max_pos = self.joint_limits["position"]

        for i, pos in enumerate(joint_positions):
            if pos < min_pos or pos > max_pos:
                self.update_safety_status(
                    "critical", f"Joint {i + 1} position out of bounds: {pos}"
                )
                return

    def update_safety_status(self, status, message):
        """
        Updates and publishes safety status
        """
        # Only update if new status is more critical than current
        status_priority = {"normal": 0, "warning": 1, "critical": 2}

        if status_priority.get(status, 0) >= status_priority.get(self.safety_status, 0):
            self.safety_status = status

            # Log message
            if status == "warning":
                rospy.logwarn(f"Safety warning: {message}")
            elif status == "critical":
                rospy.logerr(f"Safety critical: {message}")
            else:
                rospy.loginfo(f"Safety normal: {message}")

            # Publish status
            self.safety_publisher.publish(status)

    def is_operation_safe(self):
        """
        Checks if current state is safe for operations
        """
        return self.safety_status != "critical"


class ManipulationSupervisor:
    """
    The ManipulationSupervisor class coordinates the overall manipulation process.
    It receives TargetDishPosition from ReasoningController and information from SafetyMonitor.
    It sends Trajectory to ArmController, GraspCommand to GripperController,
    and ManipulationStatus to Navigation's SLAM component.
    """

    def __init__(
        self,
        arm_controller=None,
        gripper_controller=None,
        safety_monitor=None,
        tiago_platform=None,
    ):
        self.arm_controller = arm_controller
        self.gripper_controller = gripper_controller
        self.safety_monitor = safety_monitor
        self.tiago = tiago_platform

        self.target_position = None
        self.manipulation_status = (
            "idle"  # idle, moving, grasping, placing, completed, failed
        )

        # Publishers
        self.status_publisher = rospy.Publisher(
            "manipulation_status", String, queue_size=10
        )
        self.trajectory_publisher = rospy.Publisher(
            "trajectory_commands", String, queue_size=10
        )
        self.grasp_publisher = rospy.Publisher("grasp_commands", String, queue_size=10)

        # Subscribers
        rospy.Subscriber("target_dish_position", Point, self.receive_target_position)
        rospy.Subscriber("safety_status", String, self.receive_safety_status)
        rospy.Subscriber("perception_data", String, self.receive_perception_data)

    def receive_target_position(self, position_msg):
        """
        Receives target dish position from ReasoningController
        """
        self.target_position = [position_msg.x, position_msg.y, position_msg.z]
        rospy.loginfo(f"Received target position: {self.target_position}")

        # Start manipulation sequence
        self.plan_and_execute_manipulation()

    def receive_safety_status(self, status_msg):
        """
        Receives safety status updates
        """
        safety_status = status_msg.data

        # If critical safety issue occurs during manipulation, abort
        if safety_status == "critical" and self.manipulation_status != "idle":
            self.abort_manipulation("Safety critical condition detected")

    def receive_perception_data(self, perception_msg):
        """
        Processes perception data for object recognition
        """
        # In a real system, this would process detailed perception data
        # For simulation, we'll just acknowledge receipt
        rospy.loginfo("Received perception data for manipulation")

    def plan_and_execute_manipulation(self):
        """
        Plans and executes the manipulation sequence
        """
        if not self.target_position or not self.safety_monitor.is_operation_safe():
            self.update_status(
                "failed",
                "Cannot plan manipulation - invalid target or unsafe condition",
            )
            return False

        self.update_status("moving", "Planning arm trajectory")

        # Plan approach trajectory
        approach_trajectory = self._plan_trajectory("approach")

        # Send trajectory to arm controller
        if approach_trajectory:
            trajectory_msg = f"trajectory:{approach_trajectory}"
            self.trajectory_publisher.publish(trajectory_msg)

            # Wait for completion
            # In a real system, we'd use callbacks or services
            rospy.sleep(2.0)  # Simulate waiting for completion

            # Execute grasp
            self.execute_grasp()

            # Plan retreat trajectory
            retreat_trajectory = self._plan_trajectory("retreat")

            if retreat_trajectory:
                trajectory_msg = f"trajectory:{retreat_trajectory}"
                self.trajectory_publisher.publish(trajectory_msg)

                # Wait for completion
                rospy.sleep(2.0)  # Simulate waiting for completion

                self.update_status(
                    "completed", "Manipulation sequence completed successfully"
                )
                return True

        self.update_status("failed", "Failed to plan or execute manipulation")
        return False

    def execute_grasp(self):
        """
        Executes grasp or release operation
        """
        if self.manipulation_status == "moving":
            # We're approaching to grasp
            self.update_status("grasping", "Executing grasp operation")
            self.grasp_publisher.publish("grasp")
        else:
            # We're approaching to place
            self.update_status("placing", "Executing release operation")
            self.grasp_publisher.publish("release")

        # Wait for completion
        rospy.sleep(1.0)  # Simulate waiting for completion

    def abort_manipulation(self, reason):
        """
        Aborts the current manipulation operation
        """
        rospy.logwarn(f"Aborting manipulation: {reason}")

        # Stop arm movement
        if self.arm_controller:
            self.arm_controller.abort_trajectory()

        self.update_status("failed", f"Manipulation aborted: {reason}")

    def update_status(self, status, message):
        """
        Updates and publishes manipulation status
        """
        self.manipulation_status = status

        # Log message
        if status == "failed":
            rospy.logwarn(f"Manipulation failed: {message}")
        else:
            rospy.loginfo(f"Manipulation status: {status} - {message}")

        # Publish status update
        self.status_publisher.publish(status)

    def _plan_trajectory(self, trajectory_type):
        """
        Plans trajectory for approach or retreat
        Returns a string representation of the trajectory
        """
        if trajectory_type == "approach":
            # Plan approach to target position
            # In a real system, this would use motion planning
            # For simulation, we'll create a simple linear approach

            # Assume current position is origin
            current_position = [0.0, 0.0, 0.2]

            # Generate intermediate waypoints
            waypoint1 = self._interpolate_position(
                current_position, self.target_position, 0.3
            )
            waypoint2 = self._interpolate_position(
                current_position, self.target_position, 0.7
            )

            # Add orientation information (roll, pitch, yaw)
            waypoint1.extend([0.0, 0.0, 0.0])
            waypoint2.extend([0.0, 0.0, 0.0])
            self.target_position.extend([0.0, 0.0, 0.0])

            # Create trajectory string
            trajectory = f"{','.join([str(x) for x in waypoint1])}|{','.join([str(x) for x in waypoint2])}|{','.join([str(x) for x in self.target_position])}"

            return trajectory

        elif trajectory_type == "retreat":
            # Plan retreat from target position
            # For simulation, move upward then back

            # Add orientation information if not already present
            if len(self.target_position) == 3:
                self.target_position.extend([0.0, 0.0, 0.0])

            # Create waypoints for retreat
            retreat_point1 = list(self.target_position)
            retreat_point1[2] += 0.1  # Move up

            retreat_point2 = list(retreat_point1)
            retreat_point2[0] -= 0.2  # Move back

            # Create trajectory string
            trajectory = f"{','.join([str(x) for x in retreat_point1])}|{','.join([str(x) for x in retreat_point2])}"

            return trajectory

        return None

    def _interpolate_position(self, start_pos, end_pos, factor):
        """
        Interpolates between two positions by a factor
        """
        result = []
        for i in range(min(len(start_pos), len(end_pos))):
            result.append(start_pos[i] + (end_pos[i] - start_pos[i]) * factor)
        return result


class ManipulationSystem:
    """
    ManipulationSystem is the main interface for the manipulation subsystem.
    It integrates all manipulation components and provides a unified interface.
    """

    _instance = None

    def __new__(cls, tiago_platform=None):
        """
        Create a singleton instance of ManipulationSystem
        """
        if cls._instance is None:
            cls._instance = super(ManipulationSystem, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, tiago_platform=None):
        """
        Initialize the manipulation system with all its components
        """
        if not hasattr(self, "_initialized") or not self._initialized:
            self.tiago = tiago_platform

            # Initialize components
            self.force_sensor = ForceSensor()
            self.gripper_motors = GripperMotors()
            self.joints_motors = JointsMotors()

            self.gripper_controller = GripperController(
                self.force_sensor, self.gripper_motors
            )
            self.safety_monitor = SafetyMonitor(self.force_sensor)
            self.arm_controller = ArmController(self.joints_motors)

            self.manipulation_supervisor = ManipulationSupervisor(
                self.arm_controller,
                self.gripper_controller,
                self.safety_monitor,
                self.tiago,
            )

            self._initialized = True

    def execute_manipulation(self, target_position):
        """
        Main method to execute a manipulation task
        """
        # Convert target position to Point message
        target_point = Point()
        target_point.x = target_position[0]
        target_point.y = target_position[1]
        target_point.z = target_position[2] if len(target_position) > 2 else 0.0

        # Send to manipulation supervisor
        self.manipulation_supervisor.receive_target_position(target_point)

        # In a real system, we would use callbacks to track progress
        # For simulation, we'll just return success after waiting
        rospy.sleep(5.0)  # Simulate waiting for completion

        # Return status
        return self.manipulation_supervisor.manipulation_status == "completed"
