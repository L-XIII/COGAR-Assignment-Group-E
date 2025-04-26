#!/usr/bin/env python3

import rospy
import math
import random
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point


class ForceSensor:
    """
    The ForceSensor class provides force feedback during manipulation tasks.
    
    This class handles:
    - Force measurement during manipulation operations
    - Collision detection based on force thresholds
    - Publishing force data to other components
    
    Communication:
    - Publishes to: 'force_data' (Float32)
    - Used by: SafetyMonitor, GripperController
    """

    def __init__(self):
        self.force_threshold = 10.0  # Newtons - threshold for collision detection
        self.current_force = 0.0  # Current measured force value

        # Publisher for force data
        self.force_publisher = rospy.Publisher("force_data", Float32, queue_size=10)

    def measure_force(self):
        """
        Simulates force measurement during manipulation tasks.
        
        In a real system, this would read from actual force sensors.
        For simulation, we generate simulated force readings with noise.
        
        Returns:
            float: Current force value in Newtons
        """
        # Simulate force readings with base value and random noise
        base_force = 2.0  # baseline force in Newtons
        noise = random.uniform(-1.0, 1.0)  # Random noise component
        self.current_force = base_force + noise

        # Publish force data for other components
        self.force_publisher.publish(self.current_force)

        return self.current_force

    def detect_collision(self):
        """
        Detects if the measured force exceeds the safe threshold.
        
        Returns:
            bool: True if collision is detected (force > threshold), False otherwise
        """
        return self.current_force > self.force_threshold

    def reset(self):
        """
        Resets force sensor readings to zero.
        Called after completing a manipulation task or when recalibrating.
        """
        self.current_force = 0.0


class GripperMotors:
    """
    The GripperMotors class controls the physical motors of the gripper.
    
    This class handles:
    - Moving the gripper to specified positions
    - Tracking gripper status (position, operational state)
    - Receiving control commands from GripperController
    
    Communication:
    - Subscribes to: 'gripper_commands' (Float32)
    - Controlled by: GripperController
    """

    def __init__(self):
        self.position = 0.0  # 0.0 = fully open, 1.0 = fully closed
        self.max_force = 15.0  # Maximum force in Newtons
        self.status = "idle"  # Current status: idle, moving, holding, error

        # Subscriber for gripper commands
        rospy.Subscriber("gripper_commands", Float32, self.receive_command)

    def receive_command(self, command_msg):
        """
        Receives and processes motor position commands.
        
        Args:
            command_msg (Float32): Target position value (0.0-1.0)
        """
        target_position = command_msg.data
        self.move_to_position(target_position)

    def move_to_position(self, target_position):
        """
        Moves the gripper to the specified target position.
        
        In a real system, this would send commands to actual motors.
        For simulation, we simulate movement time and success probability.
        
        Args:
            target_position (float): Target position (0.0-1.0)
            
        Returns:
            bool: Success of the movement operation
        """
        self.status = "moving"  # Update status to moving

        # Simulate movement time
        rospy.sleep(0.5)  # Simulate 0.5 second movement time

        # Simulate success with 95% probability
        success = random.random() < 0.95

        if success:
            self.position = target_position
            # Update status based on position (holding if closed, idle if open)
            self.status = "holding" if target_position > 0.5 else "idle"
            return True
        else:
            self.status = "error"  # Movement failed
            return False

    def get_status(self):
        """
        Returns the current status of the gripper motors.
        
        Returns:
            dict: Contains position and status information
        """
        return {"position": self.position, "status": self.status}


class GripperController:
    """
    The GripperController class coordinates higher-level gripper operations.
    
    This class handles:
    - Processing grasp commands (grasp, release, adjust)
    - Monitoring force feedback and adjusting grip strength
    - Controlling the gripper motors
    
    Communication:
    - Subscribes to: 'grasp_commands' (String), 'force_data' (Float32)
    - Publishes to: 'gripper_commands' (Float32)
    - Controls: GripperMotors
    - Receives data from: ForceSensor, ManipulationSupervisor
    """

    def __init__(self, force_sensor=None, gripper_motors=None):
        self.force_sensor = force_sensor
        self.gripper_motors = gripper_motors
        self.grip_strength = 0.7  # Default grip strength (0-1)
        self.gripping_object = False  # Whether currently holding an object

        # Publishers and subscribers
        self.grip_command_publisher = rospy.Publisher(
            "gripper_commands", Float32, queue_size=10
        )
        rospy.Subscriber("grasp_commands", String, self.receive_grasp_command)
        rospy.Subscriber("force_data", Float32, self.monitor_force)

    def receive_grasp_command(self, command_msg):
        """
        Processes grasp commands from ManipulationSupervisor.
        
        Supported commands:
        - "grasp": Close gripper to grasp an object
        - "release": Open gripper to release an object
        - "adjust_stronger": Increase grip strength
        - "adjust_gentler": Decrease grip strength
        
        Args:
            command_msg (String): Command string
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
        Monitors force data to adjust gripper behavior for safety.
        
        If force exceeds safe threshold while gripping, reduces grip strength.
        
        Args:
            force_msg (Float32): Force sensor reading
        """
        force = force_msg.data

        # If force exceeds safe threshold, loosen grip
        if force > 12.0 and self.gripping_object:
            self.grip_strength = max(0.5, self.grip_strength - 0.1)
            self._send_grip_command(self.grip_strength)

    def execute_grasp(self):
        """
        Executes a grasp operation to pick up an object.
        
        Returns:
            bool: Success status of the grasp operation
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
        Executes a release operation to put down an object.
        
        Returns:
            bool: Success status of the release operation
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
        Sends grip command to gripper motors.
        
        Args:
            position (float): Target position value (0.0-1.0)
            
        Returns:
            bool: Success status of the command
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
        Adjusts grip strength based on command.
        
        Args:
            command (str): "adjust_stronger" or "adjust_gentler"
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
    
    This class handles:
    - Moving arm joints to specified positions
    - Tracking joint positions, velocities, and torques
    - Publishing motor feedback for monitoring
    
    Communication:
    - Subscribes to: 'joint_commands' (String)
    - Publishes to: 'motors_feedback' (String)
    - Controlled by: ArmController
    - Provides feedback to: SafetyMonitor
    """

    def __init__(self):
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 7-DOF arm
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_torques = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.status = "idle"  # Current status: idle, moving, error

        # Publishers for feedback
        self.feedback_publisher = rospy.Publisher(
            "motors_feedback", String, queue_size=10
        )

        # Subscribers for commands
        rospy.Subscriber("joint_commands", String, self.receive_command)

    def receive_command(self, command_msg):
        """
        Receives and processes joint position commands.
        
        Command format: "position:j1,j2,j3,j4,j5,j6,j7"
        where j1-j7 are joint position values
        
        Args:
            command_msg (String): Joint command string
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
        Moves joints to target positions.
        
        In a real system, this would send commands to actual motors.
        For simulation, we simulate movement time and success probability.
        
        Args:
            target_positions (list): List of 7 joint position values
            
        Returns:
            bool: Success status of the movement operation
        """
        # Update status to moving
        self.status = "moving"

        # Publish status update
        self.feedback_publisher.publish("status:moving")

        # Simulate movement time
        rospy.sleep(0.8)  # Simulate 0.8 second movement time

        # Simulate success with 95% probability
        success = random.random() < 0.95

        if success:
            self.joint_positions = target_positions
            self.status = "idle"

            # Generate and publish feedback data
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
        Generates and publishes simulated joint feedback data.
        
        This simulates sensor readings from the joints including:
        - Small variations in actual position
        - Random velocity and torque values within expected ranges
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
        Returns the current status of all joints.
        
        Returns:
            dict: Contains positions, velocities, torques and status
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
    
    This class handles:
    - Processing and executing trajectory commands
    - Converting cartesian waypoints to joint positions
    - Monitoring safety status and aborting if necessary
    
    Communication:
    - Subscribes to: 'trajectory_commands' (String), 'safety_status' (String), 'motors_feedback' (String)
    - Publishes to: 'joint_commands' (String)
    - Controls: JointsMotors
    - Receives data from: ManipulationSupervisor, SafetyMonitor
    """

    def __init__(self, joints_motors=None):
        self.joints_motors = joints_motors
        self.current_trajectory = None  # Current trajectory being executed
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
        Receives trajectory commands from ManipulationSupervisor.
        
        Trajectory format: "trajectory:x1,y1,z1,r1,p1,y1|x2,y2,z2,r2,p2,y2|..."
        Each waypoint contains position (x,y,z) and orientation (roll,pitch,yaw)
        
        Args:
            trajectory_msg (String): Trajectory command string
        """
        trajectory_data = trajectory_msg.data

        # Parse trajectory data
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
        Receives safety status updates from SafetyMonitor.
        
        If safety becomes critical during execution, aborts the trajectory.
        
        Args:
            status_msg (String): Safety status ("normal", "warning", "critical")
        """
        self.safety_status = status_msg.data

        # If safety critical, abort current trajectory
        if self.safety_status == "critical" and self.execution_status == "executing":
            self.abort_trajectory()

    def receive_motor_feedback(self, feedback_msg):
        """
        Receives feedback from JointsMotors.
        
        Updates execution status based on motor feedback.
        
        Args:
            feedback_msg (String): Motor feedback status
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
        Executes the current trajectory by processing each waypoint sequentially.
        
        Returns:
            bool: Success status of trajectory execution
        """
        if not self.current_trajectory or self.safety_status == "critical":
            self.execution_status = "aborted"
            return False

        self.execution_status = "executing"
        rospy.loginfo("Starting trajectory execution")

        # Execute each waypoint sequentially
        for waypoint_idx, waypoint in enumerate(self.current_trajectory):
            # Check safety status before proceeding
            if self.safety_status == "critical":
                self.abort_trajectory()
                return False

            # Convert cartesian waypoint to joint positions
            joint_positions = self._simplified_inverse_kinematics(waypoint)

            # Send command to joints motors
            command = f"position:{','.join([str(jp) for jp in joint_positions])}"
            self.command_publisher.publish(command)

            # Wait for completion before proceeding to next waypoint
            rospy.sleep(1.0)  # Simulate execution time

            # Log progress
            rospy.loginfo(
                f"Waypoint {waypoint_idx + 1}/{len(self.current_trajectory)} executed"
            )

        self.execution_status = "completed"
        return True

    def abort_trajectory(self):
        """
        Aborts the current trajectory execution for safety.
        
        Stops movement by commanding the joints to hold their current positions.
        """
        if self.execution_status == "executing":
            rospy.logwarn("Aborting trajectory execution")

            # Stop movement - send command to move to current position
            if self.joints_motors:
                current_positions = self.joints_motors.joint_positions
                command = f"position:{','.join([str(jp) for jp in current_positions])}"
                self.command_publisher.publish(command)

            self.execution_status = "aborted"

    def _simplified_inverse_kinematics(self, cartesian_point):
        """
        Simplified inverse kinematics computation for simulation.
        
        Converts from cartesian coordinates and orientation to joint angles.
        In a real system, this would be replaced with an accurate IK solver.
        
        Args:
            cartesian_point (list): [x, y, z, roll, pitch, yaw]
            
        Returns:
            list: 7 joint position values
        """
        x, y, z, roll, pitch, yaw = cartesian_point

        # This is a simplified approximation of inverse kinematics
        j1 = math.atan2(y, x)  # Base rotation
        r_xy = math.sqrt(x**2 + y**2)
        j2 = math.atan2(z, r_xy)  # Shoulder
        j3 = math.sin(roll) * 0.5  # Elbow
        j4 = math.sin(pitch) * 0.5  # Wrist 1
        j5 = math.sin(yaw) * 0.5  # Wrist 2
        j6 = r_xy * 0.1  # Wrist 3
        j7 = z * 0.1  # Gripper joint

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
    
    This class handles:
    - Monitoring joint positions, velocities, and torques
    - Processing force sensor data to detect collisions
    - Publishing safety status updates
    
    Communication:
    - Subscribes to: 'motors_feedback' (String), 'force_data' (Float32)
    - Publishes to: 'safety_status' (String)
    - Provides safety information to: ManipulationSupervisor, ArmController
    - Receives data from: JointsMotors, ForceSensor
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
        Processes feedback from JointsMotors to detect safety issues.
        
        Checks if joint positions are within safe limits.
        
        Args:
            feedback_msg (String): Motor feedback data
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
        Processes force data to detect potential collisions.
        
        Updates safety status based on force thresholds:
        - Above 12.0N: Warning
        - Above 15.0N: Critical
        
        Args:
            force_msg (Float32): Force sensor reading
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
        Checks if joint positions are within safe limits.
        
        Args:
            joint_positions (list): List of joint position values
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
        Updates and publishes safety status.
        
        Only updates if new status is more critical than current status
        (normal < warning < critical).
        
        Args:
            status (str): New safety status
            message (str): Descriptive message
        """
        # Only update if new status is more critical than current
        status_priority = {"normal": 0, "warning": 1, "critical": 2}

        if status_priority.get(status, 0) >= status_priority.get(self.safety_status, 0):
            self.safety_status = status

            # Log message with appropriate level
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
        Checks if current state is safe for operations.
        
        Returns:
            bool: True if safe (not critical), False otherwise
        """
        return self.safety_status != "critical"


class ManipulationSupervisor:
    """
    The ManipulationSupervisor class coordinates the overall manipulation process.
    
    This class handles:
    - Receiving target positions from the reasoning system
    - Planning and coordinating manipulation sequences
    - Monitoring safety and operation status
    
    Communication:
    - Subscribes to: 'target_dish_position' (Point), 'safety_status' (String), 'perception_data' (String)
    - Publishes to: 'manipulation_status' (String), 'trajectory_commands' (String), 'grasp_commands' (String)
    - Coordinates: ArmController, GripperController
    - Receives data from: ReasoningController, SafetyMonitor, PerceptionSystem
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

        self.target_position = None  # Target position for manipulation
        # Status: idle, moving, grasping, placing, completed, failed
        self.manipulation_status = "idle"

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
        Receives target dish position from ReasoningController.
        
        Starts the manipulation planning and execution sequence.
        
        Args:
            position_msg (Point): Target position (x, y, z)
        """
        self.target_position = [position_msg.x, position_msg.y, position_msg.z]
        rospy.loginfo(f"Received target position: {self.target_position}")

        # Start manipulation sequence
        self.plan_and_execute_manipulation()

    def receive_safety_status(self, status_msg):
        """
        Receives safety status updates from SafetyMonitor.
        
        Aborts manipulation if safety becomes critical.
        
        Args:
            status_msg (String): Safety status
        """
        safety_status = status_msg.data

        # If critical safety issue occurs during manipulation, abort
        if safety_status == "critical" and self.manipulation_status != "idle":
            self.abort_manipulation("Safety critical condition detected")

    def receive_perception_data(self, perception_msg):
        """
        Processes perception data for object recognition.
        
        In a real system, this would use perception data to refine manipulation.
        
        Args:
            perception_msg (String): Perception data
        """
        # In a real system, this would process detailed perception data
        rospy.loginfo("Received perception data for manipulation")

    def plan_and_execute_manipulation(self):
        """
        Plans and executes the complete manipulation sequence.
        
        Steps:
        1. Plan approach trajectory
        2. Execute approach trajectory
        3. Execute grasp/release
        4. Plan and execute retreat trajectory
        
        Returns:
            bool: Success status of the manipulation sequence
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
        Executes grasp or release operation depending on the current status.
        
        When approaching to grasp: executes grasp
        When approaching to place: executes release
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
        Aborts the current manipulation operation.
        
        Updates status and logs the reason for aborting.
        
        Args:
            reason (str): Reason for aborting
        """
        rospy.logwarn(f"Aborting manipulation: {reason}")

        # Stop arm movement
        if self.arm_controller:
            self.arm_controller.abort_trajectory()

        self.update_status("failed", f"Manipulation aborted: {reason}")

    def update_status(self, status, message):
        """
        Updates and publishes manipulation status.
        
        Args:
            status (str): New status (idle, moving, grasping, placing, completed, failed)
            message (str): Descriptive message
        """
        self.manipulation_status = status

        # Log message with appropriate level
        if status == "failed":
            rospy.logwarn(f"Manipulation failed: {message}")
        else:
            rospy.loginfo(f"Manipulation status: {status} - {message}")

        # Publish status update
        self.status_publisher.publish(status)

    def _plan_trajectory(self, trajectory_type):
        """
        Plans approach or retreat trajectories.
        
        Args:
            trajectory_type (str): "approach" or "retreat"
            
        Returns:
            str: String representation of the trajectory waypoints
        """
        if trajectory_type == "approach":
            # Plan approach to target position
            # Assume current position is origin for simulation
            current_position = [0.0, 0.0, 0.2]

            # Generate intermediate waypoints for smooth approach
            waypoint1 = self._interpolate_position(
                current_position, self.target_position, 0.3
            )
            waypoint2 = self._interpolate_position(
                current_position, self.target_position, 0.7
            )

            # Add orientation information (roll, pitch, yaw)
            waypoint1.extend([0.0, 0.0, 0.0])
            waypoint2.extend([0.0, 0.0, 0.0])
            target_with_orientation = list(self.target_position) + [0.0, 0.0, 0.0]

            # Create trajectory string
            trajectory = f"{','.join([str(x) for x in waypoint1])}|{','.join([str(x) for x in waypoint2])}|{','.join([str(x) for x in target_with_orientation])}"

            return trajectory

        elif trajectory_type == "retreat":
            # Plan retreat from target position
            # Add orientation information if not already present
            if len(self.target_position) == 3:
                target_with_orientation = list(self.target_position) + [0.0, 0.0, 0.0]
            else:
                target_with_orientation = list(self.target_position)

            # Create waypoints for retreat
            retreat_point1 = list(target_with_orientation)
            retreat_point1[2] += 0.1  # Move up

            retreat_point2 = list(retreat_point1)
            retreat_point2[0] -= 0.2  # Move back

            # Create trajectory string
            trajectory = f"{','.join([str(x) for x in retreat_point1])}|{','.join([str(x) for x in retreat_point2])}"

            return trajectory

        return None

    def _interpolate_position(self, start_pos, end_pos, factor):
        """
        Interpolates between two positions by a factor.
        
        Args:
            start_pos (list): Starting position coordinates
            end_pos (list): Ending position coordinates
            factor (float): Interpolation factor (0.0-1.0)
            
        Returns:
            list: Interpolated position
        """
        result = []
        for i in range(min(len(start_pos), len(end_pos))):
            result.append(start_pos[i] + (end_pos[i] - start_pos[i]) * factor)
        return result


class ManipulationSystem:
    """
    ManipulationSystem is the main interface for the manipulation subsystem.
    
    This class integrates all manipulation components and provides a unified interface
    to the rest of the robot system. It uses the singleton pattern to ensure only 
    one instance exists.
    
    Communication:
    - Coordinates all manipulation components
    - Provides main interface for external systems
    """

    _instance = None

    def __new__(cls, tiago_platform=None):
        """
        Creates a singleton instance of ManipulationSystem.
        
        Args:
            tiago_platform: Reference to the TIAGo robot instance
            
        Returns:
            ManipulationSystem: The singleton instance
        """
        if cls._instance is None:
            cls._instance = super(ManipulationSystem, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, tiago_platform=None):
        """
        Initializes the manipulation system with all its components.
        
        Only initializes on first instantiation due to singleton pattern.
        
        Args:
            tiago_platform: Reference to the TIAGo robot instance
        """
        if not hasattr(self, "_initialized") or not self._initialized:
            self.tiago = tiago_platform

            # Initialize components in dependency order
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
        Main method to execute a manipulation task.
        
        This is the primary entry point for external systems to request
        manipulation operations.
        
        Args:
            target_position (list): Target position coordinates [x, y, z]
            
        Returns:
            bool: True if manipulation was successful, False otherwise
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
