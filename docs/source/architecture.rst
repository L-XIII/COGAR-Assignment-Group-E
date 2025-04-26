***********************
Architecture Overview
***********************


Component Diagram
=================

.. figure:: /_static/diagrams/Component_diagram_E.png
   :alt: System Component Diagram
   :align: center

   Fig. 1: Component diagram of the Sushi Resturant

Description 
===========

How different design
patters could play a role in the final architecture implementation? 

**Singleton Pattern**

The singleton pattern is used when a class must have only one instance in a whole system. 
It is rather easy to find a relevant usage of this pattern in the case of a sushi restaurant where the delivery is assured by autonomous robots. 
In our case, all the TIAGo robots are managed by a unique orchestration manager (although the component diagram shows only one TIAGo robot for the sake of simplicity...).
Therefore, we have to ensure that the Python class describing the Orchestration Manager can have only one instance.
 Our code implements the singleton pattern to address this issue. By doing this, we ensure that there cannot be two instances of the Orchestration Manager class giving orders at the same time, disrupting the proper functioning of the restaurant.

In the same way, each TIAGo robot has a unique Manipulation System to control its actuators, a unique Navigation System to manage the trajectory planning, a unique Order Verification System to verify that the orders have been correctly executed, and a unique system to reason about the food placement.
 We have therefore implemented the singleton pattern for each of these systems.

**Adapter Pattern**


The adapter pattern is used as a sort of software bridge that allows objects with incompatible interfaces to work and interact together. We can find several places to use it in our architecture.

* Firstly, the adapter pattern can be potentially used in the Manipulation System to transform the data coming from the force sensors and the sensors integrated to the joint actuators into data that the other components of the system can understand.

* Secondly, it can also be integrated - with a higher level of complexity - into the Speech Interface, for example, to transform the recordings of what the clients say into words so that an analysis component can determine the meaning of what the clients tell the robots.

**Mediator Pattern**


In our software architecture, the Orchestration Manager plays the role of the Mediator between the point of sale and the TIAGo robots by managing the communication between them. It boosts the efficiency of the whole system by assigning to each robot its task (every robot doesn’t have to determine if it is able to perform a task and if it is the best placed to perform the task), significantly decreasing the number of calculations that need to be performed by the whole system.
 It also enhances the scalability because adding a new TIAGo platform or a new point of sale amounts just to adding a new ROS node.

**Observer Pattern**


In our software architecture, the Observer pattern is widely used, in the form of communication between modules via ROS. This kind of communication allows greater flexibility in terms of execution, suppressing the need for synchronization between the point of sale publishing frequency and the Orchestration Manager reading frequency, and between the [rest of the sentence seems cut off in the original text].



Components Interfaces discription
=================================

This section details the provided and required interfaces for each component in the system, describing their characteristics according to the component-based software architecture paradigm.


PointOfSale (POS)
-----------------

**Provided Interfaces:**

* ``/orders`` (ROS Topic)
    * **Description:** Publishes new customer orders.
    * **Type:** Data interface (publishes order data).
    * **State:** Stateless (each published order is independent).
    * **Typing:** Loosely-typed (uses ``std_msgs/String`` but contains structured data like *"Table : X, dish : Y, ..."* within the string, requiring parsing).


OrchestrationManager
--------------------

**Required Interfaces:**

* ``/orders`` (ROS Topic)
    * **Description:** Subscribes to receive new orders from the POS.
    * **Type:** Data interface.
    * **State:** Stateless.
    * **Typing:** Loosely-typed (``std_msgs/String``, see POS description).
* ``/availability`` (ROS Topic)
    * **Description:** Subscribes to receive robot availability status.
    * **Type:** Data interface (receives robot state data).
    * **State:** Stateless (each message represents current status).
    * **Typing:** Loosely-typed (receives ``std_msgs/String`` like *"TIAGo X : available"*).
* ``/position`` (ROS Topic)
    * **Description:** Subscribes to receive robot positions.
    * **Type:** Data interface (receives robot state data).
    * **State:** Stateless (each message represents the current position).
    * **Typing:** Strongly-typed (uses ``geometry_msgs/Point``).

**Provided Interfaces:**

* ``/order_TIAGo`` (ROS Topic)
    * **Description:** Publishes specific task assignments to robots.
    * **Type:** Data interface (publishes order messages). Could also be seen as triggering a service via topic.
    * **State:** Stateless (each message is a new assignment).
    * **Typing:** Loosely-typed (uses ``std_msgs/String`` like *"TIAGo n°X, table : Y, dish : Z"*).
* ``/error_messages`` (ROS Topic)
    * **Description:** Publishes error messages for staff/logging.
    * **Type:** Data interface.
    * **State:** Stateless (each message is an independent error report).
    * **Typing:** Loosely-typed (uses ``std_msgs/String``, typically unstructured text).


TIAGo (Main Robot)
------------------

**Required Interfaces:**

* ``/order_TIAGo`` (ROS Topic)
    * **Description:** Subscribes to receive task assignments from OrchestrationManager.
    * *(See OrchestrationManager provided interfaces for details)*.

**Provided Interfaces:**

* ``/availability`` (ROS Topic)
    * **Description:** Publishes its current status ("available" or "occupied").
    * **Type:** Data interface.
    * **State:** Stateless.
    * **Typing:** Loosely-typed (``std_msgs/String``).
* ``/position`` (ROS Topic)
    * **Description:** Publishes its current position (x, y) and ID (in z).
    * **Type:** Data interface.
    * **State:** Stateless.
    * **Typing:** Strongly-typed (``geometry_msgs/Point``).
* ``/orders`` (ROS Topic)
    * **Description:** Publishes requests for table clearing (Note: Same topic name as POS output, ensure correct connection/namespacing in implementation).
    * **Type:** Data interface.
    * **State:** Stateless.
    * **Typing:** Strongly-typed (uses ``geometry_msgs/Point`` - *Verify if this type is correct for a clearing request*).

**Internal Interfaces (Method Calls):**

* Calls methods on PerceptionSystem, OrderVerificationSystem, NavigationSystem, ManipulationSystem, ReasoningSystem.
    * **Type:** Service interfaces.
    * **State:** Often stateful (triggering actions).
    * **Typing:** Strongly-typed.


PerceptionSystem
----------------

**Provided Interfaces (Conceptual - via TIAGo method calls):**

* ``perception_for_navigation()``
    * **Description:** Returns an obstacle detection boolean.
    * **Type:** Service interface (provides a computation result on demand).
    * **State:** Stateless (result depends only on current sensor simulation).
    * **Typing:** Strongly-typed (returns a boolean).
* ``verification_of_grasping_and_placement(operation)``
    * **Description:** Returns a success boolean based on simulated probability.
    * **Type:** Service interface.
    * **State:** Stateless (result based on probability for the given operation).
    * **Typing:** Strongly-typed (takes a string input, returns a boolean).


NavigationSystem
----------------

**Provided Interfaces (ROS Topics):**

* ``/mp`` (ROS Topic)
    * **Description:** Publishes the occupancy grid map.
    * **Type:** Data interface.
    * **State:** Stateful (the map represents accumulated knowledge).
    * **Typing:** Strongly-typed (``nav_msgs/OccupancyGrid``).
* ``/robot_pose`` (ROS Topic)
    * **Description:** Publishes the estimated robot pose.
    * **Type:** Data interface.
    * **State:** Stateful (pose depends on previous states).
    * **Typing:** Strongly-typed (``geometry_msgs/PoseStamped``).
* ``/trajectory`` (ROS Topic)
    * **Description:** Publishes the planned path.
    * **Type:** Data interface.
    * **State:** Stateless (represents the current plan for the current goal).
    * **Typing:** Strongly-typed (``nav_msgs/Path``).

**Required Interfaces (ROS Topics):**

* ``/goal`` (ROS Topic)
    * **Description:** Subscribes to receive navigation goals.
    * **Type:** Data interface (receives command data).
    * **State:** Stateless (each goal is independent).
    * **Typing:** Strongly-typed (``geometry_msgs/PoseStamped``).
* Sensor Topics (e.g., ``/imu_data``, ``/left_encoder``, ``/right_encoder``, ``/object_detection``, ``/manipulation_status``)
    * **Description:** Subscribes to various sensor data streams for SLAM/localization/navigation.
    * **Type:** Data interfaces.
    * **State:** Stateless (each message is current sensor reading).
    * **Typing:** Varies (Strongly or Loosely-typed depending on the specific sensor and message type).

**Provided Interfaces (Method calls from TIAGo):**

* ``navigate_to()``
    * **Type:** Service interface.
    * **State:** Stateful (triggers navigation behavior).
    * **Typing:** Strongly-typed.
* ``update_map()``, ``update_position()``
    * **Type:** Service interfaces.
    * **State:** Stateless (trigger updates based on current data).
    * **Typing:** Strongly-typed.
* ``stop()``
    * **Type:** Service interface.
    * **State:** Stateless (triggers an immediate action).
    * **Typing:** Strongly-typed.


ManipulationSystem
------------------

**Provided Interfaces (ROS Topics):**

* ``/force_data`` (ROS Topic)
    * **Type:** Data interface.
    * **State:** Stateless.
    * **Typing:** Strongly-typed (``std_msgs/Float32``).
* ``/motors_feedback`` (ROS Topic)
    * **Type:** Data interface.
    * **State:** Stateful (reflects motor state).
    * **Typing:** Loosely-typed (``std_msgs/String`` containing structured data).
* ``/safety_status`` (ROS Topic)
    * **Type:** Data interface.
    * **State:** Stateful.
    * **Typing:** Strongly-typed (``std_msgs/String`` representing an enum).
* ``/manipulation_status`` (ROS Topic)
    * **Type:** Data interface.
    * **State:** Stateful.
    * **Typing:** Strongly-typed (``std_msgs/String`` representing an enum).

**Required Interfaces (ROS Topics):**

* ``/gripper_commands`` (ROS Topic)
    * **Type:** Data/Command interface.
    * **State:** Stateless.
    * **Typing:** Strongly-typed (``std_msgs/Float32``).
* ``/grasp_commands`` (ROS Topic)
    * **Type:** Data/Command interface.
    * **State:** Stateless.
    * **Typing:** Strongly-typed (``std_msgs/Float32``).
* ``/joint_commands`` (ROS Topic)
    * **Type:** Data/Command interface.
    * **State:** Stateless.
    * **Typing:** Loosely-typed (``std_msgs/String`` encoding positions).
* ``/trajectory_commands`` (ROS Topic)
    * **Type:** Data/Command interface.
    * **State:** Stateless.
    * **Typing:** Loosely-typed (``std_msgs/String`` encoding waypoints).
* ``/target_dish_position`` (ROS Topic)
    * **Type:** Data interface.
    * **State:** Stateless.
    * **Typing:** Strongly-typed (``geometry_msgs/Point``).
* ``/force_data`` (ROS Topic - Internal Subscription)
    * *(See Provided Interfaces)*
* ``/safety_status`` (ROS Topic - Internal Subscription)
    * *(See Provided Interfaces)*
* ``/motors_feedback`` (ROS Topic - Internal Subscription)
    * *(See Provided Interfaces)*
* ``/perception_data`` (ROS Topic)
    * **Description:** Generic placeholder for perception input needed for manipulation.
    * **Type:** Data interface.
    * **State:** Stateless.
    * **Typing:** Loosely-typed (``std_msgs/String``).

**Provided Interfaces (Method calls from TIAGo):**

* ``execute_manipulation()``
    * **Type:** Service interface.
    * **State:** Stateful (triggers manipulation action).
    * **Typing:** Strongly-typed.


ReasoningSystem
---------------

**Provided Interfaces:**

* ``/target_dish_position`` (ROS Topic)
    * **Description:** Publishes the calculated optimal position for the dish.
    * **Type:** Data interface.
    * **State:** Stateless (result of a specific request).
    * **Typing:** Strongly-typed (``geometry_msgs/Point``).
* ``reason_about_placement()`` (Method call from TIAGo)
    * **Type:** Service interface.
    * **State:** Stateless (performs calculation based on input).
    * **Typing:** Strongly-typed.

**Required Interfaces:**

* *(Implicit)* Access to perception data, likely passed via the ``reason_about_placement`` method call rather than explicit ROS subscriptions shown here.


OrderVerificationSystem
-----------------------

**Required Interfaces:**

* Access to TIAGo instance attributes (``perception_system``, ``id``, ``target_table``)
    * **Type:** Internal interface / Dependency Injection.
* ``verify_delivery_client()`` (Method call to SpeechInterface)
    * **Type:** Service interface.
    * **State:** Stateful.
    * **Typing:** Strongly-typed.
* ``verify_item_served()`` (Method call to PerceptionSystem - *Note: Assumed method name*)
    * **Type:** Service interface.
    * **State:** Stateless.
    * **Typing:** Strongly-typed.

**Provided Interfaces (Method calls from TIAGo):**

* ``verify_served_order()``
    * **Type:** Service interface.
    * **State:** Stateless.
    * **Typing:** Strongly-typed.
* ``verify_delivery_client()``
    * **Type:** Service interface.
    * **State:** Stateless (triggers the interaction).
    * **Typing:** Strongly-typed.

**Provided Interfaces (ROS Topic):**

* ``/error_messages`` (ROS Topic)
    * **Description:** Publishes error messages related to verification failures.
    * **Type:** Data interface.
    * **State:** Stateless.
    * **Typing:** Loosely-typed (``std_msgs/String``).


SpeechInterface
----------------

**Provided Interfaces (Method calls from OrderVerificationSystem):**

* ``verify_delivery_client()``
    * **Type:** Service interface.
    * **State:** Stateful (manages conversation state).
    * **Typing:** Strongly-typed.

