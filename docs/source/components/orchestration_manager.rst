
*********************
Orchestration Manager
*********************

This component is responsible for managing tasks, resources (robots), and communication flow.


Behavioral Diagram
==================


.. figure:: /_static/diagrams/Orchestration_statemachine.drawio.png
   :alt: Orchestration Manager Behavior Diagram
   :align: center
   :target: https://raw.githubusercontent.com/L-XIII/COGAR-Assignment-Group-E/refs/heads/main/docs/source/_static/diagrams/Orchestration_statemachine.drawio.png
   :width: 1500px

   Fig. 2:  Orchestration Manager state machine diagram

**Why state machine diagram?**

 This system's core function involves managing the state of tasks (Queued, Assigned, In Progress, Completed, Failed) and potentially the state of robots (Available, Busy, Charging). It reacts to events like new orders, task completions, and robot status updates. A State Machine diagram is ideal for visualizing these states, transitions, and the events that trigger them. 

Description
-----------

This state machine diagram illustrates the behavior of the Orchestration Manager component. It handles incoming orders, robot status updates, and assigns tasks accordingly.

**States:**

* **Idle:**
    * This is the initial and default state.
    * **Entry Action:** ``initialize_listeners()`` (Sets up subscriptions to ROS topics like ``/orders``, ``/availability``, ``/position``).
    * **Do Action:** ``check_conditions()`` (Potentially periodic internal checks).
    * **Transitions:**
        * Receives ``newOrderReceived(msg)``: Stores the order via ``order_storing(msg)`` and remains in Idle.
        * Receives ``availabilityReceived(msg)``: Updates robot status via ``manage_availability(msg)`` and remains in Idle.
        * Receives ``positionReceived(msg)``: Updates robot position via ``manage_position(msg)`` and remains in Idle.
        * Triggered by ``timerTick``: Moves to ``CheckingForTask`` state.

* **CheckingForTask:**
    * **Entry Action:** ``checkQueuesAndAvailability()`` (Checks if there are pending orders in the queue and if any robots are available).
    * **Transitions:**
        * If ``orderQueueNotEmpty AND robotAvailable`` is true: Transitions to the ``Assigning task`` composite state.
        * Otherwise (``[else]`` guard): Transitions back to ``Idle``.

* **Assigning task (Composite State):**
    * This state encapsulates the process of selecting a robot and sending it a command.
    * **Initial Transition Action:** ``getOrderDetails()`` (Retrieves details of the order to be assigned).
    * **Internal States:**
        * **SelectingRobot:**
            * **Entry Action:** ``findNearestRobot()`` (Identifies the most suitable available robot, likely based on proximity or other criteria).
            * **Transitions:**
                * If ``robot found``: Transitions to ``SendingCommand``.
                * Otherwise (``[else]`` guard, no suitable robot found): Transitions back out of the composite state to ``CheckingForTask`` (Note: the diagram shows transition back to the choice pseudostate before Idle, implying it might re-evaluate immediately or go Idle if conditions changed).
        * **SendingCommand:**
            * **Entry Actions:** ``formatCommand()`` (Prepares the command message in the required format) and ``publishCommand()`` (Sends the command to the selected robot via the ``/order_TIAGo`` topic).
            * **Transition:**
                * On ``commandSent``: Transitions to the final state, indicating successful task assignment.

* **HandlingFailure:**
    * Entered when an unrecoverable error occurs (transition not explicitly shown but implied).
    * **Entry Actions:** ``logFailure()`` (Records the error) and ``notifyStaff()`` (Alerts personnel).
    * **Do Action:** ``attemptRecovery()`` (Executes automated recovery procedures if possible).
    * **Transitions:**
        * If ``[Unrecoverable]``: Transitions to a final error state (X).
        * If ``[RecoveryAttempted]``: Performs ``cleanup()`` actions and transitions back to ``Idle``.

**Overall Flow:**

The Orchestration Manager starts in the ``Idle`` state, listening for orders and robot updates. Periodically (on ``timerTick``), it checks if there are tasks to assign (``CheckingForTask``). If an order exists and a robot is free, it enters the ``Assigning task`` process: it selects the best robot (``SelectingRobot``) and sends the command (``SendingCommand``). If successful, it returns towards the idle/checking loop. If any part fails irrecoverably, it enters ``HandlingFailure`` to log, notify, and potentially attempt recovery before either terminating or returning to ``Idle``.


Unit Test KPIs
==============

Orders Handling
---------------

* **Percentage of orders successfully received:**
    Computed by taking the ratio between the number of orders received by the Orchestration Manager and the number of orders sent by the Points of Sale, multiplied by 100. This verifies if the Orchestration Manager successfully receives all order messages published by the POS.

* **Percentage of incorrect orders successfully analyzed:**
    Computed by taking the ratio between the number of incorrect orders correctly processed and the number of incorrect orders received, multiplied by 100. By deliberately sending incorrect orders, this parameter assesses the robustness of the Orchestration Manager's order handling in case of unexpected errors.

* **Percentage of correct orders correctly analyzed:**
    Computed by taking the ratio between the number of correct orders correctly processed and the number of correct orders received, multiplied by 100. This assesses how well the Orchestration Manager analyzes correct orders under nominal conditions.

Handling of TIAGo Availability Updates
--------------------------------------

* **Percentage of availability messages successfully received:**
    Computed by taking the ratio between the number of availability messages received by the Orchestration Manager and the number sent by the TIAGo robots, multiplied by 100. This verifies if the Orchestration Manager successfully receives all availability messages published by the TIAGo robots.

* **Percentage of correct availability messages successfully analyzed:**
    Computed by taking the ratio between the number of correct availability messages successfully analyzed and the number of correct availability messages received, multiplied by 100. This verifies if the statuses of the TIAGo robots are correctly processed under nominal conditions.

* **Percentage of incorrect availability messages generating an error message:**
    Computed by taking the ratio between the number of incorrect availability messages that generate an error publication and the number of incorrect availability messages received, multiplied by 100. This verifies if the Orchestration Manager efficiently raises error messages when a TIAGo robot provides an invalid status.

Handling of TIAGo Position Updates
----------------------------------

* **Percentage of position messages successfully received:**
    Computed by taking the ratio between the number of position messages received by the Orchestration Manager and the number sent by the TIAGo robots, multiplied by 100. This verifies if the Orchestration Manager successfully receives all position messages published by the TIAGo robots.

* **Percentage of correct position messages successfully analyzed:**
    Computed by taking the ratio between the number of correct position messages successfully analyzed and the number of correct position messages received, multiplied by 100. "Correct" means the position does not cause the robot model to intersect with walls or tables. This verifies if robot positions are correctly processed and updated under nominal conditions.

* **Percentage of incorrect position messages generating an error message:**
    Computed by taking the ratio between the number of incorrect position messages generating an error publication and the number of incorrect position messages received, multiplied by 100. This verifies if the Orchestration Manager efficiently raises errors when a TIAGo robot reports a physically impossible position (e.g., intersecting with obstacles).

Distance Computation
--------------------

* **Percentage of distances computed correctly:**
    Computed by taking the ratio of the number of correctly computed distances over the total number of distances computed, multiplied by 100. This verifies if the Orchestration Manager successfully computes distances between objects, essential for assigning orders to the optimal TIAGo robot.

Publication of Orders to TIAGo Robots
-------------------------------------

* **Idleness coefficient:**
    Computed by taking the minimum of: (a) the number of TIAGo robots available at the end of a control loop, and (b) the number of orders remaining to be assigned. A lower value is better. A value greater than zero indicates inefficiency, as there are both available robots and unassigned orders.

Analysis of Message Transmission
--------------------------------

* **Percentage of TIAGo order messages correctly sent:**
    Computed by dividing the number of times order messages were correctly sent to TIAGo robots by the total number of attempts, multiplied by 100. This assesses the ability of the Orchestration Manager to reliably send orders to the robots.

* **Percentage of error messages correctly sent:**
    Computed by dividing the number of times error messages were correctly sent by the total number of attempts, multiplied by 100. This assesses the ability of the Orchestration Manager to reliably send error messages to staff/monitoring systems.

