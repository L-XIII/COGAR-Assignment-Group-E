
***********************************
Order Verification & Error Handling
***********************************


Behavioral Diagram
==================


.. figure:: /_static/diagrams/OrderVerificationSequence.png
   :alt: Verification Component Behavior Diagram
   :align: center

   Fig. 4:  Order Verification system sequence diagram.


**Why sequence diagram?**

 This component is primarily interaction-focused. It involves a sequence of messages between the robot (specifically this system via the Speech Interface), the customer, and potentially the staff/Orchestration System (for error reporting). A Sequence Diagram clearly visualizes the timing and order of these interactions. 

 
Description
------------

This sequence diagram illustrates the interactions involved in the order verification process, typically initiated by a `Caller` such as the main TIAGo robot logic. It involves verifying both the placement of the item and the customer's satisfaction.

**Participants:**

* `Caller`: The entity initiating the verification (e.g., TIAGo robot).
* `OrderVerificationSystem`: The main component handling the verification process.
* `OrderVerifier`: A sub-component or class likely responsible for the core verification logic.
* `ErrorHandler`: A component responsible for handling and reporting errors.
* `PerceptionSystem`: Provides visual verification capabilities.
* `SpeechInterface`: Handles interaction with the customer via speech.
* `customer`: The human customer receiving the order.
* `Staff/Manager`: Personnel notified in case of errors.

**Interaction Flow:**

1.  **Verify Item Served:**
    * The `Caller` sends a `verify_served_order()` message to the `OrderVerificationSystem`.
    * The `OrderVerificationSystem` delegates this by sending `handle_verify_served_order()` to the `OrderVerifier`.
    * The `OrderVerifier` interacts with the `PerceptionSystem` by sending `verify_item_served()`.
    * The `PerceptionSystem` performs its check and returns a `success / fail` status to the `OrderVerifier`.
    * The `OrderVerifier` processes the perception result and returns the outcome (`result: served_order_error or None`) to the `OrderVerificationSystem`.
    * The `OrderVerificationSystem` returns this `result` to the `Caller`. 

2.  **Verify Delivery with Client (Separate Step):**
    * The `Caller` initiates the client verification step by sending `verify_delivery_client()` to the `OrderVerificationSystem`.
    * The `OrderVerificationSystem` delegates by sending `handle_verify_delivery_client()` to the `OrderVerifier`.
    * The `OrderVerifier` interacts with the `SpeechInterface` by sending `verify_delivery_client(tiago_instance)`.
    * The `SpeechInterface` initiates interaction with the `customer` (e.g., "Speak: Is your command okay?").
    * The `customer` responds (e.g., `Customer Response (Audio)`).
    * The `SpeechInterface` processes the response (simulated) and returns the result (`problem: 'wrong dish', 'dirty cutlery', etc. or None`) to the `OrderVerifier`.

3.  **Handle Client Response (Alternative Fragment - `alt`):**
    * **[Problem is not None]:** If the `SpeechInterface` detected a problem:
        * The `OrderVerifier` sends `reportProblem(problem)` to the `ErrorHandler`.
        * The `ErrorHandler` handles the problem, potentially by sending `publish_error_messages(error_msg)` (e.g., to a ROS topic).
        * The `ErrorHandler` notifies the `Staff/Manager` ("Error reported to manager").
        * The `ErrorHandler` returns a `handeled_status` to the `OrderVerifier`.
    * **[Problem is None]:** If no problem was detected: 
        * The `OrderVerifier` logs the successful verification (`logVerificationOk()`).

4.  **Return Final Result:**
    * The `OrderVerifier` returns the final outcome (`result: problem or None`) to the `OrderVerificationSystem`.
    * The `OrderVerificationSystem` returns this final `result` to the `Caller`.

**Overall Outcome:**

The process verifies the order the at the serving area via perception and the customer's confirmation via speech. If any issues are detected, particularly from the customer interaction, the `ErrorHandler` is invoked to log the issue and notify staff. The final result indicating success or the nature of the problem is returned to the original `Caller`.


Unit Test KPIs
==============

Spatial Analysis
----------------

* **Percentage of times correct grasp determined:**
    Computed by taking the ratio of the number of times the Order Verification and Error Handling component correctly determines if a plate is properly grasped to the total number of grasp verification tests performed, multiplied by 100. This assesses the reliability of verifying grasping operations.

* **Percentage of times correct placement determined:**
    Computed by taking the ratio of the number of times the Order Verification and Error Handling component correctly determines if a plate is correctly laid on the table to the total number of placement verification tests performed, multiplied by 100. This assesses the reliability of verifying placement operations.

Sound Analysis
--------------

* **Percentage of times client problem correctly determined:**
    Computed by taking the ratio of the number of times the Order Verification and Error Handling component correctly determines (via simulated discussion analysis) whether a client has a problem to the total number of interaction tests performed, multiplied by 100. This assesses the component's ability to effectively interpret client interactions.

