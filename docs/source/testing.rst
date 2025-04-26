
**************************
KPI's Integration Testing
**************************

This section describes the Key Performance Indicators (KPIs) used for integration testing and the results obtained.

**Description and results**

The following key performance indicators were measured in the end-to-end service workflow test and all targets were met:

1. Workflow Latency

   :Definition: Time elapsed from POS order emission until TIAGo returns to "available" state.
   
   :Target: <= 5.0 seconds
   :Result: Within target

2. POS -> OrchestrationManager Delivery Rate

   :Definition: Ratio of orders received by OrchestrationManager to orders sent by POS.
   
   :Target: 100%
   :Result: 1/1 orders (100%)

3. Correct Order Processing Rate

   :Definition: Ratio of orders successfully stored and processed by OrchestrationManager to orders received.
   
   :Target: 100%
   :Result: 1/1 orders (100%)

4. Availability Update Rate

   :Definition: Ratio of TIAGo availability messages received and handled by OrchestrationManager to messages published by TIAGo.
   
   :Target: 100%
   :Result: 1/1 messages (100%)

5. Position Update Rate

   :Definition: Ratio of TIAGo position messages received and handled by OrchestrationManager to messages published by TIAGo.
   
   :Target: 100%
   :Result: 1/1 messages (100%)

6. Distance Computation Accuracy

   :Definition: Percentage of correct Euclidean distance calculations between TIAGo and target location.
   
   :Target: 100%
   :Result: Computed distance 5.0 vs expected 5.0 (100%)

7. Idle-Coefficient

   :Definition: Minimum of (number of available TIAGo robots at loop end) and (number of pending orders).
   
   :Target: 0
   :Result: 0 (no idle robots while orders remained)

