Robotic Navigation

The code is separated into two nodes, one consisting determining a state, turning, and another, an intention, to get pass people. The 'turn.cpp' file is under the folder 'turn_movement' and the 'crowded.cpp' file is under the folder 'crowded_movement'.

For the turning node, a publisher was added that published data msgs that is either "No turning", "Turning right", or "Turning left".

For the blocking node, the publisher publishes 'true' or 'false' based on the bot is being blocked or not. I messed around with the 'EVandPlanner' node but I kept getting errors. However, when I used Global_planner/plan. it started working.

For this to work, I get the current and previous plan, and averaged the distances between the two and set a condition on it in the method getAverageDiffGlobalPath(). Also, I added leg_detector and speak_message_service. For the leg_detector, I added another method, blockLegs() which would return true if legs are detected near the robot. Overall, the condition for the blockLegs() method and getAverageDiffGlobalPath() are sufficiently correct, but needs more improvements.
