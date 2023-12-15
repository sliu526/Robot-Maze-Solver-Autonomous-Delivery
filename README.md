# Robot-Maze-Solver-Autonomous-Delivery
Project focused on programming iRobot Roomba to perform autonomous tasks.

Our primary goal was to understand and implement software that enables intelligent autonomous navigation using the iRobot® Create® 3. By the end of this project, we have:
 - Acquired the skills to program a robot for autonomous navigation to a destination, including understanding and applying sensor data for real-time navigation decisions;
 - Developed algorithms for a robot to detect obstacles and navigate around them effectively;
 - Crafted algorithms that enable a robot to dynamically alter its path based on environmental feedback, simulating real-world navigation scenarios;
 - Sharpened the ability to identify and solve programming and operational issues in autonomous systems.

All of these programs incorporate event-driven fail-safe mechanisms that respond to collisions and button presses to ensure operational safety and prevent damage.
# Maze Solver
Cities consist of intricate networks of streets, featuring complex intersections, dead-ends, and occasional blockages. These challenges necessitate dynamic path planning. Consequently, mazes provide an excellent analog for simulating navigation in such complex urban layouts. Maze solving is a classic challenge in both mathematics and computer science, often used to evaluate and compare the effectiveness of various path planning strategies. Specifically, it tests their ability to find the most efficient, shortest paths through these labyrinthine environments.

We programmed our robot to explore and exit out of a maze of which it had no previous knowledge. It dynamically navigates around maze walls until it reaches the cell corresponding to the exit. Key accomplishments include:
 - Accurate navigation: The robot interprets its environment using IR sensors and accurately determines its position and orientation within the maze. This includes identifying walls and potential obstacles and updating its path dynamically based on these inputs.
 - Collision avoidance: The robot navigates the maze without colliding with any walls or obstacles. This showcases the effectiveness of the implemented sensor-based decision-making and path planning.
 - Dynamic pathfinding: The robot adapts its route in real-time, recalculating paths when encountering unforeseen obstacles or dead ends. This ability is crucial in demonstrating the effectiveness of the maze navigation algorithms.
 - Reaching the destination: The ultimate goal is for the robot to find and reach the specified destination point within the maze. Successfully doing so would indicate that the robot can combine sensor data, orientation, and pathfinding algorithms to solve complex navigational challenges.
 - Visual and auditory feedback: Upon reaching the destination, the robot provides a clear indication of its success, such as changing the color of its lights, to signal the completion of its task.
# Autonomous Delivery
The autonomous delivery market is on the rise as e-commerce continues to grow. Companies increasingly employ robots to deliver a wide range of items, including groceries, pharmaceuticals, and critical medical supplies, particularly to the elderly and in remote locations. These robots are programmed with a set destination and rely on autonomous navigation systems to avoid obstacles and adapt to varied conditions during transit. 

In this project, we designed and implemented a simulation of this technology within the confines of a single room. Our system enabled a robot to move toward a specified goal while intelligently avoiding collisions with various obstacles. Key accomplishments include:
 - Autonomous navigation: The robot independently charts its course to the destination, making real-time path adjustments as needed to navigate through the environment.
 - Collision avoidance: A crucial aspect of the robot's functionality is its ability to detect and circumvent obstacles, ensuring a collision-free journey.
 - Speed and path management: The robot demonstrates the capability to modulate its speed and alter its path in response to environmental factors and obstacles encountered along its route.
 - Feedback mechanisms: The robot employs visual and auditory signals to communicate its actions and status, enhancing its interaction with humans in its vicinity.
 - Manual override and safety: In case of manual interventions like button or bumper activations, the robot immediately halts to ensure safety, demonstrating a reliable fail-safe mechanism.
 - Demonstration of successful navigation: The ultimate measure of success is the robot's ability to reach the designated endpoint without any navigation errors or collisions, proving the effectiveness of the system.
