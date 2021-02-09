# Particle Filter Project
___
## Team
Kiana Hobbs, Elizabeth Singer

## Writeup

### Objective
*Describe the goal of this project.*

The goal of this project was to use principles about robot localization to help the robot locate itself within a room. Using just a map of the space and Monte Carlo localization, the robot must localize itself within the context of the map and use that information to navigate towards the exit.
___
### High-level Approach
*High-level description (1 paragraph): At a high-level, describe how you solved the problem of robot localization. What are the main components of your approach?*

To solve the problem of robot localization, we used a randomized particle cloud to pinpoint the position of the robot using information about the map, the robot's actual movements, and the proximity of objects within the space. Furthermore, we parsed the assignment into three parts: movement, computation of importance weights, and resampling. After initializing a particle cloud using the map information about occupancy (what spaces could the robot inhabit given the map), we mirrored the robot's movements onto the particles within the particle cloud to get a sense of what the particle's positioning would be as the robot. To compute the importance weights, we used parts of the likelihood fields calculation to determine how closely what the robot was sensing around itself aligned the surroundings for each particle. After computing the updated weights for the particles, we resampled the particles within the particle cloud to reflect the updated weighted probabilities.
___
### In-depth Approach
*For each of the 3 main steps of the particle filter (movement, computation of importance weights, and resampling), please provide the following:*
1. *Code location (1-3 sentences): Please describe where in your code you implemented this step of the particle filter.*
2. *Functions/code description (1-3 sentences per function / portion of code): Describe the structure of your code. For the functions you wrote, describe what each of them does and how they contribute to this step of the particle filter.*
#### Movement

Movement is implemented through the ```update_particles_with_motion_model()``` function and somewhat with the ```update_estimated_robot_pose()```.

* ```update_particles_with_motion_model()```: This function uses the robot's actual movements to reproduce those movements for each particle. To do so, we found the change in the robot's x, y, and yaw, and added that change to the particle's x, y, and yaw.

* ```update_estimated_robot_pose()```:  To find the robot's estimated pose, we calculated the average particle from the particle cloud and set that as the estimation. As such, the estimated pose served as a collective of the existing particles.

#### Computation of Importance Weights
 Computation of importance weights is implemented in ```update_particle_weights_with_measurement_model()``` and those weights are normalized in ```normalize_particles()```.

* ```update_particle_weights_with_measurement_model()```: This function generates updated particles weights by comparing scan readings of the robot and the particle about objects surrounding the robot/particle. To do so, we utilized the likelihood fields for range finders at angles 0, 90, 180, and 270 degrees to determine if a given particle was in the location of the robot, how likely would it sense the same objects.

* ```normalize_particles()```: This function updates the weight of each particle such that the sum of the particle weights equates to one. This function ensures that all the weights are scaled relatively to the total weights of the particles.

#### Resampling

Resampling is implemented in the ```resample_particles()``` function with those particles being initialized in ```initialize_particle_cloud() ```.

* ```initialize_particle_cloud()```: This function generates the initial particles that will compose the particle cloud. In the function, we use the occupancy field information from the map data to determine which spaces in the map the robot could occupy, and then generate a random set of "n" particles from those possible positions with each particle having the same probability for being selected. 

* ```resample_particles()```: This function regenerates the particle cloud using the updated particle weights after normalization and robot movements. In the function, we randomly select "n" particles as the new particle cloud using the particle weights as the probability for each particle.

____
### Challenges
*Challenges (1 paragraph): Describe the challenges you faced and how you overcame them.*

One challenge we faced was accounting for concurrency issue between our algorithm’s computation time and the reception of transformation data. This issue only emerged when we used many particles: more than 5,000 particles. To resolve this issue, we ensured that the script ran the transformations in simulation time by incorporating a sim_time parameter into our ROS launch files.
____
### Future Work
*Future work (1 paragraph): If you had more time, how would you improve your particle filter?*

To improve on our particle filter, we could examine more than just the 4 cardinal angles of the robot when calculating the particles' weight using the measurement model. By expanding beyond just 0, 90, 180, and 270 degrees, we can get a more accurate weight calculation for each particle and thus a more accurate particle cloud depiction for resampling. Moreover, another means for improving our filter would be to explore ways to make our algorithm more efficient, such that we could initialize a larger sum of particles without enduring a toll on the algorithms processing speed.
____
### Takeaways
*Takeaways (at least 2 bullet points with 2-3 sentences per bullet point): What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.*

1. Work collaborately when you can on each section: By jointly working on each function, not only were we able to gain a deeper grasp of the subject matter, but we were also able to develop more efficient and concise algorithms while learning from each other during the process. This method of collaboration offered a stronger sense of understanding and material connection, versus had we were to create a hard split between the functions and contributions.
2. 
____
### Gif
![Particles in motion](particle_filter.gif)

![Motion and navigate to goal](particle_filter_movie.gif)

## Implementation Plan
### Initialize Particle Cloud
How you will initialize your particle cloud (initialize_particle_cloud()):


Given a map of the room, we will randomly pick particles inside the house, while verifying that it is a place where the robot could be using the free threshold of 0.196. If the particle is in a valid location, we will randomly pick an orientation between 0 and 360 degrees, else we will try again until we create *n* valid particles. 
</br></br>
To test this function, we will first ensure that we can determine a point on the map at random, and then check that the point is a valid placement. We will then test that we can generate a few such points, which can be inspected in Gazebo using rviz.

### Particle Positions Update
How you will update the position of the particles will be updated based on the movements of the robot (update_particles_with_motion_model()):


We will monitor the odometry messages and apply the twist messages to each particle over the time since the last update. Moreover, we will include small random perturbations to each particle position to allow them to drift.
</br></br>
We will verify that we can observe the twist messages from the odometry topic and print these to the log. We will then verify that we can apply the motion from a twist message to a single particle and observe the change in both logs and on Gazebo, and later observe this phenomenon in all the particles as the simulation progresses. 

### Importance Weights
How you will compute the importance weights of each particle after receiving the robot's laser scan data (update_particle_weights_with_measurement_model()):


For each particle, k=1..N, we will compute what the laser scan data would be if the robot were at that location and orientation, scan(k,theta). We may do this for just one theta, being the direction to the closest object in the map, or for all theta. We will then compute the sum of the squares of the errors between the hypothetical laser scan, scan(k,theta) and the true laser scan, scan_true(theta) for the thetas considered and call this the particle sensor error e_k = sum (scan(k,theta)-scan_true(theta))^2, over all theta’s considered, or just for the closest direction to the closest object. Then each particle weight, w_k can be updated using this particle sensor error. One way to do this is to simply make each particle new weight just equal to the inverse of this sensor error w_k = 1/e_k.  Another approach would be to update each weight proportional to this error, so w_k_new = w_k_old / e_k, or w_k_new = w_k_old * G(e_k,sigma), where G(e,sigma) is the Gaussian distribution with variance sigma^2, G(e,s) = (1/sqrt(2 pi s)) exp(-e^2/(2 s^2)). 
</br></br>
To test, we will generate some laser scans by hand, and verify that the weight update calculation is correct for these hand-calculations. We will then generate laser scans from gazebo, and add fixed errors to these, and compute the weights that come from these, then we will generate scans from random particle locations. We will test this hypothetical laser scan function separately first to verify that it provides the correct scan data, which we can do by placing the robot in that location and comparing the hypothetical scan with the scan from the messages in ros.

### Normalization and Resampling
How you will normalize the particles' importance weights (normalize_particles()) and resample the particles (resample_particles()):


To normalize the particle’s importance weights we will calculate the sum of all the weights (sum w_k, k=1..N) and divide each particle weight by the sum of the weights so that each one will add to one, W=sum(w_k), w_k_new = w_k / W. (Divide each weight by the sum of the weights). To resample the particles, there are a few ways to do this. One way is to use the draw_random_sample function provided in the code, which just selects N particles at random from the given particle list and their weights. Another way would be to look at the particles that have small weights, below some threshold value, and replace these particles with new particles, drawn at random using the draw_random_sample function given in the code. This way, particles with larger weights will remain, and particles that become negligibly small in probability will be replaced with more likely particles. 
</br></br>
For this function, we can test normalizing weights by first running this on some fixed weights generated by hand, and verifying they are correct. Then we can generate large sets of weights and normalize them and see that they still sum to 1. We can test resampling by making sure that for a small number of particles, the resampling function throws away particles that are below the threshold and picks new particles that have high probabilities. 

### Pose Estimation
How you will update the estimated pose of the robot (update_estimated_robot_pose()):


We will use the expected value of the pose, given the distribution obtained from the particles. To compute this, we will use the weighted sum of poses from all the particles within some distance of the highest probability particle and take the weighted average of the poses of those particles.
</br></br>
Through visual observation and by hand calculation, we can compare our calculations with those calculated by the function for accuracy. Futhermore, we can check that the most likely particle selection works by inspecting these by hand, and we can check that the hybrid works similarly.

### Noise
How you will incorporate noise into your particle filter.


Some of our plans for incorporating noise includes, accounting for noise in our motion update model with drifting, using the Gaussian Model when updating weights, and by removing low weight particles as well as those with zero weights.
</br></br> 
As mentioned previously, we will test the motion model noise by first setting the noise to be zero, and verifying that the particles move according to the twist messages, then using small noise to the motion, verifying that the particles are close to the zero noise case. For resampling we can verify that only low probability particles are thrown away and that the number of high probability particles is much higher than those of lower probability. 

## Timeline
By February 2nd, we plan to finish the following functions: initialize_particle_cloud(), update_particles_with_motion_model(), and update_particle_weights_with_measurement_model(). Moreover, by February 9th, we finish the remaining functions: resample_particles(), normalize_particles(), and update_estimated_robot_pose().
