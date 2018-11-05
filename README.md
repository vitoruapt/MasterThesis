## Master Thesis - Alberto Franco

My name is Alberto Franco and I'm going to take my Master's degree in Automation Engineering at the Università degli Studi di Padova. I’m beginning my Master Thesis in the Laboratório de Automação e Robótica (LAR) at the Mechanical Department of the Universidade de Aveiro. My supervisor is Professor Vitor Santos.

The purpose of this blog is to, not only, document the progress of my master thesis, but also help me organize my work and my tasks for this project.

---------------------------------------------------------------------------------------------------
### Week 0 - (2nd October 2018 / 8th October 2018)
This week marks the beginning of my master thesis.

I started with the installation of the ROS environment (ROS Kinetic Kame) in my personal computer and I did all of the beginner and intermediate ROS tutorials at <http://wiki.ros.org/ROS/Tutorials> and C++/OpenCV tutorials.

I thought about the thesis proposals and I started to do research about it.

---------------------------------------------------------------------------------------------------
### Weeks 1&2 - (9th October 2018 / 22nd October 2018)
During these weeks I continued to practice with ROS and C++ doing simple exercises. Moreover these weeks were also dedicated to do some initial research and to write the preliminary report. This was important for me, not only, to introduce myself to the process of autonomous navigation and the projects that have already been developed in this field, but also, to organize my thoughts in tasks to come in this project.

[Preliminary Report](https://github.com/AlbertoFranco/MasterThesis/blob/master/Thesis_Latex/Preliminary%20Report/Preliminary_Report.pdf "Click here to open Preliminary Report.pdf")

---------------------------------------------------------------------------------------------------
### Week 3&4 - (23rd October 2018 / 5th November 2018)
During these weeks I started to implement the basic idea of ​​my project in MATLAB. I started by creating a simple nonlinear bicycle model to describe the dynamics of the ATLASCAR2. In these first examples I assumed that all the states are measurable. At the nominal operating point, the ATLASCAR2 drives east at a constant speed. I obtained a linear plant model at the nominal operating point and I have converted it into a discrete-time model to be used by the model predictive controller. The obstacles in this example are moving cars with the same size and shape of the ATLASCAR2. In the Model Predictive Control design I added different constraints on the throttle rate and on the steering angle rate and also I have specified mixed I/O constraints for obstacle avoidance maneuver. Here, I used an adaptive MPC controller because it handles the nonlinear vehicle dynamics more effectively than a traditional MPC controller. A traditional MPC controller uses a constant plant model. However, adaptive MPC allows me to provide a new plant model at each control interval. Then finally I have simulated the two different situations with only 1 moving obstacle and also with N moving obstacles (for simplicity the obstacles are in the middle of the center lane). For the collision avoidance with only 1 moving obstacle, I developed also the model in Simulink.

---------------------------------------------------------------------------------------------------
### Week 5 - (6th November 2018 / 12th November 2018)


---------------------------------------------------------------------------------------------------
### Week 6 - (13th November 2018 / 19th November 2018)


---------------------------------------------------------------------------------------------------
### Week 7 - (20th November 2018 / 26th November 2018)


---------------------------------------------------------------------------------------------------
### Week 8 - (27th November 2018 / 3rd December 2018)


---------------------------------------------------------------------------------------------------
### Week 9 - (4th December 2018 / 10th December 2018)


---------------------------------------------------------------------------------------------------
### Week 10 - (11th December 2018 / 17th December 2018)


---------------------------------------------------------------------------------------------------
### Week 11 - (18th December 2018 / 24th December 2018)


---------------------------------------------------------------------------------------------------
### Week 12 - (25th December 2018 / 31th December 2018)


---------------------------------------------------------------------------------------------------
### Week 13 - (1st January 2019 / 7th January 2019)


---------------------------------------------------------------------------------------------------




