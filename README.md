# Consistent-formation-control-algorithm
Consistent formation control algorithm in ROS2!

The node graph is like:

![a node graph](./image/rosgraph.png)

## What it can do?

~~Do a lot...~~

Such as formation control, by publishing topic MC/set_offsets...

You can test your control algorithm, while I just use position control...

It may work, I hope so...

## Test video

In test_video, github do not support open video in markdown...

## How to use it

It's based ros2 and control toolbox. You need them first.

Install ROS2 and Control Toolbox.(You may find them in github...)

Install gazebo.

~~Copy "model/**my_model**" to the model path of gazebo.~~ No need.

Then build by command "**colcon build**".

**That's all building step.**

Next, run command "**gazebo world/three_aircraft_formation.world**"

~~Then run command "**ros2 run three_aircraft_control three_aircraft_control_demo**"~~

Then run command **ros2 launch three_aircraft_control t_a_c.py.**

You may see something great.

## Update log

03.01: Add ct in code. It's 23:36 and I think it's too late to work. Still I do some work... Update REDEME

03.02: Clean code and so on...

Add limit of line speed... **It's more stable now!**

Fix dividing zero bug...

Add topic "MC/set_offsets" for formation setting, you can publish the topic to change formation...

Add two new video...

03.03: Add PID Controller, there is no  static error anymore...

Test in a real robot! It works!

But there comes many useless code...

Add a video about the difference between PID and position control...

03.03: Add a headle file "sim_car_system.hpp", which is the model of the car...

Working on MPC, but a little difficult...

03.06: Find a way to build a MPC with control toolbox, finally it's over... for now...

03.16: I ADD A MPC ON ROBOT2!!! Thanks to [JunshengFu/Model-Predictive-Control](https://github.com/JunshengFu/Model-Predictive-Control) that gives me inspiration, I change his code to my_mpc.hpp, mostly the model and the cost function.

03.17: Clean some code...

03.20: Add a launch start file, now must start with ros2 launch if you want to use distributed controller.

Add distributed controllers...

Update node graph...

03.25: Optimize cost fuction, there is a typement of a avoiding barrier...

Add repulsion force to avoid barrier... (Need to update params...)

Add some new video...

Clean model file...

## TODO:

1. ~~Add a subscription to change every robot's offset~~, choose the leader robot and so on.
2. ~~Add limit...（It's import!)~~
3. Add a controller like ~~PID~~ or MPC.(~~Now it's position control...~~)
4. ~~Fix bug...~~ **Now no bug found!**
5. ~~Make source away from hpp...~~(It's not import but easy...)
6. ~~Get some Wend-Oil~~, **I wanna sleep!!!** ~~And I'm gonna sleep!!!~~ ~~Got a lot of wend-oil now... I have to do something in my dissertation... Oh Susanna, don't you cry for me...~~
7. ~~Test algorithm in real system...~~(Hope no ~~error~~...)
8. Clean code! It's too redundancy!!!!! Code logic becomes rediculous??? (But works......)
9. Fix PID output, it's negetive? To make code work, I put a - before the output......(That's really stupid...)
10. ~~Add MPC!~~
11. **CLEARN YOUR CODE! IT IS REDUNDANCY AGAIN!!!**
12. Avoiding Barrier!
