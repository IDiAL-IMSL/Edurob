# EduRob
[Homepage](https://www.imsl.fh-dortmund.de/mobile-roboter/edurob/)
## Educational robot with support for various drive kinematics

EduRob is a handy mobile robot specially designed for education. Through clever design, it is possible to cover different types of drives such as Mecanum, Omniwheel, and differential drive.

![](https://www.imsl.fh-dortmund.de/wp-content/uploads/2023/01/EduRob-Gruppenfoto-1920-768x511.jpg "")

## Detailed information:

[Getting Started](Doc/README.md)

[Partslist](Doc/Partslist/README.md)

[PCB](PCB/README.md) 

[CAD](CAD/README.md) 

## Example projects:

[SLAM](Projects/SLAM/README.md)

[Line Follower](https://github.com/blssed/EduRob-LineFollower)



---

## Background

In the robotics lab, there is a wide range of different robots available for use in teaching and for the completion of thesis projects. However, they all share one major disadvantage: they prefer to drive themselves rather than being carried. As a result, their application is limited to the lab itself and adjacent rooms.

Especially for basic training in mobile robotics such as controlling different types of drives (Mecanum, Omniwheel, or differential drive), implementing control tasks, and connecting sensors, the experimental area of the robotics lab is not necessarily required. This led to the requirement of developing a flexible platform that can cover all types of drives, and small enough to be taken in a backpack to the home office.

# Possible configurations

EduRob can be equipped with three or four motors and different types of wheels. The motors can be flexibly reconnected by loosening two screws, and other wheels can be attached to the motor shaft.

## Mecanum 

With four motors at the four outer corners, EduRob can be turned into a robot with a classic Mecanum drive. The driving behavior is thus similar to that of OmniMan and the other Mecanum robots of the IMSL.

![](https://www.imsl.fh-dortmund.de/wp-content/uploads/2023/01/Mecanum-1920-768x512.jpg "")

## Omniwheel with three wheels (120°)

With three motors, EduRob can simulate the classic omniwheel configuration. Three wheels rotated by 120° each allow for omnidirectional driving behavior.

![](https://www.imsl.fh-dortmund.de/wp-content/uploads/2023/01/Omiwheel-120-1920-768x512.jpg "")

## Omniwheel with four wheels (90°)

In addition to the classic arrangement with three omniwheels, EduRob can also simulate the special form with four omniwheels. Here, four omniwheels are arranged with a rotation of 90° to each other. In comparison to the Mecanum arrangement, the wheels are rotated by about 45°. In this configuration, EduRob can also be driven omnidirectionally.

![](https://www.imsl.fh-dortmund.de/wp-content/uploads/2023/01/Omni-90-1920-768x512.jpg "")

## Differential drive or Skid-Steer drive

The driving behavior of the Pioneer robots in the IMSL inventory is made possible by the use of four wheels. Even though the arrangement of the wheels is similar to the Mecanum arrangement, sideways movement is not possible due to the use of classic wheels without freewheeling rollers. In addition to forward and backward movement and rotation around the center of the platform, driving in circular arcs is possible. This type of drive is also called Skid-Steer drive. Due to the 4x4 drive, this configuration is particularly suitable for rough terrain. The absence of freewheeling rollers on the wheels also means they are less likely to become contaminated or blocked.

![](https://www.imsl.fh-dortmund.de/wp-content/uploads/2023/01/Skidsteering-1920-768x512.jpg "")

By using two classic wheels and one or two non-driven Omniwheels, a proper differential drive is also possible. In this case, the rotation point moves to the midpoint between the two driven wheels. While this affects the robot's turning behavior (the front part swings out), it still maintains its limited range of movement (forward, backward, turning, and driving in circles). However, the "off-road" capability is reduced by the reduction to two driven wheels, and the use of Omniwheels increases the risk of contamination of free-rolling rollers.

![](https://www.imsl.fh-dortmund.de/wp-content/uploads/2023/01/Diff-Omni-1920-768x512.jpg "")