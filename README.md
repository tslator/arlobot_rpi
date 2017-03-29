# arlobot_rpi

## Background
This repo was originally an adaptation and extension of Arlobot to use the Raspberry Pi and the Psoc4.  The motivation for this was to
replace the Parallax Propellor board solution.  In the Chrisl8 implementation, there are two Parallax boards along with external glue
logic and circuitry.  I was looking for an hardware design that would minimize boards and components.  While the Psoc4 is great technology
and went a long way toward acheiving the goals, it became clear pretty quickly that the Psoc4 did not have sufficient resources to support 
two PWMs and two 32-bit quadrature encoders.  Further research led the Sparkfun Freesoc4 board.  With two Psoc5LPs, it has more than
sufficient resources for Arlobot with room to grow.

## Other Considerations
In the Chrisl8 implementation, a netbook physically resides on the robot.  It runs ROS and provides some CPU horsepower.  I replaced the
netbook with an Intel I7 mini-itx board and a DC power supply used in automotive audio applications.  The intent was to use the mini-itx
board for high-level ROS node implementations, such as SLAM, image processing, deep learning, etc., and use the Raspberry Pi for base
operations, e.g. control and sensing nodes.  The mini-itx and Raspberry Pi communicate via Ethernet and the Raspberry Pi communicates with
the Freesoc.

## Reality
During the integration of the Raspberry Pi with the Freesoc, I discovered a bug in the I2C slave implementation on the Raspberry Pi.
Consequently, I could not acheive the throughput needed to send velocity commands and receive odometry.  So, I went looking for an
alternative.  Quickly, the Beaglebone Black became a contender.  All of the current software now works with a Beaglebone Black; however,
truthfully, I believe there ever was a need Beaglebone-specific software.  The BBB runs ROS just like Raspberry Pi, but does not suffer
from the I2C slave bug.  So, technically, this repo should be named arlobot_bbb, or renamed to remove the qualification, maybe arlobot_low.

## Future
Having two 'computers' on the robot now seems a bit heavy-handed.  I have recently discovered the NVIDIA TX line of development kits.  The
next, and hopefully last iteration of Arlobot will use the TX2.  With the TX2, I will be able to consolidate the mini-itx and the Raspberry 
Pi functionality.  The TX2 has a dual-core processor and 256 Cuda Cores, it runs Ubuntu 64-bit and ROS, supports I2C (hopefully with no
slave bug, SPI, and CAN interfaces as well as digital camera inputs.  With the CAN interface available, I'm planning to create CAN-based
sensor modules to support up to 16 ultrasonic and 16 infrared sensors.

![alt text](http://github.com/tslator/arlobot_rpi/raw/master/src/images/arlobot.jpg)
