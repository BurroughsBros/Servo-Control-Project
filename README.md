# Servo Control Project

# Introduction

This Project aims to design a proportional-derivative (PD) control
system for a motor with an encoder attached and then implement a
discretized version onto that motor system. This will be done by using
prior measured properties of a motor system to calculate a first-order
estimation of the system. Using that, we will consider the output and
input of the system, i.e., the encoder directly attached to the motor's
output shaft and an H-bridge motor controller, to design a control
system that can precisely control the speed and position of the motor.
We will then discretize that controller to be coded in C to implement on
an STM32 microcontroller. Which will use an H-bridge controller to run
the motor while getting feedback from the encoder. To start, knowing the
physical system we used in this project will be helpful. Below are
several futures and descriptions of the system we used.

# System Physical Description

![Figure 1: Motor Encoder
System](Pictures/1000000000000FC000000BD077FB084D.jpg "fig:"){width="6.2327in"
height="5.7681in"}**Note:** The fan wheel is not attached to the
protractor shaft.

- Motor: Permanent-magnet DC motor with a normal input voltage of 24
  volts and a max of 30 volts.
- Encoder: Convert shaft's rotational motion into two digital high-low
  square wave.
- Clutch: engages and disengages the fan wheel to the tachogenerator
  wheel.
- Fan wheel: A wheel with propellers that creates drag to introduce a
  disturbance in the system.
- Engagement wheel: what the clutch used to engage the fan wheel and
  tachogenerator wheel.
- Gears: a 2.5:1 ratio that connects the hall-effect shaft and
  motor-generator shaft.
- Servo output / Protractor: This is what we are considering the
  output of the system. Protractor for test measuring.

# Calculations

Considering the inputs and outputs of this system we came up with the
following block diagram that implements a PD control system on our first
order approximation.

- ![Figure 2: Block Diagram of Motor Servo PD Control
  system](Pictures/1000000100000C1C0000070B9A1A7B0D.png "fig:"){width="6.9252in"
  height="2.55in"}K~~P~~ is proportional gain of the PD system
- K~~DAC~~ is a gain that converts a count per-second error correction
  into a PWM duty cycle that is interpreted by the H-bridge motor
  controller to spin the motor at a rate.
- K is the gain of the open loot system.
- K~~r~~ is a gain that converts the position of the motor in radians to
  a count from the encoder.
- T is the time constant of the open loop motor system.
- T~~d~~ is the time constant of the PD controller
- T~~f~~ added pole far away from the time constant of the system to
  make the system realizable. (1/10 T)

From this block diagram we came up with the following equation
representing the open loop system of the motor (G~~OL~~) and the PD
control system (G~~C~~).

$$G_{\mathit{OL}} = \frac{kk_{r}k_{\mathit{DAC}}}{s{({1 + \mathit{sT}})}{({1 + \mathit{sT}_{F}})}}$$

$${G_{C} = k_{P}}{({1 + \mathit{sT}_{D}})}$$

# System Values

To implement our system in Matlab to design and test the PD control
system, we first need to get some characteristics of the open loop
system. To start doing this, we ran our motor at several PWM Duty Cycles
and recorded the Wheel Saw-tooth period, and then converted that into a
radians per-second value. All of this data is shown below in Table 1.

![Table 1: Open Loop
Measurements](./ObjectReplacements/Object 3 "fig:"){width="2.9835in"
height="3.3535in"}We then plotted PWM input against the Velocity to find
the gain of the system (k) from the linear trend line. This graph and
slope are shown below in Figure 3.** Slope/Gain k = 0.06897**

![Figure 3: PWM Vs. Velocity
Graph](Pictures/2000041E00003E87000023259C4732B4.svm "fig:"){width="6.302in"
height="3.5425in"}  
We then determined the time constant by plotting the step response on an
oscilloscope and measuring the time it took to get to 63% of the final
value. That plot is shown below in Figure 4. From that, we found the
time constant to be 55.96 ms.

![Figure 4: Plotted Step Response](Pictures/1000000100000500000002D0A6804EA6.png)

Plotted Step Response

After this, we then found kr by looking at the outputs of the encoder
when the motor was spinning. We looked at both the encoder's check
pulses that happen once per revolution and the 2 encoder pulses. We
noted the periods of all of them and then divided the check pulses
period by the encoder's period to find the pulses per revolution, aka
kr. Those measurements are shown below in Figures 5 & 6.

![Figure 5: Encoder\'s Check Pulse
Output](Pictures/1000000100000500000002D0C89C59BB.png "fig:"){width="6.9252in"
height="3.8953in"}![Figure 6: Both Encoder\'s Pulse
Outputs](Pictures/1000000100000500000002D0BFFACF6E.png "fig:"){width="6.7173in"
height="3.7783in"}

From the above output we got a pulse per revolution **(k**~~**r**~~**) of
1150.**

# Matlab Implementation & Simulation

From everything shown above, we then made the following Matlab live
script to design and simulate the PD control system for the motor
encoder. This gave us an excellent PD controller with an infinite Gm and
a Pm of 41.4 degrees.

# Discretization & Implementation

To Implement this Controller on our STM32 we first must discretize it.
To do that we used Tustin's approximation also known as Bilinear
transform where s is approximated in the z plane by the following
approximation.

$${s \approx \frac{2}{T}}{(\frac{z - 1}{z + 1})}$$

Applying this to our controller it is easiest to first simplify it as
follows.

$$G_{c}{{(s)} = {K_{p} \ast {({1 + {s \ast T_{d}}})}} = {K_{p} + K_{d}}}s$$

Where the $$K_{d} = 0.733$$ as found in the final Matlab G~~cPD~~ output.
With this simpler form, we can now easily apply Tustin's approximation
by replacing the s with z plane approximation as shown below.

$$G_{c}{{(z)} = {K_{p} + {K_{d} \ast \frac{2}{T_{s}} \ast \frac{z - 1}{z + 1}}}}$$

Where T~~s~~ is the time constant of the open loop system. This is because
we do not need, or want, the controller to correct faster than the
physical system can react. Expanding and rearranging this we can obtain
the following.

$$G_{c}{{(z)} = \frac{{({K_{p} + \frac{2K_{d}}{T_{s}}})}{z + {({K_{p} - \frac{2K_{d}}{T_{s}}})}}}{z + 1}}$$

This corresponds to the final difference equation below.

$$m{{(k)} = {({K_{p} + \frac{2K_{p}}{T_{s}}})}}e{{(k)} - {({K_{p} - \frac{2K_{d}}{T_{s}}})}}e{({k - 1})}$$

where:

- k represents the current time step.
- k−1 is the previous time step.
- m(k) is the control output (used as the PWM signal).
- e(k) is the current error.
- e(k−1) is the previous error.

We now can use this difference equation in our Microcontroller to
control our servo's speed and position. But some calculations will only
need to be done once so to make the program more efficient we make those
α1 (alpha1) and α2 (alpha2) as shown below and only calculate them once.

$${\alpha_{1} = {({K_{p} + \frac{2K_{p}}{T_{s}}})}};{\alpha_{2} = {({K_{p} - \frac{2K_{d}}{T_{s}}})}}$$

Below are highly annotated code snip-its that explain the implementing
of our PD controller on our STM32.


# Results

When then set up the system as shown below and tested it's response with
an oscilloscope connected to the encoder's outputs.

![Figure 7: Set Up
System](Pictures/1000000000000FC000000BD04EF8E66E.jpg "fig:"){width="4.8543in"
height="3.8437in"}We then input a move from 0° to 45° and recorded the
following output on the oscilloscope shown on the next page. This
clearly shows the system moving quickly at first to get to the desired
position and then slowing down when it approaches the final position.
This indicates that our system is working as expected and is responding
quickly and accurately to given inputs.

# {width="9.4689in" height="5.3256in"}

# Conclusion

In this project, we successfully designed and implemented a
proportional-derivative (PD) control system for a motor with an encoder,
enabling precise control of both speed and position. By first obtaining
key parameters of the open-loop system, including the motor's gain and
time constant, we created an effective PD controller using a first-order
system approximation. The controller was then discretized using
Tustin's approximation for implementation on an STM32 microcontroller.
The PD controller was executed in real time, adjusting the motor's PWM
signal based on feedback from the encoder.

Through detailed simulation in Matlab, we were able to fine-tune the
controller, ensuring that it could achieve fast, stable responses. The
final system, tested in the physical setup, showed that the motor could
quickly reach the desired position and slow down as it approached the
target, demonstrating the controller's efficacy. The encoder feedback
provided precise position tracking, and the system performed as
expected, with smooth transitions and accurate motor control.

Overall, the project successfully demonstrated how to discretize and
implement a PD control system in an embedded system environment,
achieving the desired performance in both simulation and real-world
testing. The motor system exhibited good response characteristics,
validating the design and implementation approach for real-time control
systems.
