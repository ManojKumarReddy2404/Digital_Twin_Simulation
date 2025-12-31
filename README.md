**Simulation-Based Digital Twin of a Robotic Joint**

## **Project Overview**

This project presents the development of a simulation-based Digital Twin of a single-degree-of-freedom (1-DOF) robotic revolute joint. The Digital Twin is built using physics-based modeling, sensor-level simulation, and numerical execution in MATLAB.

The objective of this work is to demonstrate Digital Twin and Cyber-Physical Systems (CPS) concepts through a realistic, reproducible, and industry-oriented simulation framework.

## **Motivation**

Digital Twins are a core component of Industry 4.0, enabling:
- virtual testing,
- system validation,
- predictive analysis,
- and future hardware integration.
  
This project was developed to strengthen hands-on understanding of:
- mechanical system modeling,  
- dynamic simulation,  
- sensor realism,
- and Digital Twin architecture — without requiring physical hardware.

## **System Description**

The system modeled in this project is:
- A 1-DOF revolute robotic joint
- Actuated by a torque input
- Governed by inertia, damping, and friction
- Instrumented with virtual sensors:
  - position encoder,
  - velocity sensor,
  - torque sensor
    
The Digital Twin replicates both physical behavior and sensor-level measurements.

## **Digital Twin Scope**

The Digital Twin includes:
- Kinematic modeling
- Dynamic modeling
- Sensor modeling (quantization, noise, sampling)
- Simulation-based validation

Due to the absence of physical hardware, **realistic industrial parameters were assumed**, which is a standard and accepted approach in simulation-based Digital Twin research.

## **Tools & Technologies**

- **MATLAB**
- Numerical integration using ode45
- Physics-based rotational dynamics
- Discrete sensor simulation

  ## **Results & Validation**

The Digital Twin produces:
- Joint angle and velocity responses
- Encoder quantized output
- Velocity and torque sensor signals with noise
- Validation plots for multiple torque inputs

The results demonstrate:
- Physically consistent motion
- Correct cause–effect relationship between torque and motion
- Realistic sensor behavior

 ## **Key Learnings**
- Translating mechanical equations into executable simulation models
- Understanding the role of sensors in Digital Twins
- Validating system behavior through physics-based reasoning
- Structuring reproducible engineering projects

## **Conclusion**
This project demonstrates a complete simulation-based Digital Twin workflow, from system definition to sensor-level validation. It provides a strong foundation for advanced studies and applications in Digital Twins, CPS, robotics, and Industry 4.0.
