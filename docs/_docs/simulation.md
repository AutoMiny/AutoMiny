---
title: "Simulation"
permalink: /docs/simulation/
excerpt: "Simulation using Gazebo"
toc: true
---

**Note:** Simulation is still under development and might be subject to changes.
{: .notice--danger}

Simulation is provided by Gazebo. It uses the car's URDF description to create a physical model of the car. The car's sensors are modeled as sensors.

### Launch the simulation

```bash
roslaunch autominy Simulated.launch
```