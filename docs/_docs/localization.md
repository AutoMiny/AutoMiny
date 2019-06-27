---
title: "Localization"
permalink: /docs/localization/
excerpt: "Localization"
toc: true
---

Localization for the car is done using a combination of wheel odometry and road marking localization. Since both modules only provide relative localization an initial localization must be given. This can either be provided automatically by using our fake GPS modules with camera mounted in the ceiling or manually by using the initial pose tool in Rviz.

>### Kalman filter
We use the `robot_localization` package to fuse both wheel odometry and the localization from the `road_marking_localization` package.