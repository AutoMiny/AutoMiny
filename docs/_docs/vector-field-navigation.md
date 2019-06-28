---
title: "Vector Field Navigation"
permalink: /docs/vector-field-navigation/
excerpt: "Vector Field Navigation"
toc: true
vector_field_navigation_gallery:
  - image_path: /assets/images/Figure_15cm.png
    url: /assets/images/Figure_15cm.png
    alt: "Vector force field visualization with 0.15m resolution"
---

We include a vector field approach to navigate the car around the track. The vector field uses a force matrix with forces that pull the car towards the track. This matrix is precomputed and during navigation only a lookup in the force matrix is needed to find the force vector.

### Making the force matrix
We create the force matrix using the provided `make_force_matrixs.py` file. It expects the path as a numpy array. The path is then inserted into a KD-Tree. For every point (discretized in a certain resolution) in the map we find the closest point on the path from the KD-Tree. The point we want to steer at however, is not the closest point but a point a little bit in front. This is called the lookahead point.

You can change the parameters in the `make_force_matrixs.py` file. The forces point towards a lookahead point with configurable distance.

{% include gallery id="vector_field_navigation_gallery" caption="Vector force field visualization" %}

### Navigating with the force matrix
Once we have the force matrix we can use it to navigate the car. Using the car's position we look up the appropriate force vector from the matrix. Using the car's current orientation we apply a PID controller to make the car align with the force vector.