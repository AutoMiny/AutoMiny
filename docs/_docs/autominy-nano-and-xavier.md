---
title: "Autominy Nano and Xavier"
permalink: /docs/autominy-nano-and-xavier/
excerpt: "Autominy Nano and Xavier models"
toc: true
autominy_nano_gallery:
  - image_path: /assets/images/nano_d_.jpg
    url: /assets/images/nano_d_.jpg
    alt: "Autominy Nano assembled"
  - image_path: /assets/images/nano_description.jpg
    url: /assets/images/nano_description.jpg
    alt: "Autminy Nano exploded view"
autominy_xavier_gallery:
  - image_path: /assets/images/xavier_b_.jpg
    url: /assets/images/xavier_b_.jpg
    alt: "Autominy Xavier assembled"
  - image_path: /assets/images/xavier_description.jpg
    url: /assets/images/xavier_description.jpg
    alt: "Autminy Xavier exploded view"


---

Towards the technology being developed based on Artificial Neural Networks and Reinforcement Deep Learning and their advantages to process perception data mainly from cameras, Autominy can be upgraded to work with Jetson Nano and Jetson Xavier from NVIDIA. The Intel Stereo camera is plugged in directly into the Jetson to be processed by the inference which must be trained in an external GPU. Jetson ARM boards capabilities can be checked out [here](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) and [here](https://developer.nvidia.com/embedded/jetson-agx-xavier-developer-kit) : The Nvidia boards results are sent to the NUC through Ethernet connection in order to control the car with the desired criteria.
Additionally the camera holder has the option to install another Intel Camera, the idea behind this alternative is to provide  the car, field of view also when is driving backwards.
The components and the cars with the updated feature can be observed in the following galleries.

{% include gallery id="autominy_nano_gallery" caption="Autominy Nano" %}
{% include gallery id="autominy_xavier_gallery" caption="Autominy Xavier" %}
