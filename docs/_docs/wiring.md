---
title: "Wiring"
permalink: /docs/wiring/
excerpt: "Wiring of modules and controller board"
toc: true
wiring_gallery:
  - image_path: /assets/images/control_board_connections.jpg
    url: /assets/images/control_board_connections.jpg
    alt: "Control board connections"

---

In case of  needed maintenance or damage in one of the modules, they can be easily replaced, all the interconnections are done in the controller board, the module needs to be unplugged and connected in the correct slot of the board. Motors, LEDs and serial to USB converter have an easy plug in connector, the Engine connector has a poka yoke avoiding the user to connect it in the wrong direction, Steering motor and LED should be treated carefully and  follow the order indicated on Fig. 5, letter F is indicated for the Feedback pin  of the servo motor and D in both cases is the Data pin, notice that even 4 pins are available for the LED connector row, only three pins are active.
The user should be careful while changing the regulator or while inserting power wires for the CPU and GPU and follow the order indicated on Fig. 5, a wrong connection can result in serious damage for the modules.
The Fuse should be changed depending on the total load of the system (autominy model) 8A for Autominy core and 10A for Nano and Xavier.
As the board is a 4 layer PCB it is recommendable to change it completely if some shortcut happen during the disassembling and assembling of the modules.

{% include gallery id="wiring_gallery" caption="Control board connections" %}

