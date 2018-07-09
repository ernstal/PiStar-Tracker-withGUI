# PiStar-Tracker

I've tried to implement the ZWO-ASI 120MM-S camera to my exisiting non-GUI program and expand it with threading and a pygame GUI.

The GUI combines the opencv-video stream and a pygame window with buttons

The video stream is read via ZWO-ASI-SDK in a separated thread.

In order to a compact device, the ST-4 commands should be implemented via the ZWO-ASI-SDK, instead of using the GPIOs of the raspberry.

The controller for the ST-4 pulses is not implented from the non-GUI version.

But so far, I run out of time for this project.
