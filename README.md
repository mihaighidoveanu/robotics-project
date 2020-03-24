# robotics-project

Main paper of the project is [here](https://theses.liacs.nl/pdf/RobbinBorst.pdf).
You find above guidelines and methods for each of our tasks.

## OpenVino Setup
1. Be sure you use python 3.6 (i am using an environment in anaconda for that)
2. cd C:\Program Files (x86)\IntelSWTools\openvino\bin
3. run setupsvars.bat (This will initilize OpenVINO environment)
4. cd App folder
5. run app.py

## Choregraphe simulator setup

Search for choregraphe and download it from [here](http://doc.aldebaran.com/2-4/dev/community_software.html#retrieving-software).

For this error :
```
...lib/libz.so.1: version `ZLIB_1.2.9' not found (required by /usr/lib/x86_64-linux-gnu/libpng16.so.16)
```

use this :

```
cd /your_choregraphe_directory/lib/ (the directory in which is present libz.so.1)
sudo mv libz.so.1 libz.so.1.old
sudo ln -s /lib/x86_64-linux-gnu/libz.so.1
```

## Webots Simulator Setup
For simulations, I used [Webots](https://cyberbotics.com/?tab-language=python).
You can find installation instructions [here](https://cyberbotics.com/doc/guide/installing-webots?tab-language=python&tab-os=linux).
I used the tarball installation for Linux and it worked with no problems!
A simulated robot is controlled by a controller file. You can choose the file it runs in the app GUI by modifying the _controller_ field of the robot node. Or, for more control and debugging, you can set the _controller_ field to __extern__ and then running the controller from the command line. This way, you can see the console output and even run the controllers with the debugger. You can find more information on controllers [here](https://cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-language=python&tab-os=linux).

## Naoqisim

To use Choregraph with Webots, follow instructions [here](https://github.com/cyberbotics/naoqisim).
If after running the *make* command in the root directory, it fails at some point, try running the app anyway. It might work.
Choose the world without camera when opening Webots, from the worlds available in the *naoqisim* package. We don't need the camera for our project, and Nao will move smoother in the simulation.

