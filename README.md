# Human Pose from Image Data to NAO

- [Robin User Manual](https://docs.google.com/document/d/1SHhnQqBtguoWpOpT3Z33REXIgwW83DHRgTlEKfIoUzg/edit?usp=sharing)
- [Robin Thesis](https://theses.liacs.nl/pdf/RobbinBorst.pdf)

# Kinect scale
- Kinect values are taken from infrared 3d camera : explanation [here](https://medium.com/@lisajamhoury/understanding-kinect-v2-joints-and-coordinate-system-4f4b90b9df16)

# Install & Setup

OpenVino was run by us on a Windows computer, while the simulator was set up on a Unix System. While OpenVino probably works on Unix too, it needs a 6th to 10th generation Intel Core and Intel Xeon Processor. Our Unix computer did not meet the requirement and we had to split the workforce in two people, so we reached this compromise.

## OpenVino Setup (Done on Windows)
1. Be sure you use python 3.6 
2. cd C:\Program Files (x86)\IntelSWTools\openvino\bin
3. run setupsvars.bat (This will initilize OpenVINO environment)

## Dependencies to install (Done on Unix)

All libraries used were found in the zip file with Robin's code, in Windows format. In this repo you will find present only NAOH25, as it's only a selection of the original library and it might have been modified. The others were too heavy weight to put on github. You can check out the original zip file, maybe the files there help you install the libraries on Windows.

- Download Aldebaran SDK [here](http://doc.aldebaran.com/2-1/dev/cpp/install_guide.html)
- Setup qibuild [here](http://doc.aldebaran.com/qibuild/beginner/qibuild/aldebaran.html)

## Simulators

For the demo we used Webots, but Choreographe should also work, maybe with a slightly different setup

## Webots Simulator Setup
For simulations, I used [Webots](https://cyberbotics.com/?tab-language=python).
You can find installation instructions [here](https://cyberbotics.com/doc/guide/installing-webots?tab-language=python&tab-os=linux).
I used the tarball installation for Linux and it worked with no problems!
A simulated robot is controlled by a controller file. You can choose the file it runs in the app GUI by modifying the _controller_ field of the robot node. Or, for more control and debugging, you can set the _controller_ field to __extern__ and then running the controller from the command line. This way, you can see the console output and even run the controllers with the debugger. You can find more information on controllers [here](https://cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-language=python&tab-os=linux).

### Naoqisim

We need Naoqisim to run the simulated robot with the Naoqi C++ SDK.
Naoqisim can also be employed to use Choreographe with Webots.
Follow install instructions [here](https://github.com/cyberbotics/naoqisim).
If after running the *make* command in the root directory, it fails at some point, try running the app anyway. It might work.
Choose the world without camera when opening Webots, from the worlds available in the *naoqisim* package. We don't need the camera for our project, and Nao will move smoother in the simulation.


### Choregraphe simulator setup

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

# Run Project

## Server

1. `python server/src/app.py --images {input\_filepath}`
2. output will be saved in server/outputs/{input\_filename}

## Client

1. `cd client/src`
2. `qibuild configure`
3. `qibuild make`
4. Start Webots
5. Open *worlds/nao.wbt*
6. `build\_folder/bin/sdk/proj`
