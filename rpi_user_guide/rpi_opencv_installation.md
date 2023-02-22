# This is userguide on how to install OpenCV in Raspberry Pi
## Refer to this link for comprehensive guide. This is the best userguide i found.

Link:
https://singleboardblog.com/install-python-opencv-on-raspberry-pi/
https://www.piwheels.org/project/opencv-contrib-python/

Note:
I am using Bullseye with Python 3.9 and the latest available opencv-contrib-python version for me is ‘4.5.5.62’ (If you are using Pi Zero you need to check for armv6l packages and if you are using Pi 3, Pi 4 or Pi Zero 2W you need to check the armv7l packages.

Please refer to above link to find available packages, versions and compatability map for
opencv-contrib-python

To install openCV for Pi 3, Pi 4 and Pi Zero 2W.
- > sudo apt update
- > pip install --upgrade pip setuptools wheel
- > sudo apt-get install -y libhdf5-dev libhdf5-serial-dev python3-pyqt5 libatlas-base-dev libjasper-dev
- > python3 -V
- > pip3 -V

## Note : Below is for Rpi 4
- > pip install opencv-contrib-python==4.5.5.62

## Note : Below is for Rpi zero 2
- > sudo pip install opencv-python==4.5.3.56
- > Later you will see ImportError: numpy.core.multiarray failed to import error. To solve this error, we need to upgrade numpy as follow
- > sudo pip install -U numpy

## Check your installations
- > python3
- > import cv2
- > cv2.__version__
- > exit()


