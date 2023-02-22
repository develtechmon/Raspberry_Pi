# This is userguide on how to install OpenCV in Raspberry Pi
## Refer to this link for comprehensive guide. This is the best userguide i found.

Link:
- > https://singleboardblog.com/install-python-opencv-on-raspberry-pi/
- > https://www.piwheels.org/project/opencv-contrib-python/

Note:
I am using Bullseye with Python 3.9 and the latest available opencv-contrib-python version for me is ‘4.5.5.62’ (If you are using Pi Zero you need to check for armv6l packages and if you are using Pi 3, Pi 4 or Pi Zero 2W you need to check the armv7l packages.

Please refer to above link to find available packages, versions and compatability map for
opencv-contrib-python

To install openCV for Pi 3, Pi 4 and Pi Zero 2W.
- > sudo apt update
- > pip install --upgrade pip setuptools wheel
- > sudo apt-get install -y libhdf5-dev libhdf5-serial-dev python3-pyqt5 libatlas-base-dev libjasper-dev
- > sudo apt install libwayland-cursor0 libxfixes3 libva2 libdav1d4 libavutil56 libxcb-render0 libwavpack1 libvorbis0a libx264-160 libx265-192 libaec0 libxinerama1 libva-x11-2 libpixman-1-0 libwayland-egl1 libzvbi0 libxkbcommon0 libnorm1 libatk-bridge2.0-0 libmp3lame0 libxcb-shm0 libspeex1 libwebpmux3 libatlas3-base libpangoft2-1.0-0 libogg0 libgraphite2-3 libsoxr0 libatspi2.0-0 libdatrie1 libswscale5 librabbitmq4 libhdf5-103-1 libharfbuzz0b libbluray2 libwayland-client0 libaom0 ocl-icd-libopencl1 libsrt1.4-gnutls libopus0 libxvidcore4 libzmq5 libgsm1 libsodium23 libxcursor1 libvpx6 libavformat58 libswresample3 libgdk-pixbuf-2.0-0 libilmbase25 libssh-gcrypt-4 libopenexr25 libxdamage1 libsnappy1v5 libsz2 libdrm2 libxcomposite1 libgtk-3-0 libepoxy0 libgfortran5 libvorbisenc2 libopenmpt0 libvdpau1 libchromaprint1 libpgm-5.3-0 libcairo-gobject2 libavcodec58 libxrender1 libgme0 libpango-1.0-0 libtwolame0 libcairo2 libatk1.0-0 libxrandr2 librsvg2-2 libopenjp2-7 libpangocairo-1.0-0 libshine3 libxi6 libvorbisfile3 libcodec2-0.9 libmpg123-0 libthai0 libudfread0 libva-drm2 libtheora0

- > python3 -V
- > pip3 -V

## Note : Below is for Rpi 4
- > sudo pip install opencv-contrib-python==4.5.5.62
- > sudo pip install -U numpy / sudo pip install --upgrade numpy

## Note : Below is for Rpi zero 2
- > sudo pip install opencv-python==4.5.3.56
- > *Later you will see ImportError: numpy.core.multiarray failed to import error. To solve this error, we need to upgrade numpy as follow*
- > sudo pip install -U numpy

## Check your installations
- > python3
- > import cv2
- > cv2.__version__
- > exit()


