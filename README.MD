### Install udev rules if Arduino is connected over USB
 sudo ln -s $(pwd)/99-hue.rules /etc/udev/rules.d/99-hue.rules
 sudo udevadm control --reload
 sudo udevadm trigger

### Enable UART if Arduino is connected on RX/TX pins
## Add 
 enable_uart=1 
 dtoverlay=pi3-disable-bt
 core_freq=250
#to /boot/config.txt

## Delete
 console=serial0,115200
# from /boot/cmdline.txt

## Execute
 sudo systemctl disable hciuart

### install pyautogui
 pip install python-xlib
 sudo apt-get install scrot
 sudo apt-get install python-tk
 sudo apt-get install python-dev
 pip install pyautogui

### install wxpython
