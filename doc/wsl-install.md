# `pycrazywarm` Installation via WSL 2

While the [official documentation](https://crazyswarm.readthedocs.io/en/latest/installation.html)
explicitly says that WSL isn't supported, there are some hoops we can jump through to get it
working. I used Ubuntu 20.04 with ROS Noetic, but theoretically you could do something similar
with Ubuntu 18.04 and ROS Melodic.

## Table of Contents

1. [(Optional) Create a Separate Ubuntu Instance](#optional-create-a-separate-ubuntu-instance)
2. [Install ROS](#install-ros)
3. [Install pycrazyswarm](#install-pycrazyswarm)
4. [Install crazyflie-clients-python](#install-crazyflie-clients-pythoninstall)
5. [Connect Your Windows USB Devices with WSL Devices](#connect-your-windows-usb-devices-with-wsl-devices)

## (Optional) Create a Separate Ubuntu Instance

If you don't already have the version of Ubuntu that you want installed, skip this and
go to the next section. If you already have the version you'd like to install ROS with
but would like to separate your ROS Ubuntu instance from your normal development Ubuntu
instance, which I'd recommend if you want isolation from your normal WSL Ubuntu instance,
keep reading.

To create a fresh version of your current Ubuntu distribution (20.04 for me) without all your
packages and files, go to where your Ubuntu was installed. For me, this was something like
`C:\Program Files\WindowsApps\CanonicalGroupLimited.Ubuntu20.04onWindows_2004.2022.8.0_x64__79rhkp1fndgsc`. You probably need to be running whatever you're running as
administrator to be able to see `WindowsApps`. In there, you should see a very large file
named `install.tar.gz`. This is the fresh image we'd like use. Now, go to where your current
WSL instance is installed. For me, `$USER\AppData\Local\Packages\CanonicalGroupLimited.Ubuntu20.04onWindows_sldkfjlskdjf`.

Make a new folder in this directory of the name of the new instance you want to install:

```powershell
mkdir $USER\AppData\Local\Packages\CanonicalGroupLimited.Ubuntu20.04onWindows_ROS
```

This is our install location. Now import the fresh image as a new WSL instance by running:

```powershell
wsl.exe --import Ubuntu-20.04-ROS path\to\install\location \path\to\install.tar.gz
```

Filling in the paths as they're set up on your system. You should be able to see if that worked by doing:

```powershell
wsl -l
```

Which should give you something like this:

```powershell
PS C:\Users\ZachJW> wsl -l
Windows Subsystem for Linux Distributions:
Ubuntu-20.04_ROS (Default)
Ubuntu-20.04
```

> NOTE: you might also have to set up the new user account by following
> [this guide](https://www.cyberciti.biz/faq/create-a-user-account-on-ubuntu-linux/).
> I also had to change the default user slightly differently by following
> [this stackoverflow answer](https://askubuntu.com/a/1300672).

## Install ROS

Install your desired version of ROS on WSL2 with [this guy's blog](https://jack.kawell.us/posts/ros-windows-wsl2/).
To get Gazebo working, you'll need a few environment variables set. Add the following lines to your `~/.bashrc` file.

```bash
export DISPLAY=$(ip route list default | awk '{print $3}'):0
export LIBGL_ALWAYS_INDIRECT=1
export GAZEBO_IP=127.0.0.1
```

Part of these variables will also set up our X-server. To start that, you'll need to
open the `.xlaunch` file you saved when going through the tutorial with the right x11 configuration.

```bash
roslaunch gazebo_ros mud_world.launch
```

To double check you have that running, install `x11-apps` from the `apt` package manager:

```bash
sudo apt install x11-apps
xclock
```

If everything's working together there, you should see a little clock window pop up.

References:

- [X11-configuration](https://stackoverflow.com/questions/61110603/how-to-set-up-working-x11-forwarding-on-wsl2)
- [Gazebo Doesn't Open](https://answers.ros.org/question/301772/gazebo-command-doesnt-open-gazebo/)
- [Gazebo + RViz](https://www.youtube.com/watch?v=DW7l9LHdK5c)

## Install `pycrazyswarm`

Clone the pycrazyswarm repo:

```bash
git clone https://github.com/USC-ACTLab/crazyswarm 
```

Follow the [installation instructions](https://crazyswarm.readthedocs.io/en/latest/installation.html).
Then test the simulation by doing:

```bash
cd ros_ws/src/crazyswarm/scripts
python hello_world.py --sim
```

You should see a window pop up with a matplotlib figure showing a dot go up and down.
Make sure your X-server is running first.
Add non-root USB permissions by going to the top of the `crazyswarm` repo and running:

```bash
./pc_permissions.sh
```

References:

- [Official instructions to configure the USB permissions](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/)

## Install `crazyflie-clients-python`

Clone the repo:

```bash
git clone https://github.com/bitcraze/crazyflie-clients-python
```

Follow the [installation instructions](https://github.com/bitcraze/crazyflie-clients-python/blob/master/docs/installation/install.md#linux).
See if it worked by running:

```bash
cfclient
```

## Connect your Windows USB devices with WSL devices

If you want to actually connect to the crazyflie to flash firmware onto it or get its
radio address, you'll need to first have the dongle plugged into your laptop. If you
go to your Device Manager, you should see it. The only issue here is that your WSL
instance can't see this device, so you'll have to install a package that'll do this
for us.

First, we need to install the latest `usbipd-win` on Windows by grabbing it from
[here](https://github.com/dorssel/usbipd-win/releases). Then follow the
[instructions](https://github.com/dorssel/usbipd-win/wiki/WSL-support#wsl-setup) to
actually connect the device.

> NOTE: you'll have to have your default distro be your ROS distro since `usbipd-win` will
> just use that one.

References:

- [Blog Post with More Detail (but older packages)](https://devblogs.microsoft.com/commandline/connecting-usb-devices-to-wsl/)

Check if that worked by going to your administrative `cmd.exe` and doing:

```bash
usbipd list
```

You should see something like:

```powershell
1-2    1915:7777    Crazyradio PA USB Dongle      Shared 
...
```

Then to attach the windows busid to your WSL USB busid, do:

```powershell
usbipd wsl attach --busid 1-2
```

Then go to wsl and run `lsusb` and you should see this somewhere in the list to see if
it worked:

```bash
Bus 001 Device 003: ID 1915:7777 Nordic Semiconductor ASA Crazyradio PA USB Dongle
```

## Miscellaneous Tips

- Windows Terminal is nice to be able to switch between different WSL versions and terminals.
- If you'd like to more easily access your windows folders, you can use symbolic links to jump
  more easily between linux and windows without having to go through like 10 folders, e.g.

  ```bash
  cd ~ 
  ln -s /mnt/c/zjw/Documents/Research/ Research
  ```
