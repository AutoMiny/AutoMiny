## Updating
The car can either be updated from source code or using debian packages (Odroid XU only).

### Compiling from source
We can also compile from source on the car directly but keep in mind that a full compilation will take quite some time and will keep the car busy.

```bash
cd /opt/autominy/catkin_ws
git pull
catkin build
sudo reboot
```

### Updating from debian package (Odroid XU4 only)
Download the most recent debian package from [our FTP server](http://ftp.imp.fu-berlin.de/pub/autonomos/data/modelcar/debian-packages). Copy the debian package to the car using scp:

```bash
scp autominy-basic-packages.deb root@192.168.43.<CAR_NUMBER>:.
```

On the car install the debian package using gdebi which will resolve and install any needed dependencies automatically. Installing the package will override any changes done in `/opt/autominy`:

```bash
gdebi --n autominy-basic-packages.deb
```

### Creating a debian package (Odroid XU4 only)
The debian packages can be created inside a docker environment. Install Docker CE by following the [installation guide](https://docs.docker.com/install/linux/docker-ce/ubuntu/). Since we are building for armv7 we need qemu-armv7 for emulation:

```bash
sudo apt install qemu qemu-user-static qemu-user binfmt-support qemu-system-arm
```

Clone our debian packaging docker repository and start building the docker image:
```bash
git clone https://github.com/autominy/debian-packaging
cd debian-packaging
docker build .
```

Afterwards you can use the docker container to create a debian package:

```bash
docker run -it autominy bash
```

Compilation takes quite a while (10 - 20 minutes) since we are emulating an arm processor using qemu. Once compilation is done `autominy-basic-packages.deb` can be copied from the docker container:

```bash
docker ps # find your container ID
sudo docker cp <CONTAINER_ID>:/opt/debian-packaging/autominy-basic-packages.deb .
```