## Network configuration
The default WiFi configuration is:

|   |    |
|:---|:---|
| SSID   | ROS   |
| PASSWORD   | elfmeter   |
| SSID   | ROS   |
| IP   | 192.168.43.XXX   |
| Gateway   | 192.168.43.1   |
| DNS   | 8.8.8.8   |

The default ethernet configuration is:

|    |    |
|:---|:---|
| IP   | 192.168.43.199   |
| Gateway   | 192.168.43.1   |
| DNS   | 8.8.8.8   |

The configuration is applied through netplan and can be changed in  ```/etc/netplan/01-netconfig.yaml``` and ```/etc/netplan/02-wireless.yaml```. If you need to change the WiFi connection, either plug a monitor and keyboard to the computer or connect through ethernet.

When changing the car's IP address the environment variable ROS_HOSTNAME must be changed as well. By default these environment variables are configured in ```~/ros-config.sh```.