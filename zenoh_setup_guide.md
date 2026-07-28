# Running Zenoh RMW on your Local PC

This guide covers the necessary steps to successfully run `rmw_zenoh_cpp` on your local Ubuntu PC for ROS 2 Humble, bridge your PC to a robotic system, and troubleshoot common issues.

## 1. Prerequisites

First, ensure the Zenoh RMW implementation and its C-vendor libraries are installed on your system.

```bash
sudo apt update
sudo apt install ros-humble-rmw-zenoh-cpp ros-humble-zenoh-c
```

> [!IMPORTANT]
> **The Humble Library Bug**
> In ROS 2 Humble, the `rmw_zenoh_cpp` package often forgets to append its core library (`libzenohc.so`) to your system's library path. If this happens, your ROS 2 nodes will crash silently when trying to use Zenoh. You **must** manually add it to your `LD_LIBRARY_PATH`.

---

## 2. Quick Test (Single Terminal)

If you just want to test Zenoh in a single terminal session without modifying your system configuration permanently, run this block of code:

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Fix the missing library path
export LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:$LD_LIBRARY_PATH

# 3. Tell ROS 2 to use Zenoh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# 4. Test discovery
ros2 topic list
```

If your Wi-Fi network allows UDP multicast, your PC will automatically discover the Zenoh router running on your robot and populate the topics!

---

## 3. Permanent Setup (Recommended)

To avoid typing the export commands every time you open a terminal, you can add them to your `~/.bashrc` file so they run automatically.

```bash
echo 'export LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> ~/.bashrc
source ~/.bashrc
```

Now, any new terminal you open will automatically be ready to communicate over Zenoh.

> [!TIP]
> **The ROS 2 Daemon Cache**
> If you recently switched from Cyclone DDS to Zenoh, you might not see any topics immediately. This is because the ROS 2 background daemon caches old data under the old DDS protocol. Simply run `ros2 daemon stop` in your terminal to kill the cache. The next time you run `ros2 topic list`, the daemon will restart cleanly with Zenoh.

---

## 4. Troubleshooting: Bridging Networks

If you can't see the topics on your local PC, your Wi-Fi network (common in university or corporate environments) might be blocking UDP multicast traffic. This prevents Zenoh's auto-discovery from working.

You can bypass this by telling your PC's Zenoh client to connect **directly** to your robot's IP address.

```bash
# Replace 192.168.8.147 with your robot's actual IP
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.8.147:7447"]'
ros2 topic list
```

---

## 5. (Optional) Running a Background Router

By default, Zenoh works out of the box locally because one device (usually the robot) hosts a router (`zenohd`). If you ever need your PC to host its own router (for example, to act as a permanent bridge across complex networks), you can run it as a background systemd service.

**Create the Service File**
`sudo nano /etc/systemd/system/zenoh-router.service`

**Paste the Configuration**
```ini
[Unit]
Description=Zenoh Router for ROS 2
After=network.target

[Service]
Type=simple
User=YOUR_LINUX_USERNAME
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && export LD_LIBRARY_PATH=/opt/ros/humble/opt/zenoh_cpp_vendor/lib:$LD_LIBRARY_PATH && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && exec ros2 run rmw_zenoh_cpp rmw_zenohd"
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

**Enable and Start**
```bash
sudo systemctl daemon-reload
sudo systemctl enable zenoh-router.service
sudo systemctl start zenoh-router.service
```
