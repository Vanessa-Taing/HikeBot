# 🌿 HikeBot Identifier

HikeBot is a voice-activated plant identification assistant for the outdoors. It captures images of plants, identifies species using a trained model, retrieves information, and reads it aloud — all hands-free via voice command.

![rosgraph](https://github.com/Vanessa-Taing/HikeBot/blob/dev/hikebot_identifier/media/rosgraph.png)

## 📦 Installation Guide (ROS1, Python3)

### 1. Clone the Repository

Clone the HikeBot repository into the `src` folder of your ROS1 `catkin_ws`. If you don’t have a workspace set up yet, follow these steps:

```bash
# Create and initialize the catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# Clone the HikeBot repository
cd ~/catkin_ws/src
git clone https://github.com/Vanessa-Taing/HikeBot.git

# Build the workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### 2. Create a Python Virtual Environment

Create a Python3 virtual environment (with access to system packages, which ROS needs):

```bash
python3 -m venv ~/hikebot_env --system-site-packages
source ~/hikebot_env/bin/activate
```

---

### 3. Install Python Dependencies

Install the required Python packages:

```bash
pip install -r ~/catkin_ws/src/HikeBot/hikebot_identifier/requirements.txt
```

---

### 4. Launch the Full System

To launch all components (audio detection, camera capture, plant identification, info retrieval, TTS output, and optional GUI logger):

```bash
roslaunch hikebot_identifier hikebot.launch
```
**❗❗ Important**: HikeBot detection is activated when user speaks the wakeword "Hey HikeBot"

---

### 5. [Optional] Test Individual Nodes with `rostopic pub`

You can test nodes individually by publishing to the expected ROS topics:

#### 🧪 Audio Trigger → Start Detection
```bash
rosrun hikebot_identifier plant_identifier_node_wrapper.sh
rostopic pub /voice_commands/trigger_camera std_msgs/Bool "data: true" --once
```

#### 🧪 Plant Name → Info Retriever
```bash
rosrun hikebot_identifier info_retrieval_node_wrapper.sh
rostopic pub /plant/name std_msgs/String "data: 'Aloe Vera (0.92)'" --once
```

#### 🧪 Plant Info → Audio Output
```bash
rosrun hikebot_identifier audio_output_node_wrapper.sh
rostopic pub /plant/info std_msgs/String "data: 'The Aloe Vera is known for its medicinal properties.'" --once
```

#### 🧪 Plant Info → GUI Logger
```bash
rosrun hikebot_identifier gui_logger_node_wrapper.sh
rostopic pub /plant/info std_msgs/String "data: 'The Aloe Vera is known for its medicinal properties.'" --once
```

---

### ✅ Nodes Overview

| Node Name         | Function                         | Topic Subscribed / Published        |
|------------------|----------------------------------|-------------------------------------|
| `usb_cam`         | Captures image              | `/usb_cam/image_raw`                |
| `audio_detector` | Listens for voice trigger        | `/voice_commands/trigger_camera`   |
| `plant_identifier` | Classifies plant species upon trigger     | `/plant/name`                      |
| `info_retriever` | Retrieves plant description      | `/plant/info`                      |
| `audio_output`   | Reads plant name and description aloud      | `/audio_output`                    |
| `logger_gui`     | Displays info in terminal   | `/plant/info`                      |

---

### 🛠 Notes

- The package uses usb_cam.launch and subscribes to /usb_cam/image_raw. Ensure the camera is working with `roslaunch usb_cam usb_cam-test.launch`. More info at [wiki_ros](https://wiki.ros.org/usb_cam).
- Ensure your microphone is properly detected by ALSA.

---

### 👩‍🔬 Maintainer

**Vanessa Taing**  
Project for the Juno Robot | ROS1, Python3, OpenCV, PyTorch, ALSA, TTS  
GitHub: [Vanessa-Taing/HikeBot](https://github.com/Vanessa-Taing/HikeBot)
