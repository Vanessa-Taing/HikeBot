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

# Provide execution permission to all files
chmod +x *
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

### ✅ Nodes Pub/Sub Relationship

| Node Name         | Function                         | Topic Subscribed / Published        |
|------------------|----------------------------------|-------------------------------------|
| `usb_cam`         | Captures image              | `/usb_cam/image_raw`                |
| `audio_detector` | Listens for voice trigger        | `/voice_commands/trigger_camera`   |
| `plant_identifier` | Classifies plant species upon trigger     | `/plant/name`                      |
| `info_retriever` | Retrieves plant description      | `/plant/info`                      |
| `audio_output`   | Reads plant name and description aloud      | `/audio_output`                    |
| `logger_gui`     | Displays info in terminal   | `/plant/info`                      |

---

---
### Node Overview & Technologies

| Node Name                     | Purpose                                                      | Tools/Technologies Used                                                                                                                       |
| ----------------------------- | ------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `audio_detection_node.py`     | Detects wake word ("Hey HikeBot")                            | - [Porcupine](https://github.com/Picovoice/porcupine) wake word engine  <br> - Custom model: `model/Hey-Hike-Bot_en_linux_v3_0_0.ppn`         |
| `camera_node.py` *(obsolete)* | Captures image frames from USB camera *(now uses `usb_cam`)* | - OpenCV `cv2.VideoCapture(2)`  <br> - `CvBridge` (ROS ↔ OpenCV image conversion)                                                             |
| `plant_identifier_node.py`    | Classifies plant species from image                          | - TensorFlow Lite (model: `model/model_tf213_compatible.tflite`)  converted from [Original model](https://www.kaggle.com/code/muhammadfaizan65/plant-species-classification-vgg16/output)<br> - Scikit-learn `LabelEncoder` (`label_encoder.pkl`) for label decoding |
| `info_retrieval_node.py`      | Retrieves plant description from name                        | - JSON-based database (`config/plant_descriptions.json`)  <br> - Dictionary lookup by plant name                                              |
| `audio_output_node.py`        | Speaks out plant name and description                        | - `pyttsx3` (offline TTS)  <br> - Voice rate set with `self.engine.setProperty('rate', 150)`                                                  |
| `gui_logger_node.py`          | Logs and displays plant info                                 | - Python `print()` and logging file I/O  <br> - Extendable to GUI toolkits if needed                                                          |

---

### 🛠 Notes

- The package uses usb_cam.launch and subscribes to /usb_cam/image_raw. Ensure the camera is working with `roslaunch usb_cam usb_cam-test.launch`. More info at [wiki_ros](https://wiki.ros.org/usb_cam).
- Ensure your microphone is properly detected by ALSA.

---

### 👩‍🔬 Maintainer

**Vanessa Taing**  
Project for the Juno Robot | ROS1, Python3, OpenCV, PyTorch, ALSA, TTS  
GitHub: [Vanessa-Taing/HikeBot](https://github.com/Vanessa-Taing/HikeBot)
