#!/usr/bin/env python3

import rospy
from tensorflow.keras.applications import VGG16
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Flatten, Dropout, BatchNormalization
import tensorflow as tf
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import joblib
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os

class PlantIdentifierNode:
    def __init__(self):
        rospy.init_node('plant_identifier_node')

        # Params
        self.IMG_SIZE = 128
        # self.NUM_CLASSES = 47
        self.bridge = CvBridge()

        # Load model
        # model_path = rospy.get_param('~model_path', '/home/mustar/catkin_ws/src/HikeBot/hikebot_identifier/model/best_model.keras')
        encoder_path = rospy.get_param('~encoder_path', '/home/mustar/catkin_ws/src/HikeBot/hikebot_identifier/model/label_encoder.pkl')
        tflite_model_path = rospy.get_param('~model_path','/home/mustar/catkin_ws/src/HikeBot/hikebot_identifier/model/model_tf213_compatible.tflite')
        self.interpreter = tf.lite.Interpreter(model_path=tflite_model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        # self.model = self._create_and_load_model(model_path)
        self.label_encoder = joblib.load(encoder_path)

        # Subscriber to trigger
        self.trigger_sub = rospy.Subscriber('/voice_commands/trigger_camera', Bool, self.voice_trigger_callback)
        self.pred_pub = rospy.Publisher('/plant/name', String, queue_size=10)

        rospy.loginfo("✅ Plant Identifier Node is ready and waiting for voice trigger.")

    # def _create_model_architecture(self):
    #     base_model = VGG16(weights='imagenet', include_top=False, input_shape=(self.IMG_SIZE, self.IMG_SIZE, 3))
    #     base_model.trainable = True
    #     for layer in base_model.layers[:-4]:
    #         layer.trainable = False

    #     model = Sequential([
    #         base_model,
    #         Flatten(),
    #         Dense(512, activation='relu'),
    #         BatchNormalization(),
    #         Dropout(0.5),
    #         Dense(256, activation='relu'),
    #         BatchNormalization(),
    #         Dropout(0.5),
    #         Dense(128, activation='relu'),
    #         BatchNormalization(),
    #         Dropout(0.5),
    #         Dense(self.NUM_CLASSES, activation='softmax')
    #     ])
    #     return model

    # def _create_and_load_model(self, model_path):
    #     try:
    #         rospy.loginfo("Loading full model...")
    #         model = tf.keras.models.load_model(model_path, compile=False)
    #         return model
    #     except Exception as e:
    #         rospy.logwarn(f"Failed to load full model: {e}")
    #         rospy.loginfo("Trying custom architecture and weights...")
    #         model = self._create_model_architecture()
    #         weights_path = "/home/mustar/catkin_ws/src/HikeBot/hikebot_identifier/model/best_model/model.weights.h5"
    #         if os.path.exists(weights_path):
    #             model.load_weights(weights_path, by_name=True, skip_mismatch=True)
    #             return model
    #         else:
    #             raise FileNotFoundError(f"Weight file not found at {weights_path}")

    def _preprocess_image(self, image):
        image = cv2.resize(image, (self.IMG_SIZE, self.IMG_SIZE))
        image = image.astype(np.float32) / 255.0
        return np.expand_dims(image, axis=0)

    def predict_plant(self, image):
        processed_image = self._preprocess_image(image)
        # preds = self.model.predict(processed_image)
        # return preds
        try:
            # Set input tensor
            self.interpreter.set_tensor(self.input_details[0]['index'], processed_image)          
            # Run inference
            self.interpreter.invoke()
            
            # Get output
            output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
            
            return output_data
            
        except Exception as e:
            print(f"Error during prediction: {e}")
            return None

    def voice_trigger_callback(self, msg):
        rospy.loginfo("🎤 Voice trigger received. Capturing image...")

        try:
            # Get latest image after trigger
            img_msg = rospy.wait_for_message('/usb_cam/image_raw', Image, timeout=3.0)
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

            # Predict
            preds = self.predict_plant(cv_image)
            predicted_class = np.argmax(preds[0])
            confidence = preds[0][predicted_class]

            plant_name = self.label_encoder.inverse_transform([predicted_class])[0]
            result = f"{plant_name} ({confidence:.2f})"

            rospy.loginfo(f"🌿 Prediction: {result}")
            self.pred_pub.publish(result)

        except rospy.ROSException:
            rospy.logerr("⏳ Timeout waiting for camera image.")
        except CvBridgeError as e:
            rospy.logerr(f"❌ CvBridge error: {e}")
        except Exception as e:
            rospy.logerr(f"❌ Prediction error: {e}")

if __name__ == '__main__':
    try:
        PlantIdentifierNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
