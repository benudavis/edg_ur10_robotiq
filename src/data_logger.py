#!/usr/bin/env python

# This data logger includes image data from the Digit sensor
import sys
import os
import rospy
import numpy as np
import rosgraph
import time
from roslib.message import get_message_class
from std_msgs.msg import *
from suction_cup.srv import *
from rospy.msg import AnyMsg
import numbers
import collections
from operator import attrgetter
from datetime import datetime
from edg_ur10_robotiq.srv import GetImage, GetImageResponse
from std_srvs.srv import SetBool, SetBoolResponse
import config

# Current state of logging
isLoggingEnabled = False
logJustStarted = 0

# List of topics to record
listOfTopics = []

########################
# For each topic, which attributes do we log?
#   e.g.  "topic_name" : { "attr1":1, "attr2":5, "attr3":1, ...}
listOfAttributesInTopic = {}

# Data record before it gets dumped in the CSV file
record = {}

# For unsubscribing easily
listOfSubscribers = []

# All available topics (from master) and their corresponding types
topic_types = []

# Output CSV file references
output_file_name = {}
output_file = {}
all_output_file_names = ""

# For image data
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
helper_path = os.path.join(current_dir, "helperFunction")
sys.path.append(helper_path)

from fileSaveHelper import fileSaveHelp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()
file_helper = fileSaveHelp()

# Buffer to store image and timestamp
digit_image_buffer = []  # each entry: (rospy.Time, np.ndarray)
digit_image_topic = "/digitFrame"  # update if your topic name is different

save_digit_frames = False # Set to True to save frames

last_digit_image = None

def digit_image_callback(msg):
    global save_digit_frames
    if not save_digit_frames:
        return
    
    try:
        ros_time = msg.header.stamp
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        digit_image_buffer.append((ros_time.to_sec(), img))
    except Exception as e:
        rospy.logerr(f"[X] Failed to convert image in digit_image_callback: {e}")
    # Numeric topic logging (as-is)

def toggle_digit_frame_service(req):
    print('debug')
    global save_digit_frames
    save_digit_frames = req.data
    rospy.loginfo("[✓] Request received to log Digit frames.")
    return SetBoolResponse(success=True, message="Digit frame logging has been toggled.")

def capture_digit_image_service(req):
    global last_digit_image
    # this service captures a "single frame" average from DIGIT sensor
    try:
        rospy.loginfo("trying to capture images")
        imgs = []
        times = []
        for i in range(config.DIGIT_AVG_FRAMES):
            msg = rospy.wait_for_message("/digitFrame", Image, timeout=2.0)
            ros_time = msg.header.stamp
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            imgs.append(img)
            times.append(ros_time.to_sec())
        # Average the images
        if imgs:
            avg_img = np.mean(np.array(imgs), axis=0).astype(np.uint8)
            avg_time = np.mean(times)
            last_digit_image = avg_img
        else:
            rospy.logwarn("[X] No images captured from DIGIT sensor.")
            return SetBoolResponse(success=False, message="No images captured.")

        digit_image_buffer.append((avg_time, avg_img))
        rospy.loginfo("[✓] Digit image captured successfully.")
        return SetBoolResponse(success=True, message="Image captured successfully.")
    except rospy.ROSException:
        return SetBoolResponse(success=False, message="Timed out waiting forimage from DIGIT sensor.")
    except Exception as e:
        return SetBoolResponse(success=False, message=f"Unexpected error: {e}")
    
def get_last_digit_image_service(req):
    global last_captured_image
    if last_digit_image is not None:
        try:
            img_msg = bridge.cv2_to_imgmsg(last_digit_image, "rgb8")
            rospy.loginfo("[✓] Last captured image provided via service.")
            return GetImageResponse(captured_image=img_msg)
        except Exception as e:
            rospy.logerr(f"[X] Failed to convert image for service response: {e}")
            return GetImageResponse() # Return an empty response on failure
    else:
        rospy.logwarn("last captured image unavailable")
        return GetImageResponse()



def appendDataPoint(topic, msg):
    """
    Append one line of CSV data for `topic`, reading the numeric fields in `msg`.
    """
    global listOfAttributesInTopic, output_file

    # Build a dictionary: { attributeName -> value }
    thisDataDict = {}
    for attributeName in list(listOfAttributesInTopic[topic].keys()):
        retriever = attrgetter(attributeName)  # e.g. 'x' or 'data/2'
        value = retriever(msg)
        thisDataDict[attributeName] = value

    # Default timestamp = now
    thisTimeStamp = rospy.get_rostime()

    # If the message has a header.stamp, prefer that
    if hasattr(msg, "header"):
        try:
            stamp = attrgetter("header.stamp")(msg)
            if stamp.secs > 0:
                thisTimeStamp = stamp
        except Exception:
            pass

    # Start building the CSV line: first column is the ROS time
    # Note: str(thisTimeStamp) typically prints the full format, e.g. "1736204679.474882..."
    line = "{},".format(thisTimeStamp)

    # Next columns: each attribute's value
    for attributeName in list(listOfAttributesInTopic[topic].keys()):
        val = thisDataDict[attributeName]
        # Convert to string
        val_str = str(val)

        # If the attribute is a list/tuple, remove brackets
        # We know it's a list if our dictionary says so
        if listOfAttributesInTopic[topic][attributeName] > 1:
            val_str = val_str[1:-1]  # remove [ and ] from e.g. "[0.1, 0.2]"

        line += val_str + ","

    # Remove trailing comma, add newline
    line = line[:-1] + "\n"

    # Write it out
    output_file[topic].write(line)


def updateListofAttribute(topic, msg, attributeName=""):
    """
    Recursively discover numeric fields in `msg` and store them in
    listOfAttributesInTopic[topic].
    """
    global listOfAttributesInTopic

    # If msg is a ROS message with __slots__
    if hasattr(msg, "__slots__"):
        for slotName in msg.__slots__:
            if slotName == "header":
                continue
            slotValue = getattr(msg, slotName)
            updateListofAttribute(topic, slotValue, attributeName + "." + slotName)

    # If msg is a single numeric (int/float)
    elif isinstance(msg, numbers.Number):
        if topic not in listOfAttributesInTopic:
            listOfAttributesInTopic[topic] = {}
        # attributeName starts with "." so skip that
        cleanedName = attributeName[1:]
        listOfAttributesInTopic[topic][cleanedName] = 1  # single numeric value
        return

    # If msg is a list/tuple of numeric
    elif (isinstance(msg, (list, tuple))
          and msg
          and isinstance(msg[0], numbers.Number)):
        if topic not in listOfAttributesInTopic:
            listOfAttributesInTopic[topic] = {}
        cleanedName = attributeName[1:]
        listOfAttributesInTopic[topic][cleanedName] = len(msg)  # length of numeric array
        return

    # Otherwise, skip non-numeric or empty
    return


def writeFileHeader(topic):
    """
    Write one CSV header line for `topic`, enumerating the discovered attributes.
    """
    global listOfAttributesInTopic, output_file

    header = "ROStimestamp,"
    for attributeName in list(listOfAttributesInTopic[topic].keys()):
        count = listOfAttributesInTopic[topic][attributeName]
        if count > 1:
            # e.g., if we have array of length 3, produce: topic.attr[0], topic.attr[1], ...
            for i in range(count):
                header += "{}.{}[{}],".format(topic, attributeName, i)
        else:
            header += "{}.{},".format(topic, attributeName)

    # Remove trailing comma, add newline
    header = header[:-1] + "\n"
    output_file[topic].write(header)


def callback(data, args):
    """
    Subscriber callback for AnyMsg. We look up the actual message type,
    deserialize it, and then log it to CSV.
    """
    # First, if logging is not enabled, do nothing.
    if not isLoggingEnabled:
        return

    topic = args[0]
    msg_name = args[1]

    # Safety check: if the file got closed but we still get a message, skip
    if topic not in output_file or output_file[topic].closed:
        rospy.logwarn("File for topic '%s' is already closed or missing. Skipping data.", topic)
        return

    # E.g. "suction_cup/SensorPacket"
    msg_class = get_message_class(msg_name)
    if not msg_class:
        rospy.logerr("Failed get_message_class(%s). Are you sure it's built?", msg_name)
        return

    # Deserialize the AnyMsg
    msg = msg_class().deserialize(data._buff)

    # If first time seeing this topic, discover numeric fields & write CSV headers
    if topic not in listOfAttributesInTopic:
        updateListofAttribute(topic, msg)
        # Sort the attributes (optional, for consistent order)
        listOfAttributesInTopic[topic] = collections.OrderedDict(
            sorted(listOfAttributesInTopic[topic].items())
        )
        writeFileHeader(topic)

    # Actually write out one CSV line
    appendDataPoint(topic, msg)


def unsubscribeAllTopics():
    """
    Unsubscribe from all topics and clear data structures.
    """
    global listOfTopics, listOfSubscribers, listOfAttributesInTopic

    for sub in listOfSubscribers:
        sub.unregister()

    listOfSubscribers.clear()
    listOfTopics.clear()
    listOfAttributesInTopic.clear()


def loadConfigFile(filePath):
    """
    Read a list of topics from filePath. For each topic, find a matching type in topic_types
    and subscribe with AnyMsg + callback.
    """
    global listOfTopics, listOfSubscribers, topic_types

    # Clean up old subscriptions
    unsubscribeAllTopics()

    rospy.loginfo("Master reports these topics:")
    for (tp, ty) in topic_types:
        rospy.loginfo("  %s -> %s", tp, ty)

    # Read each line from config
    with open(filePath) as fp:
        for line in fp:
            topic_str = line.strip()  # remove newline/spaces
            if not topic_str:
                continue

            # If your published topics always begin with '/', you can force it:
            # if not topic_str.startswith("/"):
            #     topic_str = "/" + topic_str

            # Now see if we can find a type match
            msg_name = ""
            for (tp, ty) in topic_types:
                if tp == topic_str:
                    msg_name = ty
                    break

            if not msg_name:
                rospy.logwarn("No matching type found for topic '%s'. Check if it's published or spelled correctly.", 
                              topic_str)
                continue

            # Subscribe
            if msg_name == "sensor_msgs/Image":
                sub = rospy.Subscriber(topic_str, Image, digit_image_callback)
                rospy.loginfo(f"Subscribed to image topic '{topic_str}' with Image callback")
            else:
                sub = rospy.Subscriber(topic_str, AnyMsg, callback, (topic_str, msg_name))
                rospy.loginfo(f"Subscribed to '{topic_str}' with type '{msg_name}'")

            listOfTopics.append(topic_str)
            listOfSubscribers.append(sub)
            rospy.loginfo("Subscribed to '%s' with type '%s'", topic_str, msg_name)


def setLoggingState(request):
    """
    Service callback: start or stop logging to CSV files based on request.
    """
    global isLoggingEnabled
    global output_file
    global output_file_name
    global all_output_file_names
    global listOfTopics
    global record
    global logJustStarted

    desiredState = request.EnableDataLogging  # True/False

    # Enable logging
    if desiredState:
        if isLoggingEnabled != desiredState:
            # The config file must list topics we want to log
            path = os.path.dirname(__file__)
            parent = os.path.dirname(path)
            config_path = os.path.join(parent, "config", "TopicsList.txt")

            loadConfigFile(config_path)
            rospy.loginfo("Listening for these topics: %s", str(listOfTopics))

            currentTimeStr = datetime.now().strftime('%Y_%m%d_%H%M%S')
            all_output_file_names = ""

            for topicName in listOfTopics:
                # Replace '/' with '_' in filenames
                safe_topic_name = topicName.replace("/", "_")
                output_file_name[topicName] = '/tmp/dataLog_{}_{}.csv'.format(
                    currentTimeStr,
                    safe_topic_name
                )
                # Open for writing
                output_file[topicName] = open(output_file_name[topicName], 'w')
                rospy.loginfo("Writing output for %s to: %s", topicName, output_file_name[topicName])
                all_output_file_names += output_file_name[topicName] + " "

            logJustStarted = 1
            rospy.loginfo("Data logging is started.")
        else:
            rospy.loginfo("Data logging was already enabled.")
        isLoggingEnabled = True
        return EnableResponse(all_output_file_names)

    # Disable logging
    else:
        if isLoggingEnabled != desiredState:
            # 1) Unsubscribe so we no longer receive new callbacks
            unsubscribeAllTopics()

            # 2) Close all open files
            for topicName, fobj in output_file.items():
                if not fobj.closed:
                    fobj.close()

            all_output_file_names = ""
            record = {}
            logJustStarted = 0
            rospy.loginfo("Data logging is stopped.")
        else:
            rospy.loginfo("Data logging was already disabled.")

        isLoggingEnabled = False

        # Prepare dict for .mat saving
        digit_image_dict = {
            'timestamps': [t for t, _ in digit_image_buffer],
            'images': [img for _, img in digit_image_buffer]
        }

        # Save
        file_helper.saveDataParams(appendTxt='digit_data_log', image_frames=digit_image_dict)
        digit_image_buffer.clear()

        return EnableResponse(all_output_file_names)



if __name__ == '__main__':
    print("DataLogger Started")

    rospy.init_node("edg_log_node", anonymous=True)

    # Retrieve the graph of nodes
    master = rosgraph.Master(rospy.get_name())
    # Get a list of all topics and their associated message type
    topic_types = master.getTopicTypes()

    # Advertise the data_logging service
    service = rospy.Service('data_logging', Enable, setLoggingState)
    digit_service = rospy.Service('capture_digit_frame', SetBool, capture_digit_image_service)
    digit_toggle_service = rospy.Service('toggle_digit_frame', SetBool, toggle_digit_frame_service)
    last_image_service = rospy.Service('get_last_image', GetImage, get_last_digit_image_service)

    rospy.spin()
