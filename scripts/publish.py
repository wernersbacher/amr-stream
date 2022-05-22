#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from types import SimpleNamespace  
import numpy as np

import cv2

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
        
a = {
    "colors": True,
    "video_stream_provider": 0,
    "fps": 25,
    "width": 320,
    "height": 240,
    "camera_name": "camera",
    "frame_id": "camera",
    "buffer_queue_size": 10
    
}
args = SimpleNamespace(**a)


def main():
    """Publish a video as ROS messages.
    """
    # Set up node.
    rospy.init_node("video_publisher", anonymous=True)

    # load from launchfile
    args.colors = rospy.get_param('~colors')
    args.width = rospy.get_param('~width')
    args.height = rospy.get_param('~height')
    args.fps = rospy.get_param('~fps')
    args.buffer_queue_size = rospy.get_param('~buffer_queue_size')
    args.camera_name = rospy.get_param('~camera_name')
    args.frame_id = rospy.get_param('~frame_id', "camera")

    # convert to int?
    try:
        video_channel = int(args.video_stream_provider)
    except ValueError:
        video_channel = args.video_stream_provider

    rospy.loginfo ("Loaded arguments:")
    rospy.loginfo(args)
    
    
    img_pub = rospy.Publisher("/" + args.camera_name + "/image_raw", Image,
                              queue_size=1)

    # Open video.
    video = cv2.VideoCapture(video_channel)
    rospy.loginfo("Publishing %s." % (video_channel))

    fps_cam = video.get(cv2.CAP_PROP_FPS)

    # Get frame rate.
    try:
        fps_arg = int(args.fps)
        if 0 < fps_arg < fps_cam:
            fps = fps_arg
        else:
            fps = fps_cam
    except ValueError:
        fps = fps_cam

    rospy.loginfo(f"Publishing at {fps} FPS")
    rate = rospy.Rate(fps)

    # Loop through video frames.
    while not rospy.is_shutdown() and video.grab():
        tmp, img_color = video.retrieve()
        
        # converting to gray-scale
        if not args.colors:
            img = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)
        else:
            img = img_color

        if not tmp:
            rospy.logerror("Could not grab frame.")
            break

        #img_out = np.empty((args.height, args.width, img.shape[2]))  # for color
        img_out = np.empty((args.height, args.width))

        
        #rospy.loginfo(img.shape, img_out.shape)

        # Compute input/output aspect ratios.
        aspect_ratio_in = np.float(img.shape[1]) / np.float(img.shape[0])
        aspect_ratio_out = np.float(args.width) / np.float(args.height)

        if aspect_ratio_in > aspect_ratio_out:
            # Output is narrower than input -> crop left/right.
            rsz_factor = np.float(args.height) / np.float(img.shape[0])
            img_rsz = cv2.resize(img, (0, 0), fx=rsz_factor, fy=rsz_factor,
                                 interpolation=cv2.INTER_AREA)

            diff = int((img_rsz.shape[1] - args.width) / 2)

            if args.colors:
                img_out = img_rsz[:, diff:-diff-1, :] 
            else:
                img_out = img_rsz[:, diff:-diff-1]

        elif aspect_ratio_in < aspect_ratio_out:
            # Output is wider than input -> crop top/bottom.
            rsz_factor = np.float(args.width) / np.float(img.shape[1])
            img_rsz = cv2.resize(img, (0, 0), fx=rsz_factor, fy=rsz_factor,
                                 interpolation=cv2.INTER_AREA)

            diff = int((img_rsz.shape[0] - args.height) / 2)

            if args.colors:
                img_out = img_rsz[diff:-diff-1, :, :]
            else:
                img_out = img_rsz[diff:-diff-1, :]
        else:
            # Resize image.
            img_out = cv2.resize(img, (args.width, args.height))

        #assert img_out.shape[0:2] == (args.height, args.width)
        #rospy.loginfo(img_out.shape[0:2], args.height, args.width)

        try:
            # Publish image.
            if args.colors:
                img_msg = bridge.cv2_to_imgmsg(img_out, "bgr8")
            else:
                img_msg = bridge.cv2_to_imgmsg(img_out, "mono8")
                
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = args.frame_id
            img_pub.publish(img_msg)
        except CvBridgeError as err:
            rospy.logerror(err)

        rate.sleep()

    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass