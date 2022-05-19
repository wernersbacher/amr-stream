#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Publish a video as ROS messages.

    Example: rosrun amr-stream publish.py 0 --hz 25 --width 200 --height 66
"""

import argparse

import numpy as np

import cv2

import rospy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def main():
    """Publish a video as ROS messages.
    """
    # Patse arguments.
    parser = argparse.ArgumentParser(description="Convert video into a rosbag.")
    parser.add_argument("video_file", help="Input video.")
    parser.add_argument("-z", "--hz", default="-1", help="camera fps, if not set it will be read from camera")
    parser.add_argument("-c", "--camera", default="camera", help="Camera name.")
    parser.add_argument("-f", "--frame_id", default="camera",
                        help="tf frame_id.")
    parser.add_argument("--width", type=np.int32, default="640",
                        help="Image width.")
    parser.add_argument("--height", type=np.int32, default="480",
                        help="Image height.")
    parser.add_argument("--info_url", default="file:///camera.yml",
                        help="Camera calibration url.")

    args = parser.parse_args()

    try:
        video_channel = int(args.video_file)
    except ValueError:
        video_channel = args.video_file

    print("Publishing %s." % (video_channel))

    # Set up node.
    rospy.init_node("video_publisher", anonymous=True)
    img_pub = rospy.Publisher("/" + args.camera + "/image_raw", Image,
                              queue_size=1)

    # Open video.
    video = cv2.VideoCapture(video_channel)

    fps_cam = video.get(cv2.CAP_PROP_FPS)

    # Get frame rate.
    try:
        fps_arg = int(args.hz)
        if fps_arg > 0:
            fps = fps_arg
        else:
            fps = fps_cam
    except ValueError:
        fps = fps_cam

    print(f"Publishing at {fps} FPS")
    rate = rospy.Rate(fps)

    # Loop through video frames.
    while not rospy.is_shutdown() and video.grab():
        tmp, img_color = video.retrieve()
        
        # converting to gray-scale
        img = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)


        if not tmp:
            print("Could not grab frame.")
            break

        #img_out = np.empty((args.height, args.width, img.shape[2]))  # for color
        img_out = np.empty((args.height, args.width))

        
        #print(img.shape, img_out.shape)

        # Compute input/output aspect ratios.
        aspect_ratio_in = np.float(img.shape[1]) / np.float(img.shape[0])
        aspect_ratio_out = np.float(args.width) / np.float(args.height)

        if aspect_ratio_in > aspect_ratio_out:
            # Output is narrower than input -> crop left/right.
            rsz_factor = np.float(args.height) / np.float(img.shape[0])
            img_rsz = cv2.resize(img, (0, 0), fx=rsz_factor, fy=rsz_factor,
                                 interpolation=cv2.INTER_AREA)

            diff = int((img_rsz.shape[1] - args.width) / 2)

            #img_out = img_rsz[:, diff:-diff-1, :]
            img_out = img_rsz[:, diff:-diff-1]
        elif aspect_ratio_in < aspect_ratio_out:
            # Output is wider than input -> crop top/bottom.
            rsz_factor = np.float(args.width) / np.float(img.shape[1])
            img_rsz = cv2.resize(img, (0, 0), fx=rsz_factor, fy=rsz_factor,
                                 interpolation=cv2.INTER_AREA)

            diff = int((img_rsz.shape[0] - args.height) / 2)

            #img_out = img_rsz[diff:-diff-1, :, :]
            img_out = img_rsz[diff:-diff-1, :]
        else:
            # Resize image.
            img_out = cv2.resize(img, (args.width, args.height))

        #assert img_out.shape[0:2] == (args.height, args.width)
        #print(img_out.shape[0:2], args.height, args.width)

        try:
            # Publish image.
            img_msg = bridge.cv2_to_imgmsg(img_out, "mono8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = args.frame_id
            img_pub.publish(img_msg)
        except CvBridgeError as err:
            print(err)

        rate.sleep()

    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass