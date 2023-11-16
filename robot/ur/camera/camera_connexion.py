#!/usr/bin/python3
import time
from pypylon import pylon
import rospy
from ur_msgs.srv import SetIO

PIN_CAM_DEVRACAGE = 5
PIN_CAM_ORIENTATION = 4
ON, OFF = 1, 0


def camera_basler(type_camera):
    """
    This function is called for the camera Basler connexion
    :param  type_camera : 0 angle Camera , 1 pickup Camera
    """
    set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    tlFactory = pylon.TlFactory.GetInstance()
    devices = tlFactory.EnumerateDevices()
    print(devices)
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[type_camera]))
    camera.Open()
    if type_camera == 0:
        camera.Width = 2590
        camera.Height = 1942
        light = PIN_CAM_ORIENTATION

    else:
        camera.Width = 3000
        camera.Height = 2000
        light = PIN_CAM_DEVRACAGE

    camera.CenterX.SetValue(True)
    camera.CenterY.SetValue(True)
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
    set_io_interface(1, light, ON)
    if type_camera == 0:
        time.sleep(2)
    if camera.IsGrabbing():
        grab = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grab.GrabSucceeded():
            img = pylon.PylonImage()
            img.AttachGrabResultBuffer(grab)
            filename = "robot/ur/ihm_tests/images/saved_pypylon_img_{}.png".format(type_camera)
            img.Save(pylon.ImageFileFormat_Png, filename)
            set_io_interface(1, light, OFF)
            return filename
        camera.Close()
