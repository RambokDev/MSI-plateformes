#!/usr/bin/python3
import time
from pypylon import pylon


from robot.ur.camera.camera_lights_state import camera_lights_state


def camera_basler(type_camera):
    """
    This function is called for the camera Basler connexion
    :param  type_camera : 0 angle Camera , 1 pickup Camera
    """
    ON, OFF = 1, 0
    tlFactory = pylon.TlFactory.GetInstance()
    devices = tlFactory.EnumerateDevices()
    print(devices)
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[type_camera]))
    camera.Open()
    if type_camera == 0:
        camera.Width = 2590
        camera.Height = 1942
    else:
        camera.Width = 3000
        camera.Height = 2000

    camera.CenterX.SetValue(True)
    camera.CenterY.SetValue(True)
    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
    camera_lights_state(type_camera, ON)
    if type_camera == 0:
        time.sleep(2)
    if camera.IsGrabbing():
        grab = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grab.GrabSucceeded():
            img = pylon.PylonImage()
            img.AttachGrabResultBuffer(grab)
            filename = "robot/ur/ihm_tests/images/saved_pypylon_img_{}.png".format(type_camera)
            img.Save(pylon.ImageFileFormat_Png, filename)
            camera_lights_state(type_camera, OFF)

            return filename
        camera.Close()
