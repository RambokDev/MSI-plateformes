#!/usr/bin/python3
import time

import cv2
from pypylon import pylon

from robot.ur.camera.camera_lights_state import camera_lights_state
# from camera_lights_state import camera_lights_state

ON, OFF = 1, 0


def camera_basler(type_camera):
    """
    This function is called for the camera Basler connexion
    :param  type_camera : 0 angle Camera , 1 pickup Camera
    """

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


def camera_trigger_basler(type_camera):


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

    camera.StartGrabbing()
    nb_images_prises = 0
    listImages = []
    camera_lights_state(type_camera, ON)

    while camera.IsGrabbing and nb_images_prises < 4:
        grabResult = camera.RetrieveResult(100000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            init_time = -time.time()
            nb_images_prises += 1
            image = grabResult.GetArray()
            img = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
            cv2.imwrite(
                "/home/icam/IdeaProjects/MSI-plateformes/robot/ur/camera/imagestrigger/saved_pypylon_img" + str(
                    time.time()) + ".png", img)

            listImages.append(image)
            print("temps de traitement par photo", init_time + time.time(), "s")
        else:
            print("Error: ", grabResult.GetErrorCode(), grabResult.GetErrorDescription())

    print(listImages)
    for i in range(len(listImages)):
        img = cv2.cvtColor(listImages[i], cv2.COLOR_YUV2BGR_YUYV)
        cv2.imwrite(
            "/home/icam/IdeaProjects/MSI-plateformes/robot/ur/camera/imagestrigger/saved_pypylon_img" + str(
                i) + ".png", img)

    camera.StopGrabbing()
    print("> Streaming : Arrêt Streaming Caméra")
    camera_lights_state(type_camera, OFF)

if __name__ == '__main__':
    camera_trigger_basler(1)
    # camera_basler(1)
