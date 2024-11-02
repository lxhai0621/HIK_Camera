# -- coding: utf-8 --

import sys
import threading
import os
import termios
import cv2 as cv
from ctypes import *
import numpy as np
from ultralytics import YOLO

sys.path.append("./MvImport")
from MvCameraControl_class import *

g_bExit = False
frame_data = 0
# 为线程定义一个函数
def work_thread(cam=0, data_buf=0, nPayloadSize=0,yolo=0):
    stFrameInfo = MV_FRAME_OUT_INFO_EX()
    memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))

    # 创建OpenCV窗口
    cv.namedWindow("Camera Stream", cv.WINDOW_AUTOSIZE)
    while True:
        # 捕获帧
        ret = cam.MV_CC_GetOneFrameTimeout(byref(data_buf), nPayloadSize, stFrameInfo, 1000)
        if ret == 0:
            frame_data = np.frombuffer(data_buf, dtype=np.uint8)
            frame_data = frame_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, -1))
            frame_data = cv.cvtColor(frame_data, cv.COLOR_BAYER_BG2BGR)
            # 在OpenCV窗口中显示处理后的帧
            yolo.predict(source=frame_data, show=True)  # 对当前帧进行目标检测并显示结果
            #cv.imshow("Camera Stream", frame_data)

            # 按 'q' 键退出
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("no data[0x%x]" % ret)

        if g_bExit == True:
            break


def press_any_key_exit():
    fd = sys.stdin.fileno()
    old_ttyinfo = termios.tcgetattr(fd)
    new_ttyinfo = old_ttyinfo[:]
    new_ttyinfo[3] &= ~termios.ICANON
    new_ttyinfo[3] &= ~termios.ECHO
    # sys.stdout.write(msg)
    # sys.stdout.flush()
    termios.tcsetattr(fd, termios.TCSANOW, new_ttyinfo)
    os.read(fd, 7)
    termios.tcsetattr(fd, termios.TCSANOW, old_ttyinfo)


if __name__ == "__main__":
    SDKVersion = MvCamera.MV_CC_GetSDKVersion()
    print("SDKVersion[0x%x]" % SDKVersion)

    deviceList = MV_CC_DEVICE_INFO_LIST()
    tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE

    # ch:枚举设备 | en:Enum device
    ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
    if ret != 0:
        print("enum devices fail! ret[0x%x]" % ret)
        sys.exit()

    if deviceList.nDeviceNum == 0:
        print("find no device!")
        sys.exit()

    print("Find %d devices!" % deviceList.nDeviceNum)

    nConnectionNum = 0
    cam = MvCamera()

    # ch:选择设备并创建句柄| en:Select device and create handle
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents

    ret = cam.MV_CC_CreateHandle(stDeviceList)
    if ret != 0:
        print("create handle fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:打开设备 | en:Open device
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()

    ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
    if ret != 0:
        print("set trigger mode fail! ret[0x%x]" % ret)
        sys.exit()

    # ch:从文件中导入相机属性 | en:Import the camera properties from the file
    ret = cam.MV_CC_FeatureLoad("FeatureFile.ini")
    if MV_OK != ret:
        print("load feature fail! ret [0x%x]" % ret)
    print("finish import the camera properties from the file")
    # ch:获取数据包大小 | en:Get payload size
    stParam = MVCC_INTVALUE()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))

    ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
    if ret != 0:
        print("get payload size fail! ret[0x%x]" % ret)
        sys.exit()
    nPayloadSize = stParam.nCurValue

    # ch:开始取流 | en:Start grab image
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("start grabbing fail! ret[0x%x]" % ret)
        sys.exit()

    data_buf = (c_ubyte * nPayloadSize)()

    try:
        hThreadHandle = threading.Thread(target=work_thread, args=(cam, data_buf, nPayloadSize,yolo))
        hThreadHandle.start()
    except:
        print("error: unable to start thread")

    print("press a key to stop grabbing.")
    press_any_key_exit()

    g_bExit = True
    hThreadHandle.join()

    # ch:停止取流 | en:Stop grab image
    ret = cam.MV_CC_StopGrabbing()
    if ret != 0:
        print("stop grabbing fail! ret[0x%x]" % ret)
        del data_buf
        sys.exit()

    # ch:关闭设备 | Close device
    ret = cam.MV_CC_CloseDevice()
    if ret != 0:
        print("close deivce fail! ret[0x%x]" % ret)
        del data_buf
        sys.exit()

    # ch:销毁句柄 | Destroy handle
    ret = cam.MV_CC_DestroyHandle()
    if ret != 0:
        print("destroy handle fail! ret[0x%x]" % ret)
        del data_buf
        sys.exit()

    del data_buf
