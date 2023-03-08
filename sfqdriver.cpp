#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <pthread.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <future>
#include <unistd.h>
#include <sstream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "GxIAPI.h"
#include "DxImageProc.h"
#include "sfqdriver.h"

using namespace std;
using namespace cv;

ZZUcameraDriver::ZZUcameraDriver() {

    FileStorage fs("configureFiles.xml", FileStorage::READ);

    dValue.NewName = "ZZURM01";
    dValue.EifAuto = false;
    dValue.dFrameRateValue = 500.0;
    dValue.GifAuto = false;
    dValue.BifAuto  = false;

    fs["dExposureValue"] >> dValue.dExposureValue;
    fs["GainMode"] >> dValue.GainMode;
    fs["dGainValue"] >> dValue.dGainValue;
    fs["dBalanceRatioValueR"] >> dValue.dBalanceRatioValueR;
    fs["dBalanceRatioValueG"] >> dValue.dBalanceRatioValueG;
    fs["dBalanceRatioValueB"] >> dValue.dBalanceRatioValueB;

    //org
    /*dValue.GainMode = GX_GAIN_SELECTOR_ALL;
    dValue.GifAuto = false;
    dValue.dGainValue = 5;

    dValue.BalanceChann = GX_BALANCE_RATIO_SELECTOR_RED;
    dValue.BifAuto  = true;
    dValue.dBalanceRatioValue = 1;*/

    //blue armor
    /*dValue.GainMode = GX_GAIN_SELECTOR_ALL;
    dValue.GifAuto = false;
    dValue.dGainValue = 0;

    dValue.BalanceChann = GX_BALANCE_RATIO_SELECTOR_RED;
    dValue.BifAuto  = false;
    dValue.dBalanceRatioValue = 4;*/

    //red armor
    /*dValue.GainMode = GX_GAIN_SELECTOR_ALL;
    dValue.GifAuto = false;
    dValue.dGainValue = 1;

    dValue.BalanceChann = GX_BALANCE_RATIO_SELECTOR_BLUE;
    dValue.BifAuto  = false;
    dValue.dBalanceRatioValue = 4;*/

    dValue.dWidth = 640;
    dValue.dHeight = 480;

    m_rgb_image = new char[dValue.dWidth * dValue.dHeight * 3];

    status = GX_STATUS_SUCCESS;
    uint32_t nDeviceNum = 0;
    //初始化库
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) {
        cout << "status != GX_STATUS_SUCCESS" << endl;
        flag = false;
        return;
    }
    //枚举设备列表
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if (status == GX_STATUS_SUCCESS && nDeviceNum> 0) {
        GX_DEVICE_BASE_INFO *pBaseinfo = new GX_DEVICE_BASE_INFO[nDeviceNum];
        uint64_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
        //获取所有设备的基础信息
        status = GXGetAllDeviceBaseInfo(pBaseinfo, &nSize);
        showInfo(pBaseinfo); //输出设备信息
        delete []pBaseinfo;
    }

    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0)) {
        cout << "(status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0)" << endl;
        flag = false;
        return;
    }


    //打开设备
    status = GXOpenDeviceByIndex(1, &hDevice);
    if (!(status == GX_STATUS_SUCCESS)) {
        cout << "GXOpenDeviceByIndex Error!" << endl;
        flag = false;
        return;
    }

    if(SetCamName(dValue.NewName)) {
        cout << "SetCamName Error!" << endl;
        flag = false;
        return;
    }

    if(SetExposure(dValue.EifAuto, dValue.dExposureValue)) {
        cout << "SetExposure Error!" << endl;
        flag = false;
        return;
    }

    if(SetFrameRate(dValue.dFrameRateValue)) {
        cout << "SetFrameRate Error!" << endl;
        flag = false;
        return;
    }

    if(SetGain(dValue.GainMode, dValue.GifAuto, dValue.dGainValue)) {
        cout << "SetGain Error!" << endl;
        flag = false;
        return;
    }

    if(SetBalanceRatio(dValue.BifAuto, dValue.dBalanceRatioValueR, dValue.dBalanceRatioValueG, dValue.dBalanceRatioValueB)) {
        cout << "SetBalanceRatio Error!" << endl;
        flag = false;
        return;
    }

    if(SetResolution(dValue.dWidth, dValue.dHeight)) {
        cout << "SetResolution Error!" << endl;
        flag = false;
        return;
    }

    //开采
    status = GXStreamOn(hDevice);

    //GXExportConfigFile(hDevice, "Config.txt");

}

ZZUcameraDriver::~ZZUcameraDriver() {

    status = GXStreamOff(hDevice);
    status = GXCloseDevice(hDevice);
    status = GXCloseLib();
    delete[] m_rgb_image;
}

bool ZZUcameraDriver::Myread(Mat & frame) {

    bool Rflag = false;
    //定义GXDQBuf的传入参数
    PGX_FRAME_BUFFER pFrameBuffer[10];
    uint32_t nFrameCount = 0;

    if (status == GX_STATUS_SUCCESS) {
        //调用GXDQBuf/GXDQAllBufs取一/多帧图像
        status = GXDQAllBufs(hDevice, pFrameBuffer, 10, &nFrameCount, 1000);
        if (status == GX_STATUS_SUCCESS) {
            for (register uint32_t i = 0; i < nFrameCount; i++) {
                if ((pFrameBuffer[i] != NULL) &&
                        (pFrameBuffer[i]->nStatus == GX_FRAME_STATUS_SUCCESS)) {
                    //第i幅图像获取成功
                    Rflag = true;

                    frame.create(pFrameBuffer[i]->nHeight, pFrameBuffer[i]->nWidth, CV_8UC3);

                    memcpy(frame.data, pFrameBuffer[i]->pImgBuf, pFrameBuffer[i]->nWidth * pFrameBuffer[i]->nHeight);

                    DxRaw8toRGB24(pFrameBuffer[i]->pImgBuf, m_rgb_image, pFrameBuffer[i]->nWidth, pFrameBuffer[i]->nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);

                    memcpy(frame.data, m_rgb_image, pFrameBuffer[i]->nWidth * pFrameBuffer[i]->nHeight * 3);

                    //cout << (clock() - t) * 1000 / CLOCKS_PER_SEC << "ms" << endl;


                    if(frame.empty()) Rflag = false;
                }
            }
            //调用GXQBuf/GXQAllBufs将图像buf放回库中继续采图
            status = GXQAllBufs(hDevice);

        }

    }

    return Rflag;
}

//相机参数调节

//**设置用户自定义名称**
bool ZZUcameraDriver::SetCamName(const string NewName) {

    status = GXSetString(hDevice, GX_STRING_DEVICE_USERID, (char*)(NewName.c_str()));
    return status;
}

//**曝光延迟**
bool ZZUcameraDriver::SetExposure(const bool ifAuto, const float dExposureValue) {

    //获取曝光调节范围
    GX_FLOAT_RANGE shutterRange;
    status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);

    if(ifAuto) {
        //设置连续自动曝光
        status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    } else {
        if((dExposureValue < shutterRange.dMin) || (dExposureValue > shutterRange.dMax)) {
            cout << "dExposureValue Out of Range!" << endl;
            return true;
        }
        //设置曝光延迟为dExposureValue
        status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
        status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, dExposureValue);
    }
    return status;
}

//**帧率**
bool ZZUcameraDriver::SetFrameRate(const float dFrameRateValue) {

    //使能采集帧率调节模式
    status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
    //设置采集帧率
    status = GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, dFrameRateValue);
    return status;
}

//**分辨率**
bool ZZUcameraDriver::SetResolution(const int dWidth, const int dHeight) {
    //设置分辨率
    //GX_INT_WIDTH = 640;
    //GX_INT_HEIGHT = 480;
    status = GXSetInt(hDevice, GX_INT_WIDTH, dWidth);
    status = GXSetInt(hDevice, GX_INT_HEIGHT, dHeight);
    return status;
}

//**增益**
bool ZZUcameraDriver::SetGain(const int GainMode, const bool ifAuto, const float dGainValue) {

    //选择增益通道类型
    status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GainMode);
    //获取增益调节范围
    GX_FLOAT_RANGE gainRange;
    status = GXGetFloatRange(hDevice, GX_FLOAT_GAIN, &gainRange);


    if(ifAuto) {
        //设置连续自动增益
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    } else {
        if((dGainValue < gainRange.dMin) || (dGainValue > gainRange.dMax)) {
            cout << "dGainValue Out of Range!" << endl;
            return true;
        }
        //设置增益值
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
        status = GXSetFloat(hDevice, GX_FLOAT_GAIN, dGainValue);
    }
    return status;
}

//**白平衡**
bool ZZUcameraDriver::SetBalanceRatio(const bool ifAuto,
                                      const float dBalanceRatioValueR,
                                      const float dBalanceRatioValueG,
                                      const float dBalanceRatioValueB) {


    //获取白平衡调节范围
    GX_FLOAT_RANGE ratioRange;
    status = GXGetFloatRange(hDevice, GX_FLOAT_BALANCE_RATIO, &ratioRange);


    if(ifAuto) {
        //自动白平衡设置
        //设置自动白平衡光照环境,比如当前相机所处环境为荧光灯
        status = GXSetEnum(hDevice, GX_ENUM_AWB_LAMP_HOUSE,
                           GX_AWB_LAMP_HOUSE_FLUORESCENCE);
        //设置连续自动白平衡
        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO,
                           GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    } else {
        if((dBalanceRatioValueR < ratioRange.dMin) || (dBalanceRatioValueR > ratioRange.dMax)) {
            cout << "dBalanceRatioValueR Out of Range!" << endl;
            return true;
        }
        if((dBalanceRatioValueG < ratioRange.dMin) || (dBalanceRatioValueG > ratioRange.dMax)) {
            cout << "dBalanceRatioValueG Out of Range!" << endl;
            return true;
        }
        if((dBalanceRatioValueB < ratioRange.dMin) || (dBalanceRatioValueB > ratioRange.dMax)) {
            cout << "dBalanceRatioValueB Out of Range!" << endl;
            return true;
        }
        //设置白平衡
        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO,
                           GX_BALANCE_WHITE_AUTO_OFF);
        //选择白平衡通道
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,
                           GX_BALANCE_RATIO_SELECTOR_RED);
        status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, dBalanceRatioValueR);
        //选择白平衡通道
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,
                           GX_BALANCE_RATIO_SELECTOR_GREEN);
        status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, dBalanceRatioValueG);
        //选择白平衡通道
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,
                           GX_BALANCE_RATIO_SELECTOR_BLUE);
        status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, dBalanceRatioValueB);
    }
    return status;
}

void ZZUcameraDriver::showInfo(const GX_DEVICE_BASE_INFO *pBaseinfo) {

    cout << "---------------------设备基础信息---------------------" << endl;
    cout << "厂商名称------------------" << pBaseinfo->szVendorName << endl;
    cout << "设备类型名称--------------" << pBaseinfo->szModelName << endl;
    cout << "设备序列号----------------" << pBaseinfo->szSN << endl;
    cout << "设备展示名称--------------" << pBaseinfo->szDisplayName << endl;
    cout << "用户自定义名称------------" << pBaseinfo->szUserID << endl;
    cout << "设备唯一标识--------------" << pBaseinfo->szDeviceID << endl;
    cout << "设备当前支持的访问状态----";
    switch(pBaseinfo->accessStatus) {
    case 0:
        cout << "状态未知" << endl;
        break;
    case 1:
        cout << "可读可写" << endl;
        break;
    case 2:
        cout << "仅支持读" << endl;
        break;
    case 3:
        cout << "既不支持读,又不支持写" << endl;
        break;
    }
    cout << "设备种类------------------";
    switch(pBaseinfo->deviceClass) {
    case 0:
        cout << "未知设备种类" << endl;
        break;
    case 1:
        cout << "USB2.0 设备" << endl;
        break;
    case 2:
        cout << "千兆网设备(Gige Vision)" << endl;
        break;
    case 3:
        cout << "USB3.0 设备(USB3 Vision)" << endl;
        break;
    }
    cout << "------------------------------------------------------" << endl << endl;
}
