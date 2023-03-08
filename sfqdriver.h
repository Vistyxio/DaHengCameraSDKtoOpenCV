#pragma once
#include <iostream>
#include <cstdio>
#include <cstdlib>
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

using namespace std;
using namespace cv;

class ZZUcameraDriver{
public:
    bool flag = true;

    bool Myread(Mat & frame);
    ZZUcameraDriver();
    ~ZZUcameraDriver();

private:
    GX_DEV_HANDLE hDevice;
    GX_STATUS status;

    char* m_rgb_image = nullptr;

    struct Value{
        string NewName;
        bool EifAuto;
        float dExposureValue; //曝光延迟 单位us微秒
        float dFrameRateValue;
        int GainMode;
        bool GifAuto;
        float dGainValue;
        bool BifAuto;
        float dBalanceRatioValueR;
        float dBalanceRatioValueG;
        float dBalanceRatioValueB;
        int dWidth;
        int dHeight;
    } dValue;

    bool SetCamName(const string NewName);
    bool SetExposure(const bool ifAuto, const float dExposureValue);
    bool SetFrameRate(const float dFrameRateValue);
    bool SetGain(const int GainMode, const bool ifAuto, const float dGainValue);
    bool SetBalanceRatio(const bool ifAuto,
                         const float dBalanceRatioValueR,
                         const float dBalanceRatioValueG,
                         const float dBalanceRatioValueB);
    bool SetResolution(const int dWidth, const int dHeight);

    /*static void GX_STDCOnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame) {
        if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
            //对图像进行某些操作
            cout << "对图像进行某些操作" << endl;
        }
        return;
    }*/

    void showInfo(const GX_DEVICE_BASE_INFO *pBaseinfo);
};
