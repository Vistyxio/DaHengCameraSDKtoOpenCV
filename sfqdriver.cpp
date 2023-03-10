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
    //εε§εεΊ
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) {
        cout << "status != GX_STATUS_SUCCESS" << endl;
        flag = false;
        return;
    }
    //ζδΈΎθ?Ύε€εθ‘¨
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if (status == GX_STATUS_SUCCESS && nDeviceNum> 0) {
        GX_DEVICE_BASE_INFO *pBaseinfo = new GX_DEVICE_BASE_INFO[nDeviceNum];
        uint64_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
        //θ·εζζθ?Ύε€ηεΊη‘δΏ‘ζ―
        status = GXGetAllDeviceBaseInfo(pBaseinfo, &nSize);
        showInfo(pBaseinfo); //θΎεΊθ?Ύε€δΏ‘ζ―
        delete []pBaseinfo;
    }

    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0)) {
        cout << "(status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0)" << endl;
        flag = false;
        return;
    }


    //ζεΌθ?Ύε€
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

    //εΌι
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
    //ε?δΉGXDQBufηδΌ ε₯εζ°
    PGX_FRAME_BUFFER pFrameBuffer[10];
    uint32_t nFrameCount = 0;

    if (status == GX_STATUS_SUCCESS) {
        //θ°η¨GXDQBuf/GXDQAllBufsεδΈ/ε€εΈ§εΎε
        status = GXDQAllBufs(hDevice, pFrameBuffer, 10, &nFrameCount, 1000);
        if (status == GX_STATUS_SUCCESS) {
            for (register uint32_t i = 0; i < nFrameCount; i++) {
                if ((pFrameBuffer[i] != NULL) &&
                        (pFrameBuffer[i]->nStatus == GX_FRAME_STATUS_SUCCESS)) {
                    //η¬¬iεΉεΎεθ·εζε
                    Rflag = true;

                    frame.create(pFrameBuffer[i]->nHeight, pFrameBuffer[i]->nWidth, CV_8UC3);

                    memcpy(frame.data, pFrameBuffer[i]->pImgBuf, pFrameBuffer[i]->nWidth * pFrameBuffer[i]->nHeight);

                    DxRaw8toRGB24(pFrameBuffer[i]->pImgBuf, m_rgb_image, pFrameBuffer[i]->nWidth, pFrameBuffer[i]->nHeight, RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(BAYERBG), false);

                    memcpy(frame.data, m_rgb_image, pFrameBuffer[i]->nWidth * pFrameBuffer[i]->nHeight * 3);

                    //cout << (clock() - t) * 1000 / CLOCKS_PER_SEC << "ms" << endl;


                    if(frame.empty()) Rflag = false;
                }
            }
            //θ°η¨GXQBuf/GXQAllBufsε°εΎεbufζΎεεΊδΈ­η»§η»­ιεΎ
            status = GXQAllBufs(hDevice);

        }

    }

    return Rflag;
}

//ηΈζΊεζ°θ°θ

//**θ?Ύη½?η¨ζ·θͺε?δΉεη§°**
bool ZZUcameraDriver::SetCamName(const string NewName) {

    status = GXSetString(hDevice, GX_STRING_DEVICE_USERID, (char*)(NewName.c_str()));
    return status;
}

//**ζεε»ΆθΏ**
bool ZZUcameraDriver::SetExposure(const bool ifAuto, const float dExposureValue) {

    //θ·εζεθ°θθε΄
    GX_FLOAT_RANGE shutterRange;
    status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);

    if(ifAuto) {
        //θ?Ύη½?θΏη»­θͺε¨ζε
        status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    } else {
        if((dExposureValue < shutterRange.dMin) || (dExposureValue > shutterRange.dMax)) {
            cout << "dExposureValue Out of Range!" << endl;
            return true;
        }
        //θ?Ύη½?ζεε»ΆθΏδΈΊdExposureValue
        status = GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
        status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, dExposureValue);
    }
    return status;
}

//**εΈ§η**
bool ZZUcameraDriver::SetFrameRate(const float dFrameRateValue) {

    //δ½Ώθ½ιιεΈ§ηθ°θζ¨‘εΌ
    status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
    //θ?Ύη½?ιιεΈ§η
    status = GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, dFrameRateValue);
    return status;
}

//**εθΎ¨η**
bool ZZUcameraDriver::SetResolution(const int dWidth, const int dHeight) {
    //θ?Ύη½?εθΎ¨η
    //GX_INT_WIDTH = 640;
    //GX_INT_HEIGHT = 480;
    status = GXSetInt(hDevice, GX_INT_WIDTH, dWidth);
    status = GXSetInt(hDevice, GX_INT_HEIGHT, dHeight);
    return status;
}

//**ε’η**
bool ZZUcameraDriver::SetGain(const int GainMode, const bool ifAuto, const float dGainValue) {

    //ιζ©ε’ηιιη±»ε
    status = GXSetEnum(hDevice, GX_ENUM_GAIN_SELECTOR, GainMode);
    //θ·εε’ηθ°θθε΄
    GX_FLOAT_RANGE gainRange;
    status = GXGetFloatRange(hDevice, GX_FLOAT_GAIN, &gainRange);


    if(ifAuto) {
        //θ?Ύη½?θΏη»­θͺε¨ε’η
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    } else {
        if((dGainValue < gainRange.dMin) || (dGainValue > gainRange.dMax)) {
            cout << "dGainValue Out of Range!" << endl;
            return true;
        }
        //θ?Ύη½?ε’ηεΌ
        status = GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
        status = GXSetFloat(hDevice, GX_FLOAT_GAIN, dGainValue);
    }
    return status;
}

//**η½εΉ³θ‘‘**
bool ZZUcameraDriver::SetBalanceRatio(const bool ifAuto,
                                      const float dBalanceRatioValueR,
                                      const float dBalanceRatioValueG,
                                      const float dBalanceRatioValueB) {


    //θ·εη½εΉ³θ‘‘θ°θθε΄
    GX_FLOAT_RANGE ratioRange;
    status = GXGetFloatRange(hDevice, GX_FLOAT_BALANCE_RATIO, &ratioRange);


    if(ifAuto) {
        //θͺε¨η½εΉ³θ‘‘θ?Ύη½?
        //θ?Ύη½?θͺε¨η½εΉ³θ‘‘εη§η―ε’,ζ―ε¦ε½εηΈζΊζε€η―ε’δΈΊθ§εη―
        status = GXSetEnum(hDevice, GX_ENUM_AWB_LAMP_HOUSE,
                           GX_AWB_LAMP_HOUSE_FLUORESCENCE);
        //θ?Ύη½?θΏη»­θͺε¨η½εΉ³θ‘‘
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
        //θ?Ύη½?η½εΉ³θ‘‘
        status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO,
                           GX_BALANCE_WHITE_AUTO_OFF);
        //ιζ©η½εΉ³θ‘‘ιι
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,
                           GX_BALANCE_RATIO_SELECTOR_RED);
        status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, dBalanceRatioValueR);
        //ιζ©η½εΉ³θ‘‘ιι
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,
                           GX_BALANCE_RATIO_SELECTOR_GREEN);
        status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, dBalanceRatioValueG);
        //ιζ©η½εΉ³θ‘‘ιι
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_RATIO_SELECTOR,
                           GX_BALANCE_RATIO_SELECTOR_BLUE);
        status = GXSetFloat(hDevice, GX_FLOAT_BALANCE_RATIO, dBalanceRatioValueB);
    }
    return status;
}

void ZZUcameraDriver::showInfo(const GX_DEVICE_BASE_INFO *pBaseinfo) {

    cout << "---------------------θ?Ύε€εΊη‘δΏ‘ζ―---------------------" << endl;
    cout << "εεεη§°------------------" << pBaseinfo->szVendorName << endl;
    cout << "θ?Ύε€η±»εεη§°--------------" << pBaseinfo->szModelName << endl;
    cout << "θ?Ύε€εΊεε·----------------" << pBaseinfo->szSN << endl;
    cout << "θ?Ύε€ε±η€Ίεη§°--------------" << pBaseinfo->szDisplayName << endl;
    cout << "η¨ζ·θͺε?δΉεη§°------------" << pBaseinfo->szUserID << endl;
    cout << "θ?Ύε€ε―δΈζ θ―--------------" << pBaseinfo->szDeviceID << endl;
    cout << "θ?Ύε€ε½εζ―ζηθ?Ώι?ηΆζ----";
    switch(pBaseinfo->accessStatus) {
    case 0:
        cout << "ηΆζζͺη₯" << endl;
        break;
    case 1:
        cout << "ε―θ―»ε―ε" << endl;
        break;
    case 2:
        cout << "δ»ζ―ζθ―»" << endl;
        break;
    case 3:
        cout << "ζ’δΈζ―ζθ―»,εδΈζ―ζε" << endl;
        break;
    }
    cout << "θ?Ύε€η§η±»------------------";
    switch(pBaseinfo->deviceClass) {
    case 0:
        cout << "ζͺη₯θ?Ύε€η§η±»" << endl;
        break;
    case 1:
        cout << "USB2.0 θ?Ύε€" << endl;
        break;
    case 2:
        cout << "εεη½θ?Ύε€(Gige Vision)" << endl;
        break;
    case 3:
        cout << "USB3.0 θ?Ύε€(USB3 Vision)" << endl;
        break;
    }
    cout << "------------------------------------------------------" << endl << endl;
}
