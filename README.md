# 大恒相机SDK二次开发说明

<p style="text-indent:2em">实验室采购的大恒相机，无法直接使用OpenCV的VideoCapture类采集图像。虽然大恒官方提供了SDK但只有底层的接口，无法直接使用，因此需要进行二次开发并封装成类，来满足使用的需求。</p>

<p style="text-indent:2em">先初始化库、枚举设备列表、打开设备并使用SDK中的函数设置曝光延迟、帧率、分辨率、增益和白平衡等初始信息，再调用GXStreamOn函数开始采集。使用SDK中的GXDQAllBufs函数获取图像，然后将PGX_FRAME_BUFFER类型的图像转换为Mat类型。以下为主要函数Myread。</p>

```c++
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
                    
                    //图像转换
                    frame.create(pFrameBuffer[i]->nHeight, pFrameBuffer[i]->nWidth, CV_8UC3);
                    memcpy(frame.data, pFrameBuffer[i]->pImgBuf, 
                           pFrameBuffer[i]->nWidth * pFrameBuffer[i]->nHeight);
                    DxRaw8toRGB24(pFrameBuffer[i]->pImgBuf, m_rgb_image, pFrameBuffer[i]->nWidth, 
                                  pFrameBuffer[i]->nHeight, RAW2RGB_NEIGHBOUR, 
                                  DX_PIXEL_COLOR_FILTER(BAYERBG), false);
                    memcpy(frame.data, m_rgb_image, 
                           pFrameBuffer[i]->nWidth * pFrameBuffer[i]->nHeight * 3);

                    if(frame.empty()) Rflag = false;
                }
            }
            //调用GXQBuf/GXQAllBufs将图像buf放回库中继续采图
            status = GXQAllBufs(hDevice);
        }
    }
    return Rflag;
}
```

<p style="text-indent:2em">使用方法如下。</p>

```c++
ZZUcameraDriver *zzuCamera = new ZZUcameraDriver();
if(!(zzuCamera->flag)) {
    printf("Open Failed!\n");
    return -1;
}
while(zzuCamera->Myread(src)) {
    //循环读取图像进行处理
}
```

<p style="text-indent:2em">结束时在析构函数中调用SDK来结束采集、关闭设备、关闭库，以下为析构函数。</p>

```c++
ZZUcameraDriver::~ZZUcameraDriver() {
    status = GXStreamOff(hDevice);
    status = GXCloseDevice(hDevice);
    status = GXCloseLib();
    delete[] m_rgb_image;
}
```
