//------------------------------------------------------------------------------
// <copyright file="KinectEasyGrabber.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "NuiApi.h"
#include "ImageRenderer.h"

//#define RECORD_PLAYER
#define TOTAL_FRAMES 400
#define DRAW_FRAMES

class KinectEasyGrabber
{
    static const int        cBytesPerPixel    = 4;

	#ifdef RECORD_PLAYER
    static const NUI_IMAGE_RESOLUTION cDepthResolution = NUI_IMAGE_RESOLUTION_320x240;
	#else
	static const NUI_IMAGE_RESOLUTION cDepthResolution = NUI_IMAGE_RESOLUTION_640x480;
	#endif
    
    // Background will also be scaled to this resolution
    static const NUI_IMAGE_RESOLUTION cColorResolution = NUI_IMAGE_RESOLUTION_640x480;

    static const int        cStatusMessageMaxLen = MAX_PATH*2;

public:
    /// <summary>
    /// Constructor
    /// </summary>
    KinectEasyGrabber();

    /// <summary>
    /// Destructor
    /// </summary>
    ~KinectEasyGrabber();

    /// <summary>
    /// Handles window messages, passes most to the class instance to handle
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Handle windows messages for a class instance
    /// </summary>
    /// <param name="hWnd">window message is for</param>
    /// <param name="uMsg">message</param>
    /// <param name="wParam">message data</param>
    /// <param name="lParam">additional message data</param>
    /// <returns>result of message processing</returns>
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    /// <param name="hInstance"></param>
    /// <param name="nCmdShow"></param>
    int                     Run(HINSTANCE hInstance, int nCmdShow);

private:
    HWND                    m_hWnd;

    bool                    m_bNearMode;

    // Current Kinect
    INuiSensor*             m_pNuiSensor;

    // Direct2D
    ImageRenderer*          m_pDrawKinectEasyGrabber;
    ID2D1Factory*           m_pD2DFactory;
    
    HANDLE                  m_pDepthStreamHandle;
    HANDLE                  m_hNextDepthFrameEvent;

    HANDLE                  m_pColorStreamHandle;
    HANDLE                  m_hNextColorFrameEvent;

    LONG                    m_depthWidth;
    LONG                    m_depthHeight;

    LONG                    m_colorWidth;
    LONG                    m_colorHeight;

    LONG                    m_colorToDepthDivisor;
	//-----------------------------------------
	char*					m_frameBasename;
	int						m_frameIndex;
	int						m_totalFrames;

	USHORT**                m_outputArrayDepthD16;
	BYTE**                  m_outputArrayRGBX;
	LONG**                  m_outputArrayColorCoordinates;

	LARGE_INTEGER*          m_depthArrayTimeStamp;
    LARGE_INTEGER*          m_colorArrayTimeStamp;

	bool					m_dumped;

	//USHORT*                 m_outputDepthD16;
	//unsigned char*			m_outputPlayerUC8;
	//-----------------------------------------
    USHORT*                 m_depthD16;
    BYTE*                   m_colorRGBX;
    BYTE*                   m_backgroundRGBX;
    BYTE*                   m_outputRGBX;
    LONG*                   m_colorCoordinates;

    LARGE_INTEGER           m_depthTimeStamp;
    LARGE_INTEGER           m_colorTimeStamp;

    /// <summary>
    /// Load an image from a resource into a buffer
    /// </summary>
    /// <param name="resourceName">name of image resource to load</param>
    /// <param name="resourceType">type of resource to load</param>
    /// <param name="cOutputBuffer">size of output buffer, in bytes</param>
    /// <param name="outputBuffer">buffer that will hold the loaded image</param>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 LoadResourceImage(PCWSTR resourceName, PCWSTR resourceType, DWORD cOutputBuffer, BYTE* outputBuffer);

    /// <summary>
    /// Main processing function
    /// </summary>
    void                    Update();
	void                    Record();
	void                    RecordArray();
	void                    RecordArrayToDisk();
	void                    Play();
	void					dumpToDisk(int frameIndex, char* frameBasename, USHORT* depthD16, BYTE* colorRGBX, LONG* colorCoordinates, LARGE_INTEGER depthTimeStamp, LARGE_INTEGER colorTimeStamp);
    /// <summary>
    /// Create the first connected Kinect found 
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 CreateFirstConnected();

    /// <summary>
    /// Handle new depth data
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 ProcessDepth();

    /// <summary>
    /// Handle new color data
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 ProcessColor();

    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
    void                    SetStatusMessage(WCHAR* szMessage);
};
