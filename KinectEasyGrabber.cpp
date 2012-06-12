#include "stdafx.h"
#include <strsafe.h>
#include "KinectEasyGrabber.h"
#include "resource.h"

#include <Wincodec.h>

#ifndef HINST_THISCOMPONENT
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#define HINST_THISCOMPONENT ((HINSTANCE)&__ImageBase)
#endif


/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
{
    KinectEasyGrabber application;
    application.Run(hInstance, nCmdShow);
}

/// <summary>
/// Constructor
/// </summary>
KinectEasyGrabber::KinectEasyGrabber() :
    m_pD2DFactory(NULL),
    m_pDrawKinectEasyGrabber(NULL),
    m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE),
    m_hNextColorFrameEvent(INVALID_HANDLE_VALUE),
    m_pDepthStreamHandle(INVALID_HANDLE_VALUE),
    m_pColorStreamHandle(INVALID_HANDLE_VALUE),
    m_bNearMode(false),
    m_pNuiSensor(NULL)
{
    // get resolution as DWORDS, but store as LONGs to avoid casts later
    DWORD width = 0;
    DWORD height = 0;

    NuiImageResolutionToSize(cDepthResolution, width, height);
    m_depthWidth  = static_cast<LONG>(width);
    m_depthHeight = static_cast<LONG>(height);

    NuiImageResolutionToSize(cColorResolution, width, height);
    m_colorWidth  = static_cast<LONG>(width);
    m_colorHeight = static_cast<LONG>(height);

    m_colorToDepthDivisor = m_colorWidth/m_depthWidth;

    m_depthTimeStamp.QuadPart = 0;
    m_colorTimeStamp.QuadPart = 0;

    // create heap storage for depth pixel data in RGBX format
    m_depthD16 = new USHORT[m_depthWidth*m_depthHeight];
    m_colorCoordinates = new LONG[m_depthWidth*m_depthHeight*2];
    m_colorRGBX = new BYTE[m_colorWidth*m_colorHeight*cBytesPerPixel];

	//m_backgroundRGBX = NULL;
	m_outputRGBX = NULL;
    m_backgroundRGBX = new BYTE[m_colorWidth*m_colorHeight*cBytesPerPixel];
    //m_outputRGBX = new BYTE[m_colorWidth*m_colorHeight*cBytesPerPixel];

	//m_outputDepthD16 = new USHORT[m_depthWidth*m_depthHeight];
	//m_outputPlayerUC8 = new unsigned char[m_depthWidth*m_depthHeight];

	//USHORT**                m_outputArrayDepthD16;
	//unsigned char**			m_outputArrayPlayerUC8;
	//LONG*                   m_outputArrayColorCoordinates;
	//BYTE**                  m_outputArrayRGBX;

	m_frameBasename = new char[256];
	m_frameIndex = 0;
	m_totalFrames = TOTAL_FRAMES;
	m_dumped = false;

	m_outputArrayDepthD16 = new USHORT*[m_totalFrames];
	m_outputArrayColorCoordinates = new LONG*[m_totalFrames];
	m_outputArrayRGBX = new BYTE*[m_totalFrames];

	m_depthArrayTimeStamp = new LARGE_INTEGER[m_totalFrames];
    m_colorArrayTimeStamp = new LARGE_INTEGER[m_totalFrames];

	// Heavy memory allocations
	for(int i=0; i < m_totalFrames; i++)
		m_outputArrayDepthD16[i] = new USHORT[m_depthWidth*m_depthHeight];
	for(int i=0; i < m_totalFrames; i++)
		m_outputArrayColorCoordinates[i] = new LONG[m_depthWidth*m_depthHeight*2];
	for(int i=0; i < m_totalFrames; i++)
		m_outputArrayRGBX[i] = new BYTE[m_colorWidth*m_colorHeight*cBytesPerPixel];
}


/// <summary>
/// Destructor
/// </summary>
KinectEasyGrabber::~KinectEasyGrabber()
{
    if (m_pNuiSensor)
    {
        m_pNuiSensor->NuiShutdown();
    }

    if (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextDepthFrameEvent);
    }

    if (m_hNextColorFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextColorFrameEvent);
    }

    // clean up Direct2D renderer
    delete m_pDrawKinectEasyGrabber;
    m_pDrawKinectEasyGrabber = NULL;

    // done with pixel data
    delete[] m_depthD16;
    delete[] m_colorCoordinates;
    delete[] m_colorRGBX;

    //delete[] m_backgroundRGBX;
    //delete[] m_outputRGBX;

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    SafeRelease(m_pNuiSensor);

	delete [] m_frameBasename;
	//delete [] m_outputDepthD16;
	//delete [] m_outputPlayerUC8;

	//// Heavy memory deallocations -> now in RecorToDisk()
	//for(int i=0; i < m_totalFrames; i++)
	//	delete [] m_outputArrayDepthD16[i];
	//for(int i=0; i < m_totalFrames; i++)
	//	delete [] m_outputArrayColorCoordinates[i];
	//for(int i=0; i < m_totalFrames; i++)
	//	delete [] m_outputArrayRGBX[i];

	//delete [] m_outputArrayDepthD16;
	//delete [] m_outputArrayColorCoordinates;
	//delete [] m_outputArrayRGBX;
	//delete [] m_colorArrayTimeStamp;
	//delete [] m_depthArrayTimeStamp;
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int KinectEasyGrabber::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;
    wc.cbWndExtra    = DLGWINDOWEXTRA;
    wc.hInstance     = hInstance;
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));
    wc.lpfnWndProc   = DefDlgProcW;
    wc.lpszClassName = L"KinectEasyGrabberAppDlgWndClass";

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window
    HWND hWndApp = CreateDialogParamW(
        hInstance,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)KinectEasyGrabber::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window
    ShowWindow(hWndApp, nCmdShow);

    const int eventCount = 2;
    HANDLE hEvents[eventCount];

    LoadResourceImage(L"Background", L"Image", m_colorWidth*m_colorHeight*cBytesPerPixel, m_backgroundRGBX);

    // Main message loop
    while (WM_QUIT != msg.message)
    {
        hEvents[0] = m_hNextDepthFrameEvent;
        hEvents[1] = m_hNextColorFrameEvent;

        // Check to see if we have either a message (by passing in QS_ALLINPUT)
        // Or a Kinect event (hEvents)
        // Update() will check for Kinect events individually, in case more than one are signalled
        DWORD dwEvent = MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLINPUT);

        // Check if this is an event we're waiting on and not a timeout or message
        if (WAIT_OBJECT_0 == dwEvent || WAIT_OBJECT_0 + 1 == dwEvent)
        {
            //Update();
			//Record();
			RecordArray();
        }

        if (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if ((hWndApp != NULL) && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}


/// <summary>
/// Main processing function
/// </summary>
void KinectEasyGrabber::RecordArray()
{
    if (NULL == m_pNuiSensor)
    {
        return;
    }

    bool needToDraw = false;

    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
    {
        // if we have received any valid new depth data we may need to draw
        if ( SUCCEEDED(ProcessDepth()) )
        {
            needToDraw = true;
        }
    }

    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0) )
    {
        // if we have received any valid new color data we may need to draw
        if ( SUCCEEDED(ProcessColor()) )
        {
            needToDraw = true;
        }
    }

    // Depth is 30 fps.  For any given combination of FPS, we should ensure we are within half a frame of the more frequent of the two.  
    // But depth is always the greater (or equal) of the two, so just use depth FPS.
    const int depthFps = 30;
    const int halfADepthFrameMs = (1000 / depthFps) / 2;

    // If we have not yet received any data for either color or depth since we started up, we shouldn't draw
    if (m_colorTimeStamp.QuadPart == 0 || m_depthTimeStamp.QuadPart == 0)
    {
        needToDraw = false;
    }

    // If the color frame is more than half a depth frame ahead of the depth frame we have,
    // then we should wait for another depth frame.  Otherwise, just go with what we have.
    if (m_colorTimeStamp.QuadPart - m_depthTimeStamp.QuadPart > halfADepthFrameMs)
    {
        needToDraw = false;
    }

	// cantidad de frames a grabar
	if(m_frameIndex >= m_totalFrames){ RecordArrayToDisk(); return; }
	if(m_dumped) return;

	if (needToDraw)
    {
		//Hard copy into ram
		memcpy(m_outputArrayDepthD16[m_frameIndex],m_depthD16, m_depthWidth*m_depthHeight*sizeof(USHORT));
		memcpy(m_outputArrayRGBX[m_frameIndex],m_colorRGBX, m_colorWidth*m_colorHeight*cBytesPerPixel*sizeof(BYTE));
		memcpy(m_outputArrayColorCoordinates[m_frameIndex],m_colorCoordinates, m_depthWidth*m_depthHeight*2*sizeof(LONG));

		m_colorArrayTimeStamp[m_frameIndex] = m_colorTimeStamp;
		m_depthArrayTimeStamp[m_frameIndex] = m_depthTimeStamp;

		// Draw the data with Direct2D
		#ifdef DRAW_FRAMES
        m_pDrawKinectEasyGrabber->Draw(m_colorRGBX, m_colorWidth * m_colorHeight * cBytesPerPixel);
		#endif
		m_frameIndex++;
	}
}



/// <summary>
/// Main processing function
/// </summary>
void KinectEasyGrabber::RecordArrayToDisk(){
	
	if(m_frameIndex < m_totalFrames) return;
	if(m_dumped) return;

	m_pDrawKinectEasyGrabber->Draw(m_colorRGBX, m_colorWidth * m_colorHeight * cBytesPerPixel);

	for(int i=0; i < m_totalFrames; i++){
		dumpToDisk(i, m_frameBasename, m_outputArrayDepthD16[i], m_outputArrayRGBX[i], m_outputArrayColorCoordinates[i], m_depthArrayTimeStamp[i], m_colorArrayTimeStamp[i]);
		delete [] m_outputArrayDepthD16[i];
		delete [] m_outputArrayColorCoordinates[i];
		delete [] m_outputArrayRGBX[i];
		m_outputArrayDepthD16[i] = NULL;
		m_outputArrayColorCoordinates[i] = NULL;
		m_outputArrayRGBX[i] = NULL;
	}

	//-------------------------------------------------------
	// TIMES STAMP --------------------------------
	//-------------------------------------------------------
	FILE* fid = 0;
	sprintf(m_frameBasename,"data/video_ts.txt");			
	fid = fopen(m_frameBasename,"w");
	fprintf(fid,"Frames=%d, Time=%fseg, Depth size=%dx%d, RGB size=%dx%d\n",
		m_totalFrames, (m_depthArrayTimeStamp[m_totalFrames-1].QuadPart - m_depthArrayTimeStamp[0].QuadPart)/1000.0f,
		m_depthWidth,m_depthHeight,m_colorWidth,m_colorHeight);
	fprintf(fid, "Frame\tDepth\tColor\tD_i-D_i-1\tC_i-C_i-1\tC_i-D_i\n");
	if(!fid){
		printf("ERROR abriendo el archivo %d\n",m_frameBasename);
		exit(-1);
	}

	LONGLONG dprev=0, dact=0, cprev=0, cact=0;
	for(int i=0; i < m_totalFrames; i++){
		dprev = dact; 
		cprev = cact;
		dact = m_depthArrayTimeStamp[i].QuadPart;
		cact = m_colorArrayTimeStamp[i].QuadPart;
		fprintf(fid, "%d\t%llu\t%llu\t%llu\t%llu\t%llu\n", i, dact, cact, (dact-dprev), (cact-cprev), abs(cact-dact));
	}
	fclose(fid);
	
	m_dumped = true;
	m_pDrawKinectEasyGrabber->Draw(m_backgroundRGBX, m_colorWidth * m_colorHeight * cBytesPerPixel);

	delete [] m_outputArrayDepthD16;
	delete [] m_outputArrayColorCoordinates;
	delete [] m_outputArrayRGBX;
	delete [] m_colorArrayTimeStamp;
	delete [] m_depthArrayTimeStamp;
}


/// <summary>
/// Main processing function
/// </summary>
void KinectEasyGrabber::Record()
{
    if (NULL == m_pNuiSensor)
    {
        return;
    }

    bool needToDraw = false;

    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
    {
        // if we have received any valid new depth data we may need to draw
        if ( SUCCEEDED(ProcessDepth()) )
        {
            needToDraw = true;
        }
    }

    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0) )
    {
        // if we have received any valid new color data we may need to draw
        if ( SUCCEEDED(ProcessColor()) )
        {
            needToDraw = true;
        }
    }

    // Depth is 30 fps.  For any given combination of FPS, we should ensure we are within half a frame of the more frequent of the two.  
    // But depth is always the greater (or equal) of the two, so just use depth FPS.
    const int depthFps = 30;
    const int halfADepthFrameMs = (1000 / depthFps) / 2;

    // If we have not yet received any data for either color or depth since we started up, we shouldn't draw
    if (m_colorTimeStamp.QuadPart == 0 || m_depthTimeStamp.QuadPart == 0)
    {
        needToDraw = false;
    }

    // If the color frame is more than half a depth frame ahead of the depth frame we have,
    // then we should wait for another depth frame.  Otherwise, just go with what we have.
    if (m_colorTimeStamp.QuadPart - m_depthTimeStamp.QuadPart > halfADepthFrameMs)
    {
        needToDraw = false;
    }

	if (needToDraw)
    {
		if(m_frameIndex >= 50) return;

		dumpToDisk(m_frameIndex, m_frameBasename, m_depthD16, m_colorRGBX, m_colorCoordinates, m_depthTimeStamp, m_colorTimeStamp);

		// Draw the data with Direct2D
        m_pDrawKinectEasyGrabber->Draw(m_colorRGBX, m_colorWidth * m_colorHeight * cBytesPerPixel);

		m_frameIndex++;
	}
}


void KinectEasyGrabber::dumpToDisk(int frameIndex, char* frameBasename, USHORT* depthD16, BYTE* colorRGBX, LONG* colorCoordinates, LARGE_INTEGER depthTimeStamp, LARGE_INTEGER colorTimeStamp){

		if(m_dumped) return;
		FILE* fid = 0;

		//-------------------------------------------------------
		// DEPTH & PLAYER INDEX ---------------------------------
		//-------------------------------------------------------
		#ifdef RECORD_PLAYER
		unsigned char* outputPlayerUC8 = new unsigned char[m_depthWidth * m_depthHeight];
		for(USHORT* pBufferEnd = (depthD16 + (m_depthWidth * m_depthHeight)); 
			depthD16 != pBufferEnd; depthD16++, outputPlayerUC8++){
			*outputPlayerUC8 = NuiDepthPixelToPlayerIndex(*depthD16);
            *depthD16 = NuiDepthPixelToDepth(*depthD16);			
		}
		depthD16 -= (m_depthWidth * m_depthHeight); //restablezco el puntero
		outputPlayerUC8 -= (m_depthWidth * m_depthHeight); //restablezco el puntero

		// Player index -----------------------------
		sprintf (frameBasename,"data/player/video_player_%d.pgm",frameIndex);
		fid = fopen(frameBasename,"wb");
		if(!fid){
			printf("ERROR abriendo el archivo %d\n",frameBasename);
			exit(-1);
		}
		fprintf(fid, "P5\n#TS=%llu\n", depthTimeStamp.QuadPart);
		fprintf(fid, "%i %i\n%i\n", m_depthWidth, m_depthHeight, 7);

		fwrite(outputPlayerUC8,1,m_depthWidth*m_depthHeight*sizeof(unsigned char),fid);
		fclose(fid);
		delete [] outputPlayerUC8;
		#endif

		//depth -----
		sprintf (frameBasename,"data/depth/video_depth_%d.pgm",frameIndex);
		fid = fopen(frameBasename,"wb");
		if(!fid){
			printf("ERROR abriendo el archivo %d\n",frameBasename);
			exit(-1);
		}
		fprintf(fid, "P5\n#TS=%llu\n", depthTimeStamp.QuadPart);
		fprintf(fid, "%i %i\n%i\n", m_depthWidth, m_depthHeight, 65535);

		fwrite(depthD16,1,m_depthWidth*m_depthHeight*sizeof(USHORT),fid);
		fclose(fid);
		//-------------------------------------------------------
		// COLOR FRAME ------------------------------------------
		//-------------------------------------------------------
		sprintf (frameBasename,"data/rgb/video_rgb_%d.ppm",frameIndex);
		fid = fopen(frameBasename,"wb");
		if(!fid){
			printf("ERROR abriendo el archivo %d\n",frameBasename);
			exit(-1);
		}
		fprintf(fid, "P6\n#TS=%llu\n", colorTimeStamp.QuadPart);
		fprintf(fid, "%i %i\n%i\n", m_colorWidth, m_colorHeight, 255);
		//optimize this---
		BYTE firsts[6] = {colorRGBX[2],colorRGBX[1],colorRGBX[0],
						  colorRGBX[6],colorRGBX[5],colorRGBX[4]};
		for(int i=0; i < 2; i++)
			colorRGBX[i] = firsts[i];
		//------
		for(int i=2; i < m_colorWidth*m_colorHeight; i++){
			colorRGBX[i*3+2] = colorRGBX[i*4];
			colorRGBX[i*3+1] = colorRGBX[i*4+1];
			colorRGBX[i*3] = colorRGBX[i*4+2];
		}
		fwrite(colorRGBX,1,m_colorWidth*m_colorHeight*3,fid);
		fclose(fid);

		//-------------------------------------------------------
		// COLOR COORDINATES --------------------------------
		//-------------------------------------------------------
		sprintf (frameBasename,"data/map/video_map_%d.coord",frameIndex);			
		fid = fopen(frameBasename,"wb");
		if(!fid){
			printf("ERROR abriendo el archivo %d\n",frameBasename);
			exit(-1);
		}
		fprintf(fid, "%d %d\n", m_depthWidth, m_depthHeight);
		fwrite(colorCoordinates,1,m_depthWidth*m_depthHeight*2*sizeof(LONG),fid);
		fclose(fid);
}

/// <summary>
/// Main processing function
/// </summary>
void KinectEasyGrabber::Update()
{
    if (NULL == m_pNuiSensor)
    {
        return;
    }

    bool needToDraw = false;

    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
    {
        // if we have received any valid new depth data we may need to draw
        if ( SUCCEEDED(ProcessDepth()) )
        {
            needToDraw = true;
        }
    }

    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0) )
    {
        // if we have received any valid new color data we may need to draw
        if ( SUCCEEDED(ProcessColor()) )
        {
            needToDraw = true;
        }
    }

    // Depth is 30 fps.  For any given combination of FPS, we should ensure we are within half a frame of the more frequent of the two.  
    // But depth is always the greater (or equal) of the two, so just use depth FPS.
    const int depthFps = 30;
    const int halfADepthFrameMs = (1000 / depthFps) / 2;

    // If we have not yet received any data for either color or depth since we started up, we shouldn't draw
    if (m_colorTimeStamp.QuadPart == 0 || m_depthTimeStamp.QuadPart == 0)
    {
        needToDraw = false;
    }

    // If the color frame is more than half a depth frame ahead of the depth frame we have,
    // then we should wait for another depth frame.  Otherwise, just go with what we have.
    if (m_colorTimeStamp.QuadPart - m_depthTimeStamp.QuadPart > halfADepthFrameMs)
    {
        needToDraw = false;
    }

    if (needToDraw)
    {
        int outputIndex = 0;
        LONG* pDest;
        LONG* pSrc;

        // loop over each row and column of the color
        for (LONG y = 0; y < m_colorHeight; ++y)
        {
            for (LONG x = 0; x < m_colorWidth; ++x)
            {
                // calculate index into depth array
                //int depthIndex = x/m_colorToDepthDivisor + y/m_colorToDepthDivisor * m_depthWidth;
				int depthIndex = x + y* m_depthWidth;

                USHORT depth  = m_depthD16[depthIndex];
                USHORT player = NuiDepthPixelToPlayerIndex(depth);

                // default setting source to copy from the background pixel
                pSrc  = (LONG *)m_backgroundRGBX + outputIndex;

                // if we're tracking a player for the current pixel, draw from the color camera
                if ( player > 0 )
                {
                    // retrieve the depth to color mapping for the current depth pixel
                    LONG colorInDepthX = m_colorCoordinates[depthIndex * 2];
                    LONG colorInDepthY = m_colorCoordinates[depthIndex * 2 + 1];

                    // make sure the depth pixel maps to a valid point in color space
                    if ( colorInDepthX >= 0 && colorInDepthX < m_colorWidth && colorInDepthY >= 0 && colorInDepthY < m_colorHeight )
                    {
                        // calculate index into color array
                        LONG colorIndex = colorInDepthX + colorInDepthY * m_colorWidth;

                        // set source for copy to the color pixel
                        pSrc  = (LONG *)m_colorRGBX + colorIndex;
                    }
                }

                // calculate output pixel location
                pDest = (LONG *)m_outputRGBX + outputIndex++;

                // write output
                *pDest = *pSrc;
            }
        }

        // Draw the data with Direct2D
        m_pDrawKinectEasyGrabber->Draw(m_outputRGBX, m_colorWidth * m_colorHeight * cBytesPerPixel);
    }
}



/// <summary>
/// Main processing function
/// </summary>
void KinectEasyGrabber::Play()
{
    if (NULL == m_pNuiSensor)
    {
        return;
    }

    bool needToDraw = false;

    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
    {
        // if we have received any valid new depth data we may need to draw
        if ( SUCCEEDED(ProcessDepth()) )
        {
            needToDraw = true;
        }
    }

    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextColorFrameEvent, 0) )
    {
        // if we have received any valid new color data we may need to draw
        if ( SUCCEEDED(ProcessColor()) )
        {
            needToDraw = true;
        }
    }

    // Depth is 30 fps.  For any given combination of FPS, we should ensure we are within half a frame of the more frequent of the two.  
    // But depth is always the greater (or equal) of the two, so just use depth FPS.
    const int depthFps = 30;
    const int halfADepthFrameMs = (1000 / depthFps) / 2;

    // If we have not yet received any data for either color or depth since we started up, we shouldn't draw
    if (m_colorTimeStamp.QuadPart == 0 || m_depthTimeStamp.QuadPart == 0)
    {
        needToDraw = false;
    }

    // If the color frame is more than half a depth frame ahead of the depth frame we have,
    // then we should wait for another depth frame.  Otherwise, just go with what we have.
    if (m_colorTimeStamp.QuadPart - m_depthTimeStamp.QuadPart > halfADepthFrameMs)
    {
        needToDraw = false;
    }

    if (needToDraw)
    {
        int outputIndex = 0;
        LONG* pDest;
        LONG* pSrc;

        // loop over each row and column of the color
        for (LONG y = 0; y < m_colorHeight; ++y)
        {
            for (LONG x = 0; x < m_colorWidth; ++x)
            {
                // calculate index into depth array
                int depthIndex = x/m_colorToDepthDivisor + y/m_colorToDepthDivisor * m_depthWidth;

                USHORT depth  = m_depthD16[depthIndex];
                USHORT player = NuiDepthPixelToPlayerIndex(depth);

                // default setting source to copy from the background pixel
                pSrc  = (LONG *)m_backgroundRGBX + outputIndex;

                // if we're tracking a player for the current pixel, draw from the color camera
                if ( player > 0 )
                {
                    // retrieve the depth to color mapping for the current depth pixel
                    LONG colorInDepthX = m_colorCoordinates[depthIndex * 2];
                    LONG colorInDepthY = m_colorCoordinates[depthIndex * 2 + 1];

                    // make sure the depth pixel maps to a valid point in color space
                    if ( colorInDepthX >= 0 && colorInDepthX < m_colorWidth && colorInDepthY >= 0 && colorInDepthY < m_colorHeight )
                    {
                        // calculate index into color array
                        LONG colorIndex = colorInDepthX + colorInDepthY * m_colorWidth;

                        // set source for copy to the color pixel
                        pSrc  = (LONG *)m_colorRGBX + colorIndex;
                    }
                }

                // calculate output pixel location
                pDest = (LONG *)m_outputRGBX + outputIndex++;

                // write output
                *pDest = *pSrc;
            }
        }

        // Draw the data with Direct2D
        m_pDrawKinectEasyGrabber->Draw(m_outputRGBX, m_colorWidth * m_colorHeight * cBytesPerPixel);
    }
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK KinectEasyGrabber::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    KinectEasyGrabber* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<KinectEasyGrabber*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<KinectEasyGrabber*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK KinectEasyGrabber::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen
            m_pDrawKinectEasyGrabber = new ImageRenderer();
            HRESULT hr = m_pDrawKinectEasyGrabber->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, m_colorWidth, m_colorHeight, m_colorWidth * sizeof(long));
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.");
            }

            // Look for a connected Kinect, and create it if found
            CreateFirstConnected();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND:
            // If it was for the near mode control and a clicked event, change near mode
            if (IDC_CHECK_NEARMODE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
            {
                // Toggle out internal state for near mode
                m_bNearMode = !m_bNearMode;

                if (NULL != m_pNuiSensor)
                {
                    // Set near mode based on our internal state
                    m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_pDepthStreamHandle, m_bNearMode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);
                }
            }
            break;
    }

    return FALSE;
}

/// <summary>
/// Create the first connected Kinect found 
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectEasyGrabber::CreateFirstConnected()
{
    INuiSensor * pNuiSensor;
    HRESULT hr;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr))
    {
        return hr;
    }

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr))
        {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr)
        {
            m_pNuiSensor = pNuiSensor;
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }

    if (NULL != m_pNuiSensor)
    {
        // Initialize the Kinect and specify that we'll be using depth
		#ifdef RECORD_PLAYER
        hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_COLOR); 
		#else
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR); 
		#endif
        if (SUCCEEDED(hr))
        {
            // Create an event that will be signaled when depth data is available
            m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

#ifdef RECORD_PLAYER
            // Open a depth image stream to receive depth frames
            hr = m_pNuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
                cDepthResolution,
                0,
                2,
                m_hNextDepthFrameEvent,
                &m_pDepthStreamHandle);
#else
            // Open a depth image stream to receive depth frames
            hr = m_pNuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_DEPTH,
                cDepthResolution,
                0,
                2,
                m_hNextDepthFrameEvent,
                &m_pDepthStreamHandle);

#endif
            // Create an event that will be signaled when color data is available
            m_hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

            // Open a color image stream to receive depth frames
            hr = m_pNuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_COLOR,
                cColorResolution,
                0,
                2,
                m_hNextColorFrameEvent,
                &m_pColorStreamHandle);
        }
    }

    if (NULL == m_pNuiSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!");
        return E_FAIL;
    }

    m_pNuiSensor->NuiSkeletonTrackingDisable();

    return hr;
}

/// <summary>
/// Handle new depth data
/// </summary>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectEasyGrabber::ProcessDepth()
{
    HRESULT hr = S_OK;
    NUI_IMAGE_FRAME imageFrame;

    // Attempt to get the depth frame
    hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
    if (FAILED(hr))
    {
        return hr;
    }

    m_depthTimeStamp = imageFrame.liTimeStamp;

    INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
    NUI_LOCKED_RECT LockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
    pTexture->LockRect(0, &LockedRect, NULL, 0);

    // Make sure we've received valid data
    if (LockedRect.Pitch != 0)
    {
        memcpy(m_depthD16, LockedRect.pBits, LockedRect.size);
    }

    // We're done with the texture so unlock it
    pTexture->UnlockRect(0);

    // Release the frame
    m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);

	//LONG* prev_colorCoordinates = new LONG[m_depthWidth*m_depthHeight*2];
	//memcpy(prev_colorCoordinates, m_colorCoordinates, m_depthWidth*m_depthHeight*2);

    // Get of x, y coordinates for color in depth space
    // This will allow us to later compensate for the differences in location, angle, etc between the depth and color cameras
    m_pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
        cColorResolution,
        cDepthResolution,
        m_depthWidth*m_depthHeight,
        m_depthD16,
        m_depthWidth*m_depthHeight*2,
        m_colorCoordinates
        );

	//---
	//double diff = 0;
	//for(int y = 0; y < m_depthHeight; y ++)
	// for(int x=0; x < m_depthWidth; x++ ){
	//	 int depthIndex = x + y* m_depthWidth;
	//	 double xdiff = abs(prev_colorCoordinates[2*depthIndex] - m_colorCoordinates[2*depthIndex]);
	//	 double ydiff = abs(prev_colorCoordinates[2*depthIndex+1] - m_colorCoordinates[2*depthIndex+1]);
	//	diff += xdiff+ ydiff;
	// }
	//printf("diff=f\n",diff);
	//delete [] prev_colorCoordinates;
	//---


    return hr;
}

/// <summary>
/// Handle new color data
/// </summary>
/// <returns>S_OK for success or error code</returns>
HRESULT KinectEasyGrabber::ProcessColor()
{
    HRESULT hr = S_OK;
    NUI_IMAGE_FRAME imageFrame;

    // Attempt to get the depth frame
    hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pColorStreamHandle, 0, &imageFrame);
    if (FAILED(hr))
    {
        return hr;
    }

    m_colorTimeStamp = imageFrame.liTimeStamp;

    INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
    NUI_LOCKED_RECT LockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
    pTexture->LockRect(0, &LockedRect, NULL, 0);

    // Make sure we've received valid data
    if (LockedRect.Pitch != 0)
    {
        memcpy(m_colorRGBX, LockedRect.pBits, LockedRect.size);
    }

    // We're done with the texture so unlock it
    pTexture->UnlockRect(0);

    // Release the frame
    m_pNuiSensor->NuiImageStreamReleaseFrame(m_pColorStreamHandle, &imageFrame);

    return hr;
}

/// <summary>
/// Load an image from a resource into a buffer
/// </summary>
/// <param name="resourceName">name of image resource to load</param>
/// <param name="resourceType">type of resource to load</param>
/// <param name="cOutputBuffer">size of output buffer, in bytes</param>
/// <param name="outputBuffer">buffer that will hold the loaded image</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT KinectEasyGrabber::LoadResourceImage(
    PCWSTR resourceName,
    PCWSTR resourceType,
    DWORD cOutputBuffer,
    BYTE* outputBuffer
    )
{
    HRESULT hr = S_OK;

    IWICImagingFactory* pIWICFactory = NULL;
    IWICBitmapDecoder* pDecoder = NULL;
    IWICBitmapFrameDecode* pSource = NULL;
    IWICStream* pStream = NULL;
    IWICFormatConverter* pConverter = NULL;
    IWICBitmapScaler* pScaler = NULL;

    HRSRC imageResHandle = NULL;
    HGLOBAL imageResDataHandle = NULL;
    void *pImageFile = NULL;
    DWORD imageFileSize = 0;

    hr = CoCreateInstance(CLSID_WICImagingFactory, NULL, CLSCTX_INPROC_SERVER, IID_IWICImagingFactory, (LPVOID*)&pIWICFactory);
    if ( FAILED(hr) ) return hr;

    // Locate the resource.
    imageResHandle = FindResourceW(HINST_THISCOMPONENT, resourceName, resourceType);
    hr = imageResHandle ? S_OK : E_FAIL;

    if (SUCCEEDED(hr))
    {
        // Load the resource.
        imageResDataHandle = LoadResource(HINST_THISCOMPONENT, imageResHandle);
        hr = imageResDataHandle ? S_OK : E_FAIL;
    }

    if (SUCCEEDED(hr))
    {
        // Lock it to get a system memory pointer.
        pImageFile = LockResource(imageResDataHandle);
        hr = pImageFile ? S_OK : E_FAIL;
    }

    if (SUCCEEDED(hr))
    {
        // Calculate the size.
        imageFileSize = SizeofResource(HINST_THISCOMPONENT, imageResHandle);
        hr = imageFileSize ? S_OK : E_FAIL;
    }

    if (SUCCEEDED(hr))
    {
        // Create a WIC stream to map onto the memory.
        hr = pIWICFactory->CreateStream(&pStream);
    }

    if (SUCCEEDED(hr))
    {
        // Initialize the stream with the memory pointer and size.
        hr = pStream->InitializeFromMemory(
            reinterpret_cast<BYTE*>(pImageFile),
            imageFileSize
            );
    }

    if (SUCCEEDED(hr))
    {
        // Create a decoder for the stream.
        hr = pIWICFactory->CreateDecoderFromStream(
            pStream,
            NULL,
            WICDecodeMetadataCacheOnLoad,
            &pDecoder
            );
    }

    if (SUCCEEDED(hr))
    {
        // Create the initial frame.
        hr = pDecoder->GetFrame(0, &pSource);
    }

    if (SUCCEEDED(hr))
    {
        // Convert the image format to 32bppPBGRA
        // (DXGI_FORMAT_B8G8R8A8_UNORM + D2D1_ALPHA_MODE_PREMULTIPLIED).
        hr = pIWICFactory->CreateFormatConverter(&pConverter);
    }

    if (SUCCEEDED(hr))
    {
        hr = pIWICFactory->CreateBitmapScaler(&pScaler);
    }

    if (SUCCEEDED(hr))
    {
        hr = pScaler->Initialize(
            pSource,
            m_colorWidth,
            m_colorHeight,
            WICBitmapInterpolationModeCubic
            );
    }

    if (SUCCEEDED(hr))
    {
        hr = pConverter->Initialize(
            pScaler,
            GUID_WICPixelFormat32bppPBGRA,
            WICBitmapDitherTypeNone,
            NULL,
            0.f,
            WICBitmapPaletteTypeMedianCut
            );
    }

    UINT width = 0;
    UINT height = 0;
    if (SUCCEEDED(hr))
    {
        hr = pConverter->GetSize(&width, &height);
    }

    // make sure the output buffer is large enough
    if (SUCCEEDED(hr))
    {
        if ( width*height*cBytesPerPixel > cOutputBuffer )
        {
            hr = E_FAIL;
        }
    }

    if (SUCCEEDED(hr))
    {
        hr = pConverter->CopyPixels(NULL, width*cBytesPerPixel, cOutputBuffer, outputBuffer);
    }

    SafeRelease(pScaler);
    SafeRelease(pConverter);
    SafeRelease(pSource);
    SafeRelease(pDecoder);
    SafeRelease(pStream);
    SafeRelease(pIWICFactory);

    return hr;
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
void KinectEasyGrabber::SetStatusMessage(WCHAR * szMessage)
{
    SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, (LPARAM)szMessage);
}
