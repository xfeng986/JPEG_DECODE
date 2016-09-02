/*************************************************************************
 * Nuvoton Electronics Corporation confidential
 *
 * Copyright (c) 2008 by Nuvoton Electronics Corporation
 * All rights reserved
 *
 * FILENAME
 *     main.c
 *
 * VERSION
 *     1.0
 *
 * DESCRIPTION
 *     NUC930 USB Host integration test program
 *
 * HISTORY
 *     2008.06.24       Created
 *
 * REMARK
 *     None
 **************************************************************************/
#ifdef ECOS
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "drv_api.h"
#include "diag.h"
#include "wbtypes.h"
#include "wbio.h"
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wbtypes.h"
#include "wblib.h"
#endif

#include "nvtfat.h"
#include "W55FA93_reg.h"
#include "w55fa93_vpost.h"
#include "w55fa93_spi.h"
#include "jpegcodec.h"
/* USB */
#include "usb.h"
#include "usbvideo.h"
#include "usbkbd.h"
#include "nvtfat.h"
/*png*/
#include "png.h"

#include "w55fa93_sic.h"
#include "spu.h"


#define SYSTEM_CLOCK		12000000
#define UART_BAUD_RATE		115200

#define DUMMY_BUFFER_SIZE		(64 * 1024)

/*spi*/
#define TEST_SIZE	512 * 2 * 64
//__align(4096) UINT8 WriteBuffer[TEST_SIZE];
//__align(4096) UINT8 ReadBuffer[TEST_SIZE];
PUINT8 WriteBuffer;
PUINT8 ReadBuffer;

#define SPIFLASH_DISK_SIZE 	((1024*1024)*8)
INT8 i8SFlashDisk[SPIFLASH_DISK_SIZE];

CHAR	suFileName2[100];
#ifdef ECOS
#define sysGetTicks(TIMER0)   cyg_current_time()
#endif

#ifdef __JPEG_DEC_OUTPUT_RGB888__
	#define JPEG_DECODE_OUTPUT_FORMAT	JPEG_DEC_PRIMARY_PACKET_RGB888
#endif 

#ifndef __JPEG_DEC_OUTPUT_RGB565__
	#define JPEG_DECODE_OUTPUT_FORMAT	JPEG_DEC_PRIMARY_PACKET_RGB565
#endif 

#ifdef __JPEG_DEC_OUTPUT_RGB555__
	#define JPEG_DECODE_OUTPUT_FORMAT	JPEG_DEC_PRIMARY_PACKET_RGB555
#endif 

#ifdef __JPEG_DEC_OUTPUT_YUV422__
	#define	JPEG_DECODE_OUTPUT_FORMAT	JPEG_DEC_PRIMARY_PACKET_YUV422	
#endif 

#ifdef __JPEG_DEC_OUTPUT_YUV__
	#define JPEG_DECODE_OUTPUT_FORMAT	JPEG_DEC_PRIMARY_PLANAR_YUV
#endif 
#define PLANAR_DEC_BUFFER_SIZE	0x100000	/* Decode Output Buffer size for Planar */

#define PANEL_WIDTH		480					/* PANEL Width (Raw data output width for Panel Test) */
#define PANEL_HEIGHT	272     			/* PANEL Height (Raw data output height for Panel Test) */

#define TARGET_WIDTH	320					/* JPEG decode output width for __PANEL_TEST__ */
#define TARGET_HEIGHT	240 				/* JPEG decode output height for __PANEL_TEST__ */
          
#define PANEL_BPP		2
#define FB_ADDR		0x500000
/*png*/
#define PNG_BYTES_TO_CHECK 4

/*-----------------------------------------------------------------------*/
/*  JPEG OPTIONAL FUNCTION ENABLE DEFINITION                             */
/*-----------------------------------------------------------------------*/
#ifndef __JPEG_DEC_OUTPUT_YUV__
	#define __PANEL_TEST__			/* Set Decode Stride to PANEL_WIDTH & put decoded image at the center of frame buffer */	
#endif	

#if 0
#define ENC_WIDTH		640					/* Encode Width	*/
#define ENC_HEIGHT		480					/* Encode Height */
#else
#define ENC_WIDTH		720					/* Encode Width	*/
#define ENC_HEIGHT		576					/* Encode Height */
#endif
//static UINT8  _pucDummy[DUMMY_BUFFER_SIZE];

PUINT8 g_pu8JpegBuffer;						/* The source bit stream data for decoding */
PUINT8 g_pu8DecFrameBuffer;					/* The buffer for decoding output */
UINT8 __align(32) g_au8BitstreamBuffer[0x100000];		/* The buffer for encoding output */
PUINT8 g_pu8EncFrameBuffer;								/* Source image data for encoding */
UINT8	_JpegImage[256 * 1024];
UINT8	_JpegImageR[256 * 1024];

CHAR	decodePath[50];
CHAR g_szOutputString[15];

CHAR png[500];
UINT8 pos=0;

extern UINT32	_QueuedSize;

extern INT  W99683_HasImageQueued(VOID);
/************************************************************************
* png
************************************************************************/
/************************************************************************
* pngend
************************************************************************/
CHAR *intToStr(UINT32 u32quotient)
{
    UINT32 u32i = 0,
           u32reverseStringElementNo,
           u32remainder;
    CHAR szReverseString[15];
         
    do
    {
        u32remainder = u32quotient % 10;
        szReverseString[u32i] = (char)(u32remainder + 0x30);
        u32quotient /= 10;
        u32i++;
    } while(u32quotient != 0);
    
    u32reverseStringElementNo = u32i - 1;
    
    for(u32i = 0; u32i <= u32reverseStringElementNo; u32i++)
        g_szOutputString[u32i] = szReverseString[u32reverseStringElementNo - u32i];

    g_szOutputString[u32i] = '\0';
    
    return g_szOutputString;
}            
/*-----------------------------------------------------------------------*/
/* LCD                            */
/*-----------------------------------------------------------------------*/            
static UINT32 bIsInitVpost=FALSE;
LCDFORMATEX lcdInfo;
void initVPost4test(void)
{
	if(bIsInitVpost==FALSE)
	{
		bIsInitVpost = TRUE;
		//lcdInfo.ucVASrcFormat = DRVVPOST_FRAME_YCBYCR;	
		lcdInfo.ucVASrcFormat = DRVVPOST_FRAME_RGB565;	//显示源格式
		lcdInfo.nScreenWidth = PANEL_WIDTH;	//输出宽
		lcdInfo.nScreenHeight = PANEL_HEIGHT;//输出高
		vpostLCMInit(&lcdInfo, (UINT32*)FB_ADDR);
		//backLightEnable();
	}	
}

/*-----------------------------------------------------------------------*/
/*  Header Decode Complete Callback function                             */
/*-----------------------------------------------------------------------*/            
BOOL JpegDecHeaderComplete(VOID)                  
{	
	UINT32 u32FrameBuffer;		
	JPEG_INFO_T jpegInfo;
	UINT32 u32BufferSize;
#ifdef __PANEL_TEST__	
	INT8	fill = 0xFF;
	UINT32 u32OutputOffset;
	UINT32 u32TargetWidth,u32TargetHeight;
#else
	UINT16 u16Width,u16Height;		
#endif	

	/* Get the JPEG information(jpeg_width, jpeg_height, and yuvformat are valid) */
	jpegGetInfo(&jpegInfo);		
	
	if(jpegInfo.jpeg_width == 0 || jpegInfo.jpeg_height == 0)	
		return FALSE;		
		
#ifdef __PANEL_TEST__	
	/* Allocate the Raw Data Buffer for Decode Operation */	
	if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PACKET_RGB888)
		u32BufferSize = PANEL_WIDTH * PANEL_HEIGHT * 4;
	else
		u32BufferSize = PANEL_WIDTH * PANEL_HEIGHT * 2;
		
	/* Allocate Raw Data Buffer for Decode Operation */	
//	g_pu8DecFrameBuffer = (PUINT8)malloc(sizeof(CHAR) * u32BufferSize);
    g_pu8DecFrameBuffer = (PUINT8)FB_ADDR;
	
	if(g_pu8DecFrameBuffer == NULL)
	{
		sysprintf("\tCan't allocate the buffer for decode (size 0x%X)\n",u32BufferSize);
		return FALSE;
	}
		
	u32FrameBuffer =  (UINT32) g_pu8DecFrameBuffer | 0x80000000;
	
	sysprintf("\tThe decoded data starts from 0x%X, Size is 0x%X\n", u32FrameBuffer,u32BufferSize);	
	
		 
	if(jpegInfo.jpeg_width > PANEL_WIDTH || jpegInfo.jpeg_height > PANEL_HEIGHT)	
	{
		/* Set Downscale to QVGA */
		jpegIoctl(JPEG_IOCTL_SET_DECODE_DOWNSCALE, TARGET_HEIGHT, TARGET_WIDTH);
		u32TargetHeight = TARGET_HEIGHT;
		u32TargetWidth = TARGET_WIDTH;	
		 	
	}		
	else
	{
		u32TargetHeight = jpegInfo.jpeg_height;
		u32TargetWidth = jpegInfo.jpeg_width;
	}
			
	/* Set Decode Stride to Panel width */	
	jpegIoctl(JPEG_IOCTL_SET_DECODE_STRIDE, PANEL_WIDTH, 0);		
	
	/* Clear Buffer */
	if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PACKET_YUV422)
		fill = 0x7F;
	
	/* The pixel offset for putting the image at the center of Frame Buffer */	
	
	u32OutputOffset = (PANEL_WIDTH * (PANEL_HEIGHT - u32TargetHeight) / 2) + (PANEL_WIDTH - u32TargetWidth) / 2;	
	
		
	if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PACKET_RGB888)
	{
		memset((UINT32*)u32FrameBuffer,fill, PANEL_WIDTH * PANEL_HEIGHT * 4);
		u32OutputOffset *= 4; 
	}
	else
	{
		if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PACKET_YUV422)
		{
			UINT32 i;
			for(i = 0;i<PANEL_WIDTH * PANEL_HEIGHT * 2;i++)
			{
				if(i%2)
					*((PUINT8)(u32FrameBuffer + i)) = 0x80;	
				else
					*((PUINT8)(u32FrameBuffer + i)) = 0xFF;
			}
		}
		else
			memset((UINT32*)u32FrameBuffer,fill, PANEL_WIDTH * PANEL_HEIGHT * 2);				
		u32OutputOffset *= 2; 
	}
	
	/* Change Raw data Output address */	
	jpegIoctl(JPEG_IOCTL_SET_YADDR, (UINT32) u32FrameBuffer + u32OutputOffset, 0);

#else
	/* For Normal Decode buffer allocation */
	if(JPEG_DECODE_OUTPUT_FORMAT != JPEG_DEC_PRIMARY_PLANAR_YUV)
	{		
		u16Width = jpegInfo.jpeg_width;
		u16Height = jpegInfo.jpeg_height;		
	
	    if(jpegInfo.yuvformat == JPEG_DEC_YUV422)   
	    {
		    /* alignment for YUV422 raw data */
		    if(u16Width % 16)
		        u16Width = (u16Width & 0xFFFFFFF0) + 16;     
		        
		    if(u16Height  % 8)
		        u16Height  = (u16Height  & 0xFFFFFFF8) + 8; 		          
	    }
	    else if(jpegInfo.yuvformat == JPEG_DEC_YUV444)
		{
		    /* alignment for YUV444 raw data */
		    if(u16Width % 8)
		        u16Width = (u16Width & 0xFFFFFFF8) + 8; 
		        
		    if(u16Height  % 8)
		        u16Height  = (u16Height  & 0xFFFFFFF8) + 8; 		        
		}
		else
		{	/* alignment for YUV420 raw data */		    
		    if(u16Width % 16)
		        u16Width = (u16Width & 0xFFFFFFF0) + 16;
		    if(u16Height % 16)
		        u16Height = (u16Height & 0xFFFFFFF0) + 16;		        
		} 

		/* Raw Data Buffer for Decode Operation */	
		if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PACKET_RGB888)
			u32BufferSize = u16Width * u16Height * 4;
		else
			u32BufferSize = u16Width * u16Height * 2;			
	
		/* Allocate Raw Data Buffer for Decode Operation */	
		g_pu8DecFrameBuffer = (PUINT8)malloc(sizeof(CHAR) * u32BufferSize *3);			
		
		if(g_pu8DecFrameBuffer == NULL)
		{
			sysprintf("\tCan't allocate the buffer for decode (size 0x%X)\n",u32BufferSize);
			return FALSE;
		}
			
		u32FrameBuffer =  (UINT32) g_pu8DecFrameBuffer | 0x80000000;
		
		sysprintf("\tThe decoded data starts from 0x%X, Size is 0x%X\n", u32FrameBuffer,u32BufferSize);
		
		/* Set Decoded Image Address (Can be set before Decode Trigger)*/
		jpegIoctl(JPEG_IOCTL_SET_YADDR, u32FrameBuffer, 0);	
	}
#endif
	
	return TRUE;	/* Return TRUE to continue Decode operation, Otherwise, Stop Decode operation */
}

INT JpegEncTest (UINT32 u32FrameBuffer, UINT32 u32OutputAddr)
{
	UINT16 u16Width = ENC_WIDTH, u16Height = ENC_HEIGHT;
	JPEG_INFO_T jpegInfo;
	INT len;
	
	sysprintf("\tJPEG Bitstream is located 0x%X\n", u32OutputAddr);
	    
	/* JPEG Open */
	jpegOpen ();
	
	/* JPEG Init */
	jpegInit();	
	
	/* Set Source Address */	
	jpegIoctl(JPEG_IOCTL_SET_YADDR, u32FrameBuffer, 0);
			
	/* Set Source Y/U/V Stride */	       
    jpegIoctl(JPEG_IOCTL_SET_YSTRIDE, u16Width, 0);
    jpegIoctl(JPEG_IOCTL_SET_USTRIDE, u16Width/2, 0);
    jpegIoctl(JPEG_IOCTL_SET_VSTRIDE, u16Width/2, 0);
     														
    /* Set Bit stream Address */   
    jpegIoctl(JPEG_IOCTL_SET_BITSTREAM_ADDR, u32OutputAddr, 0);

	/* Encode mode, encoding primary image, YUV 4:2:2/4:2:0 */	
    jpegIoctl(JPEG_IOCTL_SET_ENCODE_MODE, JPEG_ENC_SOURCE_PACKET, JPEG_ENC_PRIMARY_YUV422);
//    jpegIoctl(JPEG_IOCTL_SET_ENCODE_MODE, JPEG_ENC_SOURCE_PLANAR, JPEG_ENC_PRIMARY_YUV422);
    
    /* Primary Encode Image Width / Height */    
    jpegIoctl(JPEG_IOCTL_SET_DIMENSION, u16Height, u16Width);
       
#ifdef __ENC_SOFTWARE_RESERVE__
	/* Reserve memory space for user application 
	   # Reserve memory space Start address is Bit stream Address + 6 and driver will add the app marker (FF E0 xx xx)for user automatically. 
	   # User can set the data before or after trigger JPEG (Engine will not write the reserved memory space).	
	   # The size parameter is the actual size that can used by user and it must be multiple of 2 but not be multiple of 4 (Max is 65530). 	   
	   # User can get the marker length (reserved length + 2) from byte 4 and byte 5 in the bitstream. 
	   		Byte 0 & 1 :  FF D8
			Byte 2 & 3 :  FF E0
			Byte 4 & 5 :  [High-byte of Marker Length][Low-byte of Marker Length]
	   		Byte 6 ~ (Length + 4)  :  [(Marker Length - 2)-byte Data] for user application
	   	   
	   	 Ex : jpegIoctl(JPEG_IOCTL_ENC_RESERVED_FOR_SOFTWARE, 1024,0); 
	          FF D8 FF E0 04 02 [1024 bytes]	   	   
	*/
	jpegIoctl(JPEG_IOCTL_ENC_RESERVED_FOR_SOFTWARE, 1024,0);     
#endif       
       
    /* Primary Encode Image Width / Height */    
#ifdef __UPSCALE_TEST__	    
	/* Encode upscale 2x */
    jpegIoctl(JPEG_IOCTL_SET_ENCODE_UPSCALE, u16Height * 2, u16Width * 2);       
#endif            
    //Set Encode Source Image Height        
    jpegIoctl(JPEG_IOCTL_SET_SOURCE_IMAGE_HEIGHT, u16Height, 0);

	/* Include Quantization-Table and Huffman-Table */	
    jpegIoctl(JPEG_IOCTL_ENC_SET_HEADER_CONTROL, JPEG_ENC_PRIMARY_QTAB | JPEG_ENC_PRIMARY_HTAB, 0);
       
	/* Use the default Quantization-table 0, Quantization-table 1 */
	jpegIoctl(JPEG_IOCTL_SET_DEFAULT_QTAB, 0, 0);
	
	/* Trigger JPEG encoder */
    jpegIoctl(JPEG_IOCTL_ENCODE_TRIGGER, 0, 0);  
    
    /* Wait for complete */
	if(jpegWait())
	{
		jpegGetInfo(&jpegInfo);
		sysprintf("\tJPEG Encode Complete!!\n");
		sysprintf("\t\tJpeg Image Size = %d\n",jpegInfo.image_size[0]);	
		len = jpegInfo.image_size[0];		
    }
    else
    {
    	sysprintf("\tJPEG Encode Error!!\n");
    	len = 0;
    }
    	
    jpegClose ();   
	
	return len;   
}

void  GetJpegImage(UINT8 *image, UINT32 *len, INT interval)
{
	UINT8   *bufPTR;
	INT     bufLEN;
	UINT32	idx = 0;
	
	*len = 0;
	/* Skip frames */
	while (1)
	{
		if (W99683_GetFramePiece(&bufPTR, &bufLEN) < 0) 
			;           /* W99683 is not enabled, we wait */

		if ((bufPTR[0] == 0xFF) && (bufPTR[1] == 0xD8))
		{
			if (interval == 0)
				break;
			interval--;
		}
	}
	
	while (1)
	{	
		memcpy(&image[idx], bufPTR, bufLEN);
		idx += bufLEN;
		*len += bufLEN;
		
		if (W99683_GetFramePiece(&bufPTR, &bufLEN) < 0) 
			continue;		/* W99683 is not enabled, we wait */
		
		if ((bufPTR[0] == 0xFF) && (bufPTR[1] == 0xD8))
			return;
	}
}


static INT  Action_Compare(CHAR *suFileName1, CHAR *szAsciiName1, 
							CHAR *suFileName2, CHAR *szAsciiName2)
{
	INT		hFile1, hFile2;
	INT		nLen1, nLen2, nCount, nStatus1, nStatus2;
	UINT8	buffer1[8192], buffer2[8192];
	UINT32	nJpegLen;
	
	hFile1 = fsOpenFile(suFileName1, szAsciiName1, O_RDONLY);
	if (hFile1 < 0)
		return hFile1;
	
	hFile2 = fsOpenFile(suFileName2, szAsciiName2, O_RDONLY);
	if (hFile2 < 0)
		return hFile2;
	
	sysprintf("\nComparing file ...\n");
	nCount = 0;
	while (1)
	{
		nStatus1 = fsReadFile(hFile1, buffer1, 1024, &nLen1);
		nStatus2 = fsReadFile(hFile2, buffer2, 1024, &nLen2);

		GetJpegImage(_JpegImage, &nJpegLen, 0);
		
		if ((nStatus1 == ERR_FILE_EOF) && (nStatus2 == ERR_FILE_EOF))
		{
			sysprintf("\ncompare ok!\n");
			fsCloseFile(hFile1);
			fsCloseFile(hFile2);
			return 0;
		}
		
		if (nLen1 != nLen2)
			break;
			
		if (memcmp(buffer1, buffer2, 1024))
			break;
		
		nCount++;
		//if ((nCount % 1024) == 0)
		//	sysprintf("%d KB    \r", nCount);
	}
	
	sysprintf("\nFile Compare failed at offset %x\n", nCount * 1024);
	
	fsCloseFile(hFile1);
	fsCloseFile(hFile2);
	return -1;
}



INT  copy_file(CHAR *suSrcName, CHAR *szSrcAsciiName, 
					CHAR *suDstName, CHAR *szDstAsciiName)
{
	INT			hFileSrc, hFileDst, nByteCnt, nStatus;
	UINT8		*pucBuff, *szFNameSrc, *szFNameDst;
	UINT32		nJpegLen;

	pucBuff = (UINT8 *)malloc(4096 + MAX_FILE_NAME_LEN + 512);
	if (pucBuff == NULL)
		return ERR_NO_FREE_MEMORY;
		
	szFNameSrc = pucBuff + 4096;
	szFNameDst = szFNameSrc + MAX_FILE_NAME_LEN/2;

	hFileSrc = fsOpenFile(suSrcName, szSrcAsciiName, O_RDONLY);
	if (hFileSrc < 0)
	{
		free(pucBuff);
		return hFileSrc;
	}

	hFileDst = fsOpenFile(suDstName, szDstAsciiName, O_RDONLY);
	if (hFileDst > 0)
	{
		fsCloseFile(hFileSrc);
		fsCloseFile(hFileDst);
		free(pucBuff);
		return ERR_FILE_EXIST;
	}

	hFileDst = fsOpenFile(suDstName, szDstAsciiName, O_CREATE);
	if (hFileDst < 0)
	{
		fsCloseFile(hFileSrc);
		free(pucBuff);
		return hFileDst;
	}

   	while (1)
   	{
   		nStatus = fsReadFile(hFileSrc, pucBuff, 4096, &nByteCnt);
   		if (nStatus < 0)
   			break;

		nStatus = fsWriteFile(hFileDst, pucBuff, nByteCnt, &nByteCnt);
   		if (nStatus < 0)
   			break;

		GetJpegImage(_JpegImage, &nJpegLen, 0);
	}
	fsCloseFile(hFileSrc);
	fsCloseFile(hFileDst);
	free(pucBuff);

	if (nStatus == ERR_FILE_EOF)
		nStatus = 0;
	return nStatus;
}



INT Test()
{
	INT     nStatus;
	//CHAR	szSrcA[24] = "C:\\tape001.mpg";
	CHAR	szSrcA[24] = "C:\\1.mp4";
	CHAR	szDstA[24] = "C:\\copya";
	CHAR	suFileName1[64], suFileName2[64];
	UINT32	nJpegLen;

	while (1)
	{
		sysprintf("Delete file: %s\n", szDstA);
		fsAsciiToUnicode(szDstA, suFileName1, TRUE);
		nStatus = fsDeleteFile(suFileName1, NULL);
		if (nStatus < 0)
			sysprintf("Failed, status = %x\n", nStatus);

		while (_QueuedSize > 16*1024)
		{
			GetJpegImage(_JpegImage, &nJpegLen, 0);
			sysprintf(".");
		}

		sysprintf("Copy file: %s\n", szSrcA);
		fsAsciiToUnicode(szSrcA, suFileName1, TRUE);
		fsAsciiToUnicode(szDstA, suFileName2, TRUE);
		nStatus = copy_file(suFileName1, NULL, suFileName2, NULL);
		if (nStatus < 0)
		{
			sysprintf("Failed, status = %x\n", nStatus);
			exit(0);
		}

		sysprintf("Compare file: %s and %s\n", szSrcA, szDstA);
		fsAsciiToUnicode(szSrcA, suFileName1, TRUE);
		fsAsciiToUnicode(szDstA, suFileName2, TRUE);
		
		if (Action_Compare(suFileName1, NULL, suFileName2, NULL) < 0)
			break;
	}	
	return 0;
}


void  Isochronous_Test()
{
	CHAR    szFileName[32] = {'C',0,':',0,'\\',0,'1',0,0,0 };
	CHAR    suFileName[128];
	INT     nIdx = 0;
	UINT32	nJpegLen;
	INT    	hFile;
	INT     nWriteBytes;

	W99683Cam_Init();

	while (!W99683Cam_IsConnected())
#ifdef ECOS
		Hub_CheckIrqEvent(0);
#else	
		Hub_CheckIrqEvent();
#endif

	if (W99683Cam_Open() < 0)
	{
		sysprintf("Failed to open W99683 device!\n");
		return;           /* _W99683_Camera is freed also */
	}

	while (!W99683Cam_IsStreaming())
		;   

	/* Drop 5 pictures */
	for (nIdx = 0; nIdx < 10; nIdx++)
	//for (nIdx = 0; ; nIdx++)
	{
		sysprintf("%d GetJpegImage...\n", nIdx);
		GetJpegImage(_JpegImage, &nJpegLen, 0);
		sysprintf("ImageSize: %d, _QueuedSize: %d\n", nJpegLen, _QueuedSize);
	}
	
	//Test();
	
	//Action_WriteSpeedTest(szFileName, NULL);
	
	for (nIdx = 0; nIdx < 30; nIdx++)
	{
reget:		
		GetJpegImage(_JpegImage, &nJpegLen, 0);
		if (_QueuedSize > 200000)
		{
			goto reget;
		}
		sysprintf("ImageSize: %d, _QueuedSize: %d\n", nJpegLen, _QueuedSize);
		
		/* Open a new file for writing */
		sprintf(szFileName, "C:\\%04d.jpg", nIdx);
		fsAsciiToUnicode(szFileName, suFileName, 1);
		hFile = fsOpenFile(suFileName, NULL, O_CREATE);
		if (hFile > 0)
			sysprintf("Opene file:[%s], file handle:%d\n", szFileName, hFile);
		else
		{
			sysprintf("Failed to open file: %s (%x)\n", szFileName, hFile);
			continue;
		}
		if ((fsWriteFile(hFile, _JpegImage, nJpegLen, &nWriteBytes) < 0) ||
			(nWriteBytes != nJpegLen))
			sysprintf("File write error! %d %d\n", nWriteBytes);
		else
			sysprintf("%d bytes\n", nWriteBytes);
		fsCloseFile(hFile);
	}
	
	while (1)
		GetJpegImage(_JpegImage, &nJpegLen, 0);
}


void IntegrationTest(void)
{
	INT		t0;
	UINT32	uBlockSize, uFreeSize, uDiskSize;

	sysprintf("Please plug in pen driver, key board and W99683 camera in advance\n");

	InitUsbSystem();       
	UMAS_InitUmasDriver();
	USBKeyboardInit();
	/* wait hard disk ready */
	t0 = sysGetTicks(TIMER0);
	while (sysGetTicks(TIMER0) - t0 < 300)
		;
	
	if (fsDiskFreeSpace('C', &uBlockSize, &uFreeSize, &uDiskSize) == 0)
		sysprintf("Disk size = %d KB, Free speace = %d KB\n", uDiskSize, uFreeSize);
	else
		sysprintf("fsDiskFreeSpace failed!!\n");
	
	Isochronous_Test();
}
/* Pen driver Connect/disconnect  Test */
volatile UINT32 i32MsConnect = 0;
volatile UINT32 i32MsDisConnect = 1;
volatile UINT32 i32AccessDone = 0;
VOID MassStotrageConnection(void* umas)
{
	volatile i=0x20000;
	sysprintf("Umas driver connect  0x%x\n", (UINT32)umas);
	i32MsConnect = 1;
	i32MsDisConnect =0;
	while(i--);
	/* Stop Event check */
	//sysClearTimerEvent(TIMER0, usbTimerChanel);
}
VOID MassStotrageDisconnection(void* umas)
{
	sysprintf("Umas driver disconnect 0x%x\n", (UINT32)umas);

	i32MsConnect = 0;
	i32MsDisConnect = 1;
	i32AccessDone = 0;
	/* Restart Event Check */
	//usbTimerChanel = sysSetTimerEvent(TIMER0, 20, (PVOID)usbEvent);	/* 20 ticks/per sec */	
}





INT JpegDecTest (UINT32 u32BitStream, UINT32 u32BitstreamSize)
{	
	INT 	len;
	JPEG_INFO_T jpegInfo;
	
	/* JPEG Open */
	jpegOpen ();
	
	/* JPEG Init */
	jpegInit();
	
	/* Set Bit stream Address */   
    jpegIoctl(JPEG_IOCTL_SET_BITSTREAM_ADDR, u32BitStream, 0);
    
    
	if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PLANAR_YUV)
	{		
		UINT32 u32FrameBuffer; 			   	
		/* Allocate Raw Data Buffer for Decode Operation */	
		g_pu8DecFrameBuffer = (PUINT8)malloc(sizeof(CHAR) * PLANAR_DEC_BUFFER_SIZE);			
		
		if(g_pu8DecFrameBuffer == NULL)
		{
			sysprintf("\tCan't allocate the buffer for decode (size 0x%X)\n",PLANAR_DEC_BUFFER_SIZE);
			return FALSE;
		}
			
		u32FrameBuffer =  (UINT32) g_pu8DecFrameBuffer | 0x80000000;
		
		sysprintf("\tThe decoded data starts from 0x%X, Size is 0x%X\n", u32FrameBuffer,PLANAR_DEC_BUFFER_SIZE);
		
		/* Set Decoded Image Address (Only Can be set before Decode Trigger for Planar) */
		jpegIoctl(JPEG_IOCTL_SET_YADDR, u32FrameBuffer, 0);	
		   	
	}    
          
#ifdef __DECINPUTWAIT_TEST__   
	g_u32BitstreamSize = u32BitstreamSize;
    /* Set Decode Input Wait mode (Maximum size for the buffer is 2046 KB) */	
	jpegIoctl(JPEG_IOCTL_SET_DECINPUTWAIT_CALBACKFUN, (UINT32) JpegDecInputWait, BUFFERSIZE);
	
	if(u32BitstreamSize < BUFFERSIZE)
		len = u32BitstreamSize;
	else
		len = BUFFERSIZE;
	
	/* Prepare the data for first decode operation */	
	memcpy((char *)(u32BitStream),(char *)( (UINT32)g_pu8JpegBuffer | 0x80000000), len);		
	
	/* The bitstream size put to Bitstream Buffer */
	g_u32UsedSize += len; 
#endif          
          
    /* Decode mode */	
	jpegIoctl(JPEG_IOCTL_SET_DECODE_MODE, JPEG_DECODE_OUTPUT_FORMAT, 0);	
	
	/* Set JPEG Header Decode End Call Back Function */
	jpegIoctl(JPEG_IOCTL_SET_HEADERDECODE_CALBACKFUN, (UINT32) JpegDecHeaderComplete, 0);	

    /* Trigger JPEG decoder */
    jpegIoctl(JPEG_IOCTL_DECODE_TRIGGER, 0, 0);  
        
	/* Wait for complete */
	if(jpegWait())
	{
		strcpy(decodePath, "C:\\jpegDec_");
		
		if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PLANAR_YUV)	
			strcat(decodePath, "PLANAR_"); 	
		else
		{
			switch(JPEG_DECODE_OUTPUT_FORMAT)
			{
				case JPEG_DEC_PRIMARY_PACKET_YUV422:
					strcat(decodePath, "PACKET_YUV422_"); 	
					break;
				case JPEG_DEC_PRIMARY_PACKET_RGB555:
					strcat(decodePath, "PACKET_RGB555_"); 	
					break;
				case JPEG_DEC_PRIMARY_PACKET_RGB565:
					strcat(decodePath, "PACKET_RGB565_"); 
					break;	
				case JPEG_DEC_PRIMARY_PACKET_RGB888:
					strcat(decodePath, "PACKET_RGB888_"); 						
					break;		
			}		
		}
		jpegGetInfo(&jpegInfo);
		sysprintf("\tJPEG Decode Complete!!\n");
		sysprintf("\t\tJpeg YuvFormat ");
		switch(jpegInfo.yuvformat)
		{
			case JPEG_DEC_YUV420:
				sysprintf("YUV420\n");
				if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PLANAR_YUV)
					strcat(decodePath, "YUV420_"); 	
				break;
			case JPEG_DEC_YUV422:
				sysprintf("YUV422\n");
				if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PLANAR_YUV)
					strcat(decodePath, "YUV422_"); 	
				break;			
			case JPEG_DEC_YUV444:
				sysprintf("YUV444\n");
				if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PLANAR_YUV)
					strcat(decodePath, "YUV444_"); 	
				break;			
			case JPEG_DEC_YUV411:
				sysprintf("YUV411\n");
				if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PLANAR_YUV)
					strcat(decodePath, "YUV411_"); 	
				break;			
			case JPEG_DEC_GRAY:
				sysprintf("GRAY\n");
				if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PLANAR_YUV)				
					strcat(decodePath, "GRAY_"); 	
				break;			
			case JPEG_DEC_YUV422T:		
				sysprintf("YUV422T\n");
				if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PLANAR_YUV)
					strcat(decodePath, "YUV422T_"); 	
				break;			
		}
		sysprintf("\t\tJpeg Width = %d\n",jpegInfo.jpeg_width);			/* JPEG width in Bitstream header */
		sysprintf("\t\tJpeg Height = %d\n",jpegInfo.jpeg_height);		/* JPEG width in Bitstream header */	
		strcat(decodePath, "size");
#ifdef __PANEL_TEST__
	   	strcat(decodePath, intToStr(PANEL_WIDTH));
	   	strcat(decodePath, "x");
	   	strcat(decodePath, intToStr(PANEL_HEIGHT));		
		
#else		
	   	strcat(decodePath, intToStr(jpegInfo.width));
	   	strcat(decodePath, "x");
	   	strcat(decodePath, intToStr(jpegInfo.height));		
#endif		
		strcat(decodePath, ".dat");
		sysprintf("\t\tOutput Image Width = %d\n",jpegInfo.width);		/* Decode image width */
		sysprintf("\t\tOutput Image Height = %d\n",jpegInfo.height);	/* Decode image height */	
		
		if(jpegInfo.stride)
			sysprintf("\t\tJpeg Decode Image Stride = %d\n",jpegInfo.stride);			
			
		len = jpegInfo.image_size[0];
			
#ifdef __PANEL_TEST__		
		sysprintf ("\t\tOutput Raw data size (file) %d x %d\n", PANEL_WIDTH, PANEL_HEIGHT);
#else
		sysprintf ("\t\tOuput Raw data size (file) %d\n", len);		
#endif			
    }
    else
    {
    	sysprintf("\tJPEG Decode Error!!\n");
    	
    	len = 0;
    }

	jpegClose ();

	
	
	return len;
}
/*=====================================================================================
Test Condition:
	Plug in a pen driver to USBH port 1 or port 0. 



=====================================================================================*/
void PenDriverAccess(UINT32 u32Count)
{
	CHAR szFileName[32] = {'C',0,':',0,'\\',0,'1',0,0,0 };
	CHAR szAsciiStr[32]={0};
	CHAR suFileName[128];
	INT     nIdx = 0, nIdy=0;
	UINT32	nJpegLen= 256*1024;
	INT    	hFile;
	INT     nWriteBytes;
	
	INT encLen, decLen;
	INT nWriteLen, nStatus, nReadLen;

	CHAR	encodePath[50];
	CHAR	decodePath[50];
	CHAR	suFileName1[100];
	int volatile i;
	unsigned char *pSrc,*pDst;
	
	
	//load_png_image("C:\\bee.png");
	/*-----------------------------------------------------------------------*/
	/*  编码测试                                                        */
	/*-----------------------------------------------------------------------*/
	
	/*  Get Raw Data for Encode Operation (jpegEnc.dat) */
	sysprintf ("\nStart JPEG Encode Testing...\n");		
		
	strcpy(encodePath, "C:\\jpegEnc.dat");
	
	fsAsciiToUnicode(encodePath, suFileName1, TRUE);//将ASCII转换成Unicode
	
    hFile = fsOpenFile(suFileName1, NULL, O_RDONLY);//打开或创建文件
	if (hFile > 0)
		sysprintf("\tOpen file:[%s], file handle:%d\n", encodePath, hFile);
	else
	{
		sysprintf("\tFailed to open file: %s (%d)\n", encodePath, hFile);
		goto JPEG_DECODE;
	}	
		
	encLen = fsGetFileSize(hFile);//获取打开文件的大小
	
	sysprintf("\tRaw data size for Encode is %d\n", encLen);
	
	/* Allocate the Raw Data Buffer for Encode Operation */	
	g_pu8EncFrameBuffer = (PUINT8)malloc(sizeof(CHAR) * encLen);//向系统申请动态内存奉分配
	
#if 1
	nStatus = fsReadFile(hFile, (UINT8 *)((UINT32)g_pu8EncFrameBuffer | 0x80000000), encLen, &nReadLen);//读取打开的文件
#else
	nStatus = fsReadFile(hFile, (UINT8 *)((UINT32)g_pu8EncFrameBuffer), encLen, &nReadLen);
#endif
    
	if (nStatus < 0)
		sysprintf("\tRead error %d!!\n", nStatus);		
	
	fsCloseFile(hFile);

#if 1	
	encLen = JpegEncTest ((UINT32) g_pu8EncFrameBuffer, (UINT32) g_au8BitstreamBuffer | 0x80000000);
#else
	encLen = JpegEncTest ((UINT32) g_pu8EncFrameBuffer, (UINT32) g_au8BitstreamBuffer);
#endif
    
	if(encLen == 0)
		goto JPEG_DECODE;
		   
	/* Write to Disk */
	strcpy(encodePath, "C:\\jpegEnc");

#ifdef __UPSCALE_TEST__	
	strcat(encodePath, "o"); // "_upscale"
#endif

#ifdef 	__ENC_SOFTWARE_RESERVE__
	strcat(encodePath, "_reserved");
#endif
	
	strcat(encodePath, ".jpg");
	
	fsAsciiToUnicode(encodePath, suFileName1, TRUE);	   	   
	
	hFile = fsOpenFile(suFileName1, NULL, O_CREATE | O_TRUNC);
	if (hFile > 0)
		sysprintf("\tOpen file:[%s], file handle:%d\n", encodePath, hFile);
	else
	{
		sysprintf("\tFailed to open file: %s (%d)\n", encodePath, hFile);
	}
#if 1
	nStatus = fsWriteFile(hFile, (UINT8 *)((UINT32)g_au8BitstreamBuffer | 0x80000000), encLen, &nWriteLen);
#else
	nStatus = fsWriteFile(hFile, (UINT8 *)((UINT32)g_au8BitstreamBuffer), encLen, &nWriteLen);
#endif
	if (nStatus < 0)
		sysprintf("\tWrite error %d!!\n", nStatus);
	
	fsCloseFile(hFile);	
    
    free(g_pu8EncFrameBuffer);
    
	sysprintf ("Encode test completed!\n"); 
		
	/*-----------------------------------------------------------------------*/
	/*  解码测试                                                         */
	/*-----------------------------------------------------------------------*/
	
JPEG_DECODE:
	 sysprintf ("\nStart JPEG Decode Testing...\n");		   
    /*  Get Bitstream data for Decode Operation (jpegDec.jpg) */
		strcpy(decodePath, "C:\\jpegDec.jpg"); // 
	
		fsAsciiToUnicode(decodePath, suFileName2, TRUE);
	
    hFile = fsOpenFile(suFileName2, NULL, O_RDONLY);
	if (hFile > 0)
		sysprintf("\tOpen file:[%s], file handle:%d\n", decodePath, hFile);
	else
	{
		sysprintf("\tFailed to open file: %s (%d)\n", decodePath, hFile);
		return;
	}	
	
	decLen = fsGetFileSize(hFile);

//	
	sysprintf("\tBit stream  size for Decode is %d\n", decLen);
	
	/* Allocate the Bitstream Data Buffer for Decode Operation */	
	
	
	g_pu8JpegBuffer = (PUINT8)malloc(sizeof(CHAR) * decLen);
	WriteBuffer = (PUINT8)malloc(sizeof(CHAR) * decLen);
	ReadBuffer = (PUINT8)malloc(sizeof(CHAR) * decLen);
	
	
	pSrc = (UINT8 *)((UINT32)WriteBuffer | 0x80000000);
	pDst = (UINT8 *)((UINT32)ReadBuffer | 0x80000000);

#if 1    
	nStatus = fsReadFile(hFile, (UINT8 *)((UINT32)g_pu8JpegBuffer | 0x80000000), decLen, &nReadLen);
#else
	nStatus = fsReadFile(hFile, (UINT8 *)((UINT32)g_pu8JpegBuffer), decLen, &nReadLen);
#endif	
	if (nStatus < 0)
		sysprintf("\tRead error %d!!\n", nStatus);	
	
  	fsCloseFile(hFile);   
	
	#ifdef __DECINPUTWAIT_TEST__
	sysprintf ("\tDecode Input Test - %d KB Bitstream Buffer\n",BUFFERSIZE / 1024);
	sysprintf ("\t\tBitstream Buffer 0 0x%x \n",(UINT32)g_au8DecInputWaitBuffer | 0x80000000);
	sysprintf ("\t\tBitstream Buffer 1 0x%x \n",(UINT32)g_au8DecInputWaitBuffer | 0x80000000 + BUFFERSIZE/2);
			decLen = JpegDecTest ((UINT32) g_au8DecInputWaitBuffer | 0x80000000,nReadLen);  
#else   
	sysprintf ("\tg_au8JpegBuffer Buffer 0x%x\n",(UINT32)g_pu8JpegBuffer);

#if 1
	decLen = JpegDecTest ((UINT32) g_pu8JpegBuffer | 0x80000000,0);  
#else
	decLen = JpegDecTest ((UINT32) g_pu8JpegBuffer,0);  
#endif
#endif	
	if(decLen == 0)
		return;
	
#ifdef __PANEL_TEST__	
	if(JPEG_DECODE_OUTPUT_FORMAT == JPEG_DEC_PRIMARY_PACKET_RGB888)
		decLen = PANEL_WIDTH * PANEL_HEIGHT * 4;
	else
		decLen = PANEL_WIDTH * PANEL_HEIGHT * 2;		
#endif	

#ifdef __JPEG_DEC_OUTPUT_YUV422__
    memset(decodePath, 0, sizeof(decodePath));
    strncpy(decodePath, "0:\dec422o.yuv", strlen("0:\dec422o.yuv"));
#endif
	

	
	sysprintf ("Decode test completed!\n");
	

}	
void PenDriverConnectTest(void)
{
	UINT32	uBlockSize, uFreeSize, uDiskSize;
	
	umass_register_connect(MassStotrageConnection);	//连接注册存储设备，插入回掉
	umass_register_disconnect(MassStotrageDisconnection);	//连接断开，拔出回掉
	InitUsbSystem();//初始化usb硬盘，扫描设备
	UMAS_InitUmasDriver();//初始化usb存储驱动程序
	sysDelay(30);			/* Delay 300 ms */
	while(1){												
		if(i32MsConnect ==1){	
			if(i32AccessDone==0){
				if (fsDiskFreeSpace('E', &uBlockSize, &uFreeSize, &uDiskSize) == 0){
					sysprintf("Disk size = %d KB, Free speace = %d KB\n", uDiskSize, uFreeSize);
				}else{
					sysprintf("fsDiskFreeSpace failed!!\n");							
					sysprintf("Pen driver may plug out !\n");	
				}
				PenDriverAccess(1);	
				i32AccessDone = 1;	
				sysprintf("Please plug out the pen driver for test\n");					
			}
		}
		if(i32MsDisConnect == 0)
			;
		if(i32MsConnect == 0)
			sysprintf("Please plug in the pen driver for test\n");
//			sysprintf("\tRead and Compare SpiFlash\n");
//			spiFlashRead(0, (sizeof(decodePath) / sizeof(decodePath[0])), (UINT32 *)decodePath);//将Flash中的数据读取到pDst中
//			PenDriverAccess(1);
			
		Hub_CheckIrqEvent();
		
	}	
	
}

//void FormatRamDisk(void)
//{
//	PDISK_T       *pDiskList, *ptPDiskPtr;
//	PARTITION_T   *ptPartition;
//	INT           nDiskIdx = 0;
//	INT           nPartIdx;
//	ptPDiskPtr = pDiskList = fsGetFullDiskInfomation();
//	while (ptPDiskPtr != NULL)
//	{
//		printf("\n\n=== Disk %d (%s) ======================\n", 
//		nDiskIdx++, (ptPDiskPtr->nDiskType & 
//		DISK_TYPE_USB_DEVICE) ? "USB" : "IDE");
//		printf("    name:     [%s%s]\n", ptPDiskPtr->szManufacture, 
//		ptPDiskPtr->szProduct);
//		printf("    head:     [%d]\n", ptPDiskPtr->nHeadNum);
//		printf("    sector:   [%d]\n", ptPDiskPtr->nSectorNum);
//		printf("    cylinder: [%d]\n", ptPDiskPtr->nCylinderNum);
//		printf("    size:     [%d MB]\n", ptPDiskPtr->uDiskSize / 1024);
//				
//		ptPartition = ptPDiskPtr->ptPartList;
//		nPartIdx = 1;
//		while (ptPartition != NULL)
//		{
//			printf("\n    --- Partition %d --------------------\n", 
//			nPartIdx++);
//			printf("        active: [%s]\n", 
//			(ptPartition->ucState & 0x80) ? "Yes" : "No");
//			printf("        size:   [%d MB]\n", 
//			(ptPartition->uTotalSecN / 1024) / 2);
//			printf("        start:  [%d]\n", ptPartition->uStartSecN);
//			printf("        type:   ");
//			ptPartition = ptPartition->ptNextPart;
//		}
//		ptPDiskPtr = ptPDiskPtr->ptPDiskAllLink;
//		fsFormatFlashMemoryCard(ptPDiskPtr);
//	}	
//	fsReleaseDiskInformation(pDiskList);  
//}


INT main()
{
	UINT32 	u32Item, u32ExtFreq;
	UINT32 	u32PllOutKHz;
	WB_UART_T uart;//串口
#ifndef ECOS
	/* 初始化 UART */
	u32ExtFreq = sysGetExternalClock();//获取外部时钟

	sysSetTimerReferenceClock (TIMER0, u32ExtFreq*1000);
	sysStartTimer(TIMER0, 100, PERIODIC_MODE);
	/*清理缓冲*/
	sysDisableCache(); 	
	sysFlushCache(I_D_CACHE);		
	sysEnableCache(CACHE_WRITE_BACK);
		
	sysUartPort(1);
	uart.uiFreq = u32ExtFreq*1000;	//use APB clock
    	uart.uiBaudrate = 115200;
    	uart.uiDataBits = WB_DATA_BITS_8;
    	uart.uiStopBits = WB_STOP_BITS_1;
    	uart.uiParity = WB_PARITY_NONE;
    	uart.uiRxTriggerLevel = LEVEL_1_BYTE;
    	sysInitializeUART(&uart);    	    	

	sysSetSystemClock(eSYS_UPLL, 	//E_SYS_SRC_CLK eSrcClk,	
					192000,		//UINT32 u32PllKHz, 	
					192000,		//UINT32 u32SysKHz,
					192000,		//UINT32 u32CpuKHz,
					  192000/2,		//UINT32 u32HclkKHz,
					  192000/4);		//UINT32 u32ApbKHz
	u32PllOutKHz = sysGetPLLOutputKhz(eSYS_UPLL, u32ExtFreq);
	sysprintf("PLL out frequency %d Khz\n", u32PllOutKHz);		
	
	
	outp32(REG_AHBCLK, inp32(REG_AHBCLK) & ~USBD_CKE);
	outp32(REG_APBCLK, 0xFFFFFFFF);	
#endif
	initVPost4test();//初始化显示屏
//	fsAssignDriveNumber('D', DISK_TYPE_USB_DEVICE, 0, 1);//驱动器号分配
//	fsAssignDriveNumber('D', DISK_TYPE_HARD_DISK, 0, 1);//驱动器号分配
	
	fsInitFileSystem();//初始化文件系统
	
	fsAssignDriveNumber('D', DISK_TYPE_USB_DEVICE, 0, 1);//驱动器号分配
	fsAssignDriveNumber('E', DISK_TYPE_HARD_DISK | DISK_TYPE_DMA_MODE, 0, 1);//驱动器号分配



//		spiFlashInit();

//	InitSpiFlashDisk((UINT32)&i8SFlashDisk, SPIFLASH_DISK_SIZE)	;
//	FormatRamDisk();
//	fsSetVolumeLabel('E', "SPIFLASH", strlen("SPIFLASH")); 

  /*-----------------------------------------------------------------------*/
	/*  Init SD card                                                         */
	/*-----------------------------------------------------------------------*/
//	sicIoctl(SIC_SET_CLOCK, 200000, 0, 0);
//	sicOpen();
//	sysprintf("total sectors (%x)\n", sicSdOpen0());
//	
//	spuOpen(eDRVSPU_FREQ_8000);//设置音频时钟
//	spuDacOn(1);

		
	USB_PortInit(HOST_LIKE_PORT1);//初始化usb文件端口
	PenDriverConnectTest();	//读取文件

	return 0;
}

