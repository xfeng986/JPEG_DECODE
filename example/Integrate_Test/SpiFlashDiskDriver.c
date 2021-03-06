#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifdef W90N740
#include "supports.h"
#else
#include "wbio.h"
#endif
#include "nvtfat.h"

UINT32 _SpiFlashDiskBase;


static INT  ram_disk_init(PDISK_T *ptPDisk)
{
	return 0;
}


static INT  ram_disk_ioctl(PDISK_T *ptPDisk, INT control, VOID *param)
{
	return 0;
}


static INT  ram_disk_read(PDISK_T *ptPDisk, UINT32 uSecNo, 
								INT nSecCnt, UINT8 *pucBuff)
{
	memcpy(pucBuff, (UINT8 *)(_SpiFlashDiskBase + uSecNo * 512), nSecCnt * 512);
	return FS_OK;
}


static INT  ram_disk_write(PDISK_T *ptPDisk, UINT32 uSecNo, 
								INT nSecCnt, UINT8 *pucBuff, BOOL bWait)
{
	memcpy((UINT8 *)(_SpiFlashDiskBase + uSecNo * 512), pucBuff, nSecCnt * 512);
	return FS_OK;
}


STORAGE_DRIVER_T  _SpiFlashDiskDriver = 
{
	ram_disk_init,
	ram_disk_read,
	ram_disk_write,
	ram_disk_ioctl,
};


INT32  InitSpiFlashDisk(UINT32 uStartAddr, UINT32 uDiskSize)
{
	PDISK_T		*ptPDisk;
	
	_SpiFlashDiskBase = uStartAddr;
	ptPDisk = malloc(sizeof(PDISK_T));

	memset(ptPDisk, 0, sizeof(PDISK_T));
	memcpy(&ptPDisk->szManufacture,"NUVOTON ",sizeof("NUVOTON"));
	memcpy(&ptPDisk->szProduct,"SPIFLASHDISK",sizeof("SPIFLASHRAMDISK"));
		
	ptPDisk->nDiskType = DISK_TYPE_HARD_DISK | DISK_TYPE_DMA_MODE;
	ptPDisk->uTotalSectorN = uDiskSize / 512;
	ptPDisk->nSectorSize = 512;
	ptPDisk->uDiskSize = uDiskSize;
	ptPDisk->ptDriver = &_SpiFlashDiskDriver;
	fsPhysicalDiskConnected(ptPDisk);
	return 0;
}

