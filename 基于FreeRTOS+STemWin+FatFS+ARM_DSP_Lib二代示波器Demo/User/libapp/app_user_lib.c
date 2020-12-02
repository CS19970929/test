/*
*********************************************************************************************************
*	                                  
*	ģ������ : �û����ļ�
*	�ļ����� : app_user_lib.c
*	��    �� : V1.0
*	˵    �� : ��ǰ��Ҫ��FatFS�����Թ��ⲿ�ļ����á�
*	�޸ļ�¼ :
*		�汾��    ����          ����         ˵��
*		V1.0    2016-12-31    Eric2013       �׷�
*
*********************************************************************************************************
*/
#include "includes.h"


/*
*********************************************************************************************************
*	                                  ���ڱ��ļ��ĵ���
*********************************************************************************************************
*/
#if 0
	#define printf_libdbg printf
#else
	#define printf_libdbg(...)
#endif

/*
*********************************************************************************************************
*	                                         ����SD��
*********************************************************************************************************
*/
FRESULT result;
FIL file;   /* ר������ͼƬ�ı��� */
UINT bw;
FATFS fs;
	
/* FatFs API�ķ���ֵ */
static const char * FR_Table[]= 
{
	"FR_OK���ɹ�",				                             /* (0) Succeeded */
	"FR_DISK_ERR���ײ�Ӳ������",			                 /* (1) A hard error occurred in the low level disk I/O layer */
	"FR_INT_ERR������ʧ��",				                     /* (2) Assertion failed */
	"FR_NOT_READY����������û�й���",			             /* (3) The physical drive cannot work */
	"FR_NO_FILE���ļ�������",				                 /* (4) Could not find the file */
	"FR_NO_PATH��·��������",				                 /* (5) Could not find the path */
	"FR_INVALID_NAME����Ч�ļ���",		                     /* (6) The path name format is invalid */
	"FR_DENIED�����ڽ�ֹ���ʻ���Ŀ¼�������ʱ��ܾ�",         /* (7) Access denied due to prohibited access or directory full */
	"FR_EXIST���ļ��Ѿ�����",			                     /* (8) Access denied due to prohibited access */
	"FR_INVALID_OBJECT���ļ�����Ŀ¼������Ч",		         /* (9) The file/directory object is invalid */
	"FR_WRITE_PROTECTED������������д����",		             /* (10) The physical drive is write protected */
	"FR_INVALID_DRIVE���߼���������Ч",		                 /* (11) The logical drive number is invalid */
	"FR_NOT_ENABLED�������޹�����",			                 /* (12) The volume has no work area */
	"FR_NO_FILESYSTEM��û����Ч��FAT��",		             /* (13) There is no valid FAT volume */
	"FR_MKFS_ABORTED�����ڲ�������f_mkfs()����ֹ",	         /* (14) The f_mkfs() aborted due to any parameter error */
	"FR_TIMEOUT���ڹ涨��ʱ�����޷���÷��ʾ�����",		 /* (15) Could not get a grant to access the volume within defined period */
	"FR_LOCKED�������ļ�������Բ������ܾ�",				 /* (16) The operation is rejected according to the file sharing policy */
	"FR_NOT_ENOUGH_CORE���޷����䳤�ļ���������",		     /* (17) LFN working buffer could not be allocated */
	"FR_TOO_MANY_OPEN_FILES����ǰ�򿪵��ļ�������_FS_SHARE", /* (18) Number of open files > _FS_SHARE */
	"FR_INVALID_PARAMETER��������Ч"	                     /* (19) Given parameter is invalid */
};


/*
*********************************************************************************************************
*	�� �� ��: MountSD
*	����˵��: SD���Ĺ��ء�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void MountSD(void)
{
	result = f_mount(&fs, "0:/", 0); 	/* �����ļ�ϵͳ */

	if (result != FR_OK)
	{
		/* �������ʧ�ܣ���ز�Ҫ�ٵ���FlashFS������API��������ֹ����Ӳ���쳣 */
		printf("�����ļ�ϵͳʧ�� (%s)\r\n", FR_Table[result]);
	}
	else
	{
		printf_libdbg("�����ļ�ϵͳ�ɹ� (%s)\r\n", FR_Table[result]);
	}
	
	printf_libdbg("------------------------------------------------------------------\r\n");
}

/*
*********************************************************************************************************
*	�� �� ��: UnmountSD
*	����˵��: ж��SD��
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void UnmountSD(void)
{
	result = f_mount(NULL, "0:/", 0); 	/* �����ļ�ϵͳ */

	if (result != FR_OK)
	{
		/* ���ж��ʧ�� */
		printf("ж���ļ�ϵͳʧ�� (%s)\r\n", FR_Table[result]);
	}
	else
	{
		printf_libdbg("ж���ļ�ϵͳ�ɹ� (%s)\r\n", FR_Table[result]);
	}
	
	printf_libdbg("------------------------------------------------------------------\r\n");
}

/*
*********************************************************************************************************
*	�� �� ��: _WriteByte2File()
*	����˵��: д�ļ���SD�����������洢����
*	��    �Σ�Data Ҫд�����ݣ� p ָ��FIL���ͱ���      	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void _WriteByte2File(U8 Data, void * p) 
{
	result = f_write (p, &Data, 1, &bw);
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
