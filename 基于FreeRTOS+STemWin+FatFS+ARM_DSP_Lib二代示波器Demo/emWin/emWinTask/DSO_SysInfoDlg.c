/*
*********************************************************************************************************
*	                                  
*	ģ������ : ������ϵͳ��Ϣ�Ի���
*	�ļ����� : DSO_SysInfoDlg.c
*	��    �� : V1.0
*	˵    �� : ��ʾCPU�����ʣ�ͨ�������ͨ�������ѹ��Χ��
*	�޸ļ�¼ :
*		�汾��    ����          ����           ˵��
*		V1.0    2018-01-06     Eric2013        �׷� 
*                                           
*	Copyright (C), 2018-2028, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "includes.h"
#include "MainTask.h"



/*
*********************************************************************************************************
*	                                  ����
*********************************************************************************************************
*/
static GUI_MEMDEV_Handle   hMemSysDlg;


/*
*********************************************************************************************************
*				                      �궨��
*********************************************************************************************************
*/
#define ID_TEXT_0 		(GUI_ID_USER + 0x00)
#define ID_TEXT_1  		(GUI_ID_USER + 0x01)
#define ID_TEXT_2 		(GUI_ID_USER + 0x02)
#define ID_TEXT_3 		(GUI_ID_USER + 0x03)
#define ID_TEXT_4 		(GUI_ID_USER + 0x04)


/*
*********************************************************************************************************
*					GUI_WIDGET_CREATE_INFO��������
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreateSysInfo[] = 
{ 
    { WINDOW_CreateIndirect,     "",       0,            663,  	287,  135,  81, 0},
	{ TEXT_CreateIndirect,      "CPU: 00.00%",      ID_TEXT_0,  35,   2, 100, 16, 0, 0},
	{ TEXT_CreateIndirect,      "CH1: DC 1:1",  	ID_TEXT_1,  35,  17, 100, 16, 0, 0},
	{ TEXT_CreateIndirect,      "CH2: DC 1:1",    	ID_TEXT_2,  35,  32, 100, 16, 0, 0},
	{ TEXT_CreateIndirect,      "Input1: 0V-3.3V",  ID_TEXT_3,  35,  47, 100, 16, 0, 0},
	{ TEXT_CreateIndirect,      "Input2: 0V-3.3V",  ID_TEXT_4,  35,  62, 100, 16, 0, 0},
};

/*
*********************************************************************************************************
*	�� �� ��: PaintDialogSysInfo()
*	����˵��: ʱ�����ڵĻص������ػ�
*	��    ��: pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void PaintDialogSysInfo(WM_MESSAGE * pMsg)
{

	GUI_SetBkColor(0x905040);
	GUI_Clear();
	GUI_SetColor(GUI_BLACK);
	
	GUI_AA_FillRoundedRect(0, 0, 135-1, 81-1, 10);
	GUI_SetColor(0XEBCD9E);
	GUI_SetPenSize(2);
	
	GUI_AA_DrawRoundedRect(0, 0, 135-1, 81-1, 10);
	
	GUI_SetColor(GUI_GREEN);
	GUI_AA_FillRoundedRect(10, 4, 30, 16, 3);
	

	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('C', 16, 3);
	
	GUI_SetColor(GUI_YELLOW);
	GUI_AA_FillRoundedRect(10, 19, 30, 31, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('1', 16, 18);
	
	GUI_SetColor(GUI_GREEN);
	GUI_AA_FillRoundedRect(10, 34, 30, 47, 3);
	
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('2', 16, 32);
	
	GUI_SetColor(GUI_YELLOW);
	GUI_AA_FillRoundedRect(10, 50, 30, 62, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('1', 16, 49);
	
	GUI_SetColor(GUI_GREEN);
	GUI_AA_FillRoundedRect(10, 65, 30, 77, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('2', 16, 64);
}

/*
*********************************************************************************************************
*	�� �� ��: InitDialogSysInfo
*	����˵��: ʱ�����ڵĻص������ĳ�ʼ��
*	��    �Σ�pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void InitDialogSysInfo(WM_MESSAGE * pMsg)
{
    WM_HWIN hWin = pMsg->hWin;

    //
    // ��ʼ���ı��ؼ�
    //
    TEXT_SetTextColor(WM_GetDialogItem(hWin, ID_TEXT_0), GUI_GREEN);
    TEXT_SetFont(WM_GetDialogItem(hWin, ID_TEXT_0), &GUI_Font16_1);
	
	TEXT_SetTextColor(WM_GetDialogItem(hWin, ID_TEXT_1), 0x00ffff);
    TEXT_SetFont(WM_GetDialogItem(hWin, ID_TEXT_1), &GUI_Font16_1);
	
	TEXT_SetTextColor(WM_GetDialogItem(hWin, ID_TEXT_2), GUI_GREEN);
    TEXT_SetFont(WM_GetDialogItem(hWin, ID_TEXT_2), &GUI_Font16_1);
	
	TEXT_SetTextColor(WM_GetDialogItem(hWin, ID_TEXT_3), 0x00ffff);
    TEXT_SetFont(WM_GetDialogItem(hWin, ID_TEXT_3), &GUI_Font16_1);

	TEXT_SetTextColor(WM_GetDialogItem(hWin, ID_TEXT_4), GUI_GREEN);
    TEXT_SetFont(WM_GetDialogItem(hWin, ID_TEXT_4),&GUI_Font16_1);

	/* ������ʱ�� */
	WM_CreateTimer(hWin,   /* ������Ϣ�Ĵ��ڵľ�� */
				   0, 	   /* �û������Id���������ͬһ����ʹ�ö����ʱ������ֵ��������Ϊ�㡣 */
				   1000,   /* ���ڣ������ڹ���ָ������Ӧ�յ���Ϣ*/
				   0);	   /* ��������ʹ�ã�ӦΪ0 */	
}

/*
*********************************************************************************************************
*	�� �� ��: _cbCallbackSysInfo()
*	����˵��: ʱ�����ڵĻص�����
*	��    ��: pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbCallbackSysInfo(WM_MESSAGE * pMsg) 
{
    WM_HWIN hWin = pMsg->hWin;
	char buf[40];
	
    switch (pMsg->MsgId) 
    {
        case WM_INIT_DIALOG:
            InitDialogSysInfo(pMsg);
            break;
		
		case WM_TIMER:
			sprintf(buf, "CPU: %d%%", osGetCPUUsage());
			TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_0),  buf);
		
			sprintf(buf, "CH1: DC 1:1");
			TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_1),  buf);
			
			sprintf(buf, "CH2: DC 1:1");
			TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_2),  buf);
			
			sprintf(buf, "Input1: 0V-3.3V");
			TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_3),  buf);
			
			sprintf(buf, "Input2: 0V-3.3V");
			TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_4),  buf);
			WM_RestartTimer(pMsg->Data.v, 1000);
			break;
		 
		case WM_PAINT:
			GUI_MEMDEV_WriteAt(hMemSysDlg, 663, 287);
            break;
			
        default:
            WM_DefaultProc(pMsg);
    }
}

/*
*********************************************************************************************************
*	�� �� ��: CreateSysInfoDlg
*	����˵��: ϵͳ��Ϣ�Ի���
*	��    ��: ��   	
*	�� �� ֵ: ���ھ��
*********************************************************************************************************
*/
WM_HWIN CreateSysInfoDlg(void) 
{
	WM_HWIN hWin;

	hWin = GUI_CreateDialogBox(_aDialogCreateSysInfo, 
	                           GUI_COUNTOF(_aDialogCreateSysInfo), 
	                           &_cbCallbackSysInfo,
                               0, 
	                           0, 
	                           0);
	return hWin;
}

/*
*********************************************************************************************************
*	�� �� ��: DrawWinSysBk
*	����˵��: �����ڱ������Ƶ��洢�豸����
*	��    ��: pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DrawWinSysBk(void) 
{
	hMemSysDlg = GUI_MEMDEV_CreateFixed(0, 
										0, 
									    135, 
									    81, 
									    GUI_MEMDEV_HASTRANS, 
										GUI_MEMDEV_APILIST_16, 
										GUICC_M565);
	GUI_MEMDEV_Select(hMemSysDlg);
	PaintDialogSysInfo(NULL);
	GUI_MEMDEV_Select(0);
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
