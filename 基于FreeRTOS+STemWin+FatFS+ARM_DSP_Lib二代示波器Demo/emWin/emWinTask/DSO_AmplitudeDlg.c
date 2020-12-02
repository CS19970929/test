/*
*********************************************************************************************************
*	                                  
*	ģ������ : ʾ�����������ϵķ�ֵ����
*	�ļ����� : DSO_AmplititudeDlg.c
*	��    �� : V1.0
*	˵    �� : ��ʾʾ������ǰ�ķ�ֵ��
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
static GUI_MEMDEV_Handle   hMemAmpWindow;


/*
*********************************************************************************************************
*				                      �궨��
*********************************************************************************************************
*/
#define ID_TEXT_0 		(GUI_ID_USER + 0x00)
#define ID_TEXT_1  		(GUI_ID_USER + 0x01)
#define ID_TEXT_2 		(GUI_ID_USER + 0x02)
#define ID_TEXT_3 		(GUI_ID_USER + 0x03)


/*
*********************************************************************************************************
*	                   GUI_WIDGET_CREATE_INFO��������
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreateAmp[] = 
{
    { WINDOW_CreateIndirect,     "",                  0,   2,  444,135,35,0},
	{ TEXT_CreateIndirect,      "Amp",    ID_TEXT_0,  35,  2,  35, 16, 0, 0},
	{ TEXT_CreateIndirect,      "1.00V",  ID_TEXT_1,  75,  2,  50, 16, 0, 0},
	
	{ TEXT_CreateIndirect,      "Amp",    ID_TEXT_2,  35,  17, 35, 16, 0, 0},
	{ TEXT_CreateIndirect,      "1.00V",  ID_TEXT_3,  75,  17, 50, 16, 0, 0},
};

/*
*********************************************************************************************************
*	�� �� ��: PaintDialogAmp
*	����˵��: ��ֵ�����ػ���Ϣ�еĺ���
*	��    �Σ�pMsg  ָ���ַ      	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void PaintDialogAmp(WM_MESSAGE * pMsg)
{
	/* �屳��ɫ */
	GUI_SetBkColor(0x905040);
  	GUI_Clear();
	
	/* �������Ŀ����Բ�Ǿ��� */
	GUI_SetColor(GUI_BLACK);
	GUI_AA_FillRoundedRect(0, 0, 134, 34, 10);
	
	/* ���ƿ����Բ�Ǿ��� */
	GUI_SetColor(0XEBCD9E);
	GUI_SetPenSize(2);
	GUI_AA_DrawRoundedRect(0, 0, 134, 34, 10);
	
	/* ���ƿ����Բ�Ǿ��Σ�������ֵ1 */
	GUI_SetColor(GUI_YELLOW);
	GUI_AA_FillRoundedRect(10, 4, 30, 16, 3);
	
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('1', 16, 3);

	/* ���ƿ����Բ�Ǿ��Σ�������ֵ2 */
	GUI_SetColor(GUI_GREEN);
	GUI_AA_FillRoundedRect(10, 19, 30, 31, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('2', 16, 18);
}

/*
*********************************************************************************************************
*	�� �� ��: PaintDialogAmp
*	����˵��: ��ֵ���ڳ�ʼ������Ϣ����
*	��    �Σ�pMsg  ָ���ַ      	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void InitDialogAmp(WM_MESSAGE * pMsg)
{
	WM_HWIN hWin = pMsg->hWin;

	//
	// ��ʼ���ı��ؼ�
	//
	TEXT_SetTextColor(WM_GetDialogItem(hWin, ID_TEXT_0), 0x00ffff);
	TEXT_SetFont(WM_GetDialogItem(hWin, ID_TEXT_0), &GUI_Font16_1);

	TEXT_SetTextColor(WM_GetDialogItem(hWin, ID_TEXT_1), 0x00ffff);
	TEXT_SetFont(WM_GetDialogItem(hWin, ID_TEXT_1), &GUI_Font16_1);
	
	TEXT_SetTextColor(WM_GetDialogItem(hWin, ID_TEXT_2), GUI_GREEN);
	TEXT_SetFont(WM_GetDialogItem(hWin, ID_TEXT_2), &GUI_Font16_1);

	TEXT_SetTextColor(WM_GetDialogItem(hWin, ID_TEXT_3), GUI_GREEN);
	TEXT_SetFont(WM_GetDialogItem(hWin, ID_TEXT_3), &GUI_Font16_1);
}

/*
*********************************************************************************************************
*	�� �� ��: _cbCallbackAmp()
*	����˵��: ��ֵ���ڻص�����
*	��    ��: pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbCallbackAmp(WM_MESSAGE * pMsg) 
{
    WM_HWIN hWin = pMsg->hWin;
	
    switch (pMsg->MsgId) 
    {
        case WM_TextUpDate:
			TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_1), g_AttText[Ch1AmpId]);
			TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_3), g_AttText[Ch1AmpId]);
			break;
			
		case WM_PAINT:
			GUI_MEMDEV_WriteAt(hMemAmpWindow, 2, 444);
            break;
		
        case WM_INIT_DIALOG:
            InitDialogAmp(pMsg);
            break;
		
        default:
            WM_DefaultProc(pMsg);
    }
}

/*
*********************************************************************************************************
*	�� �� ��: CreateWindowAmplitude
*	����˵��: ������ֵ����
*	��    ��: ��         	
*	�� �� ֵ: ���������ڵľ��
*********************************************************************************************************
*/
WM_HWIN CreateAmplitudeDlg(void) 
{
	WM_HWIN hWin;

	hWin = GUI_CreateDialogBox(_aDialogCreateAmp, 
	                           GUI_COUNTOF(_aDialogCreateAmp), 
	                           &_cbCallbackAmp, 
	                           0, 
	                           0, 
	                           0);
	
	return hWin;
}

/*
*********************************************************************************************************
*	�� �� ��: DrawWinAmpBk
*	����˵��: �����ڱ������Ƶ��洢�豸����
*	��    ��: ��         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DrawWinAmpBk(void) 
{
	hMemAmpWindow = GUI_MEMDEV_CreateFixed(0, 
										   0, 
									       135, 
									       35, 
									       GUI_MEMDEV_HASTRANS, 
										   GUI_MEMDEV_APILIST_16, 
										   GUICC_M565);
	GUI_MEMDEV_Select(hMemAmpWindow);
	PaintDialogAmp(NULL);
	GUI_MEMDEV_Select(0);
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
