/*
*********************************************************************************************************
*	                                  
*	ģ������ : ʾ�����������ϵ�״̬����
*	�ļ����� : DSO_Status1Dlg.c
*	��    �� : V1.0
*	˵    �� : ʾ�����������ϵ�״̬���ڣ�������ʾ���ֵ�����ֵ��ƽ��ֵ����Сֵ��4��
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
static GUI_MEMDEV_Handle   hMemStatus1Dlg;


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
#define ID_TEXT_5 		(GUI_ID_USER + 0x05)
#define ID_TEXT_6 		(GUI_ID_USER + 0x06)
#define ID_TEXT_7 		(GUI_ID_USER + 0x07)
#define ID_TEXT_8 		(GUI_ID_USER + 0x08)
#define ID_TEXT_9 		(GUI_ID_USER + 0x09)
#define ID_TEXT_10  	(GUI_ID_USER + 0x0A)
#define ID_TEXT_11 		(GUI_ID_USER + 0x0B)
#define ID_TEXT_12 		(GUI_ID_USER + 0x0C)

#define ID_TEXT_13 		(GUI_ID_USER + 0x0D)
#define ID_TEXT_14  	(GUI_ID_USER + 0x0E)
#define ID_TEXT_15 		(GUI_ID_USER + 0x0F)
#define ID_TEXT_16 		(GUI_ID_USER + 0x10)


/*
*********************************************************************************************************
*					GUI_WIDGET_CREATE_INFO��������
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreateStatus1[] = 
{
    { WINDOW_CreateIndirect,     "",     0,          145,  444, 510, 35, 0},
	
	/* ��һ�� */
	{ TEXT_CreateIndirect,    "Mean",    ID_TEXT_0,   35,  2, 40, 16, 0, 0},
	{ TEXT_CreateIndirect,    "0.000V",  ID_TEXT_1,   80,  2, 50, 16, 0, 0},
	
	{ TEXT_CreateIndirect,    "Pk-Pk",   ID_TEXT_2,   160, 2, 40, 16, 0, 0},
	{ TEXT_CreateIndirect,    "0.000V",  ID_TEXT_3,   205, 2, 50, 16, 0, 0},
	
	{ TEXT_CreateIndirect,    "Min",     ID_TEXT_4,   285, 2, 40, 16, 0, 0},
	{ TEXT_CreateIndirect,    "0.000V",  ID_TEXT_5,   330, 2, 50, 16, 0, 0},	

	{ TEXT_CreateIndirect,    "Max",     ID_TEXT_6,   410, 2, 40, 16, 0, 0},
	{ TEXT_CreateIndirect,    "0.000V",  ID_TEXT_7,   455, 2, 50, 16, 0, 0},		
	
	/* �ڶ��� */		
    { TEXT_CreateIndirect,    "Mean",    ID_TEXT_8,    35, 17, 40, 16, 0, 0},
    { TEXT_CreateIndirect,    "0.000V",  ID_TEXT_9,    80, 17, 50, 16, 0, 0},	

    { TEXT_CreateIndirect,    "Pk-Pk",   ID_TEXT_10,  160, 17, 40, 16, 0, 0},
    { TEXT_CreateIndirect,    "0.000V",  ID_TEXT_11,  205, 17, 50, 16, 0, 0},	

	{ TEXT_CreateIndirect,    "Min",     ID_TEXT_12,  285, 17, 40, 16, 0, 0},
	{ TEXT_CreateIndirect,    "0.000V",  ID_TEXT_13,  330, 17, 50, 16, 0, 0},

	{ TEXT_CreateIndirect,    "Max",     ID_TEXT_14,  410, 17, 40, 16, 0, 0},
	{ TEXT_CreateIndirect,    "0.000V",  ID_TEXT_15,  455, 17, 50, 16, 0, 0},
};

/*
*********************************************************************************************************
*	�� �� ��: PaintDialogStatus1
*	����˵��: ״̬���ڵĻص������ػ���Ϣ
*	��    �Σ�pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void PaintDialogStatus1(WM_MESSAGE * pMsg)
{
	/* �屳��ɫ */
	GUI_SetBkColor(0x905040);
  	GUI_Clear();
	
	/* �������Ŀ����Բ�Ǿ��� */
	GUI_SetColor(GUI_BLACK);
	GUI_AA_FillRoundedRect(0, 0, 509, 34, 10);
	
	/* ���ƿ����Բ�Ǿ��� */
	GUI_SetColor(0XEBCD9E);
	GUI_SetPenSize(2);
	GUI_AA_DrawRoundedRect(0, 0, 509, 34, 10);
	
	/* ���ƿ����Բ�Ǿ��Σ�������ֵ1 */
	GUI_SetColor(GUI_YELLOW);
	GUI_AA_FillRoundedRect(10, 4, 30, 16, 3);
	
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('1', 16, 3);

	/* ���ƿ����Բ�Ǿ��Σ�������ֵ1 */
	GUI_SetColor(GUI_YELLOW);
	GUI_AA_FillRoundedRect(135, 4, 155, 16, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('1', 141, 3);
	
	/* ���ƿ����Բ�Ǿ��Σ�������ֵ1 */
	GUI_SetColor(GUI_YELLOW);
	GUI_AA_FillRoundedRect(260, 4, 280, 16, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('1', 266, 2);
	
	/* ���ƿ����Բ�Ǿ��Σ�������ֵ1 */	
	GUI_SetColor(GUI_YELLOW);
	GUI_AA_FillRoundedRect(385, 4, 405, 16, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('1', 391, 2);
	
	/* ���ƿ����Բ�Ǿ��Σ�������ֵ2 */
	GUI_SetColor(GUI_GREEN);
	GUI_AA_FillRoundedRect(10, 19, 30, 31, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('2', 16, 18);
	
	/* ���ƿ����Բ�Ǿ��Σ�������ֵ2 */
	GUI_SetColor(GUI_GREEN);
	GUI_AA_FillRoundedRect(135, 19, 155, 31, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('2', 141, 18);
	
	/* ���ƿ����Բ�Ǿ��Σ�������ֵ2 */
	GUI_SetColor(GUI_GREEN);
	GUI_AA_FillRoundedRect(260, 19, 280, 31, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('2', 266, 18);

	/* ���ƿ����Բ�Ǿ��Σ�������ֵ2 */
	GUI_SetColor(GUI_GREEN);
	GUI_AA_FillRoundedRect(385, 19, 405, 31, 3);
						   
	GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_Font16_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);					
	GUI_DispCharAt('2', 391, 18);
}

/*
*********************************************************************************************************
*	�� �� ��: InitDialogStatus1()
*	����˵��: ״̬���ڵĻص�������ʼ����Ϣ
*	��    �Σ�pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void InitDialogStatus1(WM_MESSAGE * pMsg)
{
	int i;
    WM_HWIN hWin = pMsg->hWin;

	//
    // ��ʼ���ı��ؼ�
    //
	for(i = ID_TEXT_0; i < ID_TEXT_8; i++)
	{
		TEXT_SetTextColor(WM_GetDialogItem(hWin, i), 0x00ffff);
		TEXT_SetFont(WM_GetDialogItem(hWin, i), &GUI_Font16_1);		
	}
	
	for(i = ID_TEXT_8; i < ID_TEXT_16; i++)
	{
		TEXT_SetTextColor(WM_GetDialogItem(hWin, i), GUI_GREEN);
		TEXT_SetFont(WM_GetDialogItem(hWin, i), &GUI_Font16_1);		
	}
	
	/* ������ʱ�� */
	WM_CreateTimer(hWin,   /* ������Ϣ�Ĵ��ڵľ�� */
				   0, 	   /* �û������Id���������ͬһ����ʹ�ö����ʱ������ֵ��������Ϊ�㡣 */
				   1000,   /* ���ڣ������ڹ���ָ������Ӧ�յ���Ϣ*/
				   0);	   /* ��������ʹ�ã�ӦΪ0 */	
}

/*
*********************************************************************************************************
*	�� �� ��: _cbCallbackStatus1
*	����˵��: ״̬���ڵĻص�����
*	��    ��: pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbCallbackStatus1(WM_MESSAGE * pMsg) 
{
    WM_HWIN hWin = pMsg->hWin;
	char buf[20];
	
    switch (pMsg->MsgId) 
    {
        case WM_PAINT:
			GUI_MEMDEV_WriteAt(hMemStatus1Dlg, 145, 444);
            break;
		
        case WM_INIT_DIALOG:
            InitDialogStatus1(pMsg);
            break;
		
		case WM_TIMER:
			/* ƽ��ֵ */
			if(g_DSO1->ucMeasureFlag[24] == 1)
			{
				sprintf(buf, "%5.3fV", g_DSO1->WaveMean);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_0),  g_MeasureTable[20]);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_1),  buf);
				
				sprintf(buf, "%5.3fV", g_DSO2->WaveMean);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_8),  g_MeasureTable[20]);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_9),  buf);
			}
			
			/* ���ֵ */
			if(g_DSO1->ucMeasureFlag[12] == 1)
			{
				sprintf(buf, "%5.3fV", g_DSO1->WavePkPk);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_2),  g_MeasureTable[12]);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_3),  buf);
				
				sprintf(buf, "%5.3fV", g_DSO2->WavePkPk);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_10),  g_MeasureTable[12]);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_11),  buf);				
			}

			/* ��Сֵ */
			if(g_DSO1->ucMeasureFlag[15] == 1)
			{
				sprintf(buf, "%5.3fV", g_DSO1->WaveMin);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_4),  g_MeasureTable[15]);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_5),  buf);
				
				sprintf(buf, "%5.3fV", g_DSO2->WaveMin);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_12),  g_MeasureTable[15]);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_13),  buf);
			}

			/* ���ֵ */
			if(g_DSO1->ucMeasureFlag[14] == 1)
			{
				sprintf(buf, "%5.3fV", g_DSO1->WaveMax);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_6),  g_MeasureTable[14]);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_7),  buf);
				
				sprintf(buf, "%5.3fV", g_DSO2->WaveMax);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_14),  g_MeasureTable[14]);
				TEXT_SetText(WM_GetDialogItem(hWin, ID_TEXT_15),  buf);
			}
			
			WM_RestartTimer(pMsg->Data.v, 500);
			break;
			
        default:
            WM_DefaultProc(pMsg);
    }
}

/*
*********************************************************************************************************
*	�� �� ��: CreateWindowStatus1
*	����˵��: ״̬���ڵĻص�����
*	��    �Σ�pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
WM_HWIN CreateStatus1Dlg(void) 
{
	WM_HWIN hWin;

	hWin = GUI_CreateDialogBox(_aDialogCreateStatus1, 
	                           GUI_COUNTOF(_aDialogCreateStatus1), 
	                           &_cbCallbackStatus1, 
	                           0, 
	                           0, 
	                           0);
	
	return hWin;
}

/*
*********************************************************************************************************
*	�� �� ��: DrawWinStatus1Bk
*	����˵��: �����ڱ������Ƶ��洢�豸����
*	��    ��: ��         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DrawWinStatus1Bk(void) 
{
	hMemStatus1Dlg = GUI_MEMDEV_CreateFixed(0, 
										    0, 
									        510, 
									        35, 
									        GUI_MEMDEV_HASTRANS, 
										    GUI_MEMDEV_APILIST_16, 
										    GUICC_M565);
	GUI_MEMDEV_Select(hMemStatus1Dlg);
	PaintDialogStatus1(NULL);
	GUI_MEMDEV_Select(0);
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
