/*
*********************************************************************************************************
*	                                  
*	ģ������ : ʾ�������öԻ���
*	�ļ����� : DSO_SettingDlg.c
*	��    �� : V1.0
*	˵    �� : ��Ҫ��������5������
*              1. �����Ƿ����ط�ֵ���ڣ�״̬���ڣ�ʱ�����ں�ϵͳ���ڡ�
*              2. �����Ƿ�����FFT����ķ�Ƶ��Ӧ���Ρ�
*              3. ����ˮƽ�ʹ�ֱ�����α����ʾ��
*              4. ����ˮƽ�ʹ�ֱ�����α���ƶ�������
*              5. ����ˮƽ�������Ǵ�ֱ������
*
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
*	                                  �Զ���ؼ�ID
*********************************************************************************************************
*/
#define ID_WINDOW_0  	(GUI_ID_USER + 0x00)
#define ID_CHECKBOX_0   (GUI_ID_USER + 0x01)
#define ID_CHECKBOX_1   (GUI_ID_USER + 0x02)
#define ID_RADIO_0  	(GUI_ID_USER + 0x03)
#define ID_RADIO_1  	(GUI_ID_USER + 0x04)
#define ID_RADIO_2  	(GUI_ID_USER + 0x05)
#define ID_BUTTON_0  	(GUI_ID_USER + 0x06)
#define ID_BUTTON_1  	(GUI_ID_USER + 0x07)
#define ID_TEXT_0  		(GUI_ID_USER + 0x08)


/*
*********************************************************************************************************
*	                                  ����
*********************************************************************************************************
*/
static uint32_t CheckBoxHideFFT = 0;     /* ��һ�δ�ʱĬ��δ����FFT������ʾ */
static uint32_t CheckBoxHideWindow = 0;  /* ��һ�δ�ʱĬ��δ���ش��� */
static uint32_t RadioCursorDisp = 0;     /* ��һ�δ�ʱĬ�ϲ���ʾˮƽ�ʹ�ֱ�����α� */
static uint32_t RadioCursorStep = 3;     /* ��һ�δ�ʱĬ��ѡ��ˮƽ�ʹ�ֱ�α궼���ƶ�10������ */


/*
*********************************************************************************************************
*	�� �� ��: _cbButtonBack
*	����˵��: ��ť�ص�����
*	��    ��: pMsg  ��Ϣָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbButtonBack(WM_MESSAGE * pMsg) 
{
	WM_HWIN  hWin;

	hWin  = pMsg->hWin;
	switch (pMsg->MsgId) 
	{
		case WM_PAINT:
			if (BUTTON_IsPressed(hWin)) 
			{
				GUI_SetColor(GUI_DARKGRAY);
				GUI_AA_FillCircle(72, 0, 72);
				GUI_SetBkColor(GUI_DARKGRAY);
				GUI_SetColor(GUI_WHITE); 
				GUI_DrawBitmap(&bmReturn, 40-28, 5);
			} 
			else 
			{
				GUI_SetColor(GUI_STCOLOR_LIGHTBLUE);
				GUI_AA_FillCircle(72, 0, 72);
				GUI_SetBkColor(GUI_STCOLOR_LIGHTBLUE);
				GUI_SetColor(GUI_WHITE);  
				GUI_DrawBitmap(&bmReturn, 40-28, 5);				
			}
			break;
			
		default:
			BUTTON_Callback(pMsg);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: _cbButtonSettings
*	����˵��: ��ť�ص�����
*	��    ��: pMsg  ��Ϣָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbButtonSettings(WM_MESSAGE * pMsg) 
{
	WM_HWIN  hWin;

	hWin  = pMsg->hWin;
	switch (pMsg->MsgId) 
	{
		case WM_PAINT:
			if (BUTTON_IsPressed(hWin)) 
			{
				GUI_SetColor(GUI_DARKGRAY);
				GUI_AA_FillCircle(0, 0, 72);
				GUI_SetBkColor(GUI_DARKGRAY);
				GUI_SetColor(GUI_WHITE); 
				
				GUI_DrawBitmap(&bmSettings, 0, 5);
			} 
			else 
			{
				GUI_SetColor(GUI_STCOLOR_LIGHTBLUE);
				GUI_AA_FillCircle(0, 0, 72);
				GUI_SetBkColor(GUI_STCOLOR_LIGHTBLUE);
				GUI_SetColor(GUI_WHITE);  

				GUI_DrawBitmap(&bmSettings, 0, 5);
			}
			break;
			
		default:
			BUTTON_Callback(pMsg);
	}
}

/*
*********************************************************************************************************
*					GUI_WIDGET_CREATE_INFO��������
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreateSettings[] = 
{
	{ WINDOW_CreateIndirect,   "Window",   ID_WINDOW_0, 0, 0, 800, 480, 0, 0x64, 0 },

	{ RADIO_CreateIndirect,    "Radio",    ID_RADIO_0,    20,  100, 250,  60, 0, 0x1D02, 0 },	
	{ RADIO_CreateIndirect,    "Radio",    ID_RADIO_1,    20,  190, 250,  90, 0, 0x1D03, 0 },
	{ RADIO_CreateIndirect,    "Radio",    ID_RADIO_2,    400, 100, 250, 120, 0, 0x1D04, 0 },

	{ CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_0, 20, 310, 700, 25, 0, 0x0, 0 },	
	{ CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_1, 20, 350, 700, 25, 0, 0x0, 0 },
	
	{ BUTTON_CreateIndirect, "DlgBack",    ID_BUTTON_0,   728,    0,  72, 72, 0, 0, 0},	
	{ BUTTON_CreateIndirect, "MusicList",  ID_BUTTON_1,     0,    0,  72, 72, 0, 0, 0},
	
	{ TEXT_CreateIndirect,   "Text", ID_TEXT_0,  200, 10, 400, 32, 0, 0x64, 0 },
};

/*
*********************************************************************************************************
*	�� �� ��: _cbDialogSettings
*	����˵��: ���öԻ���Ļص�����
*	��    ��: pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbDialogSettings(WM_MESSAGE * pMsg) 
{
	WM_HWIN hItem;
	int     NCode;
	int     Id;
	WM_HWIN hWin = pMsg->hWin;

	switch (pMsg->MsgId) 
	{
		/* �Ի���رպ�ʹ��һ�β���ˢ�£���ֹ�ڲ�����ͣ״̬��(ͨ��K2������������ͣ)
		   �رնԻ��򲢷��ص�ʱ�򣬲�������û��ˢ�£�һƬ�հס�
		*/
		case WM_DELETE:
			/* �Զ�������ͣ״̬ */
			if(g_Flag->hWinRunStop == 1)
			{
				g_Flag->ucWaveRefresh = 1;
			}
		
			/* ��ͨ������ͣ״̬ */
			if(TriggerFlag == 1)
			{
				TriggerFlag = 2;
			}
			break;
		
		case WM_INIT_DIALOG:
			//
			// ��ʼ��ID_RADIO_0
			//
			hItem = WM_GetDialogItem(pMsg->hWin, ID_RADIO_0);
			RADIO_SetText(hItem, "Y Motion", 0);
			RADIO_SetText(hItem, "X Motion", 1);
			RADIO_SetFont(hItem, &GUI_Font20_1);
			RADIO_SetValue(hItem, g_Flag->ucMotionXY);  /* ����Ĭ��״̬ */

			//
			// ��ʼ��ID_RADIO_1
			//
			hItem = WM_GetDialogItem(pMsg->hWin, ID_RADIO_1);
			RADIO_SetText(hItem, "Hide Cursor", 0);
			RADIO_SetText(hItem, "Display HorizontalCursor", 1);
			RADIO_SetText(hItem, "Display VerticalCursor", 2);
			RADIO_SetFont(hItem, &GUI_Font20_1);
			RADIO_SetValue(hItem, RadioCursorDisp); /* ����Ĭ��״̬ */

			//
			// ��ʼ��ID_RADIO_2
			//
			hItem = WM_GetDialogItem(pMsg->hWin, ID_RADIO_2);
			RADIO_SetText(hItem, "MoveCursorStep = 1", 0);
			RADIO_SetText(hItem, "MoveCursorStep = 2", 1);
			RADIO_SetText(hItem, "MoveCursorStep = 5", 2);
			RADIO_SetText(hItem, "MoveCursorStep = 10", 3);
			RADIO_SetFont(hItem, &GUI_Font20_1);
			RADIO_SetValue(hItem, RadioCursorStep);  /* ����Ĭ��״̬ */

			//
			// ��ʼ��ID_CHECKBOX_0
			//
			hItem = WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_0);
			CHECKBOX_SetText(hItem, "Hide RFFT2048 Display");
			CHECKBOX_SetFont(hItem, &GUI_FontLubalGraph24);
			CHECKBOX_SetState(hItem, CheckBoxHideFFT); /* ����Ĭ��״̬ */
			
			//
			// ��ʼ��ID_CHECKBOX_1
			//
			hItem = WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_1);
			CHECKBOX_SetText(hItem, "Hide AmplitudeDlg, StatusDlg, ScaleDlg and SysInfoDlg");
			CHECKBOX_SetFont(hItem, &GUI_FontLubalGraph24);
			CHECKBOX_SetState(hItem, CheckBoxHideWindow); /* ����Ĭ��״̬ */
			
			//
			// ��ʼ����ť
			//
			hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
			WM_SetHasTrans(hItem);
			WM_SetCallback(hItem, _cbButtonBack);
			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_1);
			WM_SetHasTrans(hItem);
			WM_SetCallback(hItem, _cbButtonSettings);
			
			//
			// ��ʼ���ı�
			//			
			hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
			TEXT_SetFont(hItem, &GUI_FontLubalGraph32);
			TEXT_SetTextColor(hItem, GUI_RED);
			TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
			TEXT_SetText(hItem, "Settings");
			break;

		case WM_NOTIFY_PARENT:
			Id    = WM_GetId(pMsg->hWinSrc);
			NCode = pMsg->Data.v;
			switch(Id) 
			{
				/* �رնԻ��� */
				case ID_BUTTON_0:
					switch(NCode)
					{
						case WM_NOTIFICATION_RELEASED:
							GUI_EndDialog(hWin, 0);
							break;
					}
					break;

				/* ��ѡ����Ϣ */
				case ID_CHECKBOX_0: 
					switch(NCode) 
					{
						/* ��ѡ������Ϣ */
						case WM_NOTIFICATION_CLICKED:
							break;
						
						/* ��ѡ�����ͷź���Ϣ */
						case WM_NOTIFICATION_RELEASED:
							CheckBoxHideFFT = CHECKBOX_GetState(WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_0));
							g_Flag->ucFFTDisp = CheckBoxHideFFT;
							break;

						case WM_NOTIFICATION_VALUE_CHANGED:
							break;
					}
					break;
					
				/* ��ѡ����Ϣ */
				case ID_CHECKBOX_1: 
					switch(NCode) 
					{
						/* ��ѡ������Ϣ */
						case WM_NOTIFICATION_CLICKED:
							break;
						
						/* ��ѡ�����ͷź���Ϣ */
						case WM_NOTIFICATION_RELEASED:
							CheckBoxHideWindow = CHECKBOX_GetState(WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_1));
							/* ѡ�и�ѡ�������ط�ֵ���ڣ�״̬���ں�ʱ������ */
							if(CheckBoxHideWindow == 1)
							{
								WM_HideWindow(hDlgAmp);
								WM_HideWindow(hDlgStatus1);
								WM_HideWindow(hDlgStatus2);
								WM_HideWindow(hDlgScale);
								WM_HideWindow(hDlgSysInfo);
							}
							/* ��ѡ������ʾ���� */
							else
							{
								WM_ShowWindow(hDlgAmp);
								WM_ShowWindow(hDlgStatus1);
								WM_ShowWindow(hDlgStatus2);
								WM_ShowWindow(hDlgScale);
								WM_ShowWindow(hDlgSysInfo);
							}
							break;

						case WM_NOTIFICATION_VALUE_CHANGED:
							break;
					}
					break;

				/* ��ѡ��ť��Ϣ */
				case ID_RADIO_0: 
					switch(NCode) 
					{
						case WM_NOTIFICATION_CLICKED:
							break;

						case WM_NOTIFICATION_RELEASED:
							break;

						case WM_NOTIFICATION_VALUE_CHANGED:
							/* ˮƽ�ʹ�ֱ�����л���־��ֵ��g_Flag->ucMotionXY */
							g_Flag->ucMotionXY = RADIO_GetValue( WM_GetDialogItem(pMsg->hWin, ID_RADIO_0));
							break;
					}
					break;
					
				/* ��ѡ��ť��Ϣ */
				case ID_RADIO_1: 
					switch(NCode) 
					{
						case WM_NOTIFICATION_CLICKED:
							break;

						case WM_NOTIFICATION_RELEASED:
							break;

						case WM_NOTIFICATION_VALUE_CHANGED:
							/* ��ˮƽ�ʹ�ֱ�α��״̬��ֵ��ȫ�ֱ���g_Flag->hWinCursors */
							RadioCursorDisp = RADIO_GetValue( WM_GetDialogItem(pMsg->hWin, ID_RADIO_1));
							g_Flag->hWinCursors = RadioCursorDisp;
							break;
					}
					break;
				
				/* ��ѡ��ť״̬ */
				case ID_RADIO_2: 
					switch(NCode) 
					{
						case WM_NOTIFICATION_CLICKED:
							break;

						case WM_NOTIFICATION_RELEASED:
							break;

						case WM_NOTIFICATION_VALUE_CHANGED:
						/* ˮƽ�ʹ�ֱ�α���ƶ�������ֵ��ȫ�ֱ��� */
						RadioCursorStep = RADIO_GetValue( WM_GetDialogItem(pMsg->hWin, ID_RADIO_2));
						if(RadioCursorStep == 0)
						{
							/* ÿ���ƶ�1�� */
							g_Cursors->usCursorStep = 1;
						}
						else if(RadioCursorStep == 1)
						{
							/* ÿ���ƶ�2�� */
							g_Cursors->usCursorStep = 2;			
						}
						else if(RadioCursorStep == 2)
						{
							/* ÿ���ƶ�5�� */
							g_Cursors->usCursorStep = 5;			
						}
						else if(RadioCursorStep == 3)
						{
							/* ÿ���ƶ�10�� */
							g_Cursors->usCursorStep = 10;			
						}
						break;
					}
					break;
			}
			break;
			
		default:
			WM_DefaultProc(pMsg);
			break;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: DSO_CreateSettingsDlg
*	����˵��: �������öԻ���
*	��    ��: ��        	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
WM_HWIN DSO_CreateSettingsDlg(void) 
{
	WM_HWIN hWin;

	hWin = GUI_CreateDialogBox(_aDialogCreateSettings, GUI_COUNTOF(_aDialogCreateSettings), _cbDialogSettings, WM_HBKWIN, 0, 0);
	return hWin;
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
