/*
*********************************************************************************************************
*	                                  
*	ģ������ : ʾ���������Ի���
*	�ļ����� : DSO_MeasureDlg.c
*	��    �� : V1.0
*	˵    �� : ֧��30�����ݲ������ܣ���ǰʵ��ƽ��ֵ�����ֵ����Сֵ�����ֵ��RMS��Ƶ��6����ֵ������
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
*	                                  ��������
*********************************************************************************************************
*/
const char *g_MeasureText[] =
	{
		"Snapshot All",
		"Freqency",
		"Period",
		"Rise Time",
		"Fall Time",
		"Delay",
		"Phase",
		"Positive Pulse Width",
		"Negative Pulse Width",
		"Positive Duty Cycle",
		"Negative Duty Cycle",
		"Burst Width",
		"Peak-to-peak",
		"Amplitude",
		"Max",
		"Min",
		"High",
		"Low",
		"Positive Overshoot",
		"Negative Overshoot",
		"Positive Pulse Count",
		"Negative Pulse Count",
		"RMS",
		"Cycle RMS",
		"Mean",
		"Cycle Mean",
		"Rising Edge Count",
		"Falling Edge Count",
		"Area",
		"Cycle Area",
};

/*
*********************************************************************************************************
*	                                  �Զ���ؼ�ID
*********************************************************************************************************
*/
#define ID_WINDOW_0  	(GUI_ID_USER + 0x00)
#define ID_CHECKBOX_0  	(GUI_ID_USER + 0x01)
#define ID_CHECKBOX_1  	(GUI_ID_USER + 0x02)
#define ID_CHECKBOX_2  	(GUI_ID_USER + 0x03)
#define ID_CHECKBOX_3  	(GUI_ID_USER + 0x04)
#define ID_CHECKBOX_4  	(GUI_ID_USER + 0x05)
#define ID_CHECKBOX_5  	(GUI_ID_USER + 0x06)
#define ID_CHECKBOX_6  	(GUI_ID_USER + 0x07)
#define ID_CHECKBOX_7  	(GUI_ID_USER + 0x08)
#define ID_CHECKBOX_8  	(GUI_ID_USER + 0x09)
#define ID_CHECKBOX_9  	(GUI_ID_USER + 0x0A)
#define ID_CHECKBOX_10  (GUI_ID_USER + 0x0B)
#define ID_CHECKBOX_11  (GUI_ID_USER + 0x0C)
#define ID_CHECKBOX_12  (GUI_ID_USER + 0x0D)
#define ID_CHECKBOX_13  (GUI_ID_USER + 0x0E)
#define ID_CHECKBOX_14  (GUI_ID_USER + 0x0F)
#define ID_CHECKBOX_15  (GUI_ID_USER + 0x10)
#define ID_CHECKBOX_16  (GUI_ID_USER + 0x11)
#define ID_CHECKBOX_17  (GUI_ID_USER + 0x12)
#define ID_CHECKBOX_18  (GUI_ID_USER + 0x13)
#define ID_CHECKBOX_19  (GUI_ID_USER + 0x14)
#define ID_CHECKBOX_20  (GUI_ID_USER + 0x15)
#define ID_CHECKBOX_21  (GUI_ID_USER + 0x16)
#define ID_CHECKBOX_22  (GUI_ID_USER + 0x17)
#define ID_CHECKBOX_23  (GUI_ID_USER + 0x18)
#define ID_CHECKBOX_24  (GUI_ID_USER + 0x19)
#define ID_CHECKBOX_25  (GUI_ID_USER + 0x1A)
#define ID_CHECKBOX_26  (GUI_ID_USER + 0x1B)
#define ID_CHECKBOX_27  (GUI_ID_USER + 0x1C)
#define ID_CHECKBOX_28  (GUI_ID_USER + 0x1D)
#define ID_CHECKBOX_29  (GUI_ID_USER + 0x1E)

#define ID_BUTTON_0  	(GUI_ID_USER + 0x1F)
#define ID_BUTTON_1  	(GUI_ID_USER + 0x20)

#define ID_TEXT_0 (GUI_ID_USER + 0x21)

/*
*********************************************************************************************************
*	�� �� ��: _cbButtonBack
*	����˵��: ��ť�ص�����
*	��    ��: pMsg  ��Ϣָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbButtonBack(WM_MESSAGE *pMsg)
{
	WM_HWIN hWin;

	hWin = pMsg->hWin;
	switch (pMsg->MsgId)
	{
	case WM_PAINT:
		if (BUTTON_IsPressed(hWin))
		{
			GUI_SetColor(GUI_DARKGRAY);
			GUI_AA_FillCircle(72, 0, 72);
			GUI_SetBkColor(GUI_DARKGRAY);
			GUI_SetColor(GUI_WHITE);
			GUI_DrawBitmap(&bmReturn, 40 - 28, 5);
		}
		else
		{
			GUI_SetColor(GUI_STCOLOR_LIGHTBLUE);
			GUI_AA_FillCircle(72, 0, 72);
			GUI_SetBkColor(GUI_STCOLOR_LIGHTBLUE);
			GUI_SetColor(GUI_WHITE);
			GUI_DrawBitmap(&bmReturn, 40 - 28, 5);
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
static void _cbButtonSettings(WM_MESSAGE *pMsg)
{
	WM_HWIN hWin;

	hWin = pMsg->hWin;
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
static const GUI_WIDGET_CREATE_INFO _aDialogCreateMeasure[] =
	{
		{WINDOW_CreateIndirect, "Window", ID_WINDOW_0, 0, 0, 800, 480, 0, 0x64, 0},

		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_0, 50, 80, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_1, 50, 120, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_2, 50, 160, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_3, 50, 200, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_4, 50, 240, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_5, 50, 280, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_6, 50, 320, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_7, 50, 360, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_8, 50, 400, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_9, 50, 440, 200, 25, 0, 0x0, 0},

		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_10, 300, 80, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_11, 300, 120, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_12, 300, 160, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_13, 300, 200, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_14, 300, 240, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_15, 300, 280, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_16, 300, 320, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_17, 300, 360, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_18, 300, 400, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_19, 300, 440, 200, 25, 0, 0x0, 0},

		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_20, 550, 80, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_21, 550, 120, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_22, 550, 160, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_23, 550, 200, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_24, 550, 240, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_25, 550, 280, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_26, 550, 320, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_27, 550, 360, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_28, 550, 400, 200, 25, 0, 0x0, 0},
		{CHECKBOX_CreateIndirect, "Checkbox", ID_CHECKBOX_29, 550, 440, 200, 25, 0, 0x0, 0},

		{BUTTON_CreateIndirect, "DlgBack", ID_BUTTON_0, 728, 0, 72, 72, 0, 0, 0},
		{BUTTON_CreateIndirect, "MusicList", ID_BUTTON_1, 0, 0, 72, 72, 0, 0, 0},

		{TEXT_CreateIndirect, "Text", ID_TEXT_0, 200, 10, 400, 32, 0, 0x64, 0},
};

/*
*********************************************************************************************************
*	�� �� ��: _cbDialogMeasure
*	����˵��: �����Ի���Ļص�����
*	��    �Σ�pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbDialogMeasure(WM_MESSAGE *pMsg)
{
	WM_HWIN hItem;
	int NCode;
	int Id;
	int i;
	WM_HWIN hWin = pMsg->hWin;

	switch (pMsg->MsgId)
	{
	/* �Ի���رպ�ʹ��һ�β���ˢ�£���ֹ�ڲ�����ͣ״̬��(ͨ��K2������������ͣ)
		   �رնԻ��򲢷��ص�ʱ�򣬲�������û��ˢ�£�һƬ�հס�
		*/
	case WM_DELETE:
		/* �Զ�������ͣ״̬ */
		if (g_Flag->hWinRunStop == 1)
		{
			g_Flag->ucWaveRefresh = 1;
		}

		/* ��ͨ������ͣ״̬ */
		if (TriggerFlag == 1)
		{
			TriggerFlag = 2;
		}
		break;

	case WM_INIT_DIALOG:
		//
		// ��ʼ����ѡ��
		//
		for (i = 0; i < 30; i++)
		{
			hItem = WM_GetDialogItem(pMsg->hWin, ID_CHECKBOX_0 + i);
			CHECKBOX_SetText(hItem, g_MeasureText[i]);
			CHECKBOX_SetFont(hItem, &GUI_FontLubalGraph20);
			CHECKBOX_SetState(hItem, g_DSO1->ucMeasureFlag[i]);
			if (g_DSO1->ucMeasureFlag[i] == 1)
			{
				CHECKBOX_SetTextColor(hItem, GUI_RED); /* ��ɺ����ʾ��ǰ֧�ֵĲ��� */
			}
			else
			{
				CHECKBOX_SetTextColor(hItem, GUI_BLACK); /* ��ɺ�ɫ��ʾ��ǰ֧�ֵĲ��� */
			}
		}

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
		// ��ʼ���ı��ؼ�
		//
		hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
		TEXT_SetFont(hItem, &GUI_FontLubalGraph32);
		TEXT_SetTextColor(hItem, GUI_RED);
		TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
		TEXT_SetText(hItem, "Wave Measure");
		break;

	case WM_NOTIFY_PARENT:
		Id = WM_GetId(pMsg->hWinSrc);
		NCode = pMsg->Data.v;
		switch (Id)
		{
		/* �رնԻ��� */
		case ID_BUTTON_0:
			switch (NCode)
			{
			case WM_NOTIFICATION_RELEASED:
				GUI_EndDialog(hWin, 0);
				break;
			}
			break;

		/* ��ѡ����Ϣ */
		case ID_CHECKBOX_0:
		case ID_CHECKBOX_1:
		case ID_CHECKBOX_2:
		case ID_CHECKBOX_3:
		case ID_CHECKBOX_4:
		case ID_CHECKBOX_5:
		case ID_CHECKBOX_6:
		case ID_CHECKBOX_7:
		case ID_CHECKBOX_8:
		case ID_CHECKBOX_9:
		case ID_CHECKBOX_10:
		case ID_CHECKBOX_11:
		case ID_CHECKBOX_12:
		case ID_CHECKBOX_13:
		case ID_CHECKBOX_14:
		case ID_CHECKBOX_15:
		case ID_CHECKBOX_16:
		case ID_CHECKBOX_17:
		case ID_CHECKBOX_18:
		case ID_CHECKBOX_19:
		case ID_CHECKBOX_20:
		case ID_CHECKBOX_21:
		case ID_CHECKBOX_22:
		case ID_CHECKBOX_23:
		case ID_CHECKBOX_24:
		case ID_CHECKBOX_25:
		case ID_CHECKBOX_26:
		case ID_CHECKBOX_27:
		case ID_CHECKBOX_28:
		case ID_CHECKBOX_29:
			switch (NCode)
			{
			/* ��ѡ������Ϣ */
			case WM_NOTIFICATION_CLICKED:
				break;

			/* ��ѡ���ͷ���Ϣ */
			case WM_NOTIFICATION_RELEASED:
				g_DSO1->ucMeasureFlag[Id - ID_CHECKBOX_0] = CHECKBOX_GetState(WM_GetDialogItem(pMsg->hWin, Id));
				break;

			/* ��ѡ��״̬�ı���Ϣ */
			case WM_NOTIFICATION_VALUE_CHANGED:
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
*	�� �� ��: DSO_CreateMeasureDlg
*	����˵��: ���������Ի���
*	��    ��: ��        	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
WM_HWIN DSO_CreateMeasureDlg(void)
{
	WM_HWIN hWin;

	hWin = GUI_CreateDialogBox(_aDialogCreateMeasure, GUI_COUNTOF(_aDialogCreateMeasure), _cbDialogMeasure, WM_HBKWIN, 0, 0);
	return hWin;
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
