/*
*********************************************************************************************************
*	                                  
*	ģ������ : ʾ����Math�Ի���
*	�ļ����� : DSO_MathDlg.c
*	��    �� : V1.0
*	˵    �� : �˶Ի�����Ҫ����80��Fir��ͨ�˲����Ľ�ֹƵ��ѡ��
*              1. Fs(������) = 2Mspsʱ��  ��Scale������ʾ500nsʱ��Fc(��ֹƵ��)��ѡ��
*              2. Fs(������) = 200Kspsʱ����Scale������ʾ5usʱ��  Fc(��ֹƵ��)��ѡ��
*              3. Fs(������) = 20Kspsʱ�� ��Scale������ʾ50usʱ�� Fc(��ֹƵ��)��ѡ��
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
#define ID_WINDOW_0    (GUI_ID_USER + 0x00)
#define ID_RADIO_0     (GUI_ID_USER + 0x01)
#define ID_TEXT_0      (GUI_ID_USER + 0x02)
#define ID_TEXT_1      (GUI_ID_USER + 0x03)
#define ID_TEXT_2      (GUI_ID_USER + 0x04)
#define ID_TEXT_3      (GUI_ID_USER + 0x05)
#define ID_RADIO_1     (GUI_ID_USER + 0x06)
#define ID_RADIO_2     (GUI_ID_USER + 0x07)

#define ID_BUTTON_0    (GUI_ID_USER + 0x08)
#define ID_BUTTON_1    (GUI_ID_USER + 0x09)
/*
*********************************************************************************************************
*	                                  ����
*********************************************************************************************************
*/
static uint32_t Radio_Step100KHz = 0; /* ��һ�δ�ʱĬ�ϲ������˲� */
static uint32_t Radio_Step10KHz = 0;  /* ��һ�δ�ʱĬ�ϲ������˲�  */
static uint32_t Radio_Step1KHz = 0;	  /* ��һ�δ�ʱĬ�ϲ������˲�  */

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
static const GUI_WIDGET_CREATE_INFO _aDialogCreateMath[] =
	{
		{WINDOW_CreateIndirect, "Math", ID_WINDOW_0, 0, 0, 800, 480, 0, 0x0, 0},
		{TEXT_CreateIndirect, "Text", ID_TEXT_0, 200, 10, 400, 32, 0, 0x64, 0},

		{TEXT_CreateIndirect, "Text", ID_TEXT_1, 0, 100, 266, 30, 0, 0x64, 0},
		{RADIO_CreateIndirect, "Radio", ID_RADIO_0, 73, 140, 120, 240, 0, 0x1D08, 0},

		{TEXT_CreateIndirect, "Text", ID_TEXT_2, 267, 100, 266, 30, 0, 0x64, 0},
		{RADIO_CreateIndirect, "Radio", ID_RADIO_1, 340, 140, 120, 240, 0, 0x1D08, 0},

		{TEXT_CreateIndirect, "Text", ID_TEXT_3, 533, 100, 266, 30, 0, 0x64, 0},
		{RADIO_CreateIndirect, "Radio", ID_RADIO_2, 606, 140, 120, 240, 0, 0x1D08, 0},

		{BUTTON_CreateIndirect, "DlgBack", ID_BUTTON_0, 728, 0, 72, 72, 0, 0, 0},
		{BUTTON_CreateIndirect, "MusicList", ID_BUTTON_1, 0, 0, 72, 72, 0, 0, 0},
};

/*
*********************************************************************************************************
*	�� �� ��: _cbDialogMath
*	����˵��: �����Ի���Ļص�����
*	��    �Σ�pMsg  ָ���ַ         	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbDialogMath(WM_MESSAGE *pMsg)
{
	WM_HWIN hItem;
	int NCode;
	int Id;
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
		// ��ʼ����ť
		//
		hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_0);
		WM_SetHasTrans(hItem);
		WM_SetCallback(hItem, _cbButtonBack);

		hItem = WM_GetDialogItem(pMsg->hWin, ID_BUTTON_1);
		WM_SetHasTrans(hItem);
		WM_SetCallback(hItem, _cbButtonSettings);

		//
		// ��ʼ��ID_TEXT_0
		//
		hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_0);
		TEXT_SetFont(hItem, &GUI_FontLubalGraph32);
		TEXT_SetTextColor(hItem, GUI_RED);
		TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
		TEXT_SetText(hItem, "80 Order Fir Lowpass");

		//
		// ��ʼ��ID_TEXT_1
		//
		hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_1);
		TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
		TEXT_SetText(hItem, "Fs=2Msps,Scale=500ns");
		TEXT_SetFont(hItem, &GUI_FontLubalGraph24);

		//
		// ��ʼ��ID_TEXT_2
		//
		hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_2);
		TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
		TEXT_SetText(hItem, "Fs=200Ksps,Scale=10us");
		TEXT_SetFont(hItem, &GUI_FontLubalGraph24);

		//
		// ��ʼ��ID_TEXT_3
		//
		hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
		TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
		TEXT_SetText(hItem, "Fs=20Ksps,Scale=100us");
		TEXT_SetFont(hItem, &GUI_FontLubalGraph24);

		//
		// ��ʼ��ID_RADIO_0
		//
		hItem = WM_GetDialogItem(pMsg->hWin, ID_RADIO_0);
		RADIO_SetText(hItem, "None", 0);
		RADIO_SetFont(hItem, &GUI_Font20_ASCII);
		RADIO_SetText(hItem, "Fc=100KHz", 1);
		RADIO_SetText(hItem, "Fc=200KHz", 2);
		RADIO_SetText(hItem, "Fc=300KHz", 3);
		RADIO_SetText(hItem, "Fc=400KHz", 4);
		RADIO_SetText(hItem, "Fc=500KHz", 5);
		RADIO_SetText(hItem, "Fc=600KHz", 6);
		RADIO_SetText(hItem, "Fc=700KHz", 7);
		RADIO_SetValue(hItem, Radio_Step100KHz); /* ����Ĭ��״̬ */
		//WIDGET_SetFocusable(hItem, 0);

		//
		// ��ʼ��ID_RADIO_1
		//
		hItem = WM_GetDialogItem(pMsg->hWin, ID_RADIO_1);
		RADIO_SetText(hItem, "None", 0);
		RADIO_SetText(hItem, "Fc=10KHz", 1);
		RADIO_SetFont(hItem, &GUI_Font20_ASCII);
		RADIO_SetText(hItem, "Fc=20KHz", 2);
		RADIO_SetText(hItem, "Fc=30KHz", 3);
		RADIO_SetText(hItem, "Fc=40KHz", 4);
		RADIO_SetText(hItem, "Fc=50KHz", 5);
		RADIO_SetText(hItem, "Fc=60KHz", 6);
		RADIO_SetText(hItem, "Fc=70KHz", 7);
		RADIO_SetValue(hItem, Radio_Step10KHz); /* ����Ĭ��״̬ */
		//WIDGET_SetFocusable(hItem, 0);

		//
		// ��ʼ��ID_RADIO_2
		//
		hItem = WM_GetDialogItem(pMsg->hWin, ID_RADIO_2);
		RADIO_SetText(hItem, "None", 0);
		RADIO_SetText(hItem, "Fc=1KHz", 1);
		RADIO_SetFont(hItem, &GUI_Font20_1);
		RADIO_SetText(hItem, "Fc=2KHz", 2);
		RADIO_SetText(hItem, "Fc=3KHz", 3);
		RADIO_SetText(hItem, "Fc=4KHz", 4);
		RADIO_SetText(hItem, "Fc=5KHz", 5);
		RADIO_SetText(hItem, "Fc=6KHz", 6);
		RADIO_SetText(hItem, "Fc=7KHz", 7);
		RADIO_SetValue(hItem, Radio_Step1KHz); /* ����Ĭ��״̬ */
		//WIDGET_SetFocusable(hItem, 0);
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

		/* ��ѡ��ť��Ϣ */
		case ID_RADIO_0:
			switch (NCode)
			{
			case WM_NOTIFICATION_CLICKED:
				break;

			case WM_NOTIFICATION_RELEASED:
				break;

			case WM_NOTIFICATION_VALUE_CHANGED:
				/* Fs(������) = 2Mspsʱ����Scale������ʾ500nsʱ��Fc(��ֹƵ��)��ѡ�� */
				Radio_Step100KHz = RADIO_GetValue(WM_GetDialogItem(pMsg->hWin, ID_RADIO_0));
				g_DSO1->ucFirFlter_Step100KHz = Radio_Step100KHz;
				break;
			}
			break;

		/* ��ѡ��ť��Ϣ */
		case ID_RADIO_1:
			switch (NCode)
			{
			case WM_NOTIFICATION_CLICKED:
				break;

			case WM_NOTIFICATION_RELEASED:
				break;

			case WM_NOTIFICATION_VALUE_CHANGED:
				/* Fs(������) = 200Kspsʱ����Scale������ʾ5usʱ��Fc(��ֹƵ��)��ѡ�� */
				Radio_Step10KHz = RADIO_GetValue(WM_GetDialogItem(pMsg->hWin, ID_RADIO_1));
				g_DSO1->ucFirFlter_Step10KHz = Radio_Step10KHz;
				break;
			}
			break;

		/* ��ѡ��ť��Ϣ */
		case ID_RADIO_2:
			switch (NCode)
			{
			case WM_NOTIFICATION_CLICKED:
				break;

			case WM_NOTIFICATION_RELEASED:
				break;

			case WM_NOTIFICATION_VALUE_CHANGED:
				/* Fs(������) = 20Kspsʱ����Scale������ʾ50usʱ��Fc(��ֹƵ��)��ѡ�� */
				Radio_Step1KHz = RADIO_GetValue(WM_GetDialogItem(pMsg->hWin, ID_RADIO_2));
				g_DSO1->ucFirFlter_Step1KHz = Radio_Step1KHz;
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
*	�� �� ��: DSO_CreateMathDlg
*	����˵��: �����Ի���
*	��    ��: ��        	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
WM_HWIN DSO_CreateMathDlg(void)
{
	WM_HWIN hWin;

	hWin = GUI_CreateDialogBox(_aDialogCreateMath, GUI_COUNTOF(_aDialogCreateMath), _cbDialogMath, WM_HBKWIN, 0, 0);

	return hWin;
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
