/*
*********************************************************************************************************
*	                                  
*	ģ������ : ʾ����������ĳ�ʼ��
*	�ļ����� : DSO_Init.c
*	��    �� : V1.0
*	˵    �� : ʾ������ʼ��
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
WM_HWIN hDlgAmp;	 /* ������ʾ���ȵĴ��ھ�� */
WM_HWIN hDlgStatus1; /* ������ʾƽ��ֵ�����ֵ�����ֵ����Сֵ���ھ�� */
WM_HWIN hDlgStatus2; /* ������ʾƵ�ʺ�RMS���ھ�� */
WM_HWIN hDlgScale;	 /* ������ʾ�����ʴ��ھ�� */
WM_HWIN hDlgSysInfo; /* ������ʾϵͳ��Ϣ���ھ�� */

BUTTON_Handle hButton0; /* 5����ť��� */
BUTTON_Handle hButton1;
BUTTON_Handle hButton2;
BUTTON_Handle hButton3;
BUTTON_Handle hButton4;

/*
*********************************************************************************************************
*	�� �� ��: _cbButton
*	����˵��: ��ť�ص�����
*	��    ��: pMsg  ��Ϣָ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbButton(WM_MESSAGE *pMsg)
{
	WM_HWIN hWin;
	GUI_RECT Rect;
	char buf[20];

	hWin = pMsg->hWin;

	switch (pMsg->MsgId)
	{
	case WM_PAINT:
		WM_GetClientRect(&Rect);
		if (BUTTON_IsPressed(hWin))
		{
			GUI_SetColor(GUI_DARKGRAY);
			GUI_AA_FillRoundedRect(Rect.x0, Rect.y0, Rect.x1, Rect.y1, 10);
			GUI_SetBkColor(GUI_DARKGRAY);
			GUI_SetColor(GUI_WHITE);
		}
		else
		{
			GUI_SetColor(GUI_STCOLOR_LIGHTBLUE);
			GUI_AA_FillRoundedRect(Rect.x0, Rect.y0, Rect.x1, Rect.y1, 10);

			GUI_SetColor(0XEBCD9E);
			GUI_SetPenSize(2);
			GUI_AA_DrawRoundedRect(Rect.x0, Rect.y0, Rect.x1, Rect.y1, 10);

			GUI_SetBkColor(GUI_STCOLOR_LIGHTBLUE);
			GUI_SetColor(GUI_BLACK);
		}

		BUTTON_GetText(hWin, buf, 20);
		GUI_SetFont(&GUI_FontLubalGraph24B);
		GUI_DispStringInRect(buf, &Rect, GUI_TA_HCENTER | GUI_TA_VCENTER);
		break;

	default:
		BUTTON_Callback(pMsg);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: DSO_Init
*	����˵��: ʾ�����������ʼ��
*	��    ��: ucCreateFlag  1:��ʾ��Ҫ���������ʹ��ڵȡ�
*                           0:����Ҫ������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void DSO_Init(uint8_t ucCreateFlag)
{
	char buf[10];
	uint32_t ulTrigPos;

	/* ��1����ˢ�±���*********************************************************************/
	GUI_SetBkColor(0x905040);
	GUI_Clear();

	GUI_MEMDEV_WriteAt(hMemDSO, 40, 40);

	/* ��2������ʾ��������Ϣ***************************************************************/
	GUI_SetBkColor(0x905040);
	GUI_SetColor(GUI_WHITE);
	GUI_SetFont(&GUI_FontLubalGraph24B);
	GUI_DispStringInRect("DSO2.0", &rClient, GUI_TA_HCENTER | GUI_TA_VCENTER);

	/* �Զ�����ģʽ */
	if (TriggerFlag == 0)
	{
		/* ����K2 :���ò�����ʾ���л���ͣ */
		if (g_Flag->hWinRunStop == 0)
		{
			GUI_DispStringInRect("Run", &rRunMode, GUI_TA_HCENTER | GUI_TA_VCENTER);
		}
		else
		{
			GUI_DispStringInRect("Stop", &rRunMode, GUI_TA_HCENTER | GUI_TA_VCENTER);
		}
	}
	/* ��ͨ����ģʽ */
	else
	{
		/* ����K2 :���ò�����ʾ���л���ͣ */
		if (g_Flag->ucTrig == 0)
		{
			GUI_DispStringInRect("Run", &rRunMode, GUI_TA_HCENTER | GUI_TA_VCENTER);
		}
		else
		{
			GUI_DispStringInRect("Stop", &rRunMode, GUI_TA_HCENTER | GUI_TA_VCENTER);
			;
		}
	}

	/* ����K3 :������ͨ������ʽ���Զ����� */
	if (TriggerFlag == 0)
	{
		GUI_DispStringInRect("Auto", &rTrigMode, GUI_TA_HCENTER | GUI_TA_VCENTER);
	}
	else
	{
		GUI_DispStringInRect("Trig", &rTrigMode, GUI_TA_HCENTER | GUI_TA_VCENTER);
	}

	/* ��3������ʾ�Զ������Ĵ�����ѹ**********************************************************/
	g_TrigVol->ufTrigValue = 240 - g_TrigVol->usPos;
	g_TrigVol->ufTrigValue = g_TrigVol->ufTrigValue * g_AttTable[Ch1AmpId][1] / 50000;
	sprintf(buf, "%5.3fV", g_TrigVol->ufTrigValue);

	GUI_DispStringInRect(buf, &rTrigValue, GUI_TA_HCENTER | GUI_TA_VCENTER);

	/* ��ʾ�����ش����ı�־ */
	GUI_DrawHLine(rTrigValue.y1 - 10, rTrigValue.x0 + 10, rTrigValue.x0 + 19);
	GUI_DrawLine(rTrigValue.x0 + 19, rTrigValue.y1 - 10, rTrigValue.x0 + 30, rTrigValue.y0 + 8);
	GUI_DrawHLine(rTrigValue.y0 + 8, rTrigValue.x0 + 31, rTrigValue.x0 + 41);

	/* ��4��������ҡ�˰����ĵ���״̬����������ʾ����******************************************/
	if (g_Flag->hWinButState == 0)
	{
		GUI_DispStringInRect("AdjFreq", &rButState, GUI_TA_HCENTER | GUI_TA_VCENTER);
	}
	else if (g_Flag->hWinButState == 1)
	{
		GUI_DispStringInRect("AdjAmplitude", &rButState, GUI_TA_HCENTER | GUI_TA_VCENTER);
	}
	else if (g_Flag->hWinButState == 2)
	{
		GUI_DispStringInRect("ChangeRefPos", &rButState, GUI_TA_HCENTER | GUI_TA_VCENTER);
	}
	else if (g_Flag->hWinButState == 3)
	{
		GUI_DispStringInRect("ChangeCursorVA", &rButState, GUI_TA_HCENTER | GUI_TA_VCENTER);
	}
	else if (g_Flag->hWinButState == 4)
	{
		GUI_DispStringInRect("ChangeCursorVB", &rButState, GUI_TA_HCENTER | GUI_TA_VCENTER);
	}
	else if (g_Flag->hWinButState == 5)
	{
		GUI_DispStringInRect("ChangeCursorHA", &rButState, GUI_TA_HCENTER | GUI_TA_VCENTER);
	}
	else if (g_Flag->hWinButState == 6)
	{
		GUI_DispStringInRect("ChangeCursorHB", &rButState, GUI_TA_HCENTER | GUI_TA_VCENTER);
	}
	else if (g_Flag->hWinButState == 7)
	{
		GUI_DispStringInRect("ChangeTrigger", &rButState, GUI_TA_HCENTER | GUI_TA_VCENTER);
	}

	/* ��5����ʵ�ֲ��εķ���***************************************************************/
	GUI_SetBkColor(GUI_BLACK);
	GUI_ClearRect(210, 6, 470, 33);
	GUI_SetColor(GUI_YELLOW);
	GUI_DrawHLine(20, 220, 220 + 239);
	GUI_DrawHLine(21, 220, 220 + 239);

	GUI_SetColor(0x0040f0);

	/* �Զ�����ģʽ */
	if (TriggerFlag == 0)
	{
		if ((g_DSO1->sCurTriStep + g_DSO1->sCurTriPos) < 0)
		{
			ulTrigPos = 0;
		}
		else if ((g_DSO1->sCurTriStep + g_DSO1->sCurTriPos) > 1448)
		{
			ulTrigPos = 1448 * 240;
		}
		else
		{
			ulTrigPos = (g_DSO1->sCurTriStep + g_DSO1->sCurTriPos) * 240;
		}
	}
	/* ��ͨ����ģʽ */
	else
	{
		ulTrigPos = (724 + g_DSO1->sCurTriStep) * 240;
	}

	/* ��������������ݵĴ���λ�����������ϵĴ���ͼ��λ�� */
	ulTrigPos = ulTrigPos / 2048;
	GUI_FillPolygon(&aPointsTrigBrowser[0], GUI_COUNTOF(aPointsTrigBrowser), ulTrigPos + 220, 13);

	/* ��¼ר�ŵĴ���λ�� */
	GUI_SetColor(GUI_RED);
	GUI_DrawPixel(326, 20);
	GUI_DrawPixel(326, 21);

	GUI_DrawPixel(327, 20);
	GUI_DrawPixel(327, 21);

	GUI_DrawPixel(328, 20);
	GUI_DrawPixel(328, 21);

	GUI_DrawPixel(329, 20);
	GUI_DrawPixel(329, 21);

	GUI_DrawPixel(330, 20);
	GUI_DrawPixel(330, 21);

	/* ��6����������ʾ���ı߿�*************************************************************/
	GUI_SetColor(0XEBCD9E);
	GUI_DrawRect(DSOSCREEN_STARTX - 1, /* ���Ͻ�X��λ�� */
				 DSOSCREEN_STARTY - 1, /* ���Ͻ�Y��λ�� */
				 DSOSCREEN_ENDX + 1,   /* ���½�X��λ�� */
				 DSOSCREEN_ENDY + 1);  /* ���½�Y��λ�� */

	GUI_SetColor(0XB37F63);
	GUI_DrawRect(DSOSCREEN_STARTX - 2, /* ���Ͻ�X��λ�� */
				 DSOSCREEN_STARTY - 2, /* ���Ͻ�Y��λ�� */
				 DSOSCREEN_ENDX + 2,   /* ���½�X��λ�� */
				 DSOSCREEN_ENDY + 2);  /* ���½�Y��λ�� */

	/* ������Ҫ�Ƿ���Ҫ���´��������ʹ��� */
	if (ucCreateFlag == 1)
	{
		/* ��7��������״̬����*************************************************************/
		hDlgAmp = CreateAmplitudeDlg();
		hDlgStatus1 = CreateStatus1Dlg();
		hDlgStatus2 = CreateStatus2Dlg();
		hDlgScale = CreateScaleDlg();
		hDlgSysInfo = CreateSysInfoDlg();

		//		/* ��6����������Ҫ�İ�ť*************************************************************/
		hButton0 = BUTTON_Create(670, 40, 100, 44, GUI_ID_BUTTON0, WM_CF_SHOW);
		BUTTON_SetText(hButton0, "Measure");
		WM_SetHasTrans(hButton0);
		WM_SetCallback(hButton0, _cbButton);

		hButton1 = BUTTON_Create(670, 90 - 1, 100, 44, GUI_ID_BUTTON1, WM_CF_SHOW);
		BUTTON_SetText(hButton1, "ADC");
		WM_SetHasTrans(hButton1);
		WM_SetCallback(hButton1, _cbButton);

		hButton2 = BUTTON_Create(670, 140 - 2, 100, 44, GUI_ID_BUTTON2, WM_CF_SHOW);
		BUTTON_SetText(hButton2, "DAC");
		WM_SetHasTrans(hButton2);
		WM_SetCallback(hButton2, _cbButton);

		hButton3 = BUTTON_Create(670, 190 - 3, 100, 44, GUI_ID_BUTTON3, WM_CF_SHOW);
		BUTTON_SetText(hButton3, "Math");
		WM_SetHasTrans(hButton3);
		WM_SetCallback(hButton3, _cbButton);

		hButton4 = BUTTON_Create(670, 240 - 4, 100, 44, GUI_ID_BUTTON4, WM_CF_SHOW);
		BUTTON_SetText(hButton4, "Settings");
		WM_SetHasTrans(hButton4);
		WM_SetCallback(hButton4, _cbButton);
	}

	/* ��8������ʾ�ο�����*************************************************************/
	GUI_SetColor(GUI_YELLOW);
	GUI_FillPolygon(&aPoints[0], GUI_COUNTOF(aPoints), 5, g_DSO1->usRefPos);

	GUI_SetColor(GUI_BLACK);
	GUI_SetFont(&GUI_Font20_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);
	GUI_DispCharAt('1', 10, g_DSO1->usRefPos - 9);

	GUI_SetColor(GUI_GREEN);
	GUI_FillPolygon(&aPoints[0], GUI_COUNTOF(aPoints), 5, g_DSO2->usRefPos);

	GUI_SetColor(GUI_BLACK);
	GUI_SetFont(&GUI_Font20_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);
	GUI_DispCharAt('2', 10, g_DSO2->usRefPos - 9);

	GUI_SetColor(GUI_BLACK);
	GUI_FillPolygon(&aPoints[0], GUI_COUNTOF(aPoints), 5, 430);
	GUI_SetColor(GUI_RED);
	GUI_DrawPolygon(&aPoints[0], GUI_COUNTOF(aPoints), 5, 430);

	GUI_SetColor(GUI_RED);
	GUI_SetFont(&GUI_Font20_1);
	GUI_SetTextMode(GUI_TEXTMODE_TRANS);
	GUI_DispCharAt('M', 9, 430 - 9);
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
