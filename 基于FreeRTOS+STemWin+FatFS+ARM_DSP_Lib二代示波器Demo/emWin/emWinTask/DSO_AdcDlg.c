/*
*********************************************************************************************************
*	                                  
*	ģ������ : ʾ����ADC�Ի���
*	�ļ����� : DSO_AdcDlg.c
*	��    �� : V1.0
*	˵    �� : ʾ����ADC�Ի���
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
*                                       �궨��
*********************************************************************************************************
*/
#define ID_WINDOW_0 	(GUI_ID_USER + 0x00)
#define ID_TEXT_0 	    (GUI_ID_USER + 0x01)
#define ID_TEXT_1 	    (GUI_ID_USER + 0x02)
#define ID_TEXT_2 	    (GUI_ID_USER + 0x03)
#define ID_BUTTON_0     (GUI_ID_USER + 0x04)
#define ID_BUTTON_1     (GUI_ID_USER + 0x05)
#define ID_GRAPH_0      (GUI_ID_USER + 0x06)

#define ID_TEXT_3 	    (GUI_ID_USER + 0x07)


/*
*********************************************************************************************************
*	                                     ����
*********************************************************************************************************
*/
static GRAPH_SCALE_Handle hScaleV;     
static GRAPH_DATA_Handle  ahData;
static WM_HWIN hGraphADC;


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
				GUI_AA_FillCircle(100, 0, 72);
				GUI_SetBkColor(GUI_DARKGRAY);
				GUI_SetColor(GUI_WHITE); 
				GUI_DrawBitmap(&bmReturn, 40, 5);
			} 
			else 
			{
				GUI_SetColor(GUI_STCOLOR_LIGHTBLUE);
				GUI_AA_FillCircle(100, 0, 72);
				GUI_SetBkColor(GUI_STCOLOR_LIGHTBLUE);
				GUI_SetColor(GUI_WHITE);  
				GUI_DrawBitmap(&bmReturn, 40, 5);				
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
*	               				�Ի���ؼ��б�
*********************************************************************************************************
*/
static const GUI_WIDGET_CREATE_INFO _aDialogCreateADC[] = {
	{ WINDOW_CreateIndirect, "Window",         ID_WINDOW_0,   0,   0,   800,  480,  0, 0x0, 0},
	{ TEXT_CreateIndirect,   "",               ID_TEXT_0,   250,  75,   300,  100,  0, 0},
	{ TEXT_CreateIndirect,   "ADC1=1234",      ID_TEXT_1,   560,  135,  200,   50,  0, 0},
	{ GRAPH_CreateIndirect,  "Graph",          ID_GRAPH_0,   50,  200,  700,  250,  0, 0x0,  0},
	{ TEXT_CreateIndirect,   "Range:0V-3.3V",  ID_TEXT_2,    80,  230,  100,   20,  0, 0},
	{ BUTTON_CreateIndirect, "DlgBack",        ID_BUTTON_0, 700,    0,  100,  100,  0, 0, 0},	
	{ BUTTON_CreateIndirect, "MusicList",      ID_BUTTON_1,   0,    0,  100,  100,  0, 0, 0},
	{ TEXT_CreateIndirect,   "Text",           ID_TEXT_3,    200,  10,  400,   32,  0, 0x64, 0 },
};

/*
*********************************************************************************************************
*	�� �� ��: InitDialogReserved
*	����˵��: �Ի���ص������ĳ�ʼ����Ϣ
*	��    �Σ�pMsg   ��Ϣָ�����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void InitDialogADC(WM_MESSAGE * pMsg)
{
    WM_HWIN hWin = pMsg->hWin;
	WM_HWIN hItem;
	
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
	hItem = WM_GetDialogItem(hWin, ID_TEXT_0);
	TEXT_SetTextColor(hItem, GUI_RED);
	TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
	TEXT_SetFont(hItem, &GUI_FontNI7SEG90);
    TEXT_SetText(hItem, "0.000");
	
	hItem = WM_GetDialogItem(hWin, ID_TEXT_1);
	TEXT_SetTextColor(hItem, GUI_RED);
	TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
	TEXT_SetFont(hItem, &GUI_FontLubalGraph32);
	
	hItem = WM_GetDialogItem(hWin, ID_TEXT_2);
	TEXT_SetTextColor(hItem, GUI_GREEN);
	TEXT_SetTextAlign(hItem, GUI_TA_HCENTER);
	TEXT_SetFont(hItem, &GUI_Font16_1);
	
	hItem = WM_GetDialogItem(pMsg->hWin, ID_TEXT_3);
	TEXT_SetFont(hItem, &GUI_FontLubalGraph32);
	TEXT_SetTextColor(hItem, GUI_RED);
	TEXT_SetTextAlign(hItem, GUI_TA_HCENTER | GUI_TA_VCENTER);
	TEXT_SetText(hItem, "MultiMeter");

	//
	// ��ʼ��Graph�ؼ�
	//
	hGraphADC = WM_GetDialogItem(pMsg->hWin, ID_GRAPH_0);

	/* �������ݶ��� *******************************************************/
	ahData = GRAPH_DATA_YT_Create(GUI_GREEN, 700, 0, 0);

	/* ���ݶ�����ӵ�ͼ�οؼ� */
	GRAPH_AttachData(hGraphADC, ahData);

	/* ����Y�᷽���դ���� */
	GRAPH_SetGridDistY(hGraphADC, 50);

	/* �̶�X�᷽���դ�� */
	GRAPH_SetGridFixedX(hGraphADC, 1);

	/* ����դ��ɼ� */
	GRAPH_SetGridVis(hGraphADC, 1);

	GRAPH_SetLineStyleH(hGraphADC, GUI_LS_DOT);
	GRAPH_SetLineStyleV(hGraphADC, GUI_LS_DOT);

	/* �����̶ȶ���  ***************************************************/
	hScaleV = GRAPH_SCALE_Create(15, GUI_TA_RIGHT, GRAPH_SCALE_CF_VERTICAL, 50);

	/* ����ֱ�̶ȶ�����ӵ�ͼ�οؼ� */
	GRAPH_AttachScale(hGraphADC, hScaleV);

	/* �������ñ����̶ȵ����� */
	GRAPH_SCALE_SetFactor(hScaleV, 0.02);

	/* ���ñ�ǩ������ɫ */
	GRAPH_SCALE_SetTextColor(hScaleV, GUI_RED);

	GRAPH_SCALE_SetFont(hScaleV, &GUI_Font16_1);

	/* �����������ұ߽�Ĵ�С */
	GRAPH_SetBorder(hGraphADC, 20, 10, 10, 10);	
			
	/* 
	 * ������ʱ�����书���Ǿ���ָ�����ں���ָ�����ڷ�����Ϣ��
	 * �ö�ʱ����ָ������������� 
	 */
	WM_CreateTimer(hWin,   /* ������Ϣ�Ĵ��ڵľ�� */
				   0, 	         /* �û������Id���������ͬһ����ʹ�ö����ʱ������ֵ��������Ϊ�㡣 */
				   100,          /* ���ڣ������ڹ���ָ������Ӧ�յ���Ϣ*/
				   0);	         /* ��������ʹ�ã�ӦΪ0 */	
}

/*
*********************************************************************************************************
*	�� �� ��: _cbCallbackADC
*	����˵��: �ص�����
*	��    ��: pMsg   ��Ϣָ�����
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void _cbCallbackADC(WM_MESSAGE * pMsg) 
{
    int NCode, Id;
	char buf[20];
	WM_HWIN hItem;
	uint32_t temp;
	
	WM_HWIN hWin = pMsg->hWin;
	
    switch (pMsg->MsgId) 
	{
		/* �Ի���رպ�ʹ��һ�β���ˢ�£���ֹ�ڲ�����ͣ״̬��(ͨ��K2������������ͣ)
		   �رնԻ��򲢷��ص�ʱ�򣬲�������û��ˢ�£�һƬ�հס�
		*/
		case WM_DELETE:
			/* Graph�ؼ�Ҫ����ɾ�� */
			WM_DeleteWindow(hGraphADC);
		
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
            InitDialogADC(pMsg);
            break;
		
		/* ��ʱ���� */
		case WM_TIMER:  
			g_Flag->ucDsoMsg = DspMultiMeterPro_14;
			xEventGroupSetBits(xCreatedEventGroup, g_Flag->ucDsoMsg);
						
			sprintf(buf, "%5.3f", g_uiAdcAvgSample*3.3f/4095);
			hItem = WM_GetDialogItem(hWin, ID_TEXT_0);
			TEXT_SetText(hItem, buf);
		
			sprintf(buf, "ADC1=%d", g_uiAdcAvgSample);
			hItem = WM_GetDialogItem(hWin, ID_TEXT_1);
			TEXT_SetText(hItem, buf);
		
			temp = (uint32_t)(g_uiAdcAvgSample *165) /4095;
			GRAPH_DATA_YT_AddValue(ahData, temp);
			WM_RestartTimer(pMsg->Data.v, 400);
			break;

        case WM_NOTIFY_PARENT:
            Id = WM_GetId(pMsg->hWinSrc); 
            NCode = pMsg->Data.v;        
            switch (Id) 
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
            }
            break;
        default:
            WM_DefaultProc(pMsg);
    }
}

/*
*********************************************************************************************************
*	�� �� ��: DSO_CreateAdcDlg
*	����˵��: �����Ի���
*	��    ��: ��        	
*	�� �� ֵ: ��
*********************************************************************************************************
*/
WM_HWIN DSO_CreateAdcDlg(void) 
{
	WM_HWIN hWin;

	hWin = GUI_CreateDialogBox(_aDialogCreateADC, GUI_COUNTOF(_aDialogCreateADC), _cbCallbackADC, WM_HBKWIN, 0, 0);
	return hWin;
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
