/*
*********************************************************************************************************
*
*	ģ������ : ������ģ�顣
*	�ļ����� : main.c
*	��    �� : V1.0
*	˵    �� : FreeRTOS����ϵͳ�汾�Ķ���ʾ����
*              ʵ��Ŀ�ģ�
*                1. ѧϰFreeRTOS����ϵͳ�汾�Ķ���ʾ����ʵ�֡�
*              ʵ�����ݣ�
*                1. ����ʾ�����Ľ���ʹ���ʵ���뿴��ʵ�����׵Ķ���ʾ�����̡̳�
*                2. Ĭ�������K1���������ڴ���У׼�ģ������Ҫ���ڴ�ӡ����ִ���������Ҫ��MainTask.c�ļ�
*                   �ĺ���DSO_Graph����ʹ����Ӧ���������롣ʹ�ܺ�ȫ���빤�̣����ص��������水�°���K1����
*                   ͨ�����ڴ�ӡ����ִ�������������115200������λ8����żУ��λ�ޣ�ֹͣλ1��
*                   =================================================
*                   ������      ����״̬ ���ȼ�   ʣ��ջ �������
*                   vTaskDSO        R       5       546     5
*                   vTaskStart      R       4       488     4
*                   vTaskGUI        R       1       743     1
*                   IDLE            R       0       107     6
*                   vTaskUserIF     B       2       489     2
*                   vTaskMsgPro     S       3       481     3
*                   Tmr Svc         B       6       223     7
*                   
*                   
*                   ������       ���м���         ʹ����
*                   vTaskDSO        91681           4%
*                   vTaskStart      11643           <1%
*                   vTaskGUI        1642038         71%
*                   IDLE            535318          23%
*                   vTaskUserIF     0               <1%
*                   Tmr Svc         1               <1%
*                   vTaskMsgPro     0               <1%
*                   
*                   ��ǰ��̬�ڴ�ʣ���С = 10568�ֽ�
*                   
*                  �����������ʹ��SecureCRT��V6���������д�������鿴��ӡ��Ϣ��
*                  ��������ʵ�ֵĹ������£�
*                   vTaskDSO        ����: ˫ͨ��ʾ�������ݴ�������
*                   vTaskGUI        ����: emWin����
*                   vTaskTaskUserIF ����: ��������ʱδ�õ���	
*                   vTaskMsgPro     ����: ʵ�ֽ�ͼ���ܣ���ͼƬ��BMP��ʽ���浽SD���С�
*                   vTaskStart      ����: ��������Ҳ����������ȼ���������ʵ�ְ���ɨ��ʹ�����⡣
*                2. ��������״̬�Ķ������£������洮�ڴ�ӡ��ĸB, R, D, S��Ӧ��
*                    #define tskBLOCKED_CHAR		( 'B' )  ����
*                    #define tskREADY_CHAR		    ( 'R' )  ����
*                    #define tskDELETED_CHAR		( 'D' )  ɾ��
*                    #define tskSUSPENDED_CHAR	    ( 'S' )  ����
*              ע�����
*                1. ��ʵ���Ƽ�ʹ�ô������SecureCRT��Ҫ�����ڴ�ӡЧ�������롣�������
*                   V6��������������С�
*                2. ��ؽ��༭��������������TAB����Ϊ4���Ķ����ļ���Ҫ��������ʾ�����롣
*
*	�޸ļ�¼ :
*		�汾��    ����         ����            ˵��
*       V1.0    2018-01-06   Eric2013    1. ST�̼��⵽V1.6.1�汾
*                                        2. BSP������V1.2
*                                        3. FreeRTOS�汾V8.2.3
*                                        4. STemWin�汾V5.32
*                                        5. FatFS�汾V0.11a
*                                        6. DSP��汾V1.5.2
*
*	Copyright (C), 2018-2028, ���������� www.armfly.com
*
*********************************************************************************************************
*/
#include "includes.h"
#include "MainTask.h"

/*
**********************************************************************************************************
		                          FreeRTOS������ջ�ռ�ʹ��CCM RAM��
**********************************************************************************************************
*/
/*
  64KB��CCM RAM��ȫ����������ջ��ʹ��CCM RAM�ĺô������ٶȱ�ͨ��RAMҪ��Щ��������ջ����ҪƵ��ʹ�õģ�
  ���Խ�����ջ��λ��CCM RAM���ӿ촦���ٶȣ�ȱ�����ⲿ�ֿռ䲻֧��DMA����������ʹ�õ��û��Ƚ���������
  ���ط�������������ʹ�þֲ�����ʱ�����𽫾ֲ���������DMA���䡣
*/
#if defined(__CC_ARM) /* MDK������ */
uint8_t ucHeap[64 * 1024] __attribute__((at(0x10000000)));
#elif defined(__ICCARM__) /* IAR������ */
#pragma location = 0x10000000
uint8_t ucHeap[64 * 1024];
#endif

/*
**********************************************************************************************************
											��������
**********************************************************************************************************
*/
static void vTaskGUI(void *pvParameters);
static void vTaskTaskUserIF(void *pvParameters);
static void vTaskMsgPro(void *pvParameters);
static void vTaskStart(void *pvParameters);
static void vTaskDSO(void *pvParameters);
static void AppTaskCreate(void);
static void AppObjCreate(void);
static void App_Printf(char *format, ...);

/*
**********************************************************************************************************
											  ����
**********************************************************************************************************
*/
/* ������ */
static TaskHandle_t xHandleTaskGUI = NULL;
static TaskHandle_t xHandleTaskUserIF = NULL;
static TaskHandle_t xHandleTaskStart = NULL;
TaskHandle_t xHandleTaskMsgPro = NULL;
TaskHandle_t xHandleTaskDSO = NULL;

/* ��Ϣ��� */
static SemaphoreHandle_t xMutex = NULL;
EventGroupHandle_t xCreatedEventGroup = NULL;

arm_rfft_fast_instance_f32 S;
uint32_t fftSize = 2048;
uint32_t ifftFlag = 0;

/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: ��׼c������ڡ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int main(void)
{
	/* 
	  ����������ǰ��Ϊ�˷�ֹ��ʼ��STM32����ʱ���жϷ������ִ�У������ֹȫ���ж�(����NMI��HardFault)��
	  �������ĺô��ǣ�
	  1. ��ִֹ�е��жϷ����������FreeRTOS��API������
	  2. ��֤ϵͳ�������������ܱ���ж�Ӱ�졣
	  3. �����Ƿ�ر�ȫ���жϣ���Ҹ����Լ���ʵ��������ü��ɡ�
	  ����ֲ�ļ�port.c�еĺ���prvStartFirstTask�л����¿���ȫ���жϡ�ͨ��ָ��cpsie i������__set_PRIMASK(1)
	  ��cpsie i�ǵ�Ч�ġ�
     */
	__set_PRIMASK(1);

	/* Ӳ����ʼ�� */
	bsp_Init();

	/* 1. ��ʼ��һ����ʱ���жϣ����ȸ��ڵδ�ʱ���жϣ������ſ��Ի��׼ȷ��ϵͳ��Ϣ ��������Ŀ�ģ�ʵ����
		  Ŀ�в�Ҫʹ�ã���Ϊ������ܱȽ�Ӱ��ϵͳʵʱ�ԡ�
	   2. Ϊ����ȷ��ȡFreeRTOS�ĵ�����Ϣ�����Կ��ǽ�����Ĺر��ж�ָ��__set_PRIMASK(1); ע�͵��� 
	*/
	vSetupSysInfoTest();

	/* ��������ͨ�Ż��� */
	AppObjCreate();

	/* �������� */
	AppTaskCreate();

	/* �������ȣ���ʼִ������ */
	vTaskStartScheduler();

	/* 
	  ���ϵͳ���������ǲ������е�����ģ����е����Ｋ�п��������ڶ�ʱ��������߿��������
	  heap�ռ䲻����ɴ���ʧ�ܣ���Ҫ�Ӵ�FreeRTOSConfig.h�ļ��ж����heap��С��
	  #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 17 * 1024 ) )
	*/
	while (1)
		;
}

/*
*********************************************************************************************************
*	�� �� ��: vTaskGUI
*	����˵��: emWin����
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 1  (��ֵԽС���ȼ�Խ�ͣ������uCOS�෴)
*********************************************************************************************************
*/
static void vTaskGUI(void *pvParameters)
{
	while (1)
	{
		MainTask();
	}
}

/*
*********************************************************************************************************
*	�� �� ��: vTaskTaskUserIF
*	����˵��: ��������ʱδ�õ���
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 2 
*********************************************************************************************************
*/
static void vTaskTaskUserIF(void *pvParameters)
{
	while (1)
	{
		vTaskDelay(1000);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: vTaskMsgPro
*	����˵��: ʵ�ֽ�ͼ���ܣ���ͼƬ��BMP��ʽ���浽SD����
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 4  
*********************************************************************************************************
*/
static void vTaskMsgPro(void *pvParameters)
{
	uint8_t Pic_Name = 0;
	uint32_t ulStart, ulEnd;
	char buf[20];

	while (1)
	{
		ulTaskNotifyTake(pdTRUE,		 /* �˲�������ΪpdTRUE�����յ���notification value���� */
						 portMAX_DELAY); /* ���޵ȴ� */

		sprintf(buf, "0:/PicSave/%d.bmp", Pic_Name);

		/* ��¼��ͼǰ��ʼʱ�� */
		ulStart = xTaskGetTickCount();

		/* ���������� */
		//vTaskSuspendAll();

		/* ���SD����û��PicSave�ļ�������д��� */
		result = f_mkdir("0:/PicSave");
		/* ������ͼ */
		result = f_open(&file, buf, FA_WRITE | FA_CREATE_ALWAYS);
		/* ��SD������BMPͼƬ */
		GUI_BMP_Serialize(_WriteByte2File, &file);

		/* ������ɺ�ر�file */
		result = f_close(&file);

		/* �رյ����� */
		//xTaskResumeAll ();

		/* ��¼��ͼ��ʱ�䲢��ȡ��ͼ���̺�ʱ */
		ulEnd = xTaskGetTickCount();
		ulEnd -= ulStart;

		App_Printf("��ͼ��ɣ���ʱ = %dms\r\n", ulEnd);
		Pic_Name++;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: vTaskStart
*	����˵��: ����������Ҫʵ�ְ������ʹ�����⡣
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 5  
*********************************************************************************************************
*/
static void vTaskStart(void *pvParameters)
{
	uint8_t ucCount = 0;

	while (1)
	{
		/* 1msһ�δ���ɨ�裬���败���� */
		if (g_tTP.Enable == 1)
		{
			TOUCH_Scan();

			/* ����ɨ�� */
			ucCount++;
			if (ucCount == 10)
			{
				ucCount = 0;
				bsp_KeyScan();
			}
			vTaskDelay(1);
		}

		/* 10msһ�δ���ɨ�裬���ݴ�����GT811 */
		if (g_GT811.Enable == 1)
		{
			bsp_KeyScan();
			GT811_OnePiontScan();
			vTaskDelay(10);
		}

		/* 10msһ�δ���ɨ�裬���ݴ�����FT5X06 */
		if (g_tFT5X06.Enable == 1)
		{
			bsp_KeyScan();
			FT5X06_OnePiontScan();
			vTaskDelay(10);
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: AppTaskDSO
*	����˵��: ˫ͨ��ʾ�������ݴ�������
*	��    ��: pvParameters ���ڴ���������ʱ���ݵ��β�
*	�� �� ֵ: ��
*   �� �� ��: 6  
*********************************************************************************************************
*/
static void vTaskDSO(void *pvParameters)
{
	EventBits_t uxBits;
	uint8_t pcWriteBuffer[512];

	/* ʵ������FFT���� */
	fftSize = 2048;

	/* ���任 */
	ifftFlag = 0;

	/* ��ʼ���ṹ��S�еĲ��� */
	arm_rfft_fast_init_f32(&S, fftSize);

	while (1)
	{
		/* �ȴ������������¼���־ */
		uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* �¼���־���� */
									 0xFFFF,			 /* �ȴ�0xFFFFĳһλ������ */
									 pdTRUE,			 /* �˳�ǰ0xFFFFλ�����������������0xFFFFλ�����þ͡��˳���*/
									 pdFALSE,			 /* ����ΪpdTRUE��ʾ�ȴ�0xFFFF����λ������*/
									 portMAX_DELAY);	 /* �ȴ��ӳ�ʱ�� */
		switch (uxBits)
		{
		/* ˫ͨ�����δ��� */
		case DspFFT2048Pro_15:
			/* ��ȡ����ADC3��λ�� */
			g_DSO1->usCurPos = 10240 - DMA2_Stream1->NDTR;

			/* ��ȡ����ADC1��λ�� */
			g_DSO2->usCurPos = 10240 - DMA2_Stream0->NDTR;

			DSO2_WaveTrig(g_DSO2->usCurPos);
			DSO1_WaveTrig(g_DSO1->usCurPos);
			DSO2_WaveProcess();
			DSO1_WaveProcess();
			break;

		/* ���ڼ򵥵�ADC���ݲɼ� */
		case DspMultiMeterPro_14:
			g_uiAdcAvgSample = ADC_GetSampleAvgN();
			break;

		/* �����ڵ���Ŀ�ģ���ӡ�����ִ�������Ĭ�ϲ�ʹ�� */
		case DspTaskInfo_13:
			App_Printf("=================================================\r\n");
			App_Printf("������      ����״̬ ���ȼ�   ʣ��ջ �������\r\n");
			vTaskList((char *)&pcWriteBuffer);
			App_Printf("%s\r\n", pcWriteBuffer);

			App_Printf("\r\n������       ���м���         ʹ����\r\n");
			vTaskGetRunTimeStats((char *)&pcWriteBuffer);
			App_Printf("%s\r\n", pcWriteBuffer);
			App_Printf("��ǰ��̬�ڴ�ʣ���С = %d�ֽ�\r\n", xPortGetFreeHeapSize());
			break;

		/* ����λ��δʹ�� */
		default:
			App_Printf("*ucReceive = %x\r\n", uxBits);
			break;
		}
	}
}

/*
*********************************************************************************************************
*	�� �� ��: AppTaskCreate
*	����˵��: ����Ӧ������
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AppTaskCreate(void)
{
	xTaskCreate(vTaskGUI,		 /* ������  */
				"vTaskGUI",		 /* ������    */
				1024,			 /* ����ջ��С����λword��Ҳ����4�ֽ� */
				NULL,			 /* �������  */
				1,				 /* �������ȼ�*/
				xHandleTaskGUI); /* ������  */

	xTaskCreate(vTaskTaskUserIF,	 /* ������  */
				"vTaskUserIF",		 /* ������    */
				512,				 /* ����ջ��С����λword��Ҳ����4�ֽ� */
				NULL,				 /* �������  */
				2,					 /* �������ȼ�*/
				&xHandleTaskUserIF); /* ������  */

	xTaskCreate(vTaskMsgPro,		 /* ������  */
				"vTaskMsgPro",		 /* ������    */
				512,				 /* ����ջ��С����λword��Ҳ����4�ֽ� */
				NULL,				 /* �������  */
				3,					 /* �������ȼ�*/
				&xHandleTaskMsgPro); /* ������  */

	xTaskCreate(vTaskStart,			/* ������  */
				"vTaskStart",		/* ������    */
				512,				/* ����ջ��С����λword��Ҳ����4�ֽ� */
				NULL,				/* �������  */
				4,					/* �������ȼ�*/
				&xHandleTaskStart); /* ������  */

	xTaskCreate(vTaskDSO,			/* ������  */
				"vTaskDSO",			/* ������    */
				1024,				/* ����ջ��С����λword��Ҳ����4�ֽ� */
				NULL,				/* �������  */
				5,					/* �������ȼ�*/
				&xHandleTaskStart); /* ������  */
}

/*
*********************************************************************************************************
*	�� �� ��: AppObjCreate
*	����˵��: ��������ͨ�Ż���
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void AppObjCreate(void)
{
	/* ���������ź��� */
	xMutex = xSemaphoreCreateMutex();

	if (xMutex == NULL)
	{
		/* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
	}

	/* �����¼���־�� */
	xCreatedEventGroup = xEventGroupCreate();

	if (xCreatedEventGroup == NULL)
	{
		/* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
	}

	/* ����ʾ����ͨ��1��̬�ڴ� */
	g_DSO1 = (DSO_T *)pvPortMalloc(sizeof(DSO_T));

	/* ����ʾ����ͨ��2��̬�ڴ� */
	g_DSO2 = (DSO_T *)pvPortMalloc(sizeof(DSO_T));

	/* �����α�����ṹ�������̬�ڴ� */
	g_Cursors = (CURSORS_T *)pvPortMalloc(sizeof(CURSORS_T));

	/* �����־λ�ṹ�������̬�ڴ� */
	g_Flag = (FLAG_T *)pvPortMalloc(sizeof(FLAG_T));

	/* ���봥���ṹ�������̬�ڴ� */
	g_TrigVol = (TRIVOLTAGE_T *)pvPortMalloc(sizeof(TRIVOLTAGE_T));

	/* ����FFT��̬�ڴ� */
	testInput_fft_2048 = (float32_t *)pvPortMalloc(sizeof(float32_t) * 2048);
	testOutput_fft_2048 = (float32_t *)pvPortMalloc(sizeof(float32_t) * 2048);

	/* ����RMS��̬�ڴ� */
	g_RMSBUF = (float32_t *)pvPortMalloc(sizeof(float32_t) * 600);

	/* ����FIR��̬�ڴ� */
	FirDataInput = (float32_t *)pvPortMalloc(sizeof(float32_t) * FIR_LENGTH_SAMPLES);
	FirDataOutput = (float32_t *)pvPortMalloc(sizeof(float32_t) * FIR_LENGTH_SAMPLES);
	firStateF32 = (float32_t *)pvPortMalloc(sizeof(float32_t) * FIR_StateBufSize);
}

/*
*********************************************************************************************************
*	�� �� ��: App_Printf
*	����˵��: �̰߳�ȫ��printf��ʽ		  			  
*	��    ��: ͬprintf�Ĳ�����
*             ��C�У����޷��г����ݺ���������ʵ�ε����ͺ���Ŀʱ,������ʡ�Ժ�ָ��������
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void App_Printf(char *format, ...)
{
	char buf_str[512 + 1];
	va_list v_args;

	va_start(v_args, format);
	(void)vsnprintf((char *)&buf_str[0],
					(size_t)sizeof(buf_str),
					(char const *)format,
					v_args);
	va_end(v_args);

	/* �����ź��� */
	xSemaphoreTake(xMutex, portMAX_DELAY);

	printf("%s", buf_str);

	xSemaphoreGive(xMutex);
}

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
