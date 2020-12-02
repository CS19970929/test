/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块。
*	文件名称 : main.c
*	版    本 : V1.0
*	说    明 : FreeRTOS操作系统版本的二代示波器
*              实验目的：
*                1. 学习FreeRTOS操作系统版本的二代示波器实现。
*              实验内容：
*                1. 关于示波器的讲解和代码实现请看本实例配套的二代示波器教程。
*                2. 默认情况下K1按键是用于触摸校准的，如果需要用于打印任务执行情况，需要在MainTask.c文件
*                   的函数DSO_Graph里面使能相应的条件编译。使能后全编译工程，下载到板子里面按下按键K1可以
*                   通过串口打印任务执行情况（波特率115200，数据位8，奇偶校验位无，停止位1）
*                   =================================================
*                   任务名      任务状态 优先级   剩余栈 任务序号
*                   vTaskDSO        R       5       546     5
*                   vTaskStart      R       4       488     4
*                   vTaskGUI        R       1       743     1
*                   IDLE            R       0       107     6
*                   vTaskUserIF     B       2       489     2
*                   vTaskMsgPro     S       3       481     3
*                   Tmr Svc         B       6       223     7
*                   
*                   
*                   任务名       运行计数         使用率
*                   vTaskDSO        91681           4%
*                   vTaskStart      11643           <1%
*                   vTaskGUI        1642038         71%
*                   IDLE            535318          23%
*                   vTaskUserIF     0               <1%
*                   Tmr Svc         1               <1%
*                   vTaskMsgPro     0               <1%
*                   
*                   当前动态内存剩余大小 = 10568字节
*                   
*                  串口软件建议使用SecureCRT（V6光盘里面有此软件）查看打印信息。
*                  各个任务实现的功能如下：
*                   vTaskDSO        任务: 双通道示波器数据处理任务。
*                   vTaskGUI        任务: emWin任务。
*                   vTaskTaskUserIF 任务: 保留，暂时未用到。	
*                   vTaskMsgPro     任务: 实现截图功能，将图片以BMP格式保存到SD卡中。
*                   vTaskStart      任务: 启动任务，也就是最高优先级任务，这里实现按键扫描和触摸检测。
*                2. 任务运行状态的定义如下，跟上面串口打印字母B, R, D, S对应：
*                    #define tskBLOCKED_CHAR		( 'B' )  阻塞
*                    #define tskREADY_CHAR		    ( 'R' )  就绪
*                    #define tskDELETED_CHAR		( 'D' )  删除
*                    #define tskSUSPENDED_CHAR	    ( 'S' )  挂起
*              注意事项：
*                1. 本实验推荐使用串口软件SecureCRT，要不串口打印效果不整齐。此软件在
*                   V6开发板光盘里面有。
*                2. 务必将编辑器的缩进参数和TAB设置为4来阅读本文件，要不代码显示不整齐。
*
*	修改记录 :
*		版本号    日期         作者            说明
*       V1.0    2018-01-06   Eric2013    1. ST固件库到V1.6.1版本
*                                        2. BSP驱动包V1.2
*                                        3. FreeRTOS版本V8.2.3
*                                        4. STemWin版本V5.32
*                                        5. FatFS版本V0.11a
*                                        6. DSP库版本V1.5.2
*
*	Copyright (C), 2018-2028, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#include "includes.h"
#include "MainTask.h"

/*
**********************************************************************************************************
		                          FreeRTOS的任务栈空间使用CCM RAM区
**********************************************************************************************************
*/
/*
  64KB的CCM RAM区全部用于任务栈，使用CCM RAM的好处就是速度比通用RAM要快些，而任务栈是需要频繁使用的，
  所以将任务栈定位到CCM RAM将加快处理速度，缺点是这部分空间不支持DMA操作。初次使用的用户比较容易在这
  个地方犯错误。所有在使用局部变量时，切勿将局部变量用于DMA传输。
*/
#if defined(__CC_ARM) /* MDK编译器 */
uint8_t ucHeap[64 * 1024] __attribute__((at(0x10000000)));
#elif defined(__ICCARM__) /* IAR编译器 */
#pragma location = 0x10000000
uint8_t ucHeap[64 * 1024];
#endif

/*
**********************************************************************************************************
											函数声明
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
											  变量
**********************************************************************************************************
*/
/* 任务句柄 */
static TaskHandle_t xHandleTaskGUI = NULL;
static TaskHandle_t xHandleTaskUserIF = NULL;
static TaskHandle_t xHandleTaskStart = NULL;
TaskHandle_t xHandleTaskMsgPro = NULL;
TaskHandle_t xHandleTaskDSO = NULL;

/* 消息句柄 */
static SemaphoreHandle_t xMutex = NULL;
EventGroupHandle_t xCreatedEventGroup = NULL;

arm_rfft_fast_instance_f32 S;
uint32_t fftSize = 2048;
uint32_t ifftFlag = 0;

/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
int main(void)
{
	/* 
	  在启动调度前，为了防止初始化STM32外设时有中断服务程序执行，这里禁止全局中断(除了NMI和HardFault)。
	  这样做的好处是：
	  1. 防止执行的中断服务程序中有FreeRTOS的API函数。
	  2. 保证系统正常启动，不受别的中断影响。
	  3. 关于是否关闭全局中断，大家根据自己的实际情况设置即可。
	  在移植文件port.c中的函数prvStartFirstTask中会重新开启全局中断。通过指令cpsie i开启，__set_PRIMASK(1)
	  和cpsie i是等效的。
     */
	__set_PRIMASK(1);

	/* 硬件初始化 */
	bsp_Init();

	/* 1. 初始化一个定时器中断，精度高于滴答定时器中断，这样才可以获得准确的系统信息 仅供调试目的，实际项
		  目中不要使用，因为这个功能比较影响系统实时性。
	   2. 为了正确获取FreeRTOS的调试信息，可以考虑将上面的关闭中断指令__set_PRIMASK(1); 注释掉。 
	*/
	vSetupSysInfoTest();

	/* 创建任务通信机制 */
	AppObjCreate();

	/* 创建任务 */
	AppTaskCreate();

	/* 启动调度，开始执行任务 */
	vTaskStartScheduler();

	/* 
	  如果系统正常启动是不会运行到这里的，运行到这里极有可能是用于定时器任务或者空闲任务的
	  heap空间不足造成创建失败，此要加大FreeRTOSConfig.h文件中定义的heap大小：
	  #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 17 * 1024 ) )
	*/
	while (1)
		;
}

/*
*********************************************************************************************************
*	函 数 名: vTaskGUI
*	功能说明: emWin任务
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 1  (数值越小优先级越低，这个跟uCOS相反)
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
*	函 数 名: vTaskTaskUserIF
*	功能说明: 保留，暂时未用到。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 2 
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
*	函 数 名: vTaskMsgPro
*	功能说明: 实现截图功能，将图片以BMP格式保存到SD卡中
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 4  
*********************************************************************************************************
*/
static void vTaskMsgPro(void *pvParameters)
{
	uint8_t Pic_Name = 0;
	uint32_t ulStart, ulEnd;
	char buf[20];

	while (1)
	{
		ulTaskNotifyTake(pdTRUE,		 /* 此参数设置为pdTRUE，接收到的notification value清零 */
						 portMAX_DELAY); /* 无限等待 */

		sprintf(buf, "0:/PicSave/%d.bmp", Pic_Name);

		/* 记录截图前起始时间 */
		ulStart = xTaskGetTickCount();

		/* 开启调度锁 */
		//vTaskSuspendAll();

		/* 如果SD卡中没有PicSave文件，会进行创建 */
		result = f_mkdir("0:/PicSave");
		/* 创建截图 */
		result = f_open(&file, buf, FA_WRITE | FA_CREATE_ALWAYS);
		/* 向SD卡绘制BMP图片 */
		GUI_BMP_Serialize(_WriteByte2File, &file);

		/* 创建完成后关闭file */
		result = f_close(&file);

		/* 关闭调度锁 */
		//xTaskResumeAll ();

		/* 记录截图后时间并获取截图过程耗时 */
		ulEnd = xTaskGetTickCount();
		ulEnd -= ulStart;

		App_Printf("截图完成，耗时 = %dms\r\n", ulEnd);
		Pic_Name++;
	}
}

/*
*********************************************************************************************************
*	函 数 名: vTaskStart
*	功能说明: 启动任务，主要实现按键检测和触摸检测。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 5  
*********************************************************************************************************
*/
static void vTaskStart(void *pvParameters)
{
	uint8_t ucCount = 0;

	while (1)
	{
		/* 1ms一次触摸扫描，电阻触摸屏 */
		if (g_tTP.Enable == 1)
		{
			TOUCH_Scan();

			/* 按键扫描 */
			ucCount++;
			if (ucCount == 10)
			{
				ucCount = 0;
				bsp_KeyScan();
			}
			vTaskDelay(1);
		}

		/* 10ms一次触摸扫描，电容触摸屏GT811 */
		if (g_GT811.Enable == 1)
		{
			bsp_KeyScan();
			GT811_OnePiontScan();
			vTaskDelay(10);
		}

		/* 10ms一次触摸扫描，电容触摸屏FT5X06 */
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
*	函 数 名: AppTaskDSO
*	功能说明: 双通道示波器数据处理任务。
*	形    参: pvParameters 是在创建该任务时传递的形参
*	返 回 值: 无
*   优 先 级: 6  
*********************************************************************************************************
*/
static void vTaskDSO(void *pvParameters)
{
	EventBits_t uxBits;
	uint8_t pcWriteBuffer[512];

	/* 实数序列FFT长度 */
	fftSize = 2048;

	/* 正变换 */
	ifftFlag = 0;

	/* 初始化结构体S中的参数 */
	arm_rfft_fast_init_f32(&S, fftSize);

	while (1)
	{
		/* 等待所有任务发来事件标志 */
		uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* 事件标志组句柄 */
									 0xFFFF,			 /* 等待0xFFFF某一位被设置 */
									 pdTRUE,			 /* 退出前0xFFFF位被清除，这里是任意0xFFFF位被设置就“退出”*/
									 pdFALSE,			 /* 设置为pdTRUE表示等待0xFFFF任意位被设置*/
									 portMAX_DELAY);	 /* 等待延迟时间 */
		switch (uxBits)
		{
		/* 双通道波形处理 */
		case DspFFT2048Pro_15:
			/* 读取的是ADC3的位置 */
			g_DSO1->usCurPos = 10240 - DMA2_Stream1->NDTR;

			/* 读取的是ADC1的位置 */
			g_DSO2->usCurPos = 10240 - DMA2_Stream0->NDTR;

			DSO2_WaveTrig(g_DSO2->usCurPos);
			DSO1_WaveTrig(g_DSO1->usCurPos);
			DSO2_WaveProcess();
			DSO1_WaveProcess();
			break;

		/* 用于简单的ADC数据采集 */
		case DspMultiMeterPro_14:
			g_uiAdcAvgSample = ADC_GetSampleAvgN();
			break;

		/* 仅用于调试目的，打印任务的执行情况，默认不使用 */
		case DspTaskInfo_13:
			App_Printf("=================================================\r\n");
			App_Printf("任务名      任务状态 优先级   剩余栈 任务序号\r\n");
			vTaskList((char *)&pcWriteBuffer);
			App_Printf("%s\r\n", pcWriteBuffer);

			App_Printf("\r\n任务名       运行计数         使用率\r\n");
			vTaskGetRunTimeStats((char *)&pcWriteBuffer);
			App_Printf("%s\r\n", pcWriteBuffer);
			App_Printf("当前动态内存剩余大小 = %d字节\r\n", xPortGetFreeHeapSize());
			break;

		/* 其它位暂未使用 */
		default:
			App_Printf("*ucReceive = %x\r\n", uxBits);
			break;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppTaskCreate(void)
{
	xTaskCreate(vTaskGUI,		 /* 任务函数  */
				"vTaskGUI",		 /* 任务名    */
				1024,			 /* 任务栈大小，单位word，也就是4字节 */
				NULL,			 /* 任务参数  */
				1,				 /* 任务优先级*/
				xHandleTaskGUI); /* 任务句柄  */

	xTaskCreate(vTaskTaskUserIF,	 /* 任务函数  */
				"vTaskUserIF",		 /* 任务名    */
				512,				 /* 任务栈大小，单位word，也就是4字节 */
				NULL,				 /* 任务参数  */
				2,					 /* 任务优先级*/
				&xHandleTaskUserIF); /* 任务句柄  */

	xTaskCreate(vTaskMsgPro,		 /* 任务函数  */
				"vTaskMsgPro",		 /* 任务名    */
				512,				 /* 任务栈大小，单位word，也就是4字节 */
				NULL,				 /* 任务参数  */
				3,					 /* 任务优先级*/
				&xHandleTaskMsgPro); /* 任务句柄  */

	xTaskCreate(vTaskStart,			/* 任务函数  */
				"vTaskStart",		/* 任务名    */
				512,				/* 任务栈大小，单位word，也就是4字节 */
				NULL,				/* 任务参数  */
				4,					/* 任务优先级*/
				&xHandleTaskStart); /* 任务句柄  */

	xTaskCreate(vTaskDSO,			/* 任务函数  */
				"vTaskDSO",			/* 任务名    */
				1024,				/* 任务栈大小，单位word，也就是4字节 */
				NULL,				/* 任务参数  */
				5,					/* 任务优先级*/
				&xHandleTaskStart); /* 任务句柄  */
}

/*
*********************************************************************************************************
*	函 数 名: AppObjCreate
*	功能说明: 创建任务通信机制
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppObjCreate(void)
{
	/* 创建互斥信号量 */
	xMutex = xSemaphoreCreateMutex();

	if (xMutex == NULL)
	{
		/* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
	}

	/* 创建事件标志组 */
	xCreatedEventGroup = xEventGroupCreate();

	if (xCreatedEventGroup == NULL)
	{
		/* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
	}

	/* 申请示波器通道1动态内存 */
	g_DSO1 = (DSO_T *)pvPortMalloc(sizeof(DSO_T));

	/* 申请示波器通道2动态内存 */
	g_DSO2 = (DSO_T *)pvPortMalloc(sizeof(DSO_T));

	/* 申请游标测量结构体变量动态内存 */
	g_Cursors = (CURSORS_T *)pvPortMalloc(sizeof(CURSORS_T));

	/* 申请标志位结构体变量动态内存 */
	g_Flag = (FLAG_T *)pvPortMalloc(sizeof(FLAG_T));

	/* 申请触发结构体变量动态内存 */
	g_TrigVol = (TRIVOLTAGE_T *)pvPortMalloc(sizeof(TRIVOLTAGE_T));

	/* 申请FFT动态内存 */
	testInput_fft_2048 = (float32_t *)pvPortMalloc(sizeof(float32_t) * 2048);
	testOutput_fft_2048 = (float32_t *)pvPortMalloc(sizeof(float32_t) * 2048);

	/* 申请RMS动态内存 */
	g_RMSBUF = (float32_t *)pvPortMalloc(sizeof(float32_t) * 600);

	/* 申请FIR动态内存 */
	FirDataInput = (float32_t *)pvPortMalloc(sizeof(float32_t) * FIR_LENGTH_SAMPLES);
	FirDataOutput = (float32_t *)pvPortMalloc(sizeof(float32_t) * FIR_LENGTH_SAMPLES);
	firStateF32 = (float32_t *)pvPortMalloc(sizeof(float32_t) * FIR_StateBufSize);
}

/*
*********************************************************************************************************
*	函 数 名: App_Printf
*	功能说明: 线程安全的printf方式		  			  
*	形    参: 同printf的参数。
*             在C中，当无法列出传递函数的所有实参的类型和数目时,可以用省略号指定参数表
*	返 回 值: 无
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

	/* 互斥信号量 */
	xSemaphoreTake(xMutex, portMAX_DELAY);

	printf("%s", buf_str);

	xSemaphoreGive(xMutex);
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
