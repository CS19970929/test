/*
*********************************************************************************************************
*
*	模块名称 : ASCII字符点阵
*	文件名称 : ASC12.c
*	版    本 : V1.0
*	说    明 :  ASCII字符点阵，来源与UCDOS的 ASCI12点阵字库
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2011-09-08 armfly  ST固件库V3.5.0版本。
*
*	Copyright (C), 2010-2011, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

unsigned char const g_Ascii12[] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	// ' '

	0x00,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x10,0x00,0x00,0x00,
	0x00,0x6c,0x48,0x48,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x14,0x14,0x28,0x7c,0x28,0x7c,0x28,0x50,0x50,0x00,0x00,
    0x00,0x10,0x38,0x40,0x40,0x38,0x48,0x70,0x10,0x10,0x00,0x00,
	0x00,0x20,0x50,0x20,0x0c,0x70,0x08,0x14,0x08,0x00,0x00,0x00,
	0x00,0x00,0x00,0x18,0x20,0x20,0x54,0x48,0x34,0x00,0x00,0x00,
	0x00,0x10,0x10,0x10,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x08,0x08,0x10,0x10,0x10,0x10,0x10,0x10,0x08,0x08,0x00,
	0x00,0x20,0x20,0x10,0x10,0x10,0x10,0x10,0x10,0x20,0x20,0x00,
	0x00,0x10,0x7c,0x10,0x28,0x28,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x10,0x10,0x10,0xfc,0x10,0x10,0x10,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x10,0x30,0x20,0x00,
	0x00,0x00,0x00,0x00,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,
	0x00,0x04,0x04,0x08,0x08,0x10,0x10,0x20,0x20,0x40,0x00,0x00,
	0x00,0x38,0x44,0x44,0x44,0x44,0x44,0x44,0x38,0x00,0x00,0x00,
	0x00,0x30,0x10,0x10,0x10,0x10,0x10,0x10,0x7c,0x00,0x00,0x00,
	0x00,0x38,0x44,0x04,0x08,0x10,0x20,0x44,0x7c,0x00,0x00,0x00,
	0x00,0x38,0x44,0x04,0x18,0x04,0x04,0x44,0x38,0x00,0x00,0x00,
    0x00,0x0c,0x14,0x14,0x24,0x44,0x7c,0x04,0x0c,0x00,0x00,0x00,0x00,0x3c,0x20,0x20,
    0x38,0x04,0x04,0x44,0x38,0x00,0x00,0x00,0x00,0x1c,0x20,0x40,0x78,0x44,0x44,0x44,
    0x38,0x00,0x00,0x00,0x00,0x7c,0x44,0x04,0x08,0x08,0x08,0x10,0x10,0x00,0x00,0x00,
    0x00,0x38,0x44,0x44,0x38,0x44,0x44,0x44,0x38,0x00,0x00,0x00,0x00,0x38,0x44,0x44,
    0x44,0x3c,0x04,0x08,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x30,
    0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x18,0x30,0x20,0x00,0x00,
    0x00,0x00,0x0c,0x10,0x60,0x80,0x60,0x10,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x7c,0x00,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x20,0x18,0x04,0x18,0x20,
    0xc0,0x00,0x00,0x00,0x00,0x00,0x18,0x24,0x04,0x08,0x10,0x00,0x30,0x00,0x00,0x00,
    0x38,0x44,0x44,0x4c,0x54,0x54,0x4c,0x40,0x44,0x38,0x00,0x00,0x00,0x30,0x10,0x28,
    0x28,0x28,0x7c,0x44,0xec,0x00,0x00,0x00,0x00,0xf8,0x44,0x44,0x78,0x44,0x44,0x44,
    0xf8,0x00,0x00,0x00,0x00,0x3c,0x44,0x40,0x40,0x40,0x40,0x44,0x38,0x00,0x00,0x00,
    0x00,0xf0,0x48,0x44,0x44,0x44,0x44,0x48,0xf0,0x00,0x00,0x00,0x00,0xfc,0x44,0x50,
    0x70,0x50,0x40,0x44,0xfc,0x00,0x00,0x00,0x00,0x7c,0x20,0x28,0x38,0x28,0x20,0x20,
    0x70,0x00,0x00,0x00,0x00,0x3c,0x44,0x40,0x40,0x4c,0x44,0x44,0x38,0x00,0x00,0x00,
    0x00,0xec,0x44,0x44,0x7c,0x44,0x44,0x44,0xec,0x00,0x00,0x00,0x00,0x7c,0x10,0x10,
    0x10,0x10,0x10,0x10,0x7c,0x00,0x00,0x00,0x00,0x3c,0x08,0x08,0x08,0x48,0x48,0x48,
    0x30,0x00,0x00,0x00,0x00,0xec,0x44,0x48,0x50,0x70,0x48,0x44,0xe4,0x00,0x00,0x00,
    0x00,0x70,0x20,0x20,0x20,0x20,0x24,0x24,0x7c,0x00,0x00,0x00,0x00,0xec,0x6c,0x6c,
    0x54,0x54,0x44,0x44,0xec,0x00,0x00,0x00,0x00,0xec,0x64,0x64,0x54,0x54,0x54,0x4c,
    0xec,0x00,0x00,0x00,0x00,0x38,0x44,0x44,0x44,0x44,0x44,0x44,0x38,0x00,0x00,0x00,
    0x00,0x78,0x24,0x24,0x24,0x38,0x20,0x20,0x70,0x00,0x00,0x00,0x00,0x38,0x44,0x44,
    0x44,0x44,0x44,0x44,0x38,0x1c,0x00,0x00,0x00,0xf8,0x44,0x44,0x44,0x78,0x48,0x44,
    0xe0,0x00,0x00,0x00,0x00,0x34,0x4c,0x40,0x38,0x04,0x04,0x64,0x58,0x00,0x00,0x00,
    0x00,0xfc,0x90,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00,0x00,0x00,0xec,0x44,0x44,
    0x44,0x44,0x44,0x44,0x38,0x00,0x00,0x00,0x00,0xec,0x44,0x44,0x28,0x28,0x28,0x10,
    0x10,0x00,0x00,0x00,0x00,0xec,0x44,0x44,0x54,0x54,0x54,0x54,0x28,0x00,0x00,0x00,
    0x00,0xc4,0x44,0x28,0x10,0x10,0x28,0x44,0xc4,0x00,0x00,0x00,0x00,0xec,0x44,0x28,
    0x28,0x10,0x10,0x10,0x38,0x00,0x00,0x00,0x00,0x7c,0x44,0x08,0x10,0x10,0x20,0x44,
    0x7c,0x00,0x00,0x00,0x00,0x38,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x38,0x00,
    0x00,0x40,0x20,0x20,0x20,0x10,0x10,0x08,0x08,0x08,0x00,0x00,0x00,0x38,0x08,0x08,
    0x08,0x08,0x08,0x08,0x08,0x08,0x38,0x00,0x00,0x10,0x10,0x28,0x44,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfc,
    0x00,0x10,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x38,
    0x44,0x3c,0x44,0x44,0x3c,0x00,0x00,0x00,0x00,0xc0,0x40,0x58,0x64,0x44,0x44,0x44,
    0xf8,0x00,0x00,0x00,0x00,0x00,0x00,0x3c,0x44,0x40,0x40,0x44,0x38,0x00,0x00,0x00,
    0x00,0x0c,0x04,0x34,0x4c,0x44,0x44,0x44,0x3c,0x00,0x00,0x00,0x00,0x00,0x00,0x38,
    0x44,0x7c,0x40,0x40,0x3c,0x00,0x00,0x00,0x00,0x1c,0x20,0x7c,0x20,0x20,0x20,0x20,
    0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x34,0x4c,0x44,0x44,0x44,0x3c,0x04,0x38,0x00,
    0x00,0xc0,0x40,0x58,0x64,0x44,0x44,0x44,0xec,0x00,0x00,0x00,0x00,0x10,0x00,0x70,
    0x10,0x10,0x10,0x10,0x7c,0x00,0x00,0x00,0x00,0x10,0x00,0x78,0x08,0x08,0x08,0x08,
    0x08,0x08,0x70,0x00,0x00,0xc0,0x40,0x5c,0x48,0x70,0x50,0x48,0xdc,0x00,0x00,0x00,
    0x00,0x30,0x10,0x10,0x10,0x10,0x10,0x10,0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0xe8,
    0x54,0x54,0x54,0x54,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0xd8,0x64,0x44,0x44,0x44,
    0xec,0x00,0x00,0x00,0x00,0x00,0x00,0x38,0x44,0x44,0x44,0x44,0x38,0x00,0x00,0x00,
    0x00,0x00,0x00,0xd8,0x64,0x44,0x44,0x44,0x78,0x40,0xe0,0x00,0x00,0x00,0x00,0x34,
    0x4c,0x44,0x44,0x44,0x3c,0x04,0x0c,0x00,0x00,0x00,0x00,0x6c,0x30,0x20,0x20,0x20,
    0x7c,0x00,0x00,0x00,0x00,0x00,0x00,0x3c,0x44,0x38,0x04,0x44,0x78,0x00,0x00,0x00,
    0x00,0x00,0x20,0x7c,0x20,0x20,0x20,0x20,0x1c,0x00,0x00,0x00,0x00,0x00,0x00,0xcc,
    0x44,0x44,0x44,0x4c,0x34,0x00,0x00,0x00,0x00,0x00,0x00,0xec,0x44,0x44,0x28,0x28,
    0x10,0x00,0x00,0x00,0x00,0x00,0x00,0xec,0x44,0x54,0x54,0x54,0x28,0x00,0x00,0x00,
    0x00,0x00,0x00,0xcc,0x48,0x30,0x30,0x48,0xcc,0x00,0x00,0x00,0x00,0x00,0x00,0xec,
    0x44,0x24,0x28,0x18,0x10,0x10,0x78,0x00,0x00,0x00,0x00,0x7c,0x48,0x10,0x20,0x44,
    0x7c,0x00,0x00,0x00,0x00,0x08,0x10,0x10,0x10,0x10,0x20,0x10,0x10,0x10,0x08,0x00,
    0x00,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00,0x20,0x10,0x10,
    0x10,0x10,0x08,0x10,0x10,0x10,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x24,0x58,0x00,
    0x00,0x00,0x00,0x00,
};
