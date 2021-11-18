
/********************************** (C) COPYRIGHT *******************************
* File Name          :CompatibilityHID.C
* Author             : WCH
* Version            : V1.2
* Date               : 2018/02/28
* Description        : CH554模拟HID兼容设备，支持中断上下传，支持控制端点上下传，支持设置全速，低速 
*******************************************************************************/

#include "CH554.H"
#include "Debug.H"
#include <stdio.h>
#include <string.h>

#define Fullspeed 1
#define THIS_ENDP0_SIZE 8 //低速USB，中断传输、控制传输最大包长度为8

//#define DEBUG
#if DE_PRINTF == 2
    #define PRINT_DEBUG(s) sendStrToUart(s,sizeof(s))
#else
    #define PRINT_DEBUG(s) ;
#endif


#if DE_PRINTF != 1
#pragma asm
?PR?Uart0_ISR?FTDI   SEGMENT CODE 
PUBLIC	Uart0_ISR
#pragma endasm
#endif

#if DE_PRINTF == 1
BOOL print_pin_change = FALSE;
#endif

UINT8X Ep0Buffer[DEFAULT_ENDP0_SIZE] _at_(0x0000);   //端点0 OUT&IN缓冲区，必须是偶地址
UINT8X Ep1Buffer[MAX_PACKET_SIZE]    _at_(0x0040);	//端点1 IN 发送缓冲区
UINT8X Ep2Buffer[MAX_PACKET_SIZE]    _at_(0x0200);   //端点2 OUT接收缓冲区
UINT8X RingBuf[128]                  _at_(0x0100);

USB_SETUP_REQ SetupReqBuf; //暂存Setup包
PUINT8  pDescr;             //USB配置标志

UINT8 SetupReq, UsbConfig;
volatile UINT8I pDescr_Index = 0;
volatile UINT8I VendorControl = 0;
volatile UINT8I ReDFU_Flag = 0;
volatile UINT8I Lantency_Timer = 4;
volatile UINT16I SOF_Count = 0;
volatile UINT16I SetupLen = 0;

volatile UINT8 USBOutLength = 0;
volatile UINT8 USBOutPtr = 0;
volatile UINT8 USBReceived = 0;

volatile UINT8 WritePtr = 0;
volatile UINT8 ReadPtr = 0;

volatile UINT8I FTDI_State = 0;
volatile BOOL   FTDI_Open = FALSE;
volatile BOOL   DTR_read = 0;
volatile BOOL   RTS_read = 0;
volatile BOOL   Hard_write = FALSE;
volatile BOOL   EP1_Busy_Flag = FALSE;

sbit RTSPin = P1^0;
sbit DTRPin = P1^7; 

#define FTDI_READY  0x07
#define FTDI_START  0x08

#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)

#pragma NOAREGS
/*设备描述符*/
UINT8C DevDesc[18] = {
        0x12,              //bLength
        0x01,               //bDescriptorType
        0x10, 0x01,        //bcdUSB
        0x00,              //bDeviceClass
        0x00,              //bDeviceSubClass
        0x00,              //bDeviceProtocol

        0x08,       //bMaxPackeSize0
        0x03, 0x04, //idVender
        0x01, 0x60, //idProduct
        0x00, 0x05, //bcdDevice

        0x01, //iManufacturer
        0x02, //iProduct

        0x03, //iSerialNumber
        0x01, //bNumConfigurations
};

UINT8C CfgDesc[32] =
    {
        0x09,                 //bLength
        0x02,                 //bDescriptorType

        32,                   //wTotalLength
        0,    

        0x01,                 //bNumInterfaces
        0x01,                 //bConfigurationValue
        0x00,                 //iConfiguartion
        0x80,                 //bmAttributes
        0x32,                 //bMaxPower
        //--------------------------------------------------------USB_Interface_Descriport
        0x09,                 //bLength
        0x04,                 //bDescriptorType
        0x00,                 //bInterfaceNumbuder
        0x00,                 //bAlternateSetting
        0x02,                 //bNumEndpoints
        0xff,                 //bInterfaceClass
        0xff,                 //bInterfaceSubClass
        0xff,                 //bInterfaceProtocol
        0x00,                 //iConfiguration

        //--------------------------------------------------------USB_EndPont_Descriport
        //
        //  ------------------------------
        //  EP1 BULK IN
        //  ------------------------------

        0x07,               //bLength
        0x05,               //bDescriptorType
        0x81,               //bEndpointAddress
        0x02,               //bmAttributes

        0x40,
        0x00,               //wMAXPacketSize
        0x00,

        //  ------------------------------
        //  EP2 BULK OUT
        //  ------------------------------
        0x07,               //bLength
        0x05,               //bDescriptorType
        0x02,               //bEndpointAddress
        0x02,               //bmAttributes

        0x40,
        0x00,               //wMAXPacketSize
        0x00,
};
/*字符串描述符 略*/

UINT8C USB_LanguageID[4] =
{
    0x04, //bLength
    0x03,
    0x09, 0x04
};

UINT8C USB_ProductString[16]=
{ 
    0x10,0x03,  
    0x4d,0x00,  //M  
    0x35,0x00,  //5  
    0x73,0x00,  //s  
    0x74,0x00,  //t  
    0x61,0x00,  //a  
    0x63,0x00,  //c  
    0x6b,0x00,  //k
};

UINT8C USB_ManufacturerStiring[20]=
{ 
    0x14,0x03,  
    0x48,0x00,  //H  
    0x61,0x00,  //a  
    0x64,0x00,  //d  
    0x65,0x00,  //e  
    0x73,0x00,  //s  
    0x32,0x00,  //2  
    0x30,0x00,  //0  
    0x30,0x00,  //0  
    0x31,0x00,  //1  
  
};

UINT8 Endp2Busy = 0;

UINT8C hexCode[16] = "0123456789ABCDEF";
//CODE\FTDI.C(302): warning C214: 'Argument': conversion: pointer to non-pointer
//CODE\FTDI.C(310): error C214: illegal pointer conversion
void uuidcpy(UINT8X *dest, UINT8 index, UINT8 len) /* 使用UUID生成USB Serial Number */
{
	UINT8 i;
	UINT8 p = 0; /* UUID格式, 十位十六进制数 */
	UINT8C *puuid;
	for(i = index; i < (index + len); i++)
	{
		if(i == 0)
			dest[p++] = 22; //10 * 2 + 2
		else if(i == 1)
			dest[p++] = 0x03;
		else
		{
			if(i & 0x01) //奇数
			{
				dest[p++] = 0x00;
			}
			else
			{
				puuid = (UINT8C *) (0x3ffa + (i - 2) / 4);
				if(i & 0x02)
					dest[p++] = hexCode[(*puuid) >> 4];
				else
					dest[p++] = hexCode[(*puuid) & 0x0f];
			}
		}
	}
}

void sendStrToUart(const char* str,UINT8 length)
{
    memcpy(Ep2Buffer,str,length);
    USBReceived = 1;
    USBOutPtr = 0;
    USBOutLength = length;
}

void SetBuad()
{
    #ifndef DEBUG
    UINT16 divisor;
    PCON |= SMOD; //波特率加倍
    T2MOD |= bTMR_CLK; //最高计数时钟

    divisor = UsbSetupBuf->wValueL |
                (UsbSetupBuf->wValueH << 8);
    divisor &= 0x3fff; //没法发生小数取整数部分，baudrate = 48M/16/divisor
    if(divisor == 0 || divisor == 1)
    {
        TH1 = 0xff; //实在憋不出来1.5M
    }
    else
    {
        divisor = divisor / 2; //24M CPU时钟
        if(divisor > 256)
        {
            //TH1 = 0 - 13; //统统115200
            divisor /= 8;
            if(divisor > 256)
            {
                TH1 = 0 - 13;
            }
            else
            {
                PCON &= ~(SMOD);
                T2MOD &= ~(bTMR_CLK); //低波特率
                TH1 = 0 - divisor;
            }
        }
        else
        {
            TH1 = 0 - divisor;
        }
    } 
    #endif
}

/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : USB设备模式配置,设备模式启动，收发端点配置，中断开启
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
	USB_CTRL = 0x00;												    //清空USB控制寄存器
	USB_CTRL &= ~bUC_HOST_MODE;											//该位为选择设备模式
	USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;				//USB设备和内部上拉使能,在中断期间中断标志未清除前自动返回NAK
	USB_DEV_AD = 0x00;												    //设备地址初始化

	USB_CTRL &= ~bUC_LOW_SPEED;
	UDEV_CTRL &= ~bUD_LOW_SPEED;									    //选择全速12M模式，默认方式
	UDEV_CTRL = bUD_PD_DIS;  // 禁止DP/DM下拉电阻
	UDEV_CTRL |= bUD_PORT_EN;	

 	UEP2_DMA = (UINT16) Ep2Buffer;										//端点2 OUT接收数据传输地址
	UEP2_3_MOD = 0x08;													//端点2 单缓冲接收
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;							//端点2 自动翻转同步标志位，OUT返回ACK

	UEP1_DMA = (UINT16) Ep1Buffer;										//端点1 IN 发送数据传输地址
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;							//端点1 自动翻转同步标志位，IN事务返回NAK
	UEP4_1_MOD = 0x40;													//端点1 单缓冲发送

	UEP0_DMA = (UINT16) Ep0Buffer;										//端点0数据传输地址
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;							//手动翻转，OUT事务返回ACK，IN事务返回NAK

    USB_INT_EN |= bUIE_SUSPEND;											//使能设备挂起中断
	USB_INT_EN |= bUIE_TRANSFER;										//使能USB传输完成中断
	USB_INT_EN |= bUIE_BUS_RST;											//使能设备模式USB总线复位中断
	USB_INT_EN |= bUIE_DEV_SOF;											//打开SOF中断
	USB_INT_FG |= 0x1F;													//清中断标志
	IE_USB = 1;															//使能USB中断
	EA = 1;																//允许单片机中断
}

/*******************************************************************************
* Function Name  : Enp2BlukIn()
* Description    : USB设备模式端点2的批量上传
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Enp2BlukIn()
{
    /*
    memcpy(Ep2Buffer + MAX_PACKET_SIZE, UserEp2Buf, sizeof(UserEp2Buf)); //加载上传数据
    UEP2_T_LEN = THIS_ENDP0_SIZE;                                        //上传最大包长度
    UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;             //有数据时上传数据并应答ACK
    */
}


UINT8 EPLength = 0xff;
void getStringDescReport()
{
    switch( UsbSetupBuf->wValueL )
    {
        case 0: //语言ID
            PRINT_DEBUG("Re-sL\n");
            pDescr = USB_LanguageID;
            EPLength = sizeof(USB_LanguageID);
            break;
        case 1: //获取厂商描述
            PRINT_DEBUG("Re-sM\n");
            pDescr = USB_ManufacturerStiring;
            EPLength = sizeof(USB_ManufacturerStiring);
            break;
        case 2: //获取产品描述
            PRINT_DEBUG("Re-sP\n");
            pDescr = USB_ProductString;
            EPLength = sizeof(USB_ProductString);
        break;
        case 3: //获取序列号描述
            PRINT_DEBUG("Re-sD\n");
            pDescr = (UINT8C *)0xffff;
            EPLength =22;
            //pDescr = USB_Serial_number;
            //EPLength = sizeof(USB_Serial_number);
        break;
        default:
            pDescr = (UINT8C *)0xffff;
            EPLength =22;
            break;
    }
}

void getDescReport()
{
    switch (UsbSetupBuf->wValueH)
    {
    case 1:               //设备描述符
        PRINT_DEBUG("Re-dD\n");
        pDescr = DevDesc; //把设备描述符送到要发送的缓冲区
        EPLength = sizeof(DevDesc);
        break;
    case 2:               //配置描述符
        PRINT_DEBUG("Re-dIf\n");
        pDescr = CfgDesc; //把设备描述符送到要发送的缓冲区
        EPLength = sizeof(CfgDesc);
        break;
    case 3:               //字符串描述符
        getStringDescReport();
        break;
    default:
        EPLength = 0xff; //不支持的命令或者出错
        break;
    }
    if (SetupLen > EPLength)
    {
        SetupLen = EPLength; //限制总长度
    }
    EPLength = ( SetupLen >= THIS_ENDP0_SIZE ) ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
    if( pDescr == ( UINT8C *) 0xffff )
    {
        uuidcpy(Ep0Buffer, 0, EPLength);
    }
    else
    {
        memcpy(Ep0Buffer, pDescr, EPLength);                                 //加载上传数据
    }
    SetupLen -= EPLength;
    //pDescr += EPLength;
    pDescr_Index = EPLength;
}

void cleanFeature()
{
    if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE )				  /* 清除设备 */
    {
        if( ( ( ( UINT16 )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
        {
            if( CfgDesc[ 7 ] & 0x20 )
            {
                /* 唤醒 */
            }
            else
            {
                EPLength = 0xFF;										/* 操作失败 */
            }
        }
        else
        {
            EPLength = 0xFF;											/* 操作失败 */
        }
    }
    if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) // 端点
    {
        switch (UsbSetupBuf->wIndexL)
        {
        case 0x81:
            UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
            EP1_Busy_Flag = FALSE;
            break;
        case 0x02:
            UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
            break;
        default:
            EPLength = 0xFF; // 不支持的端点
            break;
        }
    }
    else
    {
        EPLength = 0xFF; // 不是端点不支持
    }
}

void setFeature()
{
    if ((UsbSetupBuf->bRequestType & 0x1F) == 0x00) /* 设置设备 */
    {
        if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
        {
            if (CfgDesc[7] & 0x20)
            {
                while ( XBUS_AUX & bUART0_TX )
                {
                    ;	//等待发送完成
                }
                SAFE_MOD = 0x55;
                SAFE_MOD = 0xAA;
                WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;					  //USB或者RXD0/1有信号时可被唤醒
                PCON |= PD;																 //睡眠
                SAFE_MOD = 0x55;
                SAFE_MOD = 0xAA;
                WAKE_CTRL = 0x00;
            }
            else EPLength = 0xFF; /* 操作失败 */
        }
        else EPLength = 0xFF; /* 操作失败 */
    }
    else if ((UsbSetupBuf->bRequestType & 0x1F) == 0x02) /* 设置端点 */
    {
        if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
        {
            switch (((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
            {
            case 0x02:
                UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
                break;
            case 0x81:
                UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
                break;
            default:
                EPLength = 0xFF; /* 操作失败 */
                break;
            }
        }
        else
        {
            EPLength = 0xFF; /* 操作失败 */
        }
    }
    else
    {
        EPLength = 0xFF; /* 操作失败 */
    }
}

void SetHadrPin()
{
    if(DTR_read == 1 && RTS_read == 1)
    {
        //TXD1 = 1;                       //G0 = 1
        //CAP1 = 1;                       //EN = 1
    }
    if(DTR_read == 0 && RTS_read == 0)
    {
        TXD1 = 1;                       //G0 = 1
        CAP1 = 1;                       //EN = 1
    }
    if(DTR_read == 0 && RTS_read == 1)
    {
        TXD1 = 1;                       //G0 = 1
        CAP1 = 0;                       //
    }
    if(DTR_read == 1 && RTS_read == 0)
    {
        TXD1 = 0;
        CAP1 = 1;
    }
    
    #if DE_PRINTF == 1
    print_pin_change = TRUE;
    #endif
    
}

void Ftdi_Feature()
{
    if(UsbSetupBuf->bRequestType & USB_REQ_TYP_READ)
    {
        //读
        switch( SetupReq )
        {
        case 0x90: //READ EEPROM
            Ep0Buffer[0] = 0xff;
            Ep0Buffer[1] = 0xff;
            EPLength = 2;
            break;
        case 0x0a:
            Ep0Buffer[0] = Lantency_Timer;
            EPLength = 1;
            break;
        case 0x05:
            Ep0Buffer[0] = 0x01;
            Ep0Buffer[1] = 0x60;
            EPLength = 2;
            break;
        default:
            EPLength = 0xFF;
            break;
        }
    }
    else
    {
        //写
        switch( SetupReq )
        {
        case 0x02:
            FTDI_Open = TRUE;
            PRINT_DEBUG("Ft-Op\n");
            if(( FTDI_State == FTDI_START )&&( FTDI_Open == TRUE ))
            {
                //SetHadrPin();
            }
            break;
        case 0x04:
            FTDI_State = UsbSetupBuf->wValueL;
            break;
        case 0x06:
            FTDI_Open = FALSE;
            break;
        case 0x07:
        case 0x0b:
        case 0x92:
            EPLength = 0;
            break;
        case 0x91: //WRITE EEPROM, FT_PROG动作,直接跳转BL
            ReDFU_Flag = 1;
            EPLength = 0;
            break;
        case 0x00:
            //UpPoint1_Busy = 0;
            EPLength = 0;
            break;
        case 0x09: //SET LATENCY TIMER
            Lantency_Timer = UsbSetupBuf->wValueL;
            EPLength = 0;
            break;
        case 0x03:
            SetBuad();
            EPLength = 0;
            break;
        case 0x01: //MODEM Control
            if( UsbSetupBuf->wValueH & 0x01 )
            {
                if( UsbSetupBuf->wValueL & 0x01 )
                {
                    DTR_read = 1;
                }
                else
                {
                    DTR_read = 0;
                }
            }
            if( UsbSetupBuf->wValueH & 0x02 )
            {
                if( UsbSetupBuf->wValueL & 0x02 )
                {
                    RTS_read = 1;
                }
                else
                {
                    RTS_read = 0;
                }
                Hard_write = TRUE;
            }
            if(( FTDI_State == FTDI_START )&&( FTDI_Open == TRUE )&&(Hard_write == TRUE))
            {
                Hard_write = FALSE;
                SetHadrPin();
            }
            EPLength = 0;
            break;
        default:
            EPLength = 0xFF;
            break;
        }
    }
}

void SETUPInterrupt(void)
{
    EPLength = USB_RX_LEN;
    if( EPLength == (sizeof(USB_SETUP_REQ)))
    {
        //sendSETUPData();
        SetupLen = ((UINT16)UsbSetupBuf->wLengthH << 8) | (UsbSetupBuf->wLengthL);
        EPLength = 0; // 默认为成功并且上传0长度
        VendorControl = 0;
        SetupReq = UsbSetupBuf->bRequest;
        if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD)
        {
            VendorControl = 1;
            Ftdi_Feature();
        }
        else
        {
            switch( SetupReq )
            {
                case USB_GET_DESCRIPTOR:
                    getDescReport();
                    break;
                case USB_SET_ADDRESS:
                    SetupLen = UsbSetupBuf->wValueL;//暂存USB设备地址
                    break;
                case USB_GET_CONFIGURATION:         //获取配置
                    Ep0Buffer[0] = UsbConfig;
                    if (SetupLen >= 1)
                    {
                        EPLength = 1;
                    }
                    break;
                case USB_SET_CONFIGURATION:         //设置配置
                    UsbConfig = UsbSetupBuf->wValueL;
                    break;
                case USB_GET_INTERFACE:
                    break;
                case USB_CLEAR_FEATURE:	//Clear Feature
                    cleanFeature();
                    break;
                case USB_SET_FEATURE:   //Set Feature
                    setFeature();
                    break;
                case USB_GET_STATUS:
                    Ep0Buffer[0] = 0x00;
                    Ep0Buffer[1] = 0x00;
                    if (SetupLen >= 2)
                    {
                        EPLength = 2;
                    }
                    else
                    {
                        EPLength = SetupLen;
                    }
                    break;
                default:
                    EPLength = 0xff;
                    break;
            }
        }
    }
    else
    {
        EPLength = 0xff; //包长度错误
    }
    if (EPLength == 0xff)
    {
        SetupReq = 0xFF;
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; //STALL
    }
    else if (EPLength <= THIS_ENDP0_SIZE) //上传数据或者状态阶段返回0长度包
    {
        UEP0_T_LEN = EPLength;
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //默认数据包是DATA1，返回应答ACK
    }
    else
    {
        UEP0_T_LEN = 0;                                                      //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //默认数据包是DATA1,返回应答ACK
    }
}

void EP0_INInterrupt(void)
{
    switch (SetupReq)
    {
    case USB_GET_DESCRIPTOR:
        EPLength = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
        if(pDescr == (UINT8C *)0xffff)
        {
            uuidcpy(Ep0Buffer, pDescr_Index, EPLength);
        }
        else
        {
            memcpy( Ep0Buffer, pDescr + pDescr_Index, EPLength );							//加载上传数据
        }
        SetupLen -= EPLength;
        pDescr_Index += EPLength;
        UEP0_T_LEN = EPLength;
        UEP0_CTRL ^= bUEP_T_TOG; //同步标志位翻转
        break;
    case USB_SET_ADDRESS:
        if(VendorControl == 0)
        {
            USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
            UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        }
        break;
    default:
        UEP0_T_LEN = 0; //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        break;
    }
}

void EP0_OUTInterrupt(void)
{
    if (SetupReq == 0x22)
    {

    }
    else
    {
        UEP0_T_LEN = 0;
		UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;  //状态阶段，对IN响应NAK
    }
}

void EP1Interrupt(void)
{
    UEP1_T_LEN = 0;
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;		   //默认应答NAK
    EP1_Busy_Flag = FALSE;												  //清除忙标志
}

void EP2Interrupt(void)
{
    if ( U_TOG_OK )													 // 不同步的数据包将丢弃
    {
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;	   //收到一包数据就NAK，主函数处理完，由主函数修改响应方式
        USBReceived = 1;
        USBOutPtr = 0;
        USBOutLength = USB_RX_LEN;
    }
}



/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB中断处理函数
*******************************************************************************/
void DeviceInterrupt(void) interrupt INT_NO_USB //using 1 //USB中断服务程序,使用寄存器组1
{
    if ((USB_INT_ST & MASK_UIS_TOKEN) == UIS_TOKEN_SOF)
	{
		SOF_Count ++;
	}
    if (UIF_TRANSFER) //USB传输完成标志
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
            case UIS_TOKEN_SETUP | 0: //SETUP事务
                SETUPInterrupt();
                break;
            case UIS_TOKEN_IN | 0: //endpoint0 IN
                EP0_INInterrupt();
                break;
            case UIS_TOKEN_OUT | 0: //endpoint0 OUT
                EP0_OUTInterrupt();
                break;
            case UIS_TOKEN_IN | 1: //Ep1 IN
                EP1Interrupt();
                break;
            case UIS_TOKEN_OUT| 2: //Ep2 OUT
                EP2Interrupt();
                break;
            default:
            break;
        }
        UIF_TRANSFER = 0;														   //写0清空中断
    }
    if(UIF_BUS_RST)                                                                 //设备模式USB总线复位中断
    {
		UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
		UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
		USB_DEV_AD = 0x00;
		UIF_SUSPEND = 0;
		UIF_TRANSFER = 0;
		UIF_BUS_RST = 0;															 //清中断标志
		UsbConfig = 0;		  //清除配置值
		EP1_Busy_Flag = 0;

		USBOutLength = 0;
		USBOutPtr = 0;
		USBReceived = 0;                              
    }
    if (UIF_SUSPEND)                                                                 //USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
        if ( USB_MIS_ST & bUMS_SUSPEND )                                             //挂起
        {
            PRINT_DEBUG("Sleep\n");
            SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;					  //USB或者RXD0/1有信号时可被唤醒
			PCON |= PD;																 //睡眠
			SAFE_MOD = 0x55;
			SAFE_MOD = 0xAA;
			WAKE_CTRL = 0x00;
        }
    }
    else {                                                                            //意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF;                                                            //清中断标志
    }
}

#if DE_PRINTF != 1

#pragma asm

CSEG	AT	00023H
	LJMP	Uart0_ISR

    RSEG  ?PR?Uart0_ISR?FTDI
	USING	1
Uart0_ISR:

	PUSH psw 
	PUSH ACC
	push XBUS_AUX
	mov psw, #0x08
	mov XBUS_AUX, #0x01

ReadFromSerial:
	jnb RI, SendToSerial

	mov ACC, WritePtr
	mov DPL, ReadPtr

	inc A
	anl DPL, #0x7f
	anl A, #0x7f

	xrl A, DPL
	jz SendToSerial

	mov DPH, #(RingBuf >> 8) 
	mov DPL, WritePtr
	mov A, SBUF
	movx @dptr, A

	inc WritePtr 
	anl WritePtr, #0x7f

SendToSerial:
	clr RI 

	jnb TI, ISR_End

	clr c
	mov A, USBOutPtr
	subb A, USBOutLength
	jc SerialTx

UsbEpAck:
	anl	UEP2_CTRL, #0xf3
	sjmp Tx_End
    
SerialTx:
	mov DPH, #(Ep2Buffer >> 8)
	mov DPL, USBOutPtr
	movx A, @dptr
	mov SBUF, A
	inc USBOutPtr

Tx_End:
	clr TI

ISR_End:
	pop XBUS_AUX
	pop ACC
	pop psw
	reti
#pragma endasm

#endif

void JumpToBootloader()
{
    #pragma asm
    CLR ES
    CLR PS

    MOV USB_INT_EN,#00H
    MOV USB_CTRL,#06H

    CLR EA

    MOV  	R7,#064H
    MOV  	R6,#00H
    LCALL	_mDelaymS

    JMP_BL:
        LJMP 0038H
        SJMP JMP_BL

    #pragma endasm
}

void main()
{
    UINT8 i;
    volatile UINT16 Uart_Timeout = 0;
    CfgFsys();    //CH559时钟选择配置
    mDelaymS(5);  //修改主频等待内部晶振稳定,必加
    mInitSTDIO(); //串口0初始化
    PRINT_DEBUG("Start\n");

#pragma asm
    ;ANL P3_MOD_OC, #0EFH 
    ;ORL P3_DIR_PU, #010H 
    ORL P1_MOD_OC, #081H
    ;ANL P1_MOD_OC, #07eH
    ORL P1_MOD_OC, #081H
#pragma endasm

    USBDeviceInit(); //USB设备模式初始化

    UEP0_T_LEN = 0;  //预使用发送长度一定要清空
    UEP1_T_LEN = 0;  //预使用发送长度一定要清空
    UEP2_T_LEN = 0;  //预使用发送长度一定要清空

    Ep1Buffer[0] = 0x01;
	Ep1Buffer[1] = 0x60;

    while (1)
    {
        if(UsbConfig)
        {
            if( EP1_Busy_Flag == FALSE )
            {
                #if DE_PRINTF != 1
                char size = WritePtr - ReadPtr;
				if(size < 0) size = size + sizeof(RingBuf); //求余数

				if(size >= 62)
				{
					for(i = 0; i < 62; i++)
					{
						Ep1Buffer[2 + i] = RingBuf[ReadPtr++];
						ReadPtr %= sizeof(RingBuf);
					}
					EP1_Busy_Flag = 1;
					UEP1_T_LEN = 64;
					UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
				}
				else if((UINT16) (SOF_Count - Uart_Timeout) >= Lantency_Timer) //超时
				{
					Uart_Timeout = SOF_Count;
					for(i = 0; i < (UINT8)size; i++)
					{
						Ep1Buffer[2 + i] = RingBuf[ReadPtr++];
						ReadPtr %= sizeof(RingBuf);
					}
					EP1_Busy_Flag = 1;
					UEP1_T_LEN = 2 + size;
					UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;			//应答ACK
				}
                #endif

                #if DE_PRINTF == 1
                UEP1_T_LEN = 2;
				UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;			//应答ACK
                #endif
            }
        }
        #if DE_PRINTF != 1
        if(USBReceived) //IDLE状态
        {
            USBReceived = 0;
            TI = 1; //触发串口中断
        }
        #endif
        if( ReDFU_Flag )
        {
            JumpToBootloader();
        }

        #if DE_PRINTF == 1
        //#ifdef DE_PRINTF
        if( print_pin_change == TRUE)
        {
            print_pin_change = FALSE;

            CH554UART0SendByte('D');
            CH554UART0SendByte(':');
            if( TXD1 ==TRUE ) CH554UART0SendByte('1');
            else CH554UART0SendByte('0');
            CH554UART0SendByte('R');
            CH554UART0SendByte(':');
            if( CAP1 ==TRUE ) CH554UART0SendByte('1');
            else CH554UART0SendByte('0');
            CH554UART0SendByte('\n');
 
        }
        #endif
    }
}