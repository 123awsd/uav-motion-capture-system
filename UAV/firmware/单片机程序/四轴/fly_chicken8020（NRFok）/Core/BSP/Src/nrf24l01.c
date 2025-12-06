#include "nrf24l01.h"


#include "NRF24L01.h"

/* 扩展变量 ------------------------------------------------------------------*/
extern SPI_HandleTypeDef   HSPI_X;

#define CE_LOW HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET)
#define CE_HIGH HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET)
#define CS_LOW HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_SET)

#define IRQ_READ HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port, NRF_IRQ_Pin)

 uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xb0, 0x43, 0x10, 0x10, 0x01}; //发送地址
 uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0xb0, 0x43, 0x10, 0x10, 0x01};

/**
 * 函数功能: 往串行Flash读取写入一个字节数据并接收一个字节数据
 * 输入参数: byte：待发送数据
 * 返 回 值: uint8_t：接收到的数据
 * 说    明：无
 */
uint8_t SPIx_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t byte)
{
    uint8_t d_read, d_send = byte;
    if (HAL_SPI_TransmitReceive(hspi, &d_send, &d_read, 1, 0xFF) != HAL_OK)
    {
        d_read = 0xFF;
    }
    return d_read;
}

/**
 * 函数功能: 检测24L01是否存在
 * 输入参数: 无
 * 返 回 值: 0，成功;1，失败
 * 说    明：无
 */
uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t i;

    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5); //写入5个字节的地址.
    NRF24L01_Read_Buf(TX_ADDR, buf, 5);                  //读出写入的地址
    for (i = 0; i < 5; i++)
        if (buf[i] != 0XA5)
            break;
    if (i != 5)
        return 1; //检测24L01错误
    return 0;     //检测到24L01
}

/**
 * 函数功能: SPI写寄存器
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：reg:指定寄存器地址
 *
 */
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
    uint8_t status;
    CS_LOW;                                   //使能SPI传输
    status = SPIx_ReadWriteByte(&HSPI_X, reg); //发送寄存器号
    SPIx_ReadWriteByte(&HSPI_X, value);        //写入寄存器的值
    CS_HIGH;                                  //禁止SPI传输
    return (status);                          //返回状态值
}

/**
 * 函数功能: 读取SPI寄存器值
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：reg:要读的寄存器
 *
 */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t reg_val;
    CS_LOW;                                     //使能SPI传输
    SPIx_ReadWriteByte(&HSPI_X, reg);            //发送寄存器号
    reg_val = SPIx_ReadWriteByte(&HSPI_X, 0XFF); //读取寄存器内容
    CS_HIGH;                                    //禁止SPI传输
    return (reg_val);                           //返回状态值
}

/**
 * 函数功能: 在指定位置读出指定长度的数据
 * 输入参数: 无
 * 返 回 值: 此次读到的状态寄存器值
 * 说    明：无
 *
 */
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, uint8_t_ctr;

    CS_LOW;                                   //使能SPI传输
    status = SPIx_ReadWriteByte(&HSPI_X, reg); //发送寄存器值(位置),并读取状态值
    for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
    {
        pBuf[uint8_t_ctr] = SPIx_ReadWriteByte(&HSPI_X, 0XFF); //读出数据
    }
    CS_HIGH;       //关闭SPI传输
    return status; //返回读到的状态值
}

/**
 * 函数功能: 在指定位置写指定长度的数据
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：reg:寄存器(位置)  *pBuf:数据指针  len:数据长度
 *
 */
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status, uint8_t_ctr;
    CS_LOW;                                   //使能SPI传输
    status = SPIx_ReadWriteByte(&HSPI_X, reg); //发送寄存器值(位置),并读取状态值
    for (uint8_t_ctr = 0; uint8_t_ctr < len; uint8_t_ctr++)
    {
        SPIx_ReadWriteByte(&HSPI_X, *pBuf++); //写入数据
    }
    CS_HIGH;       //关闭SPI传输
    return status; //返回读到的状态值
}

/**
 * 函数功能: 启动NRF24L01发送一次数据
 * 输入参数: 无
 * 返 回 值: 发送完成状况
 * 说    明：txbuf:待发送数据首地址
 *
 */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    uint8_t sta;
	uint16_t  Time_Count =1000;
    CE_LOW;
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); //写数据到TX BUF  32个字节
    CE_HIGH;                                                //启动发送

    while (IRQ_READ != 0 && Time_Count)
      Time_Count--  ; //等待发送完成
	if(Time_Count == 0)return 0xff;
    sta = NRF24L01_Read_Reg(STATUS);                 //读取状态寄存器的值
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); //清除TX_DS或MAX_RT中断标志
    if (sta & MAX_TX)                                //达到最大重发次数
    {
        NRF24L01_Write_Reg(FLUSH_TX, 0xff); //清除TX FIFO寄存器
        return MAX_TX;
    }
    if (sta & TX_OK) //发送完成
    {
        return TX_OK;
    }
    return 0xff; //其他原因发送失败
}

/**
 * 函数功能:启动NRF24L01接收一次数据
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：无
 *
 */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t sta;
    sta = NRF24L01_Read_Reg(STATUS);                 //读取状态寄存器的值
    NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); //清除TX_DS或MAX_RT中断标志
    if (sta & RX_OK)                                 //接收到数据
    {
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH); //读取数据
        NRF24L01_Write_Reg(FLUSH_RX, 0xff);                    //清除RX FIFO寄存器
        return RX_OK;
    }
    return 1; //没收到任何数据
}

/**
 * 函数功能: 该函数初始化NRF24L01到RX模式
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：无
 *
 */
void NRF24L01_RX_Mode(void)
{
    CE_LOW;
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0F);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);     //使能通道0的自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01); //使能通道0的接收地址
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);       //设置RF通信频率
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启

    NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //选择通道0的有效数据宽度

    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); //写RX节点地址

    CE_HIGH; // CE为高,进入接收模式
//    HAL_Delay(1);
}

/**
 * 函数功能: 该函数初始化NRF24L01到TX模式
 * 输入参数: 无
 * 返 回 值: 无
 * 说    明：无
 *
 */
void NRF24L01_TX_Mode(void)
{
    CE_LOW;
    NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t *)TX_ADDRESS, TX_ADR_WIDTH);    //写TX节点地址
    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK

    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);      //使能通道0的自动应答
    NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);  //使能通道0的接收地址
    NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0x1a); //设置自动重发间隔时间:4000us + 86us;最大自动重发次数:15次
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 40);        //设置RF通道为40
    NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);   //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
    NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);     //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
    CE_HIGH;                                              // CE为高,10us后启动发送
//    HAL_Delay(1);
}

void NRF_Mode_Set(u8 model, u8 ch)
{
     uint8_t d_read, d_send = 0;
	CE_LOW;
	
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);	//写RX节点地址
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH); 		//写TX节点地址 
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 															//使能通道0的自动应答
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);														//使能通道0的接收地址
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);													//设置自动重发间隔时间:500us;最大自动重发次数:10次 2M波特率下
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,ch);																	//设置RF通道为CHANAL
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 														//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	if(model==1)				//RX
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		        					// IRQ收发完成中断开启,16位CRC,主接收
	}
	else if(model==2)		//TX
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 									// IRQ收发完成中断开启,16位CRC,主发送
	}
	else if(model==3)		//RX2
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
        NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f); 
        
        d_send = 0x50;
        HAL_SPI_TransmitReceive(&HSPI_X, &d_send, &d_read, 1, 0xFF);
        d_send = 0x73;
        HAL_SPI_TransmitReceive(&HSPI_X, &d_send, &d_read, 1, 0xFF);
        
		  		  								// IRQ收发完成中断开启,16位CRC,主接收
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}
	else								//TX2
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   											// IRQ收发完成中断开启,16位CRC,主发送
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
        
        d_send = 0x50;
        HAL_SPI_TransmitReceive(&HSPI_X, &d_send, &d_read, 1, 0xFF);
        d_send = 0x73;
        HAL_SPI_TransmitReceive(&HSPI_X, &d_send, &d_read, 1, 0xFF);
        
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}
		CE_HIGH;  
}




////////////////////////////////////////////////////////////////添加的函数///////////////////////////////////////////////////////////
//返回1表示成功，0为失败
uint8_t NRF24L01_Init(uint8_t mode ,uint8_t ch)//设置模式@NFR_MODE和通信频率(0~40)
{
	int count = 0;
	while(count <= 5)
	{
		if(NRF24L01_Check() == 0)
			break;
		printf("nrf24L01连接中：%d",count);
		count++;
		HAL_Delay(1000);
	}
	if(count>5)
    {   
        printf("nrf24L01初始化失败");
        return 0;
    }
    
    NRF_Mode_Set(mode,ch);
    
    
    
    return 1;
	
}




//检测是否接收到数据

//1.主接收模式下：用MY_NRF240l_TxPacket_AP函数设置应答的负载数据，完成通信

//主发送模式下:    用MY_NRF_TxPacket
//接收到数据返回1，没有返回0

uint8_t MY_NRF_Check_RX(uint8_t* rx_buff )
{
    u8 flag = 0;
	u8 sta = NRF24L01_Read_Reg(NRF_READ_REG + STATUS);   //读接收标志
//    printf(" %x  \r\n",sta);
	if(sta & (1<<RX_DR))																	//判断是否收到数据
	{
		u8 rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);     
//printf(" %d  ",rx_len)    ;    
		if(rx_len<33)
		{
			NRF24L01_Read_Buf(RD_RX_PLOAD,rx_buff,rx_len); //接收数据
//            printf("Da:%x ",rx_buff[0]);
            flag =  1;
		}
		else 
		{
			NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除缓冲区
		}
	}

	if(sta & (1<<TX_DS))
	{
		
	}
	//接收区满
	if(sta & (1<<MAX_RT))
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);
		}
	}
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta);
    
    return flag;
}

//将数据在应答的时候发送出去，设置为主接收时用该函数发送数据
void MY_NRF240l_TxPacket_AP(uint8_t *txbuf , uint8_t len)
{
//    CE_LOW;		 													//StandBy I模式
	NRF24L01_Write_Buf(0xa8, txbuf, len); 			 //装载数据
//	CE_HIGH;  
    //    NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
//	NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, len); 			 // 装载数据	
}

void MY_NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
//	 CE_LOW;			 //StandBy I模式	
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	NRF24L01_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // 装载数据	
//	CE_HIGH; 		 //置高CE，激发数据发送
}

