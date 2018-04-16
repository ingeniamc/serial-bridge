/* High Speed Protocol */
#include "hsp.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "error.h"
#include "gpio.h"
#include "cmsis_os.h"

#define DFLT_TIMEOUT  100
#define SIZE_WORDS    2

static uint16_t u16Irq;
static uint16_t bAbortFlag;

static void
HspSPITransfer(const HspInst* ptInst, TFrame* tInFrame, TFrame* tOutFrame);

/** Read a static data */
static EHspStatus
HspReadSpiMaster(HspInst* ptInst, uint16_t* ptNode, uint16_t* ptSubNode,
                 uint16_t* ptAaddr, uint16_t* ptCmd, uint16_t* ptData);

static EHspStatus
HspReadUartSlave(HspInst* ptInst, uint16_t* ptNode, uint16_t* ptSubNode,
                 uint16_t* ptAddr, uint16_t* ptCmd, uint16_t* ptData);

/** Write a static data */
static EHspStatus
HspWriteSpiMaster(HspInst* ptInst, uint16_t* ptNode, uint16_t* ptSubNode,
                  uint16_t* ptAddr, uint16_t* ptCmd, uint16_t* ptData,
                  size_t* ptSz);

static EHspStatus
HSPWriteUartSlave(HspInst* ptInst, uint16_t* ptNode, uint16_t* ptSubNode,
                  uint16_t* ptAddr, uint16_t* ptCmd, uint16_t* ptData,
                  size_t* ptSz);

static EHspStatus
HspCyclicSpiTranfer(HspInst* ptInst, uint16_t* ptInBuf, uint16_t* ptOutBuf);

static void HspSPITransfer(const HspInst* ptInst, TFrame* ptInFrame,
                           TFrame* ptOutFrame)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(ptInst->phSpi, (uint8_t*)ptInFrame->buf,
                                (uint8_t*)ptOutFrame->buf, ptInFrame->sz);
}

void HspInit(HspInst* ptInst, EHspIntf eIntf, EHspMode eMode)
{
    ptInst->eState = HSP_STANDBY;

    switch (eIntf)
    {
        case SPI_BASED:
            /** This must depend on the selected SPI interface */
            ptInst->phSpi = &hspi1;
            if (eMode == MASTER_MODE)
            {
                /* SPI Master mode */
                ptInst->write = &HspWriteSpiMaster;
                ptInst->read = &HspReadSpiMaster;
            }
            else
            {
                /* SPI Slave mode */
                ptInst->write = NULL;
                ptInst->read = NULL;
            }
            break;
        case UART_BASED:
            ptInst->phUsart = &huart2;
            if (eMode == MASTER_MODE)
            {
                /* UART Master mode */
                ptInst->write = NULL;
                ptInst->read = NULL;
            }
            else
            {
                /* UART Slave mode */
                ptInst->write = &HSPWriteUartSlave;
                ptInst->read = &HspReadUartSlave;
            }
            break;
        default:
            /* Nothing */
            break;
    }
    ptInst->pu16Irq = &u16Irq;
    *ptInst->pu16Irq = DISABLE;

    bAbortFlag = false;
}

void HspDeinit(HspInst* ptInst)
{
    ptInst->phSpi = NULL;
    ptInst->phUsart = NULL;
    ptInst->write = NULL;
    ptInst->read = NULL;
    ptInst->pu16Irq = NULL;
}

EHspStatus HspWriteSpiMaster(HspInst* ptInst, uint16_t* ptNode,
                             uint16_t* ptSubNode, uint16_t* ptAddr,
                             uint16_t* ptCmd, uint16_t* ptData, size_t* ptSz)
{
    switch (ptInst->eState)
    {
        case HSP_STANDBY:
            *ptInst->pu16Irq = DISABLE;
            ptInst->sz = *ptSz;
            ptInst->bPending = false;
            ptInst->eState = HSP_WRITE_REQUEST;
            break;
        case HSP_WRITE_REQUEST:
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
            {
                /* Check if static transmission should be segmented */
                if (ptInst->sz > HSP_FRM_STA_SZ)
                {
                    FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0, *ptAddr, *ptCmd,
                                HSP_FRM_SEG, &ptData[*ptSz - ptInst->sz], NULL,
                                0, false);
                    ptInst->bPending = true;
                }
                else
                {
                    FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0, *ptAddr, *ptCmd,
                                HSP_FRM_NOTSEG, &ptData[*ptSz - ptInst->sz],
                                NULL, 0, false);
                }
                if ((HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY))
                {
                    if ((ptInst->sz - HSP_FRM_HEAD_SZ) >= HSP_FRM_STA_SZ)
                    {
                        ptInst->sz -= HSP_FRM_STA_SZ;
                    }
                    *ptInst->pu16Irq = DISABLE;
                    HspSPITransfer(ptInst, &(ptInst->Txfrm), &(ptInst->Rxfrm));
                    if ((ptInst->sz - HSP_FRM_HEAD_SZ) >= HSP_FRM_STA_SZ)
                    {
                        FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0, *ptAddr,
                                    *ptCmd, HSP_FRM_SEG,
                                    &ptData[*ptSz - ptInst->sz], NULL, 0,
                                    false);
                    }
                    else
                    {
                        /* Note: We prepare the next frame before the activation of the IRQ to
                         * improve the timing of the system */
                        FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0, *ptAddr,
                                    HSP_REQ_IDLE, HSP_FRM_NOTSEG, NULL, NULL, 0,
                                    false);
                    }
                    ptInst->eState = HSP_WRITE_REQUEST_ACK;
                }
            }
            break;
        case HSP_WRITE_REQUEST_ACK:
            /** Check if data is already available (IRQ) */
            if ((HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
                && (*ptInst->pu16Irq == ENABLE))
            {
                ptInst->eState = HSP_WRITE_ANSWER;
                ptInst->sz -= HSP_FRM_STA_SZ;
                *ptInst->pu16Irq = DISABLE;
                /* Now we just need to send the already built frame */
                HspSPITransfer(ptInst, &(ptInst->Txfrm), &(ptInst->Rxfrm));
            }
            break;
        case HSP_WRITE_ANSWER:
            /** Wait until data is received */
            if ((HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
                && (*ptInst->pu16Irq == ENABLE))
            {
                /** Check reception */
                if ((MX_SPI1_CheckCrc(ptInst->phSpi) == true)
                    && (FrameGetAddr(&(ptInst->Rxfrm)) == *ptAddr))
                {
                    if (FrameGetCmd(&(ptInst->Rxfrm)) == HSP_REP_WRITE_ERROR)
                    {
                        *ptCmd = FrameGetCmd(&(ptInst->Rxfrm));
                        ptInst->eState = HSP_CANCEL;
                    }
                    else if (FrameGetCmd(&(ptInst->Rxfrm)) == HSP_REP_ACK)
                    {
                        if (ptInst->bPending == true)
                        {
                            /* Prepare next frame */
                            if (ptInst->sz > HSP_FRM_STA_SZ)
                            {
                                FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0,
                                            *ptAddr, *ptCmd, HSP_FRM_SEG,
                                            &ptData[*ptSz - ptInst->sz], NULL,
                                            0, false);
                            }
                            else if (ptInst->sz == HSP_FRM_STA_SZ)
                            {
                                FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0,
                                            *ptAddr, *ptCmd, HSP_FRM_NOTSEG,
                                            &ptData[*ptSz - ptInst->sz], NULL,
                                            0, false);
                            }
                            else
                            {
                                /* Dummy message, allow reception of last frame CRC */
                                ptInst->bPending = false;
                                FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0,
                                            *ptAddr, HSP_REQ_IDLE,
                                            HSP_FRM_NOTSEG, NULL, NULL, 0,
                                            false);
                            }
                            ptInst->eState = HSP_WRITE_REQUEST_ACK;
                        }
                        else
                        {
                            *ptCmd = FrameGetCmd(&(ptInst->Rxfrm));
                            ptInst->eState = HSP_SUCCESS;
                            *ptSz = HSP_FRM_STA_SZ;
                        }
                    }
                    else
                    {
                        ptInst->eState = HSP_CANCEL;
                    }
                }
                else
                {
                    ptInst->eState = HSP_CANCEL;
                }
            }
            break;
        case HSP_CANCEL:
            /* Cancel init transaction */
            if (HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
            {
                FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0, 0, HSP_REQ_IDLE,
                            HSP_FRM_NOTSEG, NULL, NULL, 0, false);
                HspSPITransfer(ptInst, &(ptInst->Txfrm), &(ptInst->Rxfrm));
                ptInst->eState = HSP_ERROR;
            }
            break;
        default:
            ptInst->eState = HSP_STANDBY;
            break;
    }

    return ptInst->eState;
}

EHspStatus HspReadSpiMaster(HspInst* ptInst, uint16_t* ptNode,
                            uint16_t* ptSubNode, uint16_t* ptAddr,
                            uint16_t* ptCmd, uint16_t* ptData)
{
	switch(ptInst->eState)
	{
        case HSP_STANDBY:
            *ptInst->pu16Irq = DISABLE;
            ptInst->eState = HSP_READ_REQUEST;
            break;
        case HSP_READ_REQUEST:
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
            {
                /* Send read request */
                FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0, *ptAddr,
                            HSP_REQ_READ, HSP_FRM_NOTSEG, ptData, NULL, 0,
                            false);
                *ptInst->pu16Irq = DISABLE;
                HspSPITransfer(ptInst, &(ptInst->Txfrm), &(ptInst->Rxfrm));
                ptInst->sz = 0;
                /* Note: We prepare the next frame before checking the IRQ to improve
                 * the timing of the system */
                FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0, 0, HSP_REQ_IDLE,
                            HSP_FRM_NOTSEG, NULL, NULL, 0, false);
                ptInst->eState = HSP_READ_REQUEST_ACK;
            }
            break;
        case HSP_READ_REQUEST_ACK:
            /* Check if data is already available (IRQ) */
            if ((HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
                && (*ptInst->pu16Irq == ENABLE))
            {
                /* Now we just need to send the already built frame */
                *ptInst->pu16Irq = DISABLE;
                HspSPITransfer(ptInst, &(ptInst->Txfrm), &(ptInst->Rxfrm));
                ptInst->eState = HSP_READ_ANSWER;
            }
            break;
        case HSP_READ_ANSWER:
            /** Wait until data is received */
            if (*ptInst->pu16Irq == ENABLE)
            {
                /* Check reception */
                if ((MX_SPI1_CheckCrc(ptInst->phSpi) == true)
                    && (FrameGetAddr(&(ptInst->Rxfrm)) == *ptAddr))
                {
                    /* Copy read data to buffer - Also copy it in case of error msg */
                    size_t szReaded = FrameGetStaticData(&(ptInst->Rxfrm),
                                                         ptData);
                    ptData += szReaded;
                    ptInst->sz += szReaded;
                    *ptCmd = FrameGetCmd(&(ptInst->Rxfrm));
                    if (FrameGetCmd(&(ptInst->Rxfrm)) == HSP_REP_READ_ERROR)
                    {
                        ptInst->eState = HSP_CANCEL;
                    }
                    else if (FrameGetCmd(&(ptInst->Rxfrm)) == HSP_REP_ACK)
                    {
                        if (FrameGetSegmented(&(ptInst->Rxfrm)) != false)
                        {
                            ptInst->eState = HSP_READ_REQUEST_ACK;
                        }
                        else
                        {
                            ptInst->eState = HSP_SUCCESS;
                        }
                    }
                    else
                    {
                        ptInst->eState = HSP_CANCEL;
                    }
                }
                else
                {
                    ptInst->eState = HSP_CANCEL;
                }
            }
			break;
        case HSP_CANCEL:
            /* Cancel init transaction */
            if (HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
            {
                FrameCreate(&(ptInst->Txfrm), FRAME, 0, 0, 0, HSP_REQ_IDLE,
                            HSP_FRM_NOTSEG, NULL, NULL, 0, false);
                HspSPITransfer(ptInst, &(ptInst->Txfrm), &(ptInst->Rxfrm));
                ptInst->eState = HSP_ERROR;
            }
            break;
        default:
            ptInst->eState = HSP_STANDBY;
            break;
    }

    return ptInst->eState;
}

EHspStatus HspReadUartSlave(HspInst* ptInst, uint16_t* ptNode,
                            uint16_t* ptSubNode, uint16_t* ptAddr,
                            uint16_t* ptCmd, uint16_t* ptData)
{
    switch (ptInst->eState)
    {
        case HSP_STANDBY:
            ptInst->eState = HSP_READ_REQUEST;
            ptInst->sz = 0;
            break;
        case HSP_READ_REQUEST:
            if (HAL_UART_Receive_DMA(ptInst->phUsart,
                                     (uint8_t*)ptInst->Rxfrm.buf,
                                     HSP_UART_FRM_STATIC_SIZE_BYTES)
                == HAL_OK)
            {
                bAbortFlag = false;
                ptInst->Rxfrm.tFrameType = UART_FRAME;
                ptInst->Rxfrm.sz = HSP_UART_FRM_STATIC_SIZE_BYTES / SIZE_WORDS;
                uint16_t u16Tmp;
                for (int i = 0; i < ptInst->Rxfrm.sz; i++)
                {
                    u16Tmp = ptInst->Rxfrm.buf[i];
                    ptInst->Rxfrm.buf[i] = ((u16Tmp & 0x00ff) << 8) |
                                            ((u16Tmp & 0xff00) >> 8);
                }
                if (FrameCheckCRC(&ptInst->Rxfrm))
                {
                    *ptNode = FrameGetNode(&ptInst->Rxfrm);
                    *ptSubNode = FrameGetSubNode(&ptInst->Rxfrm);
                    *ptAddr = FrameGetAddr(&ptInst->Rxfrm);
                    *ptCmd = FrameGetCmd(&ptInst->Rxfrm);
                    ptInst->sz += FrameGetStaticData(&ptInst->Rxfrm,
                                                     &ptData[ptInst->sz]);

                    /** If request is segmented type */
                    if (ptInst->sz > HSP_FRM_STA_SZ ||
                        (FrameGetSegmented(&ptInst->Rxfrm) == ENABLE))
                    {
                        if (ptInst->sz > (size_t)HSP_FRM_MAX_DATA_SZ)
                        {
                            ptInst->eState = HSP_ERROR;
                        }
                        ptInst->eState = HSP_READ_REQUEST_ACK;
                    }
                    else
                    {
                        ptInst->eState = HSP_SUCCESS;
                    }
                }
                else
                {
                    /** CRC Error */
                    ptInst->eState = HSP_ERROR;
                }
            }
            break;
        case HSP_READ_REQUEST_ACK:
            FrameCreate(&(ptInst->Txfrm), UART_FRAME, 0, 0, 0,
                        HSP_READ_REQUEST_ACK, HSP_FRM_NOTSEG, NULL, NULL, 0,
                        true);

            uint16_t u16Tmp;
            for (int i = 0; i < ptInst->Txfrm.sz; i++)
            {
                u16Tmp = ptInst->Txfrm.buf[i];
                ptInst->Txfrm.buf[i] = ((u16Tmp & 0x00ff) << 8) |
                                        ((u16Tmp & 0xff00) >> 8);
            }

            HAL_UART_Transmit_DMA(ptInst->phUsart, (uint8_t*)ptInst->Txfrm.buf,
                                  (ptInst->Txfrm.sz * SIZE_WORDS));

            if (FrameGetSegmented(&ptInst->Rxfrm) == 0)
            {
                ptInst->eState = HSP_SUCCESS;
            }
            else
            {
                ptInst->eState = HSP_READ_REQUEST;
            }
            break;
        default:
            ptInst->eState = HSP_STANDBY;
            break;
    }
    return ptInst->eState;
}

EHspStatus HSPWriteUartSlave(HspInst* ptInst, uint16_t *ptNode,
                             uint16_t *ptSubNode, uint16_t *ptAddr,
                             uint16_t *ptCmd, uint16_t *ptData, size_t *ptSz)
{
    switch (ptInst->eState)
    {
        case HSP_STANDBY:
            ptInst->eState = HSP_WRITE_ANSWER;
            ptInst->sz = *ptSz;
            break;
        case HSP_WRITE_ANSWER:
        case HSP_WRITE_ANSWER_PENDING:
            if (*ptSz <= HSP_FRM_STA_SZ)
            {
                FrameCreate(&ptInst->Txfrm, UART_FRAME, *ptNode, *ptSubNode,
                            *ptAddr, *ptCmd,
                            HSP_FRM_NOTSEG,
                            &ptData[(ptInst->sz - *ptSz)], NULL, 0, true);
            }
            else
            {
                FrameCreate(&ptInst->Txfrm, UART_FRAME, *ptNode, *ptSubNode,
                            *ptAddr, *ptCmd,
                            HSP_FRM_SEG,
                            &ptData[(ptInst->sz - *ptSz)], NULL, 0, true);
            }
            uint16_t u16Tmp;
            for (int i = 0; i < ptInst->Txfrm.sz; i++)
            {
                u16Tmp = ptInst->Txfrm.buf[i];
                ptInst->Txfrm.buf[i] = ((u16Tmp & 0x00ff) << 8) |
                                        ((u16Tmp & 0xff00) >> 8);
            }
            if (HAL_UART_Transmit_DMA(ptInst->phUsart,
                                      (uint8_t*)&ptInst->Txfrm.buf,
                                      HSP_UART_FRM_STATIC_SIZE_BYTES)
                == HAL_OK)
            {
                ptInst->eState = HSP_SUCCESS;
            }
            else
            {
                ptInst->eState = HSP_ERROR;
            }
            break;
        default:
            ptInst->eState = HSP_STANDBY;
            break;
    }

    return ptInst->eState;
}

EHspStatus HspCyclicSpiTranfer(HspInst* ptInst, uint16_t *ptInBuf,
                               uint16_t *ptOutBuf)
{
    switch (ptInst->eState)
    {
        case HSP_STANDBY:
            /* Wait fall IRQ indicating received data */
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_SET)
            {
                FrameCreate(&(ptInst->Rxfrm), FRAME, 0, 0, 0, HSP_REQ_IDLE,
                            HSP_FRM_NOTSEG, ptInBuf, ptOutBuf, 3, false);
                ptInst->eState = HSP_CYCLIC_ANSWER;
                /* Now we just need to send the already built frame */
                hspSPITransfer(ptInst, &(ptInst->Txfrm), &(ptInst->Rxfrm));
            }
            break;
        case HSP_CYCLIC_ANSWER:
            /** Wait until data is received */
            if (HAL_SPI_GetState(ptInst->phSpi) == HAL_SPI_STATE_READY)
            {
                ptInst->eState = HSP_STANDBY;
            }
            break;
        default:
            break;
    }

    return ptInst->eState;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        default:
            u16Irq = ENABLE;
            break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7)
    {

        /* Timer 7 Interrupt, 240 ms */
        uint16_t u16PendingDMAFifoBytes =
                __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
        if (u16PendingDMAFifoBytes < HSP_UART_FRM_STATIC_SIZE_BYTES)
        {
            if (bAbortFlag)
            {
                HAL_UART_Abort(&huart2);
                HAL_DMA_Abort(&hdma_usart2_rx);
                bAbortFlag = false;
            }
            else
            {
                bAbortFlag = true;
            }
        }
        else
        {
            bAbortFlag = false;
        }

    }
}
