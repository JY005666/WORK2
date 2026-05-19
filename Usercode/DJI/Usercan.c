#include"Usercan.h"
HAL_StatusTypeDef CANFilterInit(CAN_HandleTypeDef* hcan){

  CAN_FilterTypeDef  sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
	
  if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    Error_Handler();
  }
	
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

	return HAL_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan){
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];

  if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
  {
    DJI_CanMsgDecode(RxHeader.StdId, RxData);
  }
}