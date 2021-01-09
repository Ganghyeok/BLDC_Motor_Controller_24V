/*
 * stm32f103xx_dma_driver.h
 *
 *  Created on: 2021. 1. 7.
 *      Author: Ganghyeok Lim
 */

#ifndef INC_STM32F103XX_DMA_DRIVER_H_
#define INC_STM32F103XX_DMA_DRIVER_H_

#include "stm32f103xx.h"


/*=============================================================================================================================*/

/**
  * @brief  DMA Configuration Structure definition
  */
typedef struct
{
  uint32_t Direction;                 /*!< Specifies if the data will be transferred from memory to peripheral,
                                           from memory to memory or from peripheral to memory.
                                           This parameter can be a value of @ref DMA_Data_transfer_direction */

  uint32_t PeriphInc;                 /*!< Specifies whether the Peripheral address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_Peripheral_incremented_mode */

  uint32_t MemInc;                    /*!< Specifies whether the memory address register should be incremented or not.
                                           This parameter can be a value of @ref DMA_Memory_incremented_mode */

  uint32_t PeriphDataAlignment;       /*!< Specifies the Peripheral data width.
                                           This parameter can be a value of @ref DMA_Peripheral_data_size */

  uint32_t MemDataAlignment;          /*!< Specifies the Memory data width.
                                           This parameter can be a value of @ref DMA_Memory_data_size */

  uint32_t Mode;                      /*!< Specifies the operation mode of the DMAy Channelx.
                                           This parameter can be a value of @ref DMA_mode
                                           @note The circular buffer mode cannot be used if the memory-to-memory
                                                 data transfer is configured on the selected Channel */

  uint32_t Priority;                  /*!< Specifies the software priority for the DMAy Channelx.
                                           This parameter can be a value of @ref DMA_Priority_level */
} DMA_InitTypeDef;


/**
  * @brief  DMA handle Structure definition
  */
typedef struct __DMA_HandleTypeDef
{
  DMA_Channel_TypeDef   *Instance;                       /*!< Register base address                  */

  DMA_InitTypeDef       Init;                            /*!< DMA communication parameters           */

  uint8_t  				State;                           /*!< DMA transfer state                     */

  void                  *Parent;                                                      /*!< Parent object state                    */

  void                  (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);     /*!< DMA transfer complete callback         */

  void                  (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma); /*!< DMA Half transfer complete callback    */

  void                  (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);    /*!< DMA transfer error callback            */

  void                  (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);    /*!< DMA transfer abort callback            */

  __IO uint32_t         ErrorCode;                                                    /*!< DMA Error code                         */

  DMA_TypeDef            *DmaBaseAddress;                                             /*!< DMA Channel Base Address               */

  uint32_t               ChannelIndex;                                                /*!< DMA Channel Index                      */

} DMA_HandleTypeDef;


/*=============================================================================================================================*/

/** @defgroup DMA_Error_Code DMA Error Code
  * @{
  */
#define HAL_DMA_ERROR_NONE                     0x00000000U    /*!< No error             */
#define HAL_DMA_ERROR_TE                       0x00000001U    /*!< Transfer error       */
#define HAL_DMA_ERROR_NO_XFER                  0x00000004U    /*!< no ongoing transfer  */
#define HAL_DMA_ERROR_TIMEOUT                  0x00000020U    /*!< Timeout error        */
#define HAL_DMA_ERROR_NOT_SUPPORTED            0x00000100U    /*!< Not supported mode   */


/** @defgroup DMA_Data_transfer_direction DMA Data transfer direction
  * @{
  */
#define DMA_PERIPH_TO_MEMORY         0x00000000U                 /*!< Peripheral to memory direction */
#define DMA_MEMORY_TO_PERIPH         ((uint32_t)DMA_CCR_DIR)     /*!< Memory to peripheral direction */
#define DMA_MEMORY_TO_MEMORY         ((uint32_t)DMA_CCR_MEM2MEM) /*!< Memory to memory direction     */


/** @defgroup DMA_Peripheral_incremented_mode DMA Peripheral incremented mode
  * @{
  */
#define DMA_PINC_ENABLE        ((uint32_t)DMA_CCR_PINC)  /*!< Peripheral increment mode Enable */
#define DMA_PINC_DISABLE       0x00000000U               /*!< Peripheral increment mode Disable */


/** @defgroup DMA_Memory_incremented_mode DMA Memory incremented mode
  * @{
  */
#define DMA_MINC_ENABLE         ((uint32_t)DMA_CCR_MINC)  /*!< Memory increment mode Enable  */
#define DMA_MINC_DISABLE        0x00000000U               /*!< Memory increment mode Disable */


/** @defgroup DMA_Peripheral_data_size DMA Peripheral data size
  * @{
  */
#define DMA_PDATAALIGN_BYTE          0x00000000U                  /*!< Peripheral data alignment: Byte     */
#define DMA_PDATAALIGN_HALFWORD      ((uint32_t)DMA_CCR_PSIZE_0)  /*!< Peripheral data alignment: HalfWord */
#define DMA_PDATAALIGN_WORD          ((uint32_t)DMA_CCR_PSIZE_1)  /*!< Peripheral data alignment: Word     */


/** @defgroup DMA_Memory_data_size DMA Memory data size
  * @{
  */
#define DMA_MDATAALIGN_BYTE          0x00000000U                  /*!< Memory data alignment: Byte     */
#define DMA_MDATAALIGN_HALFWORD      ((uint32_t)DMA_CCR_MSIZE_0)  /*!< Memory data alignment: HalfWord */
#define DMA_MDATAALIGN_WORD          ((uint32_t)DMA_CCR_MSIZE_1)  /*!< Memory data alignment: Word     */


/** @defgroup DMA_mode DMA mode
  * @{
  */
#define DMA_NORMAL         0x00000000U                  /*!< Normal mode                  */
#define DMA_CIRCULAR       ((uint32_t)DMA_CCR_CIRC)     /*!< Circular mode                */


/** @defgroup DMA_Priority_level DMA Priority level
  * @{
  */
#define DMA_PRIORITY_LOW             0x00000000U               /*!< Priority level : Low       */
#define DMA_PRIORITY_MEDIUM          ((uint32_t)DMA_CCR_PL_0)  /*!< Priority level : Medium    */
#define DMA_PRIORITY_HIGH            ((uint32_t)DMA_CCR_PL_1)  /*!< Priority level : High      */
#define DMA_PRIORITY_VERY_HIGH       ((uint32_t)DMA_CCR_PL)    /*!< Priority level : Very_High */


/** @defgroup DMA_interrupt_enable_definitions DMA interrupt enable definitions
  * @{
  */
#define DMA_IT_TC                         ((uint32_t)DMA_CCR_TCIE)
#define DMA_IT_HT                         ((uint32_t)DMA_CCR_HTIE)
#define DMA_IT_TE                         ((uint32_t)DMA_CCR_TEIE)


/**************************************************************************************************************
 * 																											  *
 * 												User Macro Definition										  *
 * 									  																		  *
 **************************************************************************************************************/

/** @defgroup DMA_flag_definitions DMA flag definitions
  * @{
  */
#define DMA_FLAG_GL1                      0x00000001U
#define DMA_FLAG_TC1                      0x00000002U
#define DMA_FLAG_HT1                      0x00000004U
#define DMA_FLAG_TE1                      0x00000008U
#define DMA_FLAG_GL2                      0x00000010U
#define DMA_FLAG_TC2                      0x00000020U
#define DMA_FLAG_HT2                      0x00000040U
#define DMA_FLAG_TE2                      0x00000080U
#define DMA_FLAG_GL3                      0x00000100U
#define DMA_FLAG_TC3                      0x00000200U
#define DMA_FLAG_HT3                      0x00000400U
#define DMA_FLAG_TE3                      0x00000800U
#define DMA_FLAG_GL4                      0x00001000U
#define DMA_FLAG_TC4                      0x00002000U
#define DMA_FLAG_HT4                      0x00004000U
#define DMA_FLAG_TE4                      0x00008000U
#define DMA_FLAG_GL5                      0x00010000U
#define DMA_FLAG_TC5                      0x00020000U
#define DMA_FLAG_HT5                      0x00040000U
#define DMA_FLAG_TE5                      0x00080000U
#define DMA_FLAG_GL6                      0x00100000U
#define DMA_FLAG_TC6                      0x00200000U
#define DMA_FLAG_HT6                      0x00400000U
#define DMA_FLAG_TE6                      0x00800000U
#define DMA_FLAG_GL7                      0x01000000U
#define DMA_FLAG_TC7                      0x02000000U
#define DMA_FLAG_HT7                      0x04000000U
#define DMA_FLAG_TE7                      0x08000000U

/* @DMA State */
#define DMA_STATE_RESET						0		/*!< DMA not yet initialized or disabled    */
#define DMA_STATE_READY						1		/*!< DMA initialized and ready for use      */
#define DMA_STATE_BUSY						2		/*!< DMA process is ongoing                 */
#define DMA_STATE_TIMEOUT					3		/*!< DMA timeout state                      */

#define DMA_XFER_CPLT_CB_ID          		0   	/*!< Full transfer     */
#define DMA_XFER_HALFCPLT_CB_ID      		1    	/*!< Half transfer     */
#define DMA_XFER_ERROR_CB_ID         		2    	/*!< Error             */
#define DMA_XFER_ABORT_CB_ID         		3    	/*!< Abort             */
#define DMA_XFER_ALL_CB_ID           		4     	/*!< All               */

/**************************************************************************************************************
 * 																											  *
 * 												User Macro Function											  *
 * 									  																		  *
 **************************************************************************************************************/

#define ENABLE_DMA(_HANDLE_)					(SET_BIT((_HANDLE_)->Instance->CCR, DMA_CCR_EN))
#define DISABLE_DMA(_HANDLE_)					(CLEAR_BIT((_HANDLE_)->Instance->CCR, DMA_CCR_EN))
#define ENABLE_DMA_IT(_HANDLE_, _INTERRUPT_)	(SET_BIT((_HANDLE_)->Instance->CCR, (_INTERRUPT_)))
#define DISABLE_DMA_IT(_HANDLE_, _INTERRUPT_)	(CLEAR_BIT((_HANDLE_)->Instance->CCR , (_INTERRUPT_)))


/**
  * @brief  Returns the current DMA Channel transfer complete flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified transfer complete flag index.
  */
#define DMA_GET_TC_FLAG_INDEX(_HANDLE_) \
(((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_FLAG_TC1 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_FLAG_TC2 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_FLAG_TC3 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_FLAG_TC4 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_FLAG_TC5 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_FLAG_TC6 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel7))? DMA_FLAG_TC7 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_FLAG_TC1 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_FLAG_TC2 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_FLAG_TC3 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_FLAG_TC4 :\
   DMA_FLAG_TC5)

/**
  * @brief  Returns the current DMA Channel half transfer complete flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified half transfer complete flag index.
  */
#define DMA_GET_HT_FLAG_INDEX(_HANDLE_)\
(((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_FLAG_HT1 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_FLAG_HT2 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_FLAG_HT3 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_FLAG_HT4 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_FLAG_HT5 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_FLAG_HT6 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel7))? DMA_FLAG_HT7 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_FLAG_HT1 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_FLAG_HT2 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_FLAG_HT3 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_FLAG_HT4 :\
   DMA_FLAG_HT5)

/**
  * @brief  Returns the current DMA Channel transfer error flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified transfer error flag index.
  */
#define DMA_GET_TE_FLAG_INDEX(_HANDLE_)\
(((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_FLAG_TE1 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_FLAG_TE2 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_FLAG_TE3 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_FLAG_TE4 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_FLAG_TE5 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_FLAG_TE6 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel7))? DMA_FLAG_TE7 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_FLAG_TE1 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_FLAG_TE2 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_FLAG_TE3 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_FLAG_TE4 :\
   DMA_FLAG_TE5)

/**
  * @brief  Return the current DMA Channel Global interrupt flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified transfer error flag index.
  */
#define DMA_GET_GI_FLAG_INDEX(_HANDLE_)\
(((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel1))? DMA_FLAG_GL1 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel2))? DMA_FLAG_GL2 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel3))? DMA_FLAG_GL3 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel4))? DMA_FLAG_GL4 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel5))? DMA_FLAG_GL5 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel6))? DMA_FLAG_GL6 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA1_Channel7))? DMA_FLAG_GL7 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel1))? DMA_FLAG_GL1 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel2))? DMA_FLAG_GL2 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel3))? DMA_FLAG_GL3 :\
 ((uint32_t)((_HANDLE_)->Instance) == ((uint32_t)DMA2_Channel4))? DMA_FLAG_GL4 :\
   DMA_FLAG_GL5)

/**
  * @brief  Get the DMA Channel pending flags.
  * @param  __HANDLE__: DMA handle
  * @param  __FLAG__: Get the specified flag.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCx:  Transfer complete flag
  *            @arg DMA_FLAG_HTx:  Half transfer complete flag
  *            @arg DMA_FLAG_TEx:  Transfer error flag
  *         Where x can be 1_7 or 1_5 (depending on DMA1 or DMA2) to select the DMA Channel flag.
  * @retval The state of FLAG (SET or RESET).
  */
#define DMA_GET_FLAG(_HANDLE_, _FLAG_)	\
(((uint32_t)((_HANDLE_)->Instance) > (uint32_t)DMA1_Channel7)? (DMA2->ISR & (_FLAG_)) :\
  (DMA1->ISR & (_FLAG_)))

/**
  * @brief  Clears the DMA Channel pending flags.
  * @param  __HANDLE__: DMA handle
  * @param  __FLAG__: specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCx:  Transfer complete flag
  *            @arg DMA_FLAG_HTx:  Half transfer complete flag
  *            @arg DMA_FLAG_TEx:  Transfer error flag
  *         Where x can be 1_7 or 1_5 (depending on DMA1 or DMA2) to select the DMA Channel flag.
  * @retval None
  */
#define DMA_CLEAR_FLAG(_HANDLE_, _FLAG_) \
(((uint32_t)((_HANDLE_)->Instance) > (uint32_t)DMA1_Channel7)? (DMA2->IFCR = (_FLAG_)) :\
  (DMA1->IFCR = (_FLAG_)))

/**************************************************************************************************************
 * 																											  *
 * 											APIs supported by this driver									  *
 * 						For more information about the APIs, Check the function definitions					  *
 * 									  																		  *
 **************************************************************************************************************/

void DMA_Init(DMA_HandleTypeDef *pDMAHandle);
void DMA_Start_IT(DMA_HandleTypeDef *pDMAHandle, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
void DMA_RegisterCallback(DMA_HandleTypeDef *pDMAHandle, uint8_t CallbackID, void (* pCallback)( DMA_HandleTypeDef * _pDMAHandle));
void DMA_IRQ_Handling(DMA_HandleTypeDef *pDMAHandle);

#endif /* INC_STM32F103XX_DMA_DRIVER_H_ */
