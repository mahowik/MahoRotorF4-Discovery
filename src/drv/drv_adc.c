#include "board.h"
//#include <stdio.h>
#define ADC_BATTERY     0
#define ADC_CURRENT     1

// static volatile uint16_t adc1Ch4Value = 0;
static volatile uint16_t adcValues[2];

void adcInit(drv_adc_config_t *init)
{
    ADC_InitTypeDef ADC_InitStructure;
	  ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
		GPIO_InitTypeDef  GPIO_InitStructure;

    bool multiChannel = init->powerAdcChannel > 0;

     /* Enable ADC1, DMA2 clocks *************************************************/
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* DMA2 Stream0 channel0 configuration **************************************/
   DMA_DeInit(DMA2_Stream0);
   DMA_InitStructure.DMA_Channel = DMA_Channel_0;
   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcValues;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
   DMA_InitStructure.DMA_BufferSize = multiChannel ? 2 : 1;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
   DMA_Init(DMA2_Stream0, &DMA_InitStructure);
   DMA_Cmd(DMA2_Stream0, ENABLE);
 /* Configure ADC3 Channel12 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
   ADC_DeInit();
   ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
   ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
   ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
   ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
   ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
   ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
   ADC_InitStructure.ADC_ScanConvMode = multiChannel ? ENABLE : DISABLE;
   ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
   ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
   ADC_InitStructure.ADC_ExternalTrigConv = 0;
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   ADC_InitStructure.ADC_NbrOfConversion = multiChannel ? 2 : 1;
   ADC_Init(ADC1, &ADC_InitStructure);

  /* Enable ADC1 DMA */
  //ADC_DMACmd(ADC1, ENABLE);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_28Cycles);
    if (multiChannel)
        ADC_RegularChannelConfig(ADC1, init->powerAdcChannel, 2, ADC_SampleTime_28Cycles);
    ADC_DMACmd(ADC1, ENABLE);

    ADC_Cmd(ADC1, ENABLE);

    // Calibrate ADC
    //ADC_ResetCalibration(ADC1);
    //while(ADC_GetResetCalibrationStatus(ADC1));
   // ADC_StartCalibration(ADC1);
   // while(ADC_GetCalibrationStatus(ADC1));

    // Fire off ADC
    ADC_SoftwareStartConv(ADC1);
}

uint16_t adcGetChannel(uint8_t channel)
{
    return adcValues[channel];
}