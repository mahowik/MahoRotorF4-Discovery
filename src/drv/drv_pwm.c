#include "board.h"

#define PULSE_1MS       (1000) // 1ms pulse width

typedef void pwmCallbackPtr(uint8_t port, uint16_t capture);

const timerHardware_t timerHardware[] = {
    { TIM2, GPIOA, GPIO_Pin_15,TIM_Channel_1, TIM2_IRQn, 0, },          	// PWM1
    { TIM2, GPIOB, GPIO_Pin_3, TIM_Channel_2, TIM2_IRQn, 0, },          	// PWM2
    { TIM3, GPIOB, GPIO_Pin_4, TIM_Channel_1, TIM3_IRQn, 0, },          	// PWM3
    { TIM3, GPIOB, GPIO_Pin_5, TIM_Channel_2, TIM3_IRQn, 0, },          	// PWM4
    { TIM4, GPIOB, GPIO_Pin_6, TIM_Channel_1, TIM4_IRQn, 0, },          	// PWM5
    { TIM4, GPIOB, GPIO_Pin_7, TIM_Channel_2, TIM4_IRQn, 0, },          	// PWM6
    { TIM4, GPIOB, GPIO_Pin_8, TIM_Channel_3, TIM4_IRQn, 0, },  			// PWM7
    { TIM4, GPIOB, GPIO_Pin_9, TIM_Channel_4, TIM4_IRQn, 0, },				// PWM8
    { TIM1, GPIOE, GPIO_Pin_14,TIM_Channel_4, TIM1_CC_IRQn, 1, },           // PWM9
    { TIM1, GPIOE, GPIO_Pin_13,TIM_Channel_3, TIM1_CC_IRQn, 1, },       	// PWM10
    { TIM1, GPIOE, GPIO_Pin_11,TIM_Channel_2, TIM1_CC_IRQn, 1, },      		// PWM11
    { TIM1, GPIOE, GPIO_Pin_9, TIM_Channel_1, TIM1_CC_IRQn, 1, },           // PWM12
    { TIM5, GPIOA, GPIO_Pin_3, TIM_Channel_4, TIM5_IRQn, 0, },          	// PWM13
    { TIM5, GPIOA, GPIO_Pin_2, TIM_Channel_3, TIM5_IRQn, 0, },          	// PWM14
    { TIM5, GPIOA, GPIO_Pin_1, TIM_Channel_2, TIM5_IRQn, 0, },          	// PWM15
    { TIM5, GPIOA, GPIO_Pin_0, TIM_Channel_1, TIM5_IRQn, 0, },       		// PWM16
    { TIM8, GPIOC, GPIO_Pin_6, TIM_Channel_1, TIM8_CC_IRQn, 1, },           // PWM17
    { TIM8, GPIOC, GPIO_Pin_7, TIM_Channel_2, TIM8_CC_IRQn, 1, },           // PWM18
    { TIM8, GPIOC, GPIO_Pin_8, TIM_Channel_3, TIM8_CC_IRQn, 1, },           // PWM19
    { TIM8, GPIOC, GPIO_Pin_9, TIM_Channel_4, TIM8_CC_IRQn, 1, },           // PWM20
};

typedef struct {
    pwmCallbackPtr *callback;
    volatile uint32_t *ccr;
    uint16_t period;

    // for input only
    uint8_t channel;
    uint8_t state;
    uint16_t rise;
    uint16_t fall;
    uint16_t capture;
} pwmPortData_t;

enum {
    TYPE_IP = 0x1000,
    TYPE_IW = 0x2000,
    TYPE_M = 0x4000,
    TYPE_S = 0x8000
};

static pwmPortData_t pwmPorts[MAX_PORTS];
static uint16_t captures[MAX_INPUTS];
static pwmPortData_t *motors[MAX_MOTORS];
static pwmPortData_t *servos[MAX_SERVOS];
static uint16_t numMotors = 0;
static uint16_t numServos = 0;
static uint16_t  numInputs = 0;
static uint16_t failsafeThreshold = 985;
// external vars (ugh)
extern int16_t failsafeCnt;

static const uint16_t multiPPM[] = {
    PWM7  | TYPE_IP,     // PPM input
    PWM9  | TYPE_M,
    PWM10 | TYPE_M,
    PWM11 | TYPE_M,
    PWM12 | TYPE_M,
    PWM13 | TYPE_M,
    PWM14 | TYPE_M,
    PWM15 | TYPE_M,
    PWM16 | TYPE_M,
    PWM17 | TYPE_S,
    PWM18 | TYPE_S,
    PWM19 | TYPE_S,
    PWM20 | TYPE_S,
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1 | TYPE_IW,     // input #1
    PWM2 | TYPE_IW,
    PWM3 | TYPE_IW,
    PWM4 | TYPE_IW,
    PWM5 | TYPE_IW,
    PWM6 | TYPE_IW,
    PWM7 | TYPE_IW,
    PWM8 | TYPE_IW,     // input #8
    PWM9 | TYPE_M,
    PWM10 | TYPE_M,
    PWM11 | TYPE_M,
    PWM12 | TYPE_M,
    PWM13 | TYPE_M,
    PWM14 | TYPE_M,
    PWM15 | TYPE_M,
    PWM16 | TYPE_M,
    PWM17 | TYPE_S,
    PWM18 | TYPE_S,
    PWM19 | TYPE_S,
    PWM20 | TYPE_S,
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM7 | TYPE_IP,     // PPM input
    PWM9 | TYPE_M,      // motor #1
    PWM10 | TYPE_M,     // motor #2
    PWM11 | TYPE_M,      // motor #1
    PWM12 | TYPE_M,
    PWM13 | TYPE_S,     // servo #1
    PWM14 | TYPE_S,
    PWM15 | TYPE_S,
    PWM16 | TYPE_S,
    PWM17 | TYPE_S,
    PWM18 | TYPE_S,
    PWM19 | TYPE_S,
    PWM20 | TYPE_S,
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1 | TYPE_IW,     // input #1
    PWM2 | TYPE_IW,
    PWM3 | TYPE_IW,
    PWM4 | TYPE_IW,
    PWM5 | TYPE_IW,
    PWM6 | TYPE_IW,
    PWM7 | TYPE_IW,
    PWM8 | TYPE_IW,     // input #8
    PWM9 | TYPE_M,      // motor #1
    PWM10 | TYPE_M,     // motor #2
    PWM11 | TYPE_M,      // motor #1
    PWM12 | TYPE_M,
    PWM13 | TYPE_S,     // servo #1
    PWM14 | TYPE_S,
    PWM15 | TYPE_S,
    PWM16 | TYPE_S,     // servo #4
    PWM17 | TYPE_S,     // servo #1
    PWM18 | TYPE_S,
    PWM19 | TYPE_S,
    PWM20 | TYPE_S,
    0xFFFF
};

static const uint16_t *hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

static void pwmTimeBase(TIM_TypeDef *tim, uint32_t period)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1;
	if ( (tim==TIM1) || (tim==TIM8)) 
	{
		TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1;
	}
	else
	{
    TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock / 1000000)/2) - 1;
	}	
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

static void pwmNVICConfig(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = value;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
    }
}

static void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

static void pwmGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, uint8_t input)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pin;
    if (input){
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
				GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
				GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
		}
    else{
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    }
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(gpio, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

}

static pwmPortData_t *pwmOutConfig(uint8_t port, uint16_t period, uint16_t value)
{
    pwmPortData_t *p = &pwmPorts[port];
    pwmTimeBase(timerHardware[port].tim, period);
    pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin, 0);
    pwmOCConfig(timerHardware[port].tim, timerHardware[port].channel, value);
    // Needed  TIM1 & TIM8
    if (timerHardware[port].outputEnable)
        TIM_CtrlPWMOutputs(timerHardware[port].tim, ENABLE);
    TIM_Cmd(timerHardware[port].tim, ENABLE);

    switch (timerHardware[port].channel) {
        case TIM_Channel_1:
            p->ccr = &timerHardware[port].tim->CCR1;
            break;
        case TIM_Channel_2:
            p->ccr = &timerHardware[port].tim->CCR2;
            break;
        case TIM_Channel_3:
            p->ccr = &timerHardware[port].tim->CCR3;
            break;
        case TIM_Channel_4:
            p->ccr = &timerHardware[port].tim->CCR4;
            break;
    }
    return p;
}

static pwmPortData_t *pwmInConfig(uint8_t port, pwmCallbackPtr callback, uint8_t channel)
{
    pwmPortData_t *p = &pwmPorts[port];
    pwmTimeBase(timerHardware[port].tim, 0xFFFF);
    pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin, 1);
    pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Rising);
    TIM_Cmd(timerHardware[port].tim, ENABLE);
    pwmNVICConfig(timerHardware[port].irq);
    // set callback before configuring interrupts
    p->callback = callback;
    p->channel = channel;

    switch (timerHardware[port].channel) {
        case TIM_Channel_1:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC1, ENABLE);
            break;
        case TIM_Channel_2:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC2, ENABLE);
            break;
        case TIM_Channel_3:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC3, ENABLE);
            break;
        case TIM_Channel_4:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC4, ENABLE);
            break;
    }
    return p;
}

void TIM1_CC_IRQHandler(void)		//TIM1
{
    uint8_t port;

    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) {
        port = PWM12;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        pwmPorts[port].callback(port, TIM_GetCapture1(TIM1));
    } else if (TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET) {
        port = PWM11;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
        pwmPorts[port].callback(port, TIM_GetCapture4(TIM1));
    } else if (TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET) {
        port = PWM10;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
        pwmPorts[port].callback(port, TIM_GetCapture4(TIM1));
    } else if (TIM_GetITStatus(TIM1, TIM_IT_CC4) == SET) {
        port = PWM9;
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
        pwmPorts[port].callback(port, TIM_GetCapture4(TIM1));
    }
}
void TIM8_CC_IRQHandler(void)		//TIM8
{
    uint8_t port;

    if (TIM_GetITStatus(TIM8, TIM_IT_CC1) == SET) {
        port = PWM17;
        TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);
        pwmPorts[port].callback(port, TIM_GetCapture1(TIM8));
    } else if (TIM_GetITStatus(TIM8, TIM_IT_CC2) == SET) {
        port = PWM18;
        TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);
        pwmPorts[port].callback(port, TIM_GetCapture4(TIM8));
    } else if (TIM_GetITStatus(TIM8, TIM_IT_CC3) == SET) {
        port = PWM19;
        TIM_ClearITPendingBit(TIM8, TIM_IT_CC3);
        pwmPorts[port].callback(port, TIM_GetCapture4(TIM8));
    } else if (TIM_GetITStatus(TIM8, TIM_IT_CC4) == SET) {
        port = PWM20;
        TIM_ClearITPendingBit(TIM8, TIM_IT_CC4);
        pwmPorts[port].callback(port, TIM_GetCapture4(TIM8));
    }
}

 void TIM2_IRQHandler(void)		//TIM2
{
    int8_t port;
    

    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) {
        port = PWM1;
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        pwmPorts[port].callback(port, TIM_GetCapture1(TIM2));
    } else if (TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET) {
        port = PWM2;
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
        pwmPorts[port].callback(port, TIM_GetCapture2(TIM2));
    }
}


 void TIM3_IRQHandler(void)		//TIM3
{
    int8_t port;


    if (TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET) {
        port = PWM3;
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        pwmPorts[port].callback(port, TIM_GetCapture1(TIM3));
    } else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET) {
        port = PWM4;
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
        pwmPorts[port].callback(port, TIM_GetCapture2(TIM3));
    }
}
 void TIM4_IRQHandler(void)		//TIM4
{
    int8_t port;


    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET) {
        port = PWM5;
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
        pwmPorts[port].callback(port, TIM_GetCapture1(TIM4));
    } else if (TIM_GetITStatus(TIM4, TIM_IT_CC2) == SET) {
        port = PWM6;
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
        pwmPorts[port].callback(port, TIM_GetCapture2(TIM4));
    } else if (TIM_GetITStatus(TIM4, TIM_IT_CC3) == SET) {
        port = PWM7;
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
        pwmPorts[port].callback(port, TIM_GetCapture3(TIM4));
    } else if (TIM_GetITStatus(TIM4, TIM_IT_CC4) == SET) {
        port = PWM8;
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
        pwmPorts[port].callback(port, TIM_GetCapture4(TIM4));
    }
}

 void TIM5_IRQHandler(void)		//TIM5
{
    int8_t port;


    if (TIM_GetITStatus(TIM5, TIM_IT_CC1) == SET) {
        port = PWM16;
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
        pwmPorts[port].callback(port, TIM_GetCapture1(TIM5));
    } else if (TIM_GetITStatus(TIM5, TIM_IT_CC2) == SET) {
        port = PWM15;
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
        pwmPorts[port].callback(port, TIM_GetCapture2(TIM5));
    } else if (TIM_GetITStatus(TIM5, TIM_IT_CC3) == SET) {
        port = PWM14;
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
        pwmPorts[port].callback(port, TIM_GetCapture3(TIM5));
    } else if (TIM_GetITStatus(TIM5, TIM_IT_CC4) == SET) {
        port = PWM13;
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
        pwmPorts[port].callback(port, TIM_GetCapture4(TIM5));
    }
}
static void ppmCallback(uint8_t port, uint16_t capture)
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t chan = 0;
		static uint8_t GoodPulses;

    last = now;
    now = capture;
    diff = now - last;

    if (diff > 2700) { // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960 "So, if you use 2.5ms or higher as being the reset for the PPM stream start, you will be fine. I use 2.7ms just to be safe."
        chan = 0;
    } else {
        if (diff > 750 && diff < 2250 && chan < 8) {   // 750 to 2250 ms is our 'valid' channel range
            captures[chan] = diff;
					if (chan < 4 && diff > failsafeThreshold)
	                GoodPulses |= (1 << chan);      // if signal is valid - mark channel as OK
		            if (GoodPulses == 0x0F) {           // If first four chanells have good pulses, clear FailSafe counter
		                GoodPulses = 0;
		                if (failsafeCnt > 20)
		                    failsafeCnt -= 20;
		                else
		                    failsafeCnt = 0;
		            }
        }
        chan++;
        failsafeCnt = 0;
    }
}

static void pwmCallback(uint8_t port, uint16_t capture)
{
    if (pwmPorts[port].state == 0) {
        pwmPorts[port].rise = capture;
        pwmPorts[port].state = 1;
        pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Falling);
    } else {
        pwmPorts[port].fall = capture;
        // compute capture
        pwmPorts[port].capture = pwmPorts[port].fall - pwmPorts[port].rise;
        captures[pwmPorts[port].channel] = pwmPorts[port].capture;
        // switch state
        pwmPorts[port].state = 0;
        pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Rising);
        // reset failsafe
        failsafeCnt = 0;
    }
}

bool pwmInit(drv_pwm_config_t *init)
{
    int i = 0;
    const uint16_t *setup;
	 // to avoid importing cfg/mcfg
	    failsafeThreshold = init->failsafeThreshold;

    // this is pretty hacky shit, but it will do for now. array of 4 config maps, [ multiPWM multiPPM airPWM airPPM ]
    if (init->airplane)
        i = 2; // switch to air hardware config
    if (init->usePPM)
        i++; // next index is for PPM

    setup = hardwareMaps[i];

    for (i = 0; i < MAX_PORTS; i++) {
        uint16_t port = setup[i] & 0x00FF;
        uint16_t mask = setup[i] & 0xFF00;

        if (setup[i] == 0xFFFF) // terminator
            break;
		//		if (init->useServos && !init->airplane) {
            // remap PWM17-20 as servos (but not in airplane mode LOL)
      //      if (port == PWM17 || port == PWM18 || port == PWM19 || port == PWM20)
      //          mask = TYPE_S;
      //  }
        if (mask & TYPE_IP) {
            pwmInConfig(port, ppmCallback, 0);
            numInputs = 8;
        } else if (mask & TYPE_IW) {
            pwmInConfig(port, pwmCallback, numInputs);
            numInputs++;
        } else if (mask & TYPE_M) {
            motors[numMotors++] = pwmOutConfig(port, 1000000 / init->motorPwmRate, PULSE_1MS);
        } else if (mask & TYPE_S) {
            servos[numServos++] = pwmOutConfig(port, 1000000 / init->servoPwmRate, PULSE_1MS);
        }
    }

    return false;
}

void pwmWriteMotor(uint8_t index, uint16_t value)
{
    if (index < numMotors)
        *motors[index]->ccr = value;
}

void pwmWriteServo(uint8_t index, uint16_t value)
{
    if (index < numServos)
        *servos[index]->ccr = value;
}

uint16_t pwmRead(uint8_t channel)
{
    return captures[channel];
}
