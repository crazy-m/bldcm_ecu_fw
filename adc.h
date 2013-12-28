#ifndef ADC_H_
#define ADC_H_

typedef enum adc_channel
{
	ADC_CHANNEL_0		=	0x00,
	ADC_CHANNEL_1		=	0x01,
	ADC_CHANNEL_2		=	0x02,
	ADC_CHANNEL_3		=	0x03,
	ADC_CHANNEL_4		=	0x04,
	ADC_CHANNEL_5		=	0x05,
	ADC_CHANNEL_6		=	0x06,
	ADC_CHANNEL_7		=	0x07,
	ADC_CHANNEL_8		=	0x08,
	ADC_CHANNEL_9		=	0x09,
	ADC_CHANNEL_10		=	0x0A,
	ADC_CHANNEL_TEMP	=	0x0B,
	ADC_CHANNEL_VCC4	=	0x0C,
	ADC_CHANNEL_ISRC	=	0x0D,
	ADC_CHANNEL_AMP0	=	0x0E,
	ADC_CHANNEL_AMP1	=	0x0F,
	ADC_CHANNEL_AMP2	=	0x10,
	ADC_CHANNEL_BANDGAP	=	0x11,
	ADC_CHANNEL_GND		=	0x12
}adc_channel_t;

void		adc_init			(void);
uint16_t	adc_get_channel		(adc_channel_t channel);
uint16_t	adc_get_ch8			(void);

#endif /* ADC_H_ */
