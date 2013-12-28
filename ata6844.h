#ifndef ATA6844_H_
#define ATA6844_H_

#define ATA6844_EN_DDR		DDRB
#define ATA6844_EN_PORT		PORTB
#define ATA6844_EN_PIN		PB2

#define ATA6844_SLEEP_DDR	DDRC
#define ATA6844_SLEEP_PORT	PORTC
#define ATA6844_SLEEP_PIN	PC7

#define ATA6844_COAST_DDR	DDRC
#define ATA6844_COAST_PORT	PORTC
#define ATA6844_COAST_PIN	PC1

#define ATA6844_WD_DDR		DDRD
#define ATA6844_WD_PORT		PORTD
#define ATA6844_WD_PIN		PD2

#define ATA6844_WDEN_DDR	DDRD
#define ATA6844_WDEN_PORT	PORTD
#define ATA6844_WDEN_PIN	PD1

#define ATA6844_DG1_DDR		DDRD
#define ATA6844_DG1_PORT	PORTD

void	ata6844_init		(void);
void	ata6844_reset_dg1	(void);
void	ata6844_enabled		(uint8_t enable);
void	ata6844_sleep		(uint8_t sleep);
void	ata6844_coast		(uint8_t coast);
void	ata6844_wd_enable	(uint8_t enable);
uint8_t	ata6844_get_dg		(void);

#endif /* ATA6844_H_ */
