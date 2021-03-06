#ifndef MC_H_
#define MC_H_

typedef enum mc_dir
{
	MC_DIR_CW	=	0x01,
	MC_DIR_CCW	=	0x02
} mc_direction_t;

typedef enum mc_mode
{
	MC_MODE_SPEED		=	0x01,
	MC_MODE_POSITION	=	0x02
} mc_mode_t;

void		mc_init			(void);
void		mc_run			(uint8_t run);
void		mc_coast		(void);
void		mc_rpm_set		(int16_t speed);
int16_t		mc_rpm_get		(void);
void		mc_rev_set		(int16_t revs);
int16_t		mc_rev_get	(void);
uint16_t	mc_current_get	(void);
uint8_t		mc_voltage_get	(void);
int8_t		mc_temp_get		(void);

#endif /* MC_H_ */
