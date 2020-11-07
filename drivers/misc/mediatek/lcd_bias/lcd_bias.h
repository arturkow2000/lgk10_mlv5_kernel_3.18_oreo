#ifndef _LCD_BIAS_H
#define _LCM_BIAS_H


#define LCD_BIAS_VPOS_ADDR    0x00
#define LCD_BIAS_VNEG_ADDR    0x01
#define LCD_BIAS_APPS_ADDR    0x02
#define NEG_OUTPUT_APPS 0x40

#ifdef BUILD_LK
#define LCD_BIAS_I2C_BUSNUM   0	/* for I2C channel 0 */
#define LCD_BIAS_I2C_ADDR       0x3E /*for I2C slave dev addr*/

#define LCD_BIAS_ST_MODE         0
#define LCD_BIAS_MAX_ST_MODE_SPEED 100  /* khz */

#define LCD_BIAS_PRINT printf

#else

#define LCD_BIAS_PRINT printk

#define LCD_BIAS_I2C_ID_NAME "I2C_LCD_BIAS"
#define LCD_BIAS_DTS_ID_NAME "DTS_LCD_BIAS"

/* DTS state */
typedef enum tagLCD_BIAS_GPIO_STATE {
	LCD_BIAS_GPIO_STATE_ENP0,
	LCD_BIAS_GPIO_STATE_ENP1,
	LCD_BIAS_GPIO_STATE_ENN0,
	LCD_BIAS_GPIO_STATE_ENN1,
	LCD_BIAS_GPIO_STATE_MAX,	/* for array size */
} LCD_BIAS_GPIO_STATE;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
int lcd_bias_set_vspn(unsigned int value);
void lcd_bias_gpio_select_state(LCD_BIAS_GPIO_STATE s);
#endif

typedef enum {
	FIRST_VSP_AFTER_VSN = 0,
	FIRST_VSN_AFTER_VSP = 1
} LCD_BIAS_POWER_ON_SEQUENCE;

void lcd_bias_set_vspn_en(unsigned int en, unsigned int seq);

#endif /* _LCM_BIAS_H */