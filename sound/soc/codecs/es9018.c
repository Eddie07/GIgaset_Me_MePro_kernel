/*
 * es9018.c -- es9018 ALSA SoC audio driver
 *
 */

// #define DEBUG 1
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <trace/events/asoc.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include "es9018.h"
#include <linux/i2c.h>
#include <linux/timer.h>

#define SYSTEM_SETTING   0
#define INPUT_CONFIG_SOURCE 1
#define I2S_BIT_FORMAT_MASK (0x03 << 6)
#define MASTER_MODE_CONTROL 10
#define I2S_CLK_DIVID_MASK (0x03 << 5)
#define RIGHT_CHANNEL_VOLUME_15 15
#define LEFT_CHANNEL_VOLUME_16 16
#define MASTER_TRIM_VOLUME_17 17
#define MASTER_TRIM_VOLUME_18 18
#define MASTER_TRIM_VOLUME_19 19
#define MASTER_TRIM_VOLUME_20 20
#define HEADPHONE_AMPLIFIER_CONTROL 42

struct es9018_reg {
	unsigned char num;
	unsigned char value;
};

struct clock_div {
	int master_div;
	int bit_div;
};

/* We only include the analogue supplies here; the digital supplies
 * need to be available well before this driver can be probed.
 */
static const struct es9018_reg es9018_init_register[]={
#if 0
		0x00,//0  
		0x8c,//1  //:I2S input
		0x18,//2
		0x10,//3
		0x00,//4
		0x68,//5
		0x47,//6: 47= 32KHz ; 57=44.1KHz; 67=48KHz
		0x80,//7
		0x70,//8
		0xa2,//9 	:slave mode=0x22;  master mode= 0xa2
		0xAd,//10
		0x02,//11
		0x5a,//12
		0x00,//13
		0x8a,//14
		0x00,//15
		0x00,//16
		0xff,//17
		0xff,//18
		0xff,//19
		0x7f,//20
		0x00,//21
		0x00,//22
		0x00,//23
		0x32,//24
		0xff//25
#endif
#if 1
	{0, 0x00},	
	{1, 0x8c},  /* I2S input */	
	{2, 0x18},	
	{3, 0x10},	
	{4, 0xFF},	// 0x00-->0xFF
	{5, 0x68},	// 0x78-->68
	{6, 0x4a},  /* 47= 32KHz ; 57=44.1KHz; 67=48KHz */	
	{7, 0x80},	
	{8, 0x10},	
	{9, 0xa2}, //22w /* slave mode=0x22;  master mode= 0xa2 */	
	{10, 0xE5}, // 0xE5	
	{11, 0x02},	
	{12, 0x5a},	
	{13, 0x00}, // 0x40	
	{14, 0x8a},
	{15, 12},	
	{16, 12},
	{17, 0xff},	
	{18, 0xff},	
	{19, 0xff},	
	{20, 0x7f}, // 3F-->7f
	{21, 0x00},	
	{22, 0x00},	
	{23, 0x00},	
	{24, 0xF2}, // 0x00	
	{25, 0xF2}, // 0xd9	
	{26, 0x00}, // Demo board end 0x25	
	{27, 0x00},	
	{28, 0x00},	
	{29, 0x00},	
	{30, 0x00},
#if 0
	{31, 0x00},	
	{32, 0x00},	
	{33, 0x00},	
	{34, 0x00},	
	{35, 0x00},	
	{36, 0x00},	
	{37, 0x00},	
	{38, 0x00},	
	{39, 0x00},	
	{40, 0x00},	
	{41, 0x04},	
	{42, 0x43},	
	{43, 0x00}	
#endif
#endif
};


/* codec private data */

struct es9018_data {
	int reset_gpio;
	u32 pwr_num;
    int *power_gpio;
    int i2c_scl_gpio;
	int clk_en_gpio; // For 48K
	int clk_441k; // For 44.1K
	int swi_pwr_gpio;
	int swi_mute_gpio;
	int swi_sel_gpio;
	int tfa9890_rst_gpio;
	/* Add by Harry to control the VDD */
	struct regulator * dvdd_2_7;
	
	/* 
	 * Del by Harry	
	 * Not need config i2c GPIO
	 */
#if 0	 
	int i2c_sda_gpio;
	int i2c_addr;
#endif	
};

struct es9018_priv {
	struct snd_soc_codec *codec;
	struct i2c_client *i2c_client;
    struct es9018_data *es9018_data;
    struct delayed_work sleep_work;
    struct mutex power_lock;
}/* es9018_priv */;

enum {	
	CRYSTAL_45M = 0,
	CRYSTAL_49M,
};

static int es9018_power_state = 0;
static int es9018_clk_state = 0;
static int es9018_clk_44_1k_state = 0;
static int es9018_swi_mute_stu = 1;
static int es9018_swi_sel_stu = 0;
static int hifi_swi_pwr_en = 0;
static int hifi_volume = 0;
static int hifi_volume_save = -1;
static int es9018_slave_mode = 0;
// Fot save div
// static int hifi_bit_div = -1;
static int hifi_mclk_div = -1;

static struct es9018_priv *g_es9018_priv = NULL;
static int es9018_write_reg(struct i2c_client *client, int reg, u8 value);
#define ES9018_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |	\
		     SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |	\
		     SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |	\
		     SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)

#define ES9018_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE | \
		       SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
		       SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE | \
		       SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE)


static int es9018_read_reg(struct i2c_client *client, int reg);
static int es9018_write_reg(struct i2c_client *client, int reg, u8 value);

/*
 * Change by Harry
 * Del const
 */
//static /* const */ int G_volume = 0;


static void power_gpio_H(void)
{
	u32 i;

	for (i = 0; i < g_es9018_priv->es9018_data->pwr_num; i ++) {
		if ( __gpio_get_value(g_es9018_priv->es9018_data->power_gpio[i]) == 0) {
	        gpio_set_value(g_es9018_priv->es9018_data->power_gpio[i], 1);
	        pr_debug("%s(): pa_gpio_level = %d\n", __func__, 
						__gpio_get_value(g_es9018_priv->es9018_data->power_gpio[i]));
	    }
	}
}

static void power_gpio_L(void)
{
	u32 i;

	for (i = 0; i < g_es9018_priv->es9018_data->pwr_num; i ++) {
		if ( __gpio_get_value(g_es9018_priv->es9018_data->power_gpio[i]) == 1) {
	        gpio_set_value(g_es9018_priv->es9018_data->power_gpio[i], 0);
	        pr_debug("%s(): pa_gpio_level = %d\n", __func__, 
						__gpio_get_value(g_es9018_priv->es9018_data->power_gpio[i]));
	    }	
	}
}

static void reset_gpio_H(void)
{
	if ( __gpio_get_value(g_es9018_priv->es9018_data->reset_gpio) == 0) {
        gpio_set_value(g_es9018_priv->es9018_data->reset_gpio, 1);
        pr_debug("%s(): pa_gpio_level = %d\n", __func__, __gpio_get_value(g_es9018_priv->es9018_data->reset_gpio));
    }	
}

static void reset_gpio_L(void)
{
	if ( __gpio_get_value(g_es9018_priv->es9018_data->reset_gpio) == 1) {
        gpio_set_value(g_es9018_priv->es9018_data->reset_gpio, 0);
        pr_debug("%s(): pa_gpio_level = %d\n", __func__, __gpio_get_value(g_es9018_priv->es9018_data->reset_gpio));
    }	
}


static int es9018_open(void)
{
	int i = 0;
	int reg_val;
	// Enable VDD
	if (! regulator_enable(g_es9018_priv->es9018_data->dvdd_2_7));
	// gpio_set_value(g_es9018_priv->es9018_data->clk_en_gpio,1);
	msleep(50);
	power_gpio_H();
	reset_gpio_L();
    msleep(50);
	reset_gpio_H();
	msleep(50);
	for(i = 0; i < ARRAY_SIZE(es9018_init_register); i ++){ 
		es9018_write_reg(g_es9018_priv->i2c_client,es9018_init_register[i].num,	es9018_init_register[i].value);
	}
#if 0
	// Set the volume
	if ((hifi_volume_save >= 0) && (hifi_volume_save < 255)) {
		printk("%s:Set volume to %d\n",__func__,hifi_volume_save);
		es9018_write_reg(g_es9018_priv->i2c_client,15,255 - hifi_volume_save);
		es9018_write_reg(g_es9018_priv->i2c_client,16,255 - hifi_volume_save);
		hifi_volume = hifi_volume_save;
		hifi_volume_save = -1;
	}
	else {
	   /* Add by Harry
	 	* Note that : ES9018 volume register reset value is 0, the max volume
	 	* If not set volume by Android, must set a value to it
	 	*/
	 	if (hifi_volume > 255 || hifi_volume < 0) {
			pr_info("%s:set hifi_volume to default value 182\n",__func__);
			hifi_volume = 182; // default volume
	 	}
		pr_info("%s:set hifi_volume to  value %d\n",__func__,255 - hifi_volume);
	 	es9018_write_reg(g_es9018_priv->i2c_client,15,255 - hifi_volume);
		es9018_write_reg(g_es9018_priv->i2c_client,16,255 - hifi_volume);
	}
#endif

	// Add by Harry for set the div
	if ((hifi_mclk_div >= 0) && (hifi_mclk_div <= 3)) {
		printk("%s:set mclk div to %d\n",__func__,hifi_mclk_div);
		reg_val = es9018_read_reg(g_es9018_priv->i2c_client,MASTER_MODE_CONTROL);	
		reg_val &= ~(I2S_CLK_DIVID_MASK);	
		reg_val |=  (hifi_mclk_div << 5);	
		es9018_write_reg(g_es9018_priv->i2c_client,MASTER_MODE_CONTROL, reg_val);
		hifi_mclk_div = -1;
	}

#if 0
	/* Set the master or slave mode */
	if (0 /* es9018_slave_mode */) {
		val =  es9018_init_register[MASTER_MODE_CONTROL].value;
		val &= ~0x80;
		es9018_write_reg(g_es9018_priv->i2c_client,MASTER_MODE_CONTROL,	val);
	}
#endif
	printk("Write ES9018 OK!!\n");
//	for(i = 0; i < ARRAY_SIZE(es9018_init_register); i ++){ 
//		val = es9018_read_reg(g_es9018_priv->i2c_client,es9018_init_register[i].num);
//		printk(KERN_ERR"REG[%d] = 0x%x\n",es9018_init_register[i].num,val);
//	}
  	//power_gpio_L();
	return 0;
}

static int es9018_close(void)
{
	power_gpio_H();
	reset_gpio_L();
	power_gpio_L();
//	hifi_volume      = 255;
	hifi_mclk_div = -1;
	hifi_volume_save = -1;
	return 0;
}

static int es9018_get_power_state_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): power state = %d\n", __func__,
		es9018_power_state);
	ucontrol->value.enumerated.item[0] = es9018_power_state;
	pr_info("%s(): ucontrol = %d\n", __func__,
		ucontrol->value.enumerated.item[0]);

	return 0;
}

static int es9018_put_power_state_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int ret=0;
	pr_info("GAC:%s():ucontrol = %d\n", __func__,
		ucontrol->value.enumerated.item[0]);
	pr_info("GAC:%s():power state= %d\n", __func__,
		es9018_power_state);

	if (es9018_power_state == ucontrol->value.enumerated.item[0]) {
		pr_info("GAC:%s():no power state change\n", __func__);
	}

	es9018_power_state = ucontrol->value.enumerated.item[0];
		
	if (ucontrol->value.enumerated.item[0])
		ret = es9018_open();
	else 
		es9018_close();

	return ret;
}


#if 0
static int es9018_get_volume_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): Volume = %d\n", __func__,
		G_volume);
	ucontrol->value.enumerated.item[0] = G_volume;
	pr_info("%s(): ucontrol = %d\n", __func__,
		ucontrol->value.enumerated.item[0]);

	return 0;
}


static int es9018_put_volume_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int ret=0;
	pr_info("GAC:%s():ucontrol = %d\n", __func__,
		ucontrol->value.enumerated.item[0]);
	pr_info("GAC:%s():Volume= %d\n", __func__,
		G_volume);

	if (es9018_power_state == ucontrol->value.enumerated.item[0]) {
		pr_info("GAC:%s(): Volume not changed\n", __func__);
	}
    else
	{
	 G_volume = ucontrol->value.enumerated.item[0];
	 es9018_write_reg(g_es9018_priv->i2c_client,20,G_volume);
    }
		

	return ret;
}
#endif
static int es9018_get_clk_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): Clock state = %d\n", __func__,
				es9018_clk_state);
	
	ucontrol->value.enumerated.item[0] = es9018_clk_state;
	
	pr_info("%s(): ucontrol = %d\n", __func__,
				ucontrol->value.enumerated.item[0]);

	return 0;
}

static int es9018_put_clk_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): clock state = %d\n", __func__,
				es9018_clk_state);
	
	if (ucontrol->value.enumerated.item[0] == es9018_clk_state)
		return 0;

	es9018_clk_state = ucontrol->value.enumerated.item[0];
	__gpio_set_value(g_es9018_priv->es9018_data->clk_en_gpio,es9018_clk_state);
	
	return 0;
}

static int es9018_get_clk_44_1k_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): Clock state = %d\n", __func__,
				es9018_clk_44_1k_state);
	
	ucontrol->value.enumerated.item[0] = es9018_clk_44_1k_state;
	
	pr_info("%s(): ucontrol = %d\n", __func__,
				ucontrol->value.enumerated.item[0]);

	return 0;
}

static int es9018_put_clk_44_1k_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): clock state = %d\n", __func__,
				es9018_clk_44_1k_state);
	
	if (ucontrol->value.enumerated.item[0] == es9018_clk_44_1k_state)
		return 0;

	es9018_clk_44_1k_state = ucontrol->value.enumerated.item[0];
	__gpio_set_value(g_es9018_priv->es9018_data->clk_441k,es9018_clk_44_1k_state);
	
	return 0;
}

static int es9018_get_switch_mute(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): Mute state = %d\n", __func__,
				es9018_swi_mute_stu);
	
	ucontrol->value.enumerated.item[0] = es9018_swi_mute_stu;
	
	pr_info("%s(): ucontrol = %d\n", __func__,
				ucontrol->value.enumerated.item[0]);

	return 0;
}

static int es9018_put_switch_mute(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): Mute state = %d\n", __func__,
				es9018_swi_mute_stu);
	
	if (ucontrol->value.enumerated.item[0] == es9018_swi_mute_stu)
		return 0;

	es9018_swi_mute_stu = ucontrol->value.enumerated.item[0];
	__gpio_set_value(g_es9018_priv->es9018_data->swi_mute_gpio,es9018_swi_mute_stu);
	
	return 0;
}

static int es9018_get_switch_sel_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): Sel state = %d\n", __func__,
				es9018_swi_sel_stu);
	
	ucontrol->value.enumerated.item[0] = es9018_swi_sel_stu;
	
	pr_info("%s(): ucontrol = %d\n", __func__,
				ucontrol->value.enumerated.item[0]);

	return 0;
}

static int es9018_put_switch_sel_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): Sel state = %d\n", __func__,
				es9018_swi_sel_stu);
	
	if (ucontrol->value.enumerated.item[0] == es9018_swi_sel_stu)
		return 0;

	es9018_swi_sel_stu = ucontrol->value.enumerated.item[0];
	__gpio_set_value(g_es9018_priv->es9018_data->swi_sel_gpio,!! es9018_swi_sel_stu);
	
	return 0;
}

static int hifi_get_switch_pwr_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): Sel state = %d\n", __func__,
				hifi_swi_pwr_en);
	
	ucontrol->value.enumerated.item[0] = hifi_swi_pwr_en;
	
	pr_info("%s(): ucontrol = %d\n", __func__,
				ucontrol->value.enumerated.item[0]);

	return 0;
}

static int hifi_put_switch_pwr_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): Sel state = %d\n", __func__,
				hifi_swi_pwr_en);
	
	if (ucontrol->value.enumerated.item[0] == hifi_swi_pwr_en)
		return 0;

	hifi_swi_pwr_en = ucontrol->value.enumerated.item[0];
	__gpio_set_value(g_es9018_priv->es9018_data->swi_pwr_gpio,hifi_swi_pwr_en);
	
	return 0;
}

static int hifi_get_mode_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): hifi mode = %d\n", __func__,
				es9018_slave_mode);
	
	ucontrol->value.enumerated.item[0] = es9018_slave_mode;
	
	pr_info("%s(): ucontrol = %d\n", __func__,
				ucontrol->value.enumerated.item[0]);

	return 0;
}

static int hifi_put_mode_enum(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): hifi mode = %d\n", __func__,
				es9018_slave_mode);
	
	if (ucontrol->value.enumerated.item[0] == es9018_slave_mode)
		return 0;

	es9018_slave_mode = ucontrol->value.enumerated.item[0];
	
	return 0;
}

static int es9018_get_i2s_length(struct snd_kcontrol *kcontrol,	struct snd_ctl_elem_value *ucontrol)
{	
	u8 reg_val;	
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,INPUT_CONFIG_SOURCE);
	
	reg_val = reg_val >> 6;	ucontrol->value.integer.value[0] = reg_val;	

	pr_info("%s: i2s_length = 0x%x\n", __func__, reg_val);	

	return 0;
}

static int es9018_set_i2s_length(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{	
	u8 reg_val;

	printk(KERN_ERR"%s: ucontrol->value.integer.value[0]  = %ld\n",__func__, ucontrol->value.integer.value[0]);	

	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,INPUT_CONFIG_SOURCE);
	reg_val &= ~(I2S_BIT_FORMAT_MASK);	
	reg_val |=  (ucontrol->value.integer.value[0] << 6);	

	printk(KERN_ERR"reg_val = 0x%x\n",reg_val);

	es9018_write_reg(g_es9018_priv->i2c_client,INPUT_CONFIG_SOURCE, reg_val);

	return 0;
}

static int es9018_get_clk_divider(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{	
	u8 reg_val;

	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,MASTER_MODE_CONTROL);
	reg_val = reg_val >> 5;	
	ucontrol->value.integer.value[0] = reg_val;
	pr_info("%s: i2s_length = 0x%x\n", __func__, reg_val);	

	return 0;
}

static int es9018_set_clk_divider(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{	
	u8 reg_val;

	pr_info("%s: ucontrol->value.integer.value[0]  = %ld\n",__func__, ucontrol->value.integer.value[0]);	

	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,MASTER_MODE_CONTROL);	

	reg_val &= ~(I2S_CLK_DIVID_MASK);	

	reg_val |=  ucontrol->value.integer.value[0] << 5;	

	es9018_write_reg(g_es9018_priv->i2c_client,MASTER_MODE_CONTROL, reg_val);

	return 0;
}

static void clk_gpio_441k_H(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->clk_441k) == 0) {
		gpio_set_value(g_es9018_priv->es9018_data->clk_441k, 1);
		pr_info("%s(): clk_441k_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->clk_441k));
	}
}

static void clk_gpio_441k_L(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->clk_441k) == 1) {
		gpio_set_value(g_es9018_priv->es9018_data->clk_441k, 0);
		pr_info("%s(): clk_441k_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->clk_441k));
	}
}


static void clk_gpio_48k_H(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->clk_en_gpio) == 0) {
		gpio_set_value(g_es9018_priv->es9018_data->clk_en_gpio, 1);
		pr_info("%s(): clk_48k_level = %d\n", __func__,	
			__gpio_get_value(g_es9018_priv->es9018_data->clk_en_gpio));
	}
}

static void clk_gpio_48k_L(void)
{
	if (__gpio_get_value(g_es9018_priv->es9018_data->clk_en_gpio) == 1) {
		gpio_set_value(g_es9018_priv->es9018_data->clk_en_gpio, 0);
		pr_info("%s(): clk_441k_level = %d\n", __func__,
			__gpio_get_value(g_es9018_priv->es9018_data->clk_en_gpio));
	}
}

static int crystal_45M_get(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =	__gpio_get_value(g_es9018_priv->es9018_data->clk_441k);
	return 0;
}

static int crystal_45M_set(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{	
	int comp = ((struct soc_multi_mixer_control *)kcontrol->private_value)->shift;	

	if (comp == CRYSTAL_45M) {
		if (ucontrol->value.integer.value[0] == 1) {
			pr_info("%s enter, enable 45M crystal\n", __func__);
			clk_gpio_441k_H();
			mdelay(10);
		} else {
			pr_info("%s enter, disable 45M crystal\n", __func__);
			clk_gpio_441k_L();
			mdelay(10);
		}	
	}	
	return 0;
}

static int crystal_49M_get(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =	__gpio_get_value(g_es9018_priv->es9018_data->clk_en_gpio);
	return 0;
}

static int crystal_49M_set(struct snd_kcontrol *kcontrol,struct snd_ctl_elem_value *ucontrol)
{
	int comp = ((struct soc_multi_mixer_control *)kcontrol->private_value)->shift;	
	if ( comp == CRYSTAL_49M) {
		if (ucontrol->value.integer.value[0] == 1) {
			pr_info("%s enter, enable 49M crystal\n", __func__);
			clk_gpio_48k_H();
			mdelay(10);
		} else {
			pr_info("%s enter, disable 49M crystal\n", __func__);
			clk_gpio_48k_L();
			mdelay(10);	
		}
	}

	return 0;
}

static int hifi_volume_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	pr_info("GAC:%s(): HiFi volume = %d\n", __func__,
				hifi_volume);
	
	ucontrol->value.enumerated.item[0] = hifi_volume;
	
	pr_info("%s(): ucontrol = %d\n", __func__,
				ucontrol->value.enumerated.item[0]);

	return 0;
}

static int hifi_volume_set(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int ret;
	
	pr_info("GAC:%s(): HiFi volume = %d\n", __func__,
				hifi_volume);

	if (ucontrol->value.enumerated.item[0] > 255) {
		pr_err("%s:Value error!\n",__func__);
		return -1;
	}

	/* If power no turn on,save the volume */
	if (! es9018_power_state) {
		pr_info("%s: Save the volume because the ES9018 not turn on!\n",__func__);
		hifi_volume_save = ucontrol->value.enumerated.item[0];
		return 0;
	}

	if (ucontrol->value.enumerated.item[0] == hifi_volume)
		return 0;

	pr_info("%s: ucontrol->value.enumerated.item[0] = %d!\n",__func__,ucontrol->value.enumerated.item[0]);
	hifi_volume = ucontrol->value.enumerated.item[0];
	ret =  es9018_write_reg(g_es9018_priv->i2c_client,15,255 - hifi_volume);
	ret += es9018_write_reg(g_es9018_priv->i2c_client,16,255 - hifi_volume);
	pr_info("%s: ret = %d\n",__func__,ret);
	
	return ret;
}


static const char * const es9018_power_state_texts[] = {
	 "Close","Open"
};

static const struct soc_enum es9018_power_state_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(es9018_power_state_texts),
			es9018_power_state_texts);

static const char * const es9018_swi_sel_texts[] = {
	 "HI-FI","No HI-FI"
};

static const struct soc_enum es9018_switch_sel_enum = 
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(es9018_swi_sel_texts),
			es9018_swi_sel_texts);

static const char * const es9018_gain_state_texts[] = {
	"0dB", "1dB","2dB"
};

static const struct soc_enum es9018_gain_state_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(es9018_gain_state_texts),
			es9018_gain_state_texts);

static const char * const es9018_clk_state_texts[] = {
	"OFF", "ON"
};

static const struct soc_enum es9018_clk_state_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(es9018_clk_state_texts),
			es9018_clk_state_texts);

static const char * const es9018_mute_state_texts[] = {
	"UnMute", "Mute"
};
static const struct soc_enum es9018_mute_state_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(es9018_mute_state_texts),
			es9018_mute_state_texts);

static const char * const hifi_switch_pwr_texts[] = {
	"Off","On"
};

static const char * const hifi_modes_texts[] = {
	"Master","Slaves"
};

static const char * const es9018_i2s_length_texts[] = {
	"bit16", "bit24", "bit32", "bit32"
};

static const char * const es9018_clk_divider_texts[] = {
	"DIV4", "DIV8", "DIV16", "DIV16"
};

static const struct soc_enum hifi_mode_enum = 
	SOC_ENUM_SINGLE(SND_SOC_NOPM,0,ARRAY_SIZE(hifi_modes_texts),hifi_modes_texts);


static const struct soc_enum hifi_switch_pwr_enum = 
	SOC_ENUM_SINGLE(SND_SOC_NOPM,0,ARRAY_SIZE(hifi_switch_pwr_texts),hifi_switch_pwr_texts);

static const struct soc_enum es9018_i2s_length_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9018_i2s_length_texts),es9018_i2s_length_texts);

static const struct soc_enum es9018_clk_divider_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(es9018_clk_divider_texts),es9018_clk_divider_texts);



static struct snd_kcontrol_new es9018_digital_ext_snd_controls[] = {
	/* commit controls */
	
	SOC_ENUM_EXT("Set HIFI es9018 State", es9018_power_state_enum,
		       es9018_get_power_state_enum, es9018_put_power_state_enum),
	// SOC_ENUM_EXT("Set HIFI es9018 volume", es9018_gain_state_enum,
	//			   es9018_get_volume_enum, es9018_put_volume_enum),

	SOC_ENUM_EXT("Es9018 I2s Length",es9018_i2s_length_enum,es9018_get_i2s_length, es9018_set_i2s_length),
	SOC_ENUM_EXT("Es9018 CLK Divider",es9018_clk_divider_enum,es9018_get_clk_divider, es9018_set_clk_divider),	
	SOC_SINGLE_EXT("Es9018 Crystal 45M Switch", SND_SOC_NOPM, CRYSTAL_45M,	0, 0, crystal_45M_get, crystal_45M_set),	
	SOC_SINGLE_EXT("Es9018 Crystal 49M Switch", SND_SOC_NOPM, CRYSTAL_49M,	0, 0, crystal_49M_get, crystal_49M_set),
		
	/* Add by Harry */
	SOC_ENUM_EXT("HI-FI Clk_48K Enable", es9018_clk_state_enum,
				   es9018_get_clk_enum, es9018_put_clk_enum),

	SOC_ENUM_EXT("HI-FI Clk_44_1K Enable", es9018_clk_state_enum,
				   es9018_get_clk_44_1k_enum, es9018_put_clk_44_1k_enum),
	/*
	 * For HI-FI switch
	 */
	SOC_ENUM_EXT("HI-FI Switch Mute",es9018_mute_state_enum,
					es9018_get_switch_mute,es9018_put_switch_mute),
	SOC_ENUM_EXT("HI-FI Switch Sel",es9018_switch_sel_enum,
					es9018_get_switch_sel_enum,es9018_put_switch_sel_enum),
	SOC_ENUM_EXT("HI-FI Switch Power",hifi_switch_pwr_enum,
					hifi_get_switch_pwr_enum,hifi_put_switch_pwr_enum),

	SOC_SINGLE_EXT("HIFI Volume",SND_SOC_NOPM, 0, 255, 0,
		       hifi_volume_get, hifi_volume_set),

	SOC_ENUM_EXT("HI-FI Mode",hifi_mode_enum,
					hifi_get_mode_enum,hifi_put_mode_enum),

	/* End of Harry */ 
};

static int es9018_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int es9018_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret,i;

	for (i=0; i< 3; i++)
	{
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0)
		{
			dev_err(&client->dev, "%s: err %d,and try again\n", __func__, ret);
			mdelay(50);
		}
		else {
			pr_debug("es9018_write_reg OK!!\n");
			break;
		}
	}

	if (ret < 0)
		dev_err(&client->dev, "%s: write reg %d err %d\n", __func__,reg, ret);

	return ret;
}

static int es9018_populate_get_pdata(struct device *dev,
			struct es9018_data *pdata)
{
	int ret;
	u32 i;
	char name[20];
	
	pdata->reset_gpio = of_get_named_gpio(dev->of_node,
				"hi-fi,reset-gpio", 0);
	if (pdata->reset_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"dac,reset-gpio", dev->of_node->full_name,
			pdata->reset_gpio);
		ret = pdata->reset_gpio;
		goto err;
	}
	dev_dbg(dev, "%s: reset gpio %d", __func__, pdata->reset_gpio);

	/*
	 * Del by Harry
	 * Not nee I2C GPIO config
	 * Add new power config
	 */
#if 0	 
	pdata->i2c_scl_gpio= of_get_named_gpio(dev->of_node,
				"dac,i2c-scl-gpio", 0);
	if (pdata->i2c_scl_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"dac,i2c-scl-gpio", dev->of_node->full_name,
			pdata->i2c_scl_gpio);
		goto err;
	}
	dev_dbg(dev, "%s: i2c_scl_gpio %d", __func__, pdata->i2c_scl_gpio);

	pdata->i2c_sda_gpio= of_get_named_gpio(dev->of_node,
				"dac,i2c-sda-gpio", 0);
	if (pdata->i2c_sda_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"dac,i2c-sda-gpio", dev->of_node->full_name,
			pdata->i2c_sda_gpio);
		goto err;
	} 
	dev_dbg(dev, "%s: power gpio %d", __func__, pdata->power_gpio);

	pdata->power_gpio= of_get_named_gpio(dev->of_node,
				"dac,power-gpio", 0);
	if (pdata->power_gpio < 0) {
		dev_err(dev, "Looking up %s property in node %s failed %d\n",
			"dac,power-gpio", dev->of_node->full_name,
			pdata->power_gpio);
		goto err;
	}

	dev_dbg(dev, "%s: power gpio %d", __func__, pdata->power_gpio);
#endif
	/* Get switch GPIO */
	pdata->swi_mute_gpio = of_get_named_gpio(dev->of_node,"swi,mute_gpio",0);
	if (pdata->swi_mute_gpio < 0) {
		dev_err(dev,"Get switch mute GPIO error\n");
		goto err;
	}
	pdata->swi_sel_gpio = of_get_named_gpio(dev->of_node,"swi,sel_gpio",0);
	if (pdata->swi_sel_gpio < 0) {
		dev_err(dev,"Get switch sel GPIO error\n");
		goto err;
	}
	
	/*
	 * Get GPIO number
	 */
	ret = of_property_read_u32(dev->of_node,"hi-fi,pwr_pin_num",&pdata->pwr_num);
	if (ret < 0) {
		dev_err(dev,"Get hi-fi,pwr_pin_num error set to default value 3!\n");
		pdata->pwr_num = 2;
		// goto err;
	}
	if (pdata->pwr_num <= 0) {
		dev_err(dev,"Get hi-fi,pwr_pin_num error!\n");
		ret = -EINVAL;
		goto err;
	}
	/* Load gpio */
	pdata->power_gpio = (int*)devm_kmalloc(dev,sizeof(int) * pdata->pwr_num,GFP_KERNEL);
	if (! pdata->power_gpio) {
		dev_err(dev,"Malloc memory error!!\n");
		ret = -ENOMEM;
		goto err;
	}
	for (i = 0; i < pdata->pwr_num; i ++) {
		sprintf(name,"hi-fi,pwr_gpio_%d",i);
		pdata->power_gpio[i] = of_get_named_gpio(dev->of_node,name,0);
		if (pdata->power_gpio[i] < 0) {
			dev_err(dev,"Get GPIO name %s error!!\n",name);
			ret = pdata->power_gpio[i];
			goto err1;
		}
	}

	/* Clock GPIO */
	/* 48K */
	pdata->clk_en_gpio = of_get_named_gpio(dev->of_node,"hi-fi,clk1-en-gpio",0);
	if (pdata->clk_en_gpio < 0) {
		dev_err(dev,"Get GPIO name %s error!!\n","hi-fi,clk1-en-gpio");
		ret = pdata->clk_en_gpio;
		goto err1;
	}
	/* 44.1k */
	pdata->clk_441k = of_get_named_gpio(dev->of_node,"hi-fi,clk2-en-gpio",0);
	if (pdata->clk_441k < 0) {
		dev_err(dev,"Get GPIO name %s error!!\n","hi-fi,clk2-en-gpio");
		ret = pdata->clk_441k;
		goto err1;
	}

	/* Switch pwr GPIO */
	pdata->swi_pwr_gpio = of_get_named_gpio(dev->of_node,"swi,pwr_gpio",0);
	if (pdata->swi_pwr_gpio < 0) {
		dev_err(dev,"Get GPIO name %s error!!\n","swi,pwr_gpio");
		ret = pdata->swi_pwr_gpio;
		goto err1;
	}
	

	pdata->tfa9890_rst_gpio = of_get_named_gpio(dev->of_node,"tfa9890,reset-gpio",0);
	if (pdata->tfa9890_rst_gpio < 0) {
		dev_err(dev,"Get GPIO name %s error!!\n","tfa9890,reset-gpio");
		ret = pdata->tfa9890_rst_gpio;
		goto err1;
	}

	/* 
	 * For power VDD 
	 * It will del,because it used by LCD and HI-FI
	 */
	pdata->dvdd_2_7 = regulator_get(dev,"hi-fi,vdd_core");
	if (IS_ERR(pdata->dvdd_2_7)) {
		dev_err(dev,"Get VDD 2.7V error!!\n");
		goto err1;
	}
	if (regulator_set_voltage(pdata->dvdd_2_7,2650000,2750000)) {
		dev_err(dev,"regulator_set_voltage error!!\n");
		goto err1;
	}
	
	/* End of Harry */

	return 0;
	
err1:
	devm_kfree(dev,pdata->power_gpio);
err:
	devm_kfree(dev, pdata);
	
	return ret;
}

static unsigned int es9018_codec_read(struct snd_soc_codec *codec,
				      unsigned int reg)
{
	//struct es9018_priv *priv = codec->control_data;
	return 0;
}

static int es9018_codec_write(struct snd_soc_codec *codec, unsigned int reg,
			      unsigned int value)
{
	//struct es9018_priv *priv = codec->control_data;
	return 0;
}


static int es9018_set_bias_level(struct snd_soc_codec *codec,
				 enum snd_soc_bias_level level)
{
	int ret = 0;

	/* dev_dbg(codec->dev, "%s(codec, level = 0x%04x): entry\n", __func__, level); */

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		break;

	case SND_SOC_BIAS_OFF:
		break;
	}
	codec->dapm.bias_level = level;

	/* dev_dbg(codec->dev, "%s(): exit\n", __func__); */
	return ret;
}

static int es9018_suspend(struct snd_soc_codec *codec)
{
	es9018_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int es9018_resume(struct snd_soc_codec *codec)
{
	es9018_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static int es9018_get_div_value(int sample_rate,struct clock_div *div)
{
	int m_div,b_div;

	if (! div)
		return -EINVAL;

	m_div = 0;
	b_div = 0;
	if (sample_rate % 44100 == 0) {
		m_div = 1;
		b_div = sample_rate / 44100;
	} else if (sample_rate % 48000 == 0) {
		m_div = 1;
		b_div = sample_rate / 48000;
	} else if (sample_rate % 11025) {
		b_div = 4;
		m_div = 44100 / sample_rate;
	} else {
		b_div = 1;
		m_div = 1;
	}

	printk("%s:b_div = %d,m_div = %d\n",__func__,b_div,m_div);
		

	switch (b_div) {
		case 1 : 
			div->bit_div = 3;
			break;
		case 2 : 
			div->bit_div = 1;
			break;
		case 4 : 
			div->bit_div = 0;
			break;
		default : 
			return -EINVAL;
	}

	switch (m_div) {
		case 1 : 
			div->master_div = 0;
			break;
		case 2 : 
			div->master_div = 0x0C;
			break;
		case 4 : 
			div->master_div = 0x0E;
			break;
		default : 
			return -EINVAL;
	}

	return 0;
}

static int es9018_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *codec_dai)
{
	int rate,val,reg_val;
	struct clock_div div;
	
	printk("%s\n",__func__);

	rate = params_rate(params);
	printk("Sample rate = %d\n",rate);
	printk("Sample channels = %d\n",params_channels(params));
	printk("Sample bits = %d\n",hw_param_interval_c(params, SNDRV_PCM_HW_PARAM_SAMPLE_BITS)->min);

	memset(&div,0,sizeof(struct clock_div));
	val = es9018_get_div_value(rate,&div);
	printk("%s: b_div value = %d,m_div = %d\n",__func__,div.bit_div,div.master_div);
	if (val) {
		pr_err("%s:es9018_get_div_value with error!!\n",__func__);
		return val;
	}

	printk("%s: b_div value = %d,m_div = %d\n",__func__,div.bit_div,div.master_div);

	// If not power on ES9018,save the div
	if (! es9018_power_state) {
		printk("%s:ES9018 not power on,save the div value!\n",__func__);
		hifi_mclk_div = div.bit_div;
		return 0;
	}
	/*
	 * Set bit clk div
	 */
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,MASTER_MODE_CONTROL);	
	reg_val &= ~(I2S_CLK_DIVID_MASK);	
	reg_val |=  (div.bit_div << 5);	
	es9018_write_reg(g_es9018_priv->i2c_client,MASTER_MODE_CONTROL, reg_val);

#if 0 // Not need set master clk div
	/*
	 * Set master clk div
	 */
	reg_val = es9018_read_reg(g_es9018_priv->i2c_client,SYSTEM_SETTING);	
	reg_val &= ~(0xF << 4);	
	reg_val |=  (div.master_div << 4);	
	es9018_write_reg(g_es9018_priv->i2c_client,SYSTEM_SETTING, reg_val);
#endif

	return 0;
}

static int es9018_mute(struct snd_soc_dai *dai, int mute)
{
	//struct snd_soc_codec *codec = codec_dai->codec;
	//struct es9018_priv *priv = codec->control_data;
	printk("%s mute = %d\n",__func__,mute);
	return 0;

}

static int es9018_set_clkdiv(struct snd_soc_dai *codec_dai, int div_id, int div)
{
	//struct snd_soc_codec *codec = codec_dai->codec;
	//struct es9018_priv *priv = codec->control_data;
	printk("%s div_id = %d,div = %d\n",__func__,div_id,div);
	return 0;
}

static int es9018_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	//struct snd_soc_codec *codec = codec_dai->codec;
	//struct es9018_priv *priv = codec->control_data;
	printk("%s,clk_id = %d,freq = %d,dir = %d\n",__func__,clk_id,freq,dir);
	return 0;
}


static int es9018_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	//struct snd_soc_codec *codec = codec_dai->codec;
	//struct es9018_priv *priv = codec->control_data;
	printk("%s fmt = 0x%x\n",__func__,fmt);
	return 0;
}

static int es9018_set_fll(struct snd_soc_dai *codec_dai,
			  int pll_id, int source, unsigned int freq_in,
			  unsigned int freq_out)
{
	//struct snd_soc_codec *codec = codec_dai->codec;
	//struct es9018_priv *priv = codec->control_data;
	printk("%s\n",__func__);
	return 0;
}


static int es9018_pcm_trigger(struct snd_pcm_substream *substream,
			      int cmd, struct snd_soc_dai *codec_dai)
{
	//struct snd_soc_codec *codec = codec_dai->codec;
	//struct es9018_priv *priv = codec->control_data;
	printk("%s\n",__func__);
	return 0;
}


static const struct snd_soc_dai_ops es9018_dai_ops = {
	 .hw_params	= es9018_pcm_hw_params,
	 .digital_mute	= es9018_mute,
	 .trigger	= es9018_pcm_trigger,
	 .set_fmt	= es9018_set_dai_fmt,
	 .set_sysclk	= es9018_set_dai_sysclk,
	 .set_pll	= es9018_set_fll,
	 .set_clkdiv	= es9018_set_clkdiv,
};

static struct snd_soc_dai_driver es9018_dai = {
	.name = "ES9018-hifi",
	.playback = {
		.stream_name = "Tertiary MI2S Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = ES9018_RATES,
		.formats = ES9018_FORMATS,
	},
	.capture = {
		 .stream_name = "Tertiary MI2S Capture",
		 .channels_min = 2,
		 .channels_max = 2,
		 .rates = ES9018_RATES,
		 .formats = ES9018_FORMATS,
	 },
	.ops = &es9018_dai_ops,
};

static  int es9018_codec_probe(struct snd_soc_codec *codec)
{
	int rc = 0;
	struct es9018_priv *priv = snd_soc_codec_get_drvdata(codec);
	dev_info(codec->dev, "%s(): entry\n", __func__);
	dev_info(codec->dev, "%s(): codec->name = %s\n", __func__, codec->name);
    printk("es9018_codec_probe !!!!!!!!!!");


	priv->codec = codec;

	codec->control_data = snd_soc_codec_get_drvdata(codec);

	dev_info(codec->dev, "%s(): codec->control_data = 0x%lld\n", __func__, (long long)codec->control_data);

	es9018_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	
	rc = snd_soc_add_codec_controls(codec, es9018_digital_ext_snd_controls,
					ARRAY_SIZE(es9018_digital_ext_snd_controls));
	if (rc)
		dev_err(codec->dev, "%s(): es325_digital_snd_controls failed\n", __func__);
	

	dev_info(codec->dev, "%s(): exit\n", __func__);

	return 0;
}

static int  es9018_codec_remove(struct snd_soc_codec *codec)
{
	struct es9018_priv *priv = snd_soc_codec_get_drvdata(codec);
	
	es9018_set_bias_level(codec, SND_SOC_BIAS_OFF);

	kfree(priv);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_es9018 = {
	.probe =	es9018_codec_probe,
	.remove =	es9018_codec_remove,
	.suspend = 	es9018_suspend,
	.resume =	es9018_resume,
	.read = es9018_codec_read,
	.write = es9018_codec_write,
	.set_bias_level = es9018_set_bias_level,
};

static ssize_t es9018_reg_show(struct class *class, struct class_attribute *attr,char *buf)
{
	char *p;
	int i;

	p = buf;
	
	for (i = 0; i <= 30; i ++)
		p += sprintf(p,"reg[%d] = 0x%x\n",i,es9018_read_reg(g_es9018_priv->i2c_client,i));

	return (long)p - (long)buf;
}


// Format: reg,val
static ssize_t es9018_reg_store(struct class *class, struct class_attribute *attr,
			const char *buf, size_t count)
{
	char *p;
	int reg,val,ret;
	
	p = strstr(buf,",");
	if (p == NULL)
		return -EINVAL;

	// Replace ',' to \0
	*p = '\0';
	p ++;
	
	reg = simple_strtoul(buf,NULL,10);
	val = simple_strtoul(p,NULL,16);

	printk("Set reg %d to 0x%x\n",reg,val);

	if (reg > 30 || val > 255)
		return -EINVAL;

	ret = es9018_write_reg(g_es9018_priv->i2c_client,reg,val);

	if (ret < 0)
		return ret;

	return count;
}

static struct class_attribute es9018_class_attr[] = {
	__ATTR(reg,0660,es9018_reg_show,es9018_reg_store),
	__ATTR_NULL,
};

static struct class es9018_class = {
	.name  = "es9018",
	.owner = THIS_MODULE,
	.class_attrs = es9018_class_attr,
};

static int es9018_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct es9018_priv *priv;
	struct es9018_data *pdata;
    int ret = 0;
	u32 i,j;
	
    printk("es9018_probe !!!!!!!!!!\n");

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "%s: no support for i2c read/write"
				"byte data\n", __func__);
		return -EIO;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct es9018_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = es9018_populate_get_pdata(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Parsing DT failed(%d)", ret);
			return ret;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(struct es9018_priv),
			    GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;
    priv->i2c_client = client;
	priv->es9018_data = pdata;
	i2c_set_clientdata(client, priv);

	
	g_es9018_priv = priv;

	/*
	 * Changed by Harry
	 */
#if 0	 
	ret = gpio_request(pdata->power_gpio, "es9018_power");
	if (ret < 0) {
		dev_err(&client->dev, "%s(): es9018 _power_gpio request failed",
			__func__);
		goto power_gpio_request_error;
	}
	ret = gpio_direction_output(pdata->power_gpio, 1);
	if (ret < 0) {
	pr_err("%s: speaker_PA direction failed\n",
			   __func__);
		goto power_gpio_request_error;
	}
	gpio_set_value(pdata->power_gpio, 0);
#else
	for (i = 0; i < pdata->pwr_num; i ++) {
		ret = gpio_request(pdata->power_gpio[i], "es9018_power");
		if (ret < 0) {
			/* Free the OK GPIO */
			for (j = 0; j < i; j ++)
				gpio_free(pdata->power_gpio[j]);
			dev_err(&client->dev,"Request gpio %d with error[%d]\n",pdata->power_gpio[i],ret);
			goto power_gpio_request_error;
		}
		ret = gpio_direction_output(pdata->power_gpio[i],0);
		if (ret < 0) {
			dev_err(&client->dev,"Gpio %d set output with 0 error[%d]\n",pdata->power_gpio[i],ret);
			goto power_gpio_request_error;
		}
	}
#endif

	ret = gpio_request(pdata->reset_gpio, "es9018_reset");
	if (ret < 0) {
		dev_err(&client->dev, "%s(): es325_reset request failed",
			__func__);
		goto reset_gpio_request_error;
	}
	ret = gpio_direction_output(pdata->reset_gpio,0);
	if (ret < 0) {
	pr_err("%s: speaker_PA direction failed\n",
			   __func__);
		goto reset_gpio_request_error;
	}
	// gpio_set_value(pdata->reset_gpio, 1);

	/* Clock en GPIO */
	ret = gpio_request(pdata->clk_en_gpio,"es9018_clk_48k_en");
	if (ret < 0) {
		dev_err(&client->dev,"Request GPIO %d error!\n",pdata->clk_en_gpio);
		goto reset_gpio_request_error;
	}
	(void)gpio_direction_output(pdata->clk_en_gpio,0);

	ret = gpio_request(pdata->clk_441k,"es9018_clk_44_1k_en");
	if (ret < 0) {
		dev_err(&client->dev,"Request GPIO %d error!\n",pdata->clk_441k);
		goto reset_gpio_request_error;
	}
	(void)gpio_direction_output(pdata->clk_441k,0);

	/* Switch GPIO */
	ret = gpio_request(pdata->swi_sel_gpio,"es9018_swi_sel");
	if (ret < 0) {
		dev_err(&client->dev,"Request GPIO %d error\n",pdata->swi_sel_gpio);
		goto swi_sel_error;
	}
	(void)gpio_direction_output(pdata->swi_sel_gpio,0);
	
	ret = gpio_request(pdata->swi_mute_gpio,"es9018_swi_mute");
	if (ret < 0) {
		dev_err(&client->dev,"Request GPIO %d error\n",pdata->swi_mute_gpio);
		goto swi_mute_error;
	}
	(void)gpio_direction_output(pdata->swi_mute_gpio,1);

	ret = gpio_request(pdata->swi_pwr_gpio,"swi_pwr_en");
	if (ret < 0) {
		dev_err(&client->dev,"Request GPIO %d error\n",pdata->swi_pwr_gpio);
		goto swi_pwr_error;
	}
	(void)gpio_direction_output(pdata->swi_pwr_gpio,0);
	
	if (client->dev.of_node)
		dev_set_name(&client->dev, "%s", "es9018-codec");

	ret = snd_soc_register_codec(&client->dev, &soc_codec_dev_es9018,
			&es9018_dai, 1);

	hifi_volume_save = -1;

	// Register a class for debug
	class_register(&es9018_class);

    return ret;

swi_pwr_error :
	gpio_free(pdata->swi_mute_gpio);
swi_mute_error :
	gpio_free(pdata->swi_sel_gpio);
swi_sel_error :
	gpio_free(pdata->clk_en_gpio);
reset_gpio_request_error:
	gpio_free(pdata->reset_gpio);
power_gpio_request_error:
	// gpio_free(pdata->power_gpio);
	for (i = 0; i < pdata->pwr_num; i ++) {
		gpio_free(pdata->power_gpio[i]);
	}
	devm_kfree(&client->dev,pdata->power_gpio);
	
	return ret;
}

static int es9018_remove(struct i2c_client *client)
{
	u32 i;
	struct es9018_priv *priv;
	struct es9018_data *pdata;

	snd_soc_unregister_codec(&client->dev);
	
	priv = (struct es9018_priv *)i2c_get_clientdata(client);
	if (! priv)
		return -EINVAL;
	
	pdata = priv->es9018_data;
	if (! pdata) {
		devm_kfree(&client->dev,priv);
		return -EINVAL;
	}
	/* Free GPIO */
	for (i = 0; i < pdata->pwr_num; i ++)
		gpio_free(pdata->power_gpio[i]);

	gpio_free(pdata->reset_gpio);
	gpio_free(pdata->clk_en_gpio);
	gpio_free(pdata->swi_mute_gpio);
	gpio_free(pdata->swi_sel_gpio);

	/* Free mem */
	devm_kfree(&client->dev,pdata->power_gpio);
	if (client->dev.of_node)
		devm_kfree(&client->dev,pdata);

	devm_kfree(&client->dev,priv);
	
	return 0;
}

static struct of_device_id es9018_match_table[] = {
	{ .compatible = "qcom,es9018-hifi", },
	{}
};

static const struct i2c_device_id es9018_id[] = {
	{ "es9018", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, isa1200_id);

static struct i2c_driver es9018_i2c_driver = {
	.driver	= {
		.name	= "ES9018-codec",
		.of_match_table = es9018_match_table,
	},
	.probe		= es9018_probe,
	.remove		= es9018_remove,
	//.suspend	= es9018_suspend,
	//.resume		= es9018_resume,
	.id_table	= es9018_id,
};

static int __init es9018_init(void)
{
	return i2c_add_driver(&es9018_i2c_driver);
}

static void __exit es9018_exit(void)
{
	i2c_del_driver(&es9018_i2c_driver);
}

module_init(es9018_init);
module_exit(es9018_exit);

MODULE_DESCRIPTION("ASoC ES9018 driver");
MODULE_AUTHOR("ESS-LINshaodong");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:es9018-codec");


