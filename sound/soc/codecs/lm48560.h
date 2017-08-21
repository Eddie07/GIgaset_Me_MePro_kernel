/*
 * Driver for LM48560
 *
 * Author : Harry
 */

#ifndef __LM48560__
#define __LM48560__

/*
 * LM48560 register
 */
#define SHUTDOWN_CONTROL 0x00
#define NOCLIP_CONTROL   0x01
#define GAIN_CONTROL     0x02
#define TEST_MODE        0x03
 
#define LM48560_MAX_GAIN  3

struct revpa_priv {
	int shdn_gpio;
	struct snd_soc_codec *codec;
	struct i2c_client *i2c;
	/* Save kcontrols status */
	int boost_en;
	int pwr_stu;
	int gain_val;
	int clip_val;
};

extern int revpa_controls_registe(struct snd_soc_codec *codec);

#endif // __LM48560__


