/*
 * Driver for LM48560
 *
 * Author : Harry
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <sound/soc.h>

#include "lm48560.h"

static struct revpa_priv *priv = NULL;

static int revpa_i2c_write(struct i2c_client *cli,int addr,unsigned char val)
{
	int ret;
	unsigned char buf[2];
	
	struct i2c_msg msg;
	
	
	if (addr >= TEST_MODE)
		return -EINVAL;

	buf[0] = addr;
	buf[1] = val;
	
	msg.addr  = cli->addr;
	msg.flags = 0;
	msg.buf   = buf;
	msg.len   = 2;


	ret = i2c_transfer(cli->adapter,&msg,1);
	if (ret < 0) {
		pr_err("%s:Send i2c message error %d\n",__func__,ret);
		return ret;
	}

	return (ret == 1) ? 0 : -EIO;
}

static int revpa_enable_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{	
	pr_debug("%s:pwr_stu = %d\n",__func__,priv->pwr_stu);
	
	ucontrol->value.integer.value[0] = priv->pwr_stu;

	return 0;
}

static int revpa_enable_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	unsigned char val;
	int ret;
	
	if (priv->pwr_stu == ucontrol->value.integer.value[0]) {
		pr_debug("%s:Not change!\n",__func__);
		return 0;
	}
	
	priv->pwr_stu = ucontrol->value.integer.value[0];

	pr_debug("%s:set shdn stu = %d\n",__func__,priv->pwr_stu);
	
	if (priv->pwr_stu) {
		__gpio_set_value(priv->shdn_gpio,1);
		// The delay value must make sure!!!!
		msleep(6);
		// Turn on,not show down
		// See the datasheet in page13
		val = 0x0D;
		if (priv->boost_en)
			val |= 0x02;
		ret = revpa_i2c_write(priv->i2c,SHUTDOWN_CONTROL,val);
		// Set the gain to the min value
		if (! ret) {
			ret = revpa_i2c_write(priv->i2c,GAIN_CONTROL,0);
			priv->gain_val = 0;
		}
	} else {
		ret = revpa_i2c_write(priv->i2c,SHUTDOWN_CONTROL,0x00);
		__gpio_set_value(priv->shdn_gpio,0);
	}

	return ret;
}

static int revpa_boost_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{	
	pr_debug("%s:boost_en = %d\n",__func__,priv->boost_en);
	
	ucontrol->value.integer.value[0] = priv->boost_en;

	return 0;
}

static int revpa_boost_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{	
	if (priv->boost_en == ucontrol->value.integer.value[0]) {
		pr_debug("%s:Not change!\n",__func__);
		return 0;
	}
	
	priv->boost_en = ucontrol->value.integer.value[0];

	pr_debug("%s:set boost_en stu = %d\n",__func__,priv->boost_en);

	return 0;
}

static int revpa_gain_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	ucontrol->value.integer.value[0] = priv->gain_val;

	return 0;
}

static int revpa_gain_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	int gain,ret;
	struct i2c_client *cli = priv->i2c;

	gain = ucontrol->value.integer.value[0];

	if (gain == priv->gain_val)
		return 0;

	if (gain < 0 || gain > LM48560_MAX_GAIN) {
		pr_err("%s:Set gain with error value %d\n",__func__,gain);
		return -EINVAL;
	}

	pr_debug("%s:Set gain to %d\n",__func__,gain);

	ret = revpa_i2c_write(cli,GAIN_CONTROL,gain);
	if (ret) 
		pr_err("%s:Write I2C error!!\n",__func__);
	else
		priv->gain_val = gain;

	return ret;
}

static int revpa_clip_get(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	ucontrol->value.integer.value[0] = priv->clip_val;

	return 0;
}

static int revpa_clip_put(struct snd_kcontrol * kcontrol, struct snd_ctl_elem_value * ucontrol)
{
	int clip,ret;
	struct i2c_client *cli = priv->i2c;

	clip = ucontrol->value.integer.value[0];

	if (clip == priv->clip_val)
		return 0;

	if (clip < 0 || clip > 255) {
		pr_err("%s:Set clip with error value %d\n",__func__,clip);
		return -EINVAL;
	}

	pr_debug("%s:Set clip to %d\n",__func__,clip);

	ret = revpa_i2c_write(cli,NOCLIP_CONTROL,clip);
	if (ret) 
		pr_err("%s:Write I2C error!!\n",__func__);
	else
		priv->clip_val = clip;

	return ret;
}

/*
 * Note that :
 * Follow gain only show disable boost
 * if boost enable,the gain show in comment
 */
static const char *const revpa_gain_texts[] = {
	"dB0","dB6","dB12","dB18"
}; 

static const struct soc_enum revpa_gain = 
	SOC_ENUM_SINGLE(SND_SOC_NOPM,0, ARRAY_SIZE(revpa_gain_texts),revpa_gain_texts);

static const struct snd_kcontrol_new revpa_controls[] = {
	SOC_SINGLE_BOOL_EXT("RevPAEnable",0,revpa_enable_get,revpa_enable_put),
	SOC_SINGLE_BOOL_EXT("RevPABoostEnable",0,revpa_boost_get,revpa_boost_put),
	SOC_ENUM_EXT("RevPAGain",revpa_gain,revpa_gain_get,revpa_gain_put),
	SOC_SINGLE_EXT("NoClip",SND_SOC_NOPM, 0, 255, 0,
		       revpa_clip_get, revpa_clip_put),
};

int revpa_controls_registe(struct snd_soc_codec *codec)
{
	printk("revpa_controls_registe!!\n");
	
	if (priv) {
		priv->codec = codec;
		return snd_soc_add_codec_controls(codec,revpa_controls,ARRAY_SIZE(revpa_controls));
	}

	priv = (struct revpa_priv *)kzalloc(sizeof(struct revpa_priv),GFP_KERNEL);
	if (! priv) {
		pr_err("%s:No memory!\n",__func__);
		return -ENOMEM;
	}

	priv->codec = codec;
	
	return snd_soc_add_codec_controls(codec,revpa_controls,ARRAY_SIZE(revpa_controls));
}

EXPORT_SYMBOL(revpa_controls_registe);

static int revpa_probe(struct i2c_client *cli, const struct i2c_device_id *id)
{
	int ret = -EIO;

	printk("revpa_probe !!\n");
	
	if (! i2c_check_functionality(cli->adapter,I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&cli->dev,"i2c function error!\n");
		goto err;	
	}

	/*
	* The priv will inited by codec,so check it
	*/
	if (! priv) {
		priv = (struct revpa_priv*)kzalloc(sizeof(struct revpa_priv),GFP_KERNEL);
		if (! priv) {
			ret = -ENOMEM;
			dev_err(&cli->dev,"No memory!\n");
			goto err;
		}
	}

	/* Get the shut down GPIO */
	ret = of_get_named_gpio(cli->dev.of_node,"rev,shutdown-gpio",0);
	if (ret < 0) {
		dev_err(&cli->dev,"Get shut down GPIO error!\n");
		goto errm;
	}
	priv->shdn_gpio = ret;

	ret = gpio_request(priv->shdn_gpio,"revpa_shutdown");
	if (ret < 0) {
		dev_err(&cli->dev,"REQ shut down GPIO error!\n");
		goto errm;
	}
	// Defaule shotdown
	(void)gpio_direction_output(priv->shdn_gpio,0);

	priv->i2c = cli;
	i2c_set_clientdata(cli,priv);
	
	
	if (ret) 
		pr_err("%s:Write I2C error!!\n",__func__);
	else
		pr_err("%s:Write I2C OK!!\n",__func__);

	return 0;

errm :
	kfree(priv);
	priv = NULL;
err :
	return ret;
}

static int revpa_remove(struct i2c_client *cli)
{
	if (priv) {
		gpio_free(priv->shdn_gpio);
		kfree(priv);
		priv = NULL;
	}

	return 0;
}

static const struct i2c_device_id revpa_match_ids[] = {
    { "rev-lm48560", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, revpa_match_ids);

static const struct of_device_id revpa_match_table[] = {
	{
		.compatible = "qcom.rev-lm48560",
	},
	{}
};
MODULE_DEVICE_TABLE(of, revpa_match_table);

static struct i2c_driver revpa_drv = {
	.probe  = revpa_probe,
	.remove = revpa_remove,
	.id_table    = revpa_match_ids,
	.driver = {
		.name = "rev-lm48560", 
		.owner    = THIS_MODULE,
		.of_match_table = revpa_match_table, 
	}
};

static __init int revpa_init(void)
{
	printk("revpa_init !!!!!!!\n");
	return i2c_add_driver(&revpa_drv);
}

static __exit void revpa_exit(void)
{
	i2c_del_driver(&revpa_drv);
}

module_init(revpa_init);
module_exit(revpa_exit);



