/*
 * Author:             cesc.xu@gigasetdigital.com
 * Date:               2015/06/01
 * Discription: added. add gpio69 for TF support 
 */

#ifndef SDHCI_HOST_MSM_H
#define SDHCI_HOST_MSM_H


#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/gfp.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/mmc/mmc.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/dma-mapping.h>
#include <linux/irqchip/msm-mpm-irq.h>
#include <linux/iopoll.h>
#include <linux/pinctrl/consumer.h>
#include <linux/msm-bus.h>
/* by Cesc . for tf-sim burnt*/
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>

#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

#include <linux/init.h>

#include "sdhci-pltfm.h"



/* by Cesc . for tf-sim burnt */
struct mmc_gpio_ex {
	int ro_gpio;
	int cd_gpio;
	char *ro_label;
	bool status;
	char cd_label[0]; /* Must be last entry */
};

/* This structure keeps information per regulator */
struct sdhci_msm_reg_data {
	/* voltage regulator handle */
	struct regulator *reg;
	/* regulator name */
	const char *name;
	/* voltage level to be set */
	u32 low_vol_level;
	u32 high_vol_level;
	/* Load values for low power and high power mode */
	u32 lpm_uA;
	u32 hpm_uA;

	/* is this regulator enabled? */
	bool is_enabled;
	/* is this regulator needs to be always on? */
	bool is_always_on;
	/* is low power mode setting required for this regulator? */
	bool lpm_sup;
	bool set_voltage_sup;
};

/*
 * This structure keeps information for all the
 * regulators required for a SDCC slot.
 */
struct sdhci_msm_slot_reg_data {
	/* keeps VDD/VCC regulator info */
	struct sdhci_msm_reg_data *vdd_data;
	 /* keeps VDD IO regulator info */
	struct sdhci_msm_reg_data *vdd_io_data;
};

struct sdhci_msm_gpio {
	u32 no;
	const char *name;
	bool is_enabled;
};

struct sdhci_msm_gpio_data {
	struct sdhci_msm_gpio *gpio;
	u8 size;
};

struct sdhci_msm_pin_data {
	/*
	 * = 1 if controller pins are using gpios
	 * = 0 if controller has dedicated MSM pads
	 */
	u8 is_gpio;
	struct sdhci_msm_gpio_data *gpio_data;
};

struct sdhci_pinctrl_data {
	struct pinctrl          *pctrl;
	struct pinctrl_state    *pins_active;
	struct pinctrl_state    *pins_sleep;
};

struct sdhci_msm_bus_voting_data {
	struct msm_bus_scale_pdata *bus_pdata;
	unsigned int *bw_vecs;
	unsigned int bw_vecs_size;
};

struct sdhci_msm_pltfm_data {
	/* Supported UHS-I Modes */
	u32 caps;

	/* More capabilities */
	u32 caps2;

	unsigned long mmc_bus_width;
	struct sdhci_msm_slot_reg_data *vreg_data;
	bool nonremovable;
	bool nonhotplug;
	bool no_1p8v;
	bool pin_cfg_sts;
	struct sdhci_msm_pin_data *pin_data;
	struct sdhci_pinctrl_data *pctrl_data;
	u32 *cpu_dma_latency_us;
	unsigned int cpu_dma_latency_tbl_sz;
	int status_gpio; /* card detection GPIO that is configured as IRQ */
	struct sdhci_msm_bus_voting_data *voting_data;
	u32 *sup_clk_table;
	unsigned char sup_clk_cnt;
	int mpm_sdiowakeup_int;
	int sdiowakeup_irq;
	enum pm_qos_req_type cpu_affinity_type;
	u32 *cpu_affinity_mask;
	unsigned int cpu_affinity_mask_tbl_sz;
};

struct sdhci_msm_bus_vote {
	uint32_t client_handle;
	uint32_t curr_vote;
	int min_bw_vote;
	int max_bw_vote;
	bool is_max_bw_needed;
	struct delayed_work vote_work;
	struct device_attribute max_bus_bw;
};

struct sdhci_msm_host {
	struct platform_device	*pdev;
	void __iomem *core_mem;    /* MSM SDCC mapped address */
	int	pwr_irq;	/* power irq */
	struct clk	 *clk;     /* main SD/MMC bus clock */
	struct clk	 *pclk;    /* SDHC peripheral bus clock */
	struct clk	 *bus_clk; /* SDHC bus voter clock */
	struct clk	 *ff_clk; /* CDC calibration fixed feedback clock */
	struct clk	 *sleep_clk; /* CDC calibration sleep clock */
	atomic_t clks_on; /* Set if clocks are enabled */
	struct sdhci_msm_pltfm_data *pdata;
	struct mmc_host  *mmc;
	struct sdhci_pltfm_data sdhci_msm_pdata;
	u32 curr_pwr_state;
	u32 curr_io_level;
	struct completion pwr_irq_completion;
	struct sdhci_msm_bus_vote msm_bus_vote;
	struct device_attribute	polling;
#ifdef GIGASET_EDIT
/* cesc.xu@swdp.system, 2015/03/24. add property irqcounts for SD card hot-plug */
	int irq_counts; 
	/*struct device_attribute	irqcounts;*/
	int simulate_change;
	struct device_attribute	simulating;

	struct delayed_work vreg_enable_work;
	struct delayed_work pwr_state_work; /* by Cesc */
	struct delayed_work pwr_boot_work; /* boot without SD card */
	struct delayed_work pwr_boot_delay_irq_work;
	struct delayed_work pwr_boot_delay_check_work;
#endif  /* GIGASET_EDIT */
	u32 clk_rate; /* Keeps track of current clock rate that is set */
	bool tuning_done;
	bool calibration_done;
	u8 saved_tuning_phase;
	bool en_auto_cmd21;
	struct device_attribute auto_cmd21_attr;
	bool is_sdiowakeup_enabled;
	atomic_t controller_clock;
	bool use_cdclp533;
	bool use_updated_dll_reset;
	u32 caps_0;
};

enum vdd_io_level {
	/* set vdd_io_data->low_vol_level */
	VDD_IO_LOW,
	/* set vdd_io_data->high_vol_level */
	VDD_IO_HIGH,
	/*
	 * set whatever there in voltage_level (third argument) of
	 * sdhci_msm_set_vdd_io_vol() function.
	 */
	VDD_IO_SET_LEVEL,
};

int mmc_gpio_get_cd_ex(struct mmc_host *host);

#endif

