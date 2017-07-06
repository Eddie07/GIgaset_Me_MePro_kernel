/* Copyright (c) 2015 GIGASET Inc. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */



#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>


#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif


//#include <synaptics_dsx/synaptics_dsx_core.h>  //NG
#include <linux/input/synaptics_dsx_core.h> 
//#include "synaptics_dsx_core.h"    
//#include <synaptics_dsx_core.h>   

#include <linux/input/touch_common_debug.h> 


#define TOUCH_COMMON_FEATURE



extern bool touch_common_log;
extern bool touch_slim_log;
extern bool touch_monitor_log;
//extern bool touch_common_inited;

//extern bool tp_resume_valid;
extern struct touch_stc *g_touch_info;

#if 1//def 
//#define ht_print(fmt, arg...) 	      printk("[ctp jack:]"fmt"\n",##arg)
#define ht_print_if_c( format, arg...)  do {if(touch_common_log)printk("___ "format"\n", ##arg); } while (0)  
#define ht_print_if_s( format, arg...)  do {if(touch_slim_log)printk("___ "format"\n", ##arg); } while (0)  
#define ht_print_if_m( format, arg...)  do {if(touch_monitor_log)printk("___ "format"\n", ##arg); } while (0)  

#define ht_print( format, arg...)  printk("___ "format"\n", ##arg)  // printk("[ctp]"format"\n", ##arg)
#define ht_print_wo_n( format, arg...)  printk("___ "format, ##arg)  // printk("[ctp]"format"\n", ##arg)

#else
//#define ht_print(fmt, arg...) do {} while (0)
#define ht_print_if( format, arg...)  do {} while (0)
#define ht_print( format, arg...)  do {} while (0)
#define ht_print_wo_n( format, arg...)  do {} while (0)
#endif


#if 0//def NEW_07_08 //----------------------------------------------------jerry li //Drv. 2015_07_08 //
#define CTP_JACK  "[ctp_jerry] "
#ifdef dev_err
#undef dev_err
#endif
#define dev_err(dev, format, ...)	printk(CTP_JACK format, ## __VA_ARGS__)
#ifdef pr_err
#undef pr_err
#endif
//#define pr_err( format, ...)    printk(format, ## __VA_ARGS__)//
#define pr_err( format, arg...)   	printk(CTP_JACK format, ##arg)
#ifdef dev_dbg
#undef dev_dbg
#endif
#define dev_dbg(dev, format, arg...)   printk(CTP_JACK format, ##arg)  //dev_printk(KERN_DEBUG, dev, format, ##arg)
#ifdef pr_info
#undef pr_info
#endif
#define pr_info(dev, format, arg...)     printk(CTP_JACK format, ##arg)  //dev_printk(KERN_DEBUG, dev, format, ##arg)

#endif //-----------------------------------------------------------------#ifdef NEW_07_08_ //Drv. //




/////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////

//jerry li#swdp.driver,  2015/11/09, add for TOUCH_COMMON_FEATURE



//#ifdef TOUCH_COMMON_FEATURE

//#define ERROR_TOUCH_AREA_DETECTION
//#define TOUCH_DATA_IMPROVING_SOLUTION









struct touch_info_syn {
	int y;
	int x;
	int xw;	
	int yw;		
	unsigned char	finger;
	unsigned char	finger_status;
	unsigned char	fingers_to_process;
	unsigned char touch_count;
};

struct touch_info {
    int y[10];
    int x[10];
    int p[10];
    int id[10];
    int w[10];	
};

struct point_stc {
	int x;	
	int y;
	int wx;	
	int wy;	

	int step;
	u8 down_up;
	
	char status;	
	char last_status;
	char type;		
//	static u8 finger_last=0;	

	int last_x;
	int last_y;

	int diff_x;
	int diff_y;	
	int last_diff_x;
	int last_diff_y;	

	int abs_diff_x;
	int abs_diff_y;	
	int abs_last_diff_x;
	int abs_last_diff_y;	
	
//jerry li#swdp.driver,  2016/02/16, add for 
	int abs_last_last_diff_x;
	int abs_last_last_diff_y;	
	
	bool flag_point_delete;
	bool flag_point_delete_last;	
	
	bool start_point_in_detectiong_area;
	int start_point_pos_x;	
	int start_point_pos_y;
	bool point_valid;
	bool point_moving_valid;	

	u16 up_start_time;

	u64 time_down;
	u64 time_up;	
	u32 time_diff;	

	u32 sum_diff_x;
	u32 sum_diff_y;	

	int aver_speed_x;
	int aver_speed_y;	
	int aver_last_speed_x;
	int aver_last_speed_y;		
};




struct touch_stc {

	struct point_stc point[10];
	struct input_dev *input_dev;
	u8 id;

	u8 cur_point;	

	u8 fingers_to_process;		
	u8 fingers_to_process_last;	

	char log_buf1[128];
	char log_buf2[128];	
	bool log_buf_valid;
	
	struct work_struct touch_slim_log_work; 	
	struct work_struct touch_monitor_reset_work; 	
//function 
	bool eare_eage_function;
	bool super_monitor_function;
	bool data_adapter_function;	
	bool super_fliter_function;	
//for debug
	struct work_struct ht_debug_root_work; // menu;
	char cmd_buf[32];		
	char *ht_debug_cmd;

	u16 warning_cnt;
	bool flag_10sec_timer_working;
	bool flag_reset_3s_available; 
	u32 warning_cnt_arr[10];
	u16 reset_cnt;	

	u16 wxx;	
	u16 wyy;		
	
};






extern  struct delayed_work ctp_power_onoff_work;
extern  struct workqueue_struct *ctp_power_onoff_workqueue;
extern  void ctp_power_onoff_func(struct work_struct *work);

extern  ssize_t ht_enable_log_show(struct device *dev,struct device_attribute *attr, char *buf);
extern  ssize_t ht_enable_log_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count);
extern  ssize_t ht_debug_show(struct device *dev,struct device_attribute *attr, char *buf);
extern  ssize_t ht_debug_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count);



//static int check_is_point_in_detection_area(int x, int y);


int touch_common_start(struct touch_stc *touch_info) ;
int touch_common_end(struct touch_stc *touch_info) ;
int touch_eare_eage_pro(struct touch_stc *touch_info,bool *is_delete) ;
int touch_data_adapter(struct touch_stc *touch_info, struct synaptics_rmi4_data *rmi4_data); //jacky li //Drv. 2015_03_02 //
int touch_super_monitoring(struct touch_stc *touch_info,struct synaptics_rmi4_data *rmi4_data); 
int touch_super_power(struct touch_stc *touch_info,struct synaptics_rmi4_data *rmi4_data); //jacky li //Drv. 2015_03_02 //
int touch_super_filter(struct touch_info *cinfo, u8 *point_num);	

int  touch_common_init(struct input_dev *input_dev);
int  touch_common_exit(void);


bool  touch_common_is_inited(void);
bool touch_common_suspend(void);

u64 get_sys_time_ms(u64 *time_ms);
/* Removed  //jerry li  Drv. 2015_11_09 

/////////////////////////////////////////////////////////////////////////////
#ifdef ERROR_TOUCH_AREA_DETECTION
	if(false==touch_eare_eage_pro(cinfo, point_num,null))
		return false;

#endif
#ifdef TOUCH_DATA_IMPROVING_SOLUTION
	touch_data_improving_pro(cinfo, point_num);

#endif	
/////////////////////////////////////////////////////////////////////////////

*/

//#endif


