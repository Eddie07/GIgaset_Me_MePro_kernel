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

#include <linux/input/mt.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif

#include <linux/touch_common.h>



//#include <time.h>
//#include <sys/time.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/syscalls.h>

/////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////




#if defined(GIGASET_EDIT)&&defined(TOUCH_COMMON_FEATURE)


//jerry li#swdp.driver,  2015/11/27, add for 
bool touch_common_inited=0;

//debug log types
bool touch_common_log=0;
bool touch_slim_log=0;
bool touch_monitor_log=0;


bool tp_power_on_off_test=0;
bool tp_resume_point_valid=0;
u32 tp_resume_cound=0;
u32 tp_resume_fail_cound=0;


	
struct touch_stc *g_touch_info;


extern int my_panel_id;



//(cinfo->finger_status>=1) FINGER_STATUS

#define FINGER_STATUS_UP    0x0  //
#define FINGER_STATUS_DOWN_AND_PRESS    0x01






enum{
	STEP_1_DOWM,
	STEP_2,
	STEP_3
}P_STEP;

enum{
	DOWN,  //0
	PRESS,  // 1
	UP,    // 2
	STANDBY   // 3	
}DOWN_UP;


/////////////////////////////////////////////////////////////////////////
#define TP_LCD_PIXEL_HEIGHT 1920
#define TP_LCD_PIXEL_WIDTH  1080


#define TP_VALID_POINT_MOVING   80 //100 //64 //64//48
#define TP_DETECTION_AREA_Y  1500 //1920  //1500 //1000
#define TP_DETECTION_AREA_X  15 //20 //100 //20 //469 20150313

/* 
-------------
|		 	|=(TP_LCD_PIXEL_HEIGHT-TP_DETECTION_AREA_Y)
|		 	|
|		----| --
| |		     |	| top_y
| |		     |	|
| |		     |	| bottom_y
|   |		   |	|
---------------
 left_x        |  right_x |
(TP_LCD_PIXEL_WIDTH-TP_DETECTION_AREA_X)		
*/
static bool check_is_point_in_detection_area(int x, int y) 
{

#if 0//def TEST 
	if(((x>=(TP_LCD_PIXEL_WIDTH-50))||x<=50)
	&&(y>=(TP_LCD_PIXEL_HEIGHT-1400)))
	{
		return 1;
	}

	if(((x>=(TP_LCD_PIXEL_WIDTH-100))||x<=100)
	&&(y>=(TP_LCD_PIXEL_HEIGHT-120)))
	{
		return 1;
	}	

#else
	if(((x>=(TP_LCD_PIXEL_WIDTH-15))||x<=20)
	&&(y>=(TP_LCD_PIXEL_HEIGHT-1400)))
	{
		return 1;
	}

	if(((x>=(TP_LCD_PIXEL_WIDTH-30))||x<=30)
	&&(y>=(TP_LCD_PIXEL_HEIGHT-200)))
	{
		return 1;
	}
#endif
	return 0;
}

static bool monitor_check_is_point_in_detection_area(int x, int y) 
{

	if(((x>=(TP_LCD_PIXEL_WIDTH-50))||x<=50)
	&&(y>=(TP_LCD_PIXEL_HEIGHT-1600)))
	{
		return 1;
	}

	if(((x>=(TP_LCD_PIXEL_WIDTH-100))||x<=100)
	&&(y>=(TP_LCD_PIXEL_HEIGHT-400)))
	{
		return 1;
	}

	return 0;
}
#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif

//extern  synaptics_rmi4_exp_fn_data exp_data;


extern bool is_tp_fw_updating(void); //jerry li  Drv. 2016_01_08 
extern bool is_tp_suspend(void); 
extern void synaptics_touch_reset(void);
extern bool touch_monitor_check_sens_enable(void);
extern bool touch_monitor_check_sens_disable(void);
extern bool synaptics_low_sens_mode_enable_ll(void);
extern bool synaptics_low_sens_mode_disable_ll(void);		
extern bool  synaptics_is_tp_low_sens_mode(void);
extern void  synaptics_clean_low_sens_flag(void);
extern void gtp_reset_guitar(struct synaptics_rmi4_data *rmi4_data);
extern bool  synaptics_is_usb_insert(void);
extern bool synaptics_reg_write(u16 addr,u16 data);
extern bool synaptics_reg_read(u16 addr);


void print_warning_cnt_arr(void);

static struct delayed_work touch_timer_monitor_3s_work;
static struct workqueue_struct *touch_timer_monitor_3s_work_queue;
static void touch_timer_monitor_3s_work_fun(struct work_struct *work);

static struct delayed_work touch_timer_monitor_10s_work;
static struct workqueue_struct *touch_timer_monitor_10s_work_queue;
static void touch_timer_monitor_10s_work_fun(struct work_struct *work);

static struct delayed_work touch_timer_monitor_3min_work;
static struct workqueue_struct *touch_timer_monitor_3min_work_queue;
static void touch_timer_monitor_3min_work_fun(struct work_struct *work);

static void touch_timer_monitor_3s_work_fun(struct work_struct *work)
{
	g_touch_info->flag_reset_3s_available=true;
}
static void touch_timer_monitor_10s_work_fun(struct work_struct *work)
{
	ht_print("monitor warning_cnt %d reset_cnt %d clean to 0",g_touch_info->warning_cnt,g_touch_info->reset_cnt); //jerry li //Drv. 2016_01_14 //

	g_touch_info->warning_cnt=0;
	g_touch_info->reset_cnt=0;
	g_touch_info->flag_10sec_timer_working=false;
	g_touch_info->flag_reset_3s_available=true;
	
}

static void touch_timer_monitor_3min_work_fun(struct work_struct *work)
{
	ht_print("touch_timer_sens_3min_work_fun"); //jerry li //Drv. 2016_01_14 //
	
	touch_monitor_check_sens_disable();
	
}

void touch_monitor_reset_func(struct work_struct *work)
{
//	bool ret;
	g_touch_info->reset_cnt++;
		
	if(g_touch_info->reset_cnt==1)
	{

		ht_print("monitor warning[reset] ___________    warning_cnt=%d reset_cnt=%d usb=%d",
			g_touch_info->warning_cnt,g_touch_info->reset_cnt,synaptics_is_usb_insert()); 
		print_warning_cnt_arr();
		g_touch_info->warning_cnt=0;	

		synaptics_touch_reset();
		
	}
	else if(g_touch_info->reset_cnt>=2)		
	{
		ht_print("monitor warning[reset] ___________  warning_cnt=%d reset_cnt=%d usb=%d",
			g_touch_info->warning_cnt,g_touch_info->reset_cnt,synaptics_is_usb_insert());
		print_warning_cnt_arr();
		g_touch_info->warning_cnt=0;	
		
		synaptics_touch_reset();
		
		if(synaptics_is_usb_insert()) //jerry li  Drv. 2016_03_03 
		{
			ht_print("monitor warning[switch sens] ___________"); //jerry li //Drv. 2016_03_03 //

			if(!synaptics_is_tp_low_sens_mode())
			touch_monitor_check_sens_enable();
			cancel_delayed_work_sync(&touch_timer_monitor_3min_work);
			queue_delayed_work(touch_timer_monitor_3min_work_queue,&touch_timer_monitor_3min_work, 3*60*100);			
		}
		
//		g_touch_info->reset_cnt=0;
	}	

}
/////////////////////////////////////////////////////////////////////////////
// Function name:
// Purpose:
// Input Parameters:
// Return type:
// Output Parameters: 	NONE
/////////////////////////////////////////////////////////////////////////////

int touch_common_start(struct touch_stc *touch_info)
{

	u8 cur_id=0;

	
	struct touch_stc *touch=touch_info;


/////////////////////////////////////////////////////////////////////////////////////////		
	if(touch==NULL) 
	{
		ht_print_if_c("error:  touch_common_start   touch =NULL ");	
		return 0; 
	}	


	cur_id=touch->id;
//	cur_id=touch->cur_point;

	if(cur_id>=10) 
	return 0; 
	
/////////////////////////////////////////////////////////////////////////////////////////

	if(my_panel_id==4) // SHARP
	{
		if((touch->point[cur_id].status==0x60))//&&(touch->point[cur_id].last_status==0x80))
		{
			touch->point[cur_id].down_up=DOWN;
			touch->point[cur_id].step=STEP_1_DOWM;	  
		}
		else if((touch->point[cur_id].status==0x80))//&&(touch->point[cur_id].last_status==0x00))
		{
			touch->point[cur_id].down_up=UP;
			touch->point[cur_id].step++;
		}
		else if((touch->point[cur_id].status==0x00)&&(touch->point[cur_id].down_up==DOWN))  
		{
			touch->point[cur_id].down_up=PRESS;
			touch->point[cur_id].step++;		
		}
		else if((touch->point[cur_id].status==0x00)&&(touch->point[cur_id].down_up==UP))  
		{
			touch->point[cur_id].down_up=STANDBY;
		}

		if(touch->point[cur_id].down_up==PRESS)  
		{
			touch->point[cur_id].step++;		
		}

	}
	else 
	{
		if((touch->point[cur_id].status>=FINGER_STATUS_DOWN_AND_PRESS)&&(touch->point[cur_id].last_status==FINGER_STATUS_UP))
		{
			touch->point[cur_id].down_up=DOWN;
			touch->point[cur_id].step=STEP_1_DOWM;	  
		}
		else if((touch->point[cur_id].status==FINGER_STATUS_UP)&&(touch->point[cur_id].last_status>=FINGER_STATUS_DOWN_AND_PRESS))
		{
			touch->point[cur_id].down_up=UP;
			touch->point[cur_id].step++;
		}
		else if(touch->point[cur_id].status>=FINGER_STATUS_DOWN_AND_PRESS)  
		{
			touch->point[cur_id].down_up=PRESS;
			touch->point[cur_id].step++;		
		}
		else if(touch->point[cur_id].status==FINGER_STATUS_UP)
		{
			touch->point[cur_id].down_up=STANDBY;
		}		
	//	else //(cinfo->finger_status==0)
	//	{
	//       unknow_point=true;
	//	}

	}




/////////////////////////////////////////////////////////////////////////////////////////
	if(touch->point[cur_id].step==STEP_1_DOWM)//||(touch->point[cur_id].down_up==DOWN)
	{
		touch->point[cur_id].start_point_in_detectiong_area=0;
		touch->point[cur_id].start_point_pos_x=touch->point[cur_id].x;		
		touch->point[cur_id].start_point_pos_y=touch->point[cur_id].y;
		touch->point[cur_id].point_valid=false;
		touch->point[cur_id].point_moving_valid=0;
		touch->point[cur_id].flag_point_delete=0;
		touch->point[cur_id].flag_point_delete_last=0;		
		

#ifdef OLD_01_15 //----------------------------------------------------jerry li //Drv. 2016_01_15 //
//		touch->point[cur_id].last_x=0;
//		touch->point[cur_id].last_y=0;
		touch->point[cur_id].diff_x=0;
		touch->point[cur_id].diff_y=0;
		touch->point[cur_id].abs_diff_x=0;
		touch->point[cur_id].abs_diff_y=0;
		
		touch->point[cur_id].last_diff_x=0;
		touch->point[cur_id].last_diff_y=0;
		touch->point[cur_id].abs_last_diff_x=0;
		touch->point[cur_id].abs_last_diff_y=0;
#else
		touch->point[cur_id].diff_x=touch->point[cur_id].x-touch->point[cur_id].last_x;
		touch->point[cur_id].diff_y=touch->point[cur_id].y-touch->point[cur_id].last_y;

		touch->point[cur_id].abs_diff_x=(int)abs(touch->point[cur_id].diff_x);
		touch->point[cur_id].abs_diff_y=(int)abs(touch->point[cur_id].diff_y);		
#endif //-----------------------------------------------------------------OLD_01_15_
		touch->point[cur_id].aver_speed_x=0;
		touch->point[cur_id].aver_speed_y=0;
		touch->point[cur_id].aver_last_speed_x=0;
		touch->point[cur_id].aver_last_speed_y=0;	

//		touch->point[cur_id].aver_incremental_x=0;
//		touch->point[cur_id].aver_incremental_y=0;		
//		touch->point[cur_id].up_start_time=0;
#ifdef HT_ENABLE_LOG_INIT
		if((touch->point[cur_id].x<20)&&(touch->point[cur_id].y<=20)) //jerry li  Drv. 2015_09_22 
		{
			touch_common_log=!touch_common_log;	
			ht_print_if_c("touch_common_log =%d",touch_common_log);	
		}
#endif

		if(tp_power_on_off_test) //jerry li  Drv. 2016_01_06 
		{
			if((touch->point[cur_id].x>300)&&(touch->point[cur_id].x<=700)  //500  700
				&&(touch->point[cur_id].y>1100)&&(touch->point[cur_id].y<=1600)
				&&(touch->fingers_to_process==1) //1200 1400
			)
			{
				tp_resume_point_valid=true;	
			}
			else
			ht_print_if_c("test fail");		
		}


		get_sys_time_ms(&touch->point[cur_id].time_down);
		if(touch->point[cur_id].time_down>touch->point[cur_id].time_up)
		touch->point[cur_id].time_diff=(u32)(touch->point[cur_id].time_down-touch->point[cur_id].time_up);
		else
		touch->point[cur_id].time_diff=0xffff;
		
//		ht_print_if_c("%d down   %dms  (%lld-%lld)",cur_id+1,touch->point[cur_id].time_diff,touch->point[cur_id].time_down,touch->point[cur_id].time_up);
		ht_print_if_c("%d down %dms",cur_id+1,touch->point[cur_id].time_diff);

		
	}
	else if(touch->point[cur_id].step>=STEP_2)
	{
	
//		fix tdi up point (0,0) issue
		if((my_panel_id!=4)&&(touch->point[cur_id].down_up==UP)) 
		{
			touch->point[cur_id].x=touch->point[cur_id].last_x;
			touch->point[cur_id].y=touch->point[cur_id].last_y;
		}
	
		touch->point[cur_id].diff_x=touch->point[cur_id].x-touch->point[cur_id].last_x;
		touch->point[cur_id].diff_y=touch->point[cur_id].y-touch->point[cur_id].last_y;

		touch->point[cur_id].abs_diff_x=(int)abs(touch->point[cur_id].diff_x);
		touch->point[cur_id].abs_diff_y=(int)abs(touch->point[cur_id].diff_y);	

		if(touch->point[cur_id].step==STEP_2)
		{		
			touch->point[cur_id].aver_speed_x=touch->point[cur_id].abs_diff_x;
			touch->point[cur_id].aver_speed_y=touch->point[cur_id].abs_diff_y;
		}
		else if(touch->point[cur_id].step>=STEP_3)
		{
			touch->point[cur_id].aver_speed_x+=touch->point[cur_id].abs_diff_x;
			touch->point[cur_id].aver_speed_x=touch->point[cur_id].aver_speed_x>>1;
			touch->point[cur_id].aver_speed_y+=touch->point[cur_id].abs_diff_y;
			touch->point[cur_id].aver_speed_y=touch->point[cur_id].aver_speed_y>>1;
		}
		

/* Removed  //jerry li  Drv. 2016_01_21 
//
		if((touch->point[cur_id].diff_x>=0)&&(touch->point[cur_id].last_diff_x>=0))
		{
			touch->point[cur_id].aver_incremental_x+=touch->point[cur_id].abs_diff_x-touch->point[cur_id].abs_last_diff_x;
			touch->point[cur_id].aver_incremental_x=touch->point[cur_id].aver_incremental_x>>1;
		}
		else if((touch->point[cur_id].diff_x<0)&&(touch->point[cur_id].last_diff_x<0))
		{
			 touch->point[cur_id].aver_incremental_x+=touch->point[cur_id].abs_diff_x-touch->point[cur_id].abs_last_diff_x;
			 touch->point[cur_id].aver_incremental_x=touch->point[cur_id].aver_incremental_x>>1;
		}
		else
		touch->point[cur_id].aver_incremental_x=0;

		if(touch->point[cur_id].step>=STEP_3)
		{

		}
*/
	}

/////////////////////////////////////////////////////////////////////////////////////////

	if(touch->point[cur_id].down_up==UP) //jerry li  Drv. 2015_11_27 
	{

		get_sys_time_ms(&touch->point[cur_id].time_up);
	}

	if(touch->point[cur_id].down_up!=UP) //jerry li  Drv. 2015_11_27 
	{
		touch->wxx+=touch->point[cur_id].wx;
		touch->wxx>>=1;

		touch->wyy+=touch->point[cur_id].wy;
		touch->wyy>>=1;		
	}
/////////////////////////////////////////////////////////////////////////////////////////


#if 0//def OLD_12_26 //----------------------------------------------------jerry li //Drv. 2015_12_26 //
	if(touch->fingers_to_process_last!=touch->fingers_to_process)
	{
		if(touch->fingers_to_process>=touch->fingers_to_process_last)
		ht_print_if_c("add Num %d (last %d ), id= %d:",touch->fingers_to_process,touch->fingers_to_process_last,cur_id+1 );	
		else
		ht_print_if_c("leave  Num %d (last %d ), id= %d:",touch->fingers_to_process,touch->fingers_to_process_last,cur_id+1 );		
		
	}
#endif //-----------------------------------------------------------------OLD_12_26_

	ht_print_if_c("%d/%d %03d: p( %d,%d) dif[%d,%d] w[%d,%d] status=0x%x(last 0x%x)  id=%d  type=%d",
	cur_id+1,touch->fingers_to_process,
	touch->point[cur_id].step,
	touch->point[cur_id].x,touch->point[cur_id].y,
	touch->point[cur_id].diff_x,touch->point[cur_id].diff_y,	
	touch->point[cur_id].wx,touch->point[cur_id].wy,
	touch->point[cur_id].status,touch->point[cur_id].last_status,
	touch->id,touch->point[cur_id].type
//	cinfo->finger,cinfo->finger_status,cinfo->fingers_to_process,
//	is_point_in_detection_area ? "-->point in detection":""
	);	

	if((touch_common_log==0)) //jerry li  Drv. 2016_01_12 
	{
		touch->log_buf_valid=false;
		
		if(touch->point[cur_id].down_up==DOWN) //jerry li  Drv. 2015_11_27 
		{
			sprintf(touch->log_buf1, "%dms %d/%d %03d: p( %d,%d) dif[%d,%d] w[%d,%d] status=0x%x(last 0x%x)  id=%d  type=%d",
			touch->point[cur_id].time_diff,
			cur_id+1,touch->fingers_to_process,
			touch->point[cur_id].step,
			touch->point[cur_id].x,touch->point[cur_id].y,
			touch->point[cur_id].diff_x,touch->point[cur_id].diff_y,	
			touch->point[cur_id].wx,touch->point[cur_id].wy,
			touch->point[cur_id].status,touch->point[cur_id].last_status,
			touch->id,touch->point[cur_id].type
			);
//			touch->log_buf_valid=true;
		}
		else if(touch->point[cur_id].down_up==UP) //jerry li  Drv. 2015_11_27 
		{
			sprintf(touch->log_buf2, "up    %d/%d %03d: p( %d,%d) dif[%d,%d] w[%d,%d] status=0x%x(last 0x%x)  id=%d  type=%d",
			cur_id+1,touch->fingers_to_process,
			touch->point[cur_id].step,
			touch->point[cur_id].x,touch->point[cur_id].y,
			touch->point[cur_id].last_diff_x,touch->point[cur_id].last_diff_y,	
			touch->point[cur_id].wx,touch->point[cur_id].wy,
			touch->point[cur_id].status,touch->point[cur_id].last_status,
			touch->id,touch->point[cur_id].type
			);	
			touch->log_buf_valid=true;
			
			if(touch_slim_log==true)
			schedule_work(&touch->touch_slim_log_work);
		}
	}
	
/////////////////////////////////////////////////////////////////////////////////////////

		
	return true;	
}



int touch_common_end(struct touch_stc *touch_info) 
{

	u8 cur_id=0;

	
	struct touch_stc *touch=touch_info;



/////////////////////////////////////////////////////////////////////////////////////////	

	if(touch==NULL) 
	{
		ht_print_if_c(":error   touch_common_pro   touch =NULL ");	
		return 0; 
	}	

	cur_id=touch->id;

	if(cur_id>=10) 
	return 0; 	
//	cur_id=touch->cur_point;

//	if(touch->point[cur_id].down_up==STANDBY)
//	return 0;


/////////////////////////////////////////////////////////////////////////////////////////


	touch->fingers_to_process_last=touch->fingers_to_process;
	touch->point[cur_id].last_status=touch->point[cur_id].status;
	touch->point[cur_id].flag_point_delete_last=touch->point[cur_id].flag_point_delete;



	touch->point[cur_id].last_diff_x=touch->point[cur_id].diff_x;
	touch->point[cur_id].last_diff_y=touch->point[cur_id].diff_y;

//jerry li#swdp.driver,  2016/02/16, add for 
	touch->point[cur_id].abs_last_last_diff_x=touch->point[cur_id].abs_last_diff_x;
	touch->point[cur_id].abs_last_last_diff_y=touch->point[cur_id].abs_last_diff_x;

	
	touch->point[cur_id].abs_last_diff_x=(int)abs(touch->point[cur_id].last_diff_x);
	touch->point[cur_id].abs_last_diff_y=(int)abs(touch->point[cur_id].last_diff_y);
	

	touch->point[cur_id].last_x=touch->point[cur_id].x;
	touch->point[cur_id].last_y=touch->point[cur_id].y;


	touch->point[cur_id].aver_last_speed_x=touch->point[cur_id].aver_speed_x;
	touch->point[cur_id].aver_last_speed_y=touch->point[cur_id].aver_speed_y;	

//	touch->point[cur_id].last_aver_incremental_x=touch->point[cur_id].aver_incremental_x;		
/////////////////////////////////////////////////////////////////////////////////////////
	if(touch->point[cur_id].down_up==UP) //jerry li  Drv. 2015_11_27 
	{
		ht_print_if_c("%d up",cur_id+1);
              //reset
//		touch->point[cur_id].step=0;
//		eage eare detection  
		touch->point[cur_id].start_point_in_detectiong_area=0;
		touch->point[cur_id].start_point_pos_x=0;		   
		touch->point[cur_id].start_point_pos_y=0;
		touch->point[cur_id].point_valid=false;
		touch->point[cur_id].point_moving_valid=0;
		touch->point[cur_id].flag_point_delete_last=0;	

	}


	
		
	return true;	
}
	
int touch_eare_eage_pro(struct touch_stc *touch_info,bool *is_delete) 
{


	bool is_point_in_detection_area=0;
//	bool flag_point_delete=0;
//	static bool flag_point_delete_last[10]={0};
	u8 cur_id=0;

	
	struct touch_stc *touch=touch_info;
		
	
	if(touch==NULL) 
	{
		ht_print_if_c(":error   touch_eare_eage_pro   cinfo =NULL ");	
		return 0; 
	}	

	if((touch_info->eare_eage_function==0)
	||(touch->id>=10)
	||(touch->point[cur_id].down_up==STANDBY)
	)
	return 0;



	cur_id=touch->id;
	
	touch->point[cur_id].flag_point_delete=0;
/////////////////////////////////////////////////////////////////////////////////////////

	is_point_in_detection_area=check_is_point_in_detection_area(touch->point[cur_id].x,touch->point[cur_id].y);

	if((is_point_in_detection_area)&&(!touch->point[cur_id].point_valid))
	{

		if(touch->point[cur_id].step==STEP_1_DOWM)
		{
			touch->point[cur_id].start_point_in_detectiong_area=true;
		}

		if(touch->point[cur_id].step>=STEP_2)
		{
			if((abs(touch->point[cur_id].y-touch->point[cur_id].start_point_pos_y))>=TP_VALID_POINT_MOVING)	
			{
				if(!touch->point[cur_id].point_moving_valid)
				ht_print_if_c("touch->point[%d].point_moving_valid=1",cur_id+1);	
				touch->point[cur_id].point_moving_valid=true;
			}
		}

	}
	else
	{
		if((!touch->point[cur_id].point_valid)&&(touch->point[cur_id].down_up!=UP))
		{
//			ht_print_if_c("touch->point[cur_id].point_valid=1 %d",cur_id+1);	
			touch->point[cur_id].point_valid=true;
		}
	}



	if(is_point_in_detection_area
	&&!touch->point[cur_id].point_valid		
	&&touch->point[cur_id].start_point_in_detectiong_area
	&&!touch->point[cur_id].point_moving_valid		
	)
	{	
		ht_print_if_c("delete point %d",cur_id+1);	
		touch->point[cur_id].flag_point_delete=1;
	}

/* Removed  //jerry li  Drv. 2015_12_26 
//
//	if(is_point_in_detection_area) //jerry li  Drv. 2015_12_26 
	{
		ht_print_if_c("in detection %d %d %d %d %d",is_point_in_detection_area
		,touch->point[cur_id].point_valid		
		,touch->point[cur_id].start_point_in_detectiong_area
		,touch->point[cur_id].point_moving_valid
		,touch->point[cur_id].down_up);	
	}
*/


//fix tdi up point issue
	if(((touch->point[cur_id].down_up==UP)))
	{
		if(touch->point[cur_id].flag_point_delete_last==true)
		{
//			ht_print_if_c("delete point %d         up",cur_id+1);	
			touch->point[cur_id].flag_point_delete=1;
		}
	}




/////////////////////////////////////////////////////////////////////////////////////////
//	*is_delete=false;
	*is_delete=touch->point[cur_id].flag_point_delete;

	touch->point[cur_id].flag_point_delete_last=touch->point[cur_id].flag_point_delete;



		
	return *is_delete;	
}


/////////////////////////////////////////////////////////////////////////////
// Function name:
// Purpose:
// Input Parameters:
// Return type:
// Output Parameters: 	NONE
/////////////////////////////////////////////////////////////////////////////
int touch_data_adapter(struct touch_stc *touch_info, struct synaptics_rmi4_data *rmi4_data) 
{
	int i=0;
	
	static bool one_point_adapter_vaild=false;	

	static int list_diff_x[20]={0};
	static int list_diff_y[20]={0};

	static int da_diff_x[20]={0};
	static int da_diff_y[20]={0};	

	int temp_x=0;
	int temp_y=0;
	
	int da_diff_x_tmp=0;
	int da_diff_y_tmp=0;	
	
	int filled_point_num=3;	

	u8 cur_id=0;

	int filled_diff_cumulation_x=0;
	int filled_diff_cumulation_y=0;
	
	struct touch_stc *touch=touch_info;
		
/////////////////////////////////////////////////////////////////////////////////////////
	if(touch==NULL) 
	{
		ht_print_if_c("error: touch_data_adapter cinfo =NULL ");	
		return 0; 
	}	


	if((touch_info->data_adapter_function==0)
	||(touch->id>=10)
	||(touch->point[cur_id].down_up==STANDBY)
	)
	return 0;		
	
/////////////////////////////////////////////////////////////////////////////////////////	
	cur_id=touch->id;

		
	if((touch->point[cur_id].step>=2)&&(touch->point[cur_id].step<16)&&(touch->fingers_to_process==1)&&(cur_id==0))
	{
		list_diff_x[touch->point[cur_id].step]=touch->point[cur_id].diff_x;
		list_diff_y[touch->point[cur_id].step]=touch->point[cur_id].diff_y;	
		one_point_adapter_vaild=true;
	}
	else
	{
		one_point_adapter_vaild=false;
		return 0;
	}
	

		
	if(one_point_adapter_vaild
	&&(touch->point[cur_id].down_up==UP)
	&&(touch->fingers_to_process==1)&&(cur_id==0)
	&&(touch->point[cur_id].step>=3)&&(touch->point[cur_id].step<16)
	&&((abs(touch->point[cur_id].last_diff_x)>=2)||(abs(touch->point[cur_id].last_diff_y)>=2))
	&&((abs(touch->point[cur_id].last_diff_x)+abs(touch->point[cur_id].last_diff_y))<=150)
	)
	{

		
#if 1//def OLD_11_07 //----------------------------------------------------jerry li //Drv. 2015_11_07 //
		for(i=2;i<touch->point[cur_id].step;i++) //jerry li  Drv. 2015_11_07 
		{
			if((abs(touch->point[i].last_diff_x))>=1)
			da_diff_x[i+1]=(abs(list_diff_x[i+1])<<10)/(abs(list_diff_x[i]));
			else
			da_diff_x[i+1]=(1<<10);//(abs(list_diff_x[i+1])<<10);

			if((abs(touch->point[i].last_diff_y))>=1)				
			da_diff_y[i+1]=(abs(list_diff_y[i+1])<<10)/(abs(list_diff_y[i]));
			else
			da_diff_y[i+1]=(1<<10);//(abs(list_diff_y[i+1])<<10);

			if(da_diff_x[i+1]>0) //jerry li  Drv. 2015_11_07 
			{
				da_diff_x_tmp++;
			}

			if(da_diff_y[i+1]>0) //jerry li  Drv. 2015_11_07 
			{
				da_diff_y_tmp++;
			}

			if(da_diff_x[i+1]>(2<<10)) 
			da_diff_x[i+1]=(2<<10);	

			if(da_diff_y[i+1]>(2<<10)) 
			da_diff_y[i+1]=(2<<10);	
			
			
			temp_x+=da_diff_x[i+1];
			temp_y+=da_diff_y[i+1];	

//			ht_print("%02d: ( %d,%d)  [%d,%d]   [%d,%d]   [%d,%d]",i,temp_x,temp_y,da_diff_x[i+1],da_diff_y[i+1],list_diff_x[i+1],list_diff_y[i+1],list_diff_x[i],list_diff_y[i]);				
		}

		temp_x=((temp_x/da_diff_x_tmp));
		temp_y=((temp_y/da_diff_y_tmp));
		
//		ht_print(" %d,%d     %d,%d  ",temp_x,temp_y,da_diff_x_tmp,da_diff_y_tmp);		

		if(temp_x<(1<<9)) 
		temp_x=(1<<9);	
		if(temp_y<(1<<9)) 
		temp_y=(1<<9);	

		if(temp_x>(2<<10)) 
		temp_x=(2<<10);				
		if(temp_y>(2<<10)) 
		temp_y=(2<<10);	
		
#else
		for(i=2;i<=touch_step[cinfo->finger];i++) //jerry li  Drv. 2015_11_07 
		{
			if((abs(list_diff_x[i]))>=1)
			da_diff_x[i]=(abs(list_diff_x[i+1]))/(abs(list_diff_x[i]));
			else
			da_diff_x[i]=(abs(list_diff_x[i+1]));

			if((abs(list_diff_y[i]))>=1)				
			da_diff_y[i]=(abs(list_diff_y[i+1]))/(abs(list_diff_y[i]));
			else
			da_diff_y[i]=(abs(list_diff_y[i+1]));
			
			temp_x+=da_diff_x[i];
			temp_y+=da_diff_y[i];				
		}

		temp_x=((temp_x/touch_step[cinfo->finger]));
		temp_y=((temp_y/touch_step[cinfo->finger]));
			
		if(temp_x<65536) 
		temp_x=65536;	
		if(temp_y<65536) 
		temp_y=65536;	
#endif //

		if((abs(touch->point[i].last_diff_x)<=10)&&(abs(touch->point[i].last_diff_y)<=10)
			&&(abs(touch->point[i].last_diff_x)>=3||(abs(touch->point[i].last_diff_y)>=3)))
		filled_point_num=4;
		else if(touch->point[cur_id].step<=6)
		filled_point_num=3;
		else if(touch->point[cur_id].step<=12)
		filled_point_num=2;
		else
		filled_point_num=1;				

		for(i=0;i<filled_point_num;i++) //jerry li  Drv. 2015_11_06 
		{

#if 0//def TEST 
			if(last_diff_x>0) //jerry li  Drv. 2015_11_07 
				last_diff_x=1;
			else if(last_diff_x<=-1)
				last_diff_x=-1;
			if(last_diff_y>0) //jerry li  Drv. 2015_11_07 
				last_diff_y=1;
			else if(last_diff_y<=-1)
				last_diff_y=-1;
			
#endif
#if 1
			touch->point[cur_id].diff_x=(abs(list_diff_x[touch->point[cur_id].step-1])*temp_x)>>10;
			touch->point[cur_id].diff_y=(abs(list_diff_y[touch->point[cur_id].step-1])*temp_y)>>10;
#else
			if(list_diff_x[touch_step[cinfo->finger]]>=list_diff_x[touch_step[cinfo->finger]-1])
			{
				if(list_diff_x[touch_step[cinfo->finger-1]]>=2)
				diff_x=list_diff_x[touch_step[cinfo->finger]]+list_diff_x[touch_step[cinfo->finger-1]]>>1;
				else
				diff_x=list_diff_x[touch_step[cinfo->finger]]+list_diff_x[touch_step[cinfo->finger-1]];					
			}
			else
			diff_x=list_diff_x[touch_step[cinfo->finger]];	
			
			
			diff_x=(abs(last_diff_x)*temp_x)>>10;
			diff_y=(abs(last_diff_y)*temp_y)>>10;
#endif //

//check 
			if(touch->point[cur_id].diff_x>100) 
			touch->point[cur_id].diff_x=100;	
			if(touch->point[cur_id].diff_y>100) 
			touch->point[cur_id].diff_y=100;	
			
			if(touch->point[cur_id].last_diff_x<0) 
			touch->point[cur_id].diff_x=-touch->point[cur_id].diff_x;	
			if(touch->point[cur_id].last_diff_y<0) 
			touch->point[cur_id].diff_y=-touch->point[cur_id].diff_y;					

#ifdef OLD_11_30 //----------------------------------------------------jerry li //Drv. 2015_11_30 //
			if(abs(touch->point[cur_id].last_diff_x+touch->point[cur_id].diff_x)>120||abs(touch->point[cur_id].last_diff_y+touch->point[cur_id].diff_y)>120)
			break;
#else


			filled_diff_cumulation_x+=abs(touch->point[cur_id].diff_x);
			filled_diff_cumulation_y+=abs(touch->point[cur_id].diff_x);
			if((filled_diff_cumulation_y>120)||(filled_diff_cumulation_x>120))
			break;				
#endif //-----------------------------------------------------------------OLD_11_30_
			list_diff_x[touch->point[cur_id].step]=touch->point[cur_id].diff_x;
			list_diff_y[touch->point[cur_id].step]=touch->point[cur_id].diff_y;	

			touch->point[cur_id].x=touch->point[cur_id].last_x+touch->point[cur_id].diff_x;
			touch->point[cur_id].y=touch->point[cur_id].last_y+touch->point[cur_id].diff_y;
			
			input_report_key(rmi4_data->input_dev,BTN_TOUCH, 1);
			input_report_key(rmi4_data->input_dev,BTN_TOOL_FINGER, 1);
			input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_X, touch->point[cur_id].x);
			input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_Y, touch->point[cur_id].y);
			input_sync(rmi4_data->input_dev);

#if 1//def 			
			ht_print_if_c("%d/%d %03d: p( %d,%d) dif[%d,%d]    @(%d,%d)----- touch_data_adapter",
			cur_id+1,touch->fingers_to_process,
			touch->point[cur_id].step,
			touch->point[cur_id].x,touch->point[cur_id].y,
			touch->point[cur_id].diff_x,touch->point[cur_id].diff_y,
			temp_x,temp_y
			);
#else
//			msleep(5);
#endif
			touch->point[cur_id].last_diff_x=touch->point[cur_id].diff_x;
			touch->point[cur_id].last_diff_y=touch->point[cur_id].diff_y;
			
			touch->point[cur_id].last_x=touch->point[cur_id].x;
			touch->point[cur_id].last_y=touch->point[cur_id].y;

			touch->point[cur_id].step++;
		
		}

//clear all		
//		touch->point[cur_id].x=0;
//		touch->point[cur_id].y=0;

		for(i=0;i<19;i++) //jerry li  Drv. 2015_11_07 
		{
			list_diff_x[i]=0;
			list_diff_y[i]=0;
			da_diff_x[i]=0;
			da_diff_y[i]=0;
		}					
	}





	
	return 1;	
}



/////////////////////////////////////////////////////////////////////////////
// Function name:
// Purpose:
// Input Parameters:
// Return type:
// Output Parameters: 	NONE
/////////////////////////////////////////////////////////////////////////////

int touch_super_monitoring(struct touch_stc *touch_info,struct synaptics_rmi4_data *rmi4_data)
{

       u8 cur_id;
       u8 is_reset=0;	


	struct touch_stc *touch=touch_info;
	
	if(touch==NULL) 
	{
		ht_print_if_c("error: touch_jump_point_detection cinfo =NULL ");	
		return 0; 
	}	

	if((touch_info->super_monitor_function==0)
	||(touch->id>=10)
	||(touch->point[cur_id].down_up==STANDBY)
	)
	return 0;	



/////////////////////////////////////////////////////////////////////////////////////////
/* Removed  //jerry li  Drv. 2015_12_02 

	if(!synaptics_is_usb_insert()) //temp
	{
		touch->warning_cnt=0;
		touch->reset_cnt=0;	
		return 0;		
	}
*/	
	cur_id=touch->id;

	if(touch->fingers_to_process>=6) //jerry li  Drv. 2016_02_22 
	{
//		touch->warning_cnt=0;
		touch->reset_cnt=0;	
//		return 0;
	}

	if(touch->fingers_to_process>=4)  
	{
//		return 0;
	}
	
	if(monitor_check_is_point_in_detection_area(touch->point[cur_id].x,touch->point[cur_id].y))
	{
		return 0;
	}


//error 
	if((touch->point[cur_id].x<0)
	||(touch->point[cur_id].y<0)
	||(touch->point[cur_id].x>1080)
	||(touch->point[cur_id].y>1920)	
	) 
	{
		touch->warning_cnt+=20;
		touch->warning_cnt_arr[0]++;
		ht_print("monitor warning[0] cnt++=%d: id=%d step %d   (%d,%d)",
			touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].x,touch->point[cur_id].y);		
	}
	
//speed	
	if((touch->point[cur_id].down_up!=DOWN)
	&&(((touch->point[cur_id].step==1)&&((touch->point[cur_id].abs_diff_x>250)||(touch->point[cur_id].abs_diff_y>250)))
	||((touch->point[cur_id].step==2)&&((touch->point[cur_id].abs_diff_x>350)||(touch->point[cur_id].abs_diff_y>350)))	
	||((touch->point[cur_id].step==3)&&((touch->point[cur_id].abs_diff_x>620)||(touch->point[cur_id].abs_diff_y>620)))	
	||((touch->point[cur_id].step>=4)&&((touch->point[cur_id].abs_diff_x>650)||(touch->point[cur_id].abs_diff_y>650)))		
	))
	{
		touch->warning_cnt+=2;
		touch->warning_cnt_arr[1]++;	
		ht_print_if_m("monitor warning[1] cnt++=%d: speed  id=%d step %d  dif[%d,%d]",
			touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].diff_x,touch->point[cur_id].diff_y);	
		
	}


//speed sum	
	if(touch->point[cur_id].step==0)
	{
		touch->point[cur_id].sum_diff_x=0;
		touch->point[cur_id].sum_diff_y=0;
	}
	else if((touch->point[cur_id].sum_diff_x<10000)&&(touch->point[cur_id].sum_diff_y<10000)) //(touch->point[cur_id].step<=6)
	{
		touch->point[cur_id].sum_diff_x+=touch->point[cur_id].abs_diff_x;
		touch->point[cur_id].sum_diff_y+=touch->point[cur_id].abs_diff_y;		
	}
	
	if((touch->point[cur_id].step==1)&&((touch->point[cur_id].sum_diff_x<100)&&(touch->point[cur_id].sum_diff_y<100))
	&&(((touch->point[cur_id].step==2)&&((touch->point[cur_id].sum_diff_x>500)||(touch->point[cur_id].sum_diff_y>500)))
	||((touch->point[cur_id].step==3)&&((touch->point[cur_id].sum_diff_x>900)||(touch->point[cur_id].sum_diff_y>900)))	
	||((touch->point[cur_id].step==4)&&((touch->point[cur_id].sum_diff_x>1300)||(touch->point[cur_id].sum_diff_y>1300)))	
	||((touch->point[cur_id].step>=5)&&(touch->point[cur_id].step<=7)&&((touch->point[cur_id].sum_diff_x>1700)||(touch->point[cur_id].sum_diff_y>1700)))	
//	||((touch->point[cur_id].step==6)&&((tmp_x>2000)||(tmp_y>2000)))		
	))
	{
		touch->warning_cnt+=2;
		touch->warning_cnt_arr[2]++;		
		ht_print_if_m("monitor warning[2] cnt++=%d: speed_sum   id=%d step %d  sum_diff[%d,%d]",
			touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].sum_diff_x,touch->point[cur_id].sum_diff_y);	
	}	
	
//aver speed 
	if((touch->point[cur_id].step>=3)
	&&((touch->point[cur_id].aver_speed_x>=400)||(touch->point[cur_id].aver_speed_y>=400))
	)
	{
		touch->warning_cnt+=3;
		touch->warning_cnt_arr[3]++;	
		ht_print_if_m("monitor warning[3] cnt++=%d: aver speed    id=%d step %d  dif[%d,%d]",
			touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].aver_speed_x,touch->point[cur_id].aver_speed_y);	
		
	}

/* Removed  //jerry li  Drv. 2016_01_21 
//
//incrert
	if((touch->point[cur_id].step>=3)
	&&((touch->point[cur_id].aver_speed_x>=400)||(touch->point[cur_id].aver_speed_y>=400))
	)
	{
		warning_cnt+=2;
		ht_print("monitor warning[3] cnt++=%d: aver speed    step %d  dif[%d,%d]",warning_cnt,touch->point[cur_id].step,touch->point[cur_id].aver_speed_x,touch->point[cur_id].aver_speed_y);	
		
	}
*/
	
//a
	if((touch->point[cur_id].step>=3)&&(touch->point[cur_id].abs_last_diff_x>=2))
	{

   		if(((touch->point[cur_id].abs_diff_x>(touch->point[cur_id].abs_last_diff_x<<6))&&(touch->point[cur_id].abs_last_diff_x>=2))
		||((touch->point[cur_id].abs_diff_x>(touch->point[cur_id].abs_last_diff_x<<5))&&(touch->point[cur_id].abs_last_diff_x>=10))
		)
   		{
   			touch->warning_cnt+=2;		
			touch->warning_cnt_arr[4]++;	
   			ht_print_if_m("monitor warning[4] cnt++=%d: a id=%d step%d  %d/%d ",
				touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].abs_diff_x,touch->point[cur_id].abs_last_diff_x);		
   		}
	}
	if((touch->point[cur_id].step>=3)&&(touch->point[cur_id].abs_last_diff_y>=2))
	{
   		if(((touch->point[cur_id].abs_diff_y>(touch->point[cur_id].abs_last_diff_y<<6))&&(touch->point[cur_id].abs_last_diff_y>=2))
		||((touch->point[cur_id].abs_diff_y>(touch->point[cur_id].abs_last_diff_y<<5))&&(touch->point[cur_id].abs_last_diff_y>=10))
		)	
   		{
   			touch->warning_cnt+=2;	
			touch->warning_cnt_arr[4]++;	
   			ht_print_if_m("monitor warning[4] cnt++=%d: a id=%d step%d  %d/%d ",
				touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].abs_diff_y,touch->point[cur_id].abs_last_diff_y);		
   		}
	}	
//ht_print monitor warning[4] cnt++=7: a step128  549/8 
	
#define A_VALUE 3
#define B_VALUE 2
//aver a 
	if((touch->point[cur_id].step>=3)&&(touch->point[cur_id].aver_last_speed_x>=4))
	{
		if(((touch->point[cur_id].abs_diff_x>(touch->point[cur_id].aver_last_speed_x<<A_VALUE))&&(touch->point[cur_id].aver_last_speed_x>=15))
		||((touch->point[cur_id].abs_diff_x>(touch->point[cur_id].aver_last_speed_x<<B_VALUE))&&(touch->point[cur_id].aver_last_speed_x>=40))
		)
   		{
   			touch->warning_cnt+=2;	
			touch->warning_cnt_arr[5]++;	
   			ht_print_if_m("monitor warning[5] cnt++=%d:aver a1  id=%d step%d dx/last_dx %d/%d ",
				touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].abs_diff_x,touch->point[cur_id].aver_last_speed_x);		
   		}
		else if(((touch->point[cur_id].aver_last_speed_x>(touch->point[cur_id].abs_diff_x<<A_VALUE))&&(touch->point[cur_id].abs_diff_x>=15))
		||((touch->point[cur_id].aver_last_speed_x>(touch->point[cur_id].abs_diff_x<<B_VALUE))&&(touch->point[cur_id].abs_diff_x>=40))
		)
   		{
   			touch->warning_cnt+=2;	
			touch->warning_cnt_arr[5]++;	
   			ht_print_if_m("monitor warning[5] cnt++=%d: aver a2  id=%d step%d dx/last_dx %d/%d ",
				touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].abs_diff_x,touch->point[cur_id].aver_last_speed_x);		
   		}
		
	}
	if((touch->point[cur_id].step>=3)&&(touch->point[cur_id].aver_last_speed_y>=4))
	{
		if(((touch->point[cur_id].abs_diff_y>(touch->point[cur_id].aver_last_speed_y<<4))&&(touch->point[cur_id].aver_last_speed_y>=15))
		||((touch->point[cur_id].abs_diff_y>(touch->point[cur_id].aver_last_speed_y<<3))&&(touch->point[cur_id].aver_last_speed_y>=40))
		)
   		{
   			touch->warning_cnt+=2;		
			touch->warning_cnt_arr[5]++;	
   			ht_print_if_m("monitor warning[5] cnt++=%d:aver a1 id=%d step%d  dy/last_dy %d/%d ",
				touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].abs_diff_y,touch->point[cur_id].aver_last_speed_y);		
   		}	
		else if(((touch->point[cur_id].aver_last_speed_y>(touch->point[cur_id].abs_diff_y<<4))&&(touch->point[cur_id].abs_diff_y>=15))
		||((touch->point[cur_id].aver_last_speed_y>(touch->point[cur_id].abs_diff_y<<3))&&(touch->point[cur_id].abs_diff_y>=40))
		)
   		{
   			touch->warning_cnt+=2;	
			touch->warning_cnt_arr[5]++;	
   			ht_print_if_m("monitor warning[5] cnt++=%d:aver a2 id=%d step%d  dy/last_dy %d/%d ",
				touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].abs_diff_y,touch->point[cur_id].aver_last_speed_y);		
   		}
		
	}
//aver a 1point	
/* Removed  //jerry li  Drv. 2016_02_27 
//
	if((touch->point[cur_id].step>=5)&&(1==touch->fingers_to_process)&&(touch->point[cur_id].down_up!=UP)) //jerry li  Drv. 2016_02_27 
	{
		if((touch->point[cur_id].aver_last_speed_x>=1)&&(touch->point[cur_id].aver_last_speed_x<=50)&&(touch->point[cur_id].abs_diff_x>=5))
		{
			if(touch->point[cur_id].abs_diff_x>(touch->point[cur_id].aver_last_speed_x<<A_VALUE))
	   		{
	   			touch->warning_cnt+=2;	
				touch->warning_cnt_arr[5]++;	
	   			ht_print_if_m("monitor warning[5] cnt++=%d:aver a11  id=%d step%d dx/last_dx %d/%d ",
					touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].abs_diff_x,touch->point[cur_id].aver_last_speed_x);		
	   		}
			else if(touch->point[cur_id].aver_last_speed_x>(touch->point[cur_id].abs_diff_x<<A_VALUE))
	   		{
	   			touch->warning_cnt+=2;	
				touch->warning_cnt_arr[5]++;	
	   			ht_print_if_m("monitor warning[5] cnt++=%d: aver a22  id=%d step%d dx/last_dx %d/%d ",
					touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].abs_diff_x,touch->point[cur_id].aver_last_speed_x);		
	   		}
			
		}
		if((touch->point[cur_id].aver_last_speed_y>=1)&&(touch->point[cur_id].aver_last_speed_y<=50)&&(touch->point[cur_id].abs_diff_y>=5))
		{
			if(touch->point[cur_id].abs_diff_y>(touch->point[cur_id].aver_last_speed_y<<A_VALUE))
	   		{
	   			touch->warning_cnt+=2;		
				touch->warning_cnt_arr[5]++;	
	   			ht_print_if_m("monitor warning[5] cnt++=%d:aver a11 id=%d step%d  dy/last_dy %d/%d ",
					touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].abs_diff_y,touch->point[cur_id].aver_last_speed_y);		
	   		}	
			else if(touch->point[cur_id].aver_last_speed_y>(touch->point[cur_id].abs_diff_y<<A_VALUE))
	   		{
	   			touch->warning_cnt+=2;	
				touch->warning_cnt_arr[5]++;	
	   			ht_print_if_m("monitor warning[5] cnt++=%d:aver a22 id=%d step%d  dy/last_dy %d/%d ",
					touch->warning_cnt,cur_id,touch->point[cur_id].step,touch->point[cur_id].abs_diff_y,touch->point[cur_id].aver_last_speed_y);		
	   		}
			
		}	
	}
*/
//direction 
	if((touch->point[cur_id].step>=3)&&(touch->point[cur_id].abs_diff_x>=15)&&(touch->point[cur_id].abs_last_diff_x>=15)&&(touch->point[cur_id].abs_last_diff_y<=64)) 
	{
		if(((touch->point[cur_id].diff_x>0)&&(touch->point[cur_id].last_diff_x<0))||((touch->point[cur_id].diff_x<0)&&(touch->point[cur_id].last_diff_x>0)))
		{
			touch->warning_cnt+=2;	
			touch->warning_cnt_arr[6]++;	
			ht_print_if_m("monitor warning[6] cnt++=%d: direction  id=%d x +- %d %d",touch->warning_cnt,cur_id,touch->point[cur_id].diff_x,touch->point[cur_id].last_diff_x);	
		}
	}
	if((touch->point[cur_id].step>=3)&&(touch->point[cur_id].abs_diff_y>=15)&&(touch->point[cur_id].abs_last_diff_y>=15)&&(touch->point[cur_id].abs_last_diff_x<=64)) 
	{
		if(((touch->point[cur_id].diff_y>0)&&(touch->point[cur_id].last_diff_y<0))||((touch->point[cur_id].diff_y<0)&&(touch->point[cur_id].last_diff_y>0)))
		{
			touch->warning_cnt+=2;	
			touch->warning_cnt_arr[6]++;	
			ht_print_if_m("monitor warning[6] cnt++=%d: direction  id=%d y +- %d %d",touch->warning_cnt,cur_id,touch->point[cur_id].diff_y,touch->point[cur_id].last_diff_y);		
		}
	}	
	
//only 2 point
	if((touch->point[cur_id].down_up==UP)&&(touch->point[cur_id].step<=1))
	{
//		touch->warning_cnt+=1;	
		touch->warning_cnt_arr[7]++;	
//		ht_print_if_m("monitor warning[7] cnt++=%d: only 2 point  id=%d setp=%d",touch->warning_cnt,cur_id,touch->point[cur_id].step);		
	}

//jump
	if((touch->point[cur_id].time_diff<=70)&&(touch->point[cur_id].down_up==DOWN)&&(1==touch->fingers_to_process))
	{
		if((touch->point[cur_id].time_diff<=70)&&(touch->point[cur_id].abs_diff_x<130)&&(touch->point[cur_id].abs_diff_y<130)) //130
		{
			if(1==touch->fingers_to_process)
			touch->warning_cnt+=2;// 3
			else
			touch->warning_cnt+=1;	
			
			touch->warning_cnt_arr[8]++;	
			ht_print_if_m("monitor warning[8] cnt++=%d: id=%d jump time_diff=%d   d=(%d %d)",
				touch->warning_cnt,cur_id,touch->point[cur_id].time_diff,touch->point[cur_id].abs_diff_x,touch->point[cur_id].abs_diff_y);			
			
		}
		else
		if(touch->point[cur_id].time_diff<=40)
		{
//			warning_cnt+=1;		
//			ht_print("monitor warning[8] cnt++=%d: jump time_diff=%d ",warning_cnt,touch->point[cur_id].time_diff);			
		}
		else
		{
//		ht_print("monitor  %d: touch->point[cur_id].time_diff    %d   dist =%d",warning_cnt,touch->point[cur_id].time_diff,(touch->point[cur_id].abs_diff_x+touch->point[cur_id].abs_diff_y));			
		}
	}	

//dif unstable
#define D_VALUE 3
#define C_VALUE 2
	if((touch->point[cur_id].step>=3)&&(touch->point[cur_id].down_up!=UP))
	{

		if((touch->point[cur_id].abs_last_diff_x>(touch->point[cur_id].abs_last_last_diff_x<<2))
		&&(touch->point[cur_id].abs_last_diff_x>(touch->point[cur_id].abs_diff_x<<2))
			&&(touch->point[cur_id].abs_diff_x>=10)
			&&(touch->point[cur_id].abs_last_diff_x>=10)
			&&(touch->point[cur_id].abs_last_last_diff_x>=10)) 
		{
			if(1==touch->fingers_to_process)
			touch->warning_cnt+=7;	
			else
			touch->warning_cnt+=1;
			
			touch->warning_cnt_arr[9]++;	
			ht_print_if_m("monitor warning[9][x] cnt++=%d: dif unstable  id=%d setp=%d [%d %d %d]",touch->warning_cnt,cur_id,touch->point[cur_id].step,
			touch->point[cur_id].abs_diff_x,touch->point[cur_id].abs_last_diff_x,touch->point[cur_id].abs_last_last_diff_x);
		}
		else if((touch->point[cur_id].abs_last_diff_x>(touch->point[cur_id].abs_last_last_diff_x<<2))
			&&(touch->point[cur_id].abs_last_diff_x>(touch->point[cur_id].abs_diff_x<<2))
			&&(touch->point[cur_id].abs_diff_x>=2)
			&&(touch->point[cur_id].abs_last_diff_x>=2)
			&&(touch->point[cur_id].abs_last_last_diff_x>=2))
		{
			if(1==touch->fingers_to_process)
			touch->warning_cnt+=7;	
			else
			touch->warning_cnt+=1;
			
			touch->warning_cnt_arr[9]++;	
			ht_print_if_m("monitor warning[9][x2] cnt++=%d: dif unstable  id=%d setp=%d [%d %d %d]",touch->warning_cnt,cur_id,touch->point[cur_id].step,
			touch->point[cur_id].abs_diff_x,touch->point[cur_id].abs_last_diff_x,touch->point[cur_id].abs_last_last_diff_x);
		}
		
		if((touch->point[cur_id].abs_last_diff_y>(touch->point[cur_id].abs_last_last_diff_y<<2))
		&&(touch->point[cur_id].abs_last_diff_y>(touch->point[cur_id].abs_diff_y<<2))
			&&(touch->point[cur_id].abs_diff_y>=10)
			&&(touch->point[cur_id].abs_last_diff_y>=10)
			&&(touch->point[cur_id].abs_last_last_diff_y>=10))
		{
			if(1==touch->fingers_to_process)
			touch->warning_cnt+=7;	
			else
			touch->warning_cnt+=1;
				
			touch->warning_cnt_arr[9]++;	
			ht_print_if_m("monitor warning[9][y] cnt++=%d: dif unstable  id=%d setp=%d [%d %d %d]",touch->warning_cnt,cur_id,touch->point[cur_id].step,
			touch->point[cur_id].abs_diff_y,touch->point[cur_id].abs_last_diff_y,touch->point[cur_id].abs_last_last_diff_y);
		}	
		else if((touch->point[cur_id].abs_last_diff_y>(touch->point[cur_id].abs_last_last_diff_y<<2))
			&&(touch->point[cur_id].abs_last_diff_y>(touch->point[cur_id].abs_diff_y<<2))
			&&(touch->point[cur_id].abs_diff_y>=2)
			&&(touch->point[cur_id].abs_last_diff_y>=2)
			&&(touch->point[cur_id].abs_last_last_diff_y>=2))
		{
			if(1==touch->fingers_to_process)
			touch->warning_cnt+=7;	
			else
			touch->warning_cnt+=1;
				
			touch->warning_cnt_arr[9]++;	
			ht_print_if_m("monitor warning[9][y2] cnt++=%d: dif unstable id=%d setp=%d [%d %d %d]",touch->warning_cnt,cur_id,touch->point[cur_id].step,
			touch->point[cur_id].abs_diff_y,touch->point[cur_id].abs_last_diff_y,touch->point[cur_id].abs_last_last_diff_y);
		}			
	}

/////////////////////////////////////////////////////////////////


	
/////////////////////////////////////////////////////////////////
//check&reset
	if((touch->warning_cnt)&&(!touch->flag_10sec_timer_working))
	{
		touch->flag_10sec_timer_working=true;
		cancel_delayed_work_sync(&touch_timer_monitor_10s_work);
		queue_delayed_work(touch_timer_monitor_10s_work_queue,&touch_timer_monitor_10s_work, 10*100);	
	}

	if(((touch->warning_cnt>=18)&&(touch->flag_reset_3s_available))
	||(touch->warning_cnt>=150)
	)
	{
		if(touch->warning_cnt>=150)
		touch->warning_cnt=100;
		
		touch->flag_reset_3s_available=false;
		schedule_work(&touch->touch_monitor_reset_work); 
		cancel_delayed_work_sync(&touch_timer_monitor_10s_work);
		cancel_delayed_work_sync(&touch_timer_monitor_3s_work);
		queue_delayed_work(touch_timer_monitor_10s_work_queue,&touch_timer_monitor_10s_work, 10*100);
		queue_delayed_work(touch_timer_monitor_3s_work_queue,&touch_timer_monitor_3s_work, 3*100);	
	}

	
	return is_reset;	
}


int touch_super_power(struct touch_stc *touch_info,struct synaptics_rmi4_data *rmi4_data) 
{
//	int i=0;
       u8 cur_id;
//	bool one_point_adapter_vaild=0;
//    u8 is_reset=0;	
	struct touch_stc *touch=touch_info;


/////////////////////////////////////////////////////////////////////////////////////////	
	if(touch==NULL) 
	{
		ht_print_if_c("error: touch_jump_point_detection cinfo =NULL ");	
		return 0; 
	}	
	
	cur_id=touch->id;

	if(cur_id>=10) 
	return 0; 
	
	if(touch->point[cur_id].down_up==STANDBY)
	return 0;		
/* Removed  //jerry li  Drv. 2015_12_02 
//
	if((touch->fingers_to_process==1)&&(cur_id==0))
	{
		one_point_adapter_vaild=true;
	}
	else
	{
		one_point_adapter_vaild=false;
		return 0;
	}
*/

	
/////////////////////////////////////////////////////////////////////////////////////////
//super contector


	if((touch->point[cur_id].time_diff<=70)&&(touch->point[cur_id].down_up==DOWN))
	{
		if((touch->point[cur_id].time_diff<=70)&&(touch->point[cur_id].abs_diff_x<150)&&(touch->point[cur_id].abs_diff_y<150)) //130
		{
		}
	}
	
	
	return 1;	
}



/////////////////////////////////////////////////////////////////////////////
//start 
/////////////////////////////////////////////////////////////////////////////
#if 1//def BYT_TOUCH_DATA_IMPROVING_SOLUTION //jacky li //Drv. 2014_11_28 //





typedef enum
{
	STANDBY_START =0 ,
	_1ST_POINT =0,	//
	_2ND_POINT, 
	_3TH_MOVE,	
	__MOVE =999
}TOUCH_STEP;





typedef struct {
    int raw;
    int cal;
    int raw_enlarge;	
    int cal_enlarge;	

    int raw_last;
    int cal_last;	

    int raw_enlarge_last;	
    int cal_enlarge_last;		
	
}STC_POINT_CAL;//srt_point_cal;

typedef struct {
    STC_POINT_CAL x;	
    STC_POINT_CAL y;
}STC_POINT_POS;//srt_point_pos


typedef struct {
    int xx;	
    int yy;
}STC_POS_XY;//srt_point_pos

//static int  temp_value[][]={{256,8},{128,7},{64,6},{32,5}};

static int  CTP_D1=64;  // 256:8  //128:7  64:6
static int  CTP_D2=0;  // 8  

static int  averge_index=6;  // 8//

//static int Filter_R_Index[][2]={{32,5},{16,4},{64,6}};


#define MAPPING_INDEX 15    // 8=256 //10 1024  //16 65536

#define SUPPORT_MUCH_POINT




//test		
#if 0//def NEW_03_30 //----------------------------------------------------jacky li //Drv. 2015_03_30 //

static bool check_is_touch_key_event(struct touch_info *cinfo, u8 *point_num) 
{

	static bool Flag_Close_Data_Improving=0;

	if(cinfo->p[0]==0)
	{

#ifdef OLD_05_11 //----------------------------------------------------jacky li //Drv. 2015_05_11 //
		if((cinfo->x[0]>=520)&&(cinfo->y[0]<=10))
		{
				input_report_key(tpd->dev, KEY_BACK, 1);
				input_report_key(tpd->dev, KEY_BACK, 0);	
				input_mt_sync(tpd->dev);
				input_sync(tpd->dev);
				ht_print("KEY_BACK %d\n\n",Flag_Close_Data_Improving);
				return false; 
		}
		else 
#endif //-----------------------------------------------------------------#ifdef OLD_05_11_ //Drv. //
		if((cinfo->x[0]<=50)&&(cinfo->y[0]<=50))
		{
			Flag_Close_Data_Improving=!Flag_Close_Data_Improving;

			switch (Flag_Close_Data_Improving)
			{
			case 0:
				input_report_key(tpd->dev, KEY_VOLUMEUP, 1);
				input_report_key(tpd->dev, KEY_VOLUMEUP, 0);	
				input_mt_sync(tpd->dev);
				input_sync(tpd->dev);
				ht_print("KEY_VOLUMEDOWN %d\n\n",Flag_Close_Data_Improving);
				break;
			case 1:
				input_report_key(tpd->dev, KEY_VOLUMEDOWN, 1);
				input_report_key(tpd->dev, KEY_VOLUMEDOWN, 0);	
				input_mt_sync(tpd->dev);
				input_sync(tpd->dev);
				ht_print("KEY_VOLUMEUP %d\n\n",Flag_Close_Data_Improving);
				break;
			}
		}



	}

	return Flag_Close_Data_Improving; 
	
}
#else
static bool check_is_touch_key_event(struct touch_info *cinfo, u8 *point_num) 
{
	u8 tmp_n;
	if(cinfo->p[0]==0)
	{
	 tmp_n=*point_num;
	}
	return 0;
}
#endif //-----------------------------------------------------------------#ifdef NEW_03_30_ //Drv. //


/////////////////////////////////////////////////////////////////////////////
// Function name:
// Purpose:
// Input Parameters:
// Return type:
// Output Parameters: 	NONE
/////////////////////////////////////////////////////////////////////////////

int touch_super_filter(struct touch_info *cinfo, u8 *point_num)
{


	static STC_POINT_POS pos;
	STC_POS_XY tmp_cal_enlarge_last={0,0};
	STC_POS_XY distance_enlarge={0,0};	
	STC_POS_XY aver_enlarge={0,0};
	STC_POS_XY distance={0,0};	
	STC_POS_XY abs_distance={0,0};	
	STC_POS_XY tmp={0,0};
		
//	int temp_step=1;	
	int distance_z=0;
	int aver_distance_z=0;	
	static int aver_distance_z_last=0;

	int dif_z=0;
	int aver_dif_z=0;	

//	static u32 touch_step=STANDBY_START;
	static u32 touch_count=0;

	STC_POS_XY distance_enlarge_ji;
			
#ifdef SUPPORT_MUCH_POINT //----------------------------------------------------jacky li //Drv. 2015_03_30 //
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

	u16 i;	
	u16 j;	
//	u8 k=0;
	#define MAX_BUF 10

//	static bool start_point_in_detectiong_area[MAX_BUF]={0};
//	static int start_point_pos_y[MAX_BUF]={0};
//	static bool point_moving[MAX_BUF]={0};
	static u32 touch_count_id[MAX_BUF]={0};
	
	int touch_step_p[MAX_BUF]={0};
//	bool is_point_in_detection_area[MAX_BUF]={0};

//	bool flag_point_delete[MAX_BUF]={0};
	
	u8 point_num_cur=0;	
//	u8 point_num_cal=0;

	static u8 point_num_last=0;
//	static u8 point_num_cal_last=0;

	
//	bool remove_touch_event=0;
	

	u16 id=0;		
	u16 index=0;
//	u16 index_num=0;
#endif



	if(*point_num>10)
	{
// 		more point not support
		ht_print_if_c("point :  touch_data_improving_pro  do nothing num=%d ",*point_num);	
		return true; // max deal with two point
	}	

	if(cinfo==NULL) 
	{
		ht_print_if_c("point :  touch_data_improving_pro  cinfo =NULL ");	
		return true; 
	}	


#if 1//def NEW_03_30 //----------------------------------------------------jacky li //Drv. 2015_03_30 //
	if(check_is_touch_key_event(cinfo, point_num)) 
	{
		ht_print_if_c("point :  Flag_Close_Data_Improving    %d (%d,%d)",*point_num,cinfo->x[0],cinfo->y[0]);	
		return true; 
	}
#endif //-----------------------------------------------------------------#ifdef NEW_03_30_ //Drv. //



	
#ifdef SUPPORT_MUCH_POINT //----------------------------------------------------jacky li //Drv. 2015_03_30 //
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


	point_num_cur=*point_num;

//	point_num_cal=point_num_cur;
	
	for(i=point_num_cur;i>0;i--) //jacky li //Drv. 2015_02_09 //
	{
		index=i-1;
		id=cinfo->id[index];

// without p info solution
//		if((-1)==cinfo->p[index]) //jacky li //Drv. 2015_03_03 //
		if((0!=cinfo->p[index])&&(2!=cinfo->p[index])) //jacky li //Drv. 2015_03_03 //		
		{
			if(point_num_cur>point_num_last)
			{
				
				for(j=0;j<point_num_last;j++) //jacky li //Drv. 2015_03_06 //
				{
					cinfo->p[j]=2;
					ht_print_if_c("point :  new touch  cinfo->p[%d]=%d ",j,cinfo->p[j]);	
				}
	
				for(j=point_num_last;j<point_num_cur;j++) //jacky li //Drv. 2015_03_06 //
				{
					cinfo->p[j]=0;
					ht_print_if_c("point :  new touch  cinfo->p[%d]=%d ",j,cinfo->p[j]);						
				}
			
			}
			else
				cinfo->p[index]=2;
			
		}

		touch_step_p[id]=cinfo->p[index];

		if(touch_step_p[id]==0)
		{
//			start_point_in_detectiong_area[id]=0;
//			start_point_pos_y[id]=cinfo->y[index];
//			point_moving[id]=0;	

			touch_count_id[id]=0;
			

		}
		else //if(touch_step_p[id]==2)
		{
			touch_count_id[id]++;
		}

	}	
	
//only support the point of 0	
//	touch_step=touch_step_p[0];
	touch_count=touch_count_id[0];	
#endif //-----------------------------------------------------------------#ifdef NEW_03_30_ //Drv. //
	
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


//init
	pos.x.raw=cinfo->x[0];
	pos.y.raw=cinfo->y[0];	
	pos.x.raw_enlarge=pos.x.raw<<MAPPING_INDEX;
	pos.y.raw_enlarge=pos.y.raw<<MAPPING_INDEX;
	
	pos.x.cal=pos.x.raw;
	pos.y.cal=pos.y.raw;
	pos.x.cal_enlarge=pos.x.cal<<MAPPING_INDEX;
	pos.y.cal_enlarge=pos.y.cal<<MAPPING_INDEX;	

	
	if(touch_count==_1ST_POINT) 
	{

		ht_print_if_c("point %03d:  (%d,%d)dowm  tpd_touchinfo  %d %d %d %d",touch_count, pos.x.raw, pos.y.raw,touch_count,point_num_cur,cinfo->id[1],cinfo->p[1]); //jacky li //Drv. 2014_11_24 //
	}
	else if(touch_count>=_2ND_POINT)
	{

			
// cal the distance / speed
//		distance.xx=pos.x.raw -pos.x.raw_last;
//		distance.yy=pos.y.raw -pos.y.raw_last;
		distance.xx=pos.x.raw -pos.x.cal_last;
		distance.yy=pos.y.raw -pos.y.cal_last;	


		abs_distance.xx=abs(distance.xx);
		abs_distance.yy=abs(distance.yy);
		
		if(abs_distance.xx>=abs_distance.yy)
		{
			distance_z=abs_distance.xx+(abs_distance.yy>>1);
		}
		else
		{
			distance_z=abs_distance.yy+(abs_distance.xx>>1);
		}

		aver_distance_z=distance_z;

#if 1
		if((touch_count>=_3TH_MOVE)&&(distance_z<CTP_D1)) //jacky li //Drv. 2015_02_03 //
		{
			dif_z=distance_z-aver_distance_z_last;
			
			if(abs(dif_z)>=4) //jacky li //Drv. 2015_02_03 //
			{
				if(dif_z>=0) //jacky li //Drv. 2015_02_03 //
				{
					aver_dif_z=dif_z>>2;
				}
				else
				{
					dif_z=(abs(dif_z)>>2);
					aver_dif_z=-dif_z;
				}
				aver_distance_z=aver_distance_z_last+aver_dif_z;				
			}
			else
			{
				aver_distance_z=(aver_distance_z_last+distance_z)>>1;
			}
		}
#else
			aver_distance_z=(aver_distance_z_last+distance_z)>>1;
#endif

#if 0//def NEW_01_30 //----------------------------------------------------jacky li //Drv. 2015_01_30 //
		if(cinfo->y[0]>1000)//600) //\CF²\BF\C7\F8\D3\F2
		{
		//do nothing

		}else//\C9ϲ\BF\C7\F8\D3\F2
#endif //-----------------------------------------------------------------#ifdef NEW_01_30_ //Drv. //
		if(aver_distance_z>=CTP_D1 )
		{
		//do nothing

		}
		else if(aver_distance_z>=CTP_D2 )
		{	

			
			distance_enlarge.xx=distance.xx<<MAPPING_INDEX;
			distance_enlarge.yy=distance.yy<<MAPPING_INDEX;
			
			aver_enlarge.xx=((distance_enlarge.xx*aver_distance_z)>>averge_index);
			aver_enlarge.yy=((distance_enlarge.yy*aver_distance_z)>>averge_index);

			
#if 1//def NEW_02_03 //----------------------------------------------------jacky li //Drv. 2015_02_03 //
			pos.x.cal_enlarge=pos.x.cal_enlarge_last+aver_enlarge.xx;
			pos.y.cal_enlarge=pos.y.cal_enlarge_last+aver_enlarge.yy;
#else
			tmp.xx=pos.x.cal_enlarge_last+aver_enlarge.xx;
			tmp.yy=pos.y.cal_enlarge_last+aver_enlarge.yy;		
#endif //-----------------------------------------------------------------#ifdef NEW_02_03_ //Drv. //

	


			tmp_cal_enlarge_last.xx=pos.x.cal_last<<MAPPING_INDEX;
			tmp_cal_enlarge_last.yy=pos.y.cal_last<<MAPPING_INDEX;
			
			distance_enlarge_ji.xx= pos.x.cal_enlarge_last-tmp_cal_enlarge_last.xx;
			distance_enlarge_ji.yy= pos.y.cal_enlarge_last-tmp_cal_enlarge_last.yy;
			
			if(aver_enlarge.xx>0)
			aver_enlarge.xx+=distance_enlarge_ji.xx;
			else
			aver_enlarge.xx-=distance_enlarge_ji.xx;	

			if(aver_enlarge.yy>0)
			aver_enlarge.yy+=distance_enlarge_ji.yy;
			else
			aver_enlarge.yy-=distance_enlarge_ji.yy;	


#if 1//jacky li //Drv. 2015_02_03 //
			tmp.xx=tmp_cal_enlarge_last.xx+aver_enlarge.xx;
			tmp.yy=tmp_cal_enlarge_last.yy+aver_enlarge.yy;
#else
			pos.x.cal_enlarge=tmp_cal_enlarge_last.xx+aver_enlarge.xx;
			pos.y.cal_enlarge=tmp_cal_enlarge_last.yy+aver_enlarge.yy;			
#endif
		
			
		}			
		else// if(distance_z<CTP_D2)
		{
			ht_print("ERROR");//
		}

		
		pos.x.cal=pos.x.cal_enlarge>>MAPPING_INDEX;
		pos.y.cal=pos.y.cal_enlarge>>MAPPING_INDEX;

#if 1//jacky li //Drv. 2015_02_03 //
		if((0==(pos.x.cal-pos.x.cal_last))&&(0!=abs_distance.xx))
		{
			if(distance.xx>0) //jacky li //Drv. 2015_02_03 //
			{
				pos.x.cal+=1;
			}
			else
			{
				pos.x.cal-=1;
			}
			pos.x.cal_enlarge=pos.x.cal<<MAPPING_INDEX;
			
		}
			
		if((0==(pos.y.cal-pos.y.cal_last))&&(0!=abs_distance.yy))
		{
			if(distance.yy>0) //jacky li //Drv. 2015_02_03 //
			{
				pos.y.cal+=1;
			}
			else
			{
				pos.y.cal-=1;
			}
			pos.y.cal_enlarge=pos.y.cal<<MAPPING_INDEX;
			
		}
		
#endif


		cinfo->x[0]=pos.x.cal;
		cinfo->y[0]=pos.y.cal;	

#if 0//def TEST
//ԭ\B5\E3\B9켣\B7ŵ\BD\B5\DA2\B8\F6\B5\E3
		if(point_num_cur==1)
		{
			point_num_cur=2;
			cinfo->x[0]=pos.x.raw;
			if(pos.y.raw>=200) 
			cinfo->y[0]=pos.y.raw-200;
			else// if(pos.y.raw>=50)
			cinfo->y[0]=0;	
//			else 
//			cinfo->y[0]=0;

			cinfo->x[1]=pos.x.cal;
			cinfo->y[1]=pos.y.cal;		
		}
	
#endif


		

//		distance.xx=distance_enlarge.xx>>MAPPING_INDEX;
//		distance.yy=distance_enlarge.yy>>MAPPING_INDEX;		
		
		ht_print_if_c("point %03d: *(%d,%d)(%2d,%2d)/raw(%d,%d)(%2d,%2d)/(%2d,%2d)(%2d,%2d)(%2d,%2d)r-c(%2d,%2d)z=%d %d %d %d %d %d %s %s",//
//		ht_print("point %03d: *(%3d,%3d) dif(%2d,%2d) t(%2d,%2d)/raw(%3d,%3d) rdif(%2d,%2d) c-raw(%2d,%2d) z=%d %s %s",			
		touch_count, 
		cinfo->x[0], cinfo->y[0],
		cinfo->x[0]-pos.x.cal_last,cinfo->y[0]-pos.y.cal_last,//abs(cinfo->x[0]-pos.x.cal_last),abs(cinfo->y[0]-pos.y.cal_last),
//          / raw		
		pos.x.raw,pos.y.raw,
		pos.x.raw-pos.x.raw_last,pos.y.raw-pos.y.raw_last,//abs(pos.x.raw-pos.x.raw_last),abs(pos.y.raw-pos.y.raw_last),
//          /
		distance.xx,distance.yy, //=	abs(distance_enlarge.xx>>MAPPING_INDEX),abs(distance_enlarge.yy>>MAPPING_INDEX),
		(tmp.xx>>MAPPING_INDEX),(tmp.yy>>MAPPING_INDEX),
		(aver_enlarge.xx>>MAPPING_INDEX),(aver_enlarge.yy>>MAPPING_INDEX),//abs(aver_enlarge.xx>>MAPPING_INDEX),abs(aver_enlarge.yy>>MAPPING_INDEX),
//          // c-r		
//		abs(cinfo->x[0]-pos.x.raw),abs(cinfo->y[0]-pos.y.raw),
		(pos.x.raw-cinfo->x[0]),(pos.y.raw-cinfo->y[0]),
		
		aver_distance_z,distance_z,aver_dif_z,//aver_distance_z_last,
		point_num_cur,cinfo->id[0],cinfo->p[0],
		(aver_distance_z>=CTP_D1? ">32":"_"),
		(pos.y.raw>600? "y>600":"_")

		); //jacky li //Drv. 2014_11_24 //

		if(point_num_cur>1)
    		ht_print_if_c("point %03d: 2(%d,%d)    point_num_cur=%d %d %d",touch_count,cinfo->x[1], cinfo->y[1],point_num_cur,cinfo->id[1],cinfo->p[1]); 
		

			

	}

//de init	
	pos.x.raw_last=pos.x.raw;
	pos.y.raw_last=pos.y.raw;
	
	pos.x.cal_last=pos.x.cal;
	pos.y.cal_last=pos.y.cal;

	pos.x.raw_enlarge_last=pos.x.raw_enlarge;
	pos.y.raw_enlarge_last=pos.y.raw_enlarge;

	pos.x.cal_enlarge_last=pos.x.cal_enlarge;
	pos.y.cal_enlarge_last=pos.y.cal_enlarge;

	aver_distance_z_last=aver_distance_z;

#ifdef SUPPORT_MUCH_POINT 
	if(point_num_cur==0) //jacky li //Drv. 2015_02_12 //
	{
//		point_num_cal_last=0;
		point_num_last=0;
		for(i=0;i<MAX_BUF;i++) //jacky li //Drv. 2015_02_12 //
		{
//			start_point_in_detectiong_area[i]=0;
//			start_point_pos_y[i]=0;
//			point_moving[i]=0;
		}
		
	}

	point_num_last=point_num_cur;
//	point_num_cal_last=point_num_cur;		
#endif

	return 0;
}

#endif //#ifdef NEW_11_28_ //Drv. //	
/////////////////////////////////////////////////////////////////////////////
//end
/////////////////////////////////////////////////////////////////////////////

//#ifdef TOUCH_COMMON_FEATURE//jerry li  Drv. 2016_01_06 


struct delayed_work ctp_power_onoff_work;
struct workqueue_struct *ctp_power_onoff_workqueue;
void ctp_power_onoff_func(struct work_struct *work);

ssize_t ht_touch_slim_log_show(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t ht_touch_slim_log_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

ssize_t ht_debug_sys_show(struct device *dev,
		struct device_attribute *attr, char *buf);
ssize_t ht_debug_sys_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

void touch_common_slim_log_func(struct work_struct *work);

static struct device_attribute attrs[] = {

	__ATTR(touch_slim_log, (S_IRUGO | S_IWUGO),
			ht_touch_slim_log_show,
			ht_touch_slim_log_store),

	__ATTR(ff, (S_IRUGO | S_IWUGO),
			ht_debug_sys_show,
			ht_debug_sys_store),

};

bool touch_common_slim_log(void)
{
	struct touch_stc *touch=g_touch_info;

	if((touch==NULL) ||(!touch_common_is_inited()))
	{
		return 0; 
	}	

//	ht_print("touch_common_slim_log   %d %c %c %ld %ld",touch->log_buf_valid,touch->log_buf1[0],touch->log_buf2[0],strlen(touch->log_buf1),strlen(touch->log_buf2)); //jerry li //Drv. 2016_01_22 //
	
	if(touch->log_buf_valid)
	{
		if(touch->log_buf1[0])
		ht_print("%s",touch->log_buf1); //jerry li //Drv. 2016_01_13 //
		if(touch->log_buf2[0])
		ht_print("%s",touch->log_buf2); //jerry li //Drv. 2016_01_13 //

//		memset(touch->log_buf1, 0, 128);
//		memset(touch->log_buf2, 0, 128);	
		touch->log_buf1[0]=0;
		touch->log_buf2[0]=0;		
		touch->log_buf_valid=false;
		return 0;
	}
	else
	{
//		printk(".%d %c %c %ld %ld\n",touch->log_buf_valid,touch->log_buf1[0],touch->log_buf2[0],strlen(touch->log_buf1),strlen(touch->log_buf2)); //jerry li //Drv. 2016_01_22 //
	}

	return 1;
}

void touch_common_slim_log_func(struct work_struct *work)
{

	if(touch_slim_log)
	touch_common_slim_log();
	
}

bool touch_common_suspend(void)
{
	touch_common_slim_log();
	
	g_touch_info->warning_cnt=0;
	g_touch_info->reset_cnt=0;
	g_touch_info->flag_10sec_timer_working=false;
	g_touch_info->flag_reset_3s_available=true;
	
	return 1;
}

void ctp_power_onoff_func(struct work_struct *work)
{

	struct input_dev *input_dev=g_touch_info->input_dev;



	if(is_tp_fw_updating())
       return;

	if(tp_power_on_off_test==0) //stop
	return;

	if(is_tp_suspend() ==false)
	{
//\C1\C1\C6\C1״̬
		tp_resume_cound++;
		
		if(!tp_resume_point_valid) //jerry li  Drv. 2016_01_06 
		tp_resume_fail_cound++;


			
		ht_print("POWER  OFF   tp_resume_cound=[%d/%d]  %s",tp_resume_fail_cound,tp_resume_cound,input_dev->name);

		tp_resume_point_valid=0;
		input_report_key(input_dev, KEY_POWER, 1);
		input_sync(input_dev);
		input_report_key(input_dev, KEY_POWER, 0);
		input_sync(input_dev);	
		
		queue_delayed_work(ctp_power_onoff_workqueue,&ctp_power_onoff_work, 150);	//200				
	
	}
	else
	{
//   \D0\DD\C3\DF״̬
		printk("\n\n"); //jerry li //Drv. 2015_12_31 //		
		ht_print("POWER  ON   ");
		
		tp_resume_point_valid=0;
		input_report_key(input_dev, KEY_POWER, 1);
		input_sync(input_dev);
		input_report_key(input_dev, KEY_POWER, 0);
		input_sync(input_dev);			

		queue_delayed_work(ctp_power_onoff_workqueue,&ctp_power_onoff_work, 150);				

	}
	
}

//void ctp_power_onoff_key_func(struct work_struct *work)
void ctp_power_onoff_key_func(void)
{

	struct input_dev *input_dev=g_touch_info->input_dev;

	if(is_tp_suspend() ==false)
	{
		ht_print("POWER  OFF   tp_resume_cound=[%d/%d]  %s",tp_resume_fail_cound,tp_resume_cound,input_dev->name);
	}
	else
	{
		printk("\n\n"); 		
		ht_print("POWER  ON   ");
	}
	
	input_report_key(input_dev, KEY_POWER, 1);
	input_sync(input_dev);
	input_report_key(input_dev, KEY_POWER, 0);
	input_sync(input_dev);		
}

void ctp_power_onoff_test_start(void) //jerry li  Drv. 2016_01_06 
{
	queue_delayed_work(ctp_power_onoff_workqueue,&ctp_power_onoff_work, 100);
}

void issue_a_point(void) //jerry li  Drv. 2016_01_04 
{

	u16 finger=2;
	int x=100;	
	int y=100;	


	struct input_dev *input_dev=g_touch_info->input_dev;	

	
	ht_print("issue_a_point"); //jerry li //Drv. 2015_12_22 //	
	
#ifdef TYPE_B_PROTOCOL
	input_mt_slot(input_dev, finger);
	input_mt_report_slot_state(input_dev,MT_TOOL_FINGER, 1);
#endif

	input_report_key(input_dev,BTN_TOUCH, 1);
	input_report_key(input_dev,BTN_TOOL_FINGER, 1);
	input_report_abs(input_dev,ABS_MT_POSITION_X, x);
	input_report_abs(input_dev,ABS_MT_POSITION_Y, y);


#ifndef TYPE_B_PROTOCOL
	input_mt_sync(input_dev);
#endif
	input_sync(input_dev);
}

void issue_a_point_up_event(void) //jerry li  Drv. 2016_01_04 
{
	u16 finger=2;
	
	struct input_dev *input_dev=g_touch_info->input_dev;



	ht_print("issue_a_point_up_event");
	
#ifdef TYPE_B_PROTOCOL
	input_mt_slot(input_dev, finger);
	input_mt_report_slot_state(input_dev,MT_TOOL_FINGER, 0);
#endif

	input_sync(input_dev);
}

void issue_one_line_A(void) //jerry li  Drv. 2016_01_04 
{



	u16 finger=0;
	int x=500;	
	int y=1600;	
	int i=0;

	struct input_dev *input_dev=g_touch_info->input_dev;
	
	ht_print("issue_one_line_A"); //jerry li //Drv. 2015_12_22 //	

	
	for(i=0;i<20;i++) //jerry li  Drv. 2016_01_08 
	{
#ifdef TYPE_B_PROTOCOL
		input_mt_slot(input_dev, finger);
		input_mt_report_slot_state(input_dev,MT_TOOL_FINGER, 1);
#endif

		input_report_key(input_dev,BTN_TOUCH, 1);
		input_report_key(input_dev,BTN_TOOL_FINGER, 1);
		input_report_abs(input_dev,ABS_MT_POSITION_X, x);
		input_report_abs(input_dev,ABS_MT_POSITION_Y, y);
		y-=40;
	
#ifndef TYPE_B_PROTOCOL
		input_mt_sync(input_dev);
#endif
		input_sync(input_dev);
	}
	
//up
#ifdef TYPE_B_PROTOCOL
	input_mt_slot(input_dev, finger);
	input_mt_report_slot_state(input_dev,MT_TOOL_FINGER, 0);
#endif

	input_sync(input_dev);

}

//jerry li#swdp.driver,  2015/12/31, add for tp debug
ssize_t ht_touch_slim_log_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	ht_print("touch_slim_log =%d",touch_slim_log); //jerry li //Drv. 2015_12_22 //	

	return snprintf(buf, PAGE_SIZE, "touch_slim_log %d\n",touch_slim_log);
	
}
ssize_t ht_touch_slim_log_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	if(*buf=='1')
	{
		touch_slim_log=true;

	}
	else if(*buf=='0')
	{
		touch_slim_log=false;
	}
	else
		touch_slim_log=false;

//jerry li#swdp.driver,  2016/02/24, add for 
	touch_monitor_log=touch_slim_log;
			
	ht_print("touch_slim_log =%d  %s",touch_slim_log,buf); 
	
	return count;
}


void clean_warning_cnt_arr(void) //jerry li  Drv. 2016_02_19 
{
	u16 i=0;
	g_touch_info->warning_cnt=0;
	g_touch_info->wxx=g_touch_info->wyy=0;
	for(i=0;i<10;i++) //jerry li  Drv. 2016_02_19 
	{
		g_touch_info->warning_cnt_arr[i]=0;
	}

}
void print_warning_cnt_arr(void) //jerry li  Drv. 2016_02_19 
{
	u16 i=0;
	ht_print("warning_cnt =%d  avr w[%d,%d]",g_touch_info->warning_cnt,g_touch_info->wxx,g_touch_info->wyy); //jerry li //Drv. 2016_02_19 //

	for(i=0;i<10;i++) //jerry li  Drv. 2016_02_19 
	{
    		ht_print("warning_cnt_arr[%d]=%d",i,g_touch_info->warning_cnt_arr[i]); //jerry li //Drv. 2016_02_19 //
	}

}
/////////////////////////////////////////////////////////////////////////////
// Function name:
// Purpose:
// Input Parameters:
// Return type:
// Output Parameters: 	NONE
/////////////////////////////////////////////////////////////////////////////

ssize_t ht_debug_sys_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	return snprintf(buf, PAGE_SIZE, "ht_debug_menu show 0x%02x\n",touch_common_log);
	
}


ssize_t ht_debug_sys_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
//	unsigned int input;
	struct touch_stc *touch_info=g_touch_info;
	u32 leng=strlen(buf);
	
//	if (sscanf(touch_info->ht_debug_cmd, "%u", &input) != 1)
//		return -EINVAL;


	if((leng<=0)||(leng>=32)||(touch_info==NULL))
	{
//		ht_print("ht_debug_sys_store error=%d",leng);	
		return -EINVAL;	
	}

	touch_info->ht_debug_cmd=touch_info->cmd_buf;
	
	if(touch_info->ht_debug_cmd==NULL) //jerry li  Drv. 2016_02_22 
	{
		return -EINVAL;	
	}
	memcpy(touch_info->ht_debug_cmd,buf,leng);
	ht_print("len=%d  buf=%s cmd=%s",leng,buf,touch_info->ht_debug_cmd);
	
	schedule_work(&touch_info->ht_debug_root_work);

	return count;

}
void ht_debug_root_ff(struct work_struct *work)
{

	char cmd=0;
	char* cmd_buf;
	int parm1=0;
	int parm2=0;
	int i=0;	
	struct touch_stc *touch_info=g_touch_info;

//	ht_print("ht_debug_root_ff  len=%d ",(int)strlen(touch_info->ht_debug_cmd));
	printk("\n");	

	if(touch_info->ht_debug_cmd==NULL) //jerry li  Drv. 2016_02_03 
	{
		return;
	}

	cmd_buf=touch_info->ht_debug_cmd;
	cmd=*touch_info->ht_debug_cmd;
	
//    sscanf(touch_info->ht_debug_cmd, "%u", &input)
//	sscanf(buf, "%u", &cmd)

	switch (cmd_buf[0]) 
	{
	case '0':
		parm1=parm2;
		
		for(i=0;i<10;i++) //jerry li  Drv. 2016_02_24 
		{
			queue_delayed_work(touch_timer_monitor_10s_work_queue,&touch_timer_monitor_10s_work, 30);

			printk("<<\n");	
			mdelay(100);
			printk(">>\n");
			
		}
		break;	
	case '9':
		parm1=parm2;
		
		for(i=0;i<10;i++) //jerry li  Drv. 2016_02_24 
		{
			cancel_delayed_work_sync(&touch_timer_monitor_10s_work);
			queue_delayed_work(touch_timer_monitor_10s_work_queue,&touch_timer_monitor_10s_work, 50);
			printk("<<\n");	
			mdelay(100);
			printk(">>\n");
			
		}
		break;	
	case 'u':
		sscanf(cmd_buf, "u %d %d", &parm1,&parm2);
		printk("u get %d %d\n",parm1,parm2);	
		break;		
	case 'n':
		printk("g_touch_info->input_dev->name = %s\n",touch_info->input_dev->name);

		break;
	case '1':
		printk("issue_a_point\n");
		issue_a_point();
		break;
	case '2':
		printk("issue_a_point_up_event\n");
		issue_a_point_up_event();
		break;	
	case '3':
		printk("issue_one_line_A\n");
		issue_one_line_A();
		break;	
		
	case 'E':
		touch_info->eare_eage_function=!touch_info->eare_eage_function;
		printk("eare_eage_function =%d\n",touch_info->eare_eage_function);	
		break;	
	case 'M':
		touch_info->super_monitor_function=!touch_info->super_monitor_function;
		printk("super_monitor_function =%d\n",touch_info->super_monitor_function);	
		break;	
	case 'A':
		touch_info->data_adapter_function=!touch_info->data_adapter_function;
		printk("data_adapter_function =%d\n",touch_info->data_adapter_function);	
		break;			
	case 'F':
		touch_info->super_fliter_function=!touch_info->super_fliter_function;
		printk("super_fliter_function =%d\n",touch_info->super_fliter_function);	
		break;			
	case 'c':
		{
			if(synaptics_is_tp_low_sens_mode())
			synaptics_low_sens_mode_disable_ll();
			else
			synaptics_low_sens_mode_enable_ll();
		}
		break;			
	case 's':
		touch_slim_log=!touch_slim_log;
		printk("touch_slim_log =%d\n",touch_slim_log);	
		break;		
	case 'd':
		touch_common_log=!touch_common_log;
		printk("touch_common_log =%d\n",touch_common_log);	
		break;
	case 'm':
		touch_monitor_log=!touch_monitor_log;
		printk("touch_monitor_log =%d\n",touch_monitor_log);	
		break;		
	case 'o':
		tp_power_on_off_test=!tp_power_on_off_test;
		
		if(tp_power_on_off_test)
		ctp_power_onoff_test_start( );	//200	

		tp_resume_cound=0;
		tp_resume_fail_cound=0;
		
		printk("ctp_power_onoff_test_start =%d\n",tp_power_on_off_test);
		
		break;

	case 'p':
		set_bit(KEY_POWER, g_touch_info->input_dev->keybit);
		input_set_capability(g_touch_info->input_dev, EV_KEY, KEY_POWER);
		ctp_power_onoff_key_func();
		break;	

	case 'y':
		{
			struct timeval now;
			struct timeval time2;
			
			do_gettimeofday(&now);

			printk("time1 %ld %ld\n",now.tv_sec,now.tv_usec/1000);
			mdelay(10);
			do_gettimeofday(&time2);
			ht_print("time2 %ld %ld\n",time2.tv_sec,time2.tv_usec/1000);
			printk("time diff %ld %ld\n",time2.tv_sec-now.tv_sec,time2.tv_usec/1000-now.tv_usec/1000);
			
		}
		break;	
	case 't':
		{
			u64 t_ms;
			u64 t_ms2;
			
			get_sys_time_ms(&t_ms);
			mdelay(10);
			get_sys_time_ms(&t_ms2);
			
			printk("time diff %lld  (%lld-%lld),  %d\n",t_ms2-t_ms,t_ms2,t_ms,(u32)(t_ms2-t_ms));
			
		}
		break;	

	case 'a':
		clean_warning_cnt_arr( ); //jerry li  Drv. 2016_02_19 
		break;	
	case 'b':
		print_warning_cnt_arr( ); //jerry li  Drv. 2016_02_19 
		break;	
	case 'w':
		sscanf(cmd_buf, "w %x", &parm2);
		printk("get %x\n",parm2);	
		synaptics_reg_write(0x0019,parm2); //jerry li  Drv. 2016_02_19 
		break;	
	case 'r':
		sscanf(cmd_buf, "r %x", &parm2);
		printk("get %x\n",parm2);	
		synaptics_reg_read(parm2);
		break;			
	default:
		printk("\n");	
		printk("--------------------------------\n");	
		printk("cmd =%c  len=%d  buf=%s\n",cmd,(u32)strlen(touch_info->ht_debug_cmd),touch_info->ht_debug_cmd);
		
		printk("unknow cmd [%c]\n",cmd);		
		
//		printk("cmd =%c  len=%d  buf=%s",cmd,(u32)strlen(touch_info->ht_debug_cmd),touch_info->ht_debug_cmd);
		printk("r  reset tp auto test\n");	
		printk("p  issue ctp_power_onoff_key_func \n");
		printk("--------------------------------\n");			
		break;
	}
		


	return ;
	
}
void ht_debug_root_work_func(struct work_struct *work)
{
	ht_debug_root_ff(work);

}

//#endif	

/////////////////////////////////////////////////////////////////////////////
// Function name:
// Purpose:
// Input Parameters:
// Return type:
// Output Parameters: 	NONE
/////////////////////////////////////////////////////////////////////////////

u64 get_sys_time_ms(u64 *time_ms) //(u64 *time_ms)
{
	struct timeval tv;
	u64 t_ms;
	do_gettimeofday(&tv);
	t_ms=(u64)((tv.tv_sec<<10)+(tv.tv_usec>>10));
	*time_ms=t_ms;
	return t_ms;
}
bool  touch_common_is_inited(void)
{
	return touch_common_inited;
}
int  touch_common_init(struct input_dev *input_dev) //input_dev
{
//	printk("tp_common_init\n");

	unsigned char attr_count;
	int retval;	

  
	g_touch_info = kmalloc(sizeof(struct touch_stc), GFP_KERNEL);

	if(NULL==g_touch_info)
	{
		touch_common_inited=false;
		ht_print("touch_common_inited  Fail");
		return 0;
	}
	else
	{
		touch_common_inited=true;
		ht_print("touch_common_inited  Successful");
	}


	memset(g_touch_info, 0, sizeof(struct touch_stc)); //(struct touch_stc)

	g_touch_info->input_dev=input_dev;


	
	INIT_DELAYED_WORK(&touch_timer_monitor_10s_work, touch_timer_monitor_10s_work_fun);
	touch_timer_monitor_10s_work_queue = create_workqueue("touch_timer_monitor_10s_work_fun");
	INIT_DELAYED_WORK(&touch_timer_monitor_3s_work, touch_timer_monitor_3s_work_fun);
	touch_timer_monitor_3s_work_queue= create_workqueue("touch_timer_monitor_3s_work_fun");	
	INIT_DELAYED_WORK(&touch_timer_monitor_3min_work, touch_timer_monitor_3min_work_fun);
	touch_timer_monitor_3min_work_queue= create_workqueue("touch_timer_monitor_3min_work_fun");	
	INIT_DELAYED_WORK(&ctp_power_onoff_work, ctp_power_onoff_func);
	ctp_power_onoff_workqueue = create_workqueue("ctp_power_onoff");

	INIT_WORK(&g_touch_info->touch_slim_log_work, touch_common_slim_log_func);
	INIT_WORK(&g_touch_info->touch_monitor_reset_work, touch_monitor_reset_func);
	INIT_WORK(&g_touch_info->ht_debug_root_work, ht_debug_root_work_func);	
	
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
	{
		retval = sysfs_create_file(&input_dev->dev.kobj,&attrs[attr_count].attr);
		
		if (retval < 0)
		{
 			ht_print("fail: sysfs_create_file  %s/input0",dev_name(&input_dev->dev));
		}
		else
		{
 			ht_print("seccessful: sysfs_create_file %s/input0",dev_name(&input_dev->dev));
		}
	}


	set_bit(KEY_POWER, input_dev->keybit);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);

	clean_warning_cnt_arr(); 
	g_touch_info->warning_cnt=0;
	g_touch_info->reset_cnt=0;
	g_touch_info->flag_10sec_timer_working=false;
	g_touch_info->flag_reset_3s_available=true;
	g_touch_info->wxx=5;
	g_touch_info->wyy=5;
	
	g_touch_info->eare_eage_function=1;
	g_touch_info->super_monitor_function=1;	
	g_touch_info->data_adapter_function=1;		
	g_touch_info->super_fliter_function=1;	
	
#ifdef  HT_ENABLE_LOG_INIT
	touch_common_log=1;
#endif 
#ifdef  HT_ENABLE_SLIM_LOG_INIT
	touch_slim_log=1;
#endif 

#ifdef  HT_ENABLE_MONITOR_LOG_INIT
	touch_monitor_log=1;
#endif 

       return 1;
}

int touch_common_exit(void)
{

	//cancel_work_sync(gtp_esd_check_workqueue);
	cancel_delayed_work_sync(&touch_timer_monitor_10s_work);
	flush_workqueue(touch_timer_monitor_10s_work_queue);
	destroy_workqueue(touch_timer_monitor_10s_work_queue);


	cancel_delayed_work_sync(&touch_timer_monitor_3s_work);
	flush_workqueue(touch_timer_monitor_3s_work_queue);
	destroy_workqueue(touch_timer_monitor_3s_work_queue);	
	cancel_delayed_work_sync(&touch_timer_monitor_3min_work);
	flush_workqueue(touch_timer_monitor_3min_work_queue);
	destroy_workqueue(touch_timer_monitor_3min_work_queue);	

	cancel_delayed_work_sync(&ctp_power_onoff_work);
	flush_workqueue(ctp_power_onoff_workqueue);
	destroy_workqueue(ctp_power_onoff_workqueue);

       kfree(g_touch_info);	

	printk("touch_common_exit\n");

	return 1;
}

/* Removed  //jerry li  Drv. 2016_01_11 
//
module_init(tp_common_init);
module_exit(tp_common_exit);

MODULE_AUTHOR("tp_common");
MODULE_DESCRIPTION("tp_common");
MODULE_LICENSE("GPL v2");
*/


#else
bool touch_common_log=0;
bool touch_common_inited=0;


void ctp_power_onoff_func(struct work_struct *work){}

ssize_t ht_touch_slim_log_show(struct device *dev,struct device_attribute *attr, char *buf){}
ssize_t ht_touch_slim_log_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count){}

ssize_t ht_debug_sys_show(struct device *dev,struct device_attribute *attr, char *buf){}
ssize_t ht_debug_sys_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count){}

int touch_common_start(struct touch_stc *touch_info){}
int touch_common_end(struct touch_stc *touch_info){}
int touch_eare_eage_pro(struct touch_stc *touch_info,bool *is_delete){}
int touch_data_adapter(struct touch_stc *touch_info, struct synaptics_rmi4_data *rmi4_data){}
int touch_super_monitoring(struct touch_stc *touch_info,struct synaptics_rmi4_data *rmi4_data){}
int touch_super_power(struct touch_stc *touch_info,struct synaptics_rmi4_data *rmi4_data){}
int touch_super_filter(struct touch_info *cinfo, u8 *point_num){}
int touch_common_init(struct input_dev *input_dev){}
int touch_common_exit(void){}

bool  touch_common_is_inited(void){}
bool touch_common_slim_log(void){}
#endif  



















#if 0//def OLD_12_01 //----------------------------------------------------jerry li //Drv. 2015_12_01 //
/////////////////////////////////////////////////////////////////////////////
// Function name:
// Purpose:
// Input Parameters:
// Return type:
// Output Parameters: 	NONE
/////////////////////////////////////////////////////////////////////////////
int touch_eare_eage_pro(struct touch_info_syn *cinfo,bool *is_delete) 
{
//	u16 i;	
//	u16 j;	
//	u8 k=0;

	static bool start_point_in_detectiong_area[10]={0};
	static int start_point_pos_y[10]={0};
	static bool point_moving[10]={0};
	static bool point_moving_valid[10]={0};	

	static bool flag_point_delete_last[10]={0};	
//	static u16 touch_count_id[10]={0};
//	int touch_step_p[10]={0};

	bool is_point_in_detection_area=0;

	bool flag_point_delete=0;

	u8 point_num_cur=0;	
//	u8 point_num_cal=0;
	
//	static u8 point_num_last=0;
//	static u8 point_num_cal_last=0;
	
	u8 finger=0;
	static u8 finger_last=0;	
//	static u8 finger_status_last=0;	
	static u8 fingers_to_process_last=0;	

//	static int last_pos_x=0;	
//	static int last_pos_y=0;	

	static u8 last_finger_status[10]={0};
	
	u16 index=0;

	static u8 tmp_timer=0;

//	static u8 key_tp=0;
	static bool key_enable_function=1;


	static int last_x=0;
	static int last_y=0;

	int diff_x=0;
	int diff_y=0;	
	
	if(cinfo==NULL) 
	{
		ht_print(":error   touch_eare_eage_pro   cinfo =NULL ");	
		return true; 
	}	

	finger=cinfo->finger;
	point_num_cur=finger_last=index=cinfo->finger;

	if((cinfo->finger_status>=FINGER_STATUS_DOWN_AND_PRESS)&&(last_finger_status[finger]==FINGER_STATUS_UP))
	{//finger dowm 
	  //reset /clean 0
		touch_step[finger]=0;
		start_point_in_detectiong_area[finger]=0;
		start_point_pos_y[finger]=cinfo->y;
		point_moving[finger]=0;
		point_moving_valid[finger]=0;
		flag_point_delete_last[finger]=0;
		tmp_timer=0;
		ht_print_if_c(" ");
		
#if 0
		if((cinfo->x<150)&&(cinfo->y<=50)) //jerry li  Drv. 2015_09_22 
		{
			key_enable_function=!key_enable_function;	
			ht_print(" ");	
			ht_print("key_enable_function =%d",key_enable_function);	
			ht_print(" ");				
		}
#endif
#if 1
		if((cinfo->x<20)&&(cinfo->y<=20)) //jerry li  Drv. 2015_09_22 
		{
			touch_common_log=!touch_common_log;	
			ht_print_if_c("touch_common_log =%d",touch_common_log);	
		
		}
#endif

#if 0
		if((cinfo->x>=1000)&&(cinfo->y<=20)) //jerry li  Drv. 2015_09_22 
		{
			key_test_flag=1;	
			printk("key_test_flag =%d",key_test_flag);	
		
		}
#endif
	}
	else if((cinfo->finger_status==FINGER_STATUS_UP)&&(last_finger_status[finger]>=FINGER_STATUS_DOWN_AND_PRESS))
	{//up
//		touch_step[finger]=0;
//		ht_print("%d/%d  %03d:  p( %d,%d )     up    end",cinfo->finger+1,cinfo->fingers_to_process,touch_step[finger],cinfo->x,cinfo->y);
//		ht_print("}}}}}}}}}}}}}}}}}}}}}");	
//		ht_print(" ");				
	}
	else if(cinfo->finger_status>=FINGER_STATUS_DOWN_AND_PRESS)
	{//press
//		touch_step[finger]++;
	}
//	else //(cinfo->finger_status==0)
//	{//nothing thouch
//	}

	touch_step[finger]++;

	if(touch_step[cinfo->finger]==1)
	{
		diff_x=0;diff_y=0;
	}
	else if(touch_step[cinfo->finger]>=2)
	{
		diff_x=cinfo->x-last_x;
		diff_y=cinfo->y-last_y;
	}

	last_x=cinfo->x;
	last_y=cinfo->y;
	

	ht_print_if_c("%d/%d %03d: p( %d,%d) dif[%d,%d] w[%d,%d] status=0x%x",
	cinfo->finger+1,cinfo->fingers_to_process,touch_step[finger],
	cinfo->x,cinfo->y,
	diff_x,diff_y,	
	cinfo->xw,cinfo->yw,
	cinfo->finger_status
//	cinfo->finger,cinfo->finger_status,cinfo->fingers_to_process,
//	is_point_in_detection_area ? "-->point in detection":""
	);	


	is_point_in_detection_area=check_is_point_in_detection_area(cinfo->x,cinfo->y);


	if((!is_point_in_detection_area)&&(cinfo->finger_status>=FINGER_STATUS_DOWN_AND_PRESS))
	{
		point_moving_valid[finger]=true;
	}
	
	if((is_point_in_detection_area)&&(cinfo->finger_status>=FINGER_STATUS_DOWN_AND_PRESS))
	{

		if(touch_step[finger]==1)
		{
			start_point_in_detectiong_area[finger]=true;
		}
		else //(touch_step[finger]>=2)
		{
			if(abs(cinfo->y -start_point_pos_y[finger])>TP_VALID_POINT_MOVING)	
			{
				point_moving[finger]=true;
			}
		}

		if(start_point_in_detectiong_area[finger]
		&&!point_moving[finger]
		&&!point_moving_valid[finger]		
		)
		{	

			if((cinfo->x>=1079)||(cinfo->x==0))
			{

	//			if((cinfo->xw>=10)&&(cinfo->yw>=10)) //20)
				{
//					ht_print("%d/%d  %03d:  p( %d,%d ),   w[%d,%d] -------=1",cinfo->finger+1,cinfo->fingers_to_process,touch_step[finger],cinfo->x,cinfo->y,cinfo->xw,cinfo->yw);	
					ht_print_if_c("------- =1");	
					flag_point_delete=1;
				}
			}
			else if((cinfo->x>=1076)||(cinfo->x<=3))
			{
//				if((cinfo->xw>=12)&&(cinfo->yw>=15))
				{
					ht_print_if_c("------- <3");	
					flag_point_delete=1;				
				}
			}
			else if((cinfo->x>=1073)||(cinfo->x<=6))
			{
//				if((cinfo->xw>=12)&&(cinfo->yw>=15))
				{
					ht_print_if_c("------- <6");	
					flag_point_delete=1;				
				}
			}
			else if((cinfo->x>=1067)||(cinfo->x<=20)) //1059
			{
//				if((cinfo->xw>=W_X)&&(cinfo->yw>=W_Y))
				{
					ht_print_if_c("------- <20");	
					flag_point_delete=1;				
				}
			}		
			else
			{
				if((cinfo->xw>=20)&&(cinfo->yw>=25))
				{
					ht_print_if_c("-------  20<x<100");	
					flag_point_delete=1;				
				}	
				else
				{
					ht_print_if_c("++++++  20<x<100");	
				}
			}

			if((flag_point_delete==0)&&(flag_point_delete_last[finger]==1)) //jerry li  Drv. 2015_08_25 
			{
				tmp_timer++;
				if(tmp_timer<=6)	
				{
				flag_point_delete=1;
				ht_print_if_c("------- tmp_timer ++=%d",tmp_timer);	
				}
			}


		}


		if(0==flag_point_delete)
		{

			ht_print_if_c("+++++++  move=%d %d=%d-%d; move_valid %d",
			point_moving[finger],(int)abs(cinfo->y -start_point_pos_y[finger]),cinfo->y,start_point_pos_y[finger],point_moving_valid[finger]	
			);	
		}
		
	}


// up print move to here 
	if((cinfo->finger_status==FINGER_STATUS_UP)&&(last_finger_status[finger]>=FINGER_STATUS_DOWN_AND_PRESS))//(touch_step[finger]==0) //jerry li  Drv. 2015_09_18 
	{
//		touch_step[finger]=0;
		if(flag_point_delete_last[finger]==1)
		{
			flag_point_delete=1;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////


	if(fingers_to_process_last!=cinfo->fingers_to_process)
	{
		ht_print_if_c("point change      process=%d  last %d",
		cinfo->fingers_to_process,fingers_to_process_last
		);	

		if((fingers_to_process_last+1)==cinfo->fingers_to_process)
		ht_print_if_c("point new (%d,%d)",cinfo->x,cinfo->y);	
		
	}

/* Removed  //jerry li  Drv. 2015_11_07 
//
<6>[  115.403105] point change      process=223  last 1
<6>[  115.407802] 1/223 075: p( 0,0) diff[-712,-824]  case default   1/223  :  p( 0,0 ),   w[0,0] Finger=0 sta
tus=0x0 process=223
*/

//////////////////////////////////////////////////////////////////////
	finger_last=finger;
	fingers_to_process_last=cinfo->fingers_to_process;
	last_finger_status[finger]=cinfo->finger_status;
	flag_point_delete_last[finger]=flag_point_delete;

	
	if((key_enable_function)&&(flag_point_delete)) //jerry li  Drv. 2015_09_22 
	{
		*is_delete=1;
	}
	else
	{
		*is_delete=0;
	}

// disable touch_eare_eage_pro
	*is_delete=0;

	
	return *is_delete;	
}





int touch_data_adapter(struct touch_info_syn *cinfo, struct synaptics_rmi4_data *rmi4_data) 
{
	int i=0;
	
	static int last_x=0;
	static int last_y=0;

	static int last_diff_x=0;
	static int last_diff_y=0;

	static int list_diff_x[20]={0};
	static int list_diff_y[20]={0};

	static int da_diff_x[20]={0};
	static int da_diff_y[20]={0};	
	
	int diff_x=0;
	int diff_y=0;

	int temp_x=0;
	int temp_y=0;
	
	int da_diff_x_tmp=0;
	int da_diff_y_tmp=0;	
	
	int calculate_point_num=3;	

	static u8 last_finger_status[10]={0};
	


		
	if((cinfo->finger_status>=FINGER_STATUS_DOWN_AND_PRESS)&&(last_finger_status[cinfo->finger]==FINGER_STATUS_UP))
	{//finger dowm 


	}
	else if((cinfo->finger_status==FINGER_STATUS_UP)&&(last_finger_status[cinfo->finger]>=FINGER_STATUS_DOWN_AND_PRESS))
	{//up
	
		if((cinfo->finger==0)&&(touch_step[cinfo->finger]>=3)&&(touch_step[cinfo->finger]<=16)
		&&((abs(last_diff_x)>=2)||(abs(last_diff_y)>=2))&&((abs(last_diff_x)+abs(last_diff_y))<=150)
		)
		{

			
#if 1//def OLD_11_07 //----------------------------------------------------jerry li //Drv. 2015_11_07 //
			for(i=2;i<touch_step[cinfo->finger];i++) //jerry li  Drv. 2015_11_07 
			{
				if((abs(list_diff_x[i]))>=1)
				da_diff_x[i+1]=(abs(list_diff_x[i+1])<<10)/(abs(list_diff_x[i]));
				else
				da_diff_x[i+1]=(1<<10);//(abs(list_diff_x[i+1])<<10);

				if((abs(list_diff_y[i]))>=1)				
				da_diff_y[i+1]=(abs(list_diff_y[i+1])<<10)/(abs(list_diff_y[i]));
				else
				da_diff_y[i+1]=(1<<10);//(abs(list_diff_y[i+1])<<10);

				if(da_diff_x[i+1]>0) //jerry li  Drv. 2015_11_07 
				{
					da_diff_x_tmp++;
				}

				if(da_diff_y[i+1]>0) //jerry li  Drv. 2015_11_07 
				{
					da_diff_y_tmp++;
				}

				if(da_diff_x[i+1]>(2<<10)) 
				da_diff_x[i+1]=(2<<10);	

				if(da_diff_y[i+1]>(2<<10)) 
				da_diff_y[i+1]=(2<<10);	
				
				
				temp_x+=da_diff_x[i+1];
				temp_y+=da_diff_y[i+1];	

//				ht_print("%02d: ( %d,%d)  [%d,%d]   [%d,%d]   [%d,%d]",i,temp_x,temp_y,da_diff_x[i+1],da_diff_y[i+1],list_diff_x[i+1],list_diff_y[i+1],list_diff_x[i],list_diff_y[i]);				
			}

			temp_x=((temp_x/da_diff_x_tmp));
			temp_y=((temp_y/da_diff_y_tmp));
			
//			ht_print(" %d,%d     %d,%d  ",temp_x,temp_y,da_diff_x_tmp,da_diff_y_tmp);		

			if(temp_x<(1<<9)) 
			temp_x=(1<<9);	
			if(temp_y<(1<<9)) 
			temp_y=(1<<9);	

			if(temp_x>(2<<10)) 
			temp_x=(2<<10);				
			if(temp_y>(2<<10)) 
			temp_y=(2<<10);	
			
#else
			for(i=2;i<=touch_step[cinfo->finger];i++) //jerry li  Drv. 2015_11_07 
			{
				if((abs(list_diff_x[i]))>=1)
				da_diff_x[i]=(abs(list_diff_x[i+1]))/(abs(list_diff_x[i]));
				else
				da_diff_x[i]=(abs(list_diff_x[i+1]));

				if((abs(list_diff_y[i]))>=1)				
				da_diff_y[i]=(abs(list_diff_y[i+1]))/(abs(list_diff_y[i]));
				else
				da_diff_y[i]=(abs(list_diff_y[i+1]));
				
				temp_x+=da_diff_x[i];
				temp_y+=da_diff_y[i];				
			}

			temp_x=((temp_x/touch_step[cinfo->finger]));
			temp_y=((temp_y/touch_step[cinfo->finger]));
				
			if(temp_x<65536) 
			temp_x=65536;	
			if(temp_y<65536) 
			temp_y=65536;	
#endif //
			if((abs(last_diff_x)<=10)&&(abs(last_diff_y)<=10)&&(abs(last_diff_x)>=3||(abs(last_diff_y)>=3)))
			calculate_point_num=4;
			else if(touch_step[cinfo->finger]<=6)
			calculate_point_num=3;
			else if(touch_step[cinfo->finger]<=12)
			calculate_point_num=2;
			else
			calculate_point_num=1;				

			for(i=0;i<calculate_point_num;i++) //jerry li  Drv. 2015_11_06 
			{

#if 0//def TEST 
				if(last_diff_x>0) //jerry li  Drv. 2015_11_07 
					last_diff_x=1;
				else if(last_diff_x<=-1)
					last_diff_x=-1;
				if(last_diff_y>0) //jerry li  Drv. 2015_11_07 
					last_diff_y=1;
				else if(last_diff_y<=-1)
					last_diff_y=-1;
				
#endif
#if 1
				diff_x=(abs(list_diff_x[touch_step[cinfo->finger]-1])*temp_x)>>10;
				diff_y=(abs(list_diff_y[touch_step[cinfo->finger]-1])*temp_y)>>10;
#else
				if(list_diff_x[touch_step[cinfo->finger]]>=list_diff_x[touch_step[cinfo->finger]-1])
				{
					if(list_diff_x[touch_step[cinfo->finger-1]]>=2)
					diff_x=list_diff_x[touch_step[cinfo->finger]]+list_diff_x[touch_step[cinfo->finger-1]]>>1;
					else
					diff_x=list_diff_x[touch_step[cinfo->finger]]+list_diff_x[touch_step[cinfo->finger-1]];					
				}
				else
				diff_x=list_diff_x[touch_step[cinfo->finger]];	
				
				
				diff_x=(abs(last_diff_x)*temp_x)>>10;
				diff_y=(abs(last_diff_y)*temp_y)>>10;
#endif //

//check 
				if(diff_x>100) 
				diff_x=100;	
				if(diff_y>100) 
				diff_y=100;	
				
				if(last_diff_x<0) 
				diff_x=-diff_x;	
				if(last_diff_y<0) 
				diff_y=-diff_y;					

				if(abs(last_diff_x+diff_x)>120||abs(last_diff_y+diff_y)>120)
				break;
				
				list_diff_x[touch_step[cinfo->finger]]=diff_x;
				list_diff_y[touch_step[cinfo->finger]]=diff_y;	

				cinfo->x=last_x+diff_x;
				cinfo->y=last_y+diff_y;

				
				input_report_key(rmi4_data->input_dev,BTN_TOUCH, 1);
				input_report_key(rmi4_data->input_dev,BTN_TOOL_FINGER, 1);
				input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_X, cinfo->x);
				input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_Y, cinfo->y);
				input_sync(rmi4_data->input_dev);

#if 1//def 			
				ht_print_if_c("%d/%d %03d: p( %d,%d) dif[%d,%d]    @(%d,%d)----- touch_data_adapter",
				cinfo->finger+1,cinfo->fingers_to_process,touch_step[cinfo->finger],
				cinfo->x,cinfo->y,
				diff_x,diff_y,
				temp_x,temp_y
				);

#else
//				mdelay(5);
#endif

				last_x=cinfo->x;
				last_y=cinfo->y;

				touch_step[cinfo->finger]++;
			
			}
//end
/* Removed  //jerry li  Drv. 2015_11_27 
//
			for(i=2;i<touch_step[cinfo->finger];i++) //jerry li  Drv. 2015_11_07 
			{

				ht_print_if_c("%02d: ( %d,%d)  [%d,%d]  [%d,%d][%d,%d]",
				i,
				list_diff_x[i],list_diff_y[i],
				da_diff_x[i],da_diff_y[i],
				temp_x,temp_y,da_diff_x_tmp,da_diff_y_tmp
				);
			}
*/
/* Removed  //jerry li  Drv. 2015_11_07 
//
6>[  338.160766] 1/1 010: p( 615,1367) diff[0,0]    @(0,0)----- soon_touch_
6>[  338.166997] 1/1 011: p( 615,1367) diff[0,0]    @(0,0)----- soon_touch_
6>[  338.173660] 1/1 012: p( 615,1367) diff[0,0]    @(0,0)----- soon_touch_
6>[  338.180223] 1/1 013: p( 615,1367) diff[0,0]    @(0,0)----- soon_touch_
6>[  338.186479] 00: ( 0,0)  [0,0]
6>[  338.189522] 01: ( 0,0)  [0,0]
6>[  338.192672] 02: ( 609,1471)  [0,0]
6>[  338.196032] 03: ( 0,0)  [0,0]
6>[  338.199063] 04: ( -1,-6)  [0,0]
6>[  338.202328] 05: ( 0,-11)  [0,0]
6>[  338.205496] 06: ( 0,-18)  [0,0]
6>[  338.208706] 07: ( 1,-21)  [131072,131072]
6>[  338.212863] 08: ( 2,-22)  [131072,131072]
6>[  338.216867] 09: ( 4,-26)  [0,0]			
*/
//			ht_print("%02d: ( %d,%d) dif[%d,%d]  ----- soon_touch_",);
			
			cinfo->x=0;
			cinfo->y=0;
					
		}


		for(i=0;i<16;i++) //jerry li  Drv. 2015_11_07 
		{
			list_diff_x[i]=0;
			list_diff_y[i]=0;
			da_diff_x[i]=0;
			da_diff_y[i]=0;
			
		}


	}
	else if(cinfo->finger_status>=FINGER_STATUS_DOWN_AND_PRESS)
	{//press
		
	}
//	else //(cinfo->finger_status==0)
//	{//nothing thouch

//	}


//////////////////////////////////////////////////////////////////////////////////////
	if(touch_step[cinfo->finger]==1)
	{
		diff_x=0;diff_y=0;
	}
	else if(touch_step[cinfo->finger]>=2)
	{
		diff_x=cinfo->x-last_x;
		diff_y=cinfo->y-last_y;
		
		if(touch_step[cinfo->finger]<16)
		{
			list_diff_x[touch_step[cinfo->finger]]=diff_x;
			list_diff_y[touch_step[cinfo->finger]]=diff_y;		
		}
	}

	last_x=cinfo->x;
	last_y=cinfo->y;

	last_diff_x=diff_x;
	last_diff_y=diff_y;


#ifdef OLD_11_27 //----------------------------------------------------jerry li //Drv. 2015_11_27 //
	ht_print_if_c("%d/%d %03d: p( %d,%d) dif[%d,%d]   normal",
	cinfo->finger+1,cinfo->fingers_to_process,touch_step[cinfo->finger],
	cinfo->x,cinfo->y,
	diff_x,diff_y
	);	
#endif //-----------------------------------------------------------------OLD_11_27_


	last_finger_status[cinfo->finger]=cinfo->finger_status;


#if 1//def NEW_11_15 ----------------------------------------------------jerry li  Drv. 2015_11_15

	if(key_test_flag) //jerry li  Drv. 2015_11_15 
	{
//		input_report_key(rmi4_data->input_dev,BTN_TOUCH, 1);
//		input_report_key(rmi4_data->input_dev,BTN_TOOL_FINGER, 1);
//		input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_X, 200);
//		input_report_abs(rmi4_data->input_dev,ABS_MT_POSITION_Y, 200);
//		input_sync(rmi4_data->input_dev);
				
		key_test_flag=0;	
	}
#endif //-----------------------------------------------------------------NEW_11_15_
	
	return 0;	
}
#endif //-----------------------------------------------------------------OLD_12_01_

