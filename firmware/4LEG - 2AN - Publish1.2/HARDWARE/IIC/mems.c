#include "include.h"
#include "mems.h"
#include "imu.h"
#include "vmc.h"
#include "icm20602.h"
#include "cycle_cal_oldx.h"
#include "filter.h"
u8 acc_3d_step;
_MEMS mems;
float FLT_MAG=10;
void IMU_Read(void)
{ static int cnt;

   LSM6_readAcc(1);LSM6_readGyro(1);
	if(cnt++>1){cnt=0;
	 LIS3MDL_read();
	 mems.Mag_update=1;
	}
}

s32 sum_temp[7]= {0,0,0,0,0,0,0};
float sum_temp_att[2]={0};
s32 sum_temp_3d[7]= {0,0,0,0,0,0,0};
u16 acc_sum_cnt = 0,acc_sum_cnt_3d=0,acc_smple_cnt_3d=0,gyro_sum_cnt = 0;
#define OFFSET_AV_NUM_ACC 50
void MEMS_Data_Offset()
{
    if(mems.Acc_CALIBRATE == 1)
    {
        acc_sum_cnt++;
				if(mems.Cali_3d){
				  sum_temp_att[0]+=Pitch;
					sum_temp_att[1]+=Roll;
				}
				
				{
        sum_temp[A_X] += mems.Acc_I16.x;
        sum_temp[A_Y] += mems.Acc_I16.y;
        sum_temp[A_Z] += mems.Acc_I16.z - 65536/16;   // +-8G
				}

        if( acc_sum_cnt >= OFFSET_AV_NUM )
        {   
					if(vmc_all.param.cal_flag[1]==0&&vmc_all.param.cal_flag[0]==0){
            mems.Acc_Offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
            mems.Acc_Offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
            mems.Acc_Offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
					}
            acc_sum_cnt =0;
            mems.Acc_CALIBRATE = 0;
					if(module.flash)
            WRITE_PARM();
            sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
				  	sum_temp_att[1]=sum_temp_att[0]=0;
        }
    }
// 3d cal
		static xyz_f_t ACC_Reg;
		static u8 acc_3d_step_reg;
		float sphere_x,sphere_y,sphere_z,sphere_r;
		switch(acc_3d_step)
			{ 
			case 0:
				acc_smple_cnt_3d=acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;
				cycle_init_oldx(&hml_lsq);
			break;
			default:
			if(hml_lsq.size<360&&(fabs(ACC_Reg.x-mems.Acc_I16.x)>0||fabs(ACC_Reg.y-mems.Acc_I16.y)>0||fabs(ACC_Reg.z-mems.Acc_I16.z)>0))
			{
			if(acc_3d_step>acc_3d_step_reg)
			acc_smple_cnt_3d=acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;		
			acc_sum_cnt_3d++;
      sum_temp_3d[A_X] += mems.Acc_I16.x;
      sum_temp_3d[A_Y] += mems.Acc_I16.y;
      sum_temp_3d[A_Z] += mems.Acc_I16.z;   
				if(acc_sum_cnt_3d>OFFSET_AV_NUM_ACC){
					if(acc_smple_cnt_3d<12){
					acc_smple_cnt_3d++;	
					xyz_f_t data;	
					data.x = sum_temp_3d[A_X]/OFFSET_AV_NUM_ACC;
					data.y = sum_temp_3d[A_Y]/OFFSET_AV_NUM_ACC;
					data.z = sum_temp_3d[A_Z]/OFFSET_AV_NUM_ACC;	
					acc_sum_cnt_3d=sum_temp_3d[A_X] = sum_temp_3d[A_Y] = sum_temp_3d[A_Z] = sum_temp_3d[TEM] = 0;	
					cycle_data_add_oldx(&hml_lsq, (float)data.x/1000.,(float)data.y/1000.,(float)data.z/1000.);}
					else if(acc_3d_step==6){
					acc_3d_step=0;	
					cycle_cal_oldx(&hml_lsq, 666,0.001,  &sphere_x, &sphere_y, &sphere_z, &sphere_r);	
					mems.Off_3d.x=(hml_lsq.Off[0]*1000);
					mems.Off_3d.y=(hml_lsq.Off[1]*1000);
					mems.Off_3d.z=(hml_lsq.Off[2]*1000);
					mems.Gain_3d.x =  (hml_lsq.Gain[0]);
					mems.Gain_3d.y =  (hml_lsq.Gain[1]);
					mems.Gain_3d.z =  (hml_lsq.Gain[2]);	
					if(module.flash)
						WRITE_PARM();						
					}		 		
				} 
			acc_3d_step_reg=acc_3d_step;	
			}
			break;
		
		}
		ACC_Reg.x=mems.Acc_I16.x;
	  ACC_Reg.y=mems.Acc_I16.y;
		ACC_Reg.z=mems.Acc_I16.z;


    if(mems.Gyro_CALIBRATE)
    {
        gyro_sum_cnt++;
        sum_temp[G_X] += mems.Gyro_I16.x;
        sum_temp[G_Y] += mems.Gyro_I16.y;
        sum_temp[G_Z] += mems.Gyro_I16.z;
        if( gyro_sum_cnt >= OFFSET_AV_NUM )
        {
					if(vmc_all.param.cal_flag[1]==0&&vmc_all.param.cal_flag[0]==0){
            mems.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
            mems.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
            mems.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
					}
            gyro_sum_cnt =0;
					if(mems.Gyro_CALIBRATE == 1&&module.flash)
					{
						WRITE_PARM();
					}  
            mems.Gyro_CALIBRATE = 0;
            sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
        }
    }
}

void Transform(float itx,float ity,float itz,float *it_x,float *it_y,float *it_z)
{
    *it_x = itx;
    *it_y = ity;
    *it_z = itz;
}

s16 FILT_BUF[ITEMS][(FILTER_NUM + 1)];
uint8_t filter_cnt = 0,filter_cnt_old = 0;

float mems_tmp[ITEMS];
float mpu_fil_tmp[ITEMS];
float test_ang =0,test_ang_old=0,test_ang_d,test_fli_a,test_i;

void IMU_Data_Prepare(float T)
{
    u8 i;
    s32 FILT_TMP[ITEMS] = {0,0,0,0,0,0,0};
    float Gyro_tmp[3];
    MEMS_Data_Offset(); //校准函数
		mems.Acc_I16.x=lis3mdl.Acc_I16.x ;
		mems.Acc_I16.y=lis3mdl.Acc_I16.y ;
		mems.Acc_I16.z=lis3mdl.Acc_I16.z ;
		mems.Gyro_I16.x=lis3mdl.Gyro_I16.x ;
		mems.Gyro_I16.y=lis3mdl.Gyro_I16.y ;
		mems.Gyro_I16.z=lis3mdl.Gyro_I16.z ;
    Gyro_tmp[0] = mems.Gyro_I16.x ;//
    Gyro_tmp[1] = mems.Gyro_I16.y ;//
    Gyro_tmp[2] = mems.Gyro_I16.z ;//
  
//======================================================================
    if( ++filter_cnt > FILTER_NUM )
    {
        filter_cnt = 0;
        filter_cnt_old = 1;
    }
    else
    {
        filter_cnt_old = (filter_cnt == FILTER_NUM)? 0 : (filter_cnt + 1);
    }
//10 170 4056
		if(fabs(mems.Off_3d.x)>10||fabs(mems.Off_3d.y)>10||fabs(mems.Off_3d.z)>10)
			mems.Cali_3d=1;
		int en_off_3d_off=0;
    /* 得出校准后的数据 */
		if(mems.Cali_3d){
				mems_tmp[A_X] = (mems.Acc_I16.x - mems.Off_3d.x)*mems.Gain_3d.x - mems.Acc_Offset.x*en_off_3d_off;
				mems_tmp[A_Y] = (mems.Acc_I16.y - mems.Off_3d.y)*mems.Gain_3d.y - mems.Acc_Offset.y*en_off_3d_off;
				mems_tmp[A_Z] = (mems.Acc_I16.z - mems.Off_3d.z)*mems.Gain_3d.z - mems.Acc_Offset.z*en_off_3d_off;
		}
		else{	 

				mems_tmp[A_X] = (mems.Acc_I16.x - mems.Acc_Offset.x) ;
				mems_tmp[A_Y] = (mems.Acc_I16.y - mems.Acc_Offset.y) ;
				mems_tmp[A_Z] = (mems.Acc_I16.z - mems.Acc_Offset.z) ;
		}
    mems_tmp[G_X] = Gyro_tmp[0] - mems.Gyro_Offset.x ;//
    mems_tmp[G_Y] = Gyro_tmp[1] - mems.Gyro_Offset.y ;//
    mems_tmp[G_Z] = Gyro_tmp[2] - mems.Gyro_Offset.z ;//


    /* 更新滤波滑动窗口数组 */
    FILT_BUF[A_X][filter_cnt] = mems_tmp[A_X];
    FILT_BUF[A_Y][filter_cnt] = mems_tmp[A_Y];
    FILT_BUF[A_Z][filter_cnt] = mems_tmp[A_Z];
    FILT_BUF[G_X][filter_cnt] = mems_tmp[G_X];
    FILT_BUF[G_Y][filter_cnt] = mems_tmp[G_Y];
    FILT_BUF[G_Z][filter_cnt] = mems_tmp[G_Z];

    for(i=0; i<FILTER_NUM; i++)
    {
        FILT_TMP[A_X] += FILT_BUF[A_X][i];
        FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
        FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
        FILT_TMP[G_X] += FILT_BUF[G_X][i];
        FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
        FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
    }

    mpu_fil_tmp[A_X] = (float)( FILT_TMP[A_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Y] = (float)( FILT_TMP[A_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[A_Z] = (float)( FILT_TMP[A_Z] )/(float)FILTER_NUM;


    mpu_fil_tmp[G_X] = (float)( FILT_TMP[G_X] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Y] = (float)( FILT_TMP[G_Y] )/(float)FILTER_NUM;
    mpu_fil_tmp[G_Z] = (float)( FILT_TMP[G_Z] )/(float)FILTER_NUM;


    /*坐标转换*/
    Transform(mpu_fil_tmp[A_X],mpu_fil_tmp[A_Y],mpu_fil_tmp[A_Z],&mems.Acc.x,&mems.Acc.y,&mems.Acc.z);
    Transform(mpu_fil_tmp[G_X],mpu_fil_tmp[G_Y],mpu_fil_tmp[G_Z],&mems.Gyro.x,&mems.Gyro.y,&mems.Gyro.z);

    mems.Gyro_deg.x = mems.Gyro.x *TO_ANGLE;
    mems.Gyro_deg.y = mems.Gyro.y *TO_ANGLE;
    mems.Gyro_deg.z = mems.Gyro.z *TO_ANGLE;
		
		LIS_CalOffset_Mag(T);
		float temp_mag[3];
		Low_Fass_Filter(lis3mdl.Mag_Adc.x,&temp_mag[0],FLT_MAG,T) ;
		Low_Fass_Filter(lis3mdl.Mag_Adc.y,&temp_mag[1],FLT_MAG,T) ;
		Low_Fass_Filter(lis3mdl.Mag_Adc.z,&temp_mag[2],FLT_MAG,T) ;
		mems.Mag_Adc.x=temp_mag[0];mems.Mag_Adc.y=temp_mag[1];mems.Mag_Adc.z=temp_mag[2];
		
		mems.Mag_Val.x=(mems.Mag_Adc.x - mems.Mag_Offset.x)*mems.Mag_Gain.x;
		mems.Mag_Val.y=(mems.Mag_Adc.y - mems.Mag_Offset.y)*mems.Mag_Gain.y;
		mems.Mag_Val.z=(mems.Mag_Adc.z - mems.Mag_Offset.z)*mems.Mag_Gain.z;
	
	  mems.Mag.x = mems.Mag_Val.x;
    mems.Mag.y = mems.Mag_Val.y;
    mems.Mag.z = mems.Mag_Val.z;
		
}

#define MAG_CAL_TIME 15//s
int MAX_HML=2000;
int mag_outer[3];
Cal_Cycle_OLDX hml_lsq1;
void LIS_CalOffset_Mag(float dt)
{ static xyz_f_t	Mag_Reg;
	static xyz_f_t	MagMAX = { -100 , -100 , -100 }, MagMIN = { 100 , 100 , 100 }, MagSum;
	static xyz_f_t	MagMAXo = { -100 , -100 , -100 }, MagMINo = { 100 , 100 , 100 }, MagSumo;
	static u8 hml_cal_temp=0;
  static u8 state_cal_hml;
	static u8 init;
	static float cnt1,cnt_m;
	if(mems.Mag_CALIBRATE)
	{	
		if(!init){init=1;
		#if USE_CYCLE_HML_CAL	
				cycle_init_oldx(&hml_lsq)	;
			if(module.hml_imu_o)
				cycle_init_oldx(&hml_lsq1)	;
		#endif
		MagMAX.x=MagMAX.y=MagMAX.z=-100;MagMIN.x=MagMIN.y=MagMIN.z=100;	
		MagMAXo.x=MagMAXo.y=MagMAXo.z=-100;MagMINo.x=MagMINo.y=MagMINo.z=100;				
		}
		
		float norm=sqrt(mems.Gyro_deg.x*mems.Gyro_deg.x+mems.Gyro_deg.y*mems.Gyro_deg.y+mems.Gyro_deg.z*mems.Gyro_deg.z);		
		if(norm<11)
			cnt1+=dt;
		if(cnt1>5){cnt1=0;
		mems.Mag_CALIBRATE=0;}

		if(ABS(mems.Mag_Adc.x)<MAX_HML&&ABS(mems.Mag_Adc.y)<MAX_HML&&ABS(mems.Mag_Adc.z)<MAX_HML&&norm>11)
		{ cnt1=0;
			#if USE_CYCLE_HML_CAL
			if(hml_lsq.size<350&&(fabs(Mag_Reg.x-mems.Mag_Adc.x)>25||fabs(Mag_Reg.y-mems.Mag_Adc.y)>25||fabs(Mag_Reg.z-mems.Mag_Adc.z)>25)){
				cycle_data_add_oldx(&hml_lsq, (float)mems.Mag_Adc.x/1000.,(float)mems.Mag_Adc.y/1000.,(float)mems.Mag_Adc.z/1000.);
			if(module.hml_imu_o)
				cycle_data_add_oldx(&hml_lsq1, (float)mems.Mag_Adc_o.x/1000.,(float)mems.Mag_Adc_o.y/1000.,(float)mems.Mag_Adc_o.z/1000.);
		  }
			#endif
			MagMAX.x = MAX(mems.Mag_Adc.x, MagMAX.x);
			MagMAX.y = MAX(mems.Mag_Adc.y, MagMAX.y);
			MagMAX.z = MAX(mems.Mag_Adc.z, MagMAX.z);
			
			MagMIN.x = MIN(mems.Mag_Adc.x, MagMIN.x);
			MagMIN.y = MIN(mems.Mag_Adc.y, MagMIN.y);
			MagMIN.z = MIN(mems.Mag_Adc.z, MagMIN.z);		
			
			MagMAXo.x = MAX(mems.Mag_Adc_o.x, MagMAXo.x);
			MagMAXo.y = MAX(mems.Mag_Adc_o.y, MagMAXo.y);
			MagMAXo.z = MAX(mems.Mag_Adc_o.z, MagMAXo.z);
			
			MagMINo.x = MIN(mems.Mag_Adc_o.x, MagMINo.x);
			MagMINo.y = MIN(mems.Mag_Adc_o.y, MagMINo.y);
			MagMINo.z = MIN(mems.Mag_Adc_o.z, MagMINo.z);	
			
			if(cnt_m >= MAG_CAL_TIME)
			{ float sphere_x,sphere_y,sphere_z,sphere_r;
				init=0;
				#if USE_CYCLE_HML_CAL
				cycle_cal_oldx(&hml_lsq, 855,0.001,  &sphere_x, &sphere_y, &sphere_z, &sphere_r);
				if(fabs(sphere_r)>0){
				mems.Mag_Offset_c.x=(hml_lsq.Off[0]*1000);
				mems.Mag_Offset_c.y=(hml_lsq.Off[1]*1000);
				mems.Mag_Offset_c.z=(hml_lsq.Off[2]*1000);
				mems.Mag_Gain_c.x =  (hml_lsq.Gain[0]);
				mems.Mag_Gain_c.y =  (hml_lsq.Gain[1]);
				mems.Mag_Gain_c.z =  (hml_lsq.Gain[2]);	
				}
				cycle_cal_oldx(&hml_lsq1, 855,0.001,  &sphere_x, &sphere_y, &sphere_z, &sphere_r);
				if(fabs(sphere_r)>0){
				mems.Mag_Offset_co.x=(hml_lsq1.Off[0]*1000);
				mems.Mag_Offset_co.y=(hml_lsq1.Off[1]*1000);
				mems.Mag_Offset_co.z=(hml_lsq1.Off[2]*1000);
				mems.Mag_Gain_co.x =  (hml_lsq1.Gain[0]);
				mems.Mag_Gain_co.y =  (hml_lsq1.Gain[1]);
				mems.Mag_Gain_co.z =  (hml_lsq1.Gain[2]);	
				}
				#endif
				mems.Mag_Offset.x = (int16_t)((MagMAX.x + MagMIN.x) * 0.5f);
				mems.Mag_Offset.y = (int16_t)((MagMAX.y + MagMIN.y) * 0.5f);
				mems.Mag_Offset.z = (int16_t)((MagMAX.z + MagMIN.z) * 0.5f);
	      mems.Mag_Offseto.x = (int16_t)((MagMAXo.x + MagMINo.x) * 0.5f);
				mems.Mag_Offseto.y = (int16_t)((MagMAXo.y + MagMINo.y) * 0.5f);
				mems.Mag_Offseto.z = (int16_t)((MagMAXo.z + MagMINo.z) * 0.5f);
				MagSum.x = MagMAX.x - MagMIN.x;
				MagSum.y = MagMAX.y - MagMIN.y;
				MagSum.z = MagMAX.z - MagMIN.z;
				MagSumo.x = MagMAXo.x - MagMINo.x;
				MagSumo.y = MagMAXo.y - MagMINo.y;
				MagSumo.z = MagMAXo.z - MagMINo.z;
				float temp_max=MagSum.x ;
				if( MagSum.y>temp_max)
					temp_max=MagSum.y;
			  if( MagSum.z>temp_max)
					temp_max=MagSum.z;
				
				mems.Mag_Gain.x =  temp_max/MagSum.x ;
				mems.Mag_Gain.y =  temp_max/MagSum.y ;
				mems.Mag_Gain.z =  temp_max/MagSum.z ;
				temp_max=MagSumo.x ;
				if( MagSumo.y>temp_max)
					temp_max=MagSumo.y;
			  if( MagSumo.z>temp_max)
					temp_max=MagSumo.z;
				
				mems.Mag_Gaino.x =  temp_max/MagSumo.x ;
				mems.Mag_Gaino.y =  temp_max/MagSumo.y ;
				mems.Mag_Gaino.z =  temp_max/MagSumo.z ;
				#if USE_CYCLE_HML_CAL
				if(mems.Mag_Gain_c.x>0&&mems.Mag_Gain_c.y>0&&mems.Mag_Gain_c.z>0
					&&mems.Mag_Gain_c.x<1.5&&mems.Mag_Gain_c.y<1.5&&mems.Mag_Gain_c.z<1.5&&
					fabs(mems.Mag_Offset_c.x)<1500&&fabs(mems.Mag_Offset_c.y)<1500&&fabs(mems.Mag_Offset_c.z)<1500){
				mems.Mag_Gain.x =  mems.Mag_Gain_c.x;
				mems.Mag_Gain.y =  mems.Mag_Gain_c.y;
				mems.Mag_Gain.z =  mems.Mag_Gain_c.z;
				mems.Mag_Offset.x =  mems.Mag_Offset_c.x;
				mems.Mag_Offset.y =  mems.Mag_Offset_c.y;
				mems.Mag_Offset.z =  mems.Mag_Offset_c.z;
					}
				if(mems.Mag_Gain_co.x>0&&mems.Mag_Gain_co.y>0&&mems.Mag_Gain_co.z>0
					&&mems.Mag_Gain_co.x<1.5&&mems.Mag_Gain_co.y<1.5&&mems.Mag_Gain_co.z<1.5&&
					fabs(mems.Mag_Offset_co.x)<1500&&fabs(mems.Mag_Offset_co.y)<1500&&fabs(mems.Mag_Offset_co.z)<1500){
				mems.Mag_Gaino.x =  mems.Mag_Gain_co.x;
				mems.Mag_Gaino.y =  mems.Mag_Gain_co.y;
				mems.Mag_Gaino.z =  mems.Mag_Gain_co.z;
				mems.Mag_Offseto.x =  mems.Mag_Offset_co.x;
				mems.Mag_Offseto.y =  mems.Mag_Offset_co.y;
				mems.Mag_Offseto.z =  mems.Mag_Offset_co.z;
				}
				#endif
			  WRITE_PARM();
				cnt_m = 0;
				mems.Mag_CALIBRATE = 0;
			}
			Mag_Reg.x=mems.Mag_Adc.x;
			Mag_Reg.y=mems.Mag_Adc.y;
			Mag_Reg.z=mems.Mag_Adc.z;		
		}
		cnt_m+=dt;	
	}
}