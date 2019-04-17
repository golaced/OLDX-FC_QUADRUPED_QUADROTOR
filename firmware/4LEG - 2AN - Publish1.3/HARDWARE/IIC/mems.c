#include "include.h"
#include "mems.h"
#include "imu.h"
#include "vmc.h"
#include "icm20602.h"
#include "cycle_cal_oldx.h"
u8 acc_3d_step;
_MEMS mems;

void IMU_Read(void)
{
	#if defined(ICM20602)
	 Icm20602_Read();
	#else
   LSM6_readAcc(1);LSM6_readGyro(1);
	#endif
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
        sum_temp[TEM] += mems.Tempreature;

        if( acc_sum_cnt >= OFFSET_AV_NUM )
        {   
					if(vmc_all.param.cal_flag[1]==0&&vmc_all.param.cal_flag[0]==0){
            mems.Acc_Offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
            mems.Acc_Offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
            mems.Acc_Offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
					  mems.att_off[0]=(float)sum_temp_att[0]/OFFSET_AV_NUM;
					  mems.att_off[1]=(float)sum_temp_att[1]/OFFSET_AV_NUM;
            mems.Acc_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
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
        sum_temp[TEM] += mems.Tempreature;

        if( gyro_sum_cnt >= OFFSET_AV_NUM )
        {
					if(vmc_all.param.cal_flag[1]==0&&vmc_all.param.cal_flag[0]==0){
            mems.Gyro_Offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
            mems.Gyro_Offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
            mems.Gyro_Offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
            mems.Gyro_Temprea_Offset = sum_temp[TEM]/OFFSET_AV_NUM;
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
		#if defined(ICM20602)
		mems.Acc_I16.x =(s16)((((u16)mpu_buffer[0]) << 8) | mpu_buffer[1]);//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
		mems.Acc_I16.y =(s16)((((u16)mpu_buffer[2]) << 8) | mpu_buffer[3]);//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
		mems.Acc_I16.z =(s16)((((u16)mpu_buffer[4]) << 8) | mpu_buffer[5]);//>>1;// + 4 *sensor.Tempreature_C;// + 7 *sensor.Tempreature_C;

		mems.Gyro_I16.x=(s16)((((u16)mpu_buffer[ 8]) << 8) | mpu_buffer[ 9]) ;
		mems.Gyro_I16.y=(s16)((((u16)mpu_buffer[10]) << 8) | mpu_buffer[11]) ;
		mems.Gyro_I16.z=(s16)((((u16)mpu_buffer[12]) << 8) | mpu_buffer[13]) ;
		#else
		mems.Acc_I16.x=lis3mdl.Acc_I16.x ;
		mems.Acc_I16.y=lis3mdl.Acc_I16.y ;
		mems.Acc_I16.z=lis3mdl.Acc_I16.z ;
		mems.Gyro_I16.x=lis3mdl.Gyro_I16.x ;
		mems.Gyro_I16.y=lis3mdl.Gyro_I16.y ;
		mems.Gyro_I16.z=lis3mdl.Gyro_I16.z ;
		#endif
    Gyro_tmp[0] = mems.Gyro_I16.x ;//
    Gyro_tmp[1] = mems.Gyro_I16.y ;//
    Gyro_tmp[2] = mems.Gyro_I16.z ;//
   // mems.Tempreature
    mems.TEM_LPF += 2 *3.14f *T *(mems.Tempreature - mems.TEM_LPF);
    mems.Ftempreature = mems.TEM_LPF/340.0f + 36.5f;

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
}
