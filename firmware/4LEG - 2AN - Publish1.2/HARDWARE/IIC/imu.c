
#include "imu.h"
#include "include.h"
#include "my_math.h"
#include "filter.h"
u8 fly_ready;
xyz_f_t reference_v;
ref_t 	ref;
float reference_vr[3];
float Roll,Pitch,Yaw;    				//вкл╛╫г
float q0=1,q1,q2,q3;
float ref_q[4] = {1,0,0,0};
float norm_acc,norm_q;
float norm_acc_lpf;
xyz_f_t mag_sim_3d;
extern u8 fly_ready;

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
float Kp_yaw=0.5*2;
volatile float beta_start=6;
volatile float beta_end=0.3;
volatile float beta_step=0.01;
volatile float beta;
float accConfidenceDecay = 1.52f;
float hmlConfidenceDecay = 4.0;
float accConfidence      = 1.0f; 
float hmlConfidence 			= 1.0f; 
float norm_hml_view;
#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)
#define accelOneG 9.8
void calculateAccConfidence(float accMag_in)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float accMagP = 1.0f;
  float accMag=accMag_in;
	accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's

	accMagP  = HardFilter(accMagP, accMag );

	accConfidence=1-((accConfidenceDecay * sqrt(fabs(accMagP - 1.0f))));
  if(accConfidence>1)
		accConfidence=1;
	if(accConfidence<0)
		accConfidence=0;
}

float hmlOneMAG= 200.0;
void calculateHmlConfidence(float Mag_in)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity

	static float MagP = 1.0f;
  float Mag;
  Mag  = Mag_in/hmlOneMAG;  // HJI Added to convert MPS^2 to G's

	MagP  = HardFilter(MagP, Mag );

	hmlConfidence=2-((hmlConfidenceDecay*sqrt(fabs(MagP - 1.0f))));
  if(hmlConfidence>1)
		hmlConfidence=1;
	if(hmlConfidence<0)
		hmlConfidence=0;
}
// *  smpl_frq    sampling frequency of AHRS data
// *  b_start     algorithm gain starting value
// *  b_end       algorithm gain end value
// *  b_step      algorithm gain decrement step size

void mag_fail_check(float dt)
{ static u8 state;
	static float timer,timer_err;
  switch(state)
	{
		case 0:
			 if(fabs(mems.hmlOneMAG-hmlOneMAG)>0.2*hmlOneMAG)
					timer+=dt;
			 else
				  timer=0;
		   if(timer>0.68)
			 {state=1;timer=timer_err=0;mems.Mag_ERR=1;}
			 break;
		case 1:
			 if(fabs(mems.hmlOneMAG-hmlOneMAG)<0.35*hmlOneMAG)
					timer+=dt;
			 else
				  timer=0;
			 
			 timer_err+=dt;
		   if(timer>2||timer_err>10)
			 {state=0;timer=timer_err=0;mems.Mag_ERR=0;}
		break;
	}
}

u8 init_hml_norm,mag_cal_use_compass=1;
u16 init_hml_norm_cnt;

void Q_state_init(float ax, float ay, float az, float mx, float my, float mz)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(-ay, -az);
    initialPitch = atan2(ax, -az);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

    magY = my * cosRoll - mz * sinRoll;

    initialHdg = atan2f(-magY, magX);

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
		
		ref_q[0]=q0;
		ref_q[1]=q1;
		ref_q[2]=q2;
		ref_q[3]=q3;

		float rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
		float pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
		float yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;//  
		reference_vr[0] = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
		reference_vr[1] = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
		reference_vr[2] = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);
}

int madgwick_update_new(float T,float wx, float wy, float wz, float ax, float ay, float az,float mx,float my,float mz,float *rol,float *pit,float *yaw) 							
{
    static u8 init,imu_init;
	  static float init_mag_cnt=0,yaw_correct_flt;
    float recip_norm,norm;
    float s0, s1, s2, s3;
    float dq1, dq2, dq3, dq4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float yaw_correct=0,beta_use;
		if(mems.Mag_CALIBRATE)
			init=0;
		if(!init){init=1;
				beta = beta_start;
			if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
			  Q_state_init(ax,ay,az,mx,my,mz);
		}
		
    /* Check for valid magnetometer data. */
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    ;
			else if(!init_hml_norm&&init_hml_norm_cnt++>200){init_hml_norm=1;			
				hmlOneMAG = sqrt(mx*(mx) + my*(my) + mz*mz)*1.1;
		}
		mems.hmlOneACC= norm = sqrt(ax*(ax) + ay*(ay) + az*az)/4096.*9.8;	
		mems.hmlOneMAG= norm_hml_view = sqrt(mx*(mx) + my*(my) + mz*mz);	
		if(mems.Mag_ERR||mag_cal_use_compass)
			mx=my=mz=0;
		else
			Low_Fass_Filter(mems.hmlOneMAG,&hmlOneMAG,0.01,T);
		
    /* Check if beta has reached its specified end value or if it has to be
     * decremented. */
    if (beta > beta_end)
    {
        /* Decrement beta only if it does not fall below the specified end
         * value. */
        if ((beta - beta_step) > beta_end)
        {
            beta -= beta_step;
        }
        else
        {
            beta = beta_end;
        }
    }else beta=beta_end;

					
    /* Calculate quaternion rate of change of from angular velocity. */
    dq1 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
    dq2 = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
    dq3 = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
    dq4 = 0.5f * (q0 * wz + q1 * wy - q2 * wx);

    /* Calculate feedback only if accelerometer measurement is valid. This
     * prevents NaNs in acceleration normalization. */
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {   		  
				calculateAccConfidence(norm);
			  calculateHmlConfidence(norm_hml_view);
			  mag_fail_check(T);//check mag fail
			
			if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
				beta_use = beta*accConfidence ;
			else
				beta_use = beta*accConfidence * hmlConfidence;
        /* Normalize accelerometer measurement. */
        recip_norm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        /* Normalize magnetometer measurement. */
        recip_norm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;
				
        /* Auxiliary variables to avoid repeated arithmetic and therefore
         * improve performance. */
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        /* Reference direction of earth magnetic field. */
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        /* Gradient decent algorithm corrective step. */
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        /* Normalize step magnitude. */
        recip_norm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        /* Apply feedback step. */
        dq1 -= beta_use * s0;
        dq2 -= beta_use * s1;
        dq3 -= beta_use * s2;
        dq4 -= beta_use * s3;
    }
		
    /* Integrate quaternion rate of change to get quaternion describing the
     * current orientation. */
    q0 += dq1 * T;
    q1 += dq2 * T;
    q2 += dq3 * T;
    q3 += dq4 * T;

		if(imu_init==0)
			init_mag_cnt+=T;
		if( mems.Mag_update && mems.Mag_ERR==0&&mag_cal_use_compass)
		{   
				int magTmp2[3]; 
				float euler[2];
			  mems.Mag_update=0;
				magTmp2[0]=mems.Mag_Val.y;
				magTmp2[1]=-mems.Mag_Val.x;
				magTmp2[2]=mems.Mag_Val.z;
				euler[0]= Roll /57.3  ;
				euler[1]=-Pitch/57.3  ;
				float calMagY = magTmp2[0] * cos(euler[1]) + magTmp2[1] * sin(euler[1])* sin(euler[0])+magTmp2[2] * sin(euler[1]) * cos(euler[0]); 
				float calMagX = magTmp2[1] * cos(euler[0]) + magTmp2[2] * sin(euler[0]);
				float wxf,wyf,wzf;
				if( (fabs(Roll)<30 && fabs(Pitch)<30))
				mems.Yaw_Mag=To_180_degrees(fast_atan2(calMagX,calMagY)* 57.3 );
 				
					if(init_mag_cnt<5)
					{ 
						yaw_correct = Kp_yaw *100 *LIMIT( To_180_degrees(mems.Yaw_Mag - Yaw),-45,45 );
					}
					else
					{ imu_init=1;
						yaw_correct = Kp_yaw *1.5f *To_180_degrees(mems.Yaw_Mag - Yaw);
					}
					Low_Fass_Filter(yaw_correct,&yaw_correct_flt,1,T);
					wxf = (wx - reference_vr[0] *yaw_correct_flt) *ANGLE_TO_RADIAN;    
					wyf = (wy - reference_vr[1] *yaw_correct_flt) *ANGLE_TO_RADIAN;		
					wzf = (wz - reference_vr[2] *yaw_correct_flt) *ANGLE_TO_RADIAN;
					dq1 = 0.5f * (-q1 * wxf - q2 * wyf - q3 * wzf);
					dq2 = 0.5f * (q0 * wxf + q2 * wzf - q3 * wyf);
					dq3 = 0.5f * (q0 * wyf - q1 * wzf + q3 * wxf);
					dq4 = 0.5f * (q0 * wzf + q1 * wyf - q2 * wxf);
					q0 += dq1 * T;
					q1 += dq2 * T;
					q2 += dq3 * T;
					q3 += dq4 * T;
		}

    /* Normalize quaternion. */
    recip_norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;

	ref_q[0]=q0;
	ref_q[1]=q1;
	ref_q[2]=q2;
	ref_q[3]=q3;
	reference_vr[0] = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_vr[1] = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_vr[2] = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f  ;// 
   if ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)&&mag_cal_use_compass==0)
		*yaw = To_180_degrees(*yaw-90);
    return 1;
}