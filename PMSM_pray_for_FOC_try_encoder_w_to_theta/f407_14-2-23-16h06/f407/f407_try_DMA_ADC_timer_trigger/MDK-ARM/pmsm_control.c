#include "main.h"
#include "pmsm_control.h"

float spd_err = 0, actual_current = 0 , spd_err_next = 0 ;
float Q_err = 0  , Q_actual = 0 , Q_err_next = 0 ;
float D_err = 0 , D_actual = 0 , D_err_next = 0  ;
#define speed_limit 5
#define current_limit 12
#define current_limit_Q 12

float sinlkrd(float In_Sin_rad)	//  Remove temp out -needed for ... limit
{	// OPtimised
	float Out_sin;
	Out_sin =  sin_tab[((int)(In_Sin_rad*572.9577951))]; //  572.9577951 = 180/pi *10 (10 is sin_tab table)
	return Out_sin;
}

float coslkrd(float In_Cos_rad)	//  Remove temp out -needed for ... limit
{	// OPtimised
	float Out_sin;
	Out_sin =  cos_tab[((int)(In_Cos_rad*572.9577951))]; //  572.9577951 = 180/pi *10 (10 is sin_tab table)
	return Out_sin;
}
float current_calc( uint16_t ADC_value , float ADC_offset) {
			float current;
			current = (ADC_value - ADC_offset)*sensor_gain ;
	return current ;
}

float PI_speed (float temp_spd ) {
			spd_err = SPEED_REF - temp_spd ;  //SPEED_REF
			actual_current += spd_KP*(spd_err - spd_err_next ) + spd_KI_Ts*spd_err ;
			spd_err_next = spd_err ;
	if (actual_current >= speed_limit ) {actual_current = speed_limit;}
	if (actual_current < -speed_limit ) {actual_current = -speed_limit  ;}
	return actual_current ;  // is Q current
}


float PI_Q (float temp_Q, float ref_Q) {
			Q_err = ref_Q - temp_Q  ;
			Q_actual += Q_KP*(Q_err - Q_err_next ) + Q_KI_Ts*Q_err ;
		 Q_err_next = Q_err ;
	if (Q_actual >= current_limit_Q  ) {Q_actual = current_limit_Q ;}
	if (Q_actual < -current_limit_Q ) {Q_actual = -current_limit_Q  ;}
	return Q_actual ;
}

float PI_D (float temp_D) {
			D_err =  0 - temp_D  ; // 0 - temp_D  or temp_D
			D_actual += D_KP*(D_err - D_err_next ) + D_KI_Ts*D_err ;
		D_err_next = D_err ;
	if ( D_actual >= current_limit ) { D_actual = current_limit ;}
	if ( D_actual < -current_limit ) { D_actual = -current_limit ;}
	return D_actual ;
}

void abc_to_alp_bet(float a_in, float b_in,float c_in, float *al_out, float *be_out)
{
(*al_out)=(0.6666666667)*(a_in -0.5*(b_in +c_in)); //2/3= 0.66667
(*be_out)=(0.6666666667)*SQRT3div2*(b_in -c_in);
}

//float current_calc( uint16_t ADC_value ) {
//	float U_sensor = ADC_value*3.3/4095 ;
//	float U_res = (U_sensor - 1.24 )/8 ;
//	float I_return = U_res/ 0.02 ;
//	return I_return ;
//}
//	if ( mode_dac == 1 ) {
//				HAL_DAC_SetValue (&hdac , DAC_CHANNEL_1 ,DAC_ALIGN_12B_R, adcVal[PHA1] ) ; 
//			HAL_DAC_SetValue (&hdac , DAC_CHANNEL_2 ,DAC_ALIGN_12B_R, adcVal[PHA2] ) ; 
//		}
//		if (mode_dac ==2 ) {
//			 HAL_DAC_SetValue (&hdac , DAC_CHANNEL_1 ,DAC_ALIGN_12B_R,  adcVal[PHA1] ) ;
//				HAL_DAC_SetValue (&hdac , DAC_CHANNEL_2 ,DAC_ALIGN_12B_R, adcVal[PHA3] ) ; 			
//		}
//		if (mode_dac == 3) {
//			 HAL_DAC_SetValue (&hdac , DAC_CHANNEL_1 ,DAC_ALIGN_12B_R,  adcVal[PHA2] ) ; 
//			HAL_DAC_SetValue (&hdac , DAC_CHANNEL_2 ,DAC_ALIGN_12B_R, adcVal[PHA3] ) ; 
//		}

//U_alpha = 0.24*SINE_TABLE[alpha_inx] ;   //0.3= = UDC / 100 = max sin table
	//U_beta  = 0.24*SINE_TABLE[beta_inx] ;
	//U_alpha = 400+ 2*SINE_TABLE[alpha_inx] ;
	//U_beta  = 400+ 2*SINE_TABLE[beta_inx] ;
	
//			Ia = adcVal[PHA1]*0.03663 ;  //  adc/4096*3.3V/0.02ohm
//			Ib = adcVal[PHA2]*0.03663 ;
//			Ic = adcVal[PHA3]*0.03663 ;	

//float ccoss(float In_Cos_rad)	//  Remove temp out -needed for ... limit
//{	// OPtimised
//	float Out_cos; int In_Cos_Deg;
//	In_Cos_Deg =  (int)(900 + In_Cos_rad*572.9577951);
//	if( In_Cos_Deg<=3600)	{	
//	Out_cos =  sin_tab[(In_Cos_Deg)] ;
//	}
//	else {// In_Cos_Deg>3600
//	In_Cos_Deg = (In_Cos_Deg-3600);
//	Out_cos =  sin_tab[In_Cos_Deg];} //  572.9577951 = 180/pi *10 (10 is sin_tab table)
//	return Out_cos;
//}

//float ssin(float In_Sin_rad)	//  Remove temp out -needed for ... limit
//{	// OPtimised
//	float Out_sin;
//	Out_sin =  sin_tab[((int)(In_Sin_rad*572.9577951))]; //  572.9577951 = 180/pi *10 (10 is sin_tab table)
//	return Out_sin;
//}

/*
mech_w = 6283/period ;
if (direct == FORWARD )	{ temp_w  = 4*mech_w ; }
else if (direct == BACK ) { temp_w  = -4*mech_w ;}
if (w_count >= SUM_ELEC_W_NUM ) 
						{	ave_elec_w = sum_elec_w / SUM_ELEC_W_NUM; 
							w_count = 1 ;
						  sum_elec_w = ave_elec_w  ;}
else if (w_count < SUM_ELEC_W_NUM ) 
					  {	w_count ++ ;
							sum_elec_w += temp_w ;
						}  */
