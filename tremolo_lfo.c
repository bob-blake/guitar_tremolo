// tremolo_lfo.c
// PIC code to run low-frequency oscillator for guitar tremolo pedal
// Bob Blake
// 6/15/2010

#include "\tremolo_lfo.h"

/*********************************************
/  Declare pin defines
/*********************************************/
#define	PIN_SPEED_CTRL	PIN_C0
#define	PIN_DUTY_CTRL	PIN_C1
#define PIN_MULT_CTRL	PIN_C2
#define PIN_DEPTH_CTRL	PIN_C3
#define PIN_SHAPE_CTRL	PIN_C4	//Damn, need to change to an analog pin
#define PIN_TEMPO		PIN_A2

/*********************************************
/  Declare global variables
/*********************************************/

int1	calc_next_pwm=0, time_to_read_adcs=0;


//Half sine wave lookup table
//Actually a cosine table because it starts
//with max value.
static const unsigned int8 halfsine_table[256] = {
255, 255, 255, 255, 255, 255, 255, 255, 
254, 254, 254, 254, 254, 253, 253, 253, 
253, 252, 252, 252, 251, 251, 250, 250, 
249, 249, 249, 248, 248, 247, 246, 246, 
245, 245, 244, 243, 243, 242, 241, 241, 
240, 239, 238, 238, 237, 236, 235, 234, 
233, 233, 232, 231, 230, 229, 228, 227, 
226, 225, 224, 223, 222, 221, 220, 219, 
218, 216, 215, 214, 213, 212, 211, 209, 
208, 207, 206, 205, 203, 202, 201, 199, 
198, 197, 195, 194, 193, 191, 190, 189, 
187, 186, 185, 183, 182, 180, 179, 177, 
176, 175, 173, 172, 170, 169, 167, 166, 
164, 163, 161, 160, 158, 157, 155, 154, 
152, 150, 149, 147, 146, 144, 143, 141, 
140, 138, 137, 135, 133, 132, 130, 129, 
127, 126, 124, 122, 121, 119, 118, 116, 
115, 113, 111, 110, 108, 107, 105, 104, 
102, 101, 99, 98, 96, 95, 93, 92, 
90, 89, 87, 86, 84, 83, 81, 80, 
78, 77, 75, 74, 73, 71, 70, 68, 
67, 66, 64, 63, 62, 60, 59, 58, 
56, 55, 54, 52, 51, 50, 49, 47, 
46, 45, 44, 43, 41, 40, 39, 38, 
37, 36, 35, 34, 33, 32, 31, 30, 
29, 28, 27, 26, 25, 24, 23, 22, 
21, 20, 19, 19, 18, 17, 16, 15, 
15, 14, 13, 13, 12, 11, 11, 10, 
9, 9, 8, 8, 7, 7, 6, 6, 
5, 5, 4, 4, 4, 3, 3, 3, 
2, 2, 2, 2, 1, 1, 1, 1, 
1, 0, 0, 0, 0, 0, 0, 0 
};

//This is to convert from an 8-bit A/D value into a 16-bit freq_inc value
//Provides a logarithmic response to the FREQ control using a linear pot
static const unsigned int16 freq_table[256] = {
0x0016, 0x0016, 0x0016, 0x0017, 0x0017, 0x0018, 0x0018, 0x0019, 
0x001A, 0x001A, 0x001B, 0x001B, 0x001C, 0x001D, 0x001D, 0x001E, 
0x001E, 0x001F, 0x0020, 0x0020, 0x0021, 0x0022, 0x0023, 0x0023, 
0x0024, 0x0025, 0x0026, 0x0027, 0x0027, 0x0028, 0x0029, 0x002A, 
0x002B, 0x002C, 0x002D, 0x002E, 0x002F, 0x0030, 0x0031, 0x0032, 
0x0033, 0x0034, 0x0035, 0x0037, 0x0038, 0x0039, 0x003A, 0x003C, 
0x003D, 0x003E, 0x0040, 0x0041, 0x0042, 0x0044, 0x0045, 0x0047, 
0x0048, 0x004A, 0x004C, 0x004D, 0x004F, 0x0051, 0x0052, 0x0054, 
0x0056, 0x0058, 0x005A, 0x005C, 0x005E, 0x0060, 0x0062, 0x0064, 
0x0066, 0x0069, 0x006B, 0x006D, 0x0070, 0x0072, 0x0075, 0x0077, 
0x007A, 0x007C, 0x007F, 0x0082, 0x0085, 0x0088, 0x008B, 0x008E, 
0x0091, 0x0094, 0x0097, 0x009A, 0x009E, 0x00A1, 0x00A5, 0x00A8, 
0x00AC, 0x00B0, 0x00B4, 0x00B8, 0x00BC, 0x00C0, 0x00C4, 0x00C8, 
0x00CD, 0x00D1, 0x00D6, 0x00DA, 0x00DF, 0x00E4, 0x00E9, 0x00EE, 
0x00F3, 0x00F9, 0x00FE, 0x0104, 0x0109, 0x010F, 0x0115, 0x011B, 
0x0121, 0x0128, 0x012E, 0x0135, 0x013C, 0x0142, 0x014A, 0x0151, 
0x0158, 0x0160, 0x0167, 0x016F, 0x0177, 0x0180, 0x0188, 0x0190, 
0x0199, 0x01A2, 0x01AB, 0x01B5, 0x01BE, 0x01C8, 0x01D2, 0x01DC, 
0x01E7, 0x01F1, 0x01FC, 0x0207, 0x0213, 0x021E, 0x022A, 0x0236, 
0x0243, 0x024F, 0x025C, 0x026A, 0x0277, 0x0285, 0x0293, 0x02A2, 
0x02B0, 0x02BF, 0x02CF, 0x02DF, 0x02EF, 0x02FF, 0x0310, 0x0321, 
0x0333, 0x0344, 0x0357, 0x0369, 0x037D, 0x0390, 0x03A4, 0x03B9, 
0x03CD, 0x03E3, 0x03F8, 0x040F, 0x0425, 0x043D, 0x0454, 0x046D, 
0x0486, 0x049F, 0x04B9, 0x04D3, 0x04EE, 0x050A, 0x0526, 0x0543, 
0x0561, 0x057F, 0x059E, 0x05BD, 0x05DD, 0x05FE, 0x0620, 0x0642, 
0x0665, 0x0689, 0x06AE, 0x06D3, 0x06F9, 0x0720, 0x0748, 0x0771, 
0x079B, 0x07C5, 0x07F1, 0x081E, 0x084B, 0x0879, 0x08A9, 0x08DA, 
0x090B, 0x093E, 0x0972, 0x09A7, 0x09DD, 0x0A14, 0x0A4C, 0x0A86, 
0x0AC0, 0x0AFD, 0x0B3B, 0x0B7A, 0x0BBA, 0x0BFC, 0x0C3F, 0x0C84, 
0x0CCA, 0x0D12, 0x0D5B, 0x0DA6, 0x0DF2, 0x0E41, 0x0E91, 0x0EE2, 
0x0F36, 0x0F8B, 0x0FE2, 0x103B, 0x1096, 0x10F3, 0x1152, 0x11B3, 
0x1216, 0x127C, 0x12E3, 0x134D, 0x13B9, 0x1428, 0x1499, 0x150C	
};

#int_TIMER1
void  TIMER1_ISR(void)
{
	clear_interrupt(INT_TIMER1);
	time_to_read_adcs = 1;
}

// PWM timer interrupt
// Do calculations in the main loop to keep interrupt short
#int_TIMER2
void  TIMER2_isr(void) 
{ 
	clear_interrupt(INT_TIMER2);
	calc_next_pwm = 1;
}

#int_EXT
void  EXT_isr(void) 
{

}

#int_AD
void  AD_isr(void) 
{

}


#int_CCP1
void  CCP1_isr(void) 
{


}



void main()
{
	int8 	eightbit, lookup;
	int8	wavetype=4;
	int32 	phase=0; 
	int32 	freq_32,freq_inc_a,freq_inc_b;
	int16 	pwm_duty=0;
	
	int8	freq_cv=1, dist_cv=128;	// Placeholder for frequency and distortion potentiometers
									// Keep dist between 4 and 252 for best results.  128 is zero dist.
	int8	freq_mul=8;				
	int16	freq_16;
	
	
	setup_adc_ports(sAN4|sAN5|sAN6|sAN7|VSS_VDD);
	setup_adc(ADC_CLOCK_INTERNAL);
	setup_wdt(WDT_OFF);
	setup_timer_1(T1_INTERNAL|T1_DIV_BY_4);
	//setup_timer_2(T2_DIV_BY_4,127,1);	// PWM Freq. of 19.5 kHz //Prescaler of 127 limits pwm output to 8 bit
	setup_timer_2(T2_DIV_BY_1,255,1);	// PWM Freq. of 19.5 kHz
	setup_ccp1(CCP_PWM);
	set_pwm1_duty(0);

	enable_interrupts(INT_TIMER1);
	enable_interrupts(INT_TIMER2);
	//enable_interrupts(INT_EXT);
	//enable_interrupts(INT_AD);
	//enable_interrupts(INT_RDA);
	//enable_interrupts(INT_CCP1);
	enable_interrupts(GLOBAL);
	//set_pwm1_duty(duty);
   while(TRUE)
   {
		if(calc_next_pwm)
		{
			//phase = (phase + 430) % 16777216;	// 1 Hz
			if(bit_test(phase,23))							// 24-bit phase accumulator - check to see which half of waveform we're on
				phase = (phase + freq_inc_b) % 16777216;	// Limit phase to 24-bit
			else
				phase = (phase + freq_inc_a) % 16777216;	// Limit phase to 24-bit
						
			eightbit = make8((phase >> 15),0);	// MSB indicates which half of the wave, only take bits 16-23				
												// Eightbit is used to index into LUT
			switch(wavetype)
			{
				case 0:	// Sine
					lookup = halfsine_table[eightbit];
					if(bit_test(phase,23))				// If we're on second half of wave
						lookup ^= 255;					// Invert the lookup value using XOR
					pwm_duty = (int16) lookup * 4;		// Level control, set at max for now
				break;
				case 1: // Ramp up
					pwm_duty = (int16) eightbit * 4;	// Level control, set at max for now
				break;
				case 2: // Ramp down
					pwm_duty = (int16) (255 - eightbit) * 4;	// Level control, set at max for now
				break;
				case 3:	// Square
					if(bit_test(phase,23))				// If we're on second half of wave
						pwm_duty = 0;		
					else
						pwm_duty = (int16) 255 * 4;		// Level control, set at max for now
				break;
				case 4:	// Triangle
					if(bit_test(phase,23))				// If we're on second half of wave
						pwm_duty = (int16) eightbit * 4;	// Level control, set at max for now	
					else
						pwm_duty = (int16) (255 - eightbit) * 4;	// Level control, set at max for now
				break;
			}	
			
			set_pwm1_duty(pwm_duty);			// This doesn't take effect until PWM timer overflow
			calc_next_pwm = 0;
		}
		else if(time_to_read_adcs)
		{
			set_adc_channel(4);
			freq_cv = read_adc();
			set_adc_channel(5);
			dist_cv = read_adc();					// Duty
			freq_16 = freq_table[freq_cv] * 2;		// Gets screwy at high values of freq_cv (>~210)
			freq_16 *= freq_mul;
			freq_32 = (int32) freq_16 << 7;			// Multiply by 128
													// Dist seems to affect freq as well
			freq_inc_a = freq_32 / dist_cv;			// First half of wave, divide by dist value
			freq_inc_b = freq_32 / (255 - dist_cv);	// Second half of wave, divide by inverse
			time_to_read_adcs = 0;
		}	
   }
}
