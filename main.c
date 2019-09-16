//**********************************************************//
//*********BOOST VOLTAGE CONTROLLED DC-DC*******************//
//**********************************************************//

//include files
#include "p30f6010A.h"
#include "main.h"     // Include file containing processor registors definitions and function definitions
#include "asmMATH.h"  // Include definition of math functions 

//
//user variables 
int Vcount = 50; //
int Vtick = 0;

int VDC1 = 0;
int VDC2 = 0;
int VDC = 0;
int VDC_max = 920; //180V //maximum dc output trip limit

int PID_Isample = 0; //current PID
int PID_I_count =  4; //25Khz //4

int Ipv = 0;

int Vref = 46; //set output dc-link voltage ref
int Irtrip = 130; //6A //trip current limit


int IPreError = 0;

// PI gain parameters
int Vpgain = 36;
int Vigain = 9;
//

int Vpre_err = 0; //integrated error register

int Vpi_out = 0;

int PWM_max =  14000; //14208
int PWM_min = -14000;

int SAT = 1776;

//

int main(void)
{

init();                 // call processor initilisation code

PWMenable = 0;          //reset PWM control register
PWM_InterruptEnable = 0;

ADC_ON = 1; //enable adc
FAULT_ENABLE = 1; //0x000f; //reset fault register

PWM1 = 0; //reset PWM registers
PWM2 = 0;
PWM3 = 0;

T1ON = 1; //enable timers
PWMenable = 1; //enable PWM
PWM_InterruptEnable = 1; //enable interrupt

    while(1)
    {

//DC link voltage protection
if(Vtick >= Vcount)
	    {

	VDC = (VDC1+VDC2)>>1; //(Vdc1+Vdc2)%2

	if(VDC >= VDC_max) fault_Flag = 1; //check dclink peak

	Vtick = 0; //reset flag
		     }

if(PID_Isample >= PID_I_count) //Voltage PI sample
             {

    IPreError = Vpre_err; //copy old integrated error
    Vpi_out = asmPIcontroller(VDC,Vref,Vpgain,Vigain); //give dc-link ouptut voltage ref.
    Vpre_err = IPreError; //copy new integrated error

    if(Vpi_out >=  PWM_max) Vpi_out =  PWM_max;
    if(Vpi_out <=  PWM_min) Vpi_out =  PWM_min;

    Vpi_out = Vpi_out>>3;    //scale PI output
    Vpi_out = Vpi_out + SAT;

    asm("disi #0x3FFF");
    PWM1 = Vpi_out; //program PWM duty cycle 1
    asm("disi #0x0000");
    
    PID_Isample = 0; 
                     } //current sample end

                                  } //while end

                                      } //main end

//

////////////////////////////////////////////////////////////////////////////////

//T1 interrupt for program tracking
void _ISRFAST __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
  {     
     Vtick++; //update dc-link protection timer
     PID_Isample++; //update voltage PI timer
      
     T1us_Flag = 0; //reset flag
   } //T1 interupt end

///////////////////////////////////////////////////////////////////////

//PWM interrupt for MPPT
void _ISRFAST __attribute__((interrupt, no_auto_psv)) _PWMInterrupt(void)
  {
 /*read DC link voltage*/  
	asm("MOV #0x0303, W0");       //select adc channel VDC1
	asm("MOV W0, ADCHS");    
	asm("BSET ADCON1,#1");        //start converstion
	asm("LOOP1:");
	asm("BTSS ADCON1,#0");        //wait for converstion
	asm("BRA LOOP1");
	asm("BTG ADCON1,#0"); 
	asm("MOV ADCBUF0, W0");   
	asm("MOV W0, _VDC1");        //get VDC1

    asm("MOV #0x0202, W0");       //select adc channel VDC2
	asm("MOV W0, ADCHS");    
	asm("BSET ADCON1,#1");        //start converstion
	asm("LOOP2:");
	asm("BTSS ADCON1,#0");        //wait for converstion
	asm("BRA LOOP2");
	asm("BTG ADCON1,#0"); 
	asm("MOV ADCBUF0, W0");   
	asm("MOV W0, _VDC2");        //get VDC2

/*read converter input current for protection*/
    
	asm("MOV #0x0808, W0"); //0x0b0b   //select adc channel 11 for Ir
	asm("MOV W0, ADCHS");    
	asm("BSET ADCON1,#1");        //start converstion
	asm("LOOPb:");
	asm("BTSS ADCON1,#0");        //wait for converstion
	asm("BRA LOOPb");
	asm("BTG ADCON1,#0"); 
	asm("MOV ADCBUF0, W0");
    asm("SUB W5,W0,W0");
    asm("CLR W6");                //check greater than zero
    asm("CPSGT W0, W6");
    asm("CLR W0");
    asm("MOV _Irtrip, W6");       //read current max trip limit
    asm("CPSLT W0, W6");
    asm("BSET _IFS2+1,#4");       //set if fault 
	asm("MOV W0, _Ipv");

PWM_Flag = 0; //reset flag

   } //PWM interupt end

///////////////////////////////////////////////////////////////////////

//fault interrupt
void _ISRFAST __attribute__((interrupt, no_auto_psv)) _FLTBInterrupt(void)
   {
     
     PWMenable = 0; //disable pwm if fault
     SET = 0;       //all switches off
     
ClrWdt();

fault_Flag = 0; //reset flag
   }//fault end

///////////////////////////////////////////////////////////////////////


