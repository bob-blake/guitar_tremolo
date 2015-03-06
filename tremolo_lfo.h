#include <16f684.h>
#device adc=8

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES HS                       //High speed Osc (> 4mhz for PCM/PCH) (>10mhz for PCD)
#FUSES NOBROWNOUT               //No brownout reset
#FUSES PUT                      //Power Up Timer
#FUSES NOCPD                    //No EE protection
#FUSES IESO                     //Internal External Switch Over mode enabled
#FUSES MCLR                     //Master Clear pin enabled
#FUSES NOPROTECT                //Code not protected from reading

#use delay(clock=20M,xtal=20M)
//#use rs232(baud=9600,parity=N,xmit=PIN_B1,rcv=PIN_B4,bits=8)

