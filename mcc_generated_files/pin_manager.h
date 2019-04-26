/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65.2
        Device            :  PIC16F1619
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.45
        MPLAB 	          :  MPLAB X 4.15	
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set TX_PIN aliases
#define TX_PIN_TRIS                 TRISAbits.TRISA4
#define TX_PIN_LAT                  LATAbits.LATA4
#define TX_PIN_PORT                 PORTAbits.RA4
#define TX_PIN_WPU                  WPUAbits.WPUA4
#define TX_PIN_OD                   ODCONAbits.ODA4
#define TX_PIN_ANS                  ANSELAbits.ANSA4
#define TX_PIN_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define TX_PIN_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define TX_PIN_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define TX_PIN_GetValue()           PORTAbits.RA4
#define TX_PIN_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define TX_PIN_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define TX_PIN_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define TX_PIN_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define TX_PIN_SetPushPull()        do { ODCONAbits.ODA4 = 0; } while(0)
#define TX_PIN_SetOpenDrain()       do { ODCONAbits.ODA4 = 1; } while(0)
#define TX_PIN_SetAnalogMode()      do { ANSELAbits.ANSA4 = 1; } while(0)
#define TX_PIN_SetDigitalMode()     do { ANSELAbits.ANSA4 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/