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

// get/set LED_ACK aliases
#define LED_ACK_TRIS                 TRISAbits.TRISA5
#define LED_ACK_LAT                  LATAbits.LATA5
#define LED_ACK_PORT                 PORTAbits.RA5
#define LED_ACK_WPU                  WPUAbits.WPUA5
#define LED_ACK_OD                   ODCONAbits.ODA5
#define LED_ACK_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define LED_ACK_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define LED_ACK_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define LED_ACK_GetValue()           PORTAbits.RA5
#define LED_ACK_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define LED_ACK_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define LED_ACK_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define LED_ACK_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define LED_ACK_SetPushPull()        do { ODCONAbits.ODA5 = 0; } while(0)
#define LED_ACK_SetOpenDrain()       do { ODCONAbits.ODA5 = 1; } while(0)

// get/set RB6 procedures
#define RB6_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define RB6_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define RB6_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define RB6_GetValue()              PORTBbits.RB6
#define RB6_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define RB6_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define RB6_SetPullup()             do { WPUBbits.WPUB6 = 1; } while(0)
#define RB6_ResetPullup()           do { WPUBbits.WPUB6 = 0; } while(0)
#define RB6_SetAnalogMode()         do { ANSELBbits.ANSB6 = 1; } while(0)
#define RB6_SetDigitalMode()        do { ANSELBbits.ANSB6 = 0; } while(0)

// get/set RB7 procedures
#define RB7_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define RB7_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define RB7_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define RB7_GetValue()              PORTBbits.RB7
#define RB7_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define RB7_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define RB7_SetPullup()             do { WPUBbits.WPUB7 = 1; } while(0)
#define RB7_ResetPullup()           do { WPUBbits.WPUB7 = 0; } while(0)
#define RB7_SetAnalogMode()         do { ANSELBbits.ANSB7 = 1; } while(0)
#define RB7_SetDigitalMode()        do { ANSELBbits.ANSB7 = 0; } while(0)

// get/set LED_NAV aliases
#define LED_NAV_TRIS                 TRISCbits.TRISC5
#define LED_NAV_LAT                  LATCbits.LATC5
#define LED_NAV_PORT                 PORTCbits.RC5
#define LED_NAV_WPU                  WPUCbits.WPUC5
#define LED_NAV_OD                   ODCONCbits.ODC5
#define LED_NAV_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define LED_NAV_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define LED_NAV_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define LED_NAV_GetValue()           PORTCbits.RC5
#define LED_NAV_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define LED_NAV_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define LED_NAV_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define LED_NAV_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define LED_NAV_SetPushPull()        do { ODCONCbits.ODC5 = 0; } while(0)
#define LED_NAV_SetOpenDrain()       do { ODCONCbits.ODC5 = 1; } while(0)

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