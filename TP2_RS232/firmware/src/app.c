/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "GesFifoTh32.h"
#include "Mc32CalCrc16.h"
#include "Mc32gest_RS232.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Local variables */
    uint8_t Led_Counter, Cycle = 0;
    
    uint8_t Leds_Address[8] =    {BSP_LED_0, 
                                    BSP_LED_1, 
                                    BSP_LED_2, 
                                    BSP_LED_3, 
                                    BSP_LED_4, 
                                    BSP_LED_5, 
                                    BSP_LED_6, 
                                    BSP_LED_7};

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            // Initialisation du LCD
            lcd_init();
            
            // Initialisation du FIFO
            InitFifoComm();
            
            GPWM_Initialize();
            
            // Affichage sur le LCD des lignes de base
            printf_lcd("Tp2 RS232 2022-2023");
            lcd_gotoxy(1, 2);
            printf_lcd("Maelle Clerc");
            lcd_gotoxy(1, 3);
            printf_lcd("Gaja Subramaniyam");
            lcd_bl_on();
       
            // Initialisation de l'ADC
            BSP_InitADC10();
            
            // Allumer toutes les Leds
            for (Led_Counter = 0; Led_Counter < 8; Led_Counter++)
            {
                BSP_LEDOff(Leds_Address[Led_Counter]); 
            }
            
            appData.state = APP_STATE_WAIT;
            
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            // R�ception param. remote
            CommStatus = GetMessage(&PWMData);
            Cycle ++ ;
            
            // Lecture pot.
            if (CommStatus == 0)        // local ?
            {
                GPWM_GetSettings();     // local
            }
            else
            {
                GPWM_GetSettings();     // remote
            }
            
            // Affichage
            GPWM_DispSettings();
            
            // Ex�cution PWM et gestion moteur
            GPWM_ExecPWM(); 
            
            // Envoi valeurs
            if (CommStatus == 0 && Cycle == 5)                // local ?
            {
                SendMessage(&PWMData);          // local
                Cycle = 0;
            }
            else
            {
                SendMessage(&PWMDataToSend);    // remote
            }
            
            appData.state = APP_STATE_WAIT;     
            
            break;
        }

        case APP_STATE_WAIT:
        {        
            break;
        }        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

void APP_UpdateState (APP_STATES NewState)
{
    appData.state = NewState;
}

 

/*******************************************************************************
 End of File
 */
