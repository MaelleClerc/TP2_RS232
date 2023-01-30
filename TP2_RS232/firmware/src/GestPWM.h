#ifndef GestPWM_H
#define GestPWM_H
/*--------------------------------------------------------*/
// GestPWM.h
/*--------------------------------------------------------*/
//	Description :	Gestion des PWM 
//			        pour TP1 2016-2017
//
//	Auteur 		: 	C. HUBER
//
//	Version		:	V1.1
//	Compilateur	:	XC32 V1.42 + Harmony 1.08
//
/*--------------------------------------------------------*/

#include <stdint.h>



/*--------------------------------------------------------*/
// D�finition des fonctions prototypes
/*--------------------------------------------------------*/


typedef struct {
    uint8_t absSpeed;    // vitesse 0 � 99
    uint8_t absAngle;    // Angle  0 � 180
    int8_t SpeedSetting; // consigne vitesse -99 � +99
    int8_t AngleSetting; // consigne angle  -90 � +90
} S_pwmSettings;

extern S_pwmSettings PWMData; 
extern S_pwmSettings PWMDataToSend; 

void GPWM_Initialize();

// Ces 3 fonctions ont pour param�tre un pointeur sur la structure S_pwmSettings.
void GPWM_GetSettings();            // Obtention vitesse et angle
void GPWM_DispSettings(int Remote);	// Affichage
void GPWM_ExecPWM();                // Execution PWM et gestion moteur.
void GPWM_ExecPWMSoft();            // Execution PWM software.


#endif
