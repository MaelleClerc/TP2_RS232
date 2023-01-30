// Mc32Gest_RS232.C
// Canevas manipulatio TP2 RS232 SLO2 2017-18
// Fonctions d'émission et de réception des message
// CHR 20.12.2016 ajout traitement int error
// CHR 22.12.2016 evolution des marquers observation int Usart
// SCA 03.01.2018 nettoyé réponse interrupt pour ne laisser que les 3 ifs

#include <xc.h>
#include <sys/attribs.h>
#include "system_definitions.h"
// Ajout CHR
#include <GenericTypeDefs.h>
#include "app.h"
#include "GesFifoTh32.h"
#include "Mc32gest_RS232.h"
#include "gestPWM.h"
#include "Mc32CalCrc16.h"


typedef union {
        uint16_t val;
        struct {uint8_t lsb;
                uint8_t msb;} shl;
} U_manip16;


// Definition pour les messages
#define MESS_SIZE  5
// avec int8_t besoin -86 au lieu de 0xAA
#define STX_code  (-86)

// Structure décrivant le message
typedef struct {
    uint8_t Start;
    int8_t  Speed;
    int8_t  Angle;
    uint8_t MsbCrc;
    uint8_t LsbCrc;
} StruMess;


// Struct pour émission des messages
StruMess TxMess;
StruMess monTxMess;
// Struct pour réception des messages
StruMess RxMess;

// Declaration des FIFO pour réception et émission
#define FIFO_RX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages
#define FIFO_TX_SIZE ( (4*MESS_SIZE) + 1)  // 4 messages

int8_t fifoRX[FIFO_RX_SIZE];
int8_t monFifoRX[FIFO_RX_SIZE];
// Declaration du descripteur du FIFO de réception
S_fifo descrFifoRX;


int8_t fifoTX[FIFO_TX_SIZE];
// Declaration du descripteur du FIFO d'émission
S_fifo descrFifoTX;


// Initialisation de la communication sérielle
void InitFifoComm(void)
{    
    // Initialisation du fifo de réception
    InitFifo ( &descrFifoRX, FIFO_RX_SIZE, fifoRX, 0 );
    // Initialisation du fifo d'émission
    InitFifo ( &descrFifoTX, FIFO_TX_SIZE, fifoTX, 0 );
    
    // Init RTS 
    RS232_RTS = 1;   // interdit émission par l'autre
   
} // InitComm

 
// Valeur de retour 0  = pas de message reçu donc local (data non modifié)
// Valeur de retour 1  = message reçu donc en remote (data mis à jour)
int GetMessage(S_pwmSettings *pData)
{
    int commStatus = 0;
    uint8_t Decalage = 0, i = 0;
    uint16_t ValCrc16 = 0xFFFF;
    int32_t NbCharToRead;
    
    // Traitement de réception à introduire ICI
    // Lecture et décodage fifo réception
    // ...
    GetCharFromFifo(&descrFifoRX, monFifoRX); // &monFifoRX[0]]
    
    
    // Message présent dans le FIFO?
    NbCharToRead = GetReadSize(&descrFifoRX);
    // Si >= taille message alors traite
    if (NbCharToRead >= MESS_SIZE)
    {
        // Analyse du contenu du message
        //test
        if ((monFifoRX[0] == 0xAA) && (monFifoRX[5] == 0xAA))
        {
            // Calcul du Crc pour vérification du message
            ValCrc16 = updateCRC16(ValCrc16, 0xAA);
            ValCrc16 = updateCRC16(ValCrc16, monFifoRX[1]);
            ValCrc16 = updateCRC16(ValCrc16, monFifoRX[2]);
            
            // Comparaison du Crc calcule avec le Crc recu
            if (ValCrc16 == ((monFifoRX[3] << 8) + monFifoRX[4]))
            {
                // Mise en memoire des valeurs de vitesse et d'angle recues
                pData->AngleSetting = monFifoRX[1];
                pData->SpeedSetting = monFifoRX[2];
                pData->absSpeed = abs(monFifoRX[2]);
                
                // Le message etant complet, on peut le nettoyer de la fifo
                Decalage = 5;
                
                // On renvoie qu'un message a été recu
                commStatus = 1;
            }
        }
        else
        {
            // Le message n'étant pas bon, on nettoie juste le byte le plus vieux de la fifo
            Decalage = 1;
        }
        
        // Decalage du tableau intermediaire contenant le fifo selon les bytes deja lus ou utilises
        for (i = 0; i <= NbCharToRead; i++)
        {
            monFifoRX[i] = monFifoRX[Decalage + i];
        }
    }
    
    // Gestion controle de flux de la réception
    if(GetWriteSpace ( &descrFifoRX) >= (2*MESS_SIZE)) 
    {
        // autorise émission par l'autre
        RS232_RTS = 0;
    }
    return commStatus;
} // GetMessage


// Fonction d'envoi des messages, appel cyclique
void SendMessage(S_pwmSettings *pData)
{
    int8_t freeSize;
    int16_t ValCrc16 = 0xFFFF;
    
    // Traitement émission à introduire ICI
    // Formatage message et remplissage fifo émission
    // ...
    freeSize = GetWriteSpace (&descrFifoTX);
    if(freeSize >= MESS_SIZE)
    {
        //Compose le message
   
        ValCrc16 = updateCRC16(ValCrc16, 0xAA);  
        ValCrc16 = updateCRC16(ValCrc16, pData->AngleSetting);
        ValCrc16 = updateCRC16(ValCrc16, pData->SpeedSetting);
        
        monTxMess.Start = 0xAA;
        monTxMess.Angle = pData->AngleSetting;
        monTxMess.Speed = pData->SpeedSetting;
        monTxMess.MsbCrc = (ValCrc16 & 0xFF00) >> 8 ;
        monTxMess.LsbCrc = (ValCrc16 & 0x00FF);
        
        //Dépose le message dans le fifo
        
        PutCharInFifo(&descrFifoTX, monTxMess.Start);
        PutCharInFifo(&descrFifoTX, monTxMess.Angle);
        PutCharInFifo(&descrFifoTX, monTxMess.Speed);
        PutCharInFifo(&descrFifoTX, monTxMess.MsbCrc);
        PutCharInFifo(&descrFifoTX, monTxMess.LsbCrc);
    }
    
    // Gestion du controle de flux
    // si on a un caractère à envoyer et que CTS = 0
    freeSize = GetReadSize(&descrFifoTX);
    if ((RS232_CTS == 0) && (freeSize > 0))
    {
        // Autorise int émission    
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);                
    }
}


// Interruption USART1
// !!!!!!!!
// Attention ne pas oublier de supprimer la réponse générée dans system_interrupt
// !!!!!!!!
 void __ISR(_UART_1_VECTOR, ipl5AUTO) _IntHandlerDrvUsartInstance0(void)
{
    USART_ERROR UsartStatus;  
    uint8_t TXsize;
    int8_t c;
    int8_t i_cts = 0;
    BOOL TxBuffFull;


    // Marque début interruption avec Led3
    LED3_W = 1;
    
    // Is this an Error interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_ERROR) ) {
        /* Clear pending interrupt */
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
        // Traitement de l'erreur à la réception.
    }
   

    // Is this an RX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_RECEIVE) ) {

        // Oui Test si erreur parité ou overrun
        UsartStatus = PLIB_USART_ErrorsGet(USART_ID_1);

        if ( (UsartStatus & (USART_ERROR_PARITY |
                             USART_ERROR_FRAMING | USART_ERROR_RECEIVER_OVERRUN)) == 0) {

            // Traitement RX à faire ICI
            // Lecture des caractères depuis le buffer HW -> fifo SW
			//  (pour savoir s'il y a une data dans le buffer HW RX : PLIB_USART_ReceiverDataIsAvailable())
			//  (Lecture via fonction PLIB_USART_ReceiverByteReceive())
            // ...
            
            c = PLIB_USART_ReceiverByteReceive(USART_ID_1);
            PutCharInFifo(&descrFifoRX, c);
                         
            LED4_W = !LED4_R; // Toggle Led4
            // buffer is empty, clear interrupt flag
            PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
        } else {
            // Suppression des erreurs
            // La lecture des erreurs les efface sauf pour overrun
            if ( (UsartStatus & USART_ERROR_RECEIVER_OVERRUN) == USART_ERROR_RECEIVER_OVERRUN) {
                   PLIB_USART_ReceiverOverrunErrorClear(USART_ID_1);
            }
        }

        
        // Traitement controle de flux reception à faire ICI
        // Gerer sortie RS232_RTS en fonction de place dispo dans fifo reception
        // ...

        
    } // end if RX

    
    // Is this an TX interrupt ?
    if ( PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) &&
                 PLIB_INT_SourceIsEnabled(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT) ) {

        // Traitement TX à faire ICI
        // Envoi des caractères depuis le fifo SW -> buffer HW
            
        // Avant d'émettre, on vérifie 3 conditions :
        //  Si CTS = 0 autorisation d'émettre (entrée RS232_CTS)
        //  S'il y a un caratères à émettre dans le fifo
        //  S'il y a de la place dans le buffer d'émission (PLIB_USART_TransmitterBufferIsFull)
        //   (envoi avec PLIB_USART_TransmitterByteSend())
       
        TXsize = GetReadSize(&descrFifoTX);
        i_cts = RS232_CTS;
        TxBuffFull = PLIB_USART_TransmitterBufferIsFull(USART_ID_1);
        
        if ((i_cts == 0) && (TXsize > 0) && (TxBuffFull == false))
        {
            do
            {
                GetCharFromFifo(&descrFifoTX, &c);
                PLIB_USART_TransmitterByteSend(USART_ID_1, c);
                i_cts = RS232_CTS;
                TXsize = GetReadSize(&descrFifoTX);
                TxBuffFull = PLIB_USART_TransmitterBufferIsFull(USART_ID_1);
            }while ((i_cts == 0) && (TXsize > 0) && (TxBuffFull == false));
        }
       
	   
        LED5_W = !LED5_R; // Toggle Led5
		
        // disable TX interrupt (pour éviter une interrupt. inutile si plus rien à transmettre)
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        
        // Clear the TX interrupt Flag (Seulement apres TX) 
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    }
    // Marque fin interruption avec Led3
    LED3_W = 0;
 }




