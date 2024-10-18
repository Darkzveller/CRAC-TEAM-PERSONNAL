#include <Arduino.h>
#include "../ID.h" // Remonte d'un niveau pour atteindre lib

/// ATTENTION, EFFECTUER PAR UN AMATEUR EN TIMER
/// IL EST FORT PROBABLE QU'IL Y EST UNE TRES MAUVAISE GESTION DU TIMER
// VOUS RETROUVEREZ EN BAS DE CETTE PAGE, MON RAISONNEMENT MISE AU PROPRE
/// PS : Vous remarquerez que c'est la seul bibliotheque qui détient autant de commentaire
///      car je ne suis pas sur de ce que je fais
///      DANS MON CAS L UTILISATION D UN SEUL TIMER ME SUFFIT DANS UN PEU PRES TOUS LES CAS

// Variables globales pour gérer le timer
hw_timer_t *timer = NULL;        // Pointeur vers le timer matériel
volatile bool timerFlag = false; // Flag déclenché lors de l'interruption

// Fonction d'interruption appelée lorsque le timer expire
void IRAM_ATTR onTimer()
{
    timerFlag = true; // Définir le flag à vrai pour signaler l'interruption
    if (DEBUG_TIMER_0_INIT)
    {
        static int debug_flag = 0;
        Serial.printf("Test du bon fonctionnement du flag");
        Serial.printf(" incrémetation de debug flag %d", debug_flag);
        Serial.println();
    }
}

void timer_init(int numero_timer, float reglage_alarm_timer_seconde)
{
    // Initialisation du timer matériel
    // Arguments :
    // - numéro du timer (0-3), l'ESP32 dispose de 4 timers matériels
    // - préscaler : 80 signifie que le timer s'incrémente à 1 MHz (80 MHz / 80 = 1 MHz)
    // - auto-reload : vrai pour redémarrer automatiquement le timer après chaque expiration
    timer = timerBegin(numero_timer, 80, true);

    // Attacher la fonction d'interruption au timer
    timerAttachInterrupt(timer, &onTimer, true);

    // Régler l'alarme du timer pour qu'il se déclenche toutes les x seconde (x 000 000 µs)
    int reglage_alarm_timer_micro_seconde = reglage_alarm_timer_seconde * 1000000;
    timerAlarmWrite(timer, reglage_alarm_timer_micro_seconde, true);
    // Activer l'alarme du timer
    timerAlarmEnable(timer);
    if (DEBUG_TIMER_0_INIT)
    {
        Serial.printf("Initialisation du timrer est effectué");
        Serial.println();
    }
}

/*
Explication du code :Initialisation du timer :Le timer est initialisé avec la fonction timerBegin().
On choisit ici le premier timer matériel (numéro 0).
Le préscaler de 80 fait que le timer s'incrémente à une fréquence de 1 MHz (80 MHz de base pour l'ESP32 divisé par 80).
On définit également un mode "auto-reload" pour que le timer recommence automatiquement après chaque expiration.
Attachement de l'interruption :La fonction d'interruption onTimer() est appelée lorsque le timer expire.
 Cette fonction se contente de mettre à jour un flag (timerFlag) pour signaler à la boucle principale qu'une seconde s'est écoulée.

Réglage de l'alarme :L'alarme du timer est configurée avec timerAlarmWrite() pour s'activer toutes les 1 000 000 microsecondes (1 seconde).
Boucle principale :Dans la boucle principale loop(), on vérifie si le flag du timer a été déclenché.
Si c'est le cas, on effectue une action (ici, afficher un message sur le moniteur série), puis on réinitialise le flag.
*/