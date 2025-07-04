#include "Variable.h"
#include "ASSERVISSEMENT.h"
#include "MOUVEMENT.h"
#include "MOTEUR.h"
#include <mat.h>
#include "USE_FUNCTION.h"
#include "OTA.h"
#include "EncoderManager.h"
static int flag_sens_polaire = 0;

void rotation(int consigne, int vitesse)
{
    if (consigne >= 0)
    {
        sens = 1;
    }
    else
    {
        sens = -1;
    }
    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * -sens;

    int consigne_gauche = consigne;
    int consigne_droite = consigne * -1.0;
    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec);

    consigne_position_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_position_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);
    if ((etat_actuel_vit_roue_folle_gauche != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_GAUCHE) && (etat_actuel_vit_roue_folle_droite != ETAT_DECELERATION_Vitesse_ROUE_FOLLE_DROITE))
    {
        float ecart = consigne_odo_gauche_prec + consigne_odo_droite_prec;
        consigne_position_gauche = -consigne_position_droite + ecart;
    }

    // // // On force les consignes à être égales et opposées
    // consigne_position_droite = sens * consigne_regulation_moyenne;
    // consigne_position_gauche = -sens * consigne_regulation_moyenne;
}

void ligne_droite(int consigne, int vitesse)
{
    if (consigne >= 0)
    {
        sens = 1;
    }
    else if (consigne < 0)
    {
        sens = -1;
    }
    float vitesse_croisiere_gauche = vitesse * sens;
    float vitesse_croisiere_droit = vitesse * sens;

    int consigne_gauche = consigne;
    int consigne_droite = consigne;

    consigne_gauche = (consigne_gauche + consigne_odo_gauche_prec);
    consigne_droite = (consigne_droite + consigne_odo_droite_prec);

    // Calcul initial des consignes de vitesse pour chaque roue
    consigne_position_droite = regulation_vitesse_roue_folle_droite(consigne_droite, vitesse_croisiere_droit);
    consigne_position_gauche = regulation_vitesse_roue_folle_gauche(consigne_gauche, vitesse_croisiere_gauche);
    // Correction d'angle basée sur l'écart entre les consignes gauche et droite
    correction = asservissement_angle_correction(consigne_theta_prec, degrees(theta_robot));

    // // Appliquer la correction à la consigne de vitesse
    consigne_position_droite -= correction;
    consigne_position_gauche += correction;
}

void asser_polaire_tick(float coordonnee_x, float coordonnee_y, float theta_position_fin_deg)
{
    // Serial.printf(" etat_x_y_theta %d ", etat_x_y_theta);

    switch (etat_x_y_theta)
    {
    case 0:
        erreur_distance = convert_distance_mm_to_tick(sqrt(pow(coordonnee_x - odo_x, 2) + pow(coordonnee_y - odo_y, 2))); // On détermine la distance restante a parcourir
        erreur_orient = atan2((coordonnee_y - odo_y), (coordonnee_x - odo_x)) - theta_robot;                              // On détermine l'angle a parcour pour arriver a destination en radians

        erreur_orient = normaliser_angle_rad(erreur_orient);
        // Serial.printf(" er_o_avant_detec %.1f ", degrees(erreur_orient));
        // TelnetStream.printf(" er_o_avant_detec %.1f ", degrees(erreur_orient));

        // Serial.printf(" flg_sp %d ", flag_sens_polaire);

        // Maintenant on convertit erreur_orient en tick
        erreur_orient = convert_angle_radian_to_tick(normaliser_angle_rad((determination_sens_polaire(erreur_orient))));
        // erreur_orient = constrain(erreur_orient, -1250, 1250);
        consigne_rot_polaire_tick = erreur_orient;

        if ((erreur_orient > convert_angle_deg_to_tick(20)) || (erreur_orient < convert_angle_deg_to_tick(-20)))
        {
            // Serial.printf(" Vrai 2");
            consigne_dist_polaire_tick = 0;
        }
        else
        {
            // Serial.printf(" Vrai 3");
            consigne_dist_polaire_tick = consigne_dist_polaire_tick_max;
        }
        // Inverser la consigne de distance si besoin
        if (sens == -1)
        {
            // Serial.printf("dist négatif ");
            consigne_dist_polaire_tick = -consigne_dist_polaire_tick;
        }
        gestion_freinage_ligne_droite(SEUIL_ACTIVATION_DECELERATION_DISTANCE, TOLERANCE_ERREUR_DISTANCE_AUTORISER);

        consigne_position_gauche = odo_tick_gauche + coeff_dist_polaire_tick * consigne_dist_polaire_tick - coeff_rot_polaire_tick * consigne_rot_polaire_tick; // commande en tick qu'on souhaite atteindre
        consigne_position_droite = odo_tick_droit + coeff_dist_polaire_tick * consigne_dist_polaire_tick + coeff_rot_polaire_tick * consigne_rot_polaire_tick;  // commande en tick qu'on souhaite atteindre
        // Serial.printf(" cs x %.1f ", coordonnee_x);
        // Serial.printf(" cs_y %.1f ", coordonnee_y);
        // Serial.printf(" cs_t %.1f ", theta_position_fin_deg);

        // TelnetStream.printf(" cs x %.1f ", coordonnee_x);
        // TelnetStream.printf(" cs_y %.1f ", coordonnee_y);

        // Serial.printf(" cpt %d ", compteur);
        // Serial.printf(" S %d ", sens);

        // Serial.printf(" Odo x %.1f ", odo_x);
        // Serial.printf(" odo_y %.1f ", odo_y);
        // Serial.printf(" teheta %.3f ", degrees(theta_robot));
        // TelnetStream.printf(" Odo x %.3f ", odo_x);
        // TelnetStream.printf(" odo_y %.3f ", odo_y);
        // TelnetStream.printf(" teheta %.3f ", degrees(theta_robot));

        // Serial.printf(" er_d %.3f ", convert_distance_tick_to_mm(erreur_distance));
        // Serial.printf(" er_o %.3f ", convert_tick_to_angle_deg(erreur_orient));
        // Serial.printf(" consigne_dist_polaire_tick %f", consigne_dist_polaire_tick);
        // Serial.printf(" consigne_dist_polaire_tick_max %f", consigne_dist_polaire_tick_max);

        // Serial.printf(" cs_p_gauche %f ", consigne_position_gauche);
        // Serial.printf(" cs_p_droit  %f ", consigne_position_droite);

        // Serial.printf(" cmd_d %.1f ", consigne_dist_polaire_tick);
        // Serial.printf(" cmd_r %.1f ", consigne_rot_polaire_tick);
        // Serial.printf(" cff_r %.1f ", coeff_rot_polaire_tick);
        // Serial.printf(" cff_d %.1f ", coeff_dist_polaire_tick);

        // Serial.printf(" dist_dcl %.1f ", convert_distance_tick_to_mm(distance_decl_polaire_tick));
        // Serial.printf(" coef_decl %.1f ", coeff_decc_distance_polaire_tick);

        // Serial.printf(" odo_g %.0f ", odo_tick_gauche);
        // Serial.printf(" odo_d %.0f ", odo_tick_droit);

        // Serial.printf(" angl_tick %.1f ", (float)convert_angle_deg_to_tick(90));
        // Serial.printf(" angl_deg %.1f ", convert_tick_to_angle_deg(convert_angle_deg_to_tick(90)));

        break;

    case 1:
        rotation(convert_angle_deg_to_tick(theta_position_fin_deg - 7.5), SPEED_NORMAL);

        if (return_flag_asser_roue())
        {
            enregistreur_odo();

            etat_x_y_theta = 2;

            flag_fin_mvt_x_y_theta = true;
        }

        break;

    case 2:

        break;

    default:
        break;
    }
    // Serial.println();
    // TelnetStream.println();
}

bool recalage(uint8_t direction, uint8_t type_modif, float nouvelle_valeur, uint16_t consigne_rotation)
{
    bool flag_modif_fait = false;
    enum MouvementRecalage
    {
        IMMOBILE = 0,
        AVANT = 1,
        ARRIERE = 2
    };

    enum TypeModification
    {
        AUCUNE = 0,
        MODIF_X = 1,
        MODIF_Y = 2,
        MODIF_THETA = 3
    };
    int8_t sens = 0;

    // Machine à états pour le mouvement
    switch (direction)
    {
    case IMMOBILE:
        // Pas de mouvement , rien à faire ici
        // flag_modif_fait = true;
        if (type_modif == MODIF_X)
        {
            // odo_x = convert_distance_mm_to_tick(nouvelle_valeur);
            odo_x = nouvelle_valeur;
            flag_modif_fait = true;
            Serial.printf("modif effec x");
            // delay( 2000);
        }
        else if (type_modif == MODIF_Y)
        {
            // odo_y = convert_distance_mm_to_tick(nouvelle_valeur);
            odo_y = nouvelle_valeur;
            flag_modif_fait = true;

            Serial.printf("modif effec y");
            // delay( 2000);
        }
        if (type_modif == MODIF_THETA)
        {
            theta_robot = radians((nouvelle_valeur));
            consigne_theta_prec = convert_angle_deg_to_tick(nouvelle_valeur);
            flag_modif_fait = true;
        }

        break;
    case AVANT:
    case ARRIERE:

        sens = (direction == AVANT) ? 1 : -1; // condition ? valeur_si_vrai : valeur_si_faux;

        consigne_position_gauche = odo_tick_gauche + sens * SPEED_NORMAL;
        consigne_position_droite = odo_tick_droit + sens * SPEED_NORMAL;

        correction = asservissement_angle_correction(consigne_theta_prec, degrees(theta_robot));
        consigne_position_droite -= correction;
        consigne_position_gauche += correction;
        break;
    }

    // Mise à jour de l’odométrie si contact
    if (toucher_objet_solid())
    {
        if (type_modif == MODIF_X)
        {
            // odo_x = convert_distance_mm_to_tick(nouvelle_valeur);
            odo_x = nouvelle_valeur;
            flag_modif_fait = true;
            Serial.printf("modif effec x");
            // delay( 2000);
        }
        else if (type_modif == MODIF_Y)
        {
            // odo_y = convert_distance_mm_to_tick(nouvelle_valeur);
            odo_y = nouvelle_valeur;
            flag_modif_fait = true;

            Serial.printf("modif effec x");
            // delay( 2000);
        }
    }
    if (type_modif == MODIF_THETA)
    {
        theta_robot = radians((nouvelle_valeur));
        consigne_theta_prec = convert_angle_deg_to_tick(nouvelle_valeur);
        flag_modif_fait = true;
    }

    // Rotation demandée ?
    if (consigne_rotation != 0)
    {
        Serial.printf("Déclenchement rotation\n");
        liste.general_purpose = TYPE_DEPLACEMENT_ROTATION;
        liste.angle = convert_angle_deg_to_tick(consigne_rotation);
        liste.vitesse_croisiere = SPEED_NORMAL;
        lauch_flag_asser_roue(true);
    }

    Serial.printf(" Odo x %.3f ", odo_x);
    Serial.printf(" odo_y %.3f ", odo_y);
    Serial.printf(" teheta %.3f ", degrees(theta_robot));
    // Serial.printf(" consigne_position_droite %.0f ", consigne_position_droite);
    // Serial.printf(" consigne_position_gauche %.0f ", consigne_position_gauche);
    // Serial.printf(" odo_tick_droit %.0f ", odo_tick_droit);
    // Serial.printf(" odo_tick_gauche %.0f ", odo_tick_gauche);
    // Serial.println();
    Serial.printf("Changement effectuer \n");
    return flag_modif_fait;
}
bool toucher_objet_solid()
{

    static unsigned long last_time_droite = millis();
    static unsigned long last_time_gauche = millis();
    static int last_odo_droite = odo_tick_droit;
    static int last_odo_gauche = odo_tick_gauche;
    const unsigned long timeout = 500; // Temps avant de détecter un blocage (ms)

    static bool roue_droite_bloquer = false;
    static bool roue_gauche_bloquer = false;

    if (millis() - last_time_droite > timeout)
    {
        if (last_odo_droite == odo_tick_droit)
        {
            Serial.printf("R_D block");
            roue_droite_bloquer = true;
        }

        last_odo_droite = odo_tick_droit;
        last_time_droite = millis();
    }

    if (millis() - last_time_gauche > timeout)
    {
        if (last_odo_gauche == odo_tick_gauche)
        {
            Serial.printf("R_G block");
            roue_gauche_bloquer = true;
        }

        last_odo_gauche = odo_tick_gauche;
        last_time_gauche = millis();
    }

    if ((roue_gauche_bloquer == true) && (roue_droite_bloquer == true))
    {
        return true;
    }
    else
    {
        return false;
    }
}
float determination_sens_polaire(float erreur_orient_radians)
{

    // Déterminer si on doit inverser le sens au premier passage
    if (flag_sens_polaire == 0)
    {
        float erreur_orient_deg = degrees(erreur_orient_radians); // car convert_tick_to_angle_deg demande un tick normalement
        if ((erreur_orient_deg > 90.0) || (erreur_orient_deg < -90.0))
        {
            sens = -1;
        }
        else
        {
            sens = 1;
        }
        flag_sens_polaire = 1;
    }

    // Si sens inversé, corriger l'orientation
    if (sens == -1)
    {
        Serial.printf("SENS ");
        TelnetStream.printf("SENS N ");

        erreur_orient_radians += radians(180.0);
        erreur_orient_radians = normaliser_angle_rad(erreur_orient_radians); // Très important après ajout de 180°
    }
    else
    {
        TelnetStream.printf("SENS P");
    }
    return erreur_orient_radians;
}

void gestion_freinage_ligne_droite(float distance_de_seuil_minimal_et_changement_point_passage__mm, float seuil_minimal_autorise_permettant_de_quitter_asser_polaire_mm)
{
    if (erreur_distance <= convert_distance_mm_to_tick(distance_de_seuil_minimal_et_changement_point_passage__mm))
    {
        float facteur_deccel = erreur_distance / convert_distance_mm_to_tick(distance_de_seuil_minimal_et_changement_point_passage__mm);
        consigne_dist_polaire_tick = consigne_dist_polaire_tick_max * facteur_deccel;
        if (convert_distance_tick_to_mm(erreur_distance) <= seuil_minimal_autorise_permettant_de_quitter_asser_polaire_mm)
        {
            Serial.printf(" Vrai ");
            enregistreur_odo();
            reset_parametre_polaire();
            etat_x_y_theta = 1;
            lauch_flag_asser_roue(true);
        }
    }
}

void reset_parametre_polaire()
{
    flag_sens_polaire = 0;
    sens = 1;
    consigne_dist_polaire_tick_max = fabs(consigne_dist_polaire_tick_max);
    coeff_rot_polaire_tick = 0.5;
    coeff_dist_polaire_tick = VALEUR_DEFAUT_COEFF_DIST_POLAIRE_TICK;
    coeff_rot_polaire_tick = VALEUR_DEFAUT_COEFF_ROT_POLAIRE_TICK;
}