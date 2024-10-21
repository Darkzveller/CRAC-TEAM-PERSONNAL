#include "Variable.h" // Remonte d'un niveau pour atteindre lib
#include "MOTEUR.h"
#include "ASSERVISSEMENT.h"


void asservissement_roue_folle_droite_tick(double consigne, double observation)
{
  bool sens = 0;
  double sortie_roue_folle;

  double resolution_calculer = (pow(2, resolution_pwm) - 1)*POURCENT_MAX_PWM;
  double erreur = consigne - observation;

  double proportionnel = erreur * coeff_P_roue_folle_tick;

  double deriver = coeff_D_roue_folle_tick * (erreur - erreur_prec_roue_folle_droite_tick) / Te;

  somme_integral_roue_folle_droite_tick += erreur * Te;
  if (somme_integral_roue_folle_droite_tick > integral_limit_roue_folle_tick)
  {
    somme_integral_roue_folle_droite_tick = integral_limit_roue_folle_tick;
  }
  else if (somme_integral_roue_folle_droite_tick < -integral_limit_roue_folle_tick)
  {
    somme_integral_roue_folle_droite_tick = -integral_limit_roue_folle_tick;
  }

  // if ((erreur < 1) || (erreur > -1))
  // {

  //   somme_integral_roue_folle_droite_tick = 0;
  // }
  double integral = coeff_I_roue_folle_tick * somme_integral_roue_folle_droite_tick;

  double commande = proportionnel + deriver + integral;

  erreur_prec_roue_folle_droite_tick = erreur;

  // Gestion des bornes de la commande
  if (commande > 0)
  {
    sens = false;
    if (commande > resolution_calculer)
    {
      sortie_roue_folle = resolution_calculer;
    }
    else
    {
      sortie_roue_folle = commande;
    }
  }
  else
  {
    sens = true;
    if (commande < -resolution_calculer)
    {

      sortie_roue_folle = resolution_calculer;
    }
    else
    {
      sortie_roue_folle = -commande;
    }
  }
  Serial.printf("cmd %5.2f P %5.2f D %5.2f I %5.2f Sinte %5.2f err %5.2f obs %5.2f  cons %5.2f resol %4.0f ", commande, proportionnel, deriver, integral, somme_integral_roue_folle_droite_tick, erreur, observation, consigne,resolution_calculer);
  moteur_droit(sortie_roue_folle, sens);
}

void asservissement_roue_folle_gauche_tick(double consigne, double observation)
{
  bool sens = 0;
  double sortie_roue_folle;

  double resolution_calculer = pow(2, resolution_pwm) - 1;
  double erreur = consigne - observation;

  double proportionnel = erreur * coeff_P_roue_folle_tick;

  double deriver = coeff_D_roue_folle_tick * (erreur - erreur_prec_roue_folle_gauche_tick) / Te;

  somme_integral_roue_folle_gauche_tick += erreur * Te;
  if (somme_integral_roue_folle_gauche_tick > integral_limit_roue_folle_tick)
  {
    somme_integral_roue_folle_gauche_tick = integral_limit_roue_folle_tick;
  }
  else if (somme_integral_roue_folle_gauche_tick < -integral_limit_roue_folle_tick)
  {
    somme_integral_roue_folle_gauche_tick = -integral_limit_roue_folle_tick;
  }
  // if ((erreur < 1) || (erreur > -1))
  // {

  //   somme_integral_roue_folle_gauche_tick = 0;
  // }

  double integral = coeff_I_roue_folle_tick * somme_integral_roue_folle_gauche_tick;

  double commande = proportionnel + deriver + integral;

  erreur_prec_roue_folle_gauche_tick = erreur;

  // Gestion des bornes de la commande
  if (commande > 0)
  {
    sens = false;
    if (commande > resolution_calculer)
    {
      sortie_roue_folle = resolution_calculer;
    }
    else
    {
      sortie_roue_folle = commande;
    }
  }
  else
  {
    sens = true;
    if (commande < -resolution_calculer)
    {

      sortie_roue_folle = resolution_calculer;
    }
    else
    {
      sortie_roue_folle = -commande;
    }
  }

  // Serial.printf("cmd %5.2f P %5.2f D %5.2f I %5.2f Sinte %5.2f err %5.2f obs %5.2f  cons %5.2f ", commande, proportionnel, deriver, integral, somme_integral_roue_folle_tick, erreur, observation, consigne);
  moteur_gauche(sortie_roue_folle, sens);
}
