#include <Arduino.h>
#include <RadioLib.h>
#include <SSD1306Wire.h>
#include "capteurs.h"
#include "encoding.h"


// ============================================================
//  MODE TEST DE PORTEE
//  1 = CAN desactive, envoie juste un compteur (4 octets) en boucle
//      pour tester la distance/qualite du lien LoRa, sans dependre
//      du bus CAN. 0 = fonctionnement normal (CAN + encodage DBC).
//  IMPORTANT : le recepteur (LoRa_Recepteur) a le MEME define et
//  DOIT etre mis a la meme valeur, sinon il essaiera de decoder le
//  paquet compteur comme des capteurs (ou l'inverse).
// ============================================================
#define MODE_TEST_PORTEE 1

// ============================================================
//  MATERIEL - Heltec WiFi LoRa 32 V4 (ESP32-S3 + SX1262)
// ============================================================

// Broches SX1262 : NSS=8, DIO1=14, RESET=12, BUSY=13
SX1262 radio = new Module(8, 14, 12, 13);

// Ecran OLED SSD1306 : adresse 0x3c, SDA=17, SCL=18
SSD1306Wire display(0x3c, 17, 18);


// ============================================================
//  CADENCES (non bloquantes, via millis())
// ============================================================
const unsigned long INTERVALLE_ENVOI            = 200;   // ms  (5 envois/seconde) : capteurs PRIORITE_HAUTE
const unsigned long INTERVALLE_ENVOI_SECONDAIRE = 2000;  // ms  (1 envoi/2s)       : capteurs PRIORITE_SECONDAIRE
const unsigned long INTERVALLE_AFFICHAGE        = 1000;  // ms  (1 refresh/seconde)

unsigned long dernierEnvoi           = 0;
unsigned long dernierEnvoiSecondaire = 0;
unsigned long dernierAffichage       = 0;

int  compteur = 0;      // nombre de paquets transmis avec succes
bool canOK    = false;  // vrai si initCAN() a reussi

#if MODE_TEST_PORTEE
uint32_t compteurTest = 0;   // incremente a chaque paquet de test envoye
#endif


// ============================================================
//  COMPRESSION - delta encoding + zigzag/varint
// ============================================================
DeltaEncoder encoder;

// Valeurs courantes des capteurs (remplies depuis le CAN).
int sensors[NB_SENSOR] = {0};

// Buffers de travail remplis par sorting() :
uint8_t output_index[NB_SENSOR] = {0};   // index des capteurs modifies
int     output_diff[NB_SENSOR]  = {0};   // delta signe de chacun
uint16_t nb_donnees = 0;                 // combien de capteurs ont change

// Trame finale a transmettre (octets encodes).
// Pire cas : MAX_OCTETS_PAR_CAPTEUR (1 index + 5 varint) par capteur.
uint8_t  trame[TAILLE_TRAME_MAX];
uint16_t nb_octets = 0;                  // taille reelle de la trame


// ============================================================
//  AFFICHAGE OLED - systeme de buffer
//  (accumule toutes les lignes, puis un seul clear()+display())
// ============================================================
const int OLED_MAX_LIGNES    = 5;
const int OLED_HAUTEUR_LIGNE = 15;   // espacement vertical entre 2 lignes (px)

String oled_lignes[OLED_MAX_LIGNES];
int    oled_nb_lignes = 0;

void oled_reset_buffer()
{
    oled_nb_lignes = 0;
}

void oled_ajouter_ligne(const String& texte)
{
    if (oled_nb_lignes >= OLED_MAX_LIGNES)
    {
        return;
    }
    oled_lignes[oled_nb_lignes] = texte;
    oled_nb_lignes++;
}

void oled_afficher()
{
    display.clear();
    for (int i = 0; i < oled_nb_lignes; i++)
    {
        display.drawString(0, i * OLED_HAUTEUR_LIGNE, oled_lignes[i]);
    }
    display.display();
}


void setup()
{
    Serial.begin(115200);
    delay(5000);   // laisse le temps d'ouvrir le moniteur serie

    // --- Alimentation de l'ecran OLED ---
    pinMode(36, OUTPUT);
    digitalWrite(36, LOW);
    delay(100);

    // --- Reset de l'OLED ---
    pinMode(21, OUTPUT);
    digitalWrite(21, LOW);
    delay(50);
    digitalWrite(21, HIGH);
    delay(50);

    // --- Init ecran ---
    display.init();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Initialisation...");
    display.display();

    // --- Init LoRa a 915 MHz ---
    Serial.print("Initialisation LoRa... ");
    int state = radio.begin(915.0);
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println("OK!");
    }
    else
    {
        Serial.print("Erreur: ");
        Serial.println(state);
        display.clear();
        display.drawString(0, 0, "Erreur LoRa!");
        display.display();
        while (true);   // on bloque : rien a faire sans radio
    }

    // --- RF switch (DIO2) ---
    // Sur le Heltec WiFi LoRa 32 V3/V4 (SX1262), DIO2 pilote le switch
    // d'antenne TX/RX. Sans cet appel, RadioLib laisse DIO2 dans un etat
    // par defaut qui peut empecher la puissance d'emission d'atteindre
    // correctement l'antenne -> portee reduite sans erreur visible.
    if (radio.setDio2AsRfSwitch(true) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setDio2AsRfSwitch a echoue");
    }

    // --- Parametres RF ---
    radio.setSpreadingFactor(7);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);

    // Diagnostic : verifie ce que le module accepte VRAIMENT comme puissance,
    // avant de l'appliquer. Si "clipped" != 22, la puissance demandee n'est
    // pas atteignable telle quelle sur ce module.
    int8_t puissanceClippee = 0;
    int16_t etatPuissance = radio.checkOutputPower(22, &puissanceClippee);
    Serial.print("checkOutputPower(22) -> etat=");
    Serial.print(etatPuissance);
    Serial.print(" puissance_reelle_possible=");
    Serial.println(puissanceClippee);

    int16_t etatSetPower = radio.setOutputPower(22);
    if (etatSetPower != RADIOLIB_ERR_NONE)
    {
        Serial.print("Attention : setOutputPower(22) a echoue, code=");
        Serial.println(etatSetPower);
    }

    radio.setPreambleLength(8);
    radio.setSyncWord(0x12);      // DOIT etre identique cote recepteur
    radio.setCRC(true);           // DOIT etre identique cote recepteur
    Serial.println("Config RF appliquee!");

#if MODE_TEST_PORTEE
    // --- Mode test de portee : CAN volontairement desactive ---
    canOK = false;
    Serial.println("MODE TEST DE PORTEE : CAN desactive, envoi d'un compteur toutes les 200ms.");

    oled_reset_buffer();
    oled_ajouter_ligne("TEST DE PORTEE");
    oled_ajouter_ligne("CAN desactive");
    oled_afficher();
#else
    // --- Init CAN (MCP2515) ---
    // initCAN() affiche deja ses etapes de diagnostic (ETAPE A/B/C)
    // sur le moniteur serie. canOK garde le resultat pour le loop().
    canOK = initCAN();
    if (canOK)
    {
        Serial.println("CAN initialise !");
    }
    else
    {
        Serial.println("Echec init CAN - on continue sans (sensors restent a 0)");
    }

    // Ecran de demarrage.
    oled_reset_buffer();
    oled_ajouter_ligne("Emetteur");
    oled_ajouter_ligne(canOK ? "LoRa + CAN OK" : "LoRa OK / CAN KO");
    oled_afficher();
#endif

    // ============================================================
    //  Configuration de l'encodeur
    // ============================================================
    encoder.init(sensors);

    // Noms de tous les capteurs (legacy + signaux DBC), depuis la table
    // partagee kSensorNames (capteurs.h/.cpp) - meme source que celle
    // utilisee par le recepteur pour les cles JSON.
    for (int i = 0; i < NB_SENSOR; i++)
    {
        encoder.setName(i, kSensorNames[i]);
    }

    for (int i = 0; i < NB_SENSOR; i++)
    {
        encoder.setThreshold(i, 1);   // seuil croissant : capteur i -> seuil i+1
        encoder.setPriority(i, PRIORITE_SECONDAIRE);   // tout secondaire par defaut
    }

    // ============================================================
    //  Capteurs PRIORITE_HAUTE : envoyes des qu'ils changent, a chaque
    //  cycle de 200ms. Liste de DEPART pour tester le mecanisme -
    //  securite/etat critique (arret d'urgence, tous les flags de faute,
    //  vitesse, tension/courant batterie). A affiner une fois la vraie
    //  liste de priorites connue.
    // ============================================================
    const SensorIndex capteursPrioritaires[] = {
        IDX_E_STOP,
        IDX_BENDER_EARTH_FAULT, IDX_BENDER_IMD_FAULT, IDX_BENDER_SPEED_START_BAD,
        IDX_BENDER_SPEED_START_GOOD, IDX_BENDER_UV_FAULT, IDX_BENDER_HAPPY,
        IDX_BENDER_SHORT_CIRCUIT_FAULT,
        IDX_CLUSTER_CHARGING, IDX_CLUSTER_NOT_SAFE, IDX_CLUSTER_ENGAGED, IDX_CLUSTER_SAFE,
        IDX_BATTERY_OVER_VOLTAGE_FAULT, IDX_BENDER_FAULT, IDX_BATTERY_DISCONNECTION_FAULT,
        IDX_BATTERY_UNDER_VOLTAGE_FAULT, IDX_BATTERY_OVER_TEMP_FAULT,
        IDX_FAULT_CODE, IDX_FAULT_LEVEL,
        IDX_SPEED_REF, IDX_SPEED_MEASURE,
        IDX_V_BAT, IDX_I_BAT,
        IDX_BATTERY_MAX_VOLTAGE, IDX_BATTERY_MIN_VOLTAGE,
        IDX_CHARGER_HARDWARE_FAILURE, IDX_CHARGER_OVER_TEMP, IDX_CHARGER_WRONG_INPUT_VOLTAGE,
        IDX_CHARGER_BATTERY_UNDETECTED, IDX_CHARGER_COMM_ERROR,
    };
    for (unsigned int i = 0; i < sizeof(capteursPrioritaires) / sizeof(capteursPrioritaires[0]); i++)
    {
        encoder.setPriority(capteursPrioritaires[i], PRIORITE_HAUTE);
    }
}


void loop()
{
    unsigned long maintenant = millis();

    // --------------------------------------------------------
    // 1) Lecture CAN - NON bloquante.
    //    majDonneesCAN() vide toutes les trames en attente et met
    //    a jour donneesCourantes. On l'appelle a CHAQUE tour de
    //    loop() pour ne rater aucune trame.
    // --------------------------------------------------------
    if (canOK)
    {
        majDonneesCAN();
    }


    // --------------------------------------------------------
    // 2) Cycle d'ENVOI : lire CAN -> ranger -> detecter -> encoder -> transmettre
    // --------------------------------------------------------
    if (maintenant - dernierEnvoi >= INTERVALLE_ENVOI)
    {
        dernierEnvoi = maintenant;

#if MODE_TEST_PORTEE
        // --- Mode test de portee : paquet hardcode, juste un compteur 32 bits ---
        uint8_t paquetTest[4];
        paquetTest[0] = (uint8_t)(compteurTest >> 0);
        paquetTest[1] = (uint8_t)(compteurTest >> 8);
        paquetTest[2] = (uint8_t)(compteurTest >> 16);
        paquetTest[3] = (uint8_t)(compteurTest >> 24);

        int state = radio.transmit(paquetTest, sizeof(paquetTest));

        if (state == RADIOLIB_ERR_NONE)
        {
            compteur++;
            Serial.println(String("Paquet test #") + compteurTest + " envoye");
        }
        else
        {
            Serial.println(String("Erreur envoi test : ") + state);
        }

        compteurTest++;

        // Affichage OLED simple : utile si tu t'eloignes sans laptop.
        if (maintenant - dernierAffichage >= INTERVALLE_AFFICHAGE)
        {
            dernierAffichage = maintenant;
            display.clear();
            display.drawString(0, 0,  "TEST DE PORTEE");
            display.drawString(0, 15, "Envoyes: " + String(compteur));
            display.drawString(0, 27, "Compteur: " + String(compteurTest));
            display.display();
        }

        return;   // rien d'autre a faire ce tour-ci en mode test
#endif

        // (a) On recupere le dernier etat CAN et on range chaque champ
        //     dans le tableau sensors[]. Mapping arbitraire pour les
        //     tests : ordre naturel des index, partie decimale coupee
        //     (pas de virgule). Les cases sans donnee CAN restent a 0.
        // (a0) Signaux DBC (indices >= IDX_BATTERY_MAX_TEMP) : ecrase tout
        //      sensors[], donc DOIT etre appele avant les affectations
        //      legacy ci-dessous pour ne pas les effacer.
        lireValeursDBC(sensors);

        DonneesMoto donne = lireCapteurs();
        sensors[IDX_TIRE]  = (int)donne.temp_pneu;    // case 0
        sensors[IDX_BAT]   = (int)donne.pression;     // case 1
        sensors[IDX_FUEL]  = (int)donne.temp_huile;   // case 2
        sensors[IDX_SPEED] = (int)donne.vitesse;      // case 3
        // IDX_TEMP (case 4) et reserves : pas de source CAN -> restent a 0.

        // (b) Detection des changements, SANS avancer la reference :
        //     - PRIORITE_HAUTE : verifie a chaque cycle (200ms).
        //     - PRIORITE_SECONDAIRE : verifie seulement toutes les
        //       INTERVALLE_ENVOI_SECONDAIRE ms (accumule les deltas entre-temps,
        //       rien n'est perdu puisque previousValues n'avance qu'a la
        //       confirmation d'un envoi reussi).
        //     Les prioritaires sont ecrits EN PREMIER dans output_index/diff :
        //     si le paquet doit etre tronque (c), ce sont les secondaires en
        //     surplus qui passent leur tour, jamais les prioritaires.
        uint16_t nb_prioritaires = encoder.detecterParPriorite(sensors, PRIORITE_HAUTE, output_index, output_diff);

        bool cycleSecondaire = (maintenant - dernierEnvoiSecondaire >= INTERVALLE_ENVOI_SECONDAIRE);
        uint16_t nb_secondaires = 0;
        if (cycleSecondaire)
        {
            dernierEnvoiSecondaire = maintenant;   // cadence fixe, qu'il y ait des donnees ou non
            nb_secondaires = encoder.detecterParPriorite(sensors, PRIORITE_SECONDAIRE,
                                                          &output_index[nb_prioritaires],
                                                          &output_diff[nb_prioritaires]);
        }

        nb_donnees = nb_prioritaires + nb_secondaires;

        // (c) Serialisation, plafonnee a la taille max d'un paquet LoRa
        //     (255o, cf TAILLE_PAQUET_LORA_MAX) : nb_capteurs_envoyes peut
        //     etre < nb_donnees si le budget d'octets est atteint.
        uint16_t nb_capteurs_envoyes = 0;
        uint16_t taille_max_paquet = (sizeof(trame) < TAILLE_PAQUET_LORA_MAX) ? sizeof(trame) : TAILLE_PAQUET_LORA_MAX;
        nb_octets = encoder.data_formater(nb_donnees, output_index, output_diff, trame,
                                          taille_max_paquet, &nb_capteurs_envoyes);

        // (d) Transmission LoRa - seulement s'il y a quelque chose a envoyer.
        if (nb_octets > 0)
        {
            int state = radio.transmit(trame, nb_octets);

            if (state == RADIOLIB_ERR_NONE)
            {
                compteur++;

                // On ne fait avancer previousValues QUE pour les capteurs
                // reellement transmis avec succes. Si l'envoi echoue, ou si
                // certains capteurs n'ont pas tenu dans le paquet, ils restent
                // "en attente" et seront retentes au prochain cycle - rien
                // n'est jamais perdu silencieusement.
                encoder.confirmerEnvoi(output_index, output_diff, nb_capteurs_envoyes);

                Serial.println(String("Envoye : ") + nb_octets + " octets, "
                               + nb_capteurs_envoyes + "/" + nb_donnees
                               + " capteurs  (paquet #" + compteur + ")");
            }
            else
            {
                Serial.println(String("Erreur envoi : ") + state);
            }
        }
    }


    // --------------------------------------------------------
    // 3) Cycle d'AFFICHAGE debug : plus lent, pour rester lisible.
    // --------------------------------------------------------
    if (maintenant - dernierAffichage >= INTERVALLE_AFFICHAGE)
    {
        dernierAffichage = maintenant;

        oled_reset_buffer();
        oled_ajouter_ligne(String("Nb capteurs: ") + nb_donnees);

        Serial.println(String("\nCapteurs modifies: ") + nb_donnees
                       + "  |  trame: " + nb_octets + " octets");
        Serial.println("-------------------------------");

        for (int i = 0; i < nb_donnees; i++)
        {
            String ligne = String(encoder.getConfigByIndex(output_index[i]).name)
                         + ", " + output_diff[i]
                         + ", " + encoder.getConfigByIndex(output_index[i]).last_sent;

            Serial.println(ligne);
            oled_ajouter_ligne(ligne);
        }

        oled_afficher();
    }
}