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
#define MODE_TEST_PORTEE 0

// ============================================================
//  MATERIEL - Heltec WiFi LoRa 32 V4 (ESP32-S3 + SX1262)
// ============================================================

// Broches SX1262 : NSS=8, DIO1=14, RESET=12, BUSY=13
SX1262 radio = new Module(8, 14, 12, 13);

// Front-end RF externe (ampli GC1109) entre le SX1262 et l'antenne.
// Contrairement a la V3, la V4 ne l'active pas par defaut : sans ces 3
// broches a HIGH, la carte reste en mode "bypass" (perte ~40-50dB), ce qui
// expliquerait le -50dBm anormal observe a courte distance malgre le fix
// DIO2/TCXO/OCP deja en place. A activer AVANT radio.begin().
#define FEM_VFEM 7    // alimentation du front-end
#define FEM_CSD  2    // chip enable (le plus critique des 3)
#define FEM_CPS  5  // mode haute puissance (vs bypass)

// Ecran OLED SSD1306 : adresse 0x3c, SDA=17, SCL=18
SSD1306Wire display(0x3c, 17, 18);


// ============================================================
//  CADENCES (non bloquantes, via millis())
// ============================================================
const unsigned long INTERVALLE_ENVOI            = 200;   // ms  (5 envois/seconde) : cadence de chaque cycle d'envoi
const unsigned long INTERVALLE_AFFICHAGE        = 1000;  // ms  (1 refresh/seconde)

unsigned long dernierEnvoi           = 0;
unsigned long dernierAffichage       = 0;

int  compteur = 0;      // nombre de paquets transmis avec succes
bool canOK    = false;  // vrai si initCAN() a reussi

// Numero de sequence des paquets reels (hors mode test de portee, qui a deja
// son propre compteur). Voir le commentaire au point d'usage plus bas.
uint16_t numeroSequence = 0;

#if MODE_TEST_PORTEE
uint32_t compteurTest = 0;   // incremente a chaque paquet de test envoye
#endif

// ============================================================
//  RESYNC ABSOLU EN FOND (voir INDEX_RESYNC_ABSOLU dans encoding.h)
//  Le lien LoRa n'a pas d'accuse de reception : un delta perdu en
//  l'air desynchronise le recepteur pour de bon sur ce capteur. On
//  fait tourner en fond un envoi ABSOLU de quelques capteurs par
//  cycle (en plus des deltas normaux), a tour de role sur tout
//  NB_SENSOR, pour qu'un capteur desynchronise se corrige tout seul
//  au prochain passage de son tour - sans attendre qu'il "bouge" a
//  nouveau naturellement.
// ============================================================
// 60 est volontairement plus grand que ce qui peut tenir dans la place
// restante d'un paquet (255o - deltas normaux) : serialiserResync() s'arrete
// proprement quand le budget est atteint (voir son garde-fou), donc ceci
// remplit vraiment chaque paquet au lieu de laisser un petit filet fixe de
// quelques octets (ancien probleme : RESYNC_PAR_CYCLE=4 ne remplissait
// jamais plus de ~16o alors que le paquet peut monter a 255o).
const uint16_t RESYNC_PAR_CYCLE = 60;
uint16_t curseurResync = 0;


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

    // --- Front-end RF (GC1109) AVANT la radio ---
    pinMode(FEM_VFEM, OUTPUT); digitalWrite(FEM_VFEM, HIGH);
    pinMode(FEM_CSD,  OUTPUT); digitalWrite(FEM_CSD,  HIGH);
    pinMode(FEM_CPS,  OUTPUT); digitalWrite(FEM_CPS,  HIGH);
    delay(10);

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
    // d'antenne TX/RX. Reactive : le desactiver n'a rien change au symptome
    // "antenne = pire qu'a vide" observe a bout portant -> ce comportement
    // vient probablement du champ proche/couplage direct carte-a-carte a
    // courte distance (non representatif), pas de ce reglage. Voir le test
    // de portee reelle en exterieur pour une mesure valide.
    if (radio.setDio2AsRfSwitch(true) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setDio2AsRfSwitch a echoue");
    }

    // --- TCXO (reference de frequence) ---
    // Le module SX1262 du Heltec V3/V4 utilise un TCXO (pas un simple XTAL).
    // radio.begin() configure deja 1.6V par defaut (parametre tcxoVoltage),
    // mais les exemples officiels Heltec pour cette carte utilisent 1.8V.
    // Un mauvais reglage ne fait pas planter l'init, mais peut degrader la
    // stabilite de frequence et donc la sensibilite reception/portee sans
    // erreur visible -> a tester si la portee reste anormalement faible.
    if (radio.setTCXO(1.8) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setTCXO(1.8) a echoue");
    }

    // --- Parametres RF ---
    // SF7 (etait 8) : environ 2x plus rapide, coute ~2.5dB de sensibilite.
    // Choisi apres confirmation que le lien tenait 3km sans perte a SF8 sur
    // autoroute, alors que le circuit de course (Trois-Rivieres) ne demande
    // que 730m - mais en milieu urbain avec batiments, donc marge gardee
    // prudente (SF7 seul, BW125 INCHANGE) plutot que d'empiler SF7+BW250.
    // DOIT rester identique cote recepteur (LoRa_Recepteur/main.cpp).
    // A RETESTER en conditions reelles (circuit, pas juste autoroute/champ)
    // avant la course.
    radio.setSpreadingFactor(7);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);

    // Diagnostic : verifie ce que le module accepte VRAIMENT comme puissance,
    // avant de l'appliquer. Si "clipped" != 22, la puissance demandee n'est
    // pas atteignable telle quelle sur ce module.
    int8_t puissanceClippee = 0;
    int16_t etatPuissance = radio.checkOutputPower(17, &puissanceClippee);
    Serial.print("checkOutputPower(17) -> etat=");
    Serial.print(etatPuissance);
    Serial.print(" puissance_reelle_possible=");
    Serial.println(puissanceClippee);

    int16_t etatSetPower = radio.setOutputPower(17);
    if (etatSetPower != RADIOLIB_ERR_NONE)
    {
        Serial.print("Attention : setOutputPower(17) a echoue, code=");
        Serial.println(etatSetPower);
    }

    // --- Limite de courant OCP ---
    // La protection de surintensite du SX1262 est calibree par defaut pour
    // ~10dBm. Sans la relever, setOutputPower(17) reussit (pas d'erreur)
    // mais la puissance REELLEMENT emise reste bridee par l'OCP pendant la
    // transmission -> deficit de portee silencieux, invisible dans les logs.
    // 140mA reste une marge suffisante pour +17dBm sur ce chip (calibree a
    // l'origine pour +22dBm, donc large a cette puissance reduite).
    if (radio.setCurrentLimit(140) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setCurrentLimit(140) a echoue");
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
        IDX_SPEED_REF, IDX_SPEED_MEASURE, IDX_SPEED,
        IDX_V_BAT, IDX_I_BAT,
        IDX_BATTERY_MAX_VOLTAGE, IDX_BATTERY_MIN_VOLTAGE, IDX_BATTERY_MAX_TEMP,
        IDX_MOTOR_TEMP, IDX_INVERTER_TEMP,
        IDX_CHARGER_HARDWARE_FAILURE, IDX_CHARGER_OVER_TEMP, IDX_CHARGER_WRONG_INPUT_VOLTAGE,
        IDX_CHARGER_BATTERY_UNDETECTED, IDX_CHARGER_COMM_ERROR,

        // Sortie CAN du dash AiM (0x500-0x505) : GPS speed + les 6 signaux IMU
        // (accel/gyro), demandes en priorite haute par l'utilisateur.
        IDX_AIM_GPS_SPEED,
        IDX_AIM_IMU_LON_ACC, IDX_AIM_IMU_LAT_ACC, IDX_AIM_IMU_VER_ACC,
        IDX_AIM_IMU_ROLL_RATE, IDX_AIM_IMU_PITCH_RATE, IDX_AIM_IMU_YAW_RATE,

        // Les ~78 signaux individuels des 5 modules BMS (tension/temperature
        // par cellule) sont volontairement PAS ici : laisses en priorite
        // secondaire (defaut). Ils bougent souvent (bruit ADC, seuil=1) et
        // remplissaient le paquet a chaque cycle, allongeant le temps
        // d'antenne et ralentissant TOUT (y compris les signaux ci-dessus qui
        // comptent vraiment pour la course). Ils continuent d'arriver via le
        // resync de fond (RESYNC_PAR_CYCLE) - juste pas garantis a chaque
        // cycle de 200ms comme avant.
    };
    for (unsigned int i = 0; i < sizeof(capteursPrioritaires) / sizeof(capteursPrioritaires[0]); i++)
    {
        encoder.setPriority(capteursPrioritaires[i], PRIORITE_HAUTE);
    }

    // ============================================================
    //  Capteurs IGNORES : jamais detectes, jamais resync, donc jamais
    //  transmis en LoRa (ex: capteur non cable, signal bruite/inutile).
    //  Pour en ignorer, decommente et complete la liste ci-dessous.
    // ============================================================
    // const SensorIndex capteursIgnores[] = {
    //     IDX_RESERVE_5, IDX_RESERVE_6,
    // };
    // for (unsigned int i = 0; i < sizeof(capteursIgnores) / sizeof(capteursIgnores[0]); i++)
    // {
    //     encoder.setIgnore(capteursIgnores[i], true);
    // }
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

        // (b) Detection des changements, SANS avancer la reference. Les DEUX
        //     priorites sont verifiees a CHAQUE cycle (200ms) : on veut
        //     remplir chaque paquet au maximum, pas juste envoyer les
        //     prioritaires et laisser les secondaires attendre. Les
        //     prioritaires sont ecrits EN PREMIER dans output_index/diff :
        //     si le paquet doit etre tronque en (c) faute de place, ce sont
        //     les secondaires en surplus qui passent leur tour (rien n'est
        //     perdu, previousValues n'avance qu'a la confirmation d'un envoi
        //     reussi -> retentes au prochain cycle), jamais les prioritaires.
        uint16_t nb_prioritaires = encoder.detecterParPriorite(sensors, PRIORITE_HAUTE, output_index, output_diff);
        uint16_t nb_secondaires = encoder.detecterParPriorite(sensors, PRIORITE_SECONDAIRE,
                                                               &output_index[nb_prioritaires],
                                                               &output_diff[nb_prioritaires]);

        nb_donnees = nb_prioritaires + nb_secondaires;

        // (c) Serialisation, plafonnee a la taille max d'un paquet LoRa
        //     (255o, cf TAILLE_PAQUET_LORA_MAX) : nb_capteurs_envoyes peut
        //     etre < nb_donnees si le budget d'octets est atteint.
        uint16_t nb_capteurs_envoyes = 0;
        // Plafond a 90o (pas 255) : un gros paquet (~225o observe) prend plus
        // de temps d'antenne que INTERVALLE_ENVOI (200ms) a SF8/BW125 -
        // radio.transmit() etant bloquant, le cycle reel tournait alors plus
        // lentement que prevu, ralentissant TOUT (pas juste le resync de
        // fond) - y compris les capteurs prioritaires qui changent vraiment.
        // 90o reste sous ce seuil sans toucher SF/BW (donc sans sacrifier la
        // portee validee sur le terrain).
        const uint16_t TAILLE_PAQUET_CIBLE = 90;
        uint16_t taille_max_paquet = TAILLE_PAQUET_CIBLE;
        if (sizeof(trame) < taille_max_paquet) taille_max_paquet = sizeof(trame);
        if (TAILLE_PAQUET_LORA_MAX < taille_max_paquet) taille_max_paquet = TAILLE_PAQUET_LORA_MAX;

        // (c0) Numero de sequence en tete (2 octets, LSB puis MSB) : le lien
        // LoRa n'a pas d'accuse de reception, donc c'est le seul moyen pour le
        // recepteur de detecter et chiffrer les paquets perdus en l'air (un
        // trou dans la sequence = un paquet jamais arrive). Incremente a
        // chaque tentative d'envoi reel, meme si le radio.transmit() echoue
        // localement - le recepteur voit les deux cas comme "jamais recu".
        trame[0] = (uint8_t)(numeroSequence & 0xFF);
        trame[1] = (uint8_t)(numeroSequence >> 8);
        uint16_t budgetDonnees = (taille_max_paquet > 2) ? (taille_max_paquet - 2) : 0;
        nb_octets = 2 + encoder.data_formater(nb_donnees, output_index, output_diff, &trame[2],
                                              budgetDonnees, &nb_capteurs_envoyes);

        // (c2) Resync absolu en fond : quelques capteurs de plus, a tour de
        //      role, dans la place qui reste dans CE MEME paquet (voir le
        //      commentaire de RESYNC_PAR_CYCLE plus haut).
        uint8_t indicesResync[RESYNC_PAR_CYCLE];
        for (uint8_t i = 0; i < RESYNC_PAR_CYCLE; i++)
        {
            indicesResync[i] = (uint8_t)((curseurResync + i) % NB_SENSOR);
        }
        uint16_t nb_resync_ecrits = 0;
        uint16_t budgetResync = (taille_max_paquet > nb_octets) ? (taille_max_paquet - nb_octets) : 0;
        nb_octets += encoder.serialiserResync(sensors, indicesResync, RESYNC_PAR_CYCLE,
                                              &trame[nb_octets], budgetResync, &nb_resync_ecrits);

        // (d) Transmission LoRa - seulement s'il y a quelque chose a envoyer.
        // nb_octets vaut au moins 2 (numero de sequence) meme sans donnees,
        // donc on transmet aussi les paquets "vides" pour ne pas casser la
        // continuite de sequence cote recepteur.
        if (nb_octets > 0)
        {
            int state = radio.transmit(trame, nb_octets);
            numeroSequence++;   // tente : compte meme si l'envoi echoue localement

            if (state == RADIOLIB_ERR_NONE)
            {
                compteur++;

                // On ne fait avancer previousValues QUE pour les capteurs
                // reellement transmis avec succes. Si l'envoi echoue, ou si
                // certains capteurs n'ont pas tenu dans le paquet, ils restent
                // "en attente" et seront retentes au prochain cycle - rien
                // n'est jamais perdu silencieusement.
                encoder.confirmerEnvoi(output_index, output_diff, nb_capteurs_envoyes);
                encoder.confirmerResync(sensors, indicesResync, nb_resync_ecrits);
                curseurResync = (curseurResync + nb_resync_ecrits) % NB_SENSOR;

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

        if (canOK)
        {
            diagCAN();

            // Diagnostic : debit reel de la trame CAN 81 (drive_speed_temp),
            // mesure directement sur le bus - INTERVALLE_AFFICHAGE vaut 1000ms
            // donc ce nombre est directement en trames/seconde.
            Serial.println(String("Trame CAN 81 (drive_speed_temp) : ")
                           + getCompteurTrame81() + " trames/s");
            resetCompteurTrame81();

            Serial.println(String("Trame CAN 82 (drive_electric_1, v_bat/i_bat) : ")
                           + getCompteurTrame82() + " trames/s");
            resetCompteurTrame82();
        }

        oled_reset_buffer();
        oled_ajouter_ligne(String("Nb capteurs: ") + nb_donnees);

        Serial.println(String("\nCapteurs modifies: ") + nb_donnees
                       + "  |  trame: " + nb_octets + " octets");
        Serial.println("-------------------------------");

        for (int i = 0; i < nb_donnees; i++)
        {
            // Valeur actuelle (post-modification), pas le delta envoye sur
            // le lien LoRa : sensors[] contient toujours la derniere lecture
            // CAN absolue, mise a jour a chaque cycle avant detecterParPriorite().
            String ligne = String(encoder.getConfigByIndex(output_index[i]).name)
                         + ", " + sensors[output_index[i]];

            Serial.println(ligne);
            oled_ajouter_ligne(ligne);
        }

        oled_afficher();
    }
}