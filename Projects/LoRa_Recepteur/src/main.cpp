#include <Arduino.h>
#include <RadioLib.h>
#include <SSD1306Wire.h>
#include "encoding.h"   // pour NB_SENSOR + TAILLE_TRAME_MAX + varint_decode + zigzag_decode


// ============================================================
//  MODE TEST DE PORTEE - DOIT avoir la MEME valeur que dans
//  LoRa_Comm/src/main.cpp, sinon le paquet compteur sera
//  (mal) interprete comme des capteurs, ou l'inverse.
// ============================================================
#define MODE_TEST_PORTEE 0

// ============================================================
//  MATERIEL - Heltec WiFi LoRa 32 V4 (ESP32-S3 + SX1262)
//  Memes broches que l'emetteur.
// ============================================================
SX1262 radio = new Module(8, 14, 12, 13);
SSD1306Wire display(0x3c, 17, 18);

// Front-end RF externe (ampli GC1109) : voir LoRa_Comm/main.cpp pour le
// detail. Doit etre active AVANT radio.begin(), aussi en reception (LNA).
#define FEM_VFEM 7
#define FEM_CSD  2
#define FEM_CPS  5

int paquetsRecus = 0;
volatile bool paquetRecu = false;

#if MODE_TEST_PORTEE
bool     premierPaquetTest  = true;
uint32_t dernierCompteurTest = 0;
uint32_t paquetsTestPerdus   = 0;
#endif

// Etat reconstruit cote reception.
// IMPORTANT : meme base de depart que l'emetteur (100), sinon
// les valeurs absolues seront decalees. On accumule les deltas
// dessus a mesure qu'ils arrivent.
// (int32 comme l'emetteur : un delta est signe, la valeur peut
//  monter ou descendre.)
int32_t valeurCourante[NB_SENSOR];

// Diagnostic : compte chaque fois que speed_ref change reellement de valeur
// (donc chaque fois qu'une vraie mise a jour arrive par LoRa, delta ou
// resync). Inclus dans le JSON pour le voir augmenter en direct sur le
// dashboard, en plus du flash deja en place sur la ligne du tableau.
uint32_t compteurMajSpeedRef = 0;
int32_t  dernierSpeedRef     = 0;
bool     premierSpeedRef     = true;

// Suivi du numero de sequence des paquets reels (hors mode test de portee,
// qui a deja son propre compteur/detection de perte). Voir le commentaire
// au point d'usage dans loop().
bool     premierPaquetReel  = true;
uint16_t dernierNumeroReel  = 0;
uint32_t paquetsPerdusReel  = 0;


// ============================================================
//  Interruption : DIO1 se declenche quand un paquet est recu.
//  On ne fait QUE lever un drapeau ici (regle d'or des ISR :
//  rapide, pas d'appels lourds). Le vrai travail se fait dans loop().
// ============================================================
void IRAM_ATTR onReceive()
{
    paquetRecu = true;
}


void setup()
{
    Serial.begin(115200);
    delay(200);

    // Base de reference identique a l'emetteur.
    for (int i = 0; i < NB_SENSOR; i++)
    {
        valeurCourante[i] = 0;
    }

    // --- Alimentation + reset OLED ---
    pinMode(36, OUTPUT); digitalWrite(36, LOW);  delay(100);
    pinMode(21, OUTPUT); digitalWrite(21, LOW);  delay(50);
    digitalWrite(21, HIGH); delay(50);

    display.init();
    display.clear();
    display.setFont(ArialMT_Plain_10);
#if MODE_TEST_PORTEE
    display.drawString(0, 0,  "TEST DE PORTEE");
    display.drawString(0, 15, "En attente...");
#else
    display.drawString(0, 0,  "Recepteur");
    display.drawString(0, 15, "En attente...");
#endif
    display.display();

    // --- Front-end RF (GC1109) AVANT la radio ---
    pinMode(FEM_VFEM, OUTPUT); digitalWrite(FEM_VFEM, HIGH);
    pinMode(FEM_CSD,  OUTPUT); digitalWrite(FEM_CSD,  HIGH);
    pinMode(FEM_CPS,  OUTPUT); digitalWrite(FEM_CPS,  HIGH);
    delay(10);

    // --- Init LoRa ---
    int state = radio.begin(915.0);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("Erreur LoRa: "); Serial.println(state);
        display.clear(); display.drawString(0, 0, "Erreur LoRa!"); display.display();
        while (true);
    }

    // --- RF switch (DIO2) ---
    // Reactive : voir LoRa_Comm/main.cpp pour le detail (le desactiver n'a
    // pas change le symptome a bout portant, probablement du champ proche).
    if (radio.setDio2AsRfSwitch(true) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setDio2AsRfSwitch a echoue");
    }

    // --- TCXO (reference de frequence) ---
    // Meme necessite que cote emetteur : radio.begin() met 1.6V par defaut,
    // les exemples officiels Heltec pour cette carte utilisent 1.8V. A tester
    // si la portee/sensibilite reste anormalement faible malgre le fix DIO2.
    if (radio.setTCXO(1.8) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setTCXO(1.8) a echoue");
    }

    // --- Parametres RF : DOIVENT etre identiques a l'emetteur ---
    // SF7 (etait 8) - voir le commentaire dans LoRa_Comm/main.cpp pour le
    // pourquoi. A retester en conditions reelles avant la course.
    radio.setSpreadingFactor(7);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setPreambleLength(8);
    radio.setSyncWord(0x12);      // meme valeur que l'emetteur
    radio.setCRC(true);           // meme valeur que l'emetteur

    // --- Gain RX boost ---
    // +3dB de sensibilite en reception (cout : ~+0.6mA). N'a de sens que
    // cote recepteur. A comparer au retest de portee AVEC le fix GC1109
    // deja applique, pour isoler l'apport de chaque changement.
    if (radio.setRxBoostedGainMode(true) != RADIOLIB_ERR_NONE)
    {
        Serial.println("Attention : setRxBoostedGainMode(true) a echoue");
    }

    // Mode reception par interruption.
    radio.setDio1Action(onReceive);
    radio.startReceive();
    Serial.println("Recepteur pret, en ecoute 915 MHz");
}


void loop()
{
    if (!paquetRecu)
    {
        return;   // rien a faire tant qu'aucun paquet
    }
    paquetRecu = false;

    // Buffer dimensionne EXACTEMENT comme la trame de l'emetteur,
    // via la constante partagee TAILLE_TRAME_MAX (encoding.h).
    // Emetteur et recepteur restent ainsi strictement symetriques :
    // un paquet plein ne sera jamais tronque a la reception.
    uint8_t buffer[TAILLE_TRAME_MAX];

    size_t len = radio.getPacketLength();        // longueur REELLE, variable
    if (len > sizeof(buffer)) len = sizeof(buffer);

    int state = radio.readData(buffer, len);

#if MODE_TEST_PORTEE
    if (state == RADIOLIB_ERR_NONE && len == 4)
    {
        paquetsRecus++;

        uint32_t compteurRecu = (uint32_t)buffer[0]
                               | ((uint32_t)buffer[1] << 8)
                               | ((uint32_t)buffer[2] << 16)
                               | ((uint32_t)buffer[3] << 24);

        // Detecte les paquets manquants : si le compteur saute (ex: 41 -> 44),
        // 2 paquets ont ete perdus en chemin. Suppose un ordre de reception
        // conserve (vrai en LoRa point-a-point, pas de reordonnancement).
        if (premierPaquetTest)
        {
            premierPaquetTest = false;
        }
        else
        {
            uint32_t attendu = dernierCompteurTest + 1;
            if (compteurRecu > attendu)
            {
                paquetsTestPerdus += (compteurRecu - attendu);
            }
        }
        dernierCompteurTest = compteurRecu;

        float rssi = radio.getRSSI();
        float snr  = radio.getSNR();

        Serial.println(String("Paquet test #") + compteurRecu
                       + "  RSSI=" + rssi + "dBm"
                       + "  SNR=" + snr + "dB"
                       + "  perdus_total=" + paquetsTestPerdus);

        display.clear();
        display.drawString(0, 0,  "TEST DE PORTEE");
        display.drawString(0, 15, "Paquet #" + String(compteurRecu));
        display.drawString(0, 27, "RSSI: " + String(rssi, 1) + " dBm");
        display.drawString(0, 39, "SNR: " + String(snr, 1) + " dB");
        display.drawString(0, 51, "Perdus: " + String(paquetsTestPerdus));
        display.display();
    }
    else
    {
        Serial.print("Erreur lecture paquet test: "); Serial.println(state);
    }

    radio.startReceive();
    return;
#else

    if (state == RADIOLIB_ERR_NONE && len >= 2)
    {
        paquetsRecus++;

        // ====================================================
        //  NUMERO DE SEQUENCE (2 premiers octets, LSB puis MSB) : seul moyen
        //  de detecter les paquets perdus en l'air, vu que le lien LoRa n'a
        //  pas d'accuse de reception. Un trou dans la sequence = un ou
        //  plusieurs paquets jamais arrives (gere le wraparound uint16_t
        //  naturellement via la soustraction non signee).
        // ====================================================
        uint16_t numeroRecu = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
        if (!premierPaquetReel)
        {
            uint16_t attendu = (uint16_t)(dernierNumeroReel + 1);
            if (numeroRecu != attendu)
            {
                uint16_t manques = (uint16_t)(numeroRecu - attendu);
                // Garde-fou : un ecart enorme (proche de 65536) ne veut pas
                // dire "65000+ paquets perdus" - a notre debit (quelques
                // paquets/s), ca prendrait des minutes. C'est bien plus
                // probablement l'emetteur qui a redemarre (sa sequence est
                // repartie de 0) pendant que le recepteur tournait deja. On
                // resynchronise silencieusement sans gonfler le compteur.
                if (manques < 1000)
                {
                    paquetsPerdusReel += manques;
                }
            }
        }
        else
        {
            premierPaquetReel = false;
        }
        dernierNumeroReel = numeroRecu;

        // ====================================================
        //  DECODAGE zigzag + varint
        //  Format recu (apres les 2 octets de sequence) : [index][delta
        //  varint]... normalement, SAUF si index == INDEX_RESYNC_ABSOLU
        //  (0xFF) : dans ce cas c'est [0xFF][index reel][valeur ABSOLUE
        //  varint] - voir le commentaire d'INDEX_RESYNC_ABSOLU dans
        //  encoding.h. Ca corrige tout seul un capteur desynchronise par un
        //  delta perdu par le passe, sans attendre qu'il rebouge.
        //  Pas de compteur de champs en tete : on lit jusqu'a epuiser 'len'.
        // ====================================================
        size_t pos = 2;
        while (pos < len)
        {
            uint8_t idx = buffer[pos++];         // 1 octet : quel capteur

            if (idx == INDEX_RESYNC_ABSOLU)
            {
                // Il faut encore 1 octet d'index reel + au moins 1 de varint.
                if (pos >= len) break;
                uint8_t idxReel = buffer[pos++];

                if (pos >= len) break;
                uint32_t zz;
                pos += varint_decode(&buffer[pos], len - pos, &zz);
                int32_t valeurAbsolue = zigzag_decode(zz);

                if (idxReel < NB_SENSOR)
                {
                    valeurCourante[idxReel] = valeurAbsolue;   // SET, pas +=
                }
                continue;
            }

            // Securite : il faut au moins 1 octet de varint apres l'index.
            if (pos >= len) break;

            uint32_t zz;
            // varint_decode borne desormais sa lecture par le nombre
            // d'octets restants (len - pos) : si le paquet est corrompu
            // (drapeau 0x80 jamais relache), on ne lit jamais hors buffer.
            pos += varint_decode(&buffer[pos], len - pos, &zz);   // avance du nb d'octets lus
            int32_t delta = zigzag_decode(zz);                    // retrouve le signe

            if (idx < NB_SENSOR)
            {
                valeurCourante[idx] += delta;    // reconstruction de la valeur absolue
            }
        }

        // Diagnostic : speed_ref a-t-il reellement change avec ce paquet ?
        if (premierSpeedRef || valeurCourante[IDX_SPEED_REF] != dernierSpeedRef)
        {
            compteurMajSpeedRef++;
            dernierSpeedRef = valeurCourante[IDX_SPEED_REF];
            premierSpeedRef = false;
        }

        // ====================================================
        //  Sortie JSON (pour ton dashboard Flask)
        //  Une cle par capteur, dans l'ordre de kSensorNames (meme table
        //  que l'emetteur -> noms toujours synchronises). Valeurs BRUTES :
        //  le scale/offset du DBC reste a appliquer cote dashboard.
        // ====================================================
        Serial.print("{");
        for (int i = 0; i < NB_SENSOR; i++)
        {
            Serial.print("\"");
            Serial.print(kSensorNames[i]);
            Serial.print("\":");
            Serial.print(valeurCourante[i]);
            Serial.print(",");
        }
        Serial.print("\"rssi\":");
        Serial.print(radio.getRSSI());
        Serial.print(",\"paquets_recus\":");
        Serial.print(paquetsRecus);
        Serial.print(",\"paquets_perdus\":");
        Serial.print(paquetsPerdusReel);
        Serial.print(",\"speed_ref_maj\":");
        Serial.print(compteurMajSpeedRef);
        Serial.println("}");

        // ====================================================
        //  Affichage OLED
        // ====================================================
        display.clear();
        display.drawString(0, 0,  "Paquet #" + String(paquetsRecus));
        display.drawString(0, 15, "tire: "  + String(valeurCourante[0]));
        display.drawString(0, 27, "speed: " + String(valeurCourante[3]));
        display.drawString(0, 39, "temp: "  + String(valeurCourante[4]));
        display.drawString(0, 51, "perdus: " + String(paquetsPerdusReel));
        display.display();
    }
    else
    {
        Serial.print("Erreur lecture paquet: "); Serial.println(state);
    }
#endif

    // On se remet en ecoute pour le prochain paquet.
    radio.startReceive();
}