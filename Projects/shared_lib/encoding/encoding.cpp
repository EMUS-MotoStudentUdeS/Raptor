#include "encoding.h"

#include <string.h>
#include <stdlib.h>
#include <Arduino.h>



// ============================================================
//  Configuration par defaut des capteurs
//  (nom, seuil, priorite, derniere valeur envoyee)
//  Le seuil = amplitude minimale d'un changement pour qu'un
//  capteur soit juge "modifie" et donc transmis.
// ============================================================
static const SensorConfig DefaultConfigs[NB_SENSOR] = {
    // name      | threshold | priority | last_sent
    {"sensor_0", 1, 0, 0},
    {"sensor_1", 1, 0, 0},
    {"sensor_2", 1, 0, 0},
    {"sensor_3", 1, 0, 0},
    {"sensor_4", 1, 0, 0},
    {"sensor_5", 1, 0, 0},
    {"sensor_6", 1, 0, 0},
    {"sensor_7", 1, 0, 0},
    {"sensor_8", 1, 0, 0},
    {"sensor_9", 1, 0, 0}
};


// ------------------------------------------------------------
//  Constructeur : on part avec des valeurs precedentes a zero
//  et la config par defaut. isInitialized reste faux jusqu'au
//  premier appel a init() (ou au premier sorting()).
// ------------------------------------------------------------
DeltaEncoder::DeltaEncoder() : isInitialized(false)
{
    memset(previousValues, 0, sizeof(previousValues));
    memcpy(configs, DefaultConfigs, sizeof(configs));
}


// ------------------------------------------------------------
//  init : fixe la "base" de reference. Les deltas suivants
//  seront calcules par rapport a ces valeurs-la.
// ------------------------------------------------------------
void DeltaEncoder::init(const int sensors[])
{
    for (uint8_t i = 0; i < NB_SENSOR; i++)
    {
        previousValues[i] = sensors[i];
    }

    isInitialized = true;
}


// ============================================================
//  Setters / getters de configuration
// ============================================================

void DeltaEncoder::setName(uint8_t sensorNumber, const char sensorName[])
{
    // Protection contre un index invalide.
    if (sensorNumber >= NB_SENSOR)
    {
        return;
    }

    // strncpy borne la copie ; on force le terminateur nul au cas ou
    // le nom source serait plus long que le buffer.
    strncpy(configs[sensorNumber].name, sensorName, SENSOR_NAME_MAX_LEN - 1);
    configs[sensorNumber].name[SENSOR_NAME_MAX_LEN - 1] = '\0';
}

void DeltaEncoder::setThreshold(uint8_t sensorNumber, int threshold)
{
    // Protection contre un index invalide.
    if (sensorNumber >= NB_SENSOR)
    {
        return;
    }

    // On stocke la valeur absolue : un seuil est une amplitude,
    // jamais negatif.
    configs[sensorNumber].threshold = abs(threshold);
}

void DeltaEncoder::setPriority(uint8_t sensorNumber, int isPriorityValue)
{
    // Protection contre un index invalide.
    if (sensorNumber >= NB_SENSOR)
    {
        return;
    }

    // Priorite sur 2 bits (0 a 3).
    configs[sensorNumber].priority = (uint8_t)isPriorityValue & 0x03;
}

void DeltaEncoder::setIgnore(uint8_t sensorNumber, bool ignore)
{
    // Protection contre un index invalide.
    if (sensorNumber >= NB_SENSOR)
    {
        return;
    }

    configs[sensorNumber].ignored = ignore;
}

SensorConfig DeltaEncoder::getConfigByIndex(uint8_t index) const
{
    // Protection contre un index invalide.
    if (index >= NB_SENSOR)
    {
        return SensorConfig{};
    }

    return configs[index];
}


// ============================================================
//  sorting : le coeur du delta encoding.
//  Pour chaque capteur, on compare sa valeur actuelle a la
//  derniere valeur retenue. Si l'ecart depasse le seuil, on
//  garde ce capteur (son index + son delta signe) et on met
//  a jour la reference.
//
//  Sortie :
//   - out_index_capteur[] : index des capteurs modifies
//   - diff_capteur[]      : delta signe correspondant
//  Retour : nombre de capteurs modifies.
// ============================================================
uint16_t DeltaEncoder::sorting(const int sensors[], uint8_t out_index_capteur[], int diff_capteur[])
{
    uint16_t outIndex = 0;

    // Au tout premier appel, on n'a pas encore de base : on
    // l'initialise avec les valeurs recues (aucun delta ce tour-ci).
    if (!isInitialized)
    {
        init(sensors);
    }

    for (uint8_t i = 0; i < NB_SENSOR; i++)
    {
        int old_value = previousValues[i];
        int now_value = sensors[i];

        int diff = now_value - old_value;   // delta signe (peut etre negatif)

        // On ne transmet que si le changement est assez grand.
        if (abs(diff) >= configs[i].threshold)
        {
            configs[i].last_sent = now_value;

            out_index_capteur[outIndex] = i;      // quel capteur a change
            diff_capteur[outIndex]      = diff;   // de combien (signe)

            previousValues[i] = now_value;        // la nouvelle reference
            outIndex++;
        }
    }

    return outIndex;
}


// ============================================================
//  detecterParPriorite / confirmerEnvoi : variante de sorting()
//  qui separe prioritaires/secondaires et ne s'engage sur la
//  reference (previousValues) qu'apres un envoi confirme reussi.
//  Voir l'entete pour le detail du contrat.
// ============================================================
uint16_t DeltaEncoder::detecterParPriorite(const int sensors[], uint8_t priorityFiltre,
                                           uint8_t out_index[], int out_diff[])
{
    uint16_t outIndex = 0;

    if (!isInitialized)
    {
        init(sensors);
    }

    for (uint8_t i = 0; i < NB_SENSOR; i++)
    {
        if (configs[i].priority != priorityFiltre)
        {
            continue;
        }

        if (configs[i].ignored)
        {
            continue;
        }

        int diff = sensors[i] - previousValues[i];

        if (abs(diff) >= configs[i].threshold)
        {
            out_index[outIndex] = i;
            out_diff[outIndex]  = diff;
            outIndex++;
        }
    }

    return outIndex;
}

void DeltaEncoder::confirmerEnvoi(const uint8_t indices[], const int diffs[], uint16_t count)
{
    for (uint16_t i = 0; i < count; i++)
    {
        uint8_t idx = indices[i];

        if (idx >= NB_SENSOR)
        {
            continue;
        }

        previousValues[idx] += diffs[i];
        configs[idx].last_sent = previousValues[idx];
    }
}


// ============================================================
//  ENCODAGE BAS NIVEAU : zigzag + varint
//  But : transformer un delta signe en le moins d'octets
//  possible, sans jamais perdre le signe ni saturer.
// ============================================================

// --- Zigzag ---
// Mappe un entier signe vers un non signe en gardant les
// petites magnitudes petites :
//   0 -> 0 | -1 -> 1 | 1 -> 2 | -2 -> 3 | 2 -> 4 ...
// C'est ce qui rend le varint efficace : un petit delta,
// positif ou negatif, donne une petite valeur non signee.
uint32_t zigzag_encode(int32_t n)
{
    // CORRECTION : le decalage " << 1 " doit se faire sur un type
    // NON signe. Faire "n << 1" sur un int32_t negatif (ou dont le
    // bit 30 est a 1) est un comportement indefini en C/C++
    // (debordement de type signe). On caste donc n en uint32_t
    // AVANT de decaler.
    //   - (uint32_t)n << 1     : la magnitude, decalee proprement
    //   - (uint32_t)(n >> 31)  : 0x00000000 si positif, 0xFFFFFFFF si
    //                            negatif (decalage arithmetique du signe)
    // Le XOR des deux produit le motif zigzag attendu.
    return ((uint32_t)n << 1) ^ (uint32_t)(n >> 31);
}

// Operation inverse du zigzag.
int32_t zigzag_decode(uint32_t z)
{
    // (z >> 1) recupere la magnitude ; ~(z & 1) + 1 vaut 0xFFFFFFFF
    // si le bit de poids faible etait a 1 (donc negatif), 0 sinon.
    // Le XOR applique la negation seulement dans ce cas.
    return (int32_t)((z >> 1) ^ (~(z & 1) + 1));
}


// --- Varint (LEB128) ---
// Encode une valeur non signee sur un nombre variable d'octets :
// 7 bits de donnee par octet, + 1 bit de poids fort "continue".
// Petites valeurs -> 1 octet, grandes -> 2 a 5 octets.
// Retourne le nombre d'octets ecrits.
uint8_t varint_encode(uint32_t value, uint8_t buf[])
{
    uint8_t i = 0;

    // Tant qu'il reste plus de 7 bits a coder, on emet un octet
    // avec le drapeau "continue" (0x80) arme.
    while (value >= 0x80)
    {
        buf[i++] = (value & 0x7F) | 0x80;
        value >>= 7;
    }

    // Dernier octet : drapeau a 0, il signale la fin.
    buf[i++] = value & 0x7F;

    return i;
}

// Lit un varint depuis buf, ecrit la valeur decodee dans *out.
// Retourne le nombre d'octets lus.
//
// CORRECTION : on borne la lecture par buf_taille ET par la taille
// max d'un varint uint32 (VARINT_MAX_OCTETS). Sans ca, un buffer
// corrompu dont le drapeau 0x80 n'est jamais relache ferait lire la
// boucle indefiniment au-dela du buffer -> lecture memoire hors bornes.
uint8_t varint_decode(const uint8_t buf[], uint16_t buf_taille, uint32_t* out)
{
    uint32_t result = 0;
    uint8_t  shift  = 0;
    uint8_t  i      = 0;
    uint8_t  byte;

    do
    {
        // Garde-fou : on s'arrete si on atteint la fin du buffer ou
        // la longueur max d'un varint. Evite de lire hors bornes.
        if (i >= buf_taille || i >= VARINT_MAX_OCTETS)
        {
            break;
        }

        byte = buf[i++];
        // On recolle 7 bits a la fois, du moins significatif au plus.
        result |= (uint32_t)(byte & 0x7F) << shift;
        shift  += 7;
    } while (byte & 0x80);   // on continue tant que le drapeau est arme

    *out = result;
    return i;
}


// ============================================================
//  data_formater : serialise les capteurs modifies en octets
//  prets a transmettre. Pour chaque capteur :
//     [ index (1 octet) ][ delta en zigzag + varint (1 a 5 o) ]
//
//  Ecrit dans out_buf (fourni par l'appelant -> pas de globale,
//  donc reentrant) et retourne le NOMBRE D'OCTETS ecrits.
//
//  CORRECTION : on passe desormais la taille reelle du buffer
//  (out_buf_taille) et on verifie AVANT chaque capteur qu'il
//  reste au moins MAX_OCTETS_PAR_CAPTEUR d'espace. Si le buffer
//  est plein, on s'arrete proprement plutot que d'ecrire hors
//  bornes. Dimensionne le buffer avec TAILLE_TRAME_MAX.
// ============================================================
uint16_t DeltaEncoder::data_formater(uint16_t nb_donnees,
                                     uint8_t output_index[],
                                     int output_diff[],
                                     uint8_t out_buf[],
                                     uint16_t out_buf_taille,
                                     uint16_t* out_nb_capteurs_ecrits)
{
    uint16_t pos = 0;
    uint16_t i;

    for (i = 0; i < nb_donnees; i++)
    {
        // Garde-fou : il faut au moins de quoi ecrire 1 index + le
        // varint le plus long possible. Sinon on arrete la serialisation
        // pour ne jamais deborder out_buf (ou depasser TAILLE_PAQUET_LORA_MAX
        // si out_buf_taille a ete plafonne a cette valeur par l'appelant).
        if (pos + MAX_OCTETS_PAR_CAPTEUR > out_buf_taille)
        {
            break;
        }

        out_buf[pos++] = output_index[i];               // index du capteur

        uint32_t zz = zigzag_encode(output_diff[i]);    // signe gere proprement
        pos += varint_encode(zz, &out_buf[pos]);        // taille variable
    }

    if (out_nb_capteurs_ecrits != nullptr)
    {
        *out_nb_capteurs_ecrits = i;   // combien de capteurs ont reellement fini dans out_buf
    }

    return pos;
}


// ============================================================
//  serialiserResync / confirmerResync : voir INDEX_RESYNC_ABSOLU
//  dans encoding.h. Format par capteur : [0xFF][index reel][valeur
//  absolue en zigzag+varint] - 2 a 7 octets, contre 2 a 6 pour un
//  delta normal (1 octet d'index de plus, car 0xFF prend la place
//  du vrai index).
// ============================================================
uint16_t DeltaEncoder::serialiserResync(const int sensors[], const uint8_t indices[], uint16_t nbIndices,
                                        uint8_t out_buf[], uint16_t out_buf_taille,
                                        uint16_t* out_nb_ecrits)
{
    uint16_t pos = 0;
    uint16_t i;

    for (i = 0; i < nbIndices; i++)
    {
        uint8_t idx = indices[i];

        // Capteur ignore : on saute sans consommer de budget d'octets ni
        // interrompre la boucle (les suivants du tour de role restent testes).
        if (idx < NB_SENSOR && configs[idx].ignored)
        {
            continue;
        }

        // Garde-fou : 1 octet marqueur + 1 octet index + varint le plus long.
        if (pos + 2 + VARINT_MAX_OCTETS > out_buf_taille)
        {
            break;
        }

        out_buf[pos++] = INDEX_RESYNC_ABSOLU;
        out_buf[pos++] = idx;

        uint32_t zz = zigzag_encode(sensors[idx]);   // valeur ABSOLUE, pas un delta
        pos += varint_encode(zz, &out_buf[pos]);
    }

    if (out_nb_ecrits != nullptr)
    {
        *out_nb_ecrits = i;
    }

    return pos;
}

void DeltaEncoder::confirmerResync(const int sensors[], const uint8_t indices[], uint16_t count)
{
    for (uint16_t i = 0; i < count; i++)
    {
        uint8_t idx = indices[i];

        if (idx >= NB_SENSOR)
        {
            continue;
        }

        previousValues[idx] = sensors[idx];       // valeur ABSOLUE, pas +=
        configs[idx].last_sent = sensors[idx];
    }
}





// ============================================================
//  sensorIndexFromCanId : convertit un ID CAN en index de
//  capteur. Retourne -1 si l'ID est hors de la plage geree.
// ============================================================
int16_t sensorIndexFromCanId(uint16_t canId)
{
    /*
      Tous les IDs entre 0x410 et 0x45F (inclusif)
      ID 4  : ecu_sensors
      ID 6  : ecu_battery
      ID 9  : ecu_modes
      ID 81 : drive_speed_temp
      ID 82 : drive_electric_1
      ID 84 : drive_electric_2

      ATTENTION : la plage 0x410-0x45F fait 80 IDs possibles,
      alors que NB_SENSOR vaut 10. La soustraction lineaire
      ci-dessous ne suffira pas quand tu auras plus de capteurs
      ou des IDs non contigus -> il faudra une vraie table de
      correspondance ID -> index. Pour l'instant on borne au
      moins l'acces (voir decode_CAN) pour eviter les crashs.
    */

    if (canId < CAN_SENSOR_ID_MIN || canId > CAN_SENSOR_ID_MAX)
    {
        return -1;
    }

    return (int16_t)(canId - CAN_SENSOR_ID_MIN);
}


// ============================================================
//  decode_CAN : extrait (ID, valeur) d'une trame CAN et range
//  la valeur dans le tableau des capteurs.
//
//  Version simplifiee pour le prototype : CAN_data[0] = ID,
//  CAN_data[1] = valeur. A remplacer par un vrai parsing de
//  trame quand le MCP2515 fournira les donnees reelles.
//
//  Correction importante par rapport a la 1re version :
//  on VERIFIE que l'index est valide avant d'ecrire, sinon
//  un ID hors plage (index -1) ou trop grand corromprait la
//  memoire hors du tableau sensors[].
// ============================================================
void decode_CAN(const int CAN_data[], int sensors[])
{
    int id = CAN_data[0];

    int16_t sensorIndex = sensorIndexFromCanId(id);

    // Garde-fou : on ignore les trames dont l'index tombe hors
    // du tableau [0, NB_SENSOR-1].
    if (sensorIndex < 0 || sensorIndex >= NB_SENSOR)
    {
        return;
    }

    sensors[sensorIndex] = CAN_data[1];
}