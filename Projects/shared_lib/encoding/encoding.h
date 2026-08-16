#ifndef ENCODING_H
#define ENCODING_H


#include <stdint.h>
#include <stdbool.h>
#include "capteurs.h"         // Source UNIQUE de l'enum SensorIndex et NB_SENSOR


// ============================================================================
//  NOTE : l'enum SensorIndex et NB_SENSOR ne sont PLUS definis ici.
//  Ils vivent desormais uniquement dans capteurs.h (source unique de verite),
//  inclus juste au-dessus. Les redefinir ici causait une erreur
//  "multiple definition of 'enum SensorIndex'".
// ============================================================================

#define SENSOR_NAME_MAX_LEN 16  // Longueur maximale du nom d'un capteur en caractères

// --- Plages d'identifiants pour ton bus CAN ---
#define CAN_SENSOR_ID_MIN 0x410
#define CAN_SENSOR_ID_MAX 0x45F

// --- Dimensionnement du buffer de trame ---
// Pire cas par capteur : 1 octet d'index + varint du delta.
// ATTENTION : un varint LEB128 d'un uint32 peut occuper jusqu'a 5 octets
// (32 bits / 7 bits par octet = 4,57 -> 5 octets), PAS 4. Donc le vrai
// pire cas est 1 + 5 = 6 octets par capteur. Dimensionne toujours ton
// buffer de trame avec MAX_OCTETS_PAR_CAPTEUR, jamais avec une constante
// codee en dur.
#define VARINT_MAX_OCTETS       5                          // taille max d'un varint uint32
#define MAX_OCTETS_PAR_CAPTEUR  (1 + VARINT_MAX_OCTETS)    // 1 index + varint = 6
#define TAILLE_TRAME_MAX        (MAX_OCTETS_PAR_CAPTEUR * NB_SENSOR)

// Limite dure du module radio (SX126x, verifiee dans RadioLib :
// RADIOLIB_SX126X_MAX_PACKET_LENGTH = 255). Aucun paquet LoRa ne doit
// depasser cette taille, quel que soit TAILLE_TRAME_MAX.
#define TAILLE_PAQUET_LORA_MAX  255

// --- Niveaux de priorite (2 niveaux) ---
// PRIORITE_HAUTE   : envoye des qu'il change, a chaque cycle d'envoi.
// PRIORITE_SECONDAIRE : accumule, envoye seulement toutes les
//                       INTERVALLE_ENVOI_SECONDAIRE ms (voir main.cpp).
#define PRIORITE_HAUTE       0
#define PRIORITE_SECONDAIRE  1

// --- Marqueur de resync absolu ---
// Le lien LoRa n'a aucun accuse de reception : si un paquet portant un
// delta est perdu en l'air, l'emetteur croit l'avoir envoye (previousValues
// avance) mais le recepteur reste bloque sur l'ancienne valeur pour
// toujours, jusqu'au prochain vrai changement de CE capteur precis. Pour
// s'auto-corriger, l'emetteur fait tourner en fond un envoi ABSOLU (pas un
// delta) de chaque capteur a tour de role : [INDEX_RESYNC_ABSOLU][index
// reel][valeur absolue en zigzag+varint]. NB_SENSOR reste toujours << 255,
// donc cette valeur ne collisionne jamais avec un vrai index de capteur.
#define INDEX_RESYNC_ABSOLU 0xFF


// ============================================================================
//  STRUCTURES ET CLASSIFICATION POUR LA COMPRESSION DELTA
// ============================================================================

// Structure stockant la configuration propre à chaque capteur
typedef struct
{
    char name[SENSOR_NAME_MAX_LEN]; // Nom lisible du capteur (ex: "vitesse")
    int threshold;                  // Seuil de tolérance (écart min pour envoyer)
    uint8_t priority;               // Niveau de priorité de la donnée
    int last_sent;                  // Dernière valeur envoyée (pour calculer le prochain delta)
    bool ignored;                   // Si vrai : jamais detecte ni resync, donc jamais transmis
} SensorConfig;

// Classe principale gérant l'algorithme d'encodage par Delta
class DeltaEncoder
{
public:
    DeltaEncoder();

    // Initialise tous les capteurs avec les valeurs de départ actuelles.
    // Paramètres :
    // - sensors : Tableau contenant les valeurs initiales de chaque capteur
    void init(const int sensors[]);

    // Détecte les capteurs qui ont été modifiés au-delà de leur seuil configuré.
    // Paramètres :
    // - sensors : Tableau des valeurs actuelles (ex: lues sur le CAN)
    // - out_index_capteur : [Sortie] Tableau rempli avec les index des capteurs modifiés
    // - diff_capteur : [Sortie] Tableau rempli avec le delta (signé) de chaque modification
    // Retourne : Le nombre total de capteurs qui ont bougé.
    //
    // ATTENTION : avance previousValues[] immediatement, avant meme de savoir
    // si l'envoi va reussir. Conserve pour compatibilite, mais prefere
    // detecterParPriorite() + confirmerEnvoi() pour un pipeline qui ne perd
    // pas de donnees si la transmission echoue (voir plus bas).
    uint16_t sorting(const int sensors[], uint8_t out_index_capteur[], int diff_capteur[]);

    // Detecte les capteurs d'un niveau de priorite donne (PRIORITE_HAUTE ou
    // PRIORITE_SECONDAIRE) dont le delta depasse leur seuil. NE MODIFIE PAS
    // previousValues : contrairement a sorting(), la reference n'avance
    // qu'apres un appel explicite a confirmerEnvoi() (donc apres un envoi
    // radio reussi). Permet d'accumuler plusieurs cycles sans rien perdre.
    // Retourne le nombre de capteurs detectes (ecrits dans out_index/out_diff).
    uint16_t detecterParPriorite(const int sensors[], uint8_t priorityFiltre,
                                 uint8_t out_index[], int out_diff[]);

    // Avance previousValues[] et last_sent UNIQUEMENT pour les capteurs listes
    // ici (indices/diffs). A appeler seulement apres un radio.transmit()
    // reussi, avec exactement les capteurs reellement inclus dans le paquet
    // envoye (voir out_nb_capteurs_ecrits de data_formater()).
    void confirmerEnvoi(const uint8_t indices[], const int diffs[], uint16_t count);

    // Sérialise (compresse) les capteurs modifiés en un flux d'octets optimisé :
    // Pour chaque capteur changé -> [1 octet d'index] + [le delta encodé en zigzag + varint].
    // Paramètres :
    // - nb_donnees : Nombre de capteurs modifiés (provenant du retour de sorting())
    // - output_index / output_diff : Tableaux générés par la fonction sorting()
    // - out_buf : Buffer de sortie qui recevra les octets
    // - out_buf_taille : Taille reelle du buffer out_buf, pour eviter tout debordement
    // - out_nb_capteurs_ecrits : [Sortie, optionnel] nombre de capteurs REELLEMENT
    //   serialises avant que le buffer soit plein (peut etre < nb_donnees si le
    //   budget d'octets a ete atteint). Sert a savoir quel prefixe de
    //   output_index/output_diff confirmer via confirmerEnvoi().
    // Retourne : Le nombre d'OCTETS réels écrits dans out_buf (la taille de ta trame LoRa).
    uint16_t data_formater(uint16_t nb_donnees, uint8_t output_index[], int output_diff[],
                           uint8_t out_buf[], uint16_t out_buf_taille,
                           uint16_t* out_nb_capteurs_ecrits = nullptr);

    // Serialise un resync ABSOLU (pas un delta) pour chaque capteur listé
    // dans indices[] : [INDEX_RESYNC_ABSOLU][index réel][valeur absolue en
    // zigzag+varint]. Voir le commentaire d'INDEX_RESYNC_ABSOLU plus haut.
    // S'arrête proprement si out_buf_taille est atteint (out_nb_ecrits dit
    // combien ont réellement été écrits, à passer ensuite à confirmerResync()).
    uint16_t serialiserResync(const int sensors[], const uint8_t indices[], uint16_t nbIndices,
                              uint8_t out_buf[], uint16_t out_buf_taille,
                              uint16_t* out_nb_ecrits = nullptr);

    // Avance previousValues[] (à la valeur ABSOLUE, pas un delta) pour les
    // capteurs listés ici. À appeler après un radio.transmit() réussi, avec
    // exactement les capteurs réellement inclus (out_nb_ecrits ci-dessus).
    void confirmerResync(const int sensors[], const uint8_t indices[], uint16_t count);

    // --- Fonctions d'accès (Getters / Setters) ---
    void setName(uint8_t sensorNumber, const char sensorName[]);
    void setThreshold(uint8_t sensorNumber, int threshold);
    void setPriority(uint8_t sensorNumber, int isPriority);

    // Exclut (ou reinclut) un capteur de toute transmission : il ne sera
    // plus jamais retenu par detecterParPriorite() ni serialiserResync()
    // tant que ignore vaut true. Utile pour couper un capteur non cable/
    // bruyant sans toucher au reste du pipeline.
    void setIgnore(uint8_t sensorNumber, bool ignore);

    SensorConfig getConfigByIndex(uint8_t index) const;

private:
    int previousValues[NB_SENSOR];   // Historique des valeurs de tous les capteurs
    SensorConfig configs[NB_SENSOR]; // Configuration de tous les capteurs
    bool isInitialized;              // Permet de savoir si l'encodeur a démarré
};


// ============================================================================
//  FONCTIONS D'ENCODAGE BAS NIVEAU (Mécanique binaire)
// ============================================================================

// Convertit un entier signé en entier non signé (Ex: -1 devient 1, 1 devient 2, -2 devient 3)
// Permet de s'assurer que les petites magnitudes (proches de 0) restent de toutes petites valeurs.
uint32_t zigzag_encode(int32_t n);

// Opération inverse de zigzag_encode (Retrouve le vrai signe + ou - du delta)
int32_t zigzag_decode(uint32_t z);

// Écrit un entier non signé au format Varint (LEB128) dans un buffer d'octets.
// Compresse la donnée sur 1 à 5 octets selon sa taille. Retourne le nombre d'octets écrits.
uint8_t varint_encode(uint32_t value, uint8_t buf[]);

// Lit un flux d'octets au format Varint depuis un buffer et extrait la vraie valeur dans *out.
// Retourne le nombre d'octets lus dans le buffer pour faire avancer ton pointeur de lecture.
// buf_taille borne la lecture pour ne jamais depasser le buffer si les donnees sont corrompues.
uint8_t varint_decode(const uint8_t buf[], uint16_t buf_taille, uint32_t* out);


// ============================================================================
//  FONCTIONS UTILITAIRES SYSTÈME
// ============================================================================

// Fait la correspondance entre un ID reçu sur le bus CAN et l'index de ton tableau.
// Retourne -1 si l'ID CAN se trouve en dehors de la plage valide [0x410, 0x45F].
int16_t sensorIndexFromCanId(uint16_t canId);


#endif // ENCODING_H