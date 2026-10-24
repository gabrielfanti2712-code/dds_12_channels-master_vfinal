//*****************************************************************************
// Projet DDS multivoies — TM4C1294 + 3 x AD9959
// Laboratoire de Physique des Lasers / CNRS
//*****************************************************************************
//
// FICHIER PRINCIPAL DU MICROCONTROLEUR
// -----------------------------------
// Ce fichier gère le matériel du rack et les fonctions générales :
//   - initialisation horloge, GPIO, SSI, UART et timers ;
//   - réception des commandes série du GUI ;
//   - single tones, STOP et RESET ;
//   - détection des TTL6/TTL7/TTL8 ;
//   - aiguillage des interruptions vers le moteur de tables.
//
// Pour les séquences Hybrid 2.0, le code temps réel principal est dans
// ``seq_table_engine.c``. Les valeurs de registres AD9959 sont calculées sur
// le PC avant LOAD : les ISR du TM4C ne font pas de calcul flottant complexe.
//
// Une partie du moteur général historique est encore présente parce qu'elle
// reste utilisée par des chemins de compatibilité. Elle ne doit pas être
// supprimée sans vérifier le protocole UART et les modes spéciaux associés.
//
// Référence firmware : V36.5 (signature g_tbl365_signature = 1446196741).
//*****************************************************************************

// -----------------------------------------------------------------------------
// --- INCLUDES TIVAWARE (TM4C1294) ---
// -----------------------------------------------------------------------------
#include <stdbool.h> // Standard boolean
#include <stdint.h> // Standard integers
#include <stdio.h> // Standard integers
#include <stdlib.h>
#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"
#include "inc/hw_timer.h"
#include "inc/hw_memmap.h" // Base addresses
#include "inc/hw_sysctl.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/flash.h"
#include "driverlib/gpio.h"  // GPIO control register offsets
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"

/*
 * Compatibilite TivaWare ancienne revision.
 * Certaines versions de driverlib/timer.h s'arretent a TIMER_5x_SYNC,
 * alors que le TM4C1294 implemente aussi GPTM6 et GPTM7 dans GPTMSYNC.
 * Les champs SYNCT6 et SYNCT7 occupent respectivement les bits 13:12
 * et 15:14 ; la valeur 1 selectionne Timer A.
 */
#ifndef TIMER_6A_SYNC
#define TIMER_6A_SYNC 0x00001000UL
#endif
#ifndef TIMER_6B_SYNC
#define TIMER_6B_SYNC 0x00002000UL
#endif
#ifndef TIMER_7A_SYNC
#define TIMER_7A_SYNC 0x00004000UL
#endif
#ifndef TIMER_7B_SYNC
#define TIMER_7B_SYNC 0x00008000UL
#endif
#include "driverlib/pin_map.h" // Mapping of peripherals to pins for all parts

#include "utils/locator.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

#include "lwiplib.h"
#include "tcp_server.h"
#include "ARM TM4C1294.h"
#include "AD9959.h"
#include "Profile_Pin.h"
#include "seq_table_engine.h"


#ifdef RUN_HTTP_SERVER
#include "httpserver_raw/httpd.h"
#include "cgifuncs.h"
#endif

#ifdef RUN_IO_CONTROLS
#include "io.h"
#include "driverlib/rom_map.h"
#endif


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//                  Checks an SSI base address.
//
//*****************************************************************************

int tUART = 0;                          // counter for reading UART
unsigned long tempDataUART[22];          // temporary data buffer from reading UART
char sMsgLCD[20];                       // Custom message for the LCD


long long dataReadSSI = 0xFFFFFFFFFFFFFFFF;
long long dataReadSSIempty = 0x0000000000000000;
uint32_t dataReadSSIoneByte;
uint32_t dataReadSSIdump;

#define FLAG_SSI_DATA_READING            0
//static volatile unsigned long g_ulDataReadFlags;


//unsigned int CURRENT_DDS = 1;
unsigned int CURRENT_DDS = 1;
unsigned int CURRENT_ChannelDDS = 0;
unsigned long aux_CTL_REG[4];
unsigned int ChannelDDS_UART = 0;
unsigned int NbDDS_UART = 1;

const int TYPEDDS[3] = {TYPEDDS1, TYPEDDS2, TYPEDDS3};  //DDS1 DDS2 DDS3

// Variables de gestion du balayage
volatile uint32_t g_TableIndex = 0;
const uint32_t g_TableSize = 20;
// exemple de Tableau de 20 fr quences (1 MHz   20 MHz) pour SYSCLK = 500 MHz
const uint32_t g_FrequencyTable[20] = {
    // 1 MHz   10 MHz
    0x0083126E, //  1 MHz
    0x010624DD, //  2 MHz
    0x0189374B, //  3 MHz
    0x020C49BA, //  4 MHz
    0x028F5C28, //  5 MHz
    0x03126E97, //  6 MHz
    0x03958106, //  7 MHz
    0x04189374, //  8 MHz
    0x049BA5E3, //  9 MHz
    0x051EB851, // 10 MHz
    0x05A1CABE, // 11 MHz
    0x0624DD2D, // 12 MHz
    0x06A7EF9C, // 13 MHz
    0x072B020B, // 14 MHz
    0x07AE147A, // 15 MHz
    0x083126E9, // 16 MHz
    0x08B43958, // 17 MHz
    0x09374BC7, // 18 MHz
    0x09BA5E36, // 19 MHz
    0x0A3D70A5  // 20 MHz
};
unsigned short channel_DDSAD9959DDS[4];
unsigned long frequencyValueAD9959DDS[4];
unsigned short amplValueAD9959DDS[4];
unsigned short powValueAD9959DDS[4];
unsigned short select_channelDDS[4];
unsigned long controlRegisterValueAD9959DDS[4];

unsigned long bufferedTime[BUF_INST1_SIZE];
unsigned char bufferedNbCmdPerTime[BUF_INST1_SIZE];
unsigned char bufferedDDSnbAndCmd[BUF_INST2_SIZE];
unsigned long long bufferedValue[BUF_INST2_SIZE];
int currentNbTimeLines = 0;
int currentNbValueLines = 0;
unsigned int frequency_data;
int i;
void ExecuteManualSweep_DDS_Setup_uart(unsigned int NbDDS_UART,unsigned int ChannelDDS_UART,unsigned long start_freq_hz,unsigned long end_freq_hz,unsigned long step_freq_hz,double time_step_s);
void ExecuteManualSweep_DDS_Setup_ethernet(unsigned int CURRENT_DDS,unsigned int ChannelDDS,unsigned long start_freq_hz,unsigned long end_freq_hz,unsigned long step_freq_hz,double time_step_s);
void ssi0PutData_QSPI_Fast(uint8_t instruction, uint32_t data);
void ssi1PutData_QSPI_Fast(uint8_t instruction, uint32_t data);
void ssi2PutData_QSPI_Fast(uint8_t instruction, uint32_t data);
// Prototype obligatoire si la fonction est d finie plus bas
static inline bool SetFrequencyTuningWord0_DDS1_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART);
//void Start_Sweep_Run_ethernet(void);
//void Start_Sweep_Run_uart(void);
// Prototypes de la complicated sequence
void Timer3A_IntHandler(void);
void Timer4A_IntHandler(void);
void Timer5A_IntHandler(void);
void Timer6A_IntHandler(void);
void Timer7A_IntHandler(void);
void initTimer3(void);
void initTimer4(void);
void initTimer5(void);
void initTimer6(void);
void initTimer7(void);
void Sequence_AbortAndReleaseControl(void);
unsigned long FreqDeciToFTW(unsigned long frequency_decihz);
unsigned long FreqToFTW(unsigned long frequency_hz);
void Seq_HandleTriggeredMask(unsigned int trigger_flags);

/* Sélection du moteur d'exécution.
 * Le mode TABLE est utilisé par les scénarios compilés par le GUI moderne.
 * Le mode GENERAL conserve certains chemins historiques/compatibilité. */
static volatile bool g_v34_table_mode = false;
static void Engine_SwitchToV34Table(void);
static void Engine_SwitchToGeneralV34(void);

// --- VARIABLES D' TAT DU BALAYAGE GLOBAL (utilis es par les ISR) ---
volatile unsigned long g_current_ftw;      // FTW actuel du balayage
volatile unsigned long g_ftw_start_config; // FTW de d part (pour rechargement au trigger)
volatile unsigned long g_ftw_step;
volatile unsigned long g_ftw_end;
volatile unsigned int g_channel;
volatile bool g_is_rising;
volatile bool g_sweep_running = false;
volatile bool g_sweep_running2 = false;

// --- VARIABLES DE DEBUG POUR LA COMPLICATED SEQUENCE ---
#define SEQ_PREUPDATE_BUCKETS 13U
#define SEQ_TIMER4_UNPREPARED_EXTRA_TICKS 720UL

volatile unsigned long g_seq_dbg_preload_hit = 0UL;
volatile unsigned long g_seq_dbg_preload_miss = 0UL;
volatile unsigned int g_seq_active_mask = 0U;
volatile unsigned int g_seq_profile_low_pending_mask = 0U;
volatile unsigned long g_seq_dbg_trigger_isr_start = 0UL;
volatile unsigned long g_seq_dbg_trigger_to_update_ticks = 0UL;
volatile unsigned long g_seq_last_compensation_ticks = 0UL;
volatile unsigned long g_seq_dbg_hw_fast_ticks = 0UL;
volatile unsigned long g_seq_dbg_hw_fast_max_ticks = 0UL;
volatile unsigned long g_seq_dbg_hw_fast_miss = 0UL;
volatile unsigned long g_seq_hw_fast_prep_estimate_ticks = 340UL;
volatile unsigned long g_seq_step_to_step_pre_estimate_ticks = 768UL;
volatile unsigned long g_seq_dbg_step_to_step_pre_ticks = 0UL;

/* V28 : estimation adaptative du coût avant I/O_UPDATE pour 1..12 channels
 * arrivant à la même frontière Timer4. Les valeurs sont apprises par EWMA. */
volatile unsigned long g_seq_preupdate_estimate_ticks[SEQ_PREUPDATE_BUCKETS] = {
    0UL, 680UL, 860UL, 1040UL, 1220UL, 1400UL, 1580UL,
    1760UL, 1940UL, 2120UL, 2300UL, 2480UL, 2660UL
};
volatile unsigned long g_seq_dbg_preupdate_sample_ticks[SEQ_PREUPDATE_BUCKETS] = {
    0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL,
    0UL, 0UL, 0UL, 0UL, 0UL, 0UL
};
volatile unsigned long g_seq_dbg_timer4_due_count = 0UL;
volatile unsigned long g_seq_dbg_timer4_unprepared_count = 0UL;

volatile uint64_t g_seq_dbg_loop_duration_ticks = 0ULL;
volatile unsigned long g_seq_dbg_hw_exit_late_ticks = 0UL;
volatile unsigned long g_seq_dbg_loop_boundary_late_ticks = 0UL;
volatile unsigned long g_seq_dbg_hw_entry_arm_delay_ticks = 0UL;
volatile unsigned long g_seq_dbg_hw_multi_fast_ticks = 0UL;
volatile unsigned long g_seq_dbg_hw_multi_fast_max_ticks = 0UL;
volatile unsigned long g_seq_dbg_hw_multi_total_ticks = 0UL;
volatile unsigned long g_seq_dbg_hw_multi_total_max_ticks = 0UL;
volatile unsigned long g_seq_dbg_hw_multi_deadline_miss = 0UL;
volatile unsigned long g_seq_dbg_hw_deadline_resync = 0UL;
volatile unsigned long g_seq_hw_multi_prep_estimate_ticks = 440UL;
volatile unsigned int g_seq_timer_hw_owner_bit = 0U;

// ===================== COMPLICATED SEQUENCE V28 ============================
// Timer4A : scheduler STEP / linear sweep.
// Timer5A/6A/7A : un moteur de sweep TM4C dédié par DDS.
// Préchargement ARM garanti + compensation multi-channel adaptative.

#define DDS_COUNT              3U
#define DDS_CH_COUNT           4U
#define SEQ_MAX_ITEMS          40U
#define SEQ_MAX_SWEEP_ITEMS    20U

#define SEQ_ITEM_STEP          0x01U
#define SEQ_ITEM_SWEEP         0x02U  // Linear sweep interne AD9959
#define SEQ_ITEM_HW_SWEEP      0x03U  // Sweep matériel piloté par Timer5/6/7
#define SEQ_ITEM_STEP_EX       0x04U  // pas combiné F / phase / amplitude
#define SEQ_ITEM_FAST_HW_SWEEP 0x05U  // V33 : staircase FTW périodique rapide
#define SEQ_ITEM_SPECIAL_STEP  0x06U  // V33.2 : step libéré par TTL secondaire
#define SEQ_ITEM_SPECIAL_SWEEP 0x07U  // V33.2 : linear sweep libéré par TTL secondaire

#define SEQ_FAST_FLAG_WAIT_TRIGGER  0x01U
#define SEQ_FAST_FLAG_PHASE_TRIANGLE 0x02U

/* V30 : drapeaux de SEQ_ITEM_STEP_EX. */
#define SEQ_STEPEX_FLAG_FREQ   0x01U
#define SEQ_STEPEX_FLAG_PHASE  0x02U
#define SEQ_STEPEX_FLAG_AMP    0x04U

/* V30 : ACR = amplitude multiplier enable (bit 12) + ASF 10 bits. */
#define SEQ_ACR_AMP_ENABLE     0x001000UL

#define SEQ_RETRIGGER_IGNORE_ACTIVE     0U
#define SEQ_RETRIGGER_HIGHER_PRIORITY   1U
#define SEQ_RETRIGGER_ALWAYS            2U

#define SEQ_FRAME_LOAD                  0x03U
#define SEQ_FRAME_ARM                   0x04U
#define SEQ_FRAME_STOP                  0x05U
#define SEQ_FRAME_RESET                 0x06U
#define SEQ_FRAME_STATIC_SET            0x08U

#define SEQ_CFR_DAC_FULL_SCALE       0x000300UL
#define SEQ_CFR_CLEAR_SWEEP_ACCUMULATOR 0x000008UL
#define SEQ_CFR_AUTOCLEAR_SWEEP      0x000010UL
#define SEQ_CFR_LOAD_SRR_IOUPDATE    0x002000UL
#define SEQ_CFR_LINEAR_SWEEP_ENABLE  0x004000UL
#define SEQ_CFR_AFP_FREQ_SWEEP       0x800000UL
#define SEQ_CFR_AFP_PHASE_SWEEP      0xC00000UL

#define SEQ_CFR_SINGLE_TONE_SAFE     (SEQ_CFR_DAC_FULL_SCALE | \
                                      SEQ_CFR_CLEAR_SWEEP_ACCUMULATOR | \
                                      SEQ_CFR_AUTOCLEAR_SWEEP)
#define SEQ_CFR_FREQ_SWEEP_SAFE      (SEQ_CFR_AFP_FREQ_SWEEP | SEQ_CFR_LINEAR_SWEEP_ENABLE | SEQ_CFR_LOAD_SRR_IOUPDATE | SEQ_CFR_DAC_FULL_SCALE)
/* Dwell up/down phase sweep.  AUTOCLEAR_SWEEP must stay clear: the AD9959
 * autoclear is executed on every PROFILE transition and would reset the
 * accumulator at mid-step instead of launching the falling ramp.  The
 * preceding single-tone CFR holds the sweep accumulator at zero before the
 * first I/O_UPDATE, and PROFILE LOW returns it to zero before each next step. */
#define SEQ_CFR_PHASE_SWEEP_SAFE     (SEQ_CFR_AFP_PHASE_SWEEP | SEQ_CFR_LINEAR_SWEEP_ENABLE | SEQ_CFR_LOAD_SRR_IOUPDATE | SEQ_CFR_DAC_FULL_SCALE)
#define SEQ_DESC_PRELOAD_US          40UL
#define SEQ_TIMER_CLK_HZ             120000000UL
#define SEQ_TIMER_TICKS_PER_US       120UL
#define SEQ_TIMER_MIN_TICKS          360UL  // pas logique minimal accepté : 3 µs
#define SEQ_TIMER_RELOAD_MIN_TICKS    24UL   // plancher du reload après compensation : 0,2 µs
#define SEQ_HW_FREQ_TOLERANCE_DECIHZ  10UL   // tolérance de 1 Hz sur le dernier pas
#define SEQ_LOOP_DEADLINE_LEAD_US       10UL
#define SEQ_LOOP_DEADLINE_LEAD_TICKS    (SEQ_LOOP_DEADLINE_LEAD_US * SEQ_TIMER_TICKS_PER_US)
#define SEQ_HW_SWEEP_MIN_TICKS       600UL
#define SEQ_FAST_HW_SWEEP_MIN_TICKS  240UL  /* 2.0 us à 120 MHz */
#define SEQ_FAST_MAX_POINTS          1024U
#define SEQ_FAST_DIAGNOSTIC_STRIDE     16U
#define SEQ_FAST_WAKE_LEAD_TICKS       60UL  /* 0.5 us early match interrupt */
#define SEQ_PHASE_TIMER_MIN_TICKS      24UL

/* Test facultatif seulement : mettre à 1 pour empêcher Timer5/6/7 de
 * préempter Timer4. Laisser à 0 tant que les corrections d'ordonnancement
 * V30.3 n'ont pas été validées, car ce choix peut retarder un point HW. */
#define SEQ_EQUALIZE_TIMER_PRIORITIES 0U

#define SEQ_DEMCR        HWREG(0xE000EDFCUL)
#define SEQ_DWT_CTRL     HWREG(0xE0001000UL)
#define SEQ_DWT_CYCCNT   HWREG(0xE0001004UL)

#define SEQ_DEMCR_TRCENA      (1UL << 24)
#define SEQ_DWT_CYCCNTENA     (1UL << 0)

#define SEQ_TIMER4_ISR_ENTRY_ESTIMATE_TICKS 24UL

/* V28 : un moteur HW périodique dédié par DDS.
 * Timer5A -> DDS1, Timer6A -> DDS2, Timer7A -> DDS3.
 * La channel propriétaire reste dynamique. */
#define SEQ_HW_ACTION_POINT       0U
#define SEQ_HW_ACTION_CHAIN_HW    1U
#define SEQ_HW_ACTION_ENTER_STEP  2U
#define SEQ_HW_ACTION_HANDOFF     3U
#define SEQ_HW_ACTION_STOP        4U

#define SEQ_HW_RESUME_NONE        0U
#define SEQ_HW_RESUME_STEP        1U
#define SEQ_HW_RESUME_HANDOFF     2U

#define SEQ_FRAME_STATS 0x07


typedef struct {
    unsigned long freq_decihz;
    unsigned long duration_us;

    /* Pré-calculé au chargement : aucune conversion double dans Timer4. */
    unsigned long ftw;

    /* V30 : utilisés uniquement par SEQ_ITEM_STEP_EX.
     * pow  : Channel Phase Offset Word, 14 bits (delta_phi = pow*360/16384).
     * asf  : Amplitude Scale Factor, 10 bits (0..1023).
     * flags: SEQ_STEPEX_FLAG_FREQ / _PHASE / _AMP. */
    unsigned short pow;
    unsigned short asf;
    unsigned char  flags;
} seq_step_t;

typedef struct {
    unsigned long start_decihz;
    unsigned long stop_decihz;
    unsigned long step_decihz;
    unsigned long tstep_ns;
    unsigned long duration_ticks;

    /* V28.1 : configuration AD9959 entièrement pré-calculée au chargement.
     * Aucun calcul double ne doit rester dans une ISR Timer4/5/6/7. */
    unsigned long ftw_low;
    unsigned long ftw_high;
    unsigned long rdw;
    unsigned long fdw;
    unsigned long lsrr;
    unsigned char rising;
} seq_sweep_t;

typedef struct {
    unsigned long start_decihz;
    unsigned long stop_decihz;
    unsigned long step_decihz;
    unsigned long tstep_ticks;

    /* Pré-calculés au chargement : aucune conversion double dans Timer4. */
    unsigned long start_ftw;
    unsigned long stop_ftw;
    unsigned long step_ftw;

    /* V33 fast staircase. Generic HW sweeps leave these fields at zero. */
    unsigned char fast_flags;
    unsigned char gate_ttl;
    unsigned short phase_base_pow;
    unsigned short phase_peak_pow;
    unsigned long phase_half_ticks;
    unsigned long phase_delta_word;
    unsigned long phase_cw1;
    unsigned long phase_lsrr;
} seq_hw_sweep_t;

typedef struct {
    unsigned char type;
    unsigned char wait_for_trigger;
    unsigned char gate_ttl;
    seq_step_t step;
    seq_sweep_t sweep;
    seq_hw_sweep_t hw_sweep;
} seq_item_t;

typedef struct {
    bool valid;
    unsigned int n_loops;
    unsigned int n_items;
    seq_item_t items[SEQ_MAX_ITEMS];
} seq_def_t;

typedef enum {
    SEQ_RT_IDLE = 0,
    SEQ_RT_STEP,
    SEQ_RT_SWEEP_PRELOAD_DESC,
    SEQ_RT_SWEEP,
    SEQ_RT_HW_SWEEP,
    SEQ_RT_WAIT_TRIGGER
} seq_rt_phase_t;

typedef struct {
    bool active;

    unsigned int dds_idx;
    unsigned int ch_idx;

    unsigned int loop_idx;
    unsigned int item_idx;

    unsigned int active_ttl;
    unsigned int priority_rank;

    seq_rt_phase_t phase;
    bool sweep_start_pending;
    bool single_tone_ready;

    bool profile_low_after_update;
    bool first_item_preloaded;

    bool next_item_preloaded;
    unsigned int preloaded_loop_idx;
    unsigned int preloaded_item_idx;

    bool sequence_clock_started;
    bool loop_deadline_active;
    uint32_t sequence_start_ticks;
    uint64_t loop_duration_ticks;

    unsigned long remaining_ticks;
    unsigned long hw_current_decihz;
    unsigned long hw_current_ftw;

    uint32_t hw_next_update_deadline_ticks;
    bool hw_deadline_valid;
    bool fast_wait_prepared;
    uint32_t fast_wait_start_ticks;
} seq_rt_t;

volatile seq_def_t g_seq_def[DDS_COUNT][DDS_CH_COUNT];
volatile seq_rt_t  g_seq_rt[DDS_COUNT][DDS_CH_COUNT];

typedef struct {
    bool active;
    unsigned int owner_ch;
    unsigned long period_ticks;
    uint32_t next_deadline_ticks;
    unsigned char prepared_action;
    unsigned int prepared_next_loop;
    unsigned int prepared_next_item;
    unsigned long prepared_decihz;
    unsigned long prepared_ftw;

    /* Channels du même DDS dont la frontière sera validée par le prochain
     * I/O_UPDATE du moteur HW. Un seul HW sweep reste autorisé par DDS. */
    unsigned int companion_due_mask;
    unsigned int companion_end_prepared_mask;

    /* V30 : le premier intervalle du timer périodique est raccourci du temps
     * écoulé entre l'I/O_UPDATE de frontière et l'activation du timer, afin
     * d'ancrer la grille du sweep matériel sur la frontière physique.
     * Ce drapeau demande la restauration de TAILR = period au premier tick. */
    unsigned char first_reload_pending;

    /* V33 dedicated branch: fixed channel, FTW-only preload, no companions. */
    bool fast_mode;
    bool phase_triangle;
    bool profile_pwm;
    unsigned char fast_diag_divider;
    unsigned long phase_half_ticks;
} seq_hw_engine_t;

volatile seq_hw_engine_t g_seq_hw_engine[DDS_COUNT];

typedef struct {
    bool valid;
    unsigned int owner_ch;
    unsigned int item_idx;
    unsigned int count;
    volatile unsigned int next_index;
    unsigned long ftw[SEQ_FAST_MAX_POINTS];
} seq_fast_plan_t;

static seq_fast_plan_t g_seq_fast_plan[DDS_COUNT];
volatile unsigned int g_seq_hw_resume_pending_mask = 0U;
volatile unsigned char g_seq_hw_resume_kind[DDS_COUNT] = {SEQ_HW_RESUME_NONE,SEQ_HW_RESUME_NONE,SEQ_HW_RESUME_NONE};
volatile uint32_t g_seq_hw_resume_update_ticks[DDS_COUNT] = {0UL,0UL,0UL};
volatile bool g_seq_timer4_in_isr = false;
volatile bool g_seq_timer4_force_service = false;
volatile bool g_seq_timer4_active = false;
volatile uint32_t g_seq_timer4_arm_start_ticks = 0UL;

/* V30.3 : origine logique des remaining_ticks actuellement ordonnancés.
 * Contrairement à arm_start_ticks, cette origine précède aussi le temps de
 * post-traitement effectué avant l'armement physique de Timer4. Elle permet
 * à un réveil forcé HW -> Timer4 de retirer tout le temps réellement écoulé. */
volatile uint32_t g_seq_timer4_remaining_origin_ticks = 0UL;

/* V30 : deadline ABSOLUE (DWT) de la prochaine frontière Timer4.
 * Le timer est armé en avance ; l'ISR attend cette deadline avant
 * l'I/O_UPDATE, puis chaîne la suivante : deadline += durée nominale. */
volatile bool     g_seq_timer4_deadline_valid = false;
volatile uint32_t g_seq_timer4_deadline_ticks = 0UL;
volatile unsigned long g_seq_dbg_boundary_spin_ticks = 0UL;      /* attente active avant le dernier I/O_UPDATE */
volatile unsigned long g_seq_dbg_boundary_late_ticks = 0UL;      /* retard du dernier I/O_UPDATE sur sa deadline */
volatile unsigned long g_seq_dbg_boundary_late_max_ticks = 0UL;

/* Diagnostics V28. */
volatile unsigned long g_seq_dbg_hw_engine_isr_ticks[DDS_COUNT] = {0UL,0UL,0UL};
volatile unsigned long g_seq_dbg_hw_engine_isr_max_ticks[DDS_COUNT] = {0UL,0UL,0UL};
volatile unsigned long g_seq_dbg_hw_engine_late_ticks[DDS_COUNT] = {0UL,0UL,0UL};
volatile unsigned long g_seq_dbg_hw_engine_late_max_ticks[DDS_COUNT] = {0UL,0UL,0UL};
volatile unsigned long g_seq_dbg_hw_engine_miss[DDS_COUNT] = {0UL,0UL,0UL};
volatile unsigned long g_seq_dbg_hw_engine_conflict = 0UL;
volatile unsigned long g_seq_dbg_hw_companion_preload_ticks[DDS_COUNT] = {0UL,0UL,0UL};
volatile unsigned long g_seq_dbg_hw_companion_preload_max_ticks[DDS_COUNT] = {0UL,0UL,0UL};
volatile unsigned long g_seq_dbg_hw_companion_late[DDS_COUNT] = {0UL,0UL,0UL};

volatile unsigned int g_seq_arm_mask_by_ttl[3] = {0U, 0U, 0U};
volatile unsigned int g_seq_priority_ttl[3] = {SEQ_TRIGGER_TTL6, SEQ_TRIGGER_TTL8, SEQ_TRIGGER_TTL7};
volatile unsigned int g_seq_retrigger_policy = SEQ_RETRIGGER_HIGHER_PRIORITY;
volatile bool g_seq_any_running = false;
volatile unsigned long g_seq_timer_armed_ticks = 0UL;
volatile unsigned int g_seq_pending_trigger_flags = 0U;
volatile bool g_seq_preload_request = false;

/* V33.2.1: ARM is acknowledged only after a bounded preload performed from
 * main(), never from UARTIntHandler(). This prevents a blocked SSI wait from
 * freezing the UART ISR and producing the misleading "No ACK" timeout. */
static volatile bool g_seq_arm_ack_pending = false;
static volatile bool g_seq_arm_preload_bounded = false;
static volatile bool g_seq_arm_preload_write_failed = false;
#define SEQ_ARM_SSI_TIMEOUT_TICKS 240000UL /* 2 ms at 120 MHz, per register */

volatile unsigned long g_seq_dbg_arm_request_count = 0UL;
volatile unsigned long g_seq_dbg_arm_ack_count = 0UL;
volatile unsigned long g_seq_dbg_arm_nak_count = 0UL;
volatile unsigned long g_seq_dbg_arm_preload_begin_count = 0UL;
volatile unsigned long g_seq_dbg_arm_preload_end_count = 0UL;
volatile unsigned long g_seq_dbg_arm_preload_last_ticks = 0UL;
volatile unsigned long g_seq_dbg_arm_ssi_timeout_count = 0UL;
volatile unsigned long g_seq_dbg_preupdate_ticks = 0UL;
volatile unsigned long g_seq_dbg_postupdate_ticks = 0UL;
volatile unsigned long g_seq_dbg_total_boundary_ticks = 0UL;
volatile unsigned long g_seq_dbg_deadline_miss = 0UL;

/* Legacy software midpoint timer, retained only for old/non-PWM paths.
 * V33.2 does not use it for the 2-us synchronized triangle: DDS3 PROFILE is
 * generated directly by a GPTM CCP/PWM output. */
static volatile bool g_seq_phase_pulse_active = false;
static volatile unsigned int g_seq_phase_pulse_dds = 0U;
static volatile unsigned int g_seq_phase_pulse_channel = 0U;
static volatile uint32_t g_seq_phase_pulse_deadline = 0UL;
volatile unsigned long g_seq_dbg_fast_isr_ticks = 0UL;
volatile unsigned long g_seq_dbg_fast_isr_max_ticks = 0UL;
volatile unsigned long g_seq_dbg_fast_final_handoff_ticks = 0UL;
volatile unsigned long g_seq_dbg_fast_late_max_ticks = 0UL;
volatile unsigned long g_seq_dbg_phase_low_late_max_ticks = 0UL;
volatile unsigned long g_seq_dbg_fast_gate_release_count = 0UL;
volatile unsigned long g_seq_dbg_fast_points_committed = 0UL;
volatile unsigned long g_seq_dbg_fast_fifo_queue_max_ticks = 0UL;
volatile unsigned long g_seq_dbg_special_step_release_count = 0UL;
volatile unsigned long g_seq_dbg_special_linear_release_count = 0UL;
volatile unsigned long g_seq_dbg_profile_pwm_start_count = 0UL;
volatile unsigned long g_seq_dbg_profile_pwm_stop_count = 0UL;
volatile unsigned long g_seq_dbg_profile_pwm_sync_ticks = 0UL;

/* Hardware PROFILE pulse generator for DDS3.  PM3..PM0 are physically
 * connected to T3CCP1, T3CCP0, T2CCP1 and T2CCP0 respectively.  A GPTM
 * PWM output therefore generates the high/low PROFILE edges without a
 * midpoint ISR. */
static volatile bool g_seq_profile_pwm_active = false;
static volatile unsigned long g_seq_profile_pwm_timer_base = 0UL;
static volatile unsigned int g_seq_profile_pwm_timer_half = TIMER_A;
static volatile unsigned char g_seq_profile_pwm_pin = 0U;
static volatile unsigned long g_seq_profile_pwm_restore_pending_base = 0UL;

// Prototypes nouvelle architecture
static bool Seq_ShouldStartChannel(seq_rt_t *rt, unsigned int incoming_rank);
static void Seq_StartChannelsFromMask(unsigned int arm_mask, unsigned int ttl);
void Seq_HandleTriggeredMask(unsigned int trigger_flags);
void Seq_ServicePendingTriggers(void);
static void Seq_AdvanceOneChannel(unsigned int d, unsigned int c, unsigned int *dds_update_mask);
static void Seq_StartPendingSweeps(void);
static void Seq_StartPendingSweepsImmediate(void);
static void Seq_ApplyCurrentItem(unsigned int d, unsigned int c, unsigned int *dds_update_mask);
void Seq_StopAll(void);
void Seq_ResetAllChannels(void);
void Seq_AbortAndReleaseControl(void);
void Seq_Timer4A_IntHandler(void);
static inline unsigned int Seq_GlobalBit(unsigned int dds_idx, unsigned int ch_idx);
static inline unsigned int Seq_CSR_FromChannel(unsigned int ch_idx);
static inline int Seq_TtlToIndex(unsigned int ttl);
static inline bool Seq_IsValidTtl(unsigned int ttl);
static unsigned int Seq_PriorityRank(unsigned int ttl);
bool parse_seq_load_frame(const unsigned char *p, unsigned int len);
bool parse_seq_arm_frame(const unsigned char *p, unsigned int len);
static inline bool Seq_SweepIsRising(const seq_sweep_t *s);
static inline void Seq_SetProfileLow(unsigned int dds_idx, unsigned int ch_idx);
static inline void Seq_SetProfileHigh(unsigned int dds_idx, unsigned int ch_idx);
static inline void Seq_SetSingleToneFTW_Optimized(unsigned int d,unsigned int c,unsigned long ftw,seq_rt_t *rt);
static void Seq_InitCycleCounter(void);
static inline void Seq_ArmTimer4_ticks_compensated(unsigned long logical_ticks,unsigned long overhead_ticks);
static void Seq_ApplyDeferredProfileLows(void);
static bool Seq_PreloadFirstItem_NoUpdate(unsigned int d,unsigned int c);
static bool Seq_InitCurrentItemRuntime_NoWrite(unsigned int d,unsigned int c);
static bool Seq_ServicePreloadRequest(void);
static inline unsigned long Seq_UsToTicks(unsigned long delay_us);
static inline void Seq_ConfigLinearSweep_NoStart(unsigned int dds_idx,unsigned int ch_idx,const seq_sweep_t *s);
static void Seq_PrecomputeLinearSweep(seq_sweep_t *s);
static inline void Seq_WriteRegFast(unsigned int dds_idx,int address,unsigned long data,int nbytes);
static inline int Seq_FirstSetBit12(unsigned int mask);
static bool Seq_GetFollowingPosition(seq_rt_t *rt,seq_def_t *def,unsigned int *next_loop,unsigned int *next_item);
static bool Seq_PrepareItem_NoUpdate(unsigned int d,unsigned int c,
                                     const seq_item_t *item,seq_rt_t *rt);
static unsigned int Seq_CountActiveChannelsOnDDS(unsigned int d);
static void Seq_PreloadNextItemsSafe(void);
static uint64_t Seq_GetItemNominalDurationTicks(const seq_item_t *item);
static uint64_t Seq_GetLoopNominalDurationTicks(const seq_def_t *def);
static inline bool Seq_HwSweepOneIntervalRemaining(const seq_hw_sweep_t *s,unsigned long current_decihz);
static inline void Seq_UpdateLoopDeadlineStateForCurrentItem(seq_rt_t *rt,const seq_def_t *def,const seq_item_t *item);
static inline void Seq_WriteFTW_FastSelected_NoUpdate(unsigned int dds_idx,unsigned long ftw);
static inline bool Seq_IsSingleActiveHwSweep(void);
static bool Seq_ArmSingleActiveHwSweepDeadline(unsigned long logical_ticks);
static bool Seq_TryFastSingleHwSweep(uint32_t t_isr_start);
static bool Seq_TryFastHwSweepWithOtherChannels(uint32_t t_isr_start,unsigned long elapsed_ticks,unsigned int owner_bit);
static inline void Seq_ArmTimer4ForUpdateDeadline(uint32_t update_deadline_ticks,unsigned long predicted_preupdate_ticks,unsigned long logical_ticks);
static bool Seq_ArmEarliestHwSweepDeadline(unsigned long logical_ticks);
static inline void Seq_WriteFTW_ChannelFast_NoUpdate(unsigned int d,unsigned int c,unsigned long ftw);
static inline void Seq_IOUpdateOneFast(unsigned int d);
static inline uint32_t Seq_AdvanceHwDeadline(uint32_t previous_deadline,uint32_t physical_update_ticks,unsigned long logical_ticks);
static void Seq_InitHwDeadlinesAfterUpdate(uint32_t physical_update_ticks);
static bool Seq_IsSinglePreloadedStepToStep(void);
static inline unsigned long Seq_HwTimerBase(unsigned int d);
static inline bool Seq_AnyHwEngineActive(void);
static inline bool Seq_DdsBlockedByHw(unsigned int d);
static void Seq_StopAllHwEngines(void);
static void Seq_StartHwEnginesAfterUpdate(uint32_t physical_update_ticks);
static bool Seq_HwPrepareNext(unsigned int d);
static void Seq_HwTimerIntHandler(unsigned int d);
static void Seq_HwScheduleCompanions(unsigned int d,unsigned long next_period_ticks);
static void Seq_HwCommitCompanionsBeforeUpdate(unsigned int d,unsigned int *committed_mask);
static void Seq_HwFinalizeCompanionsAfterUpdate(unsigned int d,unsigned int committed_mask);
static void Seq_RequestTimer4Service(void);
static void Seq_ProcessHwResumePending(unsigned int *dds_update_mask,uint32_t now_ticks);
static void Engine_ConfigureTimersForGeneral(void);
static void Engine_StopAndClearTimer(unsigned long base);
static void Seq_StopRuntimeOnly(void);
static bool parse_seq_static_set_frame(const unsigned char *p,unsigned int len);
static inline void Seq_ConfigFastHwSweep_NoStart(unsigned int d,unsigned int c,const seq_hw_sweep_t *sweep);
static bool Seq_BuildFastPlan(unsigned int d,unsigned int c,unsigned int item_idx,const seq_hw_sweep_t *sweep);
static inline bool Seq_QueueFastFtwNonBlocking(unsigned int d,unsigned long ftw);
static bool Seq_ReleaseWaitingSpecialItems(unsigned int ttl);
static void Seq_PrepareWaitingSpecialAfterUpdate(void);
static void Seq_FastHwTimerIntHandler(unsigned int d);
static void Seq_StopPhasePulseTimer(void);
static void Seq_ArmPhasePulseLow(unsigned int d,unsigned int c,uint32_t update_ticks,unsigned long half_ticks);
static bool Seq_StartProfilePwm(unsigned int d,unsigned int c,unsigned long period_ticks,unsigned long half_ticks,unsigned long boundary_timer_base,uint32_t *sync_ticks);
static void Seq_StopProfilePwm(void);
static void Seq_ServiceProfilePwmRestore(void);


unsigned long TTL2(void)
{
  return PL7;      // 0x80 if pressed, 0x00 if not pressed
}
unsigned long TTL3(void)
{
  return PL5;      // 0x20 if pressed, 0x00 if not pressed
}
unsigned long TTL4(void)
{
  return PL4;      // 0x10 if pressed, 0x00 if not pressed
}
unsigned long TTL5(void)
{
  return PL3;      // 0x08 if pressed, 0x00 if not pressed
}
unsigned long TTL6(void)
{
  return PL2;      // 0x04 if pressed, 0x00 if not pressed
}
unsigned long TTL7(void)
{
  return PL1;      // 0x02 if pressed, 0x00 if not pressed
}
unsigned long TTL8(void)
{
  return PL0;      // 0x01 if pressed, 0x00 if not pressed
}
//*****************************************************************************
//! Source files for the internal file system image can be found in the ``fs''
//! directory.  If any of these files are changed, the file system image
//! (io_fsdata.h) should be rebuilt by running the following command from the
//! enet_io directory:
//!
//! ../../../../tools/bin/makefsfile -i fs -o io_fsdata.h -r -h -q
//!
//! For additional details on lwIP, refer to the lwIP web page at:
//! http://savannah.nongnu.org/projects/lwip/
//
//*****************************************************************************

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICKHZ                10
#define SYSTICKMS               (10000 / SYSTICKHZ)

//*****************************************************************************
//
// A set of flags for Timer0.  The flag bits are defined as follows:
//
//     0 -> An indicator that the animation timer interrupt has occurred.
//
//*****************************************************************************

#define FLAG_TIMER_TICK            0
static volatile unsigned long g_ulTimerFlags;

//*****************************************************************************
//
// Timeout for DHCP address request (in seconds).
//
//*****************************************************************************
#ifndef DHCP_EXPIRE_TIMER_SECS
#define DHCP_EXPIRE_TIMER_SECS  100
#endif

//*****************************************************************************
//
// The current IP address.
//
//*****************************************************************************
uint32_t g_ui32IPAddress;
uint32_t g_ui32PortAddress;

//*****************************************************************************
//
// The system clock frequency.  Used by the SD card driver.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*****************************************************************************
void SysTickIntHandler(void)
{
    //
    // Call the lwIP timer handler.
    //
    lwIPTimer(SYSTICKMS);
}
//*****************************************************************************
//
// Display an lwIP type IP Address.
//
//*****************************************************************************
void DisplayIPAddress(uint32_t ui32Addr)
{
    char pcBuf[16];

    //
    // Convert the IP Address into a string.
    //
    usprintf(pcBuf, "%d.%d.%d.%d", ui32Addr & 0xff, (ui32Addr >> 8) & 0xff,
            (ui32Addr >> 16) & 0xff, (ui32Addr >> 24) & 0xff);
    //
    // Display the string.
    //
    UARTprintf(pcBuf);
    sprintf(pcBuf,"%d.%d.%d.%d", ui32Addr & 0xff, (ui32Addr >> 8) & 0xff,(ui32Addr >> 16) & 0xff, (ui32Addr >> 24) & 0xff);
}

int DisplayPortAddress(uint32_t ui32Addr)
{
    char pcBuf[2];

    //
    // Convert the Port Address into a string.
    //
    usprintf(pcBuf, "%d", ui32Addr);

    //
    // Display the string.
    //
    UARTprintf(pcBuf);
    sprintf(pcBuf, "%d", ui32Addr);
    return 0;
}

int Displaydata(uint32_t ui32value)
{
    char pcBuf[16];

    //UARTprintf(pcBuf1);

    frequency_data =(unsigned int)(((double)ui32value * (double)SYSCLKDDS)/ 4294967296.0);
    //
    // Convert the data_codeur into a string.
    //

    usprintf(pcBuf, "%d",frequency_data);

    //
    // Display the string.
    //
    UARTprintf(pcBuf);
    return 0;
}

//*****************************************************************************
//
// Required by lwIP library to support any host-related timer functions.
//
//*****************************************************************************
void lwIPHostTimerHandler(void)
{
    uint32_t ui32NewIPAddress;
    uint32_t ui32NewPortAddress;

    //itoa(ui32NewPortAddress, str, 2);

    //uint32_t ui32Newvalue;

    //
    // Get the current IP address.
    //
    ui32NewIPAddress = lwIPLocalIPAddrGet();
    ui32NewPortAddress = TCP_SERVER_SERVER_PORT;

    //
    // See if the IP address has changed.
    //
    if(ui32NewIPAddress != g_ui32IPAddress)
    {
        //
        // See if there is an IP address assigned.
        //
        if(ui32NewIPAddress == 0xffffffff)
        {
            //
            // Indicate that there is no link.
            //
            UARTprintf("Waiting for link.\n");
        }
        else if(ui32NewIPAddress == 0)
        {
            //
            // There is no IP address, so indicate that the DHCP process is
            // running.
            //
            UARTprintf("Waiting for IP address.\n");
        }
        else
        {
            //
            // Display the new IP address.
            //
            UARTprintf("IP Address: ");
            DisplayIPAddress(ui32NewIPAddress);
            UARTprintf("\n");
        }

        //
        // Save the new IP address.
        //
        g_ui32IPAddress = ui32NewIPAddress;
        g_ui32PortAddress = ui32NewPortAddress;
        UARTprintf("Port Address: ");
        DisplayPortAddress(ui32NewPortAddress);
        UARTprintf("\n");
    }


    //
    // If there is not an IP address.
    //
    if((ui32NewIPAddress == 0) || (ui32NewIPAddress == 0xffffffff))
    {
        //
        // Do nothing and keep waiting.
        //
    }

    if((ui32NewPortAddress == 0) || (ui32NewPortAddress == 0xffffffff))
    {
        //
        // Do nothing and keep waiting.
        //
    }

}
//Code Optimis  (Bare-Metal + Inline) start Timer 2A. doit  tre avant l'ISR pour  tre "inlin e"
static inline void Start_Sweep_Run_uart(void)
{
    if (!g_sweep_running)
    {
        // Mise   jour des variables (RAM)
        g_current_ftw = g_ftw_start_config;
        g_sweep_running = true;
        // Activation directe du Timer pour vitesse max
        HWREG(TIMER2_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
    }
}
//Code Optimis  (Bare-Metal + Inline) start Timer 1A. doit  tre avant l'ISR pour  tre "inlin e"
static inline void Start_Sweep_Run_ethernet(void)
{
    if (!g_sweep_running2)
    {
        // Mise   jour des variables (RAM)
        g_current_ftw = g_ftw_start_config;
        g_sweep_running2 = true;
        // Activation directe du Timer pour vitesse max
        HWREG(TIMER1_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
    }
}
/*static inline void Start_Sweep_TTL3_Fast(void)
{
    if (g_sweep_running) return;

    // Chargement instantan  des variables globales (depuis le cache)
    g_current_ftw      = g_ttl3_ftw_start;
    g_ftw_end          = g_ttl3_ftw_stop;
    g_ftw_step         = g_ttl3_ftw_step;
    g_is_rising        = (g_ttl3_ftw_stop > g_ttl3_ftw_start);

    // Met   jour les globaux de contexte pour que le Timer sache quoi faire
    NbDDS_UART = TTL3_DDS_NUM;
    g_channel  = TTL3_CHANNEL;

    // Configuration SPI Rapide (Sp cifique DDS 1 / SSI0) On  crit directement
    while(!(SSI0_SR_R & SSI_SR_TNF)); // Attendre place FIFO
    SSI0_DR_R = CSR_ADRESS;
    SSI0_DR_R = g_ttl3_csr_val;       // Valeur pr -calcul e

    // Pulse I/O UPDATE (PK7 pour DDS1) (tr s rapide)
    HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;
    __asm(" NOP");
    HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00;

    // Configuration Timer
    HWREG(TIMER2_BASE + TIMER_O_TAILR) = g_ttl3_timer_load;

    // D marrage timer
    g_sweep_running = true;
    HWREG(TIMER2_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
}
void Init_TTL3_Precalculer(void)
{
    unsigned long long const_scale = 4294967296ULL; // 2^32

    // Calcul des FTW (Frequency Tuning Words)
    g_ttl3_ftw_start = (unsigned long)((TTL3_START_FREQ * const_scale) / SYSCLKDDS);
    g_ttl3_ftw_stop  = (unsigned long)((TTL3_STOP_FREQ  * const_scale) / SYSCLKDDS);
    g_ttl3_ftw_step  = (unsigned long)((TTL3_STEP_FREQ  * const_scale) / SYSCLKDDS);

    // Calcul du Timer Load
    g_ttl3_timer_load = (uint32_t)(TTL3_TIME_STEP_S * SYSCLKMICRO);

    // Pr -calcul de la valeur CSR (SPI)
    // ((Channel & 0x0F) << 4) | 0x06 (Mode Quad/Single)
    g_ttl3_csr_val = ((TTL3_CHANNEL & 0x0F) << 4) | 0x06;
}
*/
static inline void Load_And_Start_Sweep_Uart_calculate(unsigned int dds_num, unsigned int ChannelDDS_UART, unsigned long start_hz, unsigned long stop_hz, unsigned long step_hz, double time_s)
{
    NbDDS_UART = dds_num; // Indique au Timer quel DDS piloter (1, 2 ou 3)
    g_channel = ChannelDDS_UART;     // Masque des canaux (ex: 15 pour tout, 1 pour CH0)

    // ---------------------------------------------------------
    // Calculs des Tuning Words (FTW)
    // ---------------------------------------------------------

    unsigned long ftw_start = FreqDeciToFTW(start_hz);
    unsigned long ftw_stop  = FreqDeciToFTW(stop_hz);
    unsigned long ftw_step  = FreqDeciToFTW(step_hz);

    g_ftw_start_config = ftw_start;
    g_current_ftw  = ftw_start;
    g_ftw_end      = ftw_stop;
    g_ftw_step     = ftw_step;

    // sens du balayage
    g_is_rising  = (ftw_stop > ftw_start);

    // ---------------------------------------------------------
    // Configuration Hardware Initiale (SPI)
    // ---------------------------------------------------------

    // S lectionne le Chip Select du DDS
    //choiceNbDDS(dds_num, false);

    // Envoie la commande SPI pour activer les canaux demand s
    // On le fait maintenant pour ne pas perdre de temps dans l'interruption du Timer.

    // Ici on suppose le mode Single ou Quad (ex: 0x06 pour Quad compatible)
    uint8_t csr_val = ((ChannelDDS_UART & 0x0F) << 4) | 0x06;

    switch(dds_num)
       {
       case 1:
        // SSI0 pour DDS1
        while(!(SSI0_SR_R & SSI_SR_TNF)); // Attendre place dans FIFO
        SSI0_DR_R = CSR_ADRESS;           // Adresse 0x00
        SSI0_DR_R = csr_val;              // Valeur
        // Pulse I/O UPDATE (PK7)
        // Utilisation de l' criture bit-band e pour l'atomicit  et la vitesse
        HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;
        __asm(" NOP"); // D lai minime
        HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00;
        break;

       case 2:
       // SSI1 pour DDS2
       while(!(SSI1_SR_R & SSI_SR_TNF)); // Attendre place dans FIFO
       SSI1_DR_R = CSR_ADRESS;           // Adresse 0x00
       SSI1_DR_R = csr_val;              // Valeur
       // Pulse I/O UPDATE (PN3)
       HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;  // Mettre PN3   1
       // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
       __asm(" NOP");
       HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // Mettre PN3   0

       break;

       case 3:
       // SSI2 pour DDS3
       while(!(SSI2_SR_R & SSI_SR_TNF)); // Attendre place dans FIFO
       SSI2_DR_R = CSR_ADRESS;           // Adresse 0x00
       SSI2_DR_R = csr_val;              // Valeur
       // Pulse I/O UPDATE (PN0
       HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01; // Mettre PN0   1
       // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
       __asm(" NOP");
       HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00; // Mettre PN0   0
       break;
       }

    // ---------------------------------------------------------
    // Configuration du Timer 2 (Vitesse)
    // ---------------------------------------------------------
    // Calcul de la p riode en cycles d'horloge : Temps(s) * 120MHz
    uint32_t ui32Period = (uint32_t)(time_s * SYSCLKMICRO);

    // Chargement direct dans le registre TAILR (Timer A Interval Load Register)
    // HWREG est plus rapide que l'appel de fonction driverlib TimerLoadSet
    HWREG(TIMER2_BASE + TIMER_O_TAILR) = ui32Period;

    // ---------------------------------------------------------
    // D marrage du Balayage
    // ---------------------------------------------------------
    g_sweep_running = true; // ex cution du code dans Timer2A_IntHandler

    // Activation directe du bit TAEN (Timer A Enable) dans le registre de contr le (CTL)
    // Le balayage commence imm diatement.
    HWREG(TIMER2_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
}
// --- DDS 1 (SSI0, PK7) ---
static inline bool SetFrequencyTuningWord0_DDS1_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{
    if (CURRENT_ChannelDDS > 3) return false;

    // Envoi SPI Rapide
    //ssi0PutData(FTW_ADRESS, frequencyValue,0x04);
    //ssi0PutData_Burst(FTW_ADRESS, frequencyValue);

    // Envoi SPI ULTRA Rapide
    ssi0PutData_QSPI_Fast(FTW_ADRESS, frequencyValue);

    // Mise   jour tableau
    frequencyValueAD9959DDS[CURRENT_ChannelDDS] = frequencyValue;

    // On doit attendre que le SSI ait fini de vider le FIFO sur le c ble.
    // Sans  a, l'IO_UPDATE part trop t t.

    while(SSI0_SR_R & SSI_SR_BSY);

    // I/O Update (PK7) via AHB (activ  dans PeripheralEnableInit())
    HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;
    __asm(" NOP"); // Petit d lai ~8ns
    HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00;

    return true;
}
// --- DDS 2 (SSI1, PN3) ---
static inline bool SetFrequencyTuningWord0_DDS2_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{
    if (CURRENT_ChannelDDS > 3) return false;

    // Envoi SPI Rapide
    //ssi1PutData_Burst(FTW_ADRESS, frequencyValue);

    // Envoi SPI ULTRA Rapide
    ssi1PutData_QSPI_Fast(FTW_ADRESS, frequencyValue);

    // Mise   jour tableau
    frequencyValueAD9959DDS[CURRENT_ChannelDDS] = frequencyValue;

    // On doit attendre que le SSI ait fini de vider le FIFO sur le c ble.
    // Sans  a, l'IO_UPDATE part trop t t.
    while(SSI1_SR_R & SSI_SR_BSY);


    // I/O Update (PN3) - Pulse rapide
    HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08; // PN3 High
    __asm(" NOP");
    HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // PN3 Low

    return true;
}

// --- DDS 3 (SSI2, PN0) ---
static inline bool SetFrequencyTuningWord0_DDS3_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{

    if (CURRENT_ChannelDDS > 3) return false;

    // Envoi SPI Rapide
    //ssi2PutData_Burst(FTW_ADRESS, frequencyValue);

    // Envoi SPI ULTRA Rapide
    ssi2PutData_QSPI_Fast(FTW_ADRESS, frequencyValue);

    // Mise   jour tableau
    frequencyValueAD9959DDS[CURRENT_ChannelDDS] = frequencyValue;

    // On doit attendre que le SSI ait fini de vider le FIFO sur le c ble.
    // Sans  a, l'IO_UPDATE part trop t t.
    while(SSI2_SR_R & SSI_SR_BSY);

    // I/O Update (PN0) - Pulse rapide
    HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01; // PN0 High
    __asm(" NOP");
    HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00; // PN0 Low

    return true;
}

#define TTL3_DDS_NUM       1U
#define TTL3_CHANNEL       0U
#define TTL3_START_FREQ    10000000UL
#define TTL3_STOP_FREQ     20000000UL
#define TTL3_STEP_FREQ     100000UL
#define TTL3_TIME_STEP_S   0.00001
//Interrupt service routine exemple d'utilisation des interruption sur Port L ISR
void GPIOIntHandler(void)
{
    uint32_t t_gpio_isr_entry = SEQ_DWT_CYCCNT;
    //Lecture directe m moire.
    uint32_t status = HWREG(GPIO_PORTL_BASE + GPIO_O_MIS);
    // criture directe dans le registre ICR (Interrupt Clear Register)
    HWREG(GPIO_PORTL_BASE + GPIO_O_ICR) = status;

    unsigned int seq_trigger_flags = 0U;

    if (status & GPIO_INT_PIN_2) seq_trigger_flags |= (1U << 0); // TTL6
    if (status & GPIO_INT_PIN_1) seq_trigger_flags |= (1U << 1); // TTL7
    if (status & GPIO_INT_PIN_0) seq_trigger_flags |= (1U << 2); // TTL8

    if (seq_trigger_flags != 0U) {
        g_seq_dbg_trigger_isr_start = t_gpio_isr_entry;
        if (g_v34_table_mode) Tbl34_HandleTriggeredMask(seq_trigger_flags);
        else Seq_HandleTriggeredMask(seq_trigger_flags);
    }

    if (status & GPIO_INT_PIN_7)
        {
        Start_Sweep_Run_ethernet(); //PL7 -- TTL2
        }
    if (status & GPIO_INT_PIN_5)  // TTL3  -- Calcul et Configuration ---
            {
               // On charge les param tres
               Load_And_Start_Sweep_Uart_calculate(
                    TTL3_DDS_NUM,
                    TTL3_CHANNEL,
                    (unsigned long)TTL3_START_FREQ * 10UL,
                    (unsigned long)TTL3_STOP_FREQ  * 10UL,
                    (unsigned long)TTL3_STEP_FREQ  * 10UL,
                    TTL3_TIME_STEP_S
                );
            }

    /*if (status & GPIO_INT_PIN_5)  // TTL3  -- precalculer et Configuration ---
                {
                 Start_Sweep_TTL3_Fast();
                }
    */
    if (status & GPIO_INT_PIN_4)   // TTL4 -- D marrage Timer 2 (Sweep existant) ---
        {
            g_sweep_running = true;
            // Activation directe du Timer 2 pour vitesse max
            HWREG(TIMER2_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
        }

    if (status & GPIO_INT_PIN_3) // TTL5 start sweep -- Rafale Table de Fr quence (Burst)--
        {
            int i;
            // Pointeur vers la table globale (100k - 1M)
            register uint32_t *pData = (uint32_t *)g_FrequencyTable;

            for(i = 0; i < 20; i++)
            {
                // Appel direct de votre fonction d'envoi (suppos e inline)
                // Canal 0 par exemple
                SetFrequencyTuningWord0_DDS1_burst(pData[i], g_channel);
            }
        }
    

    /*if (TTL1())
     {
       GPIOIntClear(GPIO_PORTL_BASE, GPIO_INT_PIN_6);
       int state = GPIOIntStatus(GPIO_PORTL_BASE, true);
       Start_Sweep_Run_uart();  // TTL1
     }
    if (TTL2())
     {
      GPIOIntClear(GPIO_PORTL_BASE, GPIO_INT_PIN_7);
      int state = GPIOIntStatus(GPIO_PORTL_BASE, true);
      Start_Sweep_Run_ethernet();  // TTL2
     }

    // V rifier si l'interruption provient de PL5 (TTL3)
    if (TTL3())
        {}
    // V rifier si l'interruption provient de PL4 (TTL4)
    if (TTL4())
        {}
    // V rifier si l'interruption provient de PL3 (TTL5)
    if (TTL5())
        {}
    // V rifier si l'interruption provient de PL2 (TTL6)
    if (TTL6())
        {}
    // V rifier si l'interruption provient de PL1 (TTL7)
    if (TTL7())
        {}
    // V rifier si l'interruption provient de PL0 (TTL8)
    if (TTL8())
        {}
*/
}

//========== Décodage trame "complicated sequence" ==========

static unsigned int rd_u16_le(const unsigned char *p) {
    return (unsigned int)p[0] | ((unsigned int)p[1] << 8);
}

static unsigned long rd_u32_le(const unsigned char *p)
{
    return (unsigned long)p[0] | ((unsigned long)p[1] << 8) | ((unsigned long)p[2] << 16) | ((unsigned long)p[3] << 24);
}


/* V30 : une séquence est dite "complexe" si elle contient au moins un sweep
 * (linear sweep AD9959 ou sweep matériel TM4C). */
static bool Seq_DefHasSweep(const seq_def_t *def)
{
    unsigned int i;

    if (!def->valid) return false;

    for (i = 0U; i < def->n_items; i++) {
        if (def->items[i].type == SEQ_ITEM_SWEEP ||
            def->items[i].type == SEQ_ITEM_HW_SWEEP) {
            return true;
        }
    }

    return false;
}

/* V30 : refuse l'armement si deux canaux du même AD9959 sont armés avec des
 * séquences contenant chacune un sweep. Un AD9959 partage CSR, le bus SSI et
 * la broche I/O_UPDATE : deux séquences complexes sur le même composant ne
 * peuvent pas être servies avec la précision demandée. Les autres canaux du
 * DDS restent libres de jouer des STEP / STEP_EX (single tone, sauts de
 * fréquence, phase, amplitude). */
static bool Seq_ArmMasksRespectOneComplexPerDds(unsigned int mask6,
                                                unsigned int mask7,
                                                unsigned int mask8)
{
    unsigned int d;
    unsigned int c;
    unsigned int armed_union = (mask6 | mask7 | mask8) & 0x0FFFU;

    for (d = 0U; d < DDS_COUNT; d++) {
        unsigned int complex_count = 0U;

        for (c = 0U; c < DDS_CH_COUNT; c++) {
            unsigned int bit = Seq_GlobalBit(d,c);

            if ((armed_union & bit) == 0U) continue;

            if (Seq_DefHasSweep((const seq_def_t *)&g_seq_def[d][c])) {
                complex_count++;
            }
        }

        if (complex_count > 1U) return false;
    }

    return true;
}

bool parse_seq_arm_frame(const unsigned char *p, unsigned int len)
{
    unsigned int mask6;
    unsigned int mask7;
    unsigned int mask8;
    unsigned int prio0;
    unsigned int prio1;
    unsigned int prio2;
    unsigned int policy;

    if (len != 10U) return false;

    mask6 = rd_u16_le(&p[0]);
    mask7 = rd_u16_le(&p[2]);
    mask8 = rd_u16_le(&p[4]);

    prio0 = p[6];
    prio1 = p[7];
    prio2 = p[8];
    policy = p[9];

    if (mask6 > 0x0FFFU || mask7 > 0x0FFFU || mask8 > 0x0FFFU) return false;

    if (!Seq_IsValidTtl(prio0)) return false;
    if (!Seq_IsValidTtl(prio1)) return false;
    if (!Seq_IsValidTtl(prio2)) return false;

    if (prio0 == prio1 || prio0 == prio2 || prio1 == prio2) return false;

    if (policy > SEQ_RETRIGGER_ALWAYS) return false;

    /* V30 : au plus une séquence contenant un sweep par DDS physique. */
    if (!Seq_ArmMasksRespectOneComplexPerDds(mask6,mask7,mask8)) return false;

    g_seq_arm_mask_by_ttl[0] = mask6;
    g_seq_arm_mask_by_ttl[1] = mask7;
    g_seq_arm_mask_by_ttl[2] = mask8;

    g_seq_priority_ttl[0] = prio0;
    g_seq_priority_ttl[1] = prio1;
    g_seq_priority_ttl[2] = prio2;

    g_seq_retrigger_policy = policy;

    /*
     * V28 : l'ACK ARM ne doit être envoyé qu'après le préchargement réel.
     * Cette opération est volontairement faite ici : elle ne se produit pas
     * pendant l'exécution normale d'une séquence et supprime la course
     * ARM -> trigger qui produisait des preload_miss et +7 à +10 µs.
     */
    g_seq_dbg_preload_hit = 0UL;
    g_seq_dbg_preload_miss = 0UL;
    g_seq_preload_request = true;
    g_seq_arm_ack_pending = true;
    g_seq_dbg_arm_request_count++;

    /* The physical preload is deferred to main().  The ARM ACK is emitted
     * only after that preload has completed (or a NAK after a bounded SSI
     * timeout), so ACK still means "ready for the external TTL". */
    return true;
}

bool parse_seq_load_frame(const unsigned char *p, unsigned int len)
{
    unsigned int target_mask;
    unsigned int n_loops;
    unsigned int n_items;
    unsigned int pos;
    unsigned int d;
    unsigned int c;
    unsigned int i;
    unsigned int sweep_count = 0U;

    /* V30 : "static" pour ne pas consommer ~4 Ko de pile dans l'ISR UART.
     * La réception UART est strictement séquentielle : aucune réentrance. */
    static seq_def_t tmp;

    if (len < 5U) return false;

    target_mask = rd_u16_le(&p[0]);
    n_loops = rd_u16_le(&p[2]);
    n_items = p[4];

    if (target_mask == 0U || target_mask > 0x0FFFU) return false;
    if (n_items == 0U || n_items > SEQ_MAX_ITEMS) return false;
    /* V33.2: one main TTL launches exactly one execution.  The legacy
     * n_loops field is accepted for wire compatibility but ignored. */
    (void)n_loops;

    tmp.valid = true;
    tmp.n_loops = 1U;
    tmp.n_items = n_items;

    pos = 5U;

    for (i = 0U; i < n_items; i++) {
        if (pos >= len) return false;

        tmp.items[i].type = p[pos];
        tmp.items[i].wait_for_trigger = 0U;
        tmp.items[i].gate_ttl = 0U;
        pos++;

        if (tmp.items[i].type == SEQ_ITEM_SPECIAL_STEP) {
            if ((pos + 9U) > len) return false;

            tmp.items[i].type = SEQ_ITEM_STEP;
            tmp.items[i].wait_for_trigger = 1U;
            tmp.items[i].step.freq_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].step.duration_us = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].gate_ttl = p[pos++];

            if (tmp.items[i].step.duration_us == 0UL) return false;
            if (!Seq_IsValidTtl(tmp.items[i].gate_ttl)) return false;

            tmp.items[i].step.ftw = FreqDeciToFTW(tmp.items[i].step.freq_decihz);
            tmp.items[i].step.flags = SEQ_STEPEX_FLAG_FREQ;
            tmp.items[i].step.pow = 0U;
            tmp.items[i].step.asf = 0U;
        }
        else if (tmp.items[i].type == SEQ_ITEM_SPECIAL_SWEEP) {
            if ((pos + 21U) > len) return false;

            sweep_count++;
            if (sweep_count > SEQ_MAX_SWEEP_ITEMS) return false;

            tmp.items[i].type = SEQ_ITEM_SWEEP;
            tmp.items[i].wait_for_trigger = 1U;
            tmp.items[i].sweep.start_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].sweep.stop_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].sweep.step_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].sweep.tstep_ns = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].sweep.duration_ticks = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].gate_ttl = p[pos++];

            if (tmp.items[i].sweep.step_decihz == 0UL) return false;
            if (tmp.items[i].sweep.tstep_ns == 0UL) return false;
            if (tmp.items[i].sweep.duration_ticks == 0UL) return false;
            if (!Seq_IsValidTtl(tmp.items[i].gate_ttl)) return false;
            Seq_PrecomputeLinearSweep(&tmp.items[i].sweep);
        }
        else if (tmp.items[i].type == SEQ_ITEM_STEP) {
            if ((pos + 8U) > len) return false;

            tmp.items[i].step.freq_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].step.duration_us = rd_u32_le(&p[pos]); pos += 4U;

            if (tmp.items[i].step.duration_us == 0UL) return false;

            /* Conversion payée au chargement, jamais à une frontière Timer4. */
            tmp.items[i].step.ftw = FreqDeciToFTW(tmp.items[i].step.freq_decihz);
            tmp.items[i].step.flags = SEQ_STEPEX_FLAG_FREQ;
            tmp.items[i].step.pow = 0U;
            tmp.items[i].step.asf = 0U;
        }
        else if (tmp.items[i].type == SEQ_ITEM_STEP_EX) {
            /* V30 : flags(u8) freq_decihz(u32) pow(u16) asf(u16) duration_us(u32) */
            unsigned int flags;

            if ((pos + 13U) > len) return false;

            flags = p[pos]; pos += 1U;
            tmp.items[i].step.freq_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].step.pow = (unsigned short)rd_u16_le(&p[pos]); pos += 2U;
            tmp.items[i].step.asf = (unsigned short)rd_u16_le(&p[pos]); pos += 2U;
            tmp.items[i].step.duration_us = rd_u32_le(&p[pos]); pos += 4U;

            if (tmp.items[i].step.duration_us == 0UL) return false;
            if ((flags & (SEQ_STEPEX_FLAG_FREQ | SEQ_STEPEX_FLAG_PHASE | SEQ_STEPEX_FLAG_AMP)) == 0U) return false;
            if (flags > (SEQ_STEPEX_FLAG_FREQ | SEQ_STEPEX_FLAG_PHASE | SEQ_STEPEX_FLAG_AMP)) return false;
            if (tmp.items[i].step.pow > 0x3FFFU) return false;
            if (tmp.items[i].step.asf > 0x03FFU) return false;

            tmp.items[i].step.flags = (unsigned char)flags;
            tmp.items[i].step.ftw = FreqDeciToFTW(tmp.items[i].step.freq_decihz);
        }
        else if (tmp.items[i].type == SEQ_ITEM_SWEEP) {
            if ((pos + 20U) > len) return false;

            sweep_count++;
            if (sweep_count > SEQ_MAX_SWEEP_ITEMS) return false;

            tmp.items[i].sweep.start_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].sweep.stop_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].sweep.step_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].sweep.tstep_ns = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].sweep.duration_ticks = rd_u32_le(&p[pos]); pos += 4U;

            if (tmp.items[i].sweep.step_decihz == 0UL) return false;
            if (tmp.items[i].sweep.tstep_ns == 0UL) return false;
            if (tmp.items[i].sweep.duration_ticks == 0UL) return false;

            /* Les divisions en double sont payées une seule fois ici, pendant
             * le chargement UART, jamais dans le scheduler temps réel. */
            Seq_PrecomputeLinearSweep(&tmp.items[i].sweep);
        }
        else if (tmp.items[i].type == SEQ_ITEM_HW_SWEEP) {
            if ((pos + 16U) > len) return false;

            sweep_count++;
            if (sweep_count > SEQ_MAX_SWEEP_ITEMS) return false;

            tmp.items[i].hw_sweep.start_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].hw_sweep.stop_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].hw_sweep.step_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].hw_sweep.tstep_ticks = rd_u32_le(&p[pos]); pos += 4U;

            if (tmp.items[i].hw_sweep.step_decihz == 0UL) return false;
            if (tmp.items[i].hw_sweep.tstep_ticks < SEQ_HW_SWEEP_MIN_TICKS) return false;

            /* Hors temps réel : le coût des conversions FTW est payé ici,
             * jamais dans l'ISR Timer4. */
            tmp.items[i].hw_sweep.start_ftw =
                FreqDeciToFTW(tmp.items[i].hw_sweep.start_decihz);
            tmp.items[i].hw_sweep.stop_ftw =
                FreqDeciToFTW(tmp.items[i].hw_sweep.stop_decihz);
            tmp.items[i].hw_sweep.step_ftw =
                FreqDeciToFTW(tmp.items[i].hw_sweep.step_decihz);

            if (tmp.items[i].hw_sweep.step_ftw == 0UL) {
                tmp.items[i].hw_sweep.step_ftw = 1UL;
            }
            tmp.items[i].hw_sweep.fast_flags = 0U;
            tmp.items[i].hw_sweep.gate_ttl = 0U;
            tmp.items[i].hw_sweep.phase_base_pow = 0U;
            tmp.items[i].hw_sweep.phase_peak_pow = 0U;
            tmp.items[i].hw_sweep.phase_half_ticks = 0UL;
            tmp.items[i].hw_sweep.phase_delta_word = 0UL;
            tmp.items[i].hw_sweep.phase_cw1 = 0UL;
            tmp.items[i].hw_sweep.phase_lsrr = 0UL;
        }
        else if (tmp.items[i].type == SEQ_ITEM_FAST_HW_SWEEP) {
            unsigned int flags;
            unsigned int gate_ttl;
            unsigned long half_ticks;
            unsigned long ramp_intervals;
            unsigned long span_word;

            if ((pos + 22U) > len) return false;
            sweep_count++;
            if (sweep_count > SEQ_MAX_SWEEP_ITEMS) return false;

            tmp.items[i].hw_sweep.start_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].hw_sweep.stop_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].hw_sweep.step_decihz = rd_u32_le(&p[pos]); pos += 4U;
            tmp.items[i].hw_sweep.tstep_ticks = rd_u32_le(&p[pos]); pos += 4U;
            flags = p[pos++];
            gate_ttl = p[pos++];
            tmp.items[i].hw_sweep.phase_base_pow = (unsigned short)rd_u16_le(&p[pos]); pos += 2U;
            tmp.items[i].hw_sweep.phase_peak_pow = (unsigned short)rd_u16_le(&p[pos]); pos += 2U;

            if (tmp.items[i].hw_sweep.step_decihz == 0UL) return false;
            if (tmp.items[i].hw_sweep.tstep_ticks < SEQ_FAST_HW_SWEEP_MIN_TICKS) return false;
            if (flags > (SEQ_FAST_FLAG_WAIT_TRIGGER | SEQ_FAST_FLAG_PHASE_TRIANGLE)) return false;
            if ((flags & SEQ_FAST_FLAG_WAIT_TRIGGER) != 0U && !Seq_IsValidTtl(gate_ttl)) return false;

            tmp.items[i].hw_sweep.fast_flags = (unsigned char)flags;
            tmp.items[i].hw_sweep.gate_ttl = (unsigned char)gate_ttl;
            tmp.items[i].wait_for_trigger =
                ((flags & SEQ_FAST_FLAG_WAIT_TRIGGER) != 0U) ? 1U : 0U;
            tmp.items[i].gate_ttl = (unsigned char)gate_ttl;
            tmp.items[i].hw_sweep.start_ftw = FreqDeciToFTW(tmp.items[i].hw_sweep.start_decihz);
            tmp.items[i].hw_sweep.stop_ftw = FreqDeciToFTW(tmp.items[i].hw_sweep.stop_decihz);
            tmp.items[i].hw_sweep.step_ftw = FreqDeciToFTW(tmp.items[i].hw_sweep.step_decihz);
            if (tmp.items[i].hw_sweep.step_ftw == 0UL) tmp.items[i].hw_sweep.step_ftw = 1UL;

            if ((flags & SEQ_FAST_FLAG_PHASE_TRIANGLE) != 0U) {
                if (tmp.items[i].hw_sweep.phase_peak_pow <= tmp.items[i].hw_sweep.phase_base_pow) return false;
                half_ticks = tmp.items[i].hw_sweep.tstep_ticks / 2UL;
                if (half_ticks < SEQ_PHASE_TIMER_MIN_TICKS) return false;
                /* one AD9959 ramp increment every 8 ns; one Tiva tick is 8.333 ns */
                ramp_intervals = (half_ticks * 25UL) / 24UL;
                if (ramp_intervals == 0UL) ramp_intervals = 1UL;
                span_word = (unsigned long)tmp.items[i].hw_sweep.phase_peak_pow
                          - (unsigned long)tmp.items[i].hw_sweep.phase_base_pow;
                tmp.items[i].hw_sweep.phase_delta_word =
                    (span_word + ramp_intervals - 1UL) / ramp_intervals;
                if (tmp.items[i].hw_sweep.phase_delta_word == 0UL) {
                    tmp.items[i].hw_sweep.phase_delta_word = 1UL;
                }
                tmp.items[i].hw_sweep.phase_half_ticks = half_ticks;
                tmp.items[i].hw_sweep.phase_cw1 =
                    ((unsigned long)tmp.items[i].hw_sweep.phase_peak_pow & 0x3FFFUL) << 18;
                tmp.items[i].hw_sweep.phase_lsrr = 0x0101UL;
            } else {
                tmp.items[i].hw_sweep.phase_half_ticks = 0UL;
                tmp.items[i].hw_sweep.phase_delta_word = 0UL;
                tmp.items[i].hw_sweep.phase_cw1 = 0UL;
                tmp.items[i].hw_sweep.phase_lsrr = 0UL;
            }
        }
        else {
            return false;
        }
    }

    if (pos != len) return false;

    for (d = 0U; d < DDS_COUNT; d++) {
        for (c = 0U; c < DDS_CH_COUNT; c++) {
            if (target_mask & Seq_GlobalBit(d,c)) {
                unsigned int fast_count = 0U;
                g_seq_def[d][c] = tmp;
                g_seq_fast_plan[d].valid = false;

                for (i = 0U; i < tmp.n_items; i++) {
                    if (tmp.items[i].type == SEQ_ITEM_FAST_HW_SWEEP) {
                        fast_count++;
                        if (fast_count > 1U) return false;
                        if ((tmp.items[i].hw_sweep.fast_flags & SEQ_FAST_FLAG_PHASE_TRIANGLE) != 0U && d != 2U) {
                            /* Current PROFILE wiring exposes hardware CCP/PWM
                             * only on DDS3 (PM3..PM0). */
                            return false;
                        }
                        if (!Seq_BuildFastPlan(d,c,i,&tmp.items[i].hw_sweep)) return false;
                    }
                }
            }
        }
    }

    return true;
}

static bool parse_seq_static_set_frame(const unsigned char *p,unsigned int len)
{
    unsigned int d;
    unsigned int c;
    unsigned long ftw;
    unsigned int pow_word;
    unsigned int asf;

    if (len != 10U) return false;

    d = p[0];
    c = p[1];

    /*
     * V36.5 TEST:
     * Les 4 octets [2..5] contiennent désormais directement
     * le FTW 32 bits calculé côté PC.
     */
    ftw = rd_u32_le(&p[2]);

    pow_word = rd_u16_le(&p[6]);
    asf = rd_u16_le(&p[8]);

    if (d >= DDS_COUNT || c >= DDS_CH_COUNT) return false;
    if (pow_word > 0x3FFFU || asf > 0x03FFU) return false;
    if (g_seq_hw_engine[d].active) return false;

    Seq_SetProfileLow(d,c);

    Seq_WriteRegFast(
        d,
        CSR_ADRESS,
        Seq_CSR_FromChannel(c),
        CSR_NUM_BYTE
    );

    Seq_WriteRegFast(
        d,
        CFR_ADRESS,
        SEQ_CFR_SINGLE_TONE_SAFE,
        CFR_NUM_BYTE
    );

    /*
     * IMPORTANT :
     * plus aucune conversion de fréquence sur le Tiva.
     * Le FTW reçu est écrit tel quel dans l'AD9959.
     */
    Seq_WriteRegFast(
        d,
        FTW_ADRESS,
        ftw,
        FTW_NUM_BYTE
    );

    Seq_WriteRegFast(
        d,
        CPOW_ADRESS,
        (unsigned long)pow_word,
        CPOW_NUM_BYTE
    );

    Seq_WriteRegFast(
        d,
        ACR_ADRESS,
        SEQ_ACR_AMP_ENABLE | (unsigned long)asf,
        ACR_NUM_BYTE
    );

    Seq_IOUpdateOneFast(d);

    return true;
}

//===========================================================


//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
//========== Réception UART : machine à états (simple OU trame) ==========

#define SEQ_SYNC0      0xA5u
#define SEQ_SYNC1      0x5Au
#define DDS_UART_BAUDRATE 921600U
#define SEQ_FRAME_MAX  1024U // On met cette valeur importante car maintenant la séquence peut potentiellement contenir plusieurs sweep (environ 17 octets par sweep) et on ne veut pas dépasser la RAM disponible.

typedef enum {
    RX_START = 0,
    RX_LEGACY,
    RX_WAIT_SYNC1,
    RX_HDR0,
    RX_HDR1,
    RX_HDR2,
    RX_PAYLOAD,
    RX_CHK
} rx_state_t;

static rx_state_t rx_state = RX_START;
static unsigned char frame_buf[SEQ_FRAME_MAX];
static unsigned int frame_len = 0U;
static unsigned int frame_idx = 0U;
static unsigned char frame_type = 0U;
static unsigned char frame_chk = 0U;
static unsigned char frame_h0 = 0U;
static unsigned char frame_h1 = 0U;
static unsigned char frame_h2 = 0U;

/* V33 UART/ACK path: RESET is acknowledged inside the UART ISR, while the
 * potentially blocking SSI reset work is deferred to the main loop. */
static volatile bool g_seq_reset_pending = false;

/* Read these variables in CCS Expressions to locate an ACK failure exactly. */
volatile uint32_t g_v33_uart_signature = 0x56333231UL; /* retained for legacy diagnostics */
volatile uint32_t g_v34_uart_signature = 0x56333430UL; /* "V340" */
volatile uint32_t g_uart_rx_irq_count = 0UL;
volatile uint32_t g_uart_rx_byte_count = 0UL;
volatile uint32_t g_uart_valid_frame_count = 0UL;
volatile uint32_t g_uart_reset_request_count = 0UL;
volatile uint32_t g_uart_ack_count = 0UL;

/* RESET diagnostics. ACK is still sent immediately by UARTIntHandler; these
 * counters describe the deferred physical reset performed in main(). */
volatile uint32_t g_reset_hard_begin_count = 0UL;
volatile uint32_t g_reset_hard_end_count = 0UL;
volatile uint32_t g_reset_hard_last_ticks = 0UL;

/* =============================================================================
 * RECEPTION UART
 * =============================================================================
 * Cette ISR reconstruit les trames A5 5A envoyées par le GUI puis les route
 * vers STOP/RESET/SINGLE-TONE ou vers le moteur de tables.
 */
void UARTIntHandler(void)
{
    uint32_t ui32Status;
    unsigned char b;

    g_uart_rx_irq_count++;
    ui32Status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, ui32Status);

    while (UARTCharsAvail(UART0_BASE))
    {
        b = (unsigned char)UARTCharGet(UART0_BASE);
        g_uart_rx_byte_count++;

        switch (rx_state)
        {
        case RX_START:
            if (b == SEQ_SYNC0) {
                rx_state = RX_WAIT_SYNC1;          // peut-être une trame
            } else {
                tempDataUART[0] = b;               // paquet simple classique
                UARTCharPut(UART0_BASE, b);        // écho (le bouton simple lit 22 octets)
                tUART = 1;
                rx_state = RX_LEGACY;
            }
            break;

        case RX_WAIT_SYNC1:
            if (b == SEQ_SYNC1) {
                rx_state = RX_HDR0;                // trame complexe confirmée
            } else {                               // fausse alerte : paquet simple en 0xA5
                tempDataUART[0] = SEQ_SYNC0; UARTCharPut(UART0_BASE, SEQ_SYNC0);
                tempDataUART[1] = b;         UARTCharPut(UART0_BASE, b);
                tUART = 2;
                rx_state = RX_LEGACY;
            }
            break;

        case RX_LEGACY:
            tempDataUART[tUART] = b;
            UARTCharPut(UART0_BASE, b);            // écho
            tUART++;
            if (tUART == 22) {
                write_immediate();
                tUART = 0;
                rx_state = RX_START;
            }
            break;

        case RX_HDR0:
            frame_h0 = b;
            rx_state = RX_HDR1;
            break;

        case RX_HDR1:
            frame_h1 = b;
            rx_state = RX_HDR2;
            break;

        case RX_HDR2:
        {
            unsigned int body_len;
            frame_h2 = b;
            body_len = (unsigned int)frame_h1 | ((unsigned int)frame_h2 << 8);
            frame_type = frame_h0;
            frame_len = body_len;
            frame_chk = frame_h0 ^ frame_h1 ^ frame_h2;

            frame_idx = 0U;
            if (frame_len > SEQ_FRAME_MAX)  { UARTCharPut(UART0_BASE, 0x15); rx_state = RX_START; }
            else if (frame_len == 0U)       { rx_state = RX_CHK; }
            else                            { rx_state = RX_PAYLOAD; }
            break;
        }

        case RX_PAYLOAD:
            frame_buf[frame_idx++] = b;
            frame_chk ^= b;

            if (frame_idx == frame_len) {
                rx_state = RX_CHK;
            }
            break;

        case RX_CHK:
            if (b == frame_chk) {
                bool ok = false;
                bool defer_arm_ack = false;
                g_uart_valid_frame_count++;
                if (frame_type >= TBL34_FRAME_BEGIN && frame_type <= TBL365_FRAME_FAST_END) {
                    Engine_SwitchToV34Table();
                    ok = Tbl34_ParseFrame(frame_type, frame_buf, frame_len);
                    if (frame_type == TBL34_FRAME_ARM && ok) defer_arm_ack = true;
                } else if (frame_type == SEQ_FRAME_LOAD) {
                    Engine_SwitchToGeneralV34();
                    ok = parse_seq_load_frame(frame_buf, frame_len);
                } else if (frame_type == SEQ_FRAME_ARM) {
                    Engine_SwitchToGeneralV34();
                    ok = parse_seq_arm_frame(frame_buf, frame_len);
                    defer_arm_ack = ok;
                } else if (frame_type == SEQ_FRAME_STATIC_SET) {
                    if (g_v34_table_mode && Tbl34_AnyActive()) ok = false;
                    else {
                        if (g_v34_table_mode) { Tbl34_StopAll(); Engine_SwitchToGeneralV34(); }
                        ok = parse_seq_static_set_frame(frame_buf, frame_len);
                    }
                } else if (frame_type == SEQ_FRAME_STOP && frame_len == 0U) {
                    Tbl34_StopAll();
                    Tbl34_Service();
                    Seq_StopAll();
                    g_v34_table_mode = false;
                    Engine_ConfigureTimersForGeneral();
                    ok = true;
                } else if (frame_type == SEQ_FRAME_RESET && frame_len == 0U) {
                    /* ACK first; the physical DDS reset remains deferred. */
                    g_uart_reset_request_count++;
                    g_seq_reset_pending = true;
                    ok = true;
                }

                if (!defer_arm_ack) {
                    UARTCharPut(UART0_BASE, ok ? 0x06 : 0x15);
                    if (ok) g_uart_ack_count++;
                }
            } else {
                UARTCharPut(UART0_BASE, 0x15);
            }
            rx_state = RX_START;
            break;
        }
    }
}
//=======================================================================

void init_UART0(void)
{
  // Enable Peripheral UART0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  /* Un débit UART unique est utilisé pour les deux moteurs.
   * Le protocole général fonctionne lui aussi à 921600 bauds.
   * g_ui32SysClock contient la fréquence réellement retournée par
   * SysCtlClockFreqSet(), contrairement à un second calcul implicite. */
  UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, DDS_UART_BAUDRATE,
  (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}
// Conversion Fr quence (Hz) -> Mot de R glage de Fr quence (FTW 32-bits)
unsigned long FreqToFTW(unsigned long frequency_hz)
{
    uint64_t numerator;
    uint64_t denominator;

    numerator =
        ((uint64_t)frequency_hz << 32);

    denominator =
        (uint64_t)SYSCLKDDS;

    return (unsigned long)(
        (numerator + denominator / 2ULL)
        / denominator
    );
}
// Conversion déci-Hz (dixièmes de Hz) -> FTW 32 bits, arrondi au plus proche.
// L'entrée vaut Hz * 10 : 100000000 => 10.0 MHz, 100000001 => 10.0000001 MHz.
unsigned long FreqDeciToFTW(
    unsigned long frequency_decihz
)
{
    uint64_t numerator;
    uint64_t denominator;

    numerator =
        ((uint64_t)frequency_decihz << 32);

    denominator =
        10ULL * (uint64_t)SYSCLKDDS;

    return (unsigned long)(
        (numerator + denominator / 2ULL)
        / denominator
    );
}

static void Seq_PrecomputeLinearSweep(seq_sweep_t *s)
{
    unsigned long user_step_ftw;
    unsigned long diff_ftw;
    unsigned long user_srr;
    uint64_t physical_intervals;
    uint64_t duration_ticks;

    s->rising = (s->stop_decihz >= s->start_decihz) ? 1U : 0U;
    s->ftw_low = FreqDeciToFTW(s->rising ? s->start_decihz : s->stop_decihz);
    s->ftw_high = FreqDeciToFTW(s->rising ? s->stop_decihz : s->start_decihz);

    user_step_ftw = FreqDeciToFTW(s->step_decihz);
    if (user_step_ftw == 0UL) user_step_ftw = 1UL;

    diff_ftw = (s->ftw_high > s->ftw_low) ? (s->ftw_high - s->ftw_low) : 1UL;
    user_srr = (s->tstep_ns + 4UL) / 8UL;
    if (user_srr < 1UL) user_srr = 1UL;
    if (user_srr > 255UL) user_srr = 255UL;

    if (s->rising) {
        s->rdw = user_step_ftw;
        s->fdw = user_step_ftw;
        s->lsrr = (user_srr << 8) | user_srr;
    } else {
        s->rdw = diff_ftw;
        s->fdw = user_step_ftw;
        s->lsrr = (user_srr << 8) | 1UL;
    }

    /*
     * V28 : le scheduler attend désormais la durée physique du sweep AD9959,
     * calculée à partir des FTW réellement programmés. Cela évite un plateau
     * à la fréquence finale lorsque l'arrondi FTW exige un intervalle de plus
     * que le nombre de points théorique envoyé par le GUI.
     *
     * Un tick SRR vaut 8 ns. À 120 MHz, 8 ns = 24/25 tick TM4C.
     */
    physical_intervals =
        ((uint64_t)diff_ftw + (uint64_t)user_step_ftw - 1ULL)
        / (uint64_t)user_step_ftw;

    if (physical_intervals == 0ULL) physical_intervals = 1ULL;

    duration_ticks =
        (physical_intervals * (uint64_t)user_srr * 24ULL + 12ULL)
        / 25ULL;

    if (duration_ticks < (uint64_t)SEQ_TIMER_MIN_TICKS) {
        duration_ticks = (uint64_t)SEQ_TIMER_MIN_TICKS;
    }
    if (duration_ticks > 0xFFFFFFFFULL) {
        duration_ticks = 0xFFFFFFFFULL;
    }

    s->duration_ticks = (unsigned long)duration_ticks;
}

static inline unsigned int Seq_GlobalBit(unsigned int dds_idx, unsigned int ch_idx)
{
    return 1U << (dds_idx * 4U + ch_idx);
}

static inline int Seq_TtlToIndex(unsigned int ttl)
{
    if (ttl == SEQ_TRIGGER_TTL6) return 0;
    if (ttl == SEQ_TRIGGER_TTL7) return 1;
    if (ttl == SEQ_TRIGGER_TTL8) return 2;
    return -1;
}

static inline bool Seq_IsValidTtl(unsigned int ttl)
{
    return (ttl == SEQ_TRIGGER_TTL6 || ttl == SEQ_TRIGGER_TTL7 || ttl == SEQ_TRIGGER_TTL8);
}

static unsigned int Seq_PriorityRank(unsigned int ttl)
{
    unsigned int i;

    for (i = 0U; i < 3U; i++) {
        if (g_seq_priority_ttl[i] == ttl) return i;
    }

    return 3U;
}

static inline unsigned int Seq_CSR_FromChannel(unsigned int ch_idx)
{
    return ((1U << ch_idx) << 4) | 0x06U;
}

static bool Seq_WriteRegBounded(unsigned int dds_idx, int address,
                                unsigned long data, int nbytes)
{
    volatile uint32_t *dr;
    volatile uint32_t *sr;
    uint32_t t0;
    int shift;

    if (dds_idx == 0U) {
        dr = (volatile uint32_t *)0x40008008UL;
        sr = (volatile uint32_t *)0x4000800CUL;
    } else if (dds_idx == 1U) {
        dr = (volatile uint32_t *)0x40009008UL;
        sr = (volatile uint32_t *)0x4000900CUL;
    } else if (dds_idx == 2U) {
        dr = (volatile uint32_t *)0x4000A008UL;
        sr = (volatile uint32_t *)0x4000A00CUL;
    } else {
        return false;
    }

    if (nbytes <= 0 || nbytes > 4) return false;

    t0 = SEQ_DWT_CYCCNT;
    while (((*sr) & SSI_SR_TNF) == 0U) {
        if ((uint32_t)(SEQ_DWT_CYCCNT - t0) > SEQ_ARM_SSI_TIMEOUT_TICKS) {
            g_seq_dbg_arm_ssi_timeout_count++;
            return false;
        }
    }
    *dr = (uint32_t)address;

    shift = (nbytes - 1) * 8;
    while (shift >= 0) {
        t0 = SEQ_DWT_CYCCNT;
        while (((*sr) & SSI_SR_TNF) == 0U) {
            if ((uint32_t)(SEQ_DWT_CYCCNT - t0) > SEQ_ARM_SSI_TIMEOUT_TICKS) {
                g_seq_dbg_arm_ssi_timeout_count++;
                return false;
            }
        }
        *dr = (uint32_t)((data >> shift) & 0xFFUL);
        shift -= 8;
    }

    t0 = SEQ_DWT_CYCCNT;
    while (((*sr) & SSI_SR_BSY) != 0U) {
        if ((uint32_t)(SEQ_DWT_CYCCNT - t0) > SEQ_ARM_SSI_TIMEOUT_TICKS) {
            g_seq_dbg_arm_ssi_timeout_count++;
            return false;
        }
    }

    return true;
}

static inline void Seq_WriteReg(unsigned int dds_idx, int address, unsigned long data, int nbytes)
{
    if (g_seq_arm_preload_bounded) {
        if (!Seq_WriteRegBounded(dds_idx,address,data,nbytes)) {
            g_seq_arm_preload_write_failed = true;
        }
        return;
    }

    switch (dds_idx)
    {
        case 0: ssi0PutData(address,data,nbytes); break;
        case 1: ssi1PutData(address,data,nbytes); break;
        case 2: ssi2PutData(address,data,nbytes); break;
        default: break;
    }
}

static inline void Seq_WriteRegFast(unsigned int dds_idx,int address,unsigned long data,int nbytes)
{
    volatile uint32_t *dr;
    volatile uint32_t *sr;
    int shift;

    if (dds_idx == 0U) {
        dr = (volatile uint32_t *)0x40008008UL;
        sr = (volatile uint32_t *)0x4000800CUL;
    } else if (dds_idx == 1U) {
        dr = (volatile uint32_t *)0x40009008UL;
        sr = (volatile uint32_t *)0x4000900CUL;
    } else if (dds_idx == 2U) {
        dr = (volatile uint32_t *)0x4000A008UL;
        sr = (volatile uint32_t *)0x4000A00CUL;
    } else {
        return;
    }

    while (((*sr) & SSI_SR_TNF) == 0U) {}
    *dr = (uint32_t)address;

    shift = (nbytes - 1) * 8;
    while (shift >= 0) {
        while (((*sr) & SSI_SR_TNF) == 0U) {}
        *dr = (uint32_t)((data >> shift) & 0xFFUL);
        shift -= 8;
    }

    while (((*sr) & SSI_SR_BSY) != 0U) {}
}

static bool Seq_PreloadFirstItem_NoUpdate(unsigned int d,unsigned int c){
    seq_def_t *def =
        (seq_def_t *)&g_seq_def[d][c];

    seq_item_t *item;

    if (!def->valid) return false;
    if (def->n_items == 0U) return false;

    item = (seq_item_t *)&def->items[0];

    if (item->wait_for_trigger) return false;

    /*
     * Le DDS concerné est supposé inactif côté séquence.
     */
    Seq_SetProfileLow(d,c);

    if (item->type == SEQ_ITEM_STEP) {

        Seq_WriteReg(
            d,
            CSR_ADRESS,
            Seq_CSR_FromChannel(c),
            CSR_NUM_BYTE
        );

        Seq_WriteReg(
            d,
            CFR_ADRESS,
            SEQ_CFR_SINGLE_TONE_SAFE,
            CFR_NUM_BYTE
        );

        Seq_WriteReg(
            d,
            FTW_ADRESS,
            item->step.ftw,
            FTW_NUM_BYTE
        );

        return true;
    }

    if (item->type == SEQ_ITEM_STEP_EX) {

        Seq_WriteReg(d,CSR_ADRESS,Seq_CSR_FromChannel(c),CSR_NUM_BYTE);

        /* V30.4 : remise à plat inconditionnelle. Le canal peut sortir d'une
         * séquence précédente laissée en mode sweep. */

        Seq_WriteReg(d,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);

        if (item->step.flags & SEQ_STEPEX_FLAG_FREQ) {
            Seq_WriteReg(d,FTW_ADRESS,item->step.ftw,FTW_NUM_BYTE);
        }

        if (item->step.flags & SEQ_STEPEX_FLAG_PHASE) {
            Seq_WriteReg(d,CPOW_ADRESS,(unsigned long)item->step.pow,CPOW_NUM_BYTE);
        }

        if (item->step.flags & SEQ_STEPEX_FLAG_AMP) {
            Seq_WriteReg(d,ACR_ADRESS,SEQ_ACR_AMP_ENABLE | (unsigned long)item->step.asf,ACR_NUM_BYTE);
        }

        return true;
    }

    if (item->type == SEQ_ITEM_SWEEP) {

        /*
         * ATTENTION :
         * Seq_ConfigLinearSweep_NoStart()
         * ne doit plus contenir de Seq_SetProfileLow().
         */
        Seq_ConfigLinearSweep_NoStart(
            d,
            c,
            &item->sweep
        );

        return true;
    }

    if (item->type == SEQ_ITEM_FAST_HW_SWEEP) {
        if ((item->hw_sweep.fast_flags & SEQ_FAST_FLAG_WAIT_TRIGGER) != 0U) return false;
        Seq_ConfigFastHwSweep_NoStart(d,c,&item->hw_sweep);
        return true;
    }

    if (item->type == SEQ_ITEM_HW_SWEEP) {

        Seq_WriteReg(
            d,
            CSR_ADRESS,
            Seq_CSR_FromChannel(c),
            CSR_NUM_BYTE
        );

        Seq_WriteReg(
            d,
            CFR_ADRESS,
            SEQ_CFR_SINGLE_TONE_SAFE,
            CFR_NUM_BYTE
        );

        Seq_WriteReg(
            d,
            FTW_ADRESS,
            item->hw_sweep.start_ftw,
            FTW_NUM_BYTE
        );

        return true;
    }

    return false;
}

static bool Seq_InitCurrentItemRuntime_NoWrite(unsigned int d,unsigned int c)
{
    seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];
    seq_def_t *def = (seq_def_t *)&g_seq_def[d][c];
    seq_item_t *item;

    if (!rt->active) return false;
    if (rt->item_idx >= def->n_items) return false;

    item = (seq_item_t *)&def->items[rt->item_idx];

    Seq_UpdateLoopDeadlineStateForCurrentItem(rt,def,item);

    if (item->wait_for_trigger) {
        /* Special items are never activated from a generic preloaded state.
         * Seq_ApplyCurrentItem() must establish their dedicated wait state. */
        return false;
    }

    if (item->type == SEQ_ITEM_STEP) {

        rt->phase = SEQ_RT_STEP;

        rt->remaining_ticks =
            Seq_UsToTicks(
                item->step.duration_us
            );

        rt->sweep_start_pending = false;
        rt->single_tone_ready = true;

        return true;
    }

    if (item->type == SEQ_ITEM_STEP_EX) {

        rt->phase = SEQ_RT_STEP;
        rt->remaining_ticks = Seq_UsToTicks(item->step.duration_us);
        rt->sweep_start_pending = false;
        rt->single_tone_ready = true;

        return true;
    }

    if (item->type == SEQ_ITEM_SWEEP) {

        rt->single_tone_ready = false;

        if (Seq_SweepIsRising(&item->sweep)) {

            rt->phase = SEQ_RT_SWEEP;

            rt->remaining_ticks =
                item->sweep.duration_ticks;

        } else {

            rt->phase =
                SEQ_RT_SWEEP_PRELOAD_DESC;

            rt->remaining_ticks =
                Seq_UsToTicks(
                    SEQ_DESC_PRELOAD_US
                );
        }

        /*
         * Après I/O_UPDATE, le PROFILE HIGH
         * démarrera le sweep.
         */
        rt->sweep_start_pending = true;

        return true;
    }

    if (item->type == SEQ_ITEM_FAST_HW_SWEEP) {
        if ((item->hw_sweep.fast_flags & SEQ_FAST_FLAG_WAIT_TRIGGER) != 0U) return false;
        rt->phase = SEQ_RT_HW_SWEEP;
        rt->remaining_ticks = item->hw_sweep.tstep_ticks;
        rt->sweep_start_pending = false;
        rt->single_tone_ready = ((item->hw_sweep.fast_flags & SEQ_FAST_FLAG_PHASE_TRIANGLE) == 0U);
        rt->hw_current_decihz = item->hw_sweep.start_decihz;
        rt->hw_current_ftw = item->hw_sweep.start_ftw;
        rt->hw_next_update_deadline_ticks = 0UL;
        rt->hw_deadline_valid = false;
        rt->fast_wait_prepared = false;
        return true;
    }

    if (item->type == SEQ_ITEM_HW_SWEEP) {
        rt->phase = SEQ_RT_HW_SWEEP;
        rt->remaining_ticks = item->hw_sweep.tstep_ticks;
        rt->sweep_start_pending = false;
        rt->single_tone_ready = true;
        rt->hw_current_decihz = item->hw_sweep.start_decihz;
        rt->hw_current_ftw = item->hw_sweep.start_ftw;
        rt->hw_next_update_deadline_ticks = 0UL;
        rt->hw_deadline_valid = false;

        return true;
    }

    return false;
}

static bool Seq_ServicePreloadRequest(void)
{
    unsigned int d;
    unsigned int c;

    bool all_ok = true;

    if (!g_seq_preload_request) {
        return true;
    }

    g_seq_preload_request = false;

    /*
     * On repart d'un état logiciel propre.
     */
    for (d = 0U; d < DDS_COUNT; d++) {
        for (c = 0U; c < DDS_CH_COUNT; c++) {
            g_seq_rt[d][c].first_item_preloaded = false;
        }
    }

    for (d = 0U; d < DDS_COUNT; d++) {

        unsigned int dds_mask =
            0x0FU << (4U * d);

        unsigned int mask6 =
            g_seq_arm_mask_by_ttl[0] & dds_mask;

        unsigned int mask7 =
            g_seq_arm_mask_by_ttl[1] & dds_mask;

        unsigned int mask8 =
            g_seq_arm_mask_by_ttl[2] & dds_mask;

        unsigned int owner_count = 0U;
        unsigned int owner_mask = 0U;

        bool dds_busy = false;

        /*
         * Pas de préchargement si une séquence
         * est déjà active sur ce DDS.
         */
        for (c = 0U; c < DDS_CH_COUNT; c++) {
            if (g_seq_rt[d][c].active) {
                dds_busy = true;
                break;
            }
        }

        if (dds_busy) {
            continue;
        }

        if (mask6 != 0U) {
            owner_count++;
            owner_mask = mask6;
        }

        if (mask7 != 0U) {
            owner_count++;
            owner_mask = mask7;
        }

        if (mask8 != 0U) {
            owner_count++;
            owner_mask = mask8;
        }

        /*
         * Sécurité :
         * un seul TTL propriétaire de ce DDS.
         */
        if (owner_count != 1U) {
            continue;
        }

        for (c = 0U; c < DDS_CH_COUNT; c++) {

            unsigned int bit =
                Seq_GlobalBit(d,c);

            if ((owner_mask & bit) == 0U) {
                continue;
            }

            g_seq_arm_preload_bounded = true;
            g_seq_arm_preload_write_failed = false;

            if (Seq_PreloadFirstItem_NoUpdate(d,c)
                && !g_seq_arm_preload_write_failed) {

                g_seq_rt[d][c]
                    .first_item_preloaded = true;
            } else {
                all_ok = false;
            }

            g_seq_arm_preload_bounded = false;
        }
    }

    g_seq_arm_preload_bounded = false;
    return all_ok;
}

static inline void Seq_IOUpdateMask(unsigned int dds_mask)
{
    unsigned int pn_mask = 0U;

    if (dds_mask & 0x01U) {
        HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;
    }

    if (dds_mask & 0x02U) pn_mask |= 0x08U;
    if (dds_mask & 0x04U) pn_mask |= 0x01U;

    if (pn_mask != 0U) {
        HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (pn_mask << 2))) = pn_mask;
    }

    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");

    if (dds_mask & 0x01U) {
        HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00;
    }

    if (pn_mask != 0U) {
        HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (pn_mask << 2))) = 0x00;
    }
}

static inline void Seq_SetFTW_NoUpdate(unsigned int dds_idx,unsigned int ch_idx,unsigned long ftw)
{
    Seq_WriteRegFast(dds_idx,CSR_ADRESS,Seq_CSR_FromChannel(ch_idx),CSR_NUM_BYTE);
    Seq_WriteRegFast(dds_idx,FTW_ADRESS,ftw,FTW_NUM_BYTE);
}

static inline void Seq_WriteFTW_FastSelected_NoUpdate(unsigned int dds_idx,unsigned long ftw)
{
    switch (dds_idx) {
        case 0U:
            ssi0PutData_QSPI_Fast(FTW_ADRESS,(uint32_t)ftw);
            while (SSI0_SR_R & SSI_SR_BSY) {}
            break;

        case 1U:
            ssi1PutData_QSPI_Fast(FTW_ADRESS,(uint32_t)ftw);
            while (SSI1_SR_R & SSI_SR_BSY) {}
            break;

        case 2U:
            ssi2PutData_QSPI_Fast(FTW_ADRESS,(uint32_t)ftw);
            while (SSI2_SR_R & SSI_SR_BSY) {}
            break;

        default:
            break;
    }
}

static inline void Seq_SelectChannelFast_NoUpdate(unsigned int d,unsigned int c)
{
    unsigned long csr = Seq_CSR_FromChannel(c);

    switch (d) {
        case 0U:
            while (!(SSI0_SR_R & SSI_SR_TNF)) {}
            SSI0_DR_R = CSR_ADRESS;
            SSI0_DR_R = csr;
            while (SSI0_SR_R & SSI_SR_BSY) {}
            break;

        case 1U:
            while (!(SSI1_SR_R & SSI_SR_TNF)) {}
            SSI1_DR_R = CSR_ADRESS;
            SSI1_DR_R = csr;
            while (SSI1_SR_R & SSI_SR_BSY) {}
            break;

        case 2U:
            while (!(SSI2_SR_R & SSI_SR_TNF)) {}
            SSI2_DR_R = CSR_ADRESS;
            SSI2_DR_R = csr;
            while (SSI2_SR_R & SSI_SR_BSY) {}
            break;

        default:
            break;
    }
}

static inline void Seq_WriteFTW_ChannelFast_NoUpdate(unsigned int d,unsigned int c,unsigned long ftw)
{
    unsigned int local_mask = (g_seq_active_mask >> (4U * d)) & 0x0FU;
    if ((local_mask & (local_mask - 1U)) != 0U) Seq_SelectChannelFast_NoUpdate(d,c);
    Seq_WriteFTW_FastSelected_NoUpdate(d,ftw);
}

static inline void Seq_IOUpdateOneFast(unsigned int d)
{
    if (d == 0U) {
        HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;
        __asm(" NOP");
        __asm(" NOP");
        __asm(" NOP");
        HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00;
        return;
    }

    if (d == 1U) {
        HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;
        __asm(" NOP");
        __asm(" NOP");
        __asm(" NOP");
        HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00;
        return;
    }

    if (d == 2U) {
        HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01;
        __asm(" NOP");
        __asm(" NOP");
        __asm(" NOP");
        HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00;
    }
}

static inline uint32_t Seq_AdvanceHwDeadline(uint32_t previous_deadline,uint32_t physical_update_ticks,unsigned long logical_ticks)
{
    unsigned long late_ticks = 0UL;
    unsigned long resync_threshold = logical_ticks / 4UL;

    if ((int32_t)(physical_update_ticks - previous_deadline) > 0) late_ticks = (unsigned long)(physical_update_ticks - previous_deadline);
    if (resync_threshold < SEQ_TIMER_RELOAD_MIN_TICKS) resync_threshold = SEQ_TIMER_RELOAD_MIN_TICKS;

    if (late_ticks >= resync_threshold) {
        g_seq_dbg_hw_deadline_resync++;
        return physical_update_ticks + logical_ticks;
    }

    return previous_deadline + logical_ticks;
}

static inline bool Seq_IsSingleActiveHwSweep(void)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;
    int idx;
    unsigned int d;
    unsigned int c;

    if (mask == 0U) return false;
    if ((mask & (mask - 1U)) != 0U) return false;

    idx = Seq_FirstSetBit12(mask);
    if (idx < 0) return false;

    d = (unsigned int)idx / 4U;
    c = (unsigned int)idx % 4U;

    return g_seq_rt[d][c].active &&
           g_seq_rt[d][c].phase == SEQ_RT_HW_SWEEP;
}



static bool Seq_ArmSingleActiveHwSweepDeadline(unsigned long logical_ticks)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;
    int idx;
    unsigned int d;
    unsigned int c;
    seq_rt_t *rt;

    if (mask == 0U) return false;
    if ((mask & (mask - 1U)) != 0U) return false;

    idx = Seq_FirstSetBit12(mask);
    if (idx < 0) return false;

    d = (unsigned int)idx / 4U;
    c = (unsigned int)idx % 4U;
    rt = (seq_rt_t *)&g_seq_rt[d][c];

    if (!rt->active) return false;
    if (rt->phase != SEQ_RT_HW_SWEEP) return false;
    if (!rt->hw_deadline_valid) return false;

    Seq_ArmTimer4ForUpdateDeadline(rt->hw_next_update_deadline_ticks,g_seq_hw_fast_prep_estimate_ticks,logical_ticks);
    g_seq_timer_hw_owner_bit = 1U << (unsigned int)idx;
    return true;
}

static bool Seq_ArmEarliestHwSweepDeadline(unsigned long logical_ticks)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;
    unsigned int selected_bit = 0U;
    seq_rt_t *selected_rt = 0;
    unsigned long prep_estimate;
    unsigned long max_lead;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;
        unsigned int d;
        unsigned int c;
        seq_rt_t *rt;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;
        mask &= ~bit;
        rt = (seq_rt_t *)&g_seq_rt[d][c];

        if (!rt->active || rt->remaining_ticks != logical_ticks) continue;
        if (selected_rt != 0) return false;
        if (rt->phase != SEQ_RT_HW_SWEEP || !rt->hw_deadline_valid) return false;

        selected_rt = rt;
        selected_bit = bit;
    }

    if (selected_rt == 0) return false;

    if (((g_seq_active_mask & 0x0FFFU) & ((g_seq_active_mask & 0x0FFFU) - 1U)) != 0U) prep_estimate = g_seq_hw_multi_prep_estimate_ticks;
    else prep_estimate = g_seq_hw_fast_prep_estimate_ticks;

    if (logical_ticks > (2UL * SEQ_TIMER_RELOAD_MIN_TICKS)) {
        max_lead = logical_ticks - (2UL * SEQ_TIMER_RELOAD_MIN_TICKS);
        if (prep_estimate > max_lead) prep_estimate = max_lead;
    }

    Seq_ArmTimer4ForUpdateDeadline(selected_rt->hw_next_update_deadline_ticks,prep_estimate,logical_ticks);
    g_seq_timer_hw_owner_bit = selected_bit;
    return true;
}

static inline void Seq_DisableLinearSweep_NoUpdate(unsigned int dds_idx, unsigned int ch_idx){
    /*
     * IMPORTANT :
     * ne pas toucher au PROFILE ici.
     *
     * La désactivation du sweep est seulement préparée
     * dans les registres tamponnés.
     */
    Seq_WriteRegFast(dds_idx,CSR_ADRESS,Seq_CSR_FromChannel(ch_idx),CSR_NUM_BYTE);
    Seq_WriteRegFast(dds_idx,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);
}

static inline unsigned long Seq_ClampTimerTicks64(uint64_t ticks)
{
    if (ticks < (uint64_t)SEQ_TIMER_MIN_TICKS) ticks = (uint64_t)SEQ_TIMER_MIN_TICKS;
    if (ticks > 0xFFFFFFFFULL) ticks = 0xFFFFFFFFULL;
    return (unsigned long)ticks;
}

static inline unsigned long Seq_UsToTicks(unsigned long delay_us)
{
    return Seq_ClampTimerTicks64((uint64_t)delay_us * (uint64_t)SEQ_TIMER_TICKS_PER_US);
}

static uint64_t Seq_GetItemNominalDurationTicks(const seq_item_t *item)
{
    uint64_t diff;
    uint64_t step;
    uint64_t intervals;

    if (item->type == SEQ_ITEM_STEP || item->type == SEQ_ITEM_STEP_EX) {
        return (uint64_t)Seq_UsToTicks(item->step.duration_us);
    }

    if (item->type == SEQ_ITEM_SWEEP) {
        return (uint64_t)item->sweep.duration_ticks;
    }

    if (item->type == SEQ_ITEM_HW_SWEEP || item->type == SEQ_ITEM_FAST_HW_SWEEP) {
        uint64_t tolerance = (uint64_t)SEQ_HW_FREQ_TOLERANCE_DECIHZ;

        step = (uint64_t)item->hw_sweep.step_decihz;

        if (step == 0ULL) {
            return 0ULL;
        }

        if (item->hw_sweep.stop_decihz >= item->hw_sweep.start_decihz) {
            diff = (uint64_t)item->hw_sweep.stop_decihz - (uint64_t)item->hw_sweep.start_decihz;
        } else {
            diff = (uint64_t)item->hw_sweep.start_decihz - (uint64_t)item->hw_sweep.stop_decihz;
        }

        /*
         * Même règle que dans le runtime :
         * le prochain update force directement la cible dès que
         * la distance restante est <= step + tolérance.
         *
         * Nombre d'intervalles :
         * N = ceil((diff - tolérance) / step), avec au moins 1.
         */
        if (diff <= tolerance) {
            intervals = 1ULL;
        } else {
            intervals = (diff - tolerance + step - 1ULL) / step;
            if (intervals == 0ULL) intervals = 1ULL;
        }

        return intervals * (uint64_t)item->hw_sweep.tstep_ticks;
    }

    return 0ULL;
}

static uint64_t Seq_GetLoopNominalDurationTicks(const seq_def_t *def)
{
    unsigned int i;
    uint64_t total_ticks = 0ULL;

    for (i = 0U; i < def->n_items; i++) {
        total_ticks += Seq_GetItemNominalDurationTicks(&def->items[i]);
    }

    return total_ticks;
}




static inline bool Seq_HwSweepOneIntervalRemaining(const seq_hw_sweep_t *s,unsigned long current_decihz)
{
    uint64_t diff;

    if (s->stop_decihz >= current_decihz) {
        diff = (uint64_t)s->stop_decihz - (uint64_t)current_decihz;
    } else {
        diff = (uint64_t)current_decihz - (uint64_t)s->stop_decihz;
    }

    return diff <= (uint64_t)s->step_decihz;
}

static inline void Seq_UpdateLoopDeadlineStateForCurrentItem(seq_rt_t *rt,const seq_def_t *def,const seq_item_t *item)
{
    rt->loop_deadline_active = false;

    if ((rt->item_idx + 1U) != def->n_items) {
        return;
    }

    if (item->type == SEQ_ITEM_STEP || item->type == SEQ_ITEM_STEP_EX) {
        rt->loop_deadline_active = true;
        return;
    }

    if (item->type == SEQ_ITEM_SWEEP) {
        if (Seq_SweepIsRising(&item->sweep)) {
            rt->loop_deadline_active = true;
        }

        return;
    }

    if (item->type == SEQ_ITEM_HW_SWEEP || item->type == SEQ_ITEM_FAST_HW_SWEEP) {
        if (Seq_HwSweepOneIntervalRemaining(&item->hw_sweep,item->hw_sweep.start_decihz)) {
            rt->loop_deadline_active = true;
        }
    }
}

static void Seq_ApplyLoopEndDeadlineCompensation(uint32_t isr_origin_ticks)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;
        unsigned int d;
        unsigned int c;
        seq_rt_t *rt;
        uint64_t ideal_offset_ticks;
        uint32_t deadline_ticks;
        int32_t logical_ticks;

        if (idx < 0) {
            break;
        }

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;
        rt = (seq_rt_t *)&g_seq_rt[d][c];

        if (rt->phase != SEQ_RT_WAIT_TRIGGER && rt->loop_deadline_active && rt->sequence_clock_started && rt->loop_duration_ticks != 0ULL) {
            ideal_offset_ticks = (uint64_t)(rt->loop_idx + 1U) * rt->loop_duration_ticks;
            deadline_ticks = rt->sequence_start_ticks + (uint32_t)ideal_offset_ticks;
            logical_ticks = (int32_t)(deadline_ticks - isr_origin_ticks) - (int32_t)SEQ_LOOP_DEADLINE_LEAD_TICKS;

            if (logical_ticks > (int32_t)SEQ_TIMER_MIN_TICKS) {
                rt->remaining_ticks = (unsigned long)logical_ticks;
            } else {
                rt->remaining_ticks = SEQ_TIMER_MIN_TICKS;
            }
        }

        mask &= ~bit;
    }
}



static inline void Seq_ConfigLinearSweep_NoStart(unsigned int dds_idx,unsigned int ch_idx,const seq_sweep_t *s)
{
    /* V28.1 : uniquement des écritures entières pré-calculées. */
    Seq_WriteRegFast(dds_idx,CSR_ADRESS,Seq_CSR_FromChannel(ch_idx),CSR_NUM_BYTE);
    Seq_WriteRegFast(dds_idx,CFR_ADRESS,SEQ_CFR_FREQ_SWEEP_SAFE,CFR_NUM_BYTE);
    Seq_WriteRegFast(dds_idx,LSRR_ADRESS,s->lsrr,LSRR_NUM_BYTE);
    Seq_WriteRegFast(dds_idx,RDW_ADRESS,s->rdw,RDW_NUM_BYTE);
    Seq_WriteRegFast(dds_idx,FDW_ADRESS,s->fdw,FDW_NUM_BYTE);
    Seq_WriteRegFast(dds_idx,FTW_ADRESS,s->ftw_low,FTW_NUM_BYTE);
    Seq_WriteRegFast(dds_idx,FTW1_ADRESS,s->ftw_high,FTW1_NUM_BYTE);
}

static inline void Seq_ConfigFastHwSweep_NoStart(unsigned int d,unsigned int c,const seq_hw_sweep_t *sweep)
{
    Seq_SetProfileLow(d,c);
    Seq_WriteRegFast(d,CSR_ADRESS,Seq_CSR_FromChannel(c),CSR_NUM_BYTE);

    if ((sweep->fast_flags & SEQ_FAST_FLAG_PHASE_TRIANGLE) != 0U) {
        Seq_WriteRegFast(d,CFR_ADRESS,SEQ_CFR_PHASE_SWEEP_SAFE,CFR_NUM_BYTE);
        Seq_WriteRegFast(d,CPOW_ADRESS,(unsigned long)sweep->phase_base_pow,CPOW_NUM_BYTE);
        Seq_WriteRegFast(d,FTW1_ADRESS,sweep->phase_cw1,FTW1_NUM_BYTE);
        Seq_WriteRegFast(d,RDW_ADRESS,sweep->phase_delta_word,RDW_NUM_BYTE);
        Seq_WriteRegFast(d,FDW_ADRESS,sweep->phase_delta_word,FDW_NUM_BYTE);
        Seq_WriteRegFast(d,LSRR_ADRESS,sweep->phase_lsrr,LSRR_NUM_BYTE);
    } else {
        Seq_WriteRegFast(d,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);
    }

    Seq_WriteRegFast(d,FTW_ADRESS,sweep->start_ftw,FTW_NUM_BYTE);
}


static bool Seq_BuildFastPlan(unsigned int d,unsigned int c,unsigned int item_idx,const seq_hw_sweep_t *sweep)
{
    seq_fast_plan_t *plan;
    unsigned long current;
    unsigned long target;
    unsigned long step;
    unsigned int count = 0U;

    if (d >= DDS_COUNT || c >= DDS_CH_COUNT || sweep == 0) return false;

    plan = &g_seq_fast_plan[d];
    plan->valid = false;
    plan->owner_ch = c;
    plan->item_idx = item_idx;
    plan->next_index = 1U;

    current = sweep->start_decihz;
    target = sweep->stop_decihz;
    step = sweep->step_decihz;
    if (step == 0UL) return false;

    plan->ftw[count++] = sweep->start_ftw;

    while (current != target) {
        if (count >= SEQ_FAST_MAX_POINTS) return false;

        if (target > current) {
            current = ((target - current) <= step) ? target : current + step;
        } else {
            current = ((current - target) <= step) ? target : current - step;
        }

        plan->ftw[count++] = (current == target)
            ? sweep->stop_ftw
            : FreqDeciToFTW(current);
    }

    if (count < 2U) return false;
    plan->count = count;
    plan->valid = true;
    return true;
}

static inline bool Seq_QueueFastFtwNonBlocking(unsigned int d,unsigned long ftw)
{
    volatile uint32_t *dr;
    volatile uint32_t *sr;

    if (d == 0U) {
        dr = (volatile uint32_t *)0x40008008UL;
        sr = (volatile uint32_t *)0x4000800CUL;
    } else if (d == 1U) {
        dr = (volatile uint32_t *)0x40009008UL;
        sr = (volatile uint32_t *)0x4000900CUL;
    } else if (d == 2U) {
        dr = (volatile uint32_t *)0x4000A008UL;
        sr = (volatile uint32_t *)0x4000A00CUL;
    } else {
        return false;
    }

    /* The preceding five-byte transfer had the complete 2-us dwell to finish.
     * Never spin in the boundary ISR: a busy SSI is a real deadline miss. */
    if (((*sr) & SSI_SR_BSY) != 0U || ((*sr) & SSI_SR_TNF) == 0U) return false;

    *dr = (uint32_t)FTW_ADRESS;
    *dr = (uint32_t)((ftw >> 24) & 0xFFUL);
    *dr = (uint32_t)((ftw >> 16) & 0xFFUL);
    *dr = (uint32_t)((ftw >> 8) & 0xFFUL);
    *dr = (uint32_t)(ftw & 0xFFUL);
    return true;
}

/* V30 : préparation d'un STEP_EX (fréquence / phase / amplitude combinés)
 * dans les registres tamponnés, sans I/O_UPDATE. Toutes les valeurs sont
 * pré-calculées au chargement : uniquement des écritures SSI entières. */
static void Seq_PrepareStepEx_NoUpdate(unsigned int d,unsigned int c,const seq_item_t *item,seq_rt_t *rt)
{
    Seq_WriteRegFast(d,CSR_ADRESS,Seq_CSR_FromChannel(c),CSR_NUM_BYTE);

    /* V30.4 : l'état single-tone doit être restauré même pour un STEP_EX
     * ne modifiant que la phase ou l'amplitude. */
    if (rt == 0 || !rt->single_tone_ready) {
        Seq_WriteRegFast(d,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);
        if (rt != 0) rt->single_tone_ready = true;
    }

    if (item->step.flags & SEQ_STEPEX_FLAG_FREQ) {
        Seq_WriteRegFast(d,FTW_ADRESS,item->step.ftw,FTW_NUM_BYTE);
    }

    if (item->step.flags & SEQ_STEPEX_FLAG_PHASE) {
        Seq_WriteRegFast(d,CPOW_ADRESS,(unsigned long)item->step.pow,CPOW_NUM_BYTE);
    }

    if (item->step.flags & SEQ_STEPEX_FLAG_AMP) {
        Seq_WriteRegFast(d,ACR_ADRESS,SEQ_ACR_AMP_ENABLE | (unsigned long)item->step.asf,ACR_NUM_BYTE);
    }
}

static void Engine_StopAndClearTimer(unsigned long base)
{
    HWREG(base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(base + TIMER_O_ICR)  = TIMER_TIMA_TIMEOUT;
}

static void Engine_ConfigureTimersForGeneral(void)
{
    IntPrioritySet(INT_TIMER3A, 0x00);
    IntPrioritySet(INT_TIMER4A, 0x20);
    IntPrioritySet(INT_TIMER5A, 0x00);
    IntPrioritySet(INT_TIMER6A, 0x00);
    IntPrioritySet(INT_TIMER7A, 0x00);
    Engine_StopAndClearTimer(TIMER3_BASE);
    Engine_StopAndClearTimer(TIMER4_BASE);
    Engine_StopAndClearTimer(TIMER5_BASE);
    Engine_StopAndClearTimer(TIMER6_BASE);
    Engine_StopAndClearTimer(TIMER7_BASE);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerConfigure(TIMER4_BASE, TIMER_CFG_ONE_SHOT);
    TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER6_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER7_BASE, TIMER_CFG_PERIODIC);
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER6_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER7_BASE, TIMER_TIMA_TIMEOUT);
}

static void Engine_SwitchToV34Table(void)
{
    if (g_v34_table_mode) return;
    /* Stop only the V33 runtime. Do not mute continuous single tones when a
     * V34 table LOAD switches ownership of Timer4/5/6. */
    Seq_StopRuntimeOnly();
    g_seq_arm_ack_pending = false;
    g_v34_table_mode = true;
    Tbl34_EnterMode();
}

static void Engine_SwitchToGeneralV34(void)
{
    if (!g_v34_table_mode) return;
    Tbl34_StopAll();
    /* A fast DDS3 triangle may have borrowed Timer2/Timer3. Restore those
     * peripherals before handing control back to the legacy engine. */
    Tbl34_Service();
    g_v34_table_mode = false;
    Engine_ConfigureTimersForGeneral();
}





unsigned int AngleToPOW(unsigned int angle_degrees) {
    // 16384 est 2^14. La formule est Angle * (2^14 / 360)
    // Nous utilisons 16384.0 pour assurer le calcul en virgule flottante avant la conversion en entier.
    if (angle_degrees > 359) angle_degrees = 0; // S curit 
    return (unsigned int)(((double)angle_degrees * 16384.0) / 360.0);
}

static inline unsigned long Seq_ProfilePortBase(unsigned int dds_idx)
{
    if (dds_idx == 0U) return GPIO_PORTH_BASE;
    if (dds_idx == 1U) return GPIO_PORTP_BASE;
    if (dds_idx == 2U) return GPIO_PORTM_BASE;
    return 0UL;
}

static inline unsigned char Seq_ProfilePinMask(unsigned int dds_idx, unsigned int ch_idx)
{
    if (dds_idx == 0U) {
        switch (ch_idx) {
            case 0U: return 0x08U;  // DDS1 CH0 -> PH3
            case 1U: return 0x04U;  // DDS1 CH1 -> PH2
            case 2U: return 0x02U;  // DDS1 CH2 -> PH1
            case 3U: return 0x01U;  // DDS1 CH3 -> PH0
            default: return 0U;
        }
    }

    if (dds_idx == 1U) {
        switch (ch_idx) {
            case 0U: return 0x20U;  // DDS2 CH0 -> PP5
            case 1U: return 0x10U;  // DDS2 CH1 -> PP4
            case 2U: return 0x08U;  // DDS2 CH2 -> PP3
            case 3U: return 0x04U;  // DDS2 CH3 -> PP2
            default: return 0U;
        }
    }

    if (dds_idx == 2U) {
        switch (ch_idx) {
            case 0U: return 0x08U;  // DDS3 CH0 -> PM3
            case 1U: return 0x04U;  // DDS3 CH1 -> PM2
            case 2U: return 0x02U;  // DDS3 CH2 -> PM1
            case 3U: return 0x01U;  // DDS3 CH3 -> PM0
            default: return 0U;
        }
    }

    return 0U;
}

static inline void Seq_SetProfileLow(unsigned int dds_idx, unsigned int ch_idx)
{
    unsigned long base = Seq_ProfilePortBase(dds_idx);
    unsigned char pin_mask = Seq_ProfilePinMask(dds_idx,ch_idx);

    if (base == 0UL || pin_mask == 0U) return;

    HWREG(base + GPIO_O_DATA + ((unsigned long)pin_mask << 2)) = 0U;
}

static inline void Seq_SetProfileHigh(unsigned int dds_idx, unsigned int ch_idx)
{
    unsigned long base = Seq_ProfilePortBase(dds_idx);
    unsigned char pin_mask = Seq_ProfilePinMask(dds_idx,ch_idx);

    if (base == 0UL || pin_mask == 0U) return;

    HWREG(base + GPIO_O_DATA + ((unsigned long)pin_mask << 2)) = pin_mask;
}

/*
 * V33.2 deterministic PROFILE generator.
 *
 * The current rack routes DDS3 PROFILE0..3 to PM3..PM0.  Those four pins are
 * also GPTM CCP outputs, so the rising edge at every staircase boundary and
 * the falling edge at half-step are produced entirely in hardware.  DDS1
 * (PHx) and DDS2 (PPx) do not expose a CCP alternate function on the wired
 * PROFILE pins and therefore cannot guarantee a 1-us midpoint without a
 * wiring change.
 */
static bool Seq_StartProfilePwm(unsigned int d,
                                unsigned int c,
                                unsigned long period_ticks,
                                unsigned long half_ticks,
                                unsigned long boundary_timer_base,
                                uint32_t *sync_ticks)
{
    unsigned long pwm_base;
    unsigned int pwm_half;
    unsigned long pwm_config;
    unsigned long sync_mask;
    unsigned long pin_config;
    unsigned char pin;
    unsigned long mode_offset;
    unsigned long boundary_sync_mask;

    if (d != 2U || c >= DDS_CH_COUNT) return false;
    if (period_ticks < SEQ_FAST_HW_SWEEP_MIN_TICKS || period_ticks > 0xFFFFUL) return false;
    if (half_ticks == 0UL || half_ticks >= period_ticks) return false;

    switch (c) {
    case 0U: /* PM3 / T3CCP1 */
        pwm_base = TIMER3_BASE;
        pwm_half = TIMER_B;
        pwm_config = TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_PWM;
        sync_mask = TIMER_3B_SYNC;
        pin_config = GPIO_PM3_T3CCP1;
        pin = GPIO_PIN_3;
        mode_offset = TIMER_O_TBMR;
        break;
    case 1U: /* PM2 / T3CCP0 */
        pwm_base = TIMER3_BASE;
        pwm_half = TIMER_A;
        pwm_config = TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_ONE_SHOT;
        sync_mask = TIMER_3A_SYNC;
        pin_config = GPIO_PM2_T3CCP0;
        pin = GPIO_PIN_2;
        mode_offset = TIMER_O_TAMR;
        break;
    case 2U: /* PM1 / T2CCP1 */
        pwm_base = TIMER2_BASE;
        pwm_half = TIMER_B;
        pwm_config = TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_PWM;
        sync_mask = TIMER_2B_SYNC;
        pin_config = GPIO_PM1_T2CCP1;
        pin = GPIO_PIN_1;
        mode_offset = TIMER_O_TBMR;
        break;
    default: /* PM0 / T2CCP0, D3C3 */
        pwm_base = TIMER2_BASE;
        pwm_half = TIMER_A;
        pwm_config = TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_ONE_SHOT;
        sync_mask = TIMER_2A_SYNC;
        pin_config = GPIO_PM0_T2CCP0;
        pin = GPIO_PIN_0;
        mode_offset = TIMER_O_TAMR;
        break;
    }

    if (boundary_timer_base == TIMER4_BASE) boundary_sync_mask = TIMER_4A_SYNC;
    else if (boundary_timer_base == TIMER5_BASE) boundary_sync_mask = TIMER_5A_SYNC;
    else if (boundary_timer_base == TIMER6_BASE) boundary_sync_mask = TIMER_6A_SYNC;
    else if (boundary_timer_base == TIMER7_BASE) boundary_sync_mask = TIMER_7A_SYNC;
    else return false;

    Seq_StopProfilePwm();

    TimerDisable(pwm_base,TIMER_BOTH);
    HWREG(pwm_base + TIMER_O_IMR) = 0UL;
    HWREG(pwm_base + TIMER_O_ICR) = 0xFFFFFFFFUL;
    TimerConfigure(pwm_base,pwm_config);
    TimerLoadSet(pwm_base,pwm_half,period_ticks - 1UL);
    TimerMatchSet(pwm_base,pwm_half,period_ticks - half_ticks - 1UL);
    TimerControlLevel(pwm_base,pwm_half,false); /* TnPWML=0: active high. */

    /* TAPLO/TBPLO makes every reload drive CCP high.  MRSU updates the match
     * synchronously and is required by the data sheet for glitch-free timer
     * synchronization. */
    HWREG(pwm_base + mode_offset) |= 0x00000C00UL;

    GPIOPinConfigure(pin_config);
    GPIOPinTypeTimer(GPIO_PORTM_BASE,pin);

    /* Both counters are enabled, then one GPTMSYNC write creates the common
     * period origin.  The synthetic timeout is cleared while the boundary
     * interrupt is masked; the first real FTW boundary occurs one full period
     * later. */
    /* The boundary timer interrupts 0.5 us before timeout.  The ISR then
     * waits on DWT until the exact hardware period boundary, so I/O_UPDATE is
     * no longer delayed by interrupt-entry latency. */
    TimerIntDisable(boundary_timer_base,TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
    HWREG(boundary_timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(boundary_timer_base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
    HWREG(boundary_timer_base + TIMER_O_TAILR) = period_ticks - 1UL;
    TimerMatchSet(boundary_timer_base,TIMER_A,SEQ_FAST_WAKE_LEAD_TICKS - 1UL);

    TimerEnable(pwm_base,pwm_half);
    HWREG(boundary_timer_base + TIMER_O_CTL) |= TIMER_CTL_TAEN;
    HWREG(TIMER0_BASE + TIMER_O_SYNC) = sync_mask | boundary_sync_mask;
    g_seq_dbg_profile_pwm_sync_ticks = SEQ_DWT_CYCCNT;
    HWREG(boundary_timer_base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
    TimerIntEnable(boundary_timer_base,TIMER_TIMA_MATCH);

    g_seq_profile_pwm_active = true;
    g_seq_profile_pwm_timer_base = pwm_base;
    g_seq_profile_pwm_timer_half = pwm_half;
    g_seq_profile_pwm_pin = pin;
    g_seq_dbg_profile_pwm_start_count++;
    if (sync_ticks != 0) *sync_ticks = g_seq_dbg_profile_pwm_sync_ticks;
    return true;
}

static void Seq_StopProfilePwm(void)
{
    unsigned long base;

    if (!g_seq_profile_pwm_active) return;

    base = g_seq_profile_pwm_timer_base;
    TimerDisable(base,g_seq_profile_pwm_timer_half);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE,g_seq_profile_pwm_pin);
    GPIOPinWrite(GPIO_PORTM_BASE,g_seq_profile_pwm_pin,0U);

    g_seq_profile_pwm_active = false;
    g_seq_profile_pwm_timer_base = 0UL;
    g_seq_profile_pwm_timer_half = TIMER_A;
    g_seq_profile_pwm_pin = 0U;
    g_seq_profile_pwm_restore_pending_base = base;
    g_seq_dbg_profile_pwm_stop_count++;

    /* Do not call initTimer2()/initTimer3() from the final 2-us boundary ISR.
     * Their full DriverLib reconfiguration is deferred to the main loop, after
     * the timing-critical staircase has ended. */
}

static void Seq_ServiceProfilePwmRestore(void)
{
    unsigned long base = g_seq_profile_pwm_restore_pending_base;

    if (base == 0UL || g_seq_profile_pwm_active) return;
    g_seq_profile_pwm_restore_pending_base = 0UL;

    if (base == TIMER2_BASE) initTimer2();
    else if (base == TIMER3_BASE) initTimer3();
}


/************************************************************/
/* Prototype - write_immediate                              */
/*                                                          */
/*  Description                                             */
/*  Reads incoming frequency and send to DDS immediately    */
/************************************************************/
void write_immediate(void)  //AD9959
{
    unsigned long freq_start_hz;     // 4 octets pour la Fr quence de D PART (FTW)
    unsigned long freq_end_hz;       // 4 octets pour la Fr quence d'ARRIV E (FTW1/Ramp Limit)
    unsigned long freq_step_hz;      // 4 octets pour la TAILLE du Pas (RDW)
    unsigned long ampl_10bits;
    unsigned long POW_14bits;
    unsigned long FTW_UART;
    unsigned int amplitude_UART;
    unsigned int ChannelDDS_UART;
    unsigned int phase_offset_word_UART;
    double time_step_s;
    unsigned long time_step_us;
    //double time_step_s = 0.001;  //(soit 1 milliseconde)

    ampl_10bits = ((unsigned long)(tempDataUART[1]) << 8) | ((unsigned long)(tempDataUART[0]));
    freq_start_hz = ((unsigned long)(tempDataUART[5]) << 24) | ((unsigned long)(tempDataUART[4]) << 16) | ((unsigned long)(tempDataUART[3]) << 8) | ((unsigned long)(tempDataUART[2])); //Fr quence de D PART (FTW)
    NbDDS_UART = (unsigned long)(tempDataUART[6]);
    ChannelDDS_UART = (unsigned long)(tempDataUART[7]);
    POW_14bits = ((unsigned long)(tempDataUART[9]) << 8) | ((unsigned long)(tempDataUART[8]));
    freq_end_hz = ((unsigned long)(tempDataUART[13]) << 24) | ((unsigned long)(tempDataUART[12]) << 16) | ((unsigned long)(tempDataUART[11]) << 8) | ((unsigned long)(tempDataUART[10]));  //Fr quence d'ARRIV E
    freq_step_hz = ((unsigned long)(tempDataUART[17]) << 24) | ((unsigned long)(tempDataUART[16]) << 16) | ((unsigned long)(tempDataUART[15]) << 8) | ((unsigned long)(tempDataUART[14])); //Taille du Pas (RDW)
    time_step_us = ((unsigned long)(tempDataUART[21]) << 24) | ((unsigned long)(tempDataUART[20]) << 16) | ((unsigned long)(tempDataUART[19]) << 8) | ((unsigned long)(tempDataUART[18]));

    // Conversion en secondes(s)
    time_step_s = (double)time_step_us / 1000000.0;

    // --- Conversion en Mots de R glage (Tuning Words) ---
    //  (freq_*_hz portent maintenant des déci-Hz)
    unsigned long ftw_end_UART = FreqDeciToFTW(freq_end_hz);
    unsigned long ftw_step_UART = FreqDeciToFTW(freq_step_hz);

    FTW_UART = FreqDeciToFTW(freq_start_hz);//  APRÈS
    g_ftw_step = FreqDeciToFTW(freq_step_hz);
    g_ftw_end = FreqDeciToFTW(freq_end_hz);
    g_is_rising = (FreqDeciToFTW(freq_end_hz) > FreqDeciToFTW(freq_start_hz));

    g_ftw_start_config = FreqDeciToFTW(freq_start_hz);;
    amplitude_UART  = (unsigned int)ampl_10bits;
    phase_offset_word_UART = AngleToPOW((unsigned int)POW_14bits);

    Sequence_AbortAndReleaseControl();
    choiceNbDDS((unsigned int)NbDDS_UART, false);

    switch (NbDDS_UART)
    {
    case 1:
    choiceChannelDDS1((unsigned int)ChannelDDS_UART, false);  // S lection du Canal
    SetFrequencyTuningWord0_DDS1(        (unsigned long)FTW_UART,                 (unsigned int)ChannelDDS_UART + 1);  // Cible 0x04 Fr quence de D PART (FTW)
    //SetFrequencyTuningWord0_DDS1_burst((unsigned long)FTW_UART,                 (unsigned int)ChannelDDS_UART); // Cible 0x04 Fr quence de D PART (FTW)
    SetamplValueAD9959DDS1(              (unsigned short)amplitude_UART,          (unsigned int)ChannelDDS_UART + 1);  // Cible 0x06 amplitude control
    SetPOW_ValueAD9959DDS1(              (unsigned short)phase_offset_word_UART,  (unsigned int)ChannelDDS_UART + 1);   // Cible 0x05 phase control
    simpleflashMemoryAD9959DDS1(         (unsigned int)ChannelDDS_UART);
    break;

    case 2:
    choiceChannelDDS2((unsigned int)ChannelDDS_UART, false);
    SetFrequencyTuningWord0_DDS2(        (unsigned long)FTW_UART,                 (unsigned int)ChannelDDS_UART + 1);
    //SetFrequencyTuningWord0_DDS2_burst((unsigned long)FTW_UART,                 (unsigned int)ChannelDDS_UART); // Cible 0x04 Frquence de DPART (FTW)
    SetamplValueAD9959DDS2(              (unsigned short)amplitude_UART,          (unsigned int)ChannelDDS_UART + 1);
    SetPOW_ValueAD9959DDS2(              (unsigned short)phase_offset_word_UART,  (unsigned int)ChannelDDS_UART + 1);
    simpleflashMemoryAD9959DDS2(         (unsigned int)ChannelDDS_UART);
    break;

    case 3:
    choiceChannelDDS3((unsigned int)ChannelDDS_UART, false);
    SetFrequencyTuningWord0_DDS3(        (unsigned long)FTW_UART,                 (unsigned int)ChannelDDS_UART + 1);
    //SetFrequencyTuningWord0_DDS3_burst((unsigned long)FTW_UART,                 (unsigned int)ChannelDDS_UART); // Cible 0x04 Frquence de DPART (FTW)
    SetamplValueAD9959DDS3(              (unsigned short)amplitude_UART,          (unsigned int)ChannelDDS_UART + 1);
    SetPOW_ValueAD9959DDS3(              (unsigned short)phase_offset_word_UART,  (unsigned int)ChannelDDS_UART + 1);
    simpleflashMemoryAD9959DDS3(         (unsigned int)ChannelDDS_UART);
    break;

    default:
    // Gestion d'erreur si NbDDS_UART n'est pas 1, 2 ou 3
    break;
    }
    // Arr ter le balayage en cours avant de reconfigurer
                   if (g_sweep_running) {
                        // Remplacer TIMER0_BASE, TIMER_B par TIMER2_BASE, TIMER_A
                        TimerDisable(TIMER2_BASE, TIMER_A);
                        g_sweep_running = false;
                    }

    if ((freq_step_hz > 0UL) && (time_step_us > 0UL) && (freq_start_hz != freq_end_hz))
    {
        Apply_Sweep_Configuration_uart(NbDDS_UART,ChannelDDS_UART,freq_start_hz,freq_end_hz,freq_step_hz,time_step_s);
    }
}
//*****************************************************************************
//
//  DDS functions
//
//*****************************************************************************

// reset all DDS, addressing the Master Reset pin of all DDs
bool MasterResetAllDDS()
{
    //make master reset DDS1
    PK5 = 0x20; // reset DDS1
    SysCtlDelay(10);
    PK5 = 0x00;
    // make master reset DDS2
    PN4 = 0x10; // reset DDS2
    SysCtlDelay(10);
    PN4 = 0x00;
    //make master reset DDS3
    PN1 = 0x02; // reset DDS3
    SysCtlDelay(10);
    PN1 = 0x00;
    return true;
}
bool start_DDSSequence()
{

    trackRegisterValuesFromDDSOnReboot();
    SysCtlDelay(100);

    unsigned int DDSnb;
    unsigned int ChannelDDS_UART;

    for (DDSnb = 1; DDSnb < 4; ++DDSnb)
    {
        if (TYPEDDS[DDSnb-1] == 1)
        {
            controlRegisterValueAD9959DDS[DDSnb-1] = (unsigned long) 0xFFFFFFFF;
            choiceNbDDS(DDSnb, true);
            choiceChannelDDS(ChannelDDS_UART, true);

            if (DDSnb == 1)
            {
                for (ChannelDDS_UART = 0; ChannelDDS_UART < 4; ++ChannelDDS_UART)
                {
                 choiceChannelDDS1(ChannelDDS_UART, true);
                 SysCtlDelay(10);
                 simpleLoadValuesFromFlashMemoryAD9959DDS1(ChannelDDS_UART);
                }
            }
            if (DDSnb == 2)
            {
                for (ChannelDDS_UART = 0; ChannelDDS_UART < 4; ++ChannelDDS_UART)
                {
                choiceChannelDDS2(ChannelDDS_UART, true);
                SysCtlDelay(10);
                simpleLoadValuesFromFlashMemoryAD9959DDS2(ChannelDDS_UART);
                }
            }
            if (DDSnb == 3)
            {
                for (ChannelDDS_UART = 0; ChannelDDS_UART < 4; ++ChannelDDS_UART)
                {
                choiceChannelDDS3(ChannelDDS_UART, true);
                SysCtlDelay(10);
                simpleLoadValuesFromFlashMemoryAD9959DDS3(ChannelDDS_UART);
                }

            }
            SysCtlDelay(10000);
            PB0 = 0x01;   // lock on/off
            SysCtlDelay(10000);
            PB0 = 0x00;

        }
        else
        {
            return false;
        }
    }
    return true;
}

bool choiceNbDDS(unsigned int NbDDS, bool firstInit)
{
    if ((NbDDS == 1)|(NbDDS == 2)|(NbDDS == 3))
    {
        CURRENT_DDS = NbDDS;
        SysCtlDelay(1); // Need this wait to switch from one DDS to the next, if not the information is not properly passed. 100 seems to work.
    }

    switch(NbDDS)
    {
    case 1:
        PA3 = 0x00;   // CS DDS1
        PB4 = 0x10;   // CS DDS2
        PD2 = 0x04;   // CS DDS3
        return true;
    case 2:
        PB4 = 0x00;
        PA3 = 0x08;
        PD2 = 0x04;
        return true;
    case 3:
        PD2 = 0x00;
        PA3 = 0x08;
        PB4 = 0x10;
        return true;

    default:
        return false;
    }
}
bool choiceChannelDDS(unsigned int ChannelDDS_UART, bool firstInit)
{
  if( CURRENT_DDS == 1)
     {
       if ((ChannelDDS_UART == 0)|(ChannelDDS_UART == 1)|(ChannelDDS_UART == 2)|(ChannelDDS_UART == 3))
        {
        CURRENT_ChannelDDS = ChannelDDS_UART;
        SysCtlDelay(10); // Need this wait to switch from one DDS to the next, if not the information is not properly passed. 100 seems to work.
        return true;
        }

       else
       {
        return false;
       }
     }
   else if( CURRENT_DDS == 2)
     {
       if ((ChannelDDS_UART == 0)|(ChannelDDS_UART == 1)|(ChannelDDS_UART == 2)|(ChannelDDS_UART == 3))
       {
        CURRENT_ChannelDDS = ChannelDDS_UART;
        SysCtlDelay(10); // Need this wait to switch from one DDS to the next, if not the information is not properly passed. 100 seems to work.
        return true;
       }
       else
       {
        return false;
       }
     }
   else if( CURRENT_DDS == 3)
        {
          if ((ChannelDDS_UART == 0)|(ChannelDDS_UART == 1)|(ChannelDDS_UART == 2)|(ChannelDDS_UART == 3))
          {
           CURRENT_ChannelDDS = ChannelDDS_UART;
           SysCtlDelay(10); // Need this wait to switch from one DDS to the next, if not the information is not properly passed. 100 seems to work.
           return true;
          }
          else
          {
           return false;
          }
        }
         else
          {
          return false;
          }
}
bool choiceChannelDDS1(unsigned int ChannelDDS_UART, bool firstInit)
{

    if ((ChannelDDS_UART == 0)|(ChannelDDS_UART == 1)|(ChannelDDS_UART == 2)|(ChannelDDS_UART == 3))
    {
        CURRENT_ChannelDDS = ChannelDDS_UART;
        SysCtlDelay(1); // Need this wait to switch from one DDS to the next, if not the information is not properly passed. 100 seems to work.
    }
    switch(ChannelDDS_UART)
    {
    case 0:
        ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS1,CSR_NUM_BYTE);  //select CH0
        return true;
    case 1:
        ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS1,CSR_NUM_BYTE);  //select CH1
        return true;
    case 2:
        ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS1,CSR_NUM_BYTE);  //select CH2
        return true;
    case 3:
        ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS1,CSR_NUM_BYTE);  //select CH3
        return true;
    default:
        return false;
    }
}
bool choiceChannelDDS2(unsigned int ChannelDDS_UART, bool firstInit)
{

    if ((ChannelDDS_UART == 0)|(ChannelDDS_UART == 1)|(ChannelDDS_UART == 2)|(ChannelDDS_UART == 3))
    {
        CURRENT_ChannelDDS = ChannelDDS_UART;
        SysCtlDelay(1); // Need this wait to switch from one DDS to the next, if not the information is not properly passed. 100 seems to work.
    }
    switch(ChannelDDS_UART)
    {
    case 0:
        ssi1PutData(CSR_ADRESS,CSR0_4bit_DDS2,CSR_NUM_BYTE);  //select CH0
        return true;
    case 1:
        ssi1PutData(CSR_ADRESS,CSR1_4bit_DDS2,CSR_NUM_BYTE);  //select CH1
        return true;
    case 2:
        ssi1PutData(CSR_ADRESS,CSR2_4bit_DDS2,CSR_NUM_BYTE);  //select CH2
        return true;
    case 3:
        ssi1PutData(CSR_ADRESS,CSR3_4bit_DDS2,CSR_NUM_BYTE);  //select CH3
        return true;
    default:
        return false;
    }
}
bool choiceChannelDDS3(unsigned int ChannelDDS_UART, bool firstInit)
{

    if ((ChannelDDS_UART == 0)|(ChannelDDS_UART == 1)|(ChannelDDS_UART == 2)|(ChannelDDS_UART == 3))
    {
        CURRENT_ChannelDDS = ChannelDDS_UART;
        SysCtlDelay(1); // Need this wait to switch from one DDS to the next, if not the information is not properly passed. 100 seems to work.
    }
    switch(ChannelDDS_UART)
    {
    case 0:
        ssi2PutData(CSR_ADRESS,CSR0_4bit_DDS3,CSR_NUM_BYTE);  //select CH0
        return true;
    case 1:
        ssi2PutData(CSR_ADRESS,CSR1_4bit_DDS3,CSR_NUM_BYTE);  //select CH1
        return true;
    case 2:
        ssi2PutData(CSR_ADRESS,CSR2_4bit_DDS3,CSR_NUM_BYTE);  //select CH2
        return true;
    case 3:
        ssi2PutData(CSR_ADRESS,CSR3_4bit_DDS3,CSR_NUM_BYTE);  //select CH3
        return true;
    default:
        return false;
    }
}

static inline void Seq_ArmTimer4_ticks_compensated(unsigned long logical_ticks,unsigned long overhead_ticks)
{
    unsigned long hardware_ticks;
    uint32_t now_ticks = SEQ_DWT_CYCCNT;

    g_seq_timer_hw_owner_bit = 0U;

    /* SEQ_TIMER_MIN_TICKS est le minimum du PAS LOGIQUE (3 µs).
     * Il ne faut pas le réutiliser comme plancher du reload après
     * soustraction de l'overhead, sinon tout pas de 3,7 µs dérive
     * dès que l'ISR coûte plus de 0,7 µs. */
    if (logical_ticks > (overhead_ticks + SEQ_TIMER_RELOAD_MIN_TICKS)) {
        hardware_ticks = logical_ticks - overhead_ticks;
    } else {
        hardware_ticks = SEQ_TIMER_RELOAD_MIN_TICKS;
        g_seq_dbg_deadline_miss++;
    }

    g_seq_timer_armed_ticks = logical_ticks;
    g_seq_timer4_remaining_origin_ticks = now_ticks - overhead_ticks;
    g_seq_timer4_arm_start_ticks = now_ticks;
    g_seq_timer4_active = true;

    HWREG(TIMER4_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(TIMER4_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;
    HWREG(TIMER4_BASE + TIMER_O_TAILR) = hardware_ticks - 1UL;
    HWREG(TIMER4_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
}

static inline void Seq_ArmTimer4ForUpdateDeadline(uint32_t update_deadline_ticks,unsigned long predicted_preupdate_ticks,unsigned long logical_ticks)
{
    uint32_t now_ticks = SEQ_DWT_CYCCNT;
    uint32_t desired_isr_ticks = update_deadline_ticks - predicted_preupdate_ticks;
    int32_t remaining_ticks = (int32_t)(desired_isr_ticks - now_ticks);
    unsigned long hardware_ticks;

    if (remaining_ticks > (int32_t)SEQ_TIMER_RELOAD_MIN_TICKS) {
        hardware_ticks = (unsigned long)remaining_ticks;
    } else {
        hardware_ticks = SEQ_TIMER_RELOAD_MIN_TICKS;
        g_seq_dbg_hw_fast_miss++;
        g_seq_dbg_deadline_miss++;
    }

    g_seq_timer_armed_ticks = logical_ticks;
    g_seq_timer4_remaining_origin_ticks = update_deadline_ticks - (uint32_t)logical_ticks;
    g_seq_timer4_arm_start_ticks = now_ticks;
    g_seq_timer4_active = true;

    HWREG(TIMER4_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(TIMER4_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;
    HWREG(TIMER4_BASE + TIMER_O_TAILR) = hardware_ticks - 1UL;
    HWREG(TIMER4_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
}

void trackRegisterValuesFromDDSOnReboot()
{
    int DDSnb;
    for (DDSnb = 1; DDSnb <4; ++DDSnb)
    {
        if (TYPEDDS[DDSnb-1] == 1)
        {
           trackRegisterValuesFromSingleAD9959DDSOnReboot(DDSnb);
        }
    }
}
void trackRegisterValuesFromSingleAD9959DDSOnReboot(unsigned DDSnb)
{
    frequencyValueAD9959DDS[DDSnb-1] = frequencyValueAD9959DDSOnReboot;
    amplValueAD9959DDS[DDSnb-1] = amplValueAD9959DDSOnReboot;
    powValueAD9959DDS[DDSnb-1] = powValueAD9959DDSOnReboot;
}
static inline bool SetFrequencyTuningWord_ethernet_burst(unsigned long frequencyValue, unsigned int ChannelDDS_UART)
{
    if (CURRENT_ChannelDDS > 3) return false;

    //frequencyValueAD9959DDS[ChannelDDS_UART] = frequencyValue;

    // S lection du DDS et Envoi Rapide
    switch (CURRENT_DDS)
    {
        case 1:
            // Envoi SPI (DDS 1)
            //ssi0PutData_Burst(FTW_ADRESS, frequencyValue);
            // Envoi SPI ULTRA Rapide
            ssi0PutData_QSPI_Fast(FTW_ADRESS, frequencyValue);

            // Mise   jour tableau
            frequencyValueAD9959DDS[CURRENT_ChannelDDS] = frequencyValue;
            // Pulse I/O UPDATE (PK7)
            // Utilisation de l' criture bit-band e pour l'atomicit  et la vitesse

            // On doit attendre que le SSI ait fini de vider le FIFO sur le c ble.
            // Sans  a, l'IO_UPDATE part trop t t.
            while(SSI0_SR_R & SSI_SR_BSY);

            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;
            __asm(" NOP"); // D lai minime
            HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00;
            return true;

        case 2:
            // Envoi SPI (DDS 2)
            //ssi1PutData_Burst(FTW_ADRESS, frequencyValue);
            // Envoi SPI ULTRA Rapide
            ssi1PutData_QSPI_Fast(FTW_ADRESS, frequencyValue);
            // Mise   jour tableau
            frequencyValueAD9959DDS[CURRENT_ChannelDDS] = frequencyValue;

            // On doit attendre que le SSI ait fini de vider le FIFO sur le c ble.
            // Sans  a, l'IO_UPDATE part trop t t.
            while(SSI1_SR_R & SSI_SR_BSY);

            // Pulse I/O UPDATE (PN3)
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;
            __asm(" NOP");
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00;
            return true;

        case 3:
            // Envoi SPI (DDS 3)
            //ssi2PutData_Burst(FTW_ADRESS, frequencyValue);
            // Envoi SPI ULTRA Rapide
            ssi2PutData_QSPI_Fast(FTW_ADRESS, frequencyValue);
            // Mise   jour tableau
            frequencyValueAD9959DDS[CURRENT_ChannelDDS] = frequencyValue;

            // On doit attendre que le SSI ait fini de vider le FIFO sur le c ble.
            // Sans  a, l'IO_UPDATE part trop t t.
            while(SSI2_SR_R & SSI_SR_BSY);

            // Pulse I/O UPDATE (PN0)
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01;
            __asm(" NOP");
            HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00;
            return true;

        default:
            return false;
    }
}
//*****************************************************************************
//
//  Timer functions for ethernet parameters sweep
//
//*****************************************************************************
void Timer1A_IntHandler(void)
{
    // Clear the timer interrupt
    //TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Plus rapide que TimerIntClear()
    HWREG(TIMER1_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;

    // Indiquer qu'une interruption de timer a eu lieu.
    HWREGBITW(&g_ulTimerFlags, FLAG_TIMER_TICK) = 1;

    if (!g_sweep_running2) {
        // Remplace TimerDisable() qui est lent
        HWREG(TIMER1_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN);
        //TimerDisable(TIMER1_BASE, TIMER_A); // D sactive Timer A de Timer 1
        return;
    }

    // 1. Calculer le prochain FTW
    bool sweep_finished = false;

    if (g_is_rising) {
        if (g_current_ftw + g_ftw_step >= g_ftw_end) {
            g_current_ftw = g_ftw_end;
            sweep_finished = true;
        } else {
            g_current_ftw += g_ftw_step;
        }
    } else { // Balayage descendant

        if (g_current_ftw <= g_ftw_end || g_current_ftw - g_ftw_step <= g_ftw_end) {
            g_current_ftw = g_ftw_end;
            sweep_finished = true;
        } else {
            g_current_ftw -= g_ftw_step;
        }
    }

    SetFrequencyTuningWord_ethernet_burst(g_current_ftw, g_channel);

    //V rifier la fin du balayage
    if (sweep_finished) {
        g_sweep_running2 = false;
        // Arr t imm diat du Timer (Reset bit TAEN)
        HWREG(TIMER1_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN);
        //TimerDisable(TIMER1_BASE, TIMER_A); // Arr te Timer A de Timer 1
    }
}

//**************************************//
//Timer ISR for Frequency Lookup Table
//**************************************//
/*void Timer2A_IntHandler(void)
{
    // Clear interrupt & Flag
    //TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    // Plus rapide que TimerIntClear()
    HWREG(TIMER2_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;

    // Indiquer qu'une interruption de timer a eu lieu.
    HWREGBITW(&g_ulTimerFlags, FLAG_TIMER_TICK) = 1;

    if (!g_sweep_running) {
        // D sactivation directe du bit TAEN (Timer A Enable)
        HWREG(TIMER2_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN);
        //TimerDisable(TIMER2_BASE, TIMER_A); // D sactive Timer A de Timer 2
        return;
    }

    // On prend la valeur actuelle
    // Lecture Tableau (Lookup)
    uint32_t next_ftw = g_FrequencyTable[g_TableIndex];

    // Incr mentation de l'index
    g_TableIndex++;

    // Gestion de la boucle (Reset   0 si fin de tableau)
    if (g_TableIndex >= g_TableSize)
    {
     g_TableIndex = 0;
     g_sweep_running = false;
    }
    // Optionnel : g_sweep_running = false; // si vous voulez arr ter apr s 1 tour

    switch(NbDDS_UART)
        {
            case 1:
                SetFrequencyTuningWord0_DDS1_burst(next_ftw, g_channel);
                break;
            case 2:
                SetFrequencyTuningWord0_DDS2_burst(next_ftw, g_channel);
                break;
            case 3:
                SetFrequencyTuningWord0_DDS3_burst(next_ftw, g_channel);
                break;
        }

    // Gestion de fin(Acc s direct registre)
    //if (sweep_finished) {
      //  g_sweep_running = false;
        // D sactivation directe
        //HWREG(TIMER2_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN);
      //  TimerDisable(TIMER2_BASE, TIMER_A); // Arr te Timer A de Timer 2
   // }
}
*/

//================== AJOUT =========================
// ---- Helpers séquence ----
static inline void Seq_SetSingleToneFTW_Optimized(unsigned int d,unsigned int c, unsigned long ftw,seq_rt_t *rt){
    /*
     * Si nous sortons d'un linear sweep,
     * garder PROFILE HIGH jusqu'au prochain I/O_UPDATE.
     */
    if (!rt->profile_low_after_update) {
        Seq_SetProfileLow(d,c);
    }

    Seq_WriteRegFast(d,CSR_ADRESS,Seq_CSR_FromChannel(c),CSR_NUM_BYTE);

    if (!rt->single_tone_ready) {
        Seq_WriteRegFast(d,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);
        rt->single_tone_ready = true;
    }
    Seq_WriteRegFast(d,FTW_ADRESS,ftw,FTW_NUM_BYTE);
}

static inline int Seq_FirstSetBit12(unsigned int mask)
{
    unsigned int i;

    for (i = 0U; i < 12U; i++) {
        if (mask & (1U << i)) return (int)i;
    }

    return -1;
}

// Arme Timer4 pour une attente unique de "time_s" secondes
static bool Seq_FindNextDelayTicks(unsigned long *next_ticks)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;
    bool found = false;
    unsigned long best = 0xFFFFFFFFUL;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;
        unsigned int d;
        unsigned int c;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;

        if (Seq_DdsBlockedByHw(d) || g_seq_rt[d][c].phase == SEQ_RT_WAIT_TRIGGER) {
            mask &= ~bit;
            continue;
        }

        if (g_seq_rt[d][c].remaining_ticks < best) {
            best = g_seq_rt[d][c].remaining_ticks;
            found = true;
        }

        mask &= ~bit;
    }

    if (found) *next_ticks = best;
    return found;
}


static unsigned int Seq_CountChannelsDueAtDelay(unsigned long delay_ticks)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;
    unsigned int count = 0U;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;
        unsigned int d;
        unsigned int c;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;
        mask &= ~bit;

        if (Seq_DdsBlockedByHw(d)) continue;
        if (!g_seq_rt[d][c].active || g_seq_rt[d][c].phase == SEQ_RT_WAIT_TRIGGER) continue;
        if (g_seq_rt[d][c].remaining_ticks == delay_ticks) count++;
    }

    return count;
}

static unsigned int Seq_CountUnpreparedChannelsDueAtDelay(unsigned long delay_ticks)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;
    unsigned int count = 0U;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;
        unsigned int d;
        unsigned int c;
        seq_rt_t *rt;
        seq_def_t *def;
        unsigned int next_loop;
        unsigned int next_item;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;
        mask &= ~bit;

        if (Seq_DdsBlockedByHw(d)) continue;

        rt = (seq_rt_t *)&g_seq_rt[d][c];
        def = (seq_def_t *)&g_seq_def[d][c];

        if (!rt->active || rt->remaining_ticks != delay_ticks) continue;
        if (rt->phase == SEQ_RT_WAIT_TRIGGER) continue;
        if (rt->phase == SEQ_RT_SWEEP_PRELOAD_DESC) continue;
        if (rt->next_item_preloaded) continue;

        /*
         * La fin d'un STEP terminal ne demande aucune écriture DDS.
         * Une fin de linear sweep terminal doit en revanche désactiver le
         * sweep et forcer sa fréquence finale : elle reste "non préparée".
         */
        if (!Seq_GetFollowingPosition(rt,def,&next_loop,&next_item)) {
            if (rt->phase != SEQ_RT_SWEEP) continue;
        }

        count++;
    }

    return count;
}

static unsigned long Seq_PredictTimer4PreupdateTicks(unsigned long delay_ticks)
{
    unsigned int due_count = Seq_CountChannelsDueAtDelay(delay_ticks);
    unsigned int unprepared_count = Seq_CountUnpreparedChannelsDueAtDelay(delay_ticks);
    unsigned int bucket = due_count;
    unsigned long estimate;

    if (bucket == 0U) return 0UL;
    if (bucket >= SEQ_PREUPDATE_BUCKETS) bucket = SEQ_PREUPDATE_BUCKETS - 1U;

    estimate = g_seq_preupdate_estimate_ticks[bucket];
    estimate += (unsigned long)unprepared_count * SEQ_TIMER4_UNPREPARED_EXTRA_TICKS;

    return estimate;
}

static void Seq_UpdateTimer4PreupdateEstimate(
    unsigned int due_count,
    unsigned int unprepared_count,
    unsigned long sample_ticks)
{
    unsigned int bucket = due_count;
    unsigned long old_estimate;

    /*
     * Les frontières non préparées ont un coût très dépendant du type d'item.
     * Elles ne doivent pas polluer l'estimateur du chemin préchargé.
     */
    if (bucket == 0U || unprepared_count != 0U) return;
    if (bucket >= SEQ_PREUPDATE_BUCKETS) bucket = SEQ_PREUPDATE_BUCKETS - 1U;

    g_seq_dbg_preupdate_sample_ticks[bucket] = sample_ticks;
    old_estimate = g_seq_preupdate_estimate_ticks[bucket];
    g_seq_preupdate_estimate_ticks[bucket] =
        (3UL * old_estimate + sample_ticks + 2UL) / 4UL;
}

static inline bool Seq_SweepIsRising(const seq_sweep_t *s)
{
    return s->rising != 0U;
}


static bool Seq_ShouldStartChannel(seq_rt_t *rt, unsigned int incoming_rank)
{
    if (!rt->active) return true;

    if (g_seq_retrigger_policy == SEQ_RETRIGGER_IGNORE_ACTIVE) return false;
    if (g_seq_retrigger_policy == SEQ_RETRIGGER_ALWAYS) return true;

    if (g_seq_retrigger_policy == SEQ_RETRIGGER_HIGHER_PRIORITY) {
        return (incoming_rank < rt->priority_rank);
    }

    return false;
}

static void Seq_StartChannelsFromMask(unsigned int arm_mask,unsigned int ttl){
    unsigned int d;
    unsigned int c;

    unsigned int dds_update_mask = 0U;
    unsigned int started_mask = 0U;

    unsigned int rank =
        Seq_PriorityRank(ttl);

    unsigned long next_ticks;
    uint32_t first_item_start_ticks = 0UL;

    bool started_any = false;
    uint32_t first_physical_update_ticks;

    if ((g_seq_active_mask & 0x0FFFU) == 0U) {
        g_seq_dbg_deadline_miss = 0UL;
        g_seq_dbg_hw_fast_ticks = 0UL;
        g_seq_dbg_hw_fast_max_ticks = 0UL;
        g_seq_dbg_hw_fast_miss = 0UL;
        g_seq_dbg_hw_entry_arm_delay_ticks = 0UL;
        g_seq_dbg_hw_multi_fast_ticks = 0UL;
        g_seq_dbg_hw_multi_fast_max_ticks = 0UL;
        g_seq_dbg_hw_multi_total_ticks = 0UL;
        g_seq_dbg_hw_multi_total_max_ticks = 0UL;
        g_seq_dbg_hw_multi_deadline_miss = 0UL;
        g_seq_dbg_hw_deadline_resync = 0UL;
        g_seq_hw_multi_prep_estimate_ticks = 440UL;
        g_seq_timer_hw_owner_bit = 0U;
        g_seq_dbg_hw_engine_conflict = 0UL;
        g_seq_dbg_boundary_spin_ticks = 0UL;
        g_seq_dbg_boundary_late_ticks = 0UL;
        g_seq_dbg_boundary_late_max_ticks = 0UL;
        g_seq_dbg_fast_isr_ticks = 0UL;
        g_seq_dbg_fast_isr_max_ticks = 0UL;
        g_seq_dbg_fast_final_handoff_ticks = 0UL;
        g_seq_dbg_fast_late_max_ticks = 0UL;
        g_seq_dbg_phase_low_late_max_ticks = 0UL;
        g_seq_dbg_fast_gate_release_count = 0UL;
        g_seq_dbg_fast_points_committed = 0UL;
        g_seq_dbg_fast_fifo_queue_max_ticks = 0UL;
        g_seq_dbg_special_step_release_count = 0UL;
        g_seq_dbg_special_linear_release_count = 0UL;
        g_seq_dbg_profile_pwm_start_count = 0UL;
        g_seq_dbg_profile_pwm_stop_count = 0UL;
        g_seq_dbg_profile_pwm_sync_ticks = 0UL;
        g_seq_timer4_remaining_origin_ticks = 0UL;
        for (d = 0U; d < DDS_COUNT; d++) {
            g_seq_dbg_hw_engine_isr_ticks[d] = 0UL;
            g_seq_dbg_hw_engine_isr_max_ticks[d] = 0UL;
            g_seq_dbg_hw_engine_late_ticks[d] = 0UL;
            g_seq_dbg_hw_engine_late_max_ticks[d] = 0UL;
            g_seq_dbg_hw_engine_miss[d] = 0UL;
            g_seq_dbg_hw_companion_preload_ticks[d] = 0UL;
            g_seq_dbg_hw_companion_preload_max_ticks[d] = 0UL;
            g_seq_dbg_hw_companion_late[d] = 0UL;
        }
    }

    /*
     * ========================================================
     * 1. Initialisation runtime des canaux réellement démarrés
     * ========================================================
     */

    {
    unsigned int candidates = arm_mask & 0x0FFFU;

    while (candidates != 0U) {
        int idx = Seq_FirstSetBit12(candidates);
        unsigned int bit;
        seq_rt_t *rt;
        seq_def_t *def;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;

        candidates &= ~bit;

        rt = (seq_rt_t *)&g_seq_rt[d][c];
        def = (seq_def_t *)&g_seq_def[d][c];

        if (!def->valid) continue;
        if (!Seq_ShouldStartChannel(rt,rank)) continue;

        /* Pendant un HW sweep, un autre canal du même DDS ne peut pas
         * être redémarré sans risquer de mélanger les transactions SSI. */
        if (g_seq_hw_engine[d].active && g_seq_hw_engine[d].owner_ch != c) continue;

        if (g_seq_hw_engine[d].active && g_seq_hw_engine[d].owner_ch == c) {
            unsigned long hw_timer_base = Seq_HwTimerBase(d);
            HWREG(hw_timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
            HWREG(hw_timer_base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;
            if (g_seq_hw_engine[d].profile_pwm) Seq_StopProfilePwm();
            g_seq_hw_engine[d].profile_pwm = false;
            g_seq_hw_engine[d].active = false;
            g_seq_hw_resume_pending_mask &= ~(0x0FU << (4U * d));
            g_seq_hw_resume_kind[d] = SEQ_HW_RESUME_NONE;
        }

        rt->single_tone_ready = false;

        rt->active = true;
        g_seq_active_mask |= bit;

        rt->dds_idx = d;
        rt->ch_idx = c;

        rt->loop_idx = 0U;
        rt->item_idx = 0U;

        rt->active_ttl = ttl;
        rt->priority_rank = rank;

        rt->phase = SEQ_RT_IDLE;
        rt->sweep_start_pending = false;
        rt->profile_low_after_update = false;

        rt->remaining_ticks = 1UL;
        rt->hw_current_decihz = 0UL;
        rt->hw_current_ftw = 0UL;
        rt->hw_next_update_deadline_ticks = 0UL;
        rt->hw_deadline_valid = false;
        rt->fast_wait_prepared = false;
        rt->fast_wait_start_ticks = 0UL;

        rt->next_item_preloaded = false;
        rt->preloaded_loop_idx = 0U;
        rt->preloaded_item_idx = 0U;

        rt->sequence_clock_started = false;
        rt->loop_deadline_active = false;
        rt->sequence_start_ticks = 0UL;
        rt->loop_duration_ticks = Seq_GetLoopNominalDurationTicks(def);
        g_seq_dbg_loop_duration_ticks = rt->loop_duration_ticks;

        started_mask |= bit;
        started_any = true;
    }
    }

    /* Un trigger ignoré ne doit jamais réarmer ou perturber Timer4 pour les
     * canaux déjà actifs. */
    if (!started_any) return;


    /*
     * ========================================================
     * 2. Préparer le premier item
     * ========================================================
     */

    dds_update_mask = 0U;

    {
    unsigned int candidates = started_mask & 0x0FFFU;

    while (candidates != 0U) {
        int idx = Seq_FirstSetBit12(candidates);
        unsigned int bit;
        seq_rt_t *rt;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;

        candidates &= ~bit;

        rt = (seq_rt_t *)&g_seq_rt[d][c];

        if (rt->first_item_preloaded) {
            g_seq_dbg_preload_hit++;

            if (Seq_InitCurrentItemRuntime_NoWrite(d,c)) {
                dds_update_mask |= (1U << d);
                rt->first_item_preloaded = false;
                continue;
            }

            rt->first_item_preloaded = false;
        }

        g_seq_dbg_preload_miss++;
        Seq_ApplyCurrentItem(d,c,&dds_update_mask);
    }
    }

    /*
     * ========================================================
     * 3. Activation physique
     * ========================================================
     */

    Seq_IOUpdateMask(dds_update_mask);
    first_physical_update_ticks = SEQ_DWT_CYCCNT;
    Seq_PrepareWaitingSpecialAfterUpdate();
    Seq_StartPendingSweepsImmediate();
    Seq_StartHwEnginesAfterUpdate(first_physical_update_ticks);
    g_seq_dbg_trigger_to_update_ticks = (unsigned long)(first_physical_update_ticks - g_seq_dbg_trigger_isr_start);

    Seq_ApplyDeferredProfileLows();

    Seq_StartPendingSweeps();

    /*
    * Le premier item est maintenant physiquement lancé.
    * Cette origine exclut volontairement la latence trigger -> séquence.
    */
    {
        uint32_t sequence_start_ticks = first_physical_update_ticks;
        unsigned int candidates = started_mask & 0x0FFFU;

        first_item_start_ticks = sequence_start_ticks;

        while (candidates != 0U) {
            int idx = Seq_FirstSetBit12(candidates);
            unsigned int bit;
            seq_rt_t *rt;

            if (idx < 0) {
                break;
            }

            bit = 1U << (unsigned int)idx;
            d = (unsigned int)idx / 4U;
            c = (unsigned int)idx % 4U;
            candidates &= ~bit;

            rt = (seq_rt_t *)&g_seq_rt[d][c];
            rt->sequence_start_ticks = sequence_start_ticks;
            rt->sequence_clock_started = true;
        }
    }


    /*
    * ========================================================
    * 4. Premier timer
    * ========================================================
    */

    if (Seq_FindNextDelayTicks(&next_ticks)) {
        unsigned long predicted_preupdate_ticks;

        /* V30.3 : ne préparer que la prochaine frontière GLOBALE, puis armer
         * directement à partir de sa deadline absolue. Le temps déjà consommé
         * depuis le premier I/O_UPDATE est lu dans le helper au dernier moment. */
        Seq_PreloadNextItemsSafe();
        predicted_preupdate_ticks = Seq_PredictTimer4PreupdateTicks(next_ticks);

        g_seq_timer4_deadline_ticks = first_physical_update_ticks + (uint32_t)next_ticks;
        g_seq_timer4_deadline_valid = true;
        g_seq_last_compensation_ticks = predicted_preupdate_ticks;
        g_seq_any_running = true;
        Seq_ArmTimer4ForUpdateDeadline(g_seq_timer4_deadline_ticks,predicted_preupdate_ticks,next_ticks);
    } else if (Seq_AnyHwEngineActive()) {
        g_seq_any_running = true;
        g_seq_timer4_deadline_valid = false;
        g_seq_timer4_remaining_origin_ticks = first_physical_update_ticks;
    }
}

void Seq_ServicePendingTriggers(void)
{
    unsigned int flags;

    IntDisable(INT_GPIOL);
    flags = g_seq_pending_trigger_flags;
    g_seq_pending_trigger_flags = 0U;
    IntEnable(INT_GPIOL);

    if (flags != 0U) {
        Seq_HandleTriggeredMask(flags);
    }
}

void Seq_HandleTriggeredMask(unsigned int trigger_flags)
{
    unsigned int i;

    if ((trigger_flags & (1U << 0)) != 0U) (void)Seq_ReleaseWaitingSpecialItems(SEQ_TRIGGER_TTL6);
    if ((trigger_flags & (1U << 1)) != 0U) (void)Seq_ReleaseWaitingSpecialItems(SEQ_TRIGGER_TTL7);
    if ((trigger_flags & (1U << 2)) != 0U) (void)Seq_ReleaseWaitingSpecialItems(SEQ_TRIGGER_TTL8);

    /*
     * Fast path : un seul TTL actif.
     */
    if (trigger_flags == (1U << 0)) {
        Seq_StartChannelsFromMask(g_seq_arm_mask_by_ttl[0],SEQ_TRIGGER_TTL6);
        return;
    }

    if (trigger_flags == (1U << 1)) {
        Seq_StartChannelsFromMask(g_seq_arm_mask_by_ttl[1],SEQ_TRIGGER_TTL7);
        return;
    }

    if (trigger_flags == (1U << 2)) {
        Seq_StartChannelsFromMask(g_seq_arm_mask_by_ttl[2],SEQ_TRIGGER_TTL8);
        return;
    }

    /*
     * Cas rare : plusieurs TTL simultanés.
     * On conserve alors la logique de priorité.
     */
    for (i = 0U; i < 3U; i++) {
        unsigned int ttl = g_seq_priority_ttl[i];
        int idx = Seq_TtlToIndex(ttl);

        if (idx < 0) continue;

        if (trigger_flags & (1U << (unsigned int)idx)) {
            Seq_StartChannelsFromMask(g_seq_arm_mask_by_ttl[(unsigned int)idx],ttl);
        }
    }
}

static void Seq_ApplyCurrentItem(unsigned int d,unsigned int c,unsigned int *dds_update_mask)
{
    seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];
    seq_def_t *def = (seq_def_t *)&g_seq_def[d][c];
    seq_item_t *item;

    if (!rt->active) return;
    if (rt->item_idx >= def->n_items) return;

    item = (seq_item_t *)&def->items[rt->item_idx];

    Seq_UpdateLoopDeadlineStateForCurrentItem(rt,def,item);

    if (item->wait_for_trigger) {
        rt->phase = SEQ_RT_WAIT_TRIGGER;
        rt->remaining_ticks = 0UL;
        rt->sweep_start_pending = false;
        rt->fast_wait_prepared = false;
        rt->fast_wait_start_ticks = SEQ_DWT_CYCCNT;
        rt->loop_deadline_active = false;

        if (item->type == SEQ_ITEM_STEP) {
            return;
        }

        if (item->type == SEQ_ITEM_SWEEP) {
            if (Seq_SweepIsRising(&item->sweep)) Seq_SetProfileLow(d,c);
            else Seq_SetProfileHigh(d,c);
            Seq_ConfigLinearSweep_NoStart(d,c,&item->sweep);
            rt->single_tone_ready = false;
            *dds_update_mask |= (1U << d);
            return;
        }

        if (item->type == SEQ_ITEM_FAST_HW_SWEEP) {
            seq_fast_plan_t *plan = &g_seq_fast_plan[d];
            if (!plan->valid || plan->owner_ch != c || plan->item_idx != rt->item_idx) {
                rt->active = false;
                rt->phase = SEQ_RT_IDLE;
                g_seq_active_mask &= ~Seq_GlobalBit(d,c);
                g_seq_dbg_hw_engine_miss[d]++;
                return;
            }
            Seq_ConfigFastHwSweep_NoStart(d,c,&item->hw_sweep);
            plan->next_index = 1U;
            rt->hw_current_decihz = item->hw_sweep.start_decihz;
            rt->hw_current_ftw = item->hw_sweep.start_ftw;
            rt->single_tone_ready =
                ((item->hw_sweep.fast_flags & SEQ_FAST_FLAG_PHASE_TRIANGLE) == 0U);
            *dds_update_mask |= (1U << d);
            return;
        }

        rt->active = false;
        rt->phase = SEQ_RT_IDLE;
        g_seq_active_mask &= ~Seq_GlobalBit(d,c);
        return;
    }

    if (item->type == SEQ_ITEM_STEP) {
        Seq_SetSingleToneFTW_Optimized(d,c,item->step.ftw,rt);

        rt->phase = SEQ_RT_STEP;
        rt->remaining_ticks = Seq_UsToTicks(item->step.duration_us);
        rt->sweep_start_pending = false;

        *dds_update_mask |= (1U << d);
        return;
    }

    if (item->type == SEQ_ITEM_STEP_EX) {
        Seq_PrepareStepEx_NoUpdate(d,c,item,rt);

        rt->phase = SEQ_RT_STEP;
        rt->remaining_ticks = Seq_UsToTicks(item->step.duration_us);
        rt->sweep_start_pending = false;

        *dds_update_mask |= (1U << d);
        return;
    }

    if (item->type == SEQ_ITEM_SWEEP) {

        /*
        * Cas normal :
        * STEP -> SWEEP
        * le profil peut être mis LOW avant configuration.
        *
        * Cas particulier :
        * SWEEP -> SWEEP
        * garder HIGH jusqu'au I/O_UPDATE qui termine
        * réellement l'ancien sweep.
        */
        if (!rt->profile_low_after_update) {
            Seq_SetProfileLow(d,c);
        }

        Seq_ConfigLinearSweep_NoStart(
            d,
            c,
            &item->sweep
        );

        rt->single_tone_ready = false;

        if (Seq_SweepIsRising(&item->sweep)) {
            rt->phase = SEQ_RT_SWEEP;
            rt->remaining_ticks = item->sweep.duration_ticks;
        } else {
            rt->phase = SEQ_RT_SWEEP_PRELOAD_DESC;
            rt->remaining_ticks = Seq_UsToTicks(SEQ_DESC_PRELOAD_US);
        }

        rt->sweep_start_pending = true;

        *dds_update_mask |= (1U << d);
        return;
    }

    if (item->type == SEQ_ITEM_FAST_HW_SWEEP) {
        Seq_ConfigFastHwSweep_NoStart(d,c,&item->hw_sweep);
        rt->phase = SEQ_RT_HW_SWEEP;
        rt->remaining_ticks = item->hw_sweep.tstep_ticks;
        rt->sweep_start_pending = false;
        rt->single_tone_ready = ((item->hw_sweep.fast_flags & SEQ_FAST_FLAG_PHASE_TRIANGLE) == 0U);
        rt->hw_current_decihz = item->hw_sweep.start_decihz;
        rt->hw_current_ftw = item->hw_sweep.start_ftw;
        rt->hw_next_update_deadline_ticks = 0UL;
        rt->hw_deadline_valid = false;
        rt->fast_wait_prepared = false;
        *dds_update_mask |= (1U << d);
        return;
    }

    if (item->type == SEQ_ITEM_HW_SWEEP) {
        Seq_SetSingleToneFTW_Optimized(d,c,item->hw_sweep.start_ftw,rt);

        rt->phase = SEQ_RT_HW_SWEEP;
        rt->remaining_ticks = item->hw_sweep.tstep_ticks;
        rt->sweep_start_pending = false;
        rt->hw_current_decihz = item->hw_sweep.start_decihz;
        rt->hw_current_ftw = item->hw_sweep.start_ftw;
        rt->hw_next_update_deadline_ticks = 0UL;
        rt->hw_deadline_valid = false;

        *dds_update_mask |= (1U << d);
        return;
    }

    rt->active = false;
    g_seq_active_mask &= ~Seq_GlobalBit(d,c);
    rt->phase = SEQ_RT_IDLE;
    rt->sweep_start_pending = false;
    rt->profile_low_after_update = false;
}

static void Seq_StartPendingSweeps(void)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;
        unsigned int d;
        unsigned int c;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;

        if (g_seq_rt[d][c].sweep_start_pending) {
            seq_item_t *item = (seq_item_t *)&g_seq_def[d][c].items[g_seq_rt[d][c].item_idx];

            if (item->type == SEQ_ITEM_SWEEP) {
                Seq_SetProfileHigh(d,c);
            }

            g_seq_rt[d][c].sweep_start_pending = false;
        }

        mask &= ~bit;
    }
}


static void Seq_StartPendingSweepsImmediate(void)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;
        unsigned int d;
        unsigned int c;
        seq_rt_t *rt;
        seq_item_t *item;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;
        rt = (seq_rt_t *)&g_seq_rt[d][c];

        /*
         * STEP -> linear sweep :
         * aucun PROFILE LOW différé n'est requis.
         * On peut donc lever PROFILE immédiatement après I/O_UPDATE.
         *
         * SWEEP -> SWEEP reste traité par Seq_StartPendingSweeps()
         * après Seq_ApplyDeferredProfileLows().
         */
        if (rt->sweep_start_pending && !rt->profile_low_after_update) {
            item = (seq_item_t *)&g_seq_def[d][c].items[rt->item_idx];

            if (item->type == SEQ_ITEM_SWEEP) {
                Seq_SetProfileHigh(d,c);
                rt->sweep_start_pending = false;
            }
        }

        mask &= ~bit;
    }
}

static void Seq_ApplyDeferredProfileLows(void)
{
    unsigned int mask = g_seq_profile_low_pending_mask & 0x0FFFU;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;
        unsigned int d;
        unsigned int c;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;

        if (g_seq_rt[d][c].profile_low_after_update) {
            Seq_SetProfileLow(d,c);
            g_seq_rt[d][c].profile_low_after_update = false;
        }

        g_seq_profile_low_pending_mask &= ~bit;
        mask &= ~bit;
    }
}

static void Seq_AdvanceOneChannel(unsigned int d, unsigned int c, unsigned int *dds_update_mask)
{
    seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];
    seq_def_t *def = (seq_def_t *)&g_seq_def[d][c];
    seq_item_t *item;

    if (!rt->active) return;
    if (rt->item_idx >= def->n_items) return;

    item = (seq_item_t *)&def->items[rt->item_idx];

    if (rt->phase == SEQ_RT_SWEEP_PRELOAD_DESC) {
        Seq_SetProfileLow(d,c);

        rt->phase = SEQ_RT_SWEEP;
        rt->remaining_ticks = item->sweep.duration_ticks;
        rt->sweep_start_pending = false;

        if ((rt->item_idx + 1U) == def->n_items) {
            rt->loop_deadline_active = true;
        }

        return;
    }

    if (rt->next_item_preloaded) {
        if (rt->phase == SEQ_RT_SWEEP) {
            rt->profile_low_after_update = true;
            g_seq_profile_low_pending_mask |= Seq_GlobalBit(d,c);
            /* V30.4 : le préchargement d'un STEP_EX dépourvu de drapeau
            * fréquence ne réécrit pas CFR. Il faut donc quitter
            * explicitement le mode sweep, sinon le canal reste piloté par
            * des RDW/FDW/LSRR périmés à la boucle suivante. */
            {
                const seq_item_t *next = (const seq_item_t *)&def->items[rt->preloaded_item_idx];
                bool next_rewrites_cfr =
                    (next->type == SEQ_ITEM_STEP) ||
                    (next->type == SEQ_ITEM_SWEEP) || /* V30.5 : CFR déjà écrit au préchargement. */
                    (next->type == SEQ_ITEM_HW_SWEEP) ||
                    (next->type == SEQ_ITEM_FAST_HW_SWEEP) ||
                    ((next->type == SEQ_ITEM_STEP_EX) &&
                     ((next->step.flags & SEQ_STEPEX_FLAG_FREQ) != 0U));
                if (!next_rewrites_cfr) {
                    Seq_DisableLinearSweep_NoUpdate(d,c);
                    rt->single_tone_ready = true;
                }
            }
        }

        rt->loop_idx = rt->preloaded_loop_idx;
        rt->item_idx = rt->preloaded_item_idx;
        rt->next_item_preloaded = false;

        if (!Seq_InitCurrentItemRuntime_NoWrite(d,c)) {
            rt->active = false;
            rt->phase = SEQ_RT_IDLE;
            rt->sweep_start_pending = false;
            g_seq_active_mask &= ~Seq_GlobalBit(d,c);
            return;
        }

        *dds_update_mask |= (1U << d);
        return;
    }

    if (rt->phase == SEQ_RT_HW_SWEEP) {
        unsigned long current_decihz = rt->hw_current_decihz;
        unsigned long target_decihz = item->hw_sweep.stop_decihz;
        unsigned long step_decihz = item->hw_sweep.step_decihz;
        unsigned long current_ftw = rt->hw_current_ftw;
        unsigned long target_ftw = item->hw_sweep.stop_ftw;
        unsigned long step_ftw = item->hw_sweep.step_ftw;
        unsigned long next_decihz;
        unsigned long next_ftw;

        /* Tous les calculs ci-dessous sont entiers. */
        if (target_decihz >= current_decihz) {
            if ((target_decihz - current_decihz) <=
                (step_decihz + SEQ_HW_FREQ_TOLERANCE_DECIHZ)) {
                next_decihz = target_decihz;
                next_ftw = target_ftw;
            } else {
                next_decihz = current_decihz + step_decihz;
                next_ftw = current_ftw + step_ftw;
            }
        } else {
            if ((current_decihz - target_decihz) <=
                (step_decihz + SEQ_HW_FREQ_TOLERANCE_DECIHZ)) {
                next_decihz = target_decihz;
                next_ftw = target_ftw;
            } else {
                next_decihz = current_decihz - step_decihz;
                next_ftw = current_ftw - step_ftw;
            }
        }

        rt->hw_current_decihz = next_decihz;
        rt->hw_current_ftw = next_ftw;
        Seq_WriteFTW_ChannelFast_NoUpdate(d,c,next_ftw);

        *dds_update_mask |= (1U << d);

        if (next_decihz != target_decihz) {
            rt->remaining_ticks = item->hw_sweep.tstep_ticks;
            return;
        }

        /* Chaînage HW -> HW sans réécrire inutilement CFR/CSR/FTW.
         * Le même I/O_UPDATE valide le dernier point du premier sweep ;
         * le timer suivant attend simplement le premier pas du retour. */
        if ((rt->item_idx + 1U) < def->n_items) {
            seq_item_t *following =
                (seq_item_t *)&def->items[rt->item_idx + 1U];

            if ((following->type == SEQ_ITEM_HW_SWEEP || following->type == SEQ_ITEM_FAST_HW_SWEEP) &&
                following->hw_sweep.start_decihz == target_decihz) {
                rt->item_idx++;
                rt->phase = SEQ_RT_HW_SWEEP;
                rt->remaining_ticks = following->hw_sweep.tstep_ticks;
                rt->sweep_start_pending = false;
                rt->single_tone_ready = true;
                rt->hw_current_decihz = following->hw_sweep.start_decihz;
                rt->hw_current_ftw = following->hw_sweep.start_ftw;
                return;
            }
        }
    }

    if (rt->phase == SEQ_RT_SWEEP) {
        bool has_next_item;
        bool has_next_loop;

        has_next_item = ((rt->item_idx + 1U) < def->n_items);
        has_next_loop = (!has_next_item && ((rt->loop_idx + 1U) < def->n_loops));

        /*
        * On garde PROFILE HIGH pendant la préparation.
        */
        Seq_DisableLinearSweep_NoUpdate(d,c);
        rt->single_tone_ready = true;
        rt->profile_low_after_update = true;
        g_seq_profile_low_pending_mask |= Seq_GlobalBit(d,c);

        /*
        * Seulement si la séquence se termine réellement sur ce sweep,
        * on force explicitement la fréquence finale.
        *
        * Sinon l'item suivant va écrire son propre FTW avant
        * le même I/O_UPDATE : inutile d'écrire stop_decihz ici.
        */
        if (!has_next_item && !has_next_loop) {
            Seq_SetFTW_NoUpdate(d,c,FreqDeciToFTW(item->sweep.stop_decihz));
        }

        *dds_update_mask |= (1U << d);
    }

    rt->item_idx++;

    if (rt->item_idx < def->n_items) {
        Seq_ApplyCurrentItem(d,c,dds_update_mask);
        return;
    }

    rt->loop_idx++;

    if (rt->loop_idx < def->n_loops) {
        rt->item_idx = 0U;
        Seq_ApplyCurrentItem(d,c,dds_update_mask);
        return;
    }

    rt->active = false;
    g_seq_active_mask &= ~Seq_GlobalBit(d,c);
    rt->phase = SEQ_RT_IDLE;
    rt->sweep_start_pending = false;
    rt->loop_deadline_active = false;
    rt->sequence_clock_started = false;

    /*
    * ATTENTION :
    * ne pas forcément mettre profile_low_after_update à false ici
    * si un I/O_UPDATE différé doit encore être consommé.
    */
}

static bool Seq_TryFastSingleHwSweep(uint32_t t_isr_start)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;
    int idx;
    unsigned int d;
    unsigned int c;
    seq_rt_t *rt;
    seq_def_t *def;
    seq_item_t *item;
    unsigned long current_decihz;
    unsigned long target_decihz;
    unsigned long step_decihz;
    unsigned long next_decihz;
    unsigned long next_ftw;
    bool reaches_target;
    bool chain_to_hw = false;
    bool chain_to_final_step = false;
    seq_item_t *following = 0;
    unsigned long next_logical_ticks;
    uint32_t t_after_update;
    unsigned long prep_ticks;

    if (mask == 0U) return false;
    if ((mask & (mask - 1U)) != 0U) return false;

    idx = Seq_FirstSetBit12(mask);
    if (idx < 0) return false;

    d = (unsigned int)idx / 4U;
    c = (unsigned int)idx % 4U;

    rt = (seq_rt_t *)&g_seq_rt[d][c];
    def = (seq_def_t *)&g_seq_def[d][c];

    if (!rt->active) return false;
    if (rt->phase != SEQ_RT_HW_SWEEP) return false;
    if (rt->item_idx >= def->n_items) return false;

    item = (seq_item_t *)&def->items[rt->item_idx];
    if (item->type != SEQ_ITEM_HW_SWEEP && item->type != SEQ_ITEM_FAST_HW_SWEEP) return false;
    if (!rt->hw_deadline_valid) return false;

    current_decihz = rt->hw_current_decihz;
    target_decihz = item->hw_sweep.stop_decihz;
    step_decihz = item->hw_sweep.step_decihz;

    if (target_decihz >= current_decihz) {
        reaches_target = (target_decihz - current_decihz) <= (step_decihz + SEQ_HW_FREQ_TOLERANCE_DECIHZ);

        if (reaches_target) {
            next_decihz = target_decihz;
            next_ftw = item->hw_sweep.stop_ftw;
        } else {
            next_decihz = current_decihz + step_decihz;
            next_ftw = rt->hw_current_ftw + item->hw_sweep.step_ftw;
        }
    } else {
        reaches_target = (current_decihz - target_decihz) <= (step_decihz + SEQ_HW_FREQ_TOLERANCE_DECIHZ);

        if (reaches_target) {
            next_decihz = target_decihz;
            next_ftw = item->hw_sweep.stop_ftw;
        } else {
            next_decihz = current_decihz - step_decihz;
            next_ftw = rt->hw_current_ftw - item->hw_sweep.step_ftw;
        }
    }

    if (reaches_target && (rt->item_idx + 1U) < def->n_items) {
        following = (seq_item_t *)&def->items[rt->item_idx + 1U];

        if (following->type == SEQ_ITEM_HW_SWEEP && following->hw_sweep.start_decihz == target_decihz) {
            chain_to_hw = true;
        } else if (following->type == SEQ_ITEM_STEP && (rt->item_idx + 2U) == def->n_items) {
            chain_to_final_step = true;
        }
    }

    if (reaches_target && !chain_to_hw && !chain_to_final_step) {
        return false;
    }

    /*
     * HW sweep -> dernier STEP :
     * la fréquence cible du sweep n'a pas de durée propre.
     * On ne fait donc PAS un I/O_UPDATE à 90 MHz puis un second à 80 MHz.
     * Le même instant de frontière applique directement le STEP suivant.
     */
    if (chain_to_final_step) {
        uint64_t ideal_offset_ticks;
        uint32_t loop_deadline_ticks;
        uint32_t desired_isr_ticks;
        int32_t logical_wait_ticks;
        unsigned long final_step_ftw;

        /*
         * Le Timer4 a été armé en avance pour préparer cette frontière.
         * On écrit le FTW du STEP suivant avant l'échéance, puis on attend
         * la deadline absolue du dernier point du HW sweep avant I/O_UPDATE.
         * Ainsi, la dernière fréquence intermédiaire ne reste pas affichée
         * pendant une conversion flottante ou pendant la préparation du STEP.
         */
        final_step_ftw = following->step.ftw;
        Seq_WriteFTW_FastSelected_NoUpdate(d,final_step_ftw);

        while ((int32_t)(SEQ_DWT_CYCCNT - rt->hw_next_update_deadline_ticks) < 0) {
        }

        Seq_IOUpdateMask(1U << d);
        t_after_update = SEQ_DWT_CYCCNT;

        if ((int32_t)(t_after_update - rt->hw_next_update_deadline_ticks) > 0) {
            g_seq_dbg_hw_exit_late_ticks = (unsigned long)(t_after_update - rt->hw_next_update_deadline_ticks);
        } else {
            g_seq_dbg_hw_exit_late_ticks = 0UL;
        }

        rt->item_idx++;
        rt->phase = SEQ_RT_STEP;
        rt->sweep_start_pending = false;
        rt->single_tone_ready = true;
        rt->next_item_preloaded = false;
        rt->hw_current_decihz = target_decihz;
        rt->hw_current_ftw = item->hw_sweep.stop_ftw;
        rt->hw_deadline_valid = false;
        rt->hw_next_update_deadline_ticks = 0UL;

        Seq_UpdateLoopDeadlineStateForCurrentItem(rt,def,following);

        if (rt->loop_deadline_active && rt->sequence_clock_started && rt->loop_duration_ticks != 0ULL) {
            ideal_offset_ticks = (uint64_t)(rt->loop_idx + 1U) * rt->loop_duration_ticks;
            loop_deadline_ticks = rt->sequence_start_ticks + (uint32_t)ideal_offset_ticks;
            desired_isr_ticks = loop_deadline_ticks - SEQ_LOOP_DEADLINE_LEAD_TICKS;
            logical_wait_ticks = (int32_t)(desired_isr_ticks - t_isr_start);

            if (logical_wait_ticks > (int32_t)SEQ_TIMER_MIN_TICKS) {
                rt->remaining_ticks = (unsigned long)logical_wait_ticks;
            } else {
                rt->remaining_ticks = SEQ_TIMER_MIN_TICKS;
                g_seq_dbg_deadline_miss++;
            }
        } else {
            rt->remaining_ticks = Seq_UsToTicks(following->step.duration_us);
        }

        /*
         * IMPORTANT : le STEP final est déjà physiquement actif.
         * On peut maintenant précharger sans danger le premier item de la
         * boucle suivante. À la deadline de boucle, l'ISR n'aura plus de
         * transaction SSI lourde à faire : elle attendra seulement l'instant
         * absolu puis produira l'I/O_UPDATE de 80 -> 60 MHz.
         */
        Seq_PreloadNextItemsSafe();

        prep_ticks = (unsigned long)(t_after_update - t_isr_start);
        g_seq_dbg_hw_fast_ticks = prep_ticks;

        if (prep_ticks > g_seq_dbg_hw_fast_max_ticks) {
            g_seq_dbg_hw_fast_max_ticks = prep_ticks;
        }

        Seq_ArmTimer4_ticks_compensated(rt->remaining_ticks,(unsigned long)(SEQ_DWT_CYCCNT - t_isr_start));
        return true;
    }

    rt->hw_current_decihz = next_decihz;
    rt->hw_current_ftw = next_ftw;

    Seq_WriteFTW_FastSelected_NoUpdate(d,next_ftw);
    Seq_IOUpdateMask(1U << d);

    if (chain_to_hw) {
        rt->item_idx++;
        rt->phase = SEQ_RT_HW_SWEEP;
        rt->sweep_start_pending = false;
        rt->single_tone_ready = true;
        rt->hw_current_decihz = following->hw_sweep.start_decihz;
        rt->hw_current_ftw = following->hw_sweep.start_ftw;
        next_logical_ticks = following->hw_sweep.tstep_ticks;
    } else {
        next_logical_ticks = item->hw_sweep.tstep_ticks;
    }

    rt->remaining_ticks = next_logical_ticks;

    t_after_update = SEQ_DWT_CYCCNT;
    prep_ticks = (unsigned long)(t_after_update - t_isr_start);

    g_seq_dbg_hw_fast_ticks = prep_ticks;

    if (prep_ticks > g_seq_dbg_hw_fast_max_ticks) {
        g_seq_dbg_hw_fast_max_ticks = prep_ticks;
    }

    g_seq_hw_fast_prep_estimate_ticks = (3UL * g_seq_hw_fast_prep_estimate_ticks + prep_ticks + 2UL) / 4UL;

    rt->hw_next_update_deadline_ticks = Seq_AdvanceHwDeadline(rt->hw_next_update_deadline_ticks,t_after_update,next_logical_ticks);
    Seq_ArmTimer4ForUpdateDeadline(rt->hw_next_update_deadline_ticks,g_seq_hw_fast_prep_estimate_ticks,next_logical_ticks);
    g_seq_timer_hw_owner_bit = 1U << (d * 4U + c);

    return true;
}

static bool Seq_TryFastHwSweepWithOtherChannels(uint32_t t_isr_start,unsigned long elapsed_ticks,unsigned int owner_bit)
{
    unsigned int active_mask = g_seq_active_mask & 0x0FFFU;
    unsigned int idx;
    unsigned int due_idx;
    unsigned int d;
    unsigned int c;
    unsigned int bit;
    seq_rt_t *rt;
    seq_def_t *def;
    seq_item_t *item;
    seq_item_t *following = 0;
    unsigned long current_ftw;
    unsigned long target_ftw;
    unsigned long step_ftw;
    unsigned long next_ftw;
    unsigned long next_logical_ticks = 0UL;
    unsigned long other_min_ticks = 0xFFFFFFFFUL;
    unsigned long next_scheduler_ticks;
    unsigned long late_ticks = 0UL;
    unsigned long prep_ticks;
    unsigned long prep_estimate;
    unsigned long max_lead;
    unsigned long postupdate_ticks;
    bool other_found = false;
    bool reaches_target;
    bool chain_to_hw = false;
    bool chain_to_final_step = false;
    bool hw_continues = false;
    uint32_t current_deadline;
    uint32_t t_before_update;
    uint32_t physical_update_ticks;
    uint32_t t_after_arm;

    if (active_mask == 0U || (active_mask & (active_mask - 1U)) == 0U) return false;
    if (owner_bit == 0U || (owner_bit & (owner_bit - 1U)) != 0U) return false;
    if ((active_mask & owner_bit) == 0U) return false;

    due_idx = (unsigned int)Seq_FirstSetBit12(owner_bit);
    if (due_idx >= 12U) return false;

    d = due_idx / 4U;
    c = due_idx % 4U;
    rt = (seq_rt_t *)&g_seq_rt[d][c];
    def = (seq_def_t *)&g_seq_def[d][c];

    if (!rt->active || rt->phase != SEQ_RT_HW_SWEEP || !rt->hw_deadline_valid) return false;
    if (rt->item_idx >= def->n_items) return false;

    item = (seq_item_t *)&def->items[rt->item_idx];
    if (item->type != SEQ_ITEM_HW_SWEEP) return false;

    /* Une autre frontière exactement simultanée reste traitée par le chemin générique. */
    for (idx = 0U; idx < 12U; idx++) {
        bit = 1U << idx;
        if ((active_mask & bit) == 0U || bit == owner_bit) continue;
        if (g_seq_rt[idx / 4U][idx % 4U].active && g_seq_rt[idx / 4U][idx % 4U].remaining_ticks <= elapsed_ticks) return false;
    }

    /* Un seul passage simple : décrémentation et recherche du prochain événement non-HW. */
    for (idx = 0U; idx < 12U; idx++) {
        seq_rt_t *scan_rt;

        bit = 1U << idx;
        if ((active_mask & bit) == 0U) continue;

        scan_rt = (seq_rt_t *)&g_seq_rt[idx / 4U][idx % 4U];
        if (scan_rt->remaining_ticks > elapsed_ticks) scan_rt->remaining_ticks -= elapsed_ticks;
        else scan_rt->remaining_ticks = 0UL;

        if (bit != owner_bit && scan_rt->remaining_ticks < other_min_ticks) {
            other_min_ticks = scan_rt->remaining_ticks;
            other_found = true;
        }
    }

    current_ftw = rt->hw_current_ftw;
    target_ftw = item->hw_sweep.stop_ftw;
    step_ftw = item->hw_sweep.step_ftw;
    current_deadline = rt->hw_next_update_deadline_ticks;

    if (target_ftw >= current_ftw) {
        reaches_target = (target_ftw - current_ftw) <= step_ftw;
        next_ftw = reaches_target ? target_ftw : current_ftw + step_ftw;
    } else {
        reaches_target = (current_ftw - target_ftw) <= step_ftw;
        next_ftw = reaches_target ? target_ftw : current_ftw - step_ftw;
    }

    if (reaches_target && (rt->item_idx + 1U) < def->n_items) {
        following = (seq_item_t *)&def->items[rt->item_idx + 1U];
        if (following->type == SEQ_ITEM_HW_SWEEP && following->hw_sweep.start_ftw == target_ftw) chain_to_hw = true;
        if (following->type == SEQ_ITEM_STEP && (rt->item_idx + 2U) == def->n_items) chain_to_final_step = true;
    }

    if (reaches_target && !chain_to_hw && !chain_to_final_step) return false;

    if (chain_to_final_step) Seq_WriteFTW_ChannelFast_NoUpdate(d,c,following->step.ftw);
    else Seq_WriteFTW_ChannelFast_NoUpdate(d,c,next_ftw);

    t_before_update = SEQ_DWT_CYCCNT;
    while ((int32_t)(SEQ_DWT_CYCCNT - current_deadline) < 0) {}
    Seq_IOUpdateOneFast(d);
    physical_update_ticks = SEQ_DWT_CYCCNT;

    if ((int32_t)(physical_update_ticks - current_deadline) > 0) late_ticks = (unsigned long)(physical_update_ticks - current_deadline);

    rt->hw_current_ftw = next_ftw;

    if (chain_to_final_step) {
        rt->item_idx++;
        rt->phase = SEQ_RT_STEP;
        rt->remaining_ticks = Seq_UsToTicks(following->step.duration_us);
        rt->sweep_start_pending = false;
        rt->single_tone_ready = true;
        rt->next_item_preloaded = false;
        rt->hw_current_decihz = item->hw_sweep.stop_decihz;
        rt->hw_deadline_valid = false;
        rt->hw_next_update_deadline_ticks = 0UL;
        Seq_UpdateLoopDeadlineStateForCurrentItem(rt,def,following);
    } else if (chain_to_hw) {
        rt->item_idx++;
        rt->phase = SEQ_RT_HW_SWEEP;
        rt->remaining_ticks = following->hw_sweep.tstep_ticks;
        rt->sweep_start_pending = false;
        rt->single_tone_ready = true;
        rt->hw_current_decihz = following->hw_sweep.start_decihz;
        rt->hw_current_ftw = following->hw_sweep.start_ftw;
        next_logical_ticks = following->hw_sweep.tstep_ticks;
        rt->hw_next_update_deadline_ticks = Seq_AdvanceHwDeadline(current_deadline,physical_update_ticks,next_logical_ticks);
        rt->hw_deadline_valid = true;
        hw_continues = true;
    } else {
        if (item->hw_sweep.stop_decihz >= rt->hw_current_decihz) rt->hw_current_decihz += item->hw_sweep.step_decihz;
        else rt->hw_current_decihz -= item->hw_sweep.step_decihz;
        rt->remaining_ticks = item->hw_sweep.tstep_ticks;
        next_logical_ticks = item->hw_sweep.tstep_ticks;
        rt->hw_next_update_deadline_ticks = Seq_AdvanceHwDeadline(current_deadline,physical_update_ticks,next_logical_ticks);
        rt->hw_deadline_valid = true;
        hw_continues = true;
    }

    if (other_found && late_ticks != 0UL) {
        for (idx = 0U; idx < 12U; idx++) {
            seq_rt_t *scan_rt;

            bit = 1U << idx;
            if ((active_mask & bit) == 0U || bit == owner_bit) continue;

            scan_rt = (seq_rt_t *)&g_seq_rt[idx / 4U][idx % 4U];
            if (scan_rt->remaining_ticks > late_ticks) scan_rt->remaining_ticks -= late_ticks;
            else scan_rt->remaining_ticks = 0UL;
        }

        if (other_min_ticks > late_ticks) other_min_ticks -= late_ticks;
        else other_min_ticks = 0UL;
    }

    prep_ticks = (unsigned long)(t_before_update - t_isr_start) + SEQ_TIMER4_ISR_ENTRY_ESTIMATE_TICKS;
    g_seq_dbg_hw_multi_fast_ticks = prep_ticks;
    if (prep_ticks > g_seq_dbg_hw_multi_fast_max_ticks) g_seq_dbg_hw_multi_fast_max_ticks = prep_ticks;
    g_seq_hw_multi_prep_estimate_ticks = (3UL * g_seq_hw_multi_prep_estimate_ticks + prep_ticks + 2UL) / 4UL;

    if (hw_continues && (!other_found || next_logical_ticks <= other_min_ticks)) {
        prep_estimate = g_seq_hw_multi_prep_estimate_ticks;
        if (next_logical_ticks > (2UL * SEQ_TIMER_RELOAD_MIN_TICKS)) {
            max_lead = next_logical_ticks - (2UL * SEQ_TIMER_RELOAD_MIN_TICKS);
            if (prep_estimate > max_lead) prep_estimate = max_lead;
        }

        Seq_ArmTimer4ForUpdateDeadline(rt->hw_next_update_deadline_ticks,prep_estimate,next_logical_ticks);
        g_seq_timer_hw_owner_bit = owner_bit;
    } else {
        next_scheduler_ticks = rt->remaining_ticks;
        if (other_found && other_min_ticks < next_scheduler_ticks) next_scheduler_ticks = other_min_ticks;
        postupdate_ticks = (unsigned long)(SEQ_DWT_CYCCNT - physical_update_ticks);
        Seq_ArmTimer4_ticks_compensated(next_scheduler_ticks,postupdate_ticks);
    }

    t_after_arm = SEQ_DWT_CYCCNT;
    g_seq_dbg_hw_multi_total_ticks = (unsigned long)(t_after_arm - t_isr_start);
    if (g_seq_dbg_hw_multi_total_ticks > g_seq_dbg_hw_multi_total_max_ticks) g_seq_dbg_hw_multi_total_max_ticks = g_seq_dbg_hw_multi_total_ticks;

    if (hw_continues && prep_ticks + SEQ_TIMER_RELOAD_MIN_TICKS >= next_logical_ticks) g_seq_dbg_hw_multi_deadline_miss++;
    if (late_ticks >= (item->hw_sweep.tstep_ticks / 4UL)) g_seq_dbg_hw_multi_deadline_miss++;

    return true;
}

static void Seq_InitHwDeadlinesAfterUpdate(uint32_t physical_update_ticks)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;
        unsigned int d;
        unsigned int c;
        seq_rt_t *rt;
        seq_def_t *def;
        seq_item_t *item;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;
        mask &= ~bit;

        rt = (seq_rt_t *)&g_seq_rt[d][c];
        def = (seq_def_t *)&g_seq_def[d][c];

        if (!rt->active) continue;
        if (rt->phase != SEQ_RT_HW_SWEEP) continue;
        if (rt->hw_deadline_valid) continue;
        if (rt->item_idx >= def->n_items) continue;

        item = (seq_item_t *)&def->items[rt->item_idx];

        if (item->type != SEQ_ITEM_HW_SWEEP && item->type != SEQ_ITEM_FAST_HW_SWEEP) continue;

        rt->hw_next_update_deadline_ticks = physical_update_ticks + item->hw_sweep.tstep_ticks;
        rt->hw_deadline_valid = true;
    }
}



static inline unsigned long Seq_HwTimerBase(unsigned int d)
{
    if (d == 0U) return TIMER5_BASE;
    if (d == 1U) return TIMER6_BASE;
    if (d == 2U) return TIMER7_BASE;
    return 0UL;
}

static inline void Seq_SetHwTimerPriority(unsigned int d,unsigned char priority)
{
    if (d == 0U) IntPrioritySet(INT_TIMER5A,priority);
    else if (d == 1U) IntPrioritySet(INT_TIMER6A,priority);
    else if (d == 2U) IntPrioritySet(INT_TIMER7A,priority);
}

static inline void Seq_RestoreHwTimeoutInterrupt(unsigned int d)
{
    unsigned long base = Seq_HwTimerBase(d);
    if (base == 0UL) return;
    TimerIntDisable(base,TIMER_TIMA_MATCH);
    HWREG(base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
    TimerIntEnable(base,TIMER_TIMA_TIMEOUT);
}

static inline bool Seq_AnyHwEngineActive(void)
{
    return g_seq_hw_engine[0].active || g_seq_hw_engine[1].active || g_seq_hw_engine[2].active;
}

static inline bool Seq_DdsBlockedByHw(unsigned int d)
{
    unsigned int dds_mask = 0x0FU << (4U * d);
    return g_seq_hw_engine[d].active || ((g_seq_hw_resume_pending_mask & dds_mask) != 0U);
}

static void Seq_StopAllHwEngines(void)
{
    unsigned int d;

    HWREG(TIMER5_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(TIMER6_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(TIMER7_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(TIMER5_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
    HWREG(TIMER6_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
    HWREG(TIMER7_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;

    for (d = 0U; d < DDS_COUNT; d++) {
        g_seq_hw_engine[d].active = false;
        g_seq_hw_engine[d].owner_ch = 0U;
        g_seq_hw_engine[d].period_ticks = 0UL;
        g_seq_hw_engine[d].next_deadline_ticks = 0UL;
        g_seq_hw_engine[d].prepared_action = SEQ_HW_ACTION_STOP;
        g_seq_hw_engine[d].prepared_next_loop = 0U;
        g_seq_hw_engine[d].prepared_next_item = 0U;
        g_seq_hw_engine[d].prepared_decihz = 0UL;
        g_seq_hw_engine[d].prepared_ftw = 0UL;
        g_seq_hw_engine[d].companion_due_mask = 0U;
        g_seq_hw_engine[d].companion_end_prepared_mask = 0U;
        g_seq_hw_engine[d].first_reload_pending = 0U;
        g_seq_hw_engine[d].fast_mode = false;
        g_seq_hw_engine[d].phase_triangle = false;
        g_seq_hw_engine[d].profile_pwm = false;
        g_seq_hw_engine[d].fast_diag_divider = 0U;
        g_seq_hw_engine[d].phase_half_ticks = 0UL;
        Seq_RestoreHwTimeoutInterrupt(d);
        Seq_SetHwTimerPriority(d,0x00U);
        g_seq_hw_resume_kind[d] = SEQ_HW_RESUME_NONE;
        g_seq_hw_resume_update_ticks[d] = 0UL;
    }

    g_seq_hw_resume_pending_mask = 0U;
    Seq_StopProfilePwm();
}

static void Seq_RequestTimer4Service(void)
{
    uint32_t now_ticks = SEQ_DWT_CYCCNT;

    if (g_seq_timer4_in_isr) {
        g_seq_timer4_force_service = true;
        return;
    }

    if (!g_seq_timer4_active) {
        g_seq_timer4_arm_start_ticks = now_ticks;
        g_seq_timer4_remaining_origin_ticks = now_ticks;
    }

    /* V30 : ce réveil est un service (handoff HW -> Timer4), pas une
     * frontière planifiée : aucune deadline absolue à attendre. */
    g_seq_timer4_deadline_valid = false;

    g_seq_timer4_force_service = true;
    HWREG(TIMER4_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(TIMER4_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;
    HWREG(TIMER4_BASE + TIMER_O_TAILR) = SEQ_TIMER_RELOAD_MIN_TICKS - 1UL;
    HWREG(TIMER4_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
    g_seq_timer4_active = true;
}

static bool Seq_HwPrepareNext(unsigned int d)
{
    seq_hw_engine_t *engine = (seq_hw_engine_t *)&g_seq_hw_engine[d];
    unsigned int c = engine->owner_ch;
    seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];
    seq_def_t *def = (seq_def_t *)&g_seq_def[d][c];
    seq_item_t *item;
    unsigned long current_decihz;
    unsigned long target_decihz;
    unsigned long step_decihz;
    unsigned long next_decihz;
    unsigned long next_ftw;
    bool reaches_target;
    unsigned int next_loop = 0U;
    unsigned int next_item = 0U;

    if (!engine->active || !rt->active || rt->phase != SEQ_RT_HW_SWEEP || rt->item_idx >= def->n_items) return false;

    item = (seq_item_t *)&def->items[rt->item_idx];
    if (item->type != SEQ_ITEM_HW_SWEEP && item->type != SEQ_ITEM_FAST_HW_SWEEP) return false;

    current_decihz = rt->hw_current_decihz;
    target_decihz = item->hw_sweep.stop_decihz;
    step_decihz = item->hw_sweep.step_decihz;

    if (target_decihz >= current_decihz) {
        reaches_target = (target_decihz - current_decihz) <= (step_decihz + SEQ_HW_FREQ_TOLERANCE_DECIHZ);
        next_decihz = reaches_target ? target_decihz : current_decihz + step_decihz;
        next_ftw = reaches_target ? item->hw_sweep.stop_ftw : rt->hw_current_ftw + item->hw_sweep.step_ftw;
    } else {
        reaches_target = (current_decihz - target_decihz) <= (step_decihz + SEQ_HW_FREQ_TOLERANCE_DECIHZ);
        next_decihz = reaches_target ? target_decihz : current_decihz - step_decihz;
        next_ftw = reaches_target ? item->hw_sweep.stop_ftw : rt->hw_current_ftw - item->hw_sweep.step_ftw;
    }

    engine->prepared_action = SEQ_HW_ACTION_POINT;
    engine->prepared_next_loop = rt->loop_idx;
    engine->prepared_next_item = rt->item_idx;
    engine->prepared_decihz = next_decihz;
    engine->prepared_ftw = next_ftw;

    if (reaches_target) {
        if (Seq_GetFollowingPosition(rt,def,&next_loop,&next_item)) {
            seq_item_t *following = (seq_item_t *)&def->items[next_item];

            engine->prepared_next_loop = next_loop;
            engine->prepared_next_item = next_item;

            if (!engine->fast_mode && following->type == SEQ_ITEM_HW_SWEEP && following->hw_sweep.start_decihz == target_decihz) {
                engine->prepared_action = SEQ_HW_ACTION_CHAIN_HW;
            } else if (following->type == SEQ_ITEM_STEP ||
                       (following->type == SEQ_ITEM_STEP_EX &&
                        (following->step.flags & SEQ_STEPEX_FLAG_FREQ) != 0U)) {
                /* V30 : un STEP_EX qui change la fréquence peut suivre le
                 * chemin rapide ENTER_STEP. Ses écritures phase/amplitude
                 * sont faites ici, hors du tick final. */
                engine->prepared_action = SEQ_HW_ACTION_ENTER_STEP;
                engine->prepared_ftw = following->step.ftw;

                if (engine->fast_mode) {
                    Seq_WriteRegFast(d,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);
                }

                if (following->type == SEQ_ITEM_STEP_EX) {
                    if (following->step.flags & SEQ_STEPEX_FLAG_PHASE) {
                        Seq_WriteRegFast(d,CPOW_ADRESS,(unsigned long)following->step.pow,CPOW_NUM_BYTE);
                    }
                    if (following->step.flags & SEQ_STEPEX_FLAG_AMP) {
                        Seq_WriteRegFast(d,ACR_ADRESS,SEQ_ACR_AMP_ENABLE | (unsigned long)following->step.asf,ACR_NUM_BYTE);
                    }
                }
            } else {
                engine->prepared_action = SEQ_HW_ACTION_HANDOFF;
                if (engine->fast_mode) {
                    Seq_WriteRegFast(d,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);
                }
            }
        } else {
            engine->prepared_action = SEQ_HW_ACTION_STOP;
            if (engine->fast_mode) {
                Seq_WriteRegFast(d,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);
            }
        }
    }

    if (engine->fast_mode) {
        /* CSR stays selected for the full fast item: only FTW address+4 bytes. */
        Seq_WriteRegFast(d,FTW_ADRESS,engine->prepared_ftw,FTW_NUM_BYTE);
    } else {
        /* Une autre channel du même DDS peut avoir changé le CSR. */
        Seq_WriteFTW_ChannelFast_NoUpdate(d,c,engine->prepared_ftw);
    }
    return true;
}

static void Seq_HwScheduleCompanions(unsigned int d,unsigned long next_period_ticks)
{
    seq_hw_engine_t *engine = (seq_hw_engine_t *)&g_seq_hw_engine[d];
    uint32_t t_start = SEQ_DWT_CYCCNT;
    unsigned int c;

    engine->companion_due_mask = 0U;
    engine->companion_end_prepared_mask = 0U;

    for (c = 0U; c < DDS_CH_COUNT; c++) {
        seq_rt_t *rt;
        seq_def_t *def;
        seq_item_t *current;
        unsigned int next_loop;
        unsigned int next_item;
        unsigned int local_bit = 1U << c;

        if (c == engine->owner_ch) continue;

        rt = (seq_rt_t *)&g_seq_rt[d][c];
        def = (seq_def_t *)&g_seq_def[d][c];

        if (!rt->active) continue;
        if (rt->phase == SEQ_RT_HW_SWEEP) {
            g_seq_dbg_hw_engine_conflict++;
            continue;
        }
        if (rt->remaining_ticks > next_period_ticks) continue;

        if ((next_period_ticks - rt->remaining_ticks) > g_seq_dbg_hw_companion_late[d]) g_seq_dbg_hw_companion_late[d] = next_period_ticks - rt->remaining_ticks;

        /* Le cas descending PRELOAD_DESC n'exige aucune écriture SSI. */
        if (rt->phase == SEQ_RT_SWEEP_PRELOAD_DESC) {
            engine->companion_due_mask |= local_bit;
            continue;
        }

        if (!Seq_GetFollowingPosition(rt,def,&next_loop,&next_item)) {
            current = (seq_item_t *)&def->items[rt->item_idx];

            if (rt->phase == SEQ_RT_SWEEP && current->type == SEQ_ITEM_SWEEP) {
                unsigned long final_ftw = current->sweep.rising ? current->sweep.ftw_high : current->sweep.ftw_low;
                Seq_DisableLinearSweep_NoUpdate(d,c);
                Seq_SetFTW_NoUpdate(d,c,final_ftw);
                engine->companion_end_prepared_mask |= local_bit;
            }

            engine->companion_due_mask |= local_bit;
            continue;
        }

        if (def->items[next_item].type == SEQ_ITEM_HW_SWEEP) {
            /* Une seule channel HW par DDS : elle sera reprise par Timer4
             * lorsque le moteur propriétaire aura libéré le DDS. */
            continue;
        }

        if (!rt->next_item_preloaded && Seq_PrepareItem_NoUpdate(d,c,&def->items[next_item],rt)) {
            rt->next_item_preloaded = true;
            rt->preloaded_loop_idx = next_loop;
            rt->preloaded_item_idx = next_item;
        }

        if (rt->next_item_preloaded) engine->companion_due_mask |= local_bit;
    }

    g_seq_dbg_hw_companion_preload_ticks[d] = (unsigned long)(SEQ_DWT_CYCCNT - t_start);
    if (g_seq_dbg_hw_companion_preload_ticks[d] > g_seq_dbg_hw_companion_preload_max_ticks[d]) g_seq_dbg_hw_companion_preload_max_ticks[d] = g_seq_dbg_hw_companion_preload_ticks[d];
}

static void Seq_HwCommitCompanionsBeforeUpdate(unsigned int d,unsigned int *committed_mask)
{
    seq_hw_engine_t *engine = (seq_hw_engine_t *)&g_seq_hw_engine[d];
    unsigned int due = engine->companion_due_mask & 0x0FU;

    *committed_mask = 0U;

    while (due != 0U) {
        unsigned int c;
        unsigned int local_bit;
        unsigned int dummy_update_mask = 0U;
        seq_rt_t *rt;

        for (c = 0U; c < DDS_CH_COUNT; c++) {
            if (due & (1U << c)) break;
        }
        if (c >= DDS_CH_COUNT) break;

        local_bit = 1U << c;
        due &= ~local_bit;
        rt = (seq_rt_t *)&g_seq_rt[d][c];

        if (!rt->active) continue;

        if (engine->companion_end_prepared_mask & local_bit) {
            if (rt->phase == SEQ_RT_SWEEP) {
                rt->profile_low_after_update = true;
                g_seq_profile_low_pending_mask |= Seq_GlobalBit(d,c);
            }
            rt->active = false;
            rt->phase = SEQ_RT_IDLE;
            rt->remaining_ticks = 0UL;
            rt->sweep_start_pending = false;
            rt->loop_deadline_active = false;
            rt->sequence_clock_started = false;
            g_seq_active_mask &= ~Seq_GlobalBit(d,c);
        } else {
            Seq_AdvanceOneChannel(d,c,&dummy_update_mask);
        }

        *committed_mask |= local_bit;
    }

    engine->companion_due_mask = 0U;
    engine->companion_end_prepared_mask = 0U;
}

static void Seq_HwFinalizeCompanionsAfterUpdate(unsigned int d,unsigned int committed_mask)
{
    unsigned int c;

    for (c = 0U; c < DDS_CH_COUNT; c++) {
        unsigned int local_bit = 1U << c;
        seq_rt_t *rt;

        if ((committed_mask & local_bit) == 0U) continue;

        rt = (seq_rt_t *)&g_seq_rt[d][c];

        if (rt->profile_low_after_update) {
            Seq_SetProfileLow(d,c);
            rt->profile_low_after_update = false;
            g_seq_profile_low_pending_mask &= ~Seq_GlobalBit(d,c);
        }

        if (rt->sweep_start_pending) {
            seq_def_t *def = (seq_def_t *)&g_seq_def[d][c];
            if (rt->item_idx < def->n_items && def->items[rt->item_idx].type == SEQ_ITEM_SWEEP) Seq_SetProfileHigh(d,c);
            rt->sweep_start_pending = false;
        }
    }
}

static void Seq_StartHwEnginesAfterUpdate(uint32_t physical_update_ticks)
{
    unsigned int d;

    for (d = 0U; d < DDS_COUNT; d++) {
        unsigned int c;
        unsigned int owner_count = 0U;
        unsigned int owner_ch = 0U;
        unsigned long timer_base;
        seq_rt_t *rt;
        seq_def_t *def;
        seq_item_t *item;

        if (g_seq_hw_engine[d].active) continue;

        for (c = 0U; c < DDS_CH_COUNT; c++) {
            if (g_seq_rt[d][c].active && g_seq_rt[d][c].phase == SEQ_RT_HW_SWEEP) {
                owner_count++;
                owner_ch = c;
            }
        }

        if (owner_count == 0U) continue;
        if (owner_count != 1U) {
            g_seq_dbg_hw_engine_conflict++;
            for (c = 0U; c < DDS_CH_COUNT; c++) {
                if (g_seq_rt[d][c].active && g_seq_rt[d][c].phase == SEQ_RT_HW_SWEEP) {
                    g_seq_rt[d][c].active = false;
                    g_seq_rt[d][c].phase = SEQ_RT_IDLE;
                    g_seq_rt[d][c].remaining_ticks = 0UL;
                    g_seq_active_mask &= ~Seq_GlobalBit(d,c);
                }
            }
            continue;
        }

        rt = (seq_rt_t *)&g_seq_rt[d][owner_ch];
        def = (seq_def_t *)&g_seq_def[d][owner_ch];
        if (rt->item_idx >= def->n_items) continue;

        item = (seq_item_t *)&def->items[rt->item_idx];
        if (item->type != SEQ_ITEM_HW_SWEEP && item->type != SEQ_ITEM_FAST_HW_SWEEP) continue;

        g_seq_hw_engine[d].active = true;
        g_seq_hw_engine[d].owner_ch = owner_ch;
        g_seq_hw_engine[d].period_ticks = item->hw_sweep.tstep_ticks;
        g_seq_hw_engine[d].next_deadline_ticks = physical_update_ticks + item->hw_sweep.tstep_ticks;
        g_seq_hw_engine[d].companion_due_mask = 0U;
        g_seq_hw_engine[d].companion_end_prepared_mask = 0U;
        g_seq_hw_engine[d].first_reload_pending = 0U;
        g_seq_hw_engine[d].fast_mode = (item->type == SEQ_ITEM_FAST_HW_SWEEP);
        g_seq_hw_engine[d].phase_triangle =
            (item->type == SEQ_ITEM_FAST_HW_SWEEP) &&
            ((item->hw_sweep.fast_flags & SEQ_FAST_FLAG_PHASE_TRIANGLE) != 0U);
        g_seq_hw_engine[d].profile_pwm = false;
        g_seq_hw_engine[d].fast_diag_divider = 0U;
        g_seq_hw_engine[d].phase_half_ticks = item->hw_sweep.phase_half_ticks;

        /* The fast boundary ISR is now the only CPU action every 2 us.  The
         * phase midpoint is generated by GPTM PWM, not by another ISR. */
        Seq_SetHwTimerPriority(d,0x00U);

        rt->hw_deadline_valid = false;
        rt->hw_next_update_deadline_ticks = 0UL;

        /*
         * V30 : la grille du sweep matériel doit être référencée à la
         * frontière PHYSIQUE (physical_update_ticks), pas à l'instant où ce
         * code arme le timer — il y a entre les deux le coût de
         * Seq_StartPendingSweepsImmediate(), des profils différés, etc.
         * On raccourcit donc le premier intervalle du temps déjà écoulé,
         * puis on restaurera la période complète au premier tick.
         */
        {
            unsigned long full_period = item->hw_sweep.tstep_ticks;
            unsigned long elapsed = (unsigned long)(SEQ_DWT_CYCCNT - physical_update_ticks);
            unsigned long first_reload;
            uint32_t pwm_sync_ticks = 0UL;

            timer_base = Seq_HwTimerBase(d);

            if (g_seq_hw_engine[d].fast_mode && g_seq_hw_engine[d].phase_triangle) {
                if (!Seq_StartProfilePwm(
                        d,
                        owner_ch,
                        full_period,
                        g_seq_hw_engine[d].phase_half_ticks,
                        timer_base,
                        &pwm_sync_ticks)) {
                    g_seq_hw_engine[d].active = false;
                    g_seq_hw_engine[d].fast_mode = false;
                    g_seq_dbg_hw_engine_miss[d]++;
                    rt->active = false;
                    rt->phase = SEQ_RT_IDLE;
                    rt->remaining_ticks = 0UL;
                    g_seq_active_mask &= ~Seq_GlobalBit(d,owner_ch);
                    continue;
                }
                g_seq_hw_engine[d].profile_pwm = true;
                g_seq_hw_engine[d].first_reload_pending = 0U;
                g_seq_hw_engine[d].next_deadline_ticks = pwm_sync_ticks + full_period;
            } else if (g_seq_hw_engine[d].fast_mode) {
                /* Fast staircase without phase pulse: same early-match scheme,
                 * but only the boundary timer is synchronized to DWT. */
                (void)elapsed;
                (void)first_reload;
                TimerIntDisable(timer_base,TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
                HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
                HWREG(timer_base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
                HWREG(timer_base + TIMER_O_TAILR) = full_period - 1UL;
                TimerMatchSet(timer_base,TIMER_A,SEQ_FAST_WAKE_LEAD_TICKS - 1UL);
                g_seq_hw_engine[d].first_reload_pending = 0U;
                g_seq_hw_engine[d].next_deadline_ticks = SEQ_DWT_CYCCNT + full_period;
                TimerIntEnable(timer_base,TIMER_TIMA_MATCH);
                HWREG(timer_base + TIMER_O_CTL) |= TIMER_CTL_TAEN;
            } else {
                if (elapsed + SEQ_TIMER_RELOAD_MIN_TICKS < full_period) {
                    first_reload = full_period - elapsed;
                    g_seq_hw_engine[d].first_reload_pending = 1U;
                } else {
                    /* Coût anormalement élevé : on garde la période pleine. */
                    first_reload = full_period;
                }

                TimerIntDisable(timer_base,TIMER_TIMA_MATCH);
                TimerIntEnable(timer_base,TIMER_TIMA_TIMEOUT);
                HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
                HWREG(timer_base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
                HWREG(timer_base + TIMER_O_TAILR) = first_reload - 1UL;
                HWREG(timer_base + TIMER_O_CTL) |= TIMER_CTL_TAEN;
            }
        }

        if (g_seq_hw_engine[d].fast_mode) {
            seq_fast_plan_t *plan = &g_seq_fast_plan[d];
            bool queued = true;

            if (!plan->valid || plan->owner_ch != owner_ch ||
                plan->item_idx != rt->item_idx || plan->count < 2U) {
                queued = false;
            } else if (plan->next_index <= 1U) {
                plan->next_index = 1U;
                queued = Seq_QueueFastFtwNonBlocking(d,plan->ftw[1]);
                if (queued) plan->next_index = 2U;
            }

            if (!queued) {
                HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
                g_seq_hw_engine[d].active = false;
                g_seq_hw_engine[d].fast_mode = false;
                Seq_RestoreHwTimeoutInterrupt(d);
                if (g_seq_hw_engine[d].profile_pwm) Seq_StopProfilePwm();
                g_seq_hw_engine[d].profile_pwm = false;
                Seq_SetHwTimerPriority(d,0x00U);
                g_seq_dbg_hw_engine_miss[d]++;
                rt->active = false;
                rt->phase = SEQ_RT_IDLE;
                rt->remaining_ticks = 0UL;
                g_seq_active_mask &= ~Seq_GlobalBit(d,owner_ch);
            }
        } else if (!Seq_HwPrepareNext(d)) {
            HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
            g_seq_hw_engine[d].active = false;
            Seq_SetHwTimerPriority(d,0x00U);
            g_seq_dbg_hw_engine_miss[d]++;
            rt->active = false;
            rt->phase = SEQ_RT_IDLE;
            rt->remaining_ticks = 0UL;
            g_seq_active_mask &= ~Seq_GlobalBit(d,owner_ch);
        } else {
            Seq_HwScheduleCompanions(d,g_seq_hw_engine[d].period_ticks);
        }
    }
}

static void Seq_StopPhasePulseTimer(void)
{
    HWREG(TIMER3_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(TIMER3_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;
    g_seq_phase_pulse_active = false;
}

static void Seq_ArmPhasePulseLow(unsigned int d,unsigned int c,uint32_t update_ticks,unsigned long half_ticks)
{
    uint32_t deadline = update_ticks + (uint32_t)half_ticks;
    int32_t remaining = (int32_t)(deadline - SEQ_DWT_CYCCNT);
    unsigned long reload = (remaining > (int32_t)SEQ_PHASE_TIMER_MIN_TICKS)
        ? (unsigned long)remaining : SEQ_PHASE_TIMER_MIN_TICKS;

    Seq_StopPhasePulseTimer();
    g_seq_phase_pulse_dds = d;
    g_seq_phase_pulse_channel = c;
    g_seq_phase_pulse_deadline = deadline;
    g_seq_phase_pulse_active = true;
    HWREG(TIMER3_BASE + TIMER_O_TAILR) = reload - 1UL;
    HWREG(TIMER3_BASE + TIMER_O_CTL) |= TIMER_CTL_TAEN;
}

void Timer3A_IntHandler(void)
{
    uint32_t now;
    unsigned long late;

    HWREG(TIMER3_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;
    HWREG(TIMER3_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    if (!g_seq_phase_pulse_active) return;

    while ((int32_t)(SEQ_DWT_CYCCNT - g_seq_phase_pulse_deadline) < 0) {}
    now = SEQ_DWT_CYCCNT;
    late = ((int32_t)(now - g_seq_phase_pulse_deadline) > 0)
        ? (unsigned long)(now - g_seq_phase_pulse_deadline) : 0UL;
    if (late > g_seq_dbg_phase_low_late_max_ticks) g_seq_dbg_phase_low_late_max_ticks = late;
    Seq_SetProfileLow(g_seq_phase_pulse_dds,g_seq_phase_pulse_channel);
    g_seq_phase_pulse_active = false;
}

static void Seq_PrepareWaitingSpecialAfterUpdate(void)
{
    unsigned int d,c;

    for (d = 0U; d < DDS_COUNT; d++) {
        for (c = 0U; c < DDS_CH_COUNT; c++) {
            seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];
            seq_def_t *def = (seq_def_t *)&g_seq_def[d][c];
            seq_item_t *item;
            seq_fast_plan_t *plan;

            if (!rt->active || rt->phase != SEQ_RT_WAIT_TRIGGER || rt->fast_wait_prepared) continue;
            if (rt->item_idx >= def->n_items) continue;
            item = (seq_item_t *)&def->items[rt->item_idx];

            if (item->type == SEQ_ITEM_FAST_HW_SWEEP) {
                plan = &g_seq_fast_plan[d];
                if (!plan->valid || plan->owner_ch != c || plan->item_idx != rt->item_idx || plan->count < 2U) {
                    g_seq_dbg_hw_engine_miss[d]++;
                    continue;
                }
                plan->next_index = 1U;
                if (!Seq_QueueFastFtwNonBlocking(d,plan->ftw[1])) {
                    g_seq_dbg_hw_engine_miss[d]++;
                    continue;
                }
                plan->next_index = 2U;
            }

            rt->fast_wait_prepared = true;
        }
    }
}

static bool Seq_ReleaseWaitingSpecialItems(unsigned int ttl)
{
    unsigned int d,c;
    bool released = false;

    for (d = 0U; d < DDS_COUNT; d++) {
        for (c = 0U; c < DDS_CH_COUNT; c++) {
            seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];
            seq_def_t *def = (seq_def_t *)&g_seq_def[d][c];
            seq_item_t *item;
            uint32_t start_ticks;
            unsigned int dummy_update_mask = 0U;

            if (!rt->active || rt->phase != SEQ_RT_WAIT_TRIGGER || rt->item_idx >= def->n_items) continue;
            item = (seq_item_t *)&def->items[rt->item_idx];
            if (!item->wait_for_trigger || (unsigned int)item->gate_ttl != ttl) continue;

            start_ticks = SEQ_DWT_CYCCNT;
            if (rt->sequence_clock_started && rt->fast_wait_start_ticks != 0UL) {
                rt->sequence_start_ticks += (uint32_t)(start_ticks - rt->fast_wait_start_ticks);
            }
            rt->fast_wait_start_ticks = 0UL;
            rt->loop_deadline_active = false;

            if (item->type == SEQ_ITEM_STEP) {
                /* A special step is written only at the secondary TTL.  It is
                 * deliberately not left in AD9959 buffer registers while the
                 * preceding item is waiting, because any unrelated IO_UPDATE
                 * on the same DDS would otherwise release it early. */
                Seq_SetSingleToneFTW_Optimized(d,c,item->step.ftw,rt);
                Seq_IOUpdateOneFast(d);
                start_ticks = SEQ_DWT_CYCCNT;
                rt->phase = SEQ_RT_STEP;
                rt->remaining_ticks = Seq_UsToTicks(item->step.duration_us);
                rt->single_tone_ready = true;
                rt->fast_wait_prepared = false;
                g_seq_dbg_special_step_release_count++;
                Seq_RequestTimer4Service();
                released = true;
                continue;
            }

            if (item->type == SEQ_ITEM_SWEEP) {
                /* Strategy A: the complete AD9959 sweep configuration and its
                 * start frequency were committed when the wait state began.
                 * The secondary TTL performs only the PROFILE edge. */
                if (Seq_SweepIsRising(&item->sweep)) Seq_SetProfileHigh(d,c);
                else Seq_SetProfileLow(d,c);
                rt->phase = SEQ_RT_SWEEP;
                rt->remaining_ticks = item->sweep.duration_ticks;
                rt->single_tone_ready = false;
                rt->sweep_start_pending = false;
                rt->fast_wait_prepared = false;
                g_seq_dbg_special_linear_release_count++;
                Seq_RequestTimer4Service();
                released = true;
                continue;
            }

            if (item->type == SEQ_ITEM_FAST_HW_SWEEP) {
                seq_fast_plan_t *plan = &g_seq_fast_plan[d];
                if (g_seq_hw_engine[d].active) continue;
                if (!rt->fast_wait_prepared || !plan->valid || plan->owner_ch != c || plan->item_idx != rt->item_idx) {
                    g_seq_dbg_hw_engine_miss[d]++;
                    continue;
                }

                rt->phase = SEQ_RT_HW_SWEEP;
                rt->remaining_ticks = item->hw_sweep.tstep_ticks;
                rt->hw_current_decihz = item->hw_sweep.start_decihz;
                rt->hw_current_ftw = item->hw_sweep.start_ftw;
                rt->single_tone_ready = ((item->hw_sweep.fast_flags & SEQ_FAST_FLAG_PHASE_TRIANGLE) == 0U);
                rt->fast_wait_prepared = false;

                /* The start FTW is already active and FTW[1] is already
                 * shifting/latched in the DDS input buffer.  start_ticks is
                 * therefore the exact logical origin of the first 2-us dwell. */
                Seq_StartHwEnginesAfterUpdate(start_ticks);
                g_seq_dbg_fast_gate_release_count++;
                released = true;
                continue;
            }

            (void)dummy_update_mask;
        }
    }
    return released;
}

static void Seq_FastHwTimerIntHandler(unsigned int d)
{
    unsigned long timer_base = Seq_HwTimerBase(d);
    seq_hw_engine_t *engine = (seq_hw_engine_t *)&g_seq_hw_engine[d];
    seq_fast_plan_t *plan = &g_seq_fast_plan[d];
    uint32_t t0 = SEQ_DWT_CYCCNT;
    uint32_t update_ticks = 0UL;
    unsigned int next_index;
    unsigned int committed_index;
    bool sample_diagnostics;

    HWREG(timer_base + TIMER_O_ICR) = TIMER_TIMA_MATCH;
    if (!engine->active || !engine->fast_mode || !plan->valid) {
        HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
        return;
    }

    if (engine->first_reload_pending) {
        engine->first_reload_pending = 0U;
        HWREG(timer_base + TIMER_O_TAILR) = engine->period_ticks - 1UL;
    }

    /* Diagnostic sampling is deliberately sparse.  The normal 2-us path must
     * contain no generic scheduler work, no FTW arithmetic and no per-point
     * volatile counter traffic. */
    engine->fast_diag_divider++;
    sample_diagnostics = (engine->fast_diag_divider >= SEQ_FAST_DIAGNOSTIC_STRIDE);
    if (sample_diagnostics) engine->fast_diag_divider = 0U;

    /* The match interrupt woke the CPU 0.5 us early.  Wait for the exact
     * periodic deadline, which is also the hardware PWM reload edge. */
    while ((int32_t)(SEQ_DWT_CYCCNT - engine->next_deadline_ticks) < 0) { }

    /* FTW[k] was shifted during the preceding dwell.  This is the only action
     * that defines the frequency boundary. */
    Seq_IOUpdateOneFast(d);
    update_ticks = SEQ_DWT_CYCCNT;
    {
        unsigned long late = 0UL;
        if ((int32_t)(update_ticks - engine->next_deadline_ticks) > 0) {
            late = (unsigned long)(update_ticks - engine->next_deadline_ticks);
        }
        if (late > g_seq_dbg_fast_late_max_ticks) {
            g_seq_dbg_fast_late_max_ticks = late;
        }
    }

    next_index = plan->next_index;
    if (next_index == 0U || next_index > plan->count) {
        unsigned int c = engine->owner_ch;
        unsigned int owner_bit = Seq_GlobalBit(d,c);
        seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];

        HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
        engine->active = false;
        engine->fast_mode = false;
        Seq_RestoreHwTimeoutInterrupt(d);
        if (engine->profile_pwm) Seq_StopProfilePwm();
        engine->profile_pwm = false;
        Seq_SetHwTimerPriority(d,0x00U);
        rt->active = false;
        rt->phase = SEQ_RT_IDLE;
        g_seq_active_mask &= ~owner_bit;
        g_seq_dbg_hw_engine_miss[d]++;
        return;
    }

    committed_index = next_index - 1U;

    if (committed_index + 1U < plan->count) {
        /* One five-byte FTW transaction is pushed into the eight-byte SSI FIFO.
         * There is no wait for BSY and no loop: the transfer completes during
         * the new 2-us dwell. */
        if (!Seq_QueueFastFtwNonBlocking(d,plan->ftw[next_index])) {
            unsigned int c = engine->owner_ch;
            unsigned int owner_bit = Seq_GlobalBit(d,c);
            seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];

            HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
            engine->active = false;
            engine->fast_mode = false;
            Seq_RestoreHwTimeoutInterrupt(d);
            if (engine->profile_pwm) Seq_StopProfilePwm();
            engine->profile_pwm = false;
            Seq_SetHwTimerPriority(d,0x00U);
            rt->active = false;
            rt->phase = SEQ_RT_IDLE;
            g_seq_active_mask &= ~owner_bit;
            g_seq_dbg_hw_engine_miss[d]++;
            return;
        }

        plan->next_index = next_index + 1U;

        if (sample_diagnostics) {
            g_seq_dbg_fast_isr_ticks = (unsigned long)(SEQ_DWT_CYCCNT - t0);
            if (g_seq_dbg_fast_isr_ticks > g_seq_dbg_fast_isr_max_ticks) {
                g_seq_dbg_fast_isr_max_ticks = g_seq_dbg_fast_isr_ticks;
            }
        }

        engine->next_deadline_ticks += engine->period_ticks;
        return;
    }

    /* Final point: leave the periodic path first, then perform the slower
     * handoff.  The complete staircase has already respected its 2-us grid. */
    g_seq_dbg_fast_points_committed = plan->count;

    {
        unsigned int c = engine->owner_ch;
        unsigned int owner_bit = Seq_GlobalBit(d,c);
        seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];
        seq_def_t *def = (seq_def_t *)&g_seq_def[d][c];

        HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
        engine->active = false;
        engine->fast_mode = false;
        Seq_RestoreHwTimeoutInterrupt(d);
        engine->phase_triangle = false;
        if (engine->profile_pwm) Seq_StopProfilePwm();
        engine->profile_pwm = false;
        engine->first_reload_pending = 0U;
        Seq_SetHwTimerPriority(d,0x00U);
        Seq_StopPhasePulseTimer();
        Seq_SetProfileLow(d,c);

        rt->item_idx++;
        if (rt->item_idx < def->n_items) {
            rt->phase = SEQ_RT_IDLE;
            rt->remaining_ticks = 0UL;
            rt->single_tone_ready = false;
            g_seq_hw_resume_kind[d] = SEQ_HW_RESUME_HANDOFF;
            g_seq_hw_resume_update_ticks[d] = update_ticks;
            g_seq_hw_resume_pending_mask |= owner_bit;
            Seq_RequestTimer4Service();
        } else {
            rt->active = false;
            rt->phase = SEQ_RT_IDLE;
            rt->remaining_ticks = 0UL;
            rt->sequence_clock_started = false;
            rt->loop_deadline_active = false;
            g_seq_active_mask &= ~owner_bit;
            if ((g_seq_active_mask & 0x0FFFU) != 0U) Seq_RequestTimer4Service();
            else {
                g_seq_any_running = false;
                g_seq_preload_request = true;
            }
        }
    }

    g_seq_dbg_fast_final_handoff_ticks = (unsigned long)(SEQ_DWT_CYCCNT - t0);
}

static void Seq_HwTimerIntHandler(unsigned int d)
{
    unsigned long timer_base;
    seq_hw_engine_t *engine;
    uint32_t t_isr_start;
    uint32_t physical_update_ticks;
    unsigned long late_ticks = 0UL;
    unsigned int c;
    unsigned int owner_bit;
    seq_rt_t *rt;
    seq_def_t *def;
    seq_item_t *following;
    bool continues = false;
    unsigned int companion_committed_mask = 0U;
    unsigned long elapsed_period_ticks;

    if (d < DDS_COUNT && g_seq_hw_engine[d].fast_mode) {
        Seq_FastHwTimerIntHandler(d);
        return;
    }

    timer_base = Seq_HwTimerBase(d);
    engine = (seq_hw_engine_t *)&g_seq_hw_engine[d];
    t_isr_start = SEQ_DWT_CYCCNT;
    HWREG(timer_base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;

    if (!engine->active) {
        HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
        return;
    }

    c = engine->owner_ch;
    owner_bit = Seq_GlobalBit(d,c);
    rt = (seq_rt_t *)&g_seq_rt[d][c];
    def = (seq_def_t *)&g_seq_def[d][c];
    elapsed_period_ticks = engine->period_ticks;

    /* V30 : le premier intervalle a pu être raccourci pour aligner la grille
     * du sweep sur la frontière physique. On restaure la période complète
     * pour tous les ticks suivants. Le retard réel du premier point est
     * mesuré via next_deadline_ticks ci-dessous. */
    if (engine->first_reload_pending) {
        engine->first_reload_pending = 0U;
        HWREG(timer_base + TIMER_O_TAILR) = engine->period_ticks - 1UL;
    }

    /* Les frontières des autres channels du même DDS sont validées par le
     * même I/O_UPDATE que le point HW. Leurs registres ont été préparés au
     * tick précédent, donc aucune transaction lourde n'est faite ici. */
    Seq_HwCommitCompanionsBeforeUpdate(d,&companion_committed_mask);
    Seq_IOUpdateOneFast(d);
    physical_update_ticks = SEQ_DWT_CYCCNT;
    Seq_HwFinalizeCompanionsAfterUpdate(d,companion_committed_mask);

    if ((int32_t)(physical_update_ticks - engine->next_deadline_ticks) > 0) late_ticks = (unsigned long)(physical_update_ticks - engine->next_deadline_ticks);
    g_seq_dbg_hw_engine_late_ticks[d] = late_ticks;
    if (late_ticks > g_seq_dbg_hw_engine_late_max_ticks[d]) g_seq_dbg_hw_engine_late_max_ticks[d] = late_ticks;
    if (late_ticks >= engine->period_ticks) g_seq_dbg_hw_engine_miss[d]++;

    rt->hw_current_decihz = engine->prepared_decihz;
    rt->hw_current_ftw = engine->prepared_ftw;

    if (engine->prepared_action == SEQ_HW_ACTION_POINT) {
        engine->next_deadline_ticks += engine->period_ticks;
        continues = true;
    } else if (engine->prepared_action == SEQ_HW_ACTION_CHAIN_HW) {
        rt->loop_idx = engine->prepared_next_loop;
        rt->item_idx = engine->prepared_next_item;
        following = (seq_item_t *)&def->items[rt->item_idx];
        rt->phase = SEQ_RT_HW_SWEEP;
        rt->remaining_ticks = following->hw_sweep.tstep_ticks;
        rt->single_tone_ready = true;
        rt->hw_current_decihz = following->hw_sweep.start_decihz;
        rt->hw_current_ftw = following->hw_sweep.start_ftw;
        engine->period_ticks = following->hw_sweep.tstep_ticks;
        engine->next_deadline_ticks += engine->period_ticks;
        HWREG(timer_base + TIMER_O_TAILR) = engine->period_ticks - 1UL;
        continues = true;
    } else if (engine->prepared_action == SEQ_HW_ACTION_ENTER_STEP) {
        rt->loop_idx = engine->prepared_next_loop;
        rt->item_idx = engine->prepared_next_item;
        following = (seq_item_t *)&def->items[rt->item_idx];
        rt->phase = SEQ_RT_STEP;
        rt->remaining_ticks = Seq_UsToTicks(following->step.duration_us);
        rt->single_tone_ready = true;
        rt->sweep_start_pending = false;
        rt->next_item_preloaded = false;
        rt->hw_deadline_valid = false;
        rt->hw_next_update_deadline_ticks = 0UL;
        Seq_UpdateLoopDeadlineStateForCurrentItem(rt,def,following);

        HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
        engine->active = false;
        g_seq_hw_resume_kind[d] = SEQ_HW_RESUME_STEP;
        g_seq_hw_resume_update_ticks[d] = physical_update_ticks;
        g_seq_hw_resume_pending_mask |= owner_bit;
        Seq_RequestTimer4Service();
    } else if (engine->prepared_action == SEQ_HW_ACTION_HANDOFF) {
        rt->loop_idx = engine->prepared_next_loop;
        rt->item_idx = engine->prepared_next_item;
        rt->phase = SEQ_RT_IDLE;
        rt->remaining_ticks = 0UL;
        rt->single_tone_ready = true;
        rt->hw_deadline_valid = false;
        rt->hw_next_update_deadline_ticks = 0UL;

        HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
        engine->active = false;
        g_seq_hw_resume_kind[d] = SEQ_HW_RESUME_HANDOFF;
        g_seq_hw_resume_update_ticks[d] = physical_update_ticks;
        g_seq_hw_resume_pending_mask |= owner_bit;
        Seq_RequestTimer4Service();
    } else {
        /*
         * Fin réelle de la séquence sur un HW sweep.
         *
         * Aucun état n'est à reprendre pour le canal propriétaire : il vient
         * d'être désactivé. Ne surtout pas laisser owner_bit dans
         * g_seq_hw_resume_pending_mask. Sinon, lorsque ce canal était le seul
         * actif, Timer4 n'est jamais réveillé pour effacer le bit et
         * Seq_DdsBlockedByHw() bloque définitivement ce DDS au trigger suivant.
         */
        HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
        engine->active = false;
        engine->first_reload_pending = 0U;
        engine->companion_due_mask = 0U;
        engine->companion_end_prepared_mask = 0U;

        rt->active = false;
        rt->phase = SEQ_RT_IDLE;
        rt->remaining_ticks = 0UL;
        rt->sequence_clock_started = false;
        rt->loop_deadline_active = false;
        g_seq_active_mask &= ~owner_bit;

        g_seq_hw_resume_kind[d] = SEQ_HW_RESUME_NONE;
        g_seq_hw_resume_update_ticks[d] = 0UL;
        g_seq_hw_resume_pending_mask &= ~owner_bit;

        if ((g_seq_active_mask & 0x0FFFU) != 0U) {
            Seq_RequestTimer4Service();
        } else {
            g_seq_any_running = false;
            g_seq_preload_request = true;
        }
    }

    /* Pendant qu'un moteur HW possède ce DDS, Timer4 n'en décrémente pas
     * les autres channels. Leur horloge avance ici d'une période HW. */
    {
        unsigned int other_c;
        for (other_c = 0U; other_c < DDS_CH_COUNT; other_c++) {
            seq_rt_t *other_rt;
            unsigned int local_bit = 1U << other_c;

            if (other_c == c) continue;
            if (companion_committed_mask & local_bit) continue;

            other_rt = (seq_rt_t *)&g_seq_rt[d][other_c];
            if (!other_rt->active) continue;

            if (other_rt->remaining_ticks > elapsed_period_ticks) other_rt->remaining_ticks -= elapsed_period_ticks;
            else other_rt->remaining_ticks = 0UL;
        }
    }

    if (continues) {
        if (!Seq_HwPrepareNext(d)) {
            HWREG(timer_base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
            engine->active = false;
            g_seq_dbg_hw_engine_miss[d]++;
            rt->active = false;
            rt->phase = SEQ_RT_IDLE;
            rt->remaining_ticks = 0UL;
            g_seq_active_mask &= ~owner_bit;
            g_seq_hw_resume_kind[d] = SEQ_HW_RESUME_NONE;
            g_seq_hw_resume_update_ticks[d] = physical_update_ticks;
            g_seq_hw_resume_pending_mask |= owner_bit;
            Seq_RequestTimer4Service();
        } else {
            Seq_HwScheduleCompanions(d,engine->period_ticks);
        }
    }

    g_seq_dbg_hw_engine_isr_ticks[d] = (unsigned long)(SEQ_DWT_CYCCNT - t_isr_start);
    if (g_seq_dbg_hw_engine_isr_ticks[d] > g_seq_dbg_hw_engine_isr_max_ticks[d]) g_seq_dbg_hw_engine_isr_max_ticks[d] = g_seq_dbg_hw_engine_isr_ticks[d];
}

static void Seq_ProcessHwResumePending(unsigned int *dds_update_mask,uint32_t now_ticks)
{
    unsigned int pending = g_seq_hw_resume_pending_mask & 0x0FFFU;

    while (pending != 0U) {
        int idx = Seq_FirstSetBit12(pending);
        unsigned int bit;
        unsigned int d;
        unsigned int c;
        seq_rt_t *rt;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;
        rt = (seq_rt_t *)&g_seq_rt[d][c];

        if (g_seq_hw_resume_kind[d] == SEQ_HW_RESUME_STEP) {
            unsigned long elapsed = (unsigned long)(now_ticks - g_seq_hw_resume_update_ticks[d]);
            if (rt->remaining_ticks > elapsed) rt->remaining_ticks -= elapsed;
            else rt->remaining_ticks = 0UL;
        } else if (g_seq_hw_resume_kind[d] == SEQ_HW_RESUME_HANDOFF) {
            Seq_ApplyCurrentItem(d,c,dds_update_mask);
        }

        g_seq_hw_resume_kind[d] = SEQ_HW_RESUME_NONE;
        g_seq_hw_resume_update_ticks[d] = 0UL;
        g_seq_hw_resume_pending_mask &= ~bit;
        pending &= ~bit;
    }
}

/* =============================================================================
 * WRAPPERS DES TIMERS TEMPS REEL
 * =============================================================================
 * En mode TABLE, Timer4/5/6 correspondent respectivement à DDS1/DDS2/DDS3.
 * Les handlers délèguent immédiatement au moteur de tables.
 */
void Timer5A_IntHandler(void)
{
    if (g_v34_table_mode) Tbl34_Timer_DDS1_IntHandler();
    else Seq_HwTimerIntHandler(0U);
}

void Timer6A_IntHandler(void)
{
    if (g_v34_table_mode) Tbl34_Timer_DDS2_IntHandler();
    else Seq_HwTimerIntHandler(1U);
}

void Timer7A_IntHandler(void)
{
    if (g_v34_table_mode) {
        /* V36.5: Timer7A is no longer used for normal main-event preloading.
         * Keep this compatibility handler registered; the physical Timer7
         * peripheral stays stopped in Hybrid 2.0. */
        HWREG(TIMER7_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
        HWREG(TIMER7_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
        Tbl34_PreloadWorkerIntHandler();
    } else Seq_HwTimerIntHandler(2U);
}

void Seq_Timer4A_IntHandler(void)
{
    unsigned int d, c;
    unsigned long elapsed_ticks;
    unsigned long next_ticks;
    unsigned int dds_update_mask = 0U;
    bool loop_deadline_due = false;
    uint32_t loop_deadline_target_ticks = 0UL;
    uint32_t t_isr_start = SEQ_DWT_CYCCNT;
    unsigned int mask;
    uint32_t physical_update_ticks;
    bool current_boundary_is_preloaded_step_to_step;
    unsigned int current_due_count = 0U;
    unsigned int current_unprepared_count = 0U;
    bool forced_service = g_seq_timer4_force_service;

    g_seq_timer4_in_isr = true;
    g_seq_timer4_force_service = false;

    if (forced_service) {
        elapsed_ticks = g_seq_timer4_active
            ? (unsigned long)(t_isr_start - g_seq_timer4_remaining_origin_ticks)
            : 0UL;
        /* Après cette soustraction, tous les remaining_ticks sont de nouveau
         * référencés à l'instant physique de ce réveil de service. */
        g_seq_timer4_remaining_origin_ticks = t_isr_start;
    } else {
        elapsed_ticks = g_seq_timer_armed_ticks;
    }

    HWREG(TIMER4_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;
    HWREG(TIMER4_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN);
    g_seq_timer4_active = false;

    current_boundary_is_preloaded_step_to_step = Seq_IsSinglePreloadedStepToStep();

    /* Les DDS pris en charge par Timer5/6/7 sont entièrement exclus du
     * scheduler général. Cela évite toute collision SSI avec le moteur HW. */
    mask = g_seq_active_mask & 0x0FFFU;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;

        if (!Seq_DdsBlockedByHw(d) && g_seq_rt[d][c].phase != SEQ_RT_WAIT_TRIGGER) {
            if (g_seq_rt[d][c].remaining_ticks > elapsed_ticks) g_seq_rt[d][c].remaining_ticks -= elapsed_ticks;
            else g_seq_rt[d][c].remaining_ticks = 0UL;
        }

        mask &= ~bit;
    }

    /* Réintégration des channels qui viennent de quitter un HW sweep. */
    Seq_ProcessHwResumePending(&dds_update_mask,t_isr_start);

    mask = g_seq_active_mask & 0x0FFFU;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;
        seq_rt_t *rt;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;
        rt = (seq_rt_t *)&g_seq_rt[d][c];

        if (!Seq_DdsBlockedByHw(d) && rt->phase != SEQ_RT_WAIT_TRIGGER && rt->remaining_ticks == 0UL && rt->loop_deadline_active && rt->sequence_clock_started && rt->loop_duration_ticks != 0ULL) {
            uint64_t ideal_offset_ticks = (uint64_t)(rt->loop_idx + 1U) * rt->loop_duration_ticks;
            uint32_t target_ticks = rt->sequence_start_ticks + (uint32_t)ideal_offset_ticks;

            if (!loop_deadline_due) {
                loop_deadline_due = true;
                loop_deadline_target_ticks = target_ticks;
            }
        }

        mask &= ~bit;
    }

    current_due_count = Seq_CountChannelsDueAtDelay(0UL);
    current_unprepared_count = Seq_CountUnpreparedChannelsDueAtDelay(0UL);
    g_seq_dbg_timer4_due_count = current_due_count;
    g_seq_dbg_timer4_unprepared_count = current_unprepared_count;

    mask = g_seq_active_mask & 0x0FFFU;

    while (mask != 0U) {
        int idx = Seq_FirstSetBit12(mask);
        unsigned int bit;

        if (idx < 0) break;

        bit = 1U << (unsigned int)idx;
        d = (unsigned int)idx / 4U;
        c = (unsigned int)idx % 4U;

        if (!Seq_DdsBlockedByHw(d) && g_seq_rt[d][c].phase != SEQ_RT_WAIT_TRIGGER && g_seq_rt[d][c].remaining_ticks == 0UL) Seq_AdvanceOneChannel(d,c,&dds_update_mask);

        mask &= ~bit;
    }

    {
        uint32_t t_prep_done = SEQ_DWT_CYCCNT;
        uint32_t t_before_update;
        uint32_t boundary_target_ticks = 0UL;
        bool boundary_target_valid = false;

        /*
         * V30 : le coût de préparation est mesuré AVANT toute attente
         * active. Sinon l'estimateur EWMA apprendrait la marge de réveil
         * elle-même et la compensation divergerait.
         */
        {
            unsigned long sample_ticks =
                (unsigned long)(t_prep_done - t_isr_start)
                + SEQ_TIMER4_ISR_ENTRY_ESTIMATE_TICKS;

            Seq_UpdateTimer4PreupdateEstimate(
                current_due_count,
                current_unprepared_count,
                sample_ticks
            );

            if (current_boundary_is_preloaded_step_to_step) {
                g_seq_dbg_step_to_step_pre_ticks = sample_ticks;
                g_seq_step_to_step_pre_estimate_ticks =
                    g_seq_preupdate_estimate_ticks[1U];
            }
        }

        /*
         * V30 — cœur de la correction de précision.
         *
         * Timer4 est volontairement réveillé en avance (compensation
         * adaptative). En V28, l'I/O_UPDATE partait dès la fin de la
         * préparation : toute surestimation de la compensation devenait
         * une AVANCE PHYSIQUE de la frontière (jusqu'à ~22 µs avec les
         * buckets initiaux). C'est ce qui tronquait le linear sweep avant
         * 130 MHz et faisait démarrer le sweep matériel trop tôt.
         *
         * Désormais l'ISR attend la deadline absolue de la frontière
         * avant de produire l'impulsion. La marge de réveil est absorbée
         * ici, dans une attente active courte, et n'affecte plus l'instant
         * physique du changement.
         */
        if (loop_deadline_due) {
            boundary_target_ticks = loop_deadline_target_ticks;
            boundary_target_valid = true;
        } else if (!forced_service &&
                   g_seq_timer4_deadline_valid &&
                   dds_update_mask != 0U) {
            boundary_target_ticks = g_seq_timer4_deadline_ticks;
            boundary_target_valid = true;
        }

        if (boundary_target_valid) {
            g_seq_dbg_boundary_spin_ticks =
                ((int32_t)(boundary_target_ticks - SEQ_DWT_CYCCNT) > 0)
                    ? (unsigned long)(boundary_target_ticks - SEQ_DWT_CYCCNT)
                    : 0UL;

            while ((int32_t)(SEQ_DWT_CYCCNT - boundary_target_ticks) < 0) {}
        }

        t_before_update = SEQ_DWT_CYCCNT;

        Seq_IOUpdateMask(dds_update_mask);
        physical_update_ticks = SEQ_DWT_CYCCNT;
        Seq_PrepareWaitingSpecialAfterUpdate();

        if (boundary_target_valid) {
            unsigned long late =
                ((int32_t)(physical_update_ticks - boundary_target_ticks) > 0)
                    ? (unsigned long)(physical_update_ticks - boundary_target_ticks)
                    : 0UL;
            g_seq_dbg_boundary_late_ticks = late;
            if (late > g_seq_dbg_boundary_late_max_ticks) g_seq_dbg_boundary_late_max_ticks = late;
        }

        if (loop_deadline_due && (int32_t)(physical_update_ticks - loop_deadline_target_ticks) > 0) g_seq_dbg_loop_boundary_late_ticks = (unsigned long)(physical_update_ticks - loop_deadline_target_ticks);
        else if (loop_deadline_due) g_seq_dbg_loop_boundary_late_ticks = 0UL;

        Seq_StartPendingSweepsImmediate();
        Seq_StartHwEnginesAfterUpdate(physical_update_ticks);
        Seq_ApplyDeferredProfileLows();
        Seq_StartPendingSweeps();
        Seq_ApplyLoopEndDeadlineCompensation(t_isr_start);

        /* Un moteur HW peut avoir préempté Timer4 pendant cette ISR. */
        {
            unsigned int late_update_mask = 0U;
            Seq_ProcessHwResumePending(&late_update_mask,SEQ_DWT_CYCCNT);
            if (late_update_mask != 0U) {
                Seq_IOUpdateMask(late_update_mask);
                Seq_PrepareWaitingSpecialAfterUpdate();
                Seq_StartPendingSweepsImmediate();
                Seq_StartHwEnginesAfterUpdate(SEQ_DWT_CYCCNT);
                Seq_ApplyDeferredProfileLows();
                Seq_StartPendingSweeps();
            }
        }

        Seq_PreloadNextItemsSafe();

        if (Seq_FindNextDelayTicks(&next_ticks)) {
            uint32_t boundary_origin_ticks;
            uint32_t next_deadline_ticks;
            uint32_t now_before_arm;
            unsigned long predicted_next_preupdate_ticks = Seq_PredictTimer4PreupdateTicks(next_ticks);

            /*
             * V30.3 : les deadlines sont chaînées en absolu et l'armement est
             * calculé depuis la deadline elle-même, au dernier moment. On ne
             * capture plus t_before_arm AVANT la prédiction, ce qui oubliait
             * plusieurs microsecondes de parcours lorsque deux canaux étaient
             * actifs.
             */
            if (loop_deadline_due) {
                boundary_origin_ticks = loop_deadline_target_ticks;
            } else if (!forced_service && g_seq_timer4_deadline_valid) {
                boundary_origin_ticks = g_seq_timer4_deadline_ticks;
            } else {
                boundary_origin_ticks = t_isr_start;
            }

            next_deadline_ticks = boundary_origin_ticks + (uint32_t)next_ticks;
            g_seq_timer4_deadline_ticks = next_deadline_ticks;
            g_seq_timer4_deadline_valid = true;

            now_before_arm = SEQ_DWT_CYCCNT;
            g_seq_last_compensation_ticks = (unsigned long)(now_before_arm - boundary_origin_ticks) + predicted_next_preupdate_ticks;
            Seq_ArmTimer4ForUpdateDeadline(next_deadline_ticks,predicted_next_preupdate_ticks,next_ticks);

            g_seq_dbg_preupdate_ticks = (unsigned long)(t_before_update - t_isr_start);
            g_seq_dbg_postupdate_ticks = (unsigned long)(SEQ_DWT_CYCCNT - physical_update_ticks);
            g_seq_dbg_total_boundary_ticks = (unsigned long)(SEQ_DWT_CYCCNT - t_isr_start);
        } else {
            g_seq_timer4_deadline_valid = false;
            g_seq_timer4_remaining_origin_ticks = SEQ_DWT_CYCCNT;
            g_seq_any_running = Seq_AnyHwEngineActive() || ((g_seq_active_mask & 0x0FFFU) != 0U);
            if (!g_seq_any_running) g_seq_preload_request = true;
        }
    }

    {
        bool service_again = g_seq_timer4_force_service;
        g_seq_timer4_in_isr = false;
        if (service_again) Seq_RequestTimer4Service();
    }
}

void Timer4A_IntHandler(void)
{
    if (g_v34_table_mode) Tbl34_Timer_DDS0_IntHandler();
    else Seq_Timer4A_IntHandler();
}

static void Seq_StopRuntimeOnly(void)
{
    unsigned int d, c;
    Seq_StopAllHwEngines();
    Seq_StopPhasePulseTimer();
    HWREG(TIMER4_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    g_seq_timer4_active = false;
    g_seq_timer4_force_service = false;
    g_seq_timer4_in_isr = false;
    g_seq_timer4_deadline_valid = false;
    g_seq_timer4_remaining_origin_ticks = 0UL;
    g_seq_timer_armed_ticks = 0UL;
    g_seq_timer_hw_owner_bit = 0U;
    g_seq_last_compensation_ticks = 0UL;

    for (d = 0U; d < DDS_COUNT; d++) {
        for (c = 0U; c < DDS_CH_COUNT; c++) {
            g_seq_rt[d][c].active = false;
            g_seq_rt[d][c].phase = SEQ_RT_IDLE;
            g_seq_rt[d][c].sweep_start_pending = false;
            g_seq_rt[d][c].profile_low_after_update = false;
            g_seq_rt[d][c].first_item_preloaded = false;
            g_seq_rt[d][c].next_item_preloaded = false;
            g_seq_rt[d][c].sequence_clock_started = false;
            g_seq_rt[d][c].loop_deadline_active = false;
            g_seq_rt[d][c].remaining_ticks = 0UL;
            g_seq_rt[d][c].hw_deadline_valid = false;
            g_seq_rt[d][c].fast_wait_prepared = false;
            g_seq_rt[d][c].fast_wait_start_ticks = 0UL;
        }
    }
    g_seq_any_running = false;
    g_seq_active_mask = 0U;
    g_seq_profile_low_pending_mask = 0U;
}

void Seq_StopAll(void)
{
    unsigned int d, c;
    unsigned int dds_update_mask = 0U;
    Seq_StopAllHwEngines();
    Seq_StopPhasePulseTimer();

    HWREG(TIMER4_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    g_seq_timer4_active = false;
    g_seq_timer4_force_service = false;
    g_seq_timer4_in_isr = false;
    g_seq_timer4_deadline_valid = false;
    g_seq_timer4_remaining_origin_ticks = 0UL;
    g_seq_timer_armed_ticks = 0UL;
    g_seq_timer_hw_owner_bit = 0U;
    g_seq_last_compensation_ticks = 0UL;

    for (d = 0U; d < DDS_COUNT; d++) {
        for (c = 0U; c < DDS_CH_COUNT; c++) {
            g_seq_rt[d][c].active = false;
            g_seq_rt[d][c].phase = SEQ_RT_IDLE;
            g_seq_rt[d][c].sweep_start_pending = false;
            g_seq_rt[d][c].profile_low_after_update = false;
            g_seq_rt[d][c].first_item_preloaded = false;
            g_seq_rt[d][c].remaining_ticks = 0UL;
            g_seq_rt[d][c].hw_current_decihz = 0UL;
            g_seq_rt[d][c].hw_current_ftw = 0UL;
            g_seq_rt[d][c].hw_next_update_deadline_ticks = 0UL;
            g_seq_rt[d][c].hw_deadline_valid = false;
            g_seq_rt[d][c].hw_current_ftw = 0UL;
            g_seq_rt[d][c].next_item_preloaded = false;

            g_seq_rt[d][c].sequence_clock_started = false;
            g_seq_rt[d][c].loop_deadline_active = false;
            g_seq_rt[d][c].sequence_start_ticks = 0UL;
            g_seq_rt[d][c].loop_duration_ticks = 0ULL;

            Seq_SetProfileLow(d,c);
            Seq_DisableLinearSweep_NoUpdate(d,c);
            Seq_SetFTW_NoUpdate(d,c,0UL);
            Seq_WriteRegFast(d,CPOW_ADRESS,0UL,CPOW_NUM_BYTE);
            Seq_WriteRegFast(d,ACR_ADRESS,SEQ_ACR_AMP_ENABLE,ACR_NUM_BYTE);

            dds_update_mask |= (1U << d);
        }
    }

    Seq_IOUpdateMask(dds_update_mask);
    g_seq_any_running = false;
    g_seq_active_mask = 0U;
    g_seq_profile_low_pending_mask = 0U;
}

void Seq_ResetAllChannels(void)
{
    unsigned int d, c, i;
    unsigned int dds_update_mask = 0U;
    Seq_StopAllHwEngines();
    Seq_StopPhasePulseTimer();

    HWREG(TIMER4_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    g_seq_timer4_active = false;
    g_seq_timer4_force_service = false;
    g_seq_timer4_in_isr = false;
    g_seq_timer4_deadline_valid = false;
    g_seq_timer4_remaining_origin_ticks = 0UL;
    g_seq_timer_armed_ticks = 0UL;
    g_seq_timer_hw_owner_bit = 0U;
    g_seq_last_compensation_ticks = 0UL;
    g_seq_dbg_deadline_miss = 0UL;
    g_seq_pending_trigger_flags = 0U;

    g_seq_arm_mask_by_ttl[0] = 0U;
    g_seq_arm_mask_by_ttl[1] = 0U;
    g_seq_arm_mask_by_ttl[2] = 0U;

    g_seq_priority_ttl[0] = SEQ_TRIGGER_TTL6;
    g_seq_priority_ttl[1] = SEQ_TRIGGER_TTL8;
    g_seq_priority_ttl[2] = SEQ_TRIGGER_TTL7;
    g_seq_retrigger_policy = SEQ_RETRIGGER_HIGHER_PRIORITY;

    for (d = 0U; d < DDS_COUNT; d++) {
        for (c = 0U; c < DDS_CH_COUNT; c++) {
            g_seq_rt[d][c].active = false;
            g_seq_rt[d][c].dds_idx = d;
            g_seq_rt[d][c].ch_idx = c;
            g_seq_rt[d][c].loop_idx = 0U;
            g_seq_rt[d][c].item_idx = 0U;
            g_seq_rt[d][c].active_ttl = 0U;
            g_seq_rt[d][c].priority_rank = 0U;
            g_seq_rt[d][c].phase = SEQ_RT_IDLE;
            g_seq_rt[d][c].sweep_start_pending = false;
            g_seq_rt[d][c].profile_low_after_update = false;
            g_seq_rt[d][c].first_item_preloaded = false;
            g_seq_rt[d][c].next_item_preloaded = false;
            g_seq_rt[d][c].preloaded_loop_idx = 0U;
            g_seq_rt[d][c].preloaded_item_idx = 0U;
            g_seq_rt[d][c].sequence_clock_started = false;
            g_seq_rt[d][c].loop_deadline_active = false;
            g_seq_rt[d][c].sequence_start_ticks = 0UL;
            g_seq_rt[d][c].loop_duration_ticks = 0ULL;
            g_seq_rt[d][c].remaining_ticks = 0UL;
            g_seq_rt[d][c].hw_current_decihz = 0UL;
            g_seq_rt[d][c].hw_current_ftw = 0UL;
            g_seq_rt[d][c].hw_next_update_deadline_ticks = 0UL;
            g_seq_rt[d][c].hw_deadline_valid = false;
            g_seq_rt[d][c].fast_wait_prepared = false;
            g_seq_rt[d][c].fast_wait_start_ticks = 0UL;

            g_seq_def[d][c].valid = false;
            g_seq_def[d][c].n_loops = 0U;
            g_seq_def[d][c].n_items = 0U;

            for (i = 0U; i < SEQ_MAX_ITEMS; i++) {
                g_seq_def[d][c].items[i].type = 0U;
                g_seq_def[d][c].items[i].step.freq_decihz = 0UL;
                g_seq_def[d][c].items[i].step.duration_us = 0UL;
                g_seq_def[d][c].items[i].step.ftw = 0UL;
                g_seq_def[d][c].items[i].sweep.start_decihz = 0UL;
                g_seq_def[d][c].items[i].sweep.stop_decihz = 0UL;
                g_seq_def[d][c].items[i].sweep.step_decihz = 0UL;
                g_seq_def[d][c].items[i].sweep.tstep_ns = 0UL;
                g_seq_def[d][c].items[i].sweep.duration_ticks = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.start_decihz = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.stop_decihz = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.step_decihz = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.tstep_ticks = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.start_ftw = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.stop_ftw = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.step_ftw = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.fast_flags = 0U;
                g_seq_def[d][c].items[i].hw_sweep.gate_ttl = 0U;
                g_seq_def[d][c].items[i].hw_sweep.phase_base_pow = 0U;
                g_seq_def[d][c].items[i].hw_sweep.phase_peak_pow = 0U;
                g_seq_def[d][c].items[i].hw_sweep.phase_half_ticks = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.phase_delta_word = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.phase_cw1 = 0UL;
                g_seq_def[d][c].items[i].hw_sweep.phase_lsrr = 0UL;
            }

            Seq_SetProfileLow(d,c);
            Seq_DisableLinearSweep_NoUpdate(d,c);
            Seq_SetFTW_NoUpdate(d,c,0UL);
            Seq_WriteRegFast(d,CPOW_ADRESS,0UL,CPOW_NUM_BYTE);
            Seq_WriteRegFast(d,ACR_ADRESS,SEQ_ACR_AMP_ENABLE,ACR_NUM_BYTE);

            dds_update_mask |= (1U << d);
        }
    }

    Seq_IOUpdateMask(dds_update_mask);
    g_seq_any_running = false;
    g_seq_active_mask = 0U;
    g_seq_profile_low_pending_mask = 0U;
}

void Seq_AbortAndReleaseControl(void)
{
    unsigned int d, c;
    unsigned int dds_update_mask = 0U;
    Seq_StopAllHwEngines();
    Seq_StopPhasePulseTimer();

    HWREG(TIMER4_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    g_seq_timer4_active = false;
    g_seq_timer4_force_service = false;
    g_seq_timer4_in_isr = false;
    g_seq_timer4_deadline_valid = false;
    g_seq_timer4_remaining_origin_ticks = 0UL;
    g_seq_timer_armed_ticks = 0UL;
    g_seq_timer_hw_owner_bit = 0U;
    g_seq_last_compensation_ticks = 0UL;

    for (d = 0U; d < DDS_COUNT; d++) {
        for (c = 0U; c < DDS_CH_COUNT; c++) {
            g_seq_rt[d][c].active = false;
            g_seq_rt[d][c].phase = SEQ_RT_IDLE;
            g_seq_rt[d][c].sweep_start_pending = false;
            g_seq_rt[d][c].profile_low_after_update = false;
            g_seq_rt[d][c].first_item_preloaded = false;
            g_seq_rt[d][c].next_item_preloaded = false;
            g_seq_rt[d][c].sequence_clock_started = false;
            g_seq_rt[d][c].loop_deadline_active = false;
            g_seq_rt[d][c].sequence_start_ticks = 0UL;
            g_seq_rt[d][c].loop_duration_ticks = 0ULL;
            g_seq_rt[d][c].remaining_ticks = 0UL;
            g_seq_rt[d][c].hw_current_decihz = 0UL;
            g_seq_rt[d][c].hw_current_ftw = 0UL;
            g_seq_rt[d][c].hw_next_update_deadline_ticks = 0UL;
            g_seq_rt[d][c].hw_deadline_valid = false;
            Seq_SetProfileLow(d,c);
            Seq_DisableLinearSweep_NoUpdate(d,c);
            dds_update_mask |= (1U << d);
        }
    }

    Seq_IOUpdateMask(dds_update_mask);
    g_seq_any_running = false;
    g_seq_active_mask = 0U;
    g_seq_profile_low_pending_mask = 0U;
}

// garde l'ancien nom pour write_immediate()
void Sequence_AbortAndReleaseControl(void)
{
    Seq_AbortAndReleaseControl();
}

static bool Seq_GetFollowingPosition(seq_rt_t *rt,seq_def_t *def,unsigned int *next_loop,unsigned int *next_item)
{
    if ((rt->item_idx + 1U) < def->n_items) {
        *next_loop = rt->loop_idx;
        *next_item = rt->item_idx + 1U;
        return true;
    }

    if ((rt->loop_idx + 1U) < def->n_loops) {
        *next_loop = rt->loop_idx + 1U;
        *next_item = 0U;
        return true;
    }

    return false;
}

static unsigned int Seq_CountActiveChannelsOnDDS(unsigned int d)
{
    unsigned int c;
    unsigned int count = 0U;

    for (c = 0U; c < DDS_CH_COUNT; c++) {
        if (g_seq_rt[d][c].active) count++;
    }

    return count;
}

static bool Seq_IsSinglePreloadedStepToStep(void)
{
    unsigned int mask = g_seq_active_mask & 0x0FFFU;
    int idx;
    unsigned int d;
    unsigned int c;
    seq_rt_t *rt;
    seq_def_t *def;

    if (mask == 0U) return false;
    if ((mask & (mask - 1U)) != 0U) return false;

    idx = Seq_FirstSetBit12(mask);
    if (idx < 0) return false;

    d = (unsigned int)idx / 4U;
    c = (unsigned int)idx % 4U;

    rt = (seq_rt_t *)&g_seq_rt[d][c];
    def = (seq_def_t *)&g_seq_def[d][c];

    if (!rt->active) return false;
    if (rt->phase != SEQ_RT_STEP) return false;
    if (!rt->next_item_preloaded) return false;
    if (rt->preloaded_item_idx >= def->n_items) return false;

    return (def->items[rt->preloaded_item_idx].type == SEQ_ITEM_STEP) ||
           (def->items[rt->preloaded_item_idx].type == SEQ_ITEM_STEP_EX);
}

static bool Seq_PrepareItem_NoUpdate(unsigned int d,unsigned int c,
                                     const seq_item_t *item,seq_rt_t *rt)
{
    if (item->type == SEQ_ITEM_STEP) {
        Seq_WriteRegFast(d,CSR_ADRESS,Seq_CSR_FromChannel(c),CSR_NUM_BYTE);
        Seq_WriteRegFast(d,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);
        Seq_WriteRegFast(d,FTW_ADRESS,item->step.ftw,FTW_NUM_BYTE);
        rt->single_tone_ready = true;
        return true;
    }

    if (item->type == SEQ_ITEM_STEP_EX) {
        Seq_WriteRegFast(d,CSR_ADRESS,Seq_CSR_FromChannel(c),CSR_NUM_BYTE);

        /* V30.4 : CFR n'est réécrit que s'il n'est pas déjà en single tone.
         * Cela couvre le cas sweep -> STEP_EX phase/amplitude sans payer une
         * écriture SSI à chaque palier d'une fonction d'amplitude. */
        if (!rt->single_tone_ready) {
            Seq_WriteRegFast(d,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);
            rt->single_tone_ready = true;
        }

        if (item->step.flags & SEQ_STEPEX_FLAG_FREQ) {
            Seq_WriteRegFast(d,FTW_ADRESS,item->step.ftw,FTW_NUM_BYTE);
        }
        if (item->step.flags & SEQ_STEPEX_FLAG_PHASE) {
            Seq_WriteRegFast(d,CPOW_ADRESS,(unsigned long)item->step.pow,CPOW_NUM_BYTE);
        }
        if (item->step.flags & SEQ_STEPEX_FLAG_AMP) {
            Seq_WriteRegFast(d,ACR_ADRESS,SEQ_ACR_AMP_ENABLE | (unsigned long)item->step.asf,ACR_NUM_BYTE);
        }
        return true;
    }

    if (item->type == SEQ_ITEM_SWEEP) {
        Seq_ConfigLinearSweep_NoStart(d,c,&item->sweep);
        rt->single_tone_ready = false;
        return true;
    }

    if (item->type == SEQ_ITEM_FAST_HW_SWEEP) {
        /* Fast items are configured after the preceding I/O_UPDATE. This
         * prevents a waiting staircase from being latched before its gate. */
        return false;
    }

    if (item->type == SEQ_ITEM_HW_SWEEP) {
        Seq_WriteRegFast(d,CSR_ADRESS,Seq_CSR_FromChannel(c),CSR_NUM_BYTE);
        Seq_WriteRegFast(d,CFR_ADRESS,SEQ_CFR_SINGLE_TONE_SAFE,CFR_NUM_BYTE);
        Seq_WriteRegFast(d,FTW_ADRESS,item->hw_sweep.start_ftw,FTW_NUM_BYTE);
        rt->single_tone_ready = true;
        return true;
    }

    /*
     * Pas encore de look-ahead pour le HW sweep :
     * ses I/O_UPDATE intermédiaires compliquent la sécurité.
     */
    return false;
}

static void Seq_PreloadNextItemsSafe(void)
{
    unsigned int d;
    unsigned int c;
    unsigned long global_earliest_ticks = 0xFFFFFFFFUL;
    bool found = false;

    /*
     * V30.3 : priorité à la prochaine frontière GLOBALE, pas à une frontière
     * différente sur chaque DDS. L'ancienne version pouvait préparer le futur
     * linear sweep de DDS2 alors que DDS1 devait changer moins de 20 µs plus
     * tard. Timer4 était alors armé après avoir déjà manqué cette deadline.
     */
    for (d = 0U; d < DDS_COUNT; d++) {
        if (Seq_DdsBlockedByHw(d)) continue;

        for (c = 0U; c < DDS_CH_COUNT; c++) {
            seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];

            /* Un canal déjà préchargé participe quand même au choix de la
             * deadline globale : sinon un réveil forcé pourrait autoriser le
             * préchargement d'une frontière plus tardive du même DDS et
             * écraser ses registres tamponnés. */
            if (!rt->active) continue;
            if (rt->phase == SEQ_RT_HW_SWEEP) continue;

            if (!found || rt->remaining_ticks < global_earliest_ticks) {
                global_earliest_ticks = rt->remaining_ticks;
                found = true;
            }
        }
    }

    if (!found) return;

    /* Tous les canaux ayant exactement cette même deadline peuvent être
     * préparés : ils seront validés ensemble par le prochain I/O_UPDATE de
     * leur DDS respectif. Les frontières ultérieures attendent le prochain
     * passage dans cette fonction. */
    for (d = 0U; d < DDS_COUNT; d++) {
        if (Seq_DdsBlockedByHw(d)) continue;

        for (c = 0U; c < DDS_CH_COUNT; c++) {
            seq_rt_t *rt = (seq_rt_t *)&g_seq_rt[d][c];
            seq_def_t *def = (seq_def_t *)&g_seq_def[d][c];
            unsigned int next_loop;
            unsigned int next_item;
            seq_item_t *next;

            if (!rt->active || rt->next_item_preloaded) continue;
            if (rt->phase == SEQ_RT_HW_SWEEP) continue;
            /* V30.5 : ne jamais préparer l'item suivant pendant la
             * pré-frontière d'un sweep descendant. Les registres tamponnés
             * pourraient être validés par l'I/O_UPDATE d'un autre canal du
             * même DDS avant la fin du sweep courant. */
            if (rt->phase == SEQ_RT_SWEEP_PRELOAD_DESC) continue;
            if (rt->remaining_ticks != global_earliest_ticks) continue;
            if (!Seq_GetFollowingPosition(rt,def,&next_loop,&next_item)) continue;

            next = (seq_item_t *)&def->items[next_item];

            /* Un second HW sweep sur le même DDS reste interdit. */
            if (next->type == SEQ_ITEM_HW_SWEEP && g_seq_hw_engine[d].active) continue;

            if (Seq_PrepareItem_NoUpdate(d,c,next,rt)) {
                rt->next_item_preloaded = true;
                rt->preloaded_loop_idx = next_loop;
                rt->preloaded_item_idx = next_item;
            }
        }
    }
}

//================== FIN AJOUT =========================

//*****************************************************************************
//
// Timer functions for uart parameters sweep
//
//*****************************************************************************
void Timer2A_IntHandler(void)
{
    // Clear interrupt & Flag
    //TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    // Plus rapide que TimerIntClear()
    HWREG(TIMER2_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;

    // Indiquer qu'une interruption de timer a eu lieu.
    HWREGBITW(&g_ulTimerFlags, FLAG_TIMER_TICK) = 1;

    if (!g_sweep_running) {
        // D sactivation directe du bit TAEN (Timer A Enable)
        HWREG(TIMER2_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN);
        //TimerDisable(TIMER2_BASE, TIMER_A); // D sactive Timer A de Timer 2
        return;
    }

    // Calculer le prochain FTW
    bool sweep_finished = false;

    if (g_is_rising) {
        if (g_current_ftw + g_ftw_step >= g_ftw_end) {
            g_current_ftw = g_ftw_end;
            sweep_finished = true;
        } else {
            g_current_ftw += g_ftw_step;
        }
    } else { // Balayage descendant

        if (g_current_ftw <= g_ftw_end || g_current_ftw - g_ftw_step <= g_ftw_end) {
            g_current_ftw = g_ftw_end;
            sweep_finished = true;
        } else {
            g_current_ftw -= g_ftw_step;
        }
    }
    switch(NbDDS_UART)
        {
            case 1:
                SetFrequencyTuningWord0_DDS1_burst(g_current_ftw, g_channel);
                break;
            case 2:
                SetFrequencyTuningWord0_DDS2_burst(g_current_ftw, g_channel);
                break;
            case 3:
                SetFrequencyTuningWord0_DDS3_burst(g_current_ftw, g_channel);
                break;
        }

    // Gestion de fin(Acc s direct registre)
    if (sweep_finished) {
        g_sweep_running = false;
        HWREG(TIMER2_BASE + TIMER_O_CTL) &= ~(TIMER_CTL_TAEN);
        TimerDisable(TIMER2_BASE, TIMER_A);
    }   
}
//Configuration de la p riode uart com
void Init_Timer2A_Sweep(double time_step_s)
{
    uint32_t ui32Period;

    // P riode en cycles = (temps en secondes * Fr quence Horloge)
    // SYSCLKMICRO = 120MHz
    ui32Period = (uint32_t)(time_step_s * SYSCLKMICRO);

    //S CURIT  CRITIQUE (Anti-Crash)
    // L'ISR prend environ 900ns   s'ex cuter.
    // Si on descend sous 360 cycles (3us), le CPU sature
    if (ui32Period < 360)
    {
        ui32Period = 360; // On force le plancher   3us
    }

    // D sactiver le Timer A pour la configuration
    TimerDisable(TIMER2_BASE, TIMER_A);

    // Charger la valeur dans le Timer A de Timer 2 (en mode 32 bits)
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period);

    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    // Le timer d marr  ailleurs
    // TimerEnable(TIMER2_BASE, TIMER_A);
}
//Configuration de la p riode ethernet com
void Init_Timer1A_Sweep(double time_step_s)
{
    uint32_t ui32Period;

    // P riode en cycles = (temps en secondes * Fr quence Horloge)
    // SYSCLKMICRO = 120MHz
    ui32Period = (uint32_t)(time_step_s * SYSCLKMICRO);

    //S CURIT  CRITIQUE (Anti-Crash)
    // L'ISR prend environ 900ns   s'ex cuter.
    // Si on descend sous 360 cycles (3us), le CPU sature
    if (ui32Period < 360)
    {
        ui32Period = 360; // On force le plancher   3us
    }

    // D sactiver le Timer A pour la configuration
    TimerDisable(TIMER1_BASE, TIMER_A);

    // Charger la valeur dans le Timer A de Timer 2 (en mode 32 bits)
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period);

    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Le timer d marr  ailleurs
    // TimerEnable(TIMER2_BASE, TIMER_A);
}
//Configuration initiale du Timer 1
void initTimer1()
{
    // Activer l'horloge du p riph rique Timer 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    // Attendre que le p riph rique soit pr t
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)) {}

    // Configurer le Timer 1 en mode 32-bit p riodique
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC); // 32 bits Timer A (utilis  comme un seul timer)

    // D sactiver les interruptions inutiles de Timer 1 (B)
    TimerIntDisable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);

    // Activer l'interruption pour Timer 1 Subtimer A (INT_TIMER2A)
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Enregistrer la fonction d'interruption
    TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1A_IntHandler);
}
//Configuration initiale du Timer 2
void initTimer2()
{
    // Activer l'horloge du p riph rique Timer 2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    // Attendre que le p riph rique soit pr t
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER2)) {}

    // Configurer le Timer 2 en mode 32-bit p riodique
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC); // 32 bits Timer A

    // D sactiver les interruptions inutiles de Timer 2 (B)
    TimerIntDisable(TIMER2_BASE, TIMER_TIMB_TIMEOUT);

    // Activer l'interruption pour Timer 2 Subtimer A (INT_TIMER2A)
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    // Enregistrer la fonction d'interruption
    TimerIntRegister(TIMER2_BASE, TIMER_A, Timer2A_IntHandler);
}
void initTimer3()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3)) {}
    TimerConfigure(TIMER3_BASE,TIMER_CFG_ONE_SHOT);
    TimerIntDisable(TIMER3_BASE,TIMER_TIMB_TIMEOUT);
    IntEnable(INT_TIMER3A);
    TimerIntEnable(TIMER3_BASE,TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER3_BASE,TIMER_A,Timer3A_IntHandler);
}

void initTimer4()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4)) {}
    TimerConfigure(TIMER4_BASE, TIMER_CFG_ONE_SHOT);
    TimerIntDisable(TIMER4_BASE, TIMER_TIMB_TIMEOUT);
    IntEnable(INT_TIMER4A);
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER4_BASE, TIMER_A, Timer4A_IntHandler);
}
void initTimer5()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER5)) {}
    TimerConfigure(TIMER5_BASE,TIMER_CFG_PERIODIC);
    TimerIntDisable(TIMER5_BASE,TIMER_TIMB_TIMEOUT);
    IntEnable(INT_TIMER5A);
    TimerIntEnable(TIMER5_BASE,TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER5_BASE,TIMER_A,Timer5A_IntHandler);
}

void initTimer6()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER6);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER6)) {}
    TimerConfigure(TIMER6_BASE,TIMER_CFG_PERIODIC);
    TimerIntDisable(TIMER6_BASE,TIMER_TIMB_TIMEOUT);
    IntEnable(INT_TIMER6A);
    TimerIntEnable(TIMER6_BASE,TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER6_BASE,TIMER_A,Timer6A_IntHandler);
}

void initTimer7()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER7);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER7)) {}
    TimerConfigure(TIMER7_BASE,TIMER_CFG_PERIODIC);
    TimerIntDisable(TIMER7_BASE,TIMER_TIMB_TIMEOUT);
    IntEnable(INT_TIMER7A);
    TimerIntEnable(TIMER7_BASE,TIMER_TIMA_TIMEOUT);
    TimerIntRegister(TIMER7_BASE,TIMER_A,Timer7A_IntHandler);
}

static void Seq_InitCycleCounter(void)
{
    SEQ_DEMCR |= SEQ_DEMCR_TRCENA;
    SEQ_DWT_CYCCNT = 0UL;
    SEQ_DWT_CTRL |= SEQ_DWT_CYCCNTENA;
}


//*****************************************************************************
//
//  Flash/load for DDS values AD9959 by ethernet
//
//*****************************************************************************
bool simpleflashMemoryAD9959DDS_ethernet_frequency(unsigned int ChannelDDS_UART)
{
    uint32_t FLASH_memory[2];
    if (CURRENT_ChannelDDS > 3U) return false;

    FLASH_memory[0] = (uint32_t)frequencyValueAD9959DDS[CURRENT_ChannelDDS];
    FLASH_memory[1] = (uint32_t)amplValueAD9959DDS[CURRENT_ChannelDDS];

    if( CURRENT_DDS == 1)
       {

       if (CURRENT_ChannelDDS == 0)
        {
            FlashErase(SIMPLEFLASHVALUESDDS1);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS1, sizeof(FLASH_memory));
            return true;
        }
       if (CURRENT_ChannelDDS == 1)
        {
            FlashErase(SIMPLEFLASHVALUESDDS2);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS2, sizeof(FLASH_memory));
            return true;
        }
       if (CURRENT_ChannelDDS == 2)
        {
            FlashErase(SIMPLEFLASHVALUESDDS3);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS3, sizeof(FLASH_memory));
            return true;
        }
       if (CURRENT_ChannelDDS == 3)
        {
            FlashErase(SIMPLEFLASHVALUESDDS4);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS4, sizeof(FLASH_memory));
            return true;
        }
       else
       {
           return false;
       }
       }
    else if( CURRENT_DDS == 2)
    {
          if (CURRENT_ChannelDDS == 0)
           {
               FlashErase(SIMPLEFLASHVALUESDDS5);
               FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS5, sizeof(FLASH_memory));
               return true;
           }
          if (CURRENT_ChannelDDS == 1)
           {
               FlashErase(SIMPLEFLASHVALUESDDS6);
               FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS6, sizeof(FLASH_memory));
               return true;
           }
          if (CURRENT_ChannelDDS == 2)
           {
               FlashErase(SIMPLEFLASHVALUESDDS7);
               FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS7, sizeof(FLASH_memory));
               return true;
           }
          if (CURRENT_ChannelDDS == 3)
           {
               FlashErase(SIMPLEFLASHVALUESDDS8);
               FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS8, sizeof(FLASH_memory));
               return true;
           }
          else
          {
              return false;
          }

     }
    else if( CURRENT_DDS == 3)
      {
               if (CURRENT_ChannelDDS == 0)
                {
                    FlashErase(SIMPLEFLASHVALUESDDS9);
                    FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS9, sizeof(FLASH_memory));
                    return true;
                }
               if (CURRENT_ChannelDDS == 1)
                {
                    FlashErase(SIMPLEFLASHVALUESDDS10);
                    FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS10, sizeof(FLASH_memory));
                    return true;
                }
               if (CURRENT_ChannelDDS == 2)
                {
                    FlashErase(SIMPLEFLASHVALUESDDS11);
                    FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS11, sizeof(FLASH_memory));
                    return true;
                }
               if (CURRENT_ChannelDDS == 3)
                {
                    FlashErase(SIMPLEFLASHVALUESDDS12);
                    FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS12, sizeof(FLASH_memory));
                    return true;
                }
               else
               {
                return false;
               }

       }
       else
          {
              return false;
          }
}
//*************mettre en m moire DDS1*************///
bool simpleflashMemoryAD9959DDS1(unsigned int ChannelDDS_UART)
{
    if (ChannelDDS_UART > 3U) return false;

    uint32_t FLASH_memory[3];
    FLASH_memory[0] = (uint32_t)frequencyValueAD9959DDS[ChannelDDS_UART];
    FLASH_memory[1] = (uint32_t)amplValueAD9959DDS[ChannelDDS_UART];
    FLASH_memory[2] = (uint32_t)powValueAD9959DDS[ChannelDDS_UART];

       if (ChannelDDS_UART == 0)
        {
            FlashErase(SIMPLEFLASHVALUESDDS1);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS1, sizeof(FLASH_memory));
            return true;
        }
       if (ChannelDDS_UART == 1)
        {
            FlashErase(SIMPLEFLASHVALUESDDS2);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS2, sizeof(FLASH_memory));
            return true;
        }
       if (ChannelDDS_UART == 2)
        {
            FlashErase(SIMPLEFLASHVALUESDDS3);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS3, sizeof(FLASH_memory));
            return true;
        }
       if (ChannelDDS_UART == 3)
        {
            FlashErase(SIMPLEFLASHVALUESDDS4);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS4, sizeof(FLASH_memory));
            return true;
        }
       else
       {
           return false;
       }
}
//*************mettre en m moire DDS2*************///
bool simpleflashMemoryAD9959DDS2(unsigned int ChannelDDS_UART)
{
    if (ChannelDDS_UART > 3U) return false;

    uint32_t FLASH_memory[3];
    FLASH_memory[0] = (uint32_t)frequencyValueAD9959DDS[ChannelDDS_UART];
    FLASH_memory[1] = (uint32_t)amplValueAD9959DDS[ChannelDDS_UART];
    FLASH_memory[2] = (uint32_t)powValueAD9959DDS[ChannelDDS_UART];

       if (ChannelDDS_UART == 0)
        {
            FlashErase(SIMPLEFLASHVALUESDDS5);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS5, sizeof(FLASH_memory));
            return true;
        }
       if (ChannelDDS_UART == 1)
        {
            FlashErase(SIMPLEFLASHVALUESDDS6);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS6, sizeof(FLASH_memory));
            return true;
        }
       if (ChannelDDS_UART == 2)
        {
            FlashErase(SIMPLEFLASHVALUESDDS7);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS7, sizeof(FLASH_memory));
            return true;
        }
       if (ChannelDDS_UART == 3)
        {
            FlashErase(SIMPLEFLASHVALUESDDS8);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS8, sizeof(FLASH_memory));
            return true;
        }
       else
       {
           return false;
       }
}
//*************mettre en m moire DDS3*************///
bool simpleflashMemoryAD9959DDS3(unsigned int ChannelDDS_UART)
{
    if (ChannelDDS_UART > 3U) return false;

    uint32_t FLASH_memory[3];
    FLASH_memory[0] = (uint32_t)frequencyValueAD9959DDS[ChannelDDS_UART];
    FLASH_memory[1] = (uint32_t)amplValueAD9959DDS[ChannelDDS_UART];
    FLASH_memory[2] = (uint32_t)powValueAD9959DDS[ChannelDDS_UART];

       if (ChannelDDS_UART == 0)
        {
            FlashErase(SIMPLEFLASHVALUESDDS9);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS9, sizeof(FLASH_memory));
            return true;
        }
       if (ChannelDDS_UART == 1)
        {
            FlashErase(SIMPLEFLASHVALUESDDS10);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS10, sizeof(FLASH_memory));
            return true;
        }
       if (ChannelDDS_UART == 2)
        {
            FlashErase(SIMPLEFLASHVALUESDDS11);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS11, sizeof(FLASH_memory));
            return true;
        }
       if (ChannelDDS_UART == 3)
        {
            FlashErase(SIMPLEFLASHVALUESDDS12);
            FlashProgram(FLASH_memory, SIMPLEFLASHVALUESDDS12, sizeof(FLASH_memory));
            return true;
        }
       else
       {
           return false;
       }
}
// ****************** for ethernet********************///
bool flashMemoryAD9959DDS(unsigned int DDSnb)
{
    uint32_t FLASH_memory[2];
    FLASH_memory[0]  = (uint32_t)frequencyValueAD9959DDS[DDSnb-1];
    FLASH_memory[1]  = (uint32_t)amplValueAD9959DDS[DDSnb-1];
    //FLASH_memory[2] = (uint32_t)powValueAD9959DDS[DDSnb-1];

    switch(DDSnb)
    {
    case 1:
        FlashErase(FLASHVALUESDDS1);
        FlashProgram(FLASH_memory, FLASHVALUESDDS1, sizeof(FLASH_memory));
        return true;
    case 2:
        FlashErase(FLASHVALUESDDS2);
        FlashProgram(FLASH_memory, FLASHVALUESDDS2, sizeof(FLASH_memory));
        return true;
    case 3:
        FlashErase(FLASHVALUESDDS3);
        FlashProgram(FLASH_memory, FLASHVALUESDDS3, sizeof(FLASH_memory));
        return true;

    default:
        return false;
    }
}
// ****************** for ethernet********************///
bool loadValuesFromFlashMemoryAD9959DDS(unsigned int DDSnb)
{
    int auxInt;
    long auxLong;
    int addrFlash;

    if (DDSnb == 1)
    {
        addrFlash = FLASHVALUESDDS1;
    }
    else if (DDSnb == 2)
    {
        addrFlash = FLASHVALUESDDS2;
    }
    else if (DDSnb == 3)
    {
        addrFlash = FLASHVALUESDDS3;
    }
    else if (DDSnb == 4)
    {
        addrFlash = FLASHVALUESDDS4;
    }
    else
    {
        return false;
    }

    auxLong = *(long*)(addrFlash);
    auxInt = *(int *)(addrFlash + 1*4); // decalage de 4 octets

    SetFrequencyTuningWord0_DDS1((unsigned long)auxLong, DDSnb);
    SetamplValueAD9959DDS1((unsigned short)auxInt, DDSnb);
    //SetPOW_ValueAD9959DDS1((unsigned short)auxInt, DDSnb);
    return true;
}
// ****************** load memory values DDS1********************///
bool simpleLoadValuesFromFlashMemoryAD9959DDS1(unsigned int ChannelDDS_UART)
{
    int auxInt;
    int auxInt2;
    long auxLong;
    int addrFlash;


    if (ChannelDDS_UART == 0)
    {
        addrFlash = SIMPLEFLASHVALUESDDS1;
    }
    else if (ChannelDDS_UART == 1)
    {
        addrFlash = SIMPLEFLASHVALUESDDS2;
    }
    else if (ChannelDDS_UART == 2)
    {
        addrFlash = SIMPLEFLASHVALUESDDS3;
    }
    else if (ChannelDDS_UART == 3)
    {
        addrFlash = SIMPLEFLASHVALUESDDS4;
    }
    else
    {
        return false;
    }

    auxLong = *(long*)(addrFlash);
    auxInt = *(int *)(addrFlash + 1*4);  // decalage de 4 octets
    auxInt2 = *(int *)(addrFlash + 2*4); // decalage de 8 octets
    //auxInt = *(int *)(addrFlash + 2*4);  // decalage de 8 octets
    //auxInt2 = *(int *)(addrFlash + 3*4); // decalage de 12 octets ( 4 me  lement en m moire)

    if (auxInt != 0xFFFFFFFF || auxInt2 != 0xFFFFFFFF)
    {
        SetFrequencyTuningWord0_DDS1((unsigned long)auxLong, ChannelDDS_UART);
        SetamplValueAD9959DDS1((unsigned short)auxInt, ChannelDDS_UART);
        SetPOW_ValueAD9959DDS1((unsigned short)auxInt2, ChannelDDS_UART);
        return true;
    }
    else
    {
       return false;
    }
}
// ****************** load memory values DDS2********************///
bool simpleLoadValuesFromFlashMemoryAD9959DDS2(unsigned int ChannelDDS_UART)
{
    int auxInt;
    int auxInt2;
    long auxLong;
    int addrFlash;


    if (ChannelDDS_UART == 0)
    {
        addrFlash = SIMPLEFLASHVALUESDDS5;
    }
    else if (ChannelDDS_UART == 1)
    {
        addrFlash = SIMPLEFLASHVALUESDDS6;
    }
    else if (ChannelDDS_UART == 2)
    {
        addrFlash = SIMPLEFLASHVALUESDDS7;
    }
    else if (ChannelDDS_UART == 3)
    {
        addrFlash = SIMPLEFLASHVALUESDDS8;
    }
    else
    {
        return false;
    }

    auxLong = *(long*)(addrFlash);
    auxInt = *(int *)(addrFlash + 1*4);  // decalage de 4 octets
    auxInt2 = *(int *)(addrFlash + 2*4); // decalage de 8 octets

    if (auxInt != 0xFFFFFFFF || auxInt2 != 0xFFFFFFFF)
       {
           SetFrequencyTuningWord0_DDS2((unsigned long)auxLong, ChannelDDS_UART);
           SetamplValueAD9959DDS2((unsigned short)auxInt, ChannelDDS_UART);
           SetPOW_ValueAD9959DDS2((unsigned short)auxInt2, ChannelDDS_UART);
           return true;
       }
    else
    {
        return false;
    }
}
// ****************** load memory values DDS3********************///
bool simpleLoadValuesFromFlashMemoryAD9959DDS3(unsigned int ChannelDDS_UART)
{
    int auxInt;
    int auxInt2;
    long auxLong;
    int addrFlash;


    if (ChannelDDS_UART == 0)
    {
        addrFlash = SIMPLEFLASHVALUESDDS9;
    }
    else if (ChannelDDS_UART == 1)
    {
        addrFlash = SIMPLEFLASHVALUESDDS10;
    }
    else if (ChannelDDS_UART == 2)
    {
        addrFlash = SIMPLEFLASHVALUESDDS11;
    }
    else if (ChannelDDS_UART == 3)
    {
        addrFlash = SIMPLEFLASHVALUESDDS12;
    }
    else
    {
        return false;
    }

    auxLong = *(long*)(addrFlash);
    auxInt = *(int *)(addrFlash + 1*4);  // decalage de 4 octets
    auxInt2 = *(int *)(addrFlash + 2*4); // decalage de 8 octets

    if (auxInt != 0xFFFFFFFF || auxInt2 != 0xFFFFFFFF)
       {
       SetFrequencyTuningWord0_DDS3((unsigned long)auxLong, ChannelDDS_UART);
       SetamplValueAD9959DDS3((unsigned short)auxInt, ChannelDDS_UART);
       SetPOW_ValueAD9959DDS3((unsigned short)auxInt2, ChannelDDS_UART);
       return true;
       }
    else
    {
        return false;
    }
}
//*****************************************************************************
//
//  SPI and SSI0 SERIAL PORT
//
//*****************************************************************************
void init_SPI0_mode_quad_bit(void)
{
           // Enable Peripheral SSI0
                  //if Quad SPI
             //***********************//
       //***********************//
       //GPIO_PA4_SSI0XDAT0 <= BIT 4 BIT 0   only need 2 CLK cycle to send 8bits.
       //GPIO_PA5_SSI0XDAT1 <= BIT 5 BIT 1   only need 2 CLK cycle to send 8bits.
       //GPIO_PA6_SSI0XDAT2 <= BIT 6 BIT 2   only need 2 CLK cycle to send 8bits.
       //GPIO_PA7_SSI0XDAT3 <= BIT 7 BIT 3   only need 2 CLK cycle to send 8bits.
       //************************//

        //*************************************//
        //Polarity Phase       Mode
        //   0       0   SSI_FRF_MOTO_MODE_0
        //   0       1   SSI_FRF_MOTO_MODE_1
        //   1       0   SSI_FRF_MOTO_MODE_2
        //   1       1   SSI_FRF_MOTO_MODE_3
        //*************************************//

    //************************//
    // SSI1 is used with the following GPIO Pin Mapping
    // SSI0CLK   : PA2
    // SSI0FSS   : PA3
    // SSI0XDAT0 : PA4
    // SSI0XDAT1 : PA5
    // SSI0XDAT2 : PA6
    // SSI0XDAT3 : PA7
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, A5, A6, A7

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
    GPIOPinConfigure(GPIO_PA6_SSI0XDAT2);
    GPIOPinConfigure(GPIO_PA7_SSI0XDAT3);

    /* Configure pad settings */
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    //GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_4|GPIO_PIN_5, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6|GPIO_PIN_7, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, 120000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SSICLKDDS, 8);
    // Enable Quad-SSI mode for both SSI0.
    SSIAdvModeSet(SSI0_BASE, SSI_ADV_MODE_QUAD_WRITE);
    SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);
    //Hold the Fss pin low during transfers.
    //SSIAdvFrameHoldEnable(SSI0_BASE);
    //SSIAdvModeSet(SSI0_BASE, SSI_ADV_MODE_WRITE);
    //SSIAdvModeSet(SSI0_BASE,SSI_ADV_MODE_BI_WRITE);
    //
    // Enable the SSI0 module.
    //
    SSI0_CR1_R = 0x00000080;
    SSIEnable(SSI0_BASE);
}
void init_SPI0_mode_single_bit(void)
{
    // Enable Peripheral SSI0
    //if Single bit serial mode SPI
     //***********************//
    // SSI0 is used with the following GPIO Pin Mapping
    // SSI1CLK   : PA2 => SCLK
    // SSI1FSS   : PA3 => SYNC
    // SSI0XDAT0 : PA4 => SSI0Tx
    // SSI0XDAT1 : PA5 => SSI0Rx
    //************************//

    //*************************************//
    //Polarity Phase       Mode
    //   0       0   SSI_FRF_MOTO_MODE_0
    //   0       1   SSI_FRF_MOTO_MODE_1
    //   1       0   SSI_FRF_MOTO_MODE_2
    //   1       1   SSI_FRF_MOTO_MODE_3
    //*************************************//

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlDelay(3);


    GPIOPinConfigure(GPIO_PA2_SSI0CLK);     // (SCLK)
    //GPIOPinConfigure(GPIO_PA3_SSI0FSS);     // (SYNC)
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);   // (SSI0Tx -> )
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);   // (SSI0Rx)


    // Configure the GPIO settings for the SSI pins. This function also gives control of these pins to the SSI hardware.
    //GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5);
    GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5);
    SSIConfigSetExpClk(SSI0_BASE, SYSCLKMICRO, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SSICLKDDS, 8); // SSI_FRF_MOTO_MODE_0 , 2 and 3 seem to work. SCLK is 10MHz max for AD9852
    //Hold the Fss pin low during transfers.
    SSIAdvFrameHoldEnable(SSI0_BASE);
    //
    // Enable the SSI0 module.
    //
    SSI0_CR1_R = 0x00000000;
    //SSIAdvModeSet(SSI0_BASE, SSI_ADV_MODE_READ_WRITE);
    SSIEnable(SSI0_BASE); // Enable the SSI
}
void init_SPI1_mode_quad_bit(void)
{
           // Enable Peripheral SSI1
                  //if Quad SPI
             //***********************//
      //GPIO_PE4_SSI1XDAT0 <= BIT 4 BIT 0   only need 2 CLK cycle to send 8bits.
      //GPIO_PE5_SSI1XDAT1 <= BIT 5 BIT 1   only need 2 CLK cycle to send 8bits.
      //GPIO_PD4_SSI1XDAT2 <= BIT 6 BIT 2   only need 2 CLK cycle to send 8bits.
      //GPIO_PD5_SSI1XDAT3 <= BIT 7 BIT 3   only need 2 CLK cycle to send 8bits.

        //*************************************//
        //Polarity Phase       Mode
        //   0       0   SSI_FRF_MOTO_MODE_0
        //   0       1   SSI_FRF_MOTO_MODE_1
        //   1       0   SSI_FRF_MOTO_MODE_2
        //   1       1   SSI_FRF_MOTO_MODE_3
        //*************************************//

    //************************//
    // SSI1 is used with the following GPIO Pin Mapping
    // SSI1CLK   : PB5
    // SSI1FSS   : PB4
    // SSI1XDAT0 : PE4
    // SSI1XDAT1 : PE5
    // SSI1XDAT2 : PD4
    // SSI1XDAT3 : PD5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    // Configure the pin muxing for SSI1 functions on port B4, B5, E4, E5,D4, D5

    GPIOPinConfigure(GPIO_PB5_SSI1CLK);
    GPIOPinConfigure(GPIO_PB4_SSI1FSS);
    GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
    GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);
    GPIOPinConfigure(GPIO_PD4_SSI1XDAT2);
    GPIOPinConfigure(GPIO_PD5_SSI1XDAT3);

    /* Configure pad settings */
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_5|GPIO_PIN_4, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_5|GPIO_PIN_4, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.
    GPIOPinTypeSSI(GPIO_PORTB_BASE,GPIO_PIN_5|GPIO_PIN_4);
    GPIOPinTypeSSI(GPIO_PORTE_BASE,GPIO_PIN_5|GPIO_PIN_4);
    GPIOPinTypeSSI(GPIO_PORTD_BASE,GPIO_PIN_5|GPIO_PIN_4);
    SSIConfigSetExpClk(SSI1_BASE, 120000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SSICLKDDS, 8);
    // Enable Quad-SSI mode for both SSI0.
    SSIAdvModeSet(SSI1_BASE, SSI_ADV_MODE_QUAD_WRITE);
    SSIClockSourceSet(SSI1_BASE, SSI_CLOCK_SYSTEM);
    //
    // Enable the SSI1 module.
    //
    SSI1_CR1_R = 0x00000080;
    SSIEnable(SSI1_BASE);
}
void init_SPI1_mode_single_bit(void)
{
    // Enable Peripheral SSI1
    //if Single bit serial mode SPI
     //***********************//
    // SSI1 is used with the following GPIO Pin Mapping
    // SSI1CLK   : PB5
    // SSI1FSS   : PB4
    // SSI1XDAT0 : PE4
    // SSI1XDAT1 : PE5

    //*************************************//
    //Polarity Phase       Mode
    //   0       0   SSI_FRF_MOTO_MODE_0
    //   0       1   SSI_FRF_MOTO_MODE_1
    //   1       0   SSI_FRF_MOTO_MODE_2
    //   1       1   SSI_FRF_MOTO_MODE_3
    //*************************************//

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    // Configure the pin muxing for SSI1 functions on port B4, B5, E4,

    GPIOPinConfigure(GPIO_PB5_SSI1CLK);    // (SCLK)
    //GPIOPinConfigure(GPIO_PB4_SSI1FSS);    // (SYNC)
    GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);  // (SSI0Tx -> )
    GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);  // (SSI0Rx)

    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.
    GPIOPinTypeSSI(GPIO_PORTB_BASE,GPIO_PIN_5);
    //GPIOPinTypeSSI(GPIO_PORTB_BASE,GPIO_PIN_5);
    GPIOPinTypeSSI(GPIO_PORTE_BASE,GPIO_PIN_4|GPIO_PIN_5);
    SSIConfigSetExpClk(SSI1_BASE, 120000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SSICLKDDS, 8);
    //Hold the Fss pin low during transfers.
    SSIAdvFrameHoldEnable(SSI1_BASE);
    //
    // Enable the SSI1 module.
    //
    SSI1_CR1_R = 0x00000000;
    SSIEnable(SSI1_BASE);


}
void init_SPI2_mode_quad_bit(void)
{
           // Enable Peripheral SSI2
                  //if Quad SPI
             //***********************//
      //GPIO_PD1_SSI2XDAT0 <= BIT 4 BIT 0   only need 2 CLK cycle to send 8bits.
      //GPIO_PD0_SSI2XDAT1 <= BIT 5 BIT 1   only need 2 CLK cycle to send 8bits.
      //GPIO_PD7_SSI2XDAT2 <= BIT 6 BIT 2   only need 2 CLK cycle to send 8bits.
      //GPIO_PD6_SSI2XDAT3 <= BIT 7 BIT 3   only need 2 CLK cycle to send 8bits.

        //*************************************//
        //Polarity Phase       Mode
        //   0       0   SSI_FRF_MOTO_MODE_0
        //   0       1   SSI_FRF_MOTO_MODE_1
        //   1       0   SSI_FRF_MOTO_MODE_2
        //   1       1   SSI_FRF_MOTO_MODE_3
        //*************************************//

    //************************//
    // SSI2 is used with the following GPIO Pin Mapping
    // SSI1CLK   : PD3
    // SSI1FSS   : PD2
    // SSI1XDAT0 : PD1
    // SSI1XDAT1 : PD0
    // SSI1XDAT2 : PD7
    // SSI1XDAT3 : PD6
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    // Configure the pin muxing for SSI1 functions on port B4, B5, E4, E5,D4, D5

    GPIOPinConfigure(GPIO_PD3_SSI2CLK);
    GPIOPinConfigure(GPIO_PD2_SSI2FSS);
    GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
    GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
    GPIOPinConfigure(GPIO_PD7_SSI2XDAT2);
    GPIOPinConfigure(GPIO_PD6_SSI2XDAT3);

    /* Configure pad settings */
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.
    GPIOPinTypeSSI(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7);
    SSIConfigSetExpClk(SSI2_BASE, 120000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SSICLKDDS, 8);
    // Hold the Fss pin low during transfers.
    SSIAdvModeSet(SSI2_BASE, SSI_ADV_MODE_QUAD_WRITE);
    SSIClockSourceSet(SSI2_BASE, SSI_CLOCK_SYSTEM);
    //
    // Enable the SSI1 module.
    //
    SSI2_CR1_R = 0x00000080;
    SSIEnable(SSI2_BASE);
}
void init_SPI2_mode_single_bit(void)
{
    // Enable Peripheral SSI2
    //if Single bit serial mode SPI
     //***********************//
    // SSI1 is used with the following GPIO Pin Mapping
    // SSI1CLK   : PD3
    // SSI1FSS   : PD2
    // SSI1XDAT0 : PD1
    // SSI1XDAT1 : PD0

    //*************************************//
    //Polarity Phase       Mode
    //   0       0   SSI_FRF_MOTO_MODE_0
    //   0       1   SSI_FRF_MOTO_MODE_1
    //   1       0   SSI_FRF_MOTO_MODE_2
    //   1       1   SSI_FRF_MOTO_MODE_3
    //*************************************//

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);


    GPIOPinConfigure(GPIO_PD3_SSI2CLK);    // (SCLK)
    //GPIOPinConfigure(GPIO_PD2_SSI2FSS);    // (SYNC)
    GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);  // (SSI0Tx -> )
    GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);  // (SSI0Rx)

    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.
    GPIOPinTypeSSI(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3);
    //GPIOPinTypeSSI(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3);
    SSIConfigSetExpClk(SSI2_BASE, 120000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SSICLKDDS, 8);
    //Hold the Fss pin low during transfers.
    SSIAdvFrameHoldEnable(SSI2_BASE);
    //
    // Enable the SSI1 module.
    //
    SSI2_CR1_R = 0x00000000;
    SSIEnable(SSI2_BASE);
}
void ssi0PutData( int instruction,long data,int num_byte)
{
        int i;
        // Envoi de l'instruction
        while (!(SSI0_SR_R & SSI_SR_TNF))
        {
                  ; // Wait
        }
        // Send the instruction
        SSI0_DR_R = instruction;

        // Envoi des donn es (Boucle classique)
        for (i = 0; i < num_byte; i++)
           {
              // Wait until the Transmit FIFO is Not Full (TNF)
              while (!(SSI0_SR_R & SSI_SR_TNF))
              {
              ;// Wait
              }
              // D calage pour prendre l'octet de poids fort en premier
              SSI0_DR_R = (data >> ((num_byte-1-i) * 8));
           }
            // On attend que le SSI ait fini de travailler avant de rendre la main.
              while (SSI0_SR_R & SSI_SR_BSY);

}
// Version optimis e pour QSPI 60MHz FIFO frequency word
void ssi0PutData_Burst(int instruction, long data) // Supposons 4 octets fixes frequency word
{
    while (!(SSI0_SR_R & SSI_SR_TNF));
    //  criture en rafale
    SSI0_DR_R = instruction;   // L'instruction part 33ns
    SSI0_DR_R = (data >> 24) & 0xFF;
    SSI0_DR_R = (data >> 16) & 0xFF;
    SSI0_DR_R = (data >> 8) & 0xFF;
    SSI0_DR_R = data & 0xFF;
    //   60MHz, la transmission sera finie ~100ns apr s la derni re ligne de code ci-dessus.
    //while (SSI0_SR_R & SSI_SR_BSY);
}
// Version optimis e pour QSPI FAST 60MHz FIFO frequency word
void ssi0PutData_QSPI_Fast(uint8_t instruction, uint32_t data)
{

    // Au lieu de faire 4 op rations de d calage (>>), on inverse tout le bloc
    // d'un coup avec une instruction CPU native.
    // data (0xAABBCCDD) devient swapped (0xDDCCBBAA)
    uint32_t swapped = __builtin_bswap32(data);

    // Pointeur volatile rapide vers le registre FIFO
    volatile uint32_t * const p_fifo = &SSI0_DR_R;

    // 2. V rification FIFO (Optimis e)
    // On v rifie une seule fois qu'il y a de la place.
    // Le FIFO fait g n ralement 8 mots de profondeur, on en  crit 5.
    while(!(SSI0_SR_R & SSI_SR_TNF));

    // 3.  criture en RAFALE (Burst)

    *p_fifo = instruction;      // Envoi Instruction

    // On  crit les octets directement depuis la variable invers e.
    *p_fifo = swapped;          // Prend les 8 bits de poids faible (anciennement MSB)
    *p_fifo = swapped >> 8;     // Le compilateur optimise  a en acc s registre direct (UXTB)
    *p_fifo = swapped >> 16;
    *p_fifo = swapped >> 24;
}
void ssi1PutData( int instruction,long data,int num_byte)
{
    int i;
    // Wait until the Transmit FIFO is Not Full (TNF)
        while (!(SSI1_SR_R & SSI_SR_TNF))
        {
           ; // Wait
        }
        // Send the instruction
        SSI1_DR_R = instruction;

        for (i = 0; i < num_byte; i++)
           {


              // Wait until the Transmit FIFO is Not Full (TNF)
              while (!(SSI1_SR_R & SSI_SR_TNF))
              {
                ;// Wait
              }
              //Send the Multi-byte Data(MSB First)
              SSI1_DR_R =  data >>(num_byte-1-i)*8;

           }
         // On attend que le SSI ait fini de travailler avant de rendre la main.
             while (SSI1_SR_R & SSI_SR_BSY);
}
// Version optimis e pour QSPI 60MHz FIFO
void ssi1PutData_Burst(int instruction, long data) // Supposons 4 octets fixes frequency word
{
    while (!(SSI1_SR_R & SSI_SR_TNF));
    //  criture en rafale
    SSI1_DR_R = instruction;   // L'instruction part 33ns
    SSI1_DR_R = (data >> 24) & 0xFF;
    SSI1_DR_R = (data >> 16) & 0xFF;
    SSI1_DR_R = (data >> 8) & 0xFF;
    SSI1_DR_R = data & 0xFF;
    //   60MHz, la transmission sera finie ~100ns apr s la derni re ligne de code ci-dessus.
    //while (SSI0_SR_R & SSI_SR_BSY);
}
// Version optimis e pour QSPI FAST 60MHz FIFO frequency word
void ssi1PutData_QSPI_Fast(uint8_t instruction, uint32_t data)
{

    // Au lieu de faire 4 op rations de d calage (>>), on inverse tout le bloc
    // d'un coup avec une instruction CPU native.
    // data (0xAABBCCDD) devient swapped (0xDDCCBBAA)
    uint32_t swapped = __builtin_bswap32(data);

    // Pointeur volatile rapide vers le registre FIFO
    volatile uint32_t * const p_fifo = &SSI1_DR_R;

    // 2. V rification FIFO (Optimis e)
    // On v rifie une seule fois qu'il y a de la place.
    // Le FIFO fait g n ralement 8 mots de profondeur, on en  crit 5.
    // Si votre boucle principale est rapide, cette attente sera quasi nulle.
    while(!(SSI1_SR_R & SSI_SR_TNF));

    // 3.  criture en RAFALE (Burst)

    *p_fifo = instruction;      // Envoi Instruction

    // On  crit les octets directement depuis la variable invers e.
    *p_fifo = swapped;          // Prend les 8 bits de poids faible (anciennement MSB)
    *p_fifo = swapped >> 8;     // Le compilateur optimise  a en acc s registre direct (UXTB)
    *p_fifo = swapped >> 16;
    *p_fifo = swapped >> 24;
}
void ssi2PutData( int instruction,long data,int num_byte)
{
    int i;
    // Wait until the Transmit FIFO is Not Full (TNF)
        while (!(SSI2_SR_R & SSI_SR_TNF))
        {
           ; // Wait
        }
        // Send the instruction
        SSI2_DR_R = instruction;

        for (i = 0; i < num_byte; i++)
           {


              // Wait until the Transmit FIFO is Not Full (TNF)
              while (!(SSI2_SR_R & SSI_SR_TNF))
              {
                ;// Wait
              }
              //Send the Multi-byte Data(MSB First)
              SSI2_DR_R =  data >>(num_byte-1-i)*8;

           }
        // On attend que le SSI ait fini de travailler avant de rendre la main.
              while (SSI2_SR_R & SSI_SR_BSY);
}
// Version optimis e pour QSPI 60MHz FIFO
void ssi2PutData_Burst(int instruction, long data) // Supposons 4 octets fixes frequency word
{
    while (!(SSI2_SR_R & SSI_SR_TNF));
    //  criture en rafale
    SSI2_DR_R = instruction;   // L'instruction part 33ns
    SSI2_DR_R = (data >> 24) & 0xFF;
    SSI2_DR_R = (data >> 16) & 0xFF;
    SSI2_DR_R = (data >> 8) & 0xFF;
    SSI2_DR_R = data & 0xFF;
    //   60MHz, la transmission sera finie ~100ns apr s la derni re ligne de code ci-dessus.
    //while (SSI0_SR_R & SSI_SR_BSY);
}
// Version optimis e pour QSPI FAST 60MHz FIFO frequency word
void ssi2PutData_QSPI_Fast(uint8_t instruction, uint32_t data)
{

    // 1. Pr paration des donn es (SWAP d'Endianness)
    // Au lieu de faire 4 op rations de d calage (>>), on inverse tout le bloc
    // d'un coup avec une instruction CPU native.
    // data (0xAABBCCDD) devient swapped (0xDDCCBBAA)
    uint32_t swapped = __builtin_bswap32(data);

    // Pointeur volatile rapide vers le registre FIFO
    volatile uint32_t * const p_fifo = &SSI2_DR_R;

    // 2. V rification FIFO (Optimis e)
    // On v rifie une seule fois qu'il y a de la place.
    // Le FIFO fait g n ralement 8 mots de profondeur, on en  crit 5.
    // Si votre boucle principale est rapide, cette attente sera quasi nulle.
    while(!(SSI2_SR_R & SSI_SR_TNF));

    // 3.  criture en RAFALE (Burst)

    *p_fifo = instruction;      // Envoi Instruction

    // On  crit les octets directement depuis la variable invers e.
    // Le cast (uint8_t) est gratuit pour le CPU (simple s lection de registre).
    *p_fifo = swapped;          // Prend les 8 bits de poids faible (anciennement MSB)
    *p_fifo = swapped >> 8;     // Le compilateur optimise  a en acc s registre direct (UXTB)
    *p_fifo = swapped >> 16;
    *p_fifo = swapped >> 24;
}
// not working yet. Is the SDO well configured in the DDS pcb side
long long ssi0GetData(int instruction, int num_byte)
{
    //long long data;
    //int data = 0;   //0xFFFFFFFFFFFFFFFF
    int i = num_byte;
    //uint32_t data;


    // Synchronize I/O Port (Pin 17). Synchronizes the I/O port state machines without affecting the contents of the addressable registers.
    // An active high input on the IO RESET pin causes the current communication cycle to terminate.
    // After the IO RESET pin returns low (Logic 0), another communication cycle can begin, starting with the instruction byte.
    // PH0 = 0x01; // This is connect to DDS master reset (pin 71). We need I/O reset (pin 17) instead.
    // Read any residual data from the SSI port.  This makes sure the receive FIFOs are empty, so we don't read any unwanted junk.
    //while(SSIDataGetNonBlocking(SSI0_BASE, &dataReadSSIdump)) {}
    // PH0 = 0x00; // This is connect to DDS master reset (pin 71). We need I/O reset (pin 17) instead.

    while(!(SSI0_SR_R & SSI_SR_TNF)) {} // wait until there is space to write in the buffer
    //SSI0_DR_R =  0x88;
    SSI0_DR_R = instruction;
    while( i > 0 )
    {
        //while(!(SSI0_SR_R & SSI_SR_TNF)) {} // wait until there is space to write in the buffer
        //SSI0_DR_R =  data >> (num_byte-1-i)*8;

        while(!(SSI0_SR_R & SSI_SR_RNE)) {} // wait until there is data to be read.
        //while(!(HWREG(ui32Base + SSI_O_SR) & SSI_SR_RNE))   {        }
        //data = data | (SSI0_DR_R << i*8);
        //data = data | (SSI0_DR_R << (i - 1) * 8);
        //dataReadSSI = *(&SSI0_DR_R);
        //data = *(&SSI0_DR_R);

        SSIDataGet(SSI0_BASE, &dataReadSSIoneByte);
        //HWREGBITW(&g_ulDataReadFlags, FLAG_SSI_DATA_READING) = 1;
        //dataReadSSIoneByte = *(&SSI0_DR_R); // should also work instead of SSIDataGet
        dataReadSSIoneByte &= 0x00FF; // Since we are using 8-bit data, mask off the MSB.


        dataReadSSIempty = dataReadSSIempty | (dataReadSSIoneByte << ((i - 1) * 8));
        //data2 = data2 + *pData;
        //*dataPtr = *dataPtr | (SSI0_DR_R << (i - 1) * 8);

        i = i - 1;
    }
    //*dataPtr = data2;
    while(!( SSI0_SR_R & SSI_SR_TNF )) {} // wait until there is space to write in the buffer
    return (dataReadSSI & dataReadSSIempty);
}


//*****************************************************************************
//
//  Initializing GPIO ports
//
//*****************************************************************************
void PortA_Init(void)
{
    volatile unsigned long delay;

    //Activate clock for Port A
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;

    //Allow time for clock to stabilize
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R0) == 0){};

    // Disable Analog on PA7-2 (Preserve PA0/1 if they are UART)
    GPIO_PORTA_AMSEL_R &= ~0xFC;

    // Clear PCTL to select GPIO (clearing bits for PA7-2)
    GPIO_PORTA_PCTL_R &= ~0xFFFFF00;

    // DIR: PA3(8) + PA6(64) + PA7(128) = 200 (0xC8)
    // 0 = Input, 1 = Output
    GPIO_PORTA_DIR_R &= ~0xFC;  // Clear PA7-2 direction bits first (safety)
    GPIO_PORTA_DIR_R |= 0xC8;   // Set PA3, PA6, PA7 as Output

    // Disable Alternate Functions on PA7-2
    GPIO_PORTA_AFSEL_R &= ~0xFC;

    // Enable Digital I/O on PA7-2
    // Use 0xFF if you really want to enable PA0/PA1 as digital GPIOs too
    GPIO_PORTA_DEN_R |= 0xFC;
}
void PortB_Init(void)
{
    //Activate clock for Port B
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;

    //Allow time for clock to stabilize
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1) == 0){};

    // Disable Analog Functionality on PB4-0
    GPIO_PORTB_AMSEL_R &= ~0x1F;

    // Clear PCTL to select GPIO (Clear nibbles for PB4-0)
    // Each hex digit F corresponds to 4 bits of PCTL (1 pin).
    // We need to clear 5 pins, so we need 5 Fs -> 0x000FFFFF
    GPIO_PORTB_PCTL_R &= ~0x000FFFFF;
    // DIR: Configure PB4, PB3, PB2, PB1, PB0 as Output
    // 0x1F = Binary 0001 1111
    GPIO_PORTB_DIR_R |= 0x1F;

    // Disable Alternate Functions on PB4-0
    GPIO_PORTB_AFSEL_R &= ~0x1F;

    // Enable Digital I/O on PB4-0
    GPIO_PORTB_DEN_R |= 0x1F;
}
void PortD_Init(void)  //// Initialize GPIO Port D
{
  // activate clock for Port D
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // activate clock for Port D

  // allow time for clock to stabilize
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R3) == 0){};

  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;
  GPIO_PORTD_DIR_R |= 0xFF;            // make PD7-0 out
  GPIO_PORTD_AFSEL_R &= ~0xFF;     // disable alt funct on PD7-0
  //GPIO_PORTD_PUR_R = 0x03;        // enable pull-up on
  GPIO_PORTD_DEN_R |= 0xFF;         // enable digital I/O on PD7-0
  //GPIO_PORTD_PCTL_R &= ~0xFFFFFFFF; // 4) configure PD7-0 as GPIO
  GPIO_PORTD_AMSEL_R &= ~0xFF;        // disable analog functionality on PD7-0
}
void PortN_Init(void)
{
    //Activate clock for Port N
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;

    //Allow time for clock to stabilize
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};

    // Disable Analog on PN5-0
    // Port N has max 6 pins (PN0-PN5). Mask 0x3F = 0011 1111
    GPIO_PORTN_AMSEL_R &= ~0x3F;

    // Clear PCTL to select GPIO
    // We need to clear 6 nibbles (for 6 pins) -> 0x00FFFFFF
    GPIO_PORTN_PCTL_R &= ~0x00FFFFFF;

    // DIR: Configure PN5-0 as Output
    // 0x3F sets PN0, PN1, PN2, PN3, PN4, PN5 as Outputs
    GPIO_PORTN_DIR_R |= 0x3F;

    // Disable Alternate Functions on PN5-0
    GPIO_PORTN_AFSEL_R &= ~0x3F;

    // Enable Digital I/O on PN5-0
    GPIO_PORTN_DEN_R |= 0x3F;
    // --------------------------------------------------------
    // Optimisation Courant (Optionnel mais recommand  pour la vitesse)
    // --------------------------------------------------------
    // Configure les broches pour un courant plus fort (ex: 8mA) et Slew Rate rapide
    GPIOPadConfigSet(GPIO_PORTN_BASE, 0xFF,GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    // Activer le mode 8 mA pour PN0 et PN3
    // PN0 (0x01) + PN3 (0x08) = 0x09
    //HWREG(GPIO_PORTN_BASE + GPIO_O_DR8R) |= 0x09;
}
void PortH_Init(void)
{
    // Activate clock for Port H
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;

    // Allow time for clock to stabilize
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};

    // Disable Analog Functionality on PH0-PH3
    // ~0x0F = 1111...1111 0000
    GPIO_PORTH_AMSEL_R &= ~0x0F;

    // Clear PCTL to select GPIO mode
    // Clear the bottom 4 nibbles (PH0, PH1, PH2, PH3)
    GPIO_PORTH_PCTL_R &= ~0x0000FFFF;

    // Disable Alternate Functions on PH0-PH3
    GPIO_PORTH_AFSEL_R &= ~0x0F;

    // Direction: Set PH0, PH1, PH2, PH3 as Output
    // 0x0F = Binary 0000 1111
    GPIO_PORTH_DIR_R |= 0x0F;

    // Enable Digital I/O on PH0-PH3
    GPIO_PORTH_DEN_R |= 0x0F;
}
void PortM_Init(void)
{
    // Activate clock for Port M
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;

    // Allow time for clock to stabilize
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};

    // Disable Analog Functionality on PM3-0
    // Mask 0x0F = 0000 1111 (Pins 0, 1, 2, 3)
    GPIO_PORTM_AMSEL_R &= ~0x0F;

    // Clear PCTL to select GPIO mode
    // We clear the bottom 4 nibbles (PM0-PM3)
    GPIO_PORTM_PCTL_R &= ~0x0000FFFF;

    // Disable Alternate Functions on PM3-0
    // (Uncommented to ensure pure GPIO mode)
    GPIO_PORTM_AFSEL_R &= ~0x0F;

    // Direction: Set PM0, PM1, PM2, PM3 as Output
    // 0x0F = Binary 0000 1111
    GPIO_PORTM_DIR_R |= 0x0F;

    // Enable Digital I/O on PM3-0
    GPIO_PORTM_DEN_R |= 0x0F;
}
void PortP_Init(void)
{
    // Activate clock for Port P
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R13; // R13 is for Port P

    // Allow time for clock to stabilize
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R13) == 0){};

    // Disable Analog Functionality on PP5-PP2
    // We are using Pins 2, 3, 4, 5.
    // Binary: 0011 1100 = 0x3C
    GPIO_PORTP_AMSEL_R &= ~0x3C;

    // Clear PCTL to select GPIO mode
    // We need to clear nibbles for Pin 5, 4, 3, 2.
    // Mask: 0x00FFFF00
    GPIO_PORTP_PCTL_R &= ~0x00FFFF00;

    // Disable Alternate Functions on PP5-PP2
    GPIO_PORTP_AFSEL_R &= ~0x3C;

    // Direction: Set PP2, PP3, PP4, PP5 as Output
    // 0x3C = Binary 0011 1100
    GPIO_PORTP_DIR_R |= 0x3C;

    // Enable Digital I/O on PP5-PP2
    GPIO_PORTP_DEN_R |= 0x3C;
}
void PortL_Init(void)
{
    // Activate clock for Port L
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;

    // Allow time for clock to stabilize
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R10) == 0){};

    // Disable Analog Functionality on PL7-0
    GPIO_PORTL_AMSEL_R &= ~0xFF;

    // Clear PCTL to select GPIO mode
    GPIO_PORTL_PCTL_R &= ~0xFFFFFFFF;

    // DIR: Set PL7-0 as Input-
    // 0 = Input, 1 = Output.
    // We clear the bits to ensure they are 0.
    GPIO_PORTL_DIR_R &= ~0xFF;

    // Disable Alternate Functions on PL7-0
    GPIO_PORTL_AFSEL_R &= ~0xFF;

    // Enable Digital I/O on PL7-0
    GPIO_PORTL_DEN_R |= 0xFF;

    GPIO_PORTL_PDR_R |= 0xFF;
    GPIO_PORTL_PUR_R &= ~0xFF;

    // Optional: Enable Pull-Up Resistors (PUR) if buttons are connected
    // Uncomment if you need internal pull-ups for switches
    // GPIO_PORTL_PUR_R |= 0xFF;
}
void PortK_Init(void)
{
    // Activate clock for Port K
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9;

    // Allow time for clock to stabilize
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R9) == 0){};

    // Disable Analog Functionality on PK7-0
    GPIO_PORTK_AMSEL_R &= ~0xFF;

    // Clear PCTL to select GPIO mode
    GPIO_PORTK_PCTL_R &= ~0xFFFFFFFF;

    // DIR: Set PK7-0 as Output
    // 0 = Input, 1 = Output
    GPIO_PORTK_DIR_R |= 0xFF;

    // Disable Alternate Functions on PK7-0
    GPIO_PORTK_AFSEL_R &= ~0xFF;

    // Enable Digital I/O on PK7-0
    GPIO_PORTK_DEN_R |= 0xFF;
}
//Initialisation GPIOK Optimis e (AHB)
void DDS_GPIOK_Init_Fast(void)
{
    // --------------------------------------------------------
    // Activation de l'horloge et du BUS RAPIDE (AHB)
    // --------------------------------------------------------

    // Active l'horloge sur le Port K
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

    //Il faut attendre que le mat riel soit pr t
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));

    // ACTIVATION AHB : C'est cette ligne qui fait passer le bus de 100ns   20ns
    SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOK);

    // --------------------------------------------------------
    // Configuration des broches (Direction et Num rique)
    // --------------------------------------------------------

    // Configuration de tout le PORT K en SORTIE (Output)
   // Elle g re automatiquement l'adresse de base AHB (GPIO_PORTK_BASE)
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, 0xFF);

    // --------------------------------------------------------
    // Optimisation Courant (Optionnel mais recommand  pour la vitesse)
    // --------------------------------------------------------
    // Configure les broches pour un courant plus fort (ex: 8mA) et Slew Rate rapide
    GPIOPadConfigSet(GPIO_PORTK_BASE, 0xFF,GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
}
void PeripheralEnableInit(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

//#ifdef USE_PORTB_PORTN
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
//#endif

}
//Port L d clencher une interruption sur un front montant.
void IO_interrupt_portL(void)
{
    //Activation du Port et de l'Horloge
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    //Attendre que le p riph rique soit pr t
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL))
    {
    }
    //Configuration des Broches
    GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, PL0|PL1|PL2|PL3|PL4|PL5|PL6|PL7);

    //Configuration des Interruptions
    //GPIOIntTypeSet(GPIO_PORTL_BASE, PL0|PL1|PL2|PL3|PL4|PL5|PL6|PL7, GPIO_HIGH_LEVEL);
    GPIOIntTypeSet(GPIO_PORTL_BASE, GPIO_INT_PIN_0|GPIO_INT_PIN_1|GPIO_INT_PIN_2|GPIO_INT_PIN_3|GPIO_INT_PIN_4|GPIO_INT_PIN_5|GPIO_INT_PIN_6|GPIO_INT_PIN_7, GPIO_RISING_EDGE);
    //Effacer les drapeaux d'interruption existants
    GPIOIntClear(GPIO_PORTL_BASE, PL0|PL1|PL2|PL3|PL4|PL5|PL6|PL7);

    //Activation des interruptions pour les broches
    GPIOIntDisable(GPIO_PORTL_BASE, GPIO_INT_PIN_0|GPIO_INT_PIN_1|GPIO_INT_PIN_2|GPIO_INT_PIN_3|GPIO_INT_PIN_4|GPIO_INT_PIN_5|GPIO_INT_PIN_6|GPIO_INT_PIN_7);

    GPIOIntClear(GPIO_PORTL_BASE, GPIO_INT_PIN_0|GPIO_INT_PIN_1|GPIO_INT_PIN_2|GPIO_INT_PIN_3|GPIO_INT_PIN_4|GPIO_INT_PIN_5|GPIO_INT_PIN_6|GPIO_INT_PIN_7);

    IntRegister(INT_GPIOL, GPIOIntHandler);
    IntPrioritySet(INT_GPIOL, 0x40);

    IntEnable(INT_GPIOL);

    GPIOIntEnable(GPIO_PORTL_BASE, GPIO_INT_PIN_0|GPIO_INT_PIN_1|GPIO_INT_PIN_2|GPIO_INT_PIN_3|GPIO_INT_PIN_4|GPIO_INT_PIN_5|GPIO_INT_PIN_6|GPIO_INT_PIN_7);
}
/* static inline void Select_Channels_Mask_DDS1(uint8_t ChannelDDS_UART)
{
    // Le CSR (0x00) attend les canaux sur les bits 7-4.
    // (masque & 0x0F) << 4 : D placer les bits vers la gauche (ex: 0000 0011 devient 0011 0000)
    // 0x06 : On force les bits 1 et 2   '1' pour rester en mode "4-bit Serial" (Quad)
    // ATTENTION :en mode "Single Bit" (SPI standard), on remplace 0x06 par 0x00.

    uint8_t csr_value = ((ChannelDDS_UART & 0x0F) << 4) | 0x06;  // exemple CSR = 0x26  CH1 quad bit

    //Canaux   Activer    Masque(Entr e)  Calcul Binaire (CSR 7-0) Valeur Hexa   Envoyer
    //      Aucun              0              0000 0110                  0x06
    //      CH 0               1              0001 0110                  0x16
    //      CH 1               2              0010 0110                  0x26
    //      CH 2               4              0100 0110                  0x46
    //      CH 3               8              1000 0110                  0x86
    //   CH0  + CH 1           3              0011 0110                  0x36
    //   CH0  + CH 2           5              0101 0110                  0x56
    //   CH0  + CH 3           9              1001 0110                  0x96
    //   CH1  + CH 2           6              0110 0110                  0x66
    //   CH1  + CH 3           10             1010 0110                  0xA6
    //   CH2  + CH 3           12             1100 0110                  0xC6
    //  CH0  + CH2 + CH3       13             1101 0110                  0xD6
    //  CH1  + CH2 + CH3       14             1110 0110                  0xE6
    //  CH0  + CH1 + CH2       7              0111 0110                  0x76
    //  CH0 + CH1 +CH2 + CH3   15             1111 0110                  0xF6
    // On  crit dans le registre CSR (Adresse 0x00)
    ssi0PutData(CSR_ADRESS, csr_value, CSR_NUM_BYTE);
}
*/
/* static inline void Select_Channels_Mask_DDS2(uint8_t ChannelDDS_UART)
{
        // Le CSR (0x00) attend les canaux sur les bits 7-4.
        // (masque & 0x0F) << 4 : D placer les bits vers la gauche (ex: 0000 0011 devient 0011 0000)
        // 0x06 : On force les bits 1 et 2   '1' pour rester en mode "4-bit Serial" (Quad)
        // ATTENTION :en mode "Single Bit" (SPI standard), on remplace 0x06 par 0x00.

        uint8_t csr_value = ((ChannelDDS_UART & 0x0F) << 4) | 0x06;  // exemple CSR = 0x26  CH1 quad bit

        //Canaux   Activer    Masque(Entr e)  Calcul Binaire (CSR 7-0) Valeur Hexa   Envoyer
        //      Aucun              0              0000 0110                  0x06
        //      CH 0               1              0001 0110                  0x16
        //      CH 1               2              0010 0110                  0x26
        //      CH 2               4              0100 0110                  0x46
        //      CH 3               8              1000 0110                  0x86
        //   CH0  + CH 1           3              0011 0110                  0x36
        //   CH0  + CH 2           5              0101 0110                  0x56
        //   CH0  + CH 3           9              1001 0110                  0x96
        //   CH1  + CH 2           6              0110 0110                  0x66
        //   CH1  + CH 3           10             1010 0110                  0xA6
        //   CH2  + CH 3           12             1100 0110                  0xC6
        //  CH0  + CH2 + CH3       13             1101 0110                  0xD6
        //  CH1  + CH2 + CH3       14             1110 0110                  0xE6
        //  CH0  + CH1 + CH2       7              0111 0110                  0x76
        //  CH0 + CH1 +CH2 + CH3   15             1111 0110                  0xF6
        // On  crit dans le registre CSR (Adresse 0x00)
        ssi1PutData(CSR_ADRESS, csr_value, CSR_NUM_BYTE);
}
*/
/* static inline void Select_Channels_Mask_DDS3(uint8_t ChannelDDS_UART)
{
        // Le CSR (0x00) attend les canaux sur les bits 7-4.
        // (masque & 0x0F) << 4 : D placer les bits vers la gauche (ex: 0000 0011 devient 0011 0000)
        // 0x06 : On force les bits 1 et 2   '1' pour rester en mode "4-bit Serial" (Quad)
        // ATTENTION :en mode "Single Bit" (SPI standard), on remplace 0x06 par 0x00.

        uint8_t csr_value = ((ChannelDDS_UART & 0x0F) << 4) | 0x06;  // exemple CSR = 0x26  CH1 quad bit

        //Canaux   Activer    Masque(Entr e)  Calcul Binaire (CSR 7-0) Valeur Hexa   Envoyer
        //      Aucun              0              0000 0110                  0x06
        //      CH 0               1              0001 0110                  0x16
        //      CH 1               2              0010 0110                  0x26
        //      CH 2               4              0100 0110                  0x46
        //      CH 3               8              1000 0110                  0x86
        //   CH0  + CH 1           3              0011 0110                  0x36
        //   CH0  + CH 2           5              0101 0110                  0x56
        //   CH0  + CH 3           9              1001 0110                  0x96
        //   CH1  + CH 2           6              0110 0110                  0x66
        //   CH1  + CH 3           10             1010 0110                  0xA6
        //   CH2  + CH 3           12             1100 0110                  0xC6
        //  CH0  + CH2 + CH3       13             1101 0110                  0xD6
        //  CH1  + CH2 + CH3       14             1110 0110                  0xE6
        //  CH0  + CH1 + CH2       7              0111 0110                  0x76
        //  CH0 + CH1 +CH2 + CH3   15             1111 0110                  0xF6
        // On  crit dans le registre CSR (Adresse 0x00)
        ssi2PutData(CSR_ADRESS, csr_value, CSR_NUM_BYTE);
}
*/
/* static inline void Select_Channels_Mask_DDS1_eth(uint8_t g_channel)
{
           // Le CSR (0x00) attend les canaux sur les bits 7-4.
           // (masque & 0x0F) << 4 : D placer les bits vers la gauche (ex: 0000 0011 devient 0011 0000)
           // 0x06 : On force les bits 1 et 2   '1' pour rester en mode "4-bit Serial" (Quad)
           // ATTENTION :en mode "Single Bit" (SPI standard), on remplace 0x06 par 0x00.

           uint8_t csr_value = ((ChannelDDS_UART & 0x0F) << 4) | 0x06;  // exemple CSR = 0x26  CH1 quad bit

           //Canaux   Activer    Masque(Entr e)  Calcul Binaire (CSR 7-0) Valeur Hexa   Envoyer
           //      Aucun              0              0000 0110                  0x06
           //      CH 0               1              0001 0110                  0x16
           //      CH 1               2              0010 0110                  0x26
           //      CH 2               4              0100 0110                  0x46
           //      CH 3               8              1000 0110                  0x86
           //   CH0  + CH 1           3              0011 0110                  0x36
           //   CH0  + CH 2           5              0101 0110                  0x56
           //   CH0  + CH 3           9              1001 0110                  0x96
           //   CH1  + CH 2           6              0110 0110                  0x66
           //   CH1  + CH 3           10             1010 0110                  0xA6
           //   CH2  + CH 3           12             1100 0110                  0xC6
           //  CH0  + CH2 + CH3       13             1101 0110                  0xD6
           //  CH1  + CH2 + CH3       14             1110 0110                  0xE6
           //  CH0  + CH1 + CH2       7              0111 0110                  0x76
           //  CH0 + CH1 +CH2 + CH3   15             1111 0110                  0xF6
           // On  crit dans le registre CSR (Adresse 0x00)
           ssi0PutData(CSR_ADRESS, csr_value, CSR_NUM_BYTE);
}
*/
/* static inline void Select_Channels_Mask_DDS2_eth(uint8_t g_channel)
{
           // Le CSR (0x00) attend les canaux sur les bits 7-4.
           // (masque & 0x0F) << 4 : D placer les bits vers la gauche (ex: 0000 0011 devient 0011 0000)
           // 0x06 : On force les bits 1 et 2   '1' pour rester en mode "4-bit Serial" (Quad)
           // ATTENTION :en mode "Single Bit" (SPI standard), on remplace 0x06 par 0x00.

           uint8_t csr_value = ((ChannelDDS_UART & 0x0F) << 4) | 0x06;  // exemple CSR = 0x26  CH1 quad bit

           //Canaux   Activer    Masque(Entr e)  Calcul Binaire (CSR 7-0) Valeur Hexa   Envoyer
           //      Aucun              0              0000 0110                  0x06
           //      CH 0               1              0001 0110                  0x16
           //      CH 1               2              0010 0110                  0x26
           //      CH 2               4              0100 0110                  0x46
           //      CH 3               8              1000 0110                  0x86
           //   CH0  + CH 1           3              0011 0110                  0x36
           //   CH0  + CH 2           5              0101 0110                  0x56
           //   CH0  + CH 3           9              1001 0110                  0x96
           //   CH1  + CH 2           6              0110 0110                  0x66
           //   CH1  + CH 3           10             1010 0110                  0xA6
           //   CH2  + CH 3           12             1100 0110                  0xC6
           //  CH0  + CH2 + CH3       13             1101 0110                  0xD6
           //  CH1  + CH2 + CH3       14             1110 0110                  0xE6
           //  CH0  + CH1 + CH2       7              0111 0110                  0x76
           //  CH0 + CH1 +CH2 + CH3   15             1111 0110                  0xF6
           // On  crit dans le registre CSR (Adresse 0x00)
           ssi1PutData(CSR_ADRESS, csr_value, CSR_NUM_BYTE);
}
*/
/*static inline void Select_Channels_Mask_DDS3_eth(uint8_t g_channel)
{
        // Le CSR (0x00) attend les canaux sur les bits 7-4.
        // (masque & 0x0F) << 4 : D placer les bits vers la gauche (ex: 0000 0011 devient 0011 0000)
        // 0x06 : On force les bits 1 et 2   '1' pour rester en mode "4-bit Serial" (Quad)
        // ATTENTION :en mode "Single Bit" (SPI standard), on remplace 0x06 par 0x00.

        uint8_t csr_value = ((ChannelDDS_UART & 0x0F) << 4) | 0x06;  // exemple CSR = 0x26  CH1 quad bit

        //Canaux   Activer    Masque(Entr e)  Calcul Binaire (CSR 7-0) Valeur Hexa   Envoyer
        //      Aucun              0              0000 0110                  0x06
        //      CH 0               1              0001 0110                  0x16
        //      CH 1               2              0010 0110                  0x26
        //      CH 2               4              0100 0110                  0x46
        //      CH 3               8              1000 0110                  0x86
        //   CH0  + CH 1           3              0011 0110                  0x36
        //   CH0  + CH 2           5              0101 0110                  0x56
        //   CH0  + CH 3           9              1001 0110                  0x96
        //   CH1  + CH 2           6              0110 0110                  0x66
        //   CH1  + CH 3           10             1010 0110                  0xA6
        //   CH2  + CH 3           12             1100 0110                  0xC6
        //  CH0  + CH2 + CH3       13             1101 0110                  0xD6
        //  CH1  + CH2 + CH3       14             1110 0110                  0xE6
        //  CH0  + CH1 + CH2       7              0111 0110                  0x76
        //  CH0 + CH1 +CH2 + CH3   15             1111 0110                  0xF6
        // On  crit dans le registre CSR (Adresse 0x00)
        ssi2PutData(CSR_ADRESS, csr_value, CSR_NUM_BYTE);
}
*/
void ExecuteManualSweep_DDS_Setup_uart(unsigned int NbDDS_UART,unsigned int ChannelDDS_UART,unsigned long start_freq_hz,unsigned long end_freq_hz,unsigned long step_freq_hz,double time_step_s)
{
    //Initialisation de l' tat global du balayage
    NbDDS_UART = NbDDS_UART;
    g_channel = ChannelDDS_UART; // Le masque (ex: 15 pour tout, 3 pour CH0+CH1)
    g_ftw_step = FreqDeciToFTW(step_freq_hz);
    g_ftw_end = FreqDeciToFTW(end_freq_hz);
    g_is_rising = (FreqDeciToFTW(end_freq_hz) > FreqDeciToFTW(start_freq_hz));

    // Stocke la FTW de d part ( tat initial)
    g_ftw_start_config = FreqDeciToFTW(start_freq_hz);    
    g_current_ftw = g_ftw_start_config;



    //Configuration du Timer pour le pas de temps (ne d marre pas le Timer)
    Init_Timer2A_Sweep(time_step_s);
    choiceNbDDS((unsigned int)NbDDS_UART, false);

                   if(NbDDS_UART == 1)
                   {
                   // Charger la fr quence de d part sur le DDS et Trigger l'IO_UPDATE
                   choiceChannelDDS1((unsigned int)ChannelDDS_UART, false);  // S lection du Canal
                   //Select_Channels_Mask_DDS1(ChannelDDS_UART);
                   SetFrequencyTuningWord0_DDS1(g_current_ftw, g_channel);
                   HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7   1  AD9959 I/O update DDS1
                   // Petit d lai (Hold time) - 3 NOPs   120MHz = ~25ns
                   __asm(" NOP");
                   HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7   0
                   }
                   if(NbDDS_UART == 2)
                   {
                   // Charger la fr quence de d part sur le DDS et Trigger l'IO_UPDATE
                   choiceChannelDDS2((unsigned int)ChannelDDS_UART, false);  // S lection du Canal
                   //Select_Channels_Mask_DDS2(ChannelDDS_UART);
                   SetFrequencyTuningWord0_DDS2(g_current_ftw, g_channel);
                   HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;  // Mettre PN3   1  AD9959 I/O update DDS2
                   // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
                   __asm(" NOP");
                   HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // Mettre PN3   0
                   }
                   if(NbDDS_UART == 3)
                   {
                   // Charger la fr quence de d part sur le DDS et Trigger l'IO_UPDATE
                   choiceChannelDDS3((unsigned int)ChannelDDS_UART, false);  // S lection du Canal
                   //Select_Channels_Mask_DDS3(ChannelDDS_UART);
                   SetFrequencyTuningWord0_DDS3(g_current_ftw, g_channel);
                   HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01; // Mettre PN0   1  AD9959 I/O update DDS3
                   // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
                   __asm(" NOP");
                   HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00; // Mettre PN0   0
                   }

    // D finir l' tat comme pr t pour le balayage (mais pas encore en cours)
    g_sweep_running = false; // Le balayage est configur , mais arr t .
    //TimerEnable(TIMER2_BASE, TIMER_A); // Active Timer 2 Subtimer A
}
void ExecuteManualSweep_DDS_Setup_ethernet(unsigned int CURRENT_DDS,unsigned int ChannelDDS,unsigned long start_freq_hz,unsigned long end_freq_hz,unsigned long step_freq_hz,double time_step_s)
{
    //Initialisation de l' tat global du balayage
    CURRENT_DDS = CURRENT_DDS;
    time_step_s = time_step_s;
    g_channel = ChannelDDS;  // Le masque (ex: 15 pour tout, 3 pour CH0+CH1)
    g_ftw_step = FreqToFTW(step_freq_hz);
    g_ftw_end = FreqToFTW(end_freq_hz);
    //g_ftw_end = end_freq_hz;
    g_is_rising = (FreqToFTW(end_freq_hz) > FreqToFTW(start_freq_hz));

    // Stocke la FTW de d part ( tat initial)
    g_ftw_start_config = FreqToFTW(start_freq_hz);
    g_current_ftw = g_ftw_start_config;


    //Configuration du Timer pour le pas de temps (ne d marre pas le Timer)
    Init_Timer1A_Sweep(time_step_s);


                   if(CURRENT_DDS == 1 )
                   {
                          // Charger la fr quence de d part sur le DDS et Trigger l'IO_UPDATE
                          if(g_channel==0)
                              {
                               ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS1,CSR_NUM_BYTE);  //select CH0
                               //Select_Channels_Mask_DDS1_eth(g_channel);
                               SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                               HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7   1  AD9959 I/O update DDS1
                               // Petit d lai (Hold time) - 3 NOPs   120MHz = ~25ns
                               __asm(" NOP");
                               HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7   0
                               }
                           else if(g_channel==1)
                               {
                               ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS1,CSR_NUM_BYTE);  //select CH1
                               //Select_Channels_Mask_DDS1_eth(g_channel);
                               SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                               HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7   1  AD9959 I/O update DDS1
                               // Petit d lai (Hold time) - 3 NOPs   120MHz = ~25ns
                               __asm(" NOP");
                               HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7   0
                               }
                           else if(g_channel==2)
                               {
                               ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS1,CSR_NUM_BYTE);  //select CH2
                               //Select_Channels_Mask_DDS1_eth(g_channel);
                               SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                               HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7   1  AD9959 I/O update DDS1
                               // Petit d lai (Hold time) - 3 NOPs   120MHz = ~25ns
                               __asm(" NOP");
                               HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7   0
                               }
                           else if(g_channel==3)
                               {
                               ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS1,CSR_NUM_BYTE);  //select CH3
                               //Select_Channels_Mask_DDS1_eth(g_channel);
                               SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                               HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x80;  // Mettre PK7   1  AD9959 I/O update DDS1
                               // Petit d lai (Hold time) - 3 NOPs   120MHz = ~25ns
                               __asm(" NOP");
                               HWREG(GPIO_PORTK_BASE + (GPIO_O_DATA + (0x80 << 2))) = 0x00; // Mettre PK7   0
                               }
                           else
                               {
                               // return false; pour le cas g_channel invalide
                               }

                    }
                   if(CURRENT_DDS == 2 )
                    {
                            // Charger la fr quence de d part sur le DDS et Trigger l'IO_UPDATE
                            if(g_channel==0)
                             {
                              ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS2,CSR_NUM_BYTE);  //select CH0
                              SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                              HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;  // Mettre PN3   1  AD9959 I/O update DDS2
                              // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
                              __asm(" NOP");
                              HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // Mettre PN3   0
                              }
                          else if(g_channel==1)
                              {
                              ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS2,CSR_NUM_BYTE);  //select CH1
                              SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                              HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;  // Mettre PN3   1  AD9959 I/O update DDS2
                              // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
                              __asm(" NOP");
                              HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // Mettre PN3   0
                              }
                          else if(g_channel==2)
                              {
                              ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS2,CSR_NUM_BYTE);  //select CH2
                              SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                              HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;  // Mettre PN3   1  AD9959 I/O update DDS2
                              // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
                              __asm(" NOP");
                              HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // Mettre PN3   0
                              }
                          else if(g_channel==3)
                              {
                              ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS2,CSR_NUM_BYTE);  //select CH3
                              SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                              HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x08;  // Mettre PN3   1  AD9959 I/O update DDS2
                              // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
                              __asm(" NOP");
                              HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x08 << 2))) = 0x00; // Mettre PN3   0
                              }
                          else
                              {
                              // return false; pour le cas g_channel invalide
                              }
                   }
                   if(CURRENT_DDS == 3 )
                   {
                                      // Charger la fr quence de d part sur le DDS et Trigger l'IO_UPDATE
                           if(g_channel==0)
                            {
                             ssi0PutData(CSR_ADRESS,CSR0_4bit_DDS3,CSR_NUM_BYTE);  //select CH0
                             SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                             HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01; // Mettre PN0   1  AD9959 I/O update DDS3
                             // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
                             __asm(" NOP");
                             HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00; // Mettre PN0   0
                             }
                         else if(g_channel==1)
                             {
                             ssi0PutData(CSR_ADRESS,CSR1_4bit_DDS3,CSR_NUM_BYTE);  //select CH1
                             SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                             HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01; // Mettre PN0   1  AD9959 I/O update DDS3
                             // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
                             __asm(" NOP");
                             HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00; // Mettre PN0   0
                             }
                         else if(g_channel==2)
                             {
                             ssi0PutData(CSR_ADRESS,CSR2_4bit_DDS3,CSR_NUM_BYTE);  //select CH2
                             SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                             HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01; // Mettre PN0   1  AD9959 I/O update DDS3
                             // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
                             __asm(" NOP");
                             HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00; // Mettre PN0   0
                             }
                         else if(g_channel==3)
                             {
                             ssi0PutData(CSR_ADRESS,CSR3_4bit_DDS3,CSR_NUM_BYTE);  //select CH3
                             SetFrequencyTuningWord_ethernet(g_current_ftw, g_channel);
                             HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x01; // Mettre PN0   1  AD9959 I/O update DDS3
                             // Petit d lai (Hold time) - 1 NOPs   120MHz = ~8ns MAIS EN PRATIQUE = 15-20ns
                             __asm(" NOP");
                             HWREG(GPIO_PORTN_BASE + (GPIO_O_DATA + (0x01 << 2))) = 0x00; // Mettre PN0   0
                             }
                         else
                             {
                             // return false; pour le cas g_channel invalide
                             }
                  }
    // D finir l' tat comme pr t pour le balayage (mais pas encore en cours)
    g_sweep_running2 = false; // Le balayage est configur , mais arr t .
    //TimerEnable(TIMER1_BASE, TIMER_A); // Active Timer 1 Subtimer A
}
void Apply_Sweep_Configuration_uart(unsigned int NbDDS_UART,unsigned int ChannelDDS_UART,unsigned long start_freq_hz,unsigned long end_freq_hz,unsigned long step_freq_hz,double time_step_s)
{
    // Ex cute la configuration
    ExecuteManualSweep_DDS_Setup_uart(
        NbDDS_UART,
        ChannelDDS_UART,
        start_freq_hz,
        end_freq_hz,
        step_freq_hz,
        time_step_s
    );
}
void Apply_Sweep_Configuration_ethernet(unsigned int NbDDS,unsigned int ChannelDDS,unsigned long start_freq_hz,unsigned long end_freq_hz,unsigned long step_freq_hz,double time_step_s)
{
    // Ex cute la configuration
    ExecuteManualSweep_DDS_Setup_ethernet(
        CURRENT_DDS,
        ChannelDDS,
        start_freq_hz,
        end_freq_hz,
        step_freq_hz,
        time_step_s
    );
}
void Apply_Sweep_Configuration_on_interrupt_port(unsigned int NbDDS_UART,unsigned int ChannelDDS_UART,unsigned long start_freq_hz,unsigned long end_freq_hz,unsigned long step_freq_hz,double time_step_s)
{
    // Ex cute la configuration
    Load_And_Start_Sweep_Uart_calculate(
        NbDDS_UART,
        ChannelDDS_UART,
        start_freq_hz,
        end_freq_hz,
        step_freq_hz,
        time_step_s
    );
}
//***********************************************************************************
//
// MAIN
// This example demonstrates the use of the Ethernet Controller and lwIP TCP/IP stack
//
//
//***********************************************************************************
/* =============================================================================
 * POINT D'ENTREE
 * =============================================================================
 * Ordre général :
 *   1. initialiser horloge et périphériques ;
 *   2. initialiser les trois AD9959 ;
 *   3. initialiser UART + moteur de tables ;
 *   4. rester dans la boucle de service hors temps réel.
 */
int main(void)
{
    //
    // Run from the PLL at 120 MHz.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), SYSCLKMICRO);

    PeripheralEnableInit();
    SysCtlDelay(3);
    init_UART0();

    /* V32.3 UART startup fix.
     *
     * IMPORTANT: keep global interrupts disabled while initTimer1/2/4/5/6/7
     * and IO_interrupt_portL() register their handlers.  These initializers
     * enable NVIC/peripheral interrupt sources before calling IntRegister().
     * Calling IntMasterEnable() here (the broken V32.3 ordering) therefore
     * allows an interrupt to branch through an unregistered/default vector
     * during boot, leaving the CPU stuck before UART0 can receive anything.
     *
     * This restores the proven V32.2 ordering: configure the UART source now,
     * but enable global interrupts only after every handler and DDS peripheral
     * has been initialized. */
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //GPIOPinUnlockGPIO(GPIO_PORTD_BASE, GPIO_PIN_7);     // Unlock the PD7 GPIO
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7); // Configure PD7 as output

    //////////////////////////////////////////////////////////
    //
    // Timer initialization for triggered/buffered sweep frequency
    //
    /////////////////////////////////////////////////////////
    initTimer1(); // Eth  parameters sweep
    initTimer2(); // Uart parameters sweep
    initTimer3(); // legacy Timer3A path; V33.2 phase midpoint uses CCP/PWM on DDS3
    initTimer4(); // Scheduler général STEP / linear sweep
    initTimer5(); // Moteur HW sweep DDS1, channel dynamique
    initTimer6(); // Moteur HW sweep DDS2, channel dynamique
    initTimer7(); // Moteur HW sweep DDS3, channel dynamique
    Seq_InitCycleCounter();
    Tbl34_Init();
    Engine_ConfigureTimersForGeneral();

    PortA_Init(); // For UART0 (USB) interrupt and SSI0
    PortB_Init(); // lock on/off
    PortD_Init(); // SSI2 & SSI1
    PortH_Init(); // DDS1 P3-P2-P1-P0
    //DDS_GPIOK_Init_Fast();
    PortK_Init(); // Master reset DDS1 - PWR down DDS1 - i/o update DDS1
    DDS_GPIOK_Init_Fast(); // Master reset DDS1 - PWR down DDS1 - i/o update DDS1
    PortM_Init(); // DDS3 P3-P2-P1-P0
    PortL_Init(); // 8 TTLs in
    PortN_Init(); // master reset DDS2 & DDS3 - PWR down DDS2 & DDS3 - i/o update DDS2 & DDS3
    PortP_Init(); // DDS2 P3-P2-P1-P0
    IO_interrupt_portL();  //interruptions sur broches port L
    //Init_TTL3_Precalculer();
    SysCtlDelay(10);  // 0.0250us  ( 250ns)

    // Port F for Ethernet LEDs
    GPIOPinConfigure(GPIO_PF0_EN0LED0);
    GPIOPinConfigure(GPIO_PF4_EN0LED1);
    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    SysCtlDelay(10); // 0.0250us  ( 250ns)

    //*************** DDS 1**********//
    //make master reset DDS1
    PK5 = 0x20; // reset DDS1
    SysCtlDelay(10);
    PK5 = 0x00;

    //Enable Peripheral SSI0
    init_SPI0_mode_single_bit();  //single bit DDS1
    ssi0PutData(CSR_ADRESS,0xF0,CSR_NUM_BYTE);  //select CH0 and CH1 and CH2 and CH3 single bit
    ssi0PutData(CSR_ADRESS,0xF6,CSR_NUM_BYTE);  //AD9959 CH0 CH1 CH2 CH3 MSB first 4-bit serial mode

    PK7 = 0x80; // AD9959 I/O update DDS1
    SysCtlDelay(10);
    PK7 = 0x00;

    init_SPI0_mode_quad_bit();  //quad SPI DDS1

    //*************** DDS 2**********//
    // make master reset DDS2
    PN4 = 0x10; // reset DDS2
    SysCtlDelay(10);
    PN4 = 0x00;

    //Enable Peripheral SSI1
    init_SPI1_mode_single_bit();   //single bit DDS2
    ssi1PutData(CSR_ADRESS,0xF0,CSR_NUM_BYTE);  //select CH0 and CH1 and CH2  and CH3 single bit
    ssi1PutData(CSR_ADRESS,0xF6,CSR_NUM_BYTE);  //AD9959 CH0 CH1 CH2 CH3 MSB first 4-bit serial mode

    SysCtlDelay(10);
    PN3 = 0x08; // AD9959 I/O update DDS2
    SysCtlDelay(10);
    PN3 = 0x00;

    init_SPI1_mode_quad_bit();  //quad SPI DDS2


    //*************** DDS 3**********//
    //make master reset DDS3
    PN1 = 0x02; // reset DDS3
    SysCtlDelay(10);
    PN1 = 0x00;

    //Enable Peripheral SSI2
    init_SPI2_mode_single_bit();  //single bit DDS2
    ssi2PutData(CSR_ADRESS,0xF0,CSR_NUM_BYTE);  //select CH0 and CH1 and CH2  and CH3 single bit
    ssi2PutData(CSR_ADRESS,0xF6,CSR_NUM_BYTE);  //AD9959 CH0 CH1 CH2 CH3 MSB first 4-bit serial mode

    SysCtlDelay(10);
    PN0 = 0x01; // AD9959 I/O update DDS3
    SysCtlDelay(10);
    PN0 = 0x00;

    init_SPI2_mode_quad_bit();  //quad SPI DDS3

    /* V33 startup: keep the DDS initialization non-blocking after the three
     * quad-SPI interfaces are configured.  Outputs are programmed by RESET,
     * LOAD or the general-engine static-set frame. */

    SysCtlDelay(10);
    PB0 = 0x01;   // lock on/off
    SysCtlDelay(10);

    /* All Timer/GPIO handlers have now been registered and the DDS startup
     * sequence is complete.  Global interrupts may safely be enabled here,
     * exactly as in the working V32.2 firmware. */
    IntMasterEnable();

    /* Configure UARTstdio only after the board initialization, matching the
     * working V32.2 order.  The custom UART0 vector remains UARTIntHandler. */
    UARTStdioConfig(0, DDS_UART_BAUDRATE, g_ui32SysClock);

    //
    // Clear the terminal and print a banner.
    //
    UARTprintf("\033[2J\033[H");
    UARTprintf("Ethernet IO start\n\n");
    UARTprintf("IP Address: ");

    //
    // Configure SysTick for a periodic interrupt = 1Hz
    //
    // SysTickPeriodSet(g_ui32SysClock / SYSTICKHZ);
    // Systick is for LwIP
    SysTickPeriodSet(g_ui32SysClock);
    SysTickEnable();
    SysTickIntEnable();


    //////////////////////////////////////////
    //
    // Getting an IP address from router
    //
    //////////////////////////////////////////
    //
    // Tell the user what we are doing just now.
    //
    UARTprintf("Waiting for IP.\n");
    UARTprintf("UART baudrate = 921600\n");
    UARTprintf("UART OK\n");
    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split
    // MAC address needed to program the hardware registers, then program
    // the MAC address into the Ethernet Controller registers.
    //
    uint32_t ui32User0 = MACADDRESS1; // This is part 1 of the MAC address, to be changed for each new device
    uint32_t ui32User1 = MACADDRESS2; // This is part 2 of the MAC address, to be changed for each new device
    uint8_t pui8MACArray[8];

    pui8MACArray[0] = ((ui32User0 >>  0) & 0xff);
    pui8MACArray[1] = ((ui32User0 >>  8) & 0xff);
    pui8MACArray[2] = ((ui32User0 >> 16) & 0xff);
    pui8MACArray[3] = ((ui32User1 >>  0) & 0xff);
    pui8MACArray[4] = ((ui32User1 >>  8) & 0xff);
    pui8MACArray[5] = ((ui32User1 >> 16) & 0xff);


    //
    // Initialize the lwIP library, using DHCP.
    //
    lwIPInit(g_ui32SysClock, pui8MACArray, 0, 0, 0, IPADDR_USE_DHCP);


    //
    // Start lwIP tick interrupt.
    //
    //HWREGBITW(&g_sEnet.ui32Flags, FLAG_TIMER_DHCP_EN) = 1;


    //
    // Setup the device locator service.
    //
    LocatorInit();
    LocatorMACAddrSet(pui8MACArray);
    LocatorAppTitleSet("EK-TM4C1294XL enet_io");

    //////////////////////////////////////////


    //////////////////////////////////////////
    //
    // Launch the TCP server (e.g. for labscript)
    //
    //////////////////////////////////////////
    tcp_server_init();

    ///////////////////////////////////////////


    //////////////////////////////////////////
    //
    // Set the priorities for each interrupt
    //
    //////////////////////////////////////////
    IntPrioritySet(INT_EMAC0,ETHERNET_TCP_INT_PRIORITY);
    IntPrioritySet(FAULT_SYSTICK,SYSTICK_INT_PRIORITY);
    IntPrioritySet(INT_TIMER2A,TIMER2_INT_PRIORITY);
    IntPrioritySet(INT_TIMER1A,TIMER1_INT_PRIORITY);

#if SEQ_EQUALIZE_TIMER_PRIORITIES
    /* Test facultatif : Timer5/6/7 ne peuvent plus préempter Timer4. */
    IntPrioritySet(INT_TIMER5A,0x20);
    IntPrioritySet(INT_TIMER6A,0x20);
    IntPrioritySet(INT_TIMER7A,0x20);
    IntPrioritySet(INT_TIMER4A,0x20);
#else
    /* Réglage historique : protège la régularité des points HW, au prix de
     * préemptions possibles de Timer4. */
    IntPrioritySet(INT_TIMER5A,0x00);
    IntPrioritySet(INT_TIMER6A,0x00);
    IntPrioritySet(INT_TIMER7A,0x00);
    IntPrioritySet(INT_TIMER4A,0x20);
#endif
    IntPrioritySet(INT_GPIOL,0x40);
    IntPrioritySet(INT_UART0,0xC0);

    ////////////////////////////////////////////////
    //
    // Initialize the DDSs
    //
    ////////////////////////////////////////////////

    start_DDSSequence();


    ////////////////////////////////////////////////


    while(1)
    {
        /* RESET has already been acknowledged by UARTIntHandler. Perform a
         * physical, race-free reset here. The previous order programmed FTW=0
         * while sequence timers/GPIO triggers were still allowed to run, and it
         * never forced ASF=0. A pending sequence event could therefore overwrite
         * the reset immediately and the RF output remained visible. */
        if (g_seq_reset_pending) {
            uint32_t reset_t0;

            IntMasterDisable();
            reset_t0 = SEQ_DWT_CYCCNT;
            g_seq_reset_pending = false;
            g_reset_hard_begin_count++;

            /* Stop both runtimes before touching DDS registers.  V34 tables are
             * invalidated as part of RESET, so LOAD + ARM are deliberately required again. */
            Tbl34_ResetConfiguration();
            Tbl34_Service();
            g_v34_table_mode = false;
            Seq_ResetAllChannels();
            Engine_ConfigureTimersForGeneral();
            g_seq_pending_trigger_flags = 0U;
            HWREG(GPIO_PORTL_BASE + GPIO_O_ICR) = 0x07U; /* TTL6/7/8 */
            HWREG(GPIO_PORTL_BASE + GPIO_O_IM) |= 0x07U;

            g_reset_hard_last_ticks = (uint32_t)(SEQ_DWT_CYCCNT - reset_t0);
            g_reset_hard_end_count++;
            IntMasterEnable();
        }

        /*
         * V33.2.1: ARM preload is performed outside UARTIntHandler.  ACK is
         * sent only after the DDS register preload has completed.  A bounded
         * SSI failure returns NAK instead of leaving the GUI waiting forever.
         */
        if (g_v34_table_mode && Tbl34_ArmAckPending()) {
            bool preload_ok = Tbl34_ServiceArm();
            UARTCharPut(UART0_BASE, preload_ok ? 0x06 : 0x15);
            if (preload_ok) g_uart_ack_count++;
        } else if (!g_v34_table_mode && g_seq_arm_ack_pending) {
            bool preload_ok;
            uint32_t preload_t0 = SEQ_DWT_CYCCNT;

            g_seq_dbg_arm_preload_begin_count++;
            preload_ok = Seq_ServicePreloadRequest();
            g_seq_dbg_arm_preload_last_ticks =
                (unsigned long)(SEQ_DWT_CYCCNT - preload_t0);
            g_seq_dbg_arm_preload_end_count++;

            UARTCharPut(UART0_BASE, preload_ok ? 0x06 : 0x15);
            if (preload_ok) {
                g_uart_ack_count++;
                g_seq_dbg_arm_ack_count++;
            } else {
                g_seq_dbg_arm_nak_count++;
            }
            g_seq_arm_ack_pending = false;
        } else if (!g_v34_table_mode) {
            (void)Seq_ServicePreloadRequest();
        }

        /* Deferred housekeeping.  The active engine exclusively owns the
         * sequence timers and trigger semantics. */
        if (g_v34_table_mode) Tbl34_Service();
        else {
            Seq_ServiceProfilePwmRestore();
            Seq_ServicePendingTriggers();
        }

#ifdef RUN_IO_CONTROLS
       while(!g_ulFlags)
       {
            HWREGBITW(&g_ulFlags, FLAG_TICK) = 0;
       }
#endif
   }

}
