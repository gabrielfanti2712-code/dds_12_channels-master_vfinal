/*
 * Moteur de tables DDS — firmware V36.5
 *
 * ROLE
 * ----
 * Ce module joue les séquences compilées sur le PC. Une table est construite
 * pour chacun des trois AD9959. Chaque événement contient :
 *   - une deadline absolue en ticks du TM4C ;
 *   - un bloc d'octets déjà calculés pour les registres AD9959 ;
 *   - l'action à effectuer à la deadline (I/O_UPDATE, sweep, gate, FAST...).
 *
 * PRINCIPE TEMPS REEL
 * -------------------
 * 1. Précharger les registres DDS avant la deadline quand c'est possible.
 * 2. Attendre la deadline absolue avec le compteur DWT.
 * 3. Valider les registres avec I/O_UPDATE / PROFILE.
 * 4. Passer à l'événement suivant sans recalculer la séquence dans l'ISR.
 *
 * SYNCHRONISATION MULTI-DDS (V36.4, conservée en V36.5)
 * --------------------------------
 * Quand plusieurs DDS ont la même deadline, le premier réveil précharge les
 * DDS concernés avant la date cible, puis les commits sont exécutés à la suite
 * sans transfert SSI lourd entre eux. Cette stratégie privilégie le faible
 * écart relatif DDS1/DDS2/DDS3 plutôt qu'une latence TTL->RF minimale.
 *
 * MEMOIRE
 * -------
 * Les optimisations SRAM des V35+ sont conservées : pas de bitmap d'événements
 * reçus (blob_len=0xFF signifie « non reçu ») et RESET_PREPARE référence le
 * blob déjà présent dans l'arène au lieu d'en faire une copie.
 */
#include "seq_table_engine.h"

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#ifndef TIMER_6A_SYNC
#define TIMER_6A_SYNC 0x00001000UL
#endif
#ifndef TIMER_7A_SYNC
#define TIMER_7A_SYNC 0x00004000UL
#endif

#ifndef TIMER_TIMA_MATCH
#define TIMER_TIMA_MATCH 0x00000010UL
#endif

#define TBL34_DWT_CTRL      HWREG(0xE0001000UL)
#define TBL34_DWT_CYCCNT    HWREG(0xE0001004UL)
#define TBL34_DEMCR         HWREG(0xE000EDFCUL)

#define TBL34_CSR_ADDR      0x00U
#define TBL34_CFR_ADDR      0x03U
#define TBL34_FTW_ADDR      0x04U
#define TBL34_CPOW_ADDR     0x05U
#define TBL34_ACR_ADDR      0x06U

#define TBL34_INITIAL_GUARD_TICKS 120UL /* 1.0 us, common origin for all DDS. */

/* V36: descending native sweeps no longer schedule a separate 0.4-us table
 * event. The generic table ISR itself costs ~17 us in the T4/T5 validation, so
 * such an event could never be serviced on time. SWEEP_START now performs the
 * PROFILE HIGH -> 0.4-us hold -> PROFILE LOW transition locally. */
#define TBL36_DESC_PROFILE_HIGH_HOLD_TICKS 48UL

/* V36.4 timing policy.
 *
 * Short/special gaps keep the historical inline preload path.  Ordinary
 * long-gap events are marked pending and their own Timer4/5/6 one-shot wakes
 * early enough to preload all DDSs that share the same absolute deadline.
 *
 * Bench maxima before V36.4 were roughly:
 *   DDS1 preload <= 2222 ticks, DDS2 <= 3043, DDS3 <= 1565.
 * Their sum is ~6830 ticks (56.9 us).  A 9000-tick / 75-us early wake leaves
 * useful margin while remaining much shorter than the usual 100..500-us
 * experiment items.  Events with less than 80 us of remaining slack are never deferred.
 */
#define TBL364_EARLY_WAKE_LEAD_TICKS     9000UL /* 75 us */
#define TBL364_DEFER_MIN_SLACK_TICKS     9600UL /* 80 us: 5-us scheduling margin */
#define TBL364_SYNC_DEADLINE_TOL_TICKS      2UL /* <= 16.7 ns: same compiled boundary */

/* Existing main firmware initializers, used only to restore Timer2/3 after the
 * DDS3 PROFILE PWM has been stopped. */
extern void initTimer2(void);
extern void initTimer3(void);

const char g_tbl34_build_id[] = "DDS-V36.5-MEMOPT3-20260819-A";

static tbl34_event_t g_tbl34_ev[TBL34_DDS_COUNT][TBL34_MAX_EVENTS];
static uint8_t g_tbl34_blob[TBL34_DDS_COUNT][TBL34_ARENA_BYTES];
static tbl34_def_t g_tbl34_def[TBL34_DDS_COUNT];
static volatile tbl34_rt_t g_tbl34_rt[TBL34_DDS_COUNT];

static tbl34_event_t g_tbl34_gate_ev[TBL34_MAX_EVENTS];
static uint8_t g_tbl34_gate_blob[TBL34_ARENA_BYTES];
static tbl34_gate_def_t g_tbl34_gate_def;

/* V36.5: true secondary-TTL GATE and ordinary AUTO_START periodic FAST are
 * separate LOGICAL resources, but they share the existing special-stream RAM
 * pool. This is crucial on the current project: SRAM is almost full. The gate
 * occupies the beginning of g_tbl34_gate_ev/blob and the AUTO_START FAST table
 * is packed immediately after it. No second 1024-event / 8-kB arena is added. */
static tbl34_gate_def_t g_tbl365_fast_def;
static uint16_t g_tbl365_fast_event_base = 0U;
static uint16_t g_tbl365_fast_blob_base = 0U;

#define TBL365_FAST_SOURCE_NONE 0U
#define TBL365_FAST_SOURCE_GATE 1U
#define TBL365_FAST_SOURCE_AUTO 2U
static volatile uint8_t g_tbl365_fast_source = TBL365_FAST_SOURCE_NONE;
static volatile bool g_tbl365_auto_fast_done = true;
static volatile bool g_tbl365_auto_fast_running = false;
static volatile bool g_tbl365_auto_fast_event0_prepared = false;
static volatile uint32_t g_tbl365_auto_fast_origin = 0U;
/* If AUTO FAST temporarily owns a DDS timer while the merged main table has
 * later events, keep the next main index in g_tbl34_rt[d].idx and remember only
 * one resume bit. This costs one byte of SRAM and avoids duplicating events. */
static volatile uint8_t g_tbl365_main_resume_mask = 0U;

/* V35 memory optimization: no 512-byte received-event bitmaps.
 * blob_len is constrained to <= TBL34_MAX_BLOB_PER_EVENT (200), therefore
 * 0xFF can safely mark an event slot that has not yet been received during
 * the current LOAD.  The marker is private to RAM and never goes on the wire. */
#define TBL34_EVENT_UNSEEN_BLOB_LEN 0xFFU

static volatile tbl34_group_rt_t g_tbl34_group[3];
static volatile bool g_tbl34_armed = false;
static volatile bool g_tbl34_arm_pending = false;
static volatile uint8_t g_tbl34_policy = 0U;
static volatile uint8_t g_tbl34_priority[3] = {0U, 1U, 2U};
static volatile uint8_t g_tbl34_current_start_ttl = 0xFFU;
static volatile uint8_t g_tbl34_masked_main_ttl_bits = 0U;

/* V36.4 early-preload state. One bit per DDS. The event index itself stays
 * in g_tbl34_rt[d].idx, so the SRAM optimization is preserved: no duplicate
 * event pointer, blob copy or received-event bitmap is introduced. */
static volatile uint8_t g_tbl362_main_preload_pending_mask = 0U;

static volatile bool g_tbl34_gate_armed = false;
static volatile bool g_tbl34_gate_running = false;
static volatile bool g_tbl34_gate_done = true;
static volatile bool g_tbl34_gate_event0_prepared = false;
static volatile uint16_t g_tbl34_gate_idx = 0U;
static volatile uint32_t g_tbl34_gate_origin = 0U;

/* Dedicated fast gated staircase. */
static volatile bool g_tbl34_fast_active = false;
static volatile uint8_t g_tbl34_fast_dds = 0xFFU;
static volatile uint16_t g_tbl34_fast_idx = 0U;
static volatile uint32_t g_tbl34_fast_period_ticks = 0U;
static volatile uint32_t g_tbl34_fast_next_deadline = 0U;


static tbl34_gate_def_t *Tbl365_ActiveFastDef(void)
{
    return (g_tbl365_fast_source == TBL365_FAST_SOURCE_AUTO)
        ? &g_tbl365_fast_def : &g_tbl34_gate_def;
}
static tbl34_event_t *Tbl365_AutoFastEvents(void)
{
    return &g_tbl34_gate_ev[g_tbl365_fast_event_base];
}
static uint8_t *Tbl365_AutoFastBlob(void)
{
    return &g_tbl34_gate_blob[g_tbl365_fast_blob_base];
}
static tbl34_event_t *Tbl365_ActiveFastEvents(void)
{
    return (g_tbl365_fast_source == TBL365_FAST_SOURCE_AUTO)
        ? Tbl365_AutoFastEvents() : g_tbl34_gate_ev;
}
static uint8_t *Tbl365_ActiveFastBlob(void)
{
    return (g_tbl365_fast_source == TBL365_FAST_SOURCE_AUTO)
        ? Tbl365_AutoFastBlob() : g_tbl34_gate_blob;
}
static uint32_t Tbl365_ActiveFastOrigin(void)
{
    return (g_tbl365_fast_source == TBL365_FAST_SOURCE_AUTO)
        ? g_tbl365_auto_fast_origin : g_tbl34_gate_origin;
}
static void Tbl365_SetActiveFastOrigin(uint32_t origin)
{
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_AUTO)
        g_tbl365_auto_fast_origin = origin;
    else
        g_tbl34_gate_origin = origin;
}
static bool Tbl365_ActiveFastEvent0Prepared(void)
{
    return (g_tbl365_fast_source == TBL365_FAST_SOURCE_AUTO)
        ? g_tbl365_auto_fast_event0_prepared : g_tbl34_gate_event0_prepared;
}
static void Tbl365_SetActiveFastEvent0Prepared(bool prepared)
{
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_AUTO)
        g_tbl365_auto_fast_event0_prepared = prepared;
    else
        g_tbl34_gate_event0_prepared = prepared;
}

/* V35.3: validated uniform single-register microstream.
 * Event 0 is committed normally. Event 1 is queued before the periodic timer
 * starts. Each timeout then performs only I/O_UPDATE + queue next frame. */
static volatile bool g_tbl352_ultrafast_prepared = false;
static volatile uint16_t g_tbl352_ultrafast_fast_count = 0U;
static volatile uint8_t g_tbl352_ultrafast_frame_len = 0U;
static volatile uint8_t g_tbl352_ultrafast_frame_addr = 0U;
static const uint8_t *g_tbl352_ultrafast_next_frame = (const uint8_t *)0;
static volatile unsigned long g_tbl352_ultrafast_ssi_base = 0UL;
static volatile unsigned long g_tbl352_ultrafast_ioupdate_addr = 0UL;
static volatile uint32_t g_tbl352_ultrafast_ioupdate_mask = 0U;

/* Tiny runtime touched by the hot ISR. */
static volatile bool g_tbl353_direct_active = false;
static volatile uint16_t g_tbl353_direct_remaining = 0U;
static volatile uint16_t g_tbl353_direct_total_irqs = 0U;
static const uint8_t *g_tbl353_direct_frame = (const uint8_t *)0;
static volatile uint32_t g_tbl353_direct_start_tick = 0U;

/* V35.4: expensive gate validation/layout discovery is cached at ARM time. */
static volatile bool g_tbl354_fast_gate_validated = false;
static volatile bool g_tbl354_layout_cached = false;
static volatile uint32_t g_tbl354_gate_arm_commit_tick = 0U;

/* V35.6 hot-path state; bytes only except the captured fast-end tick. */
static volatile bool g_tbl356_auto_event0_committed = false;
static volatile bool g_tbl356_triangle_exit_pending = false;
static volatile uint32_t g_tbl356_direct_fast_end_tick = 0U;

/* DDS3 PROFILE PWM state. */
static volatile bool g_tbl34_profile_pwm_active = false;
static volatile unsigned long g_tbl34_profile_pwm_timer_base = 0UL;
static volatile unsigned int g_tbl34_profile_pwm_timer_half = TIMER_A;
static volatile unsigned char g_tbl34_profile_pwm_pin = 0U;
static volatile unsigned long g_tbl34_profile_restore_pending_base = 0UL;

/* Fast timing hardware is fully configured at GATE_ARM and only enabled by
 * the secondary TTL.  This keeps the first ~2-us dwell from absorbing costly
 * DriverLib TimerConfigure calls. */
static volatile bool g_tbl34_fast_timing_prepared = false;
static volatile bool g_tbl34_fast_timing_triangle = false;
static volatile unsigned long g_tbl34_fast_boundary_base = 0UL;
static volatile unsigned long g_tbl34_fast_pwm_base = 0UL;
static volatile unsigned int g_tbl34_fast_pwm_half = TIMER_A;
static volatile unsigned long g_tbl34_fast_pwm_sync_mask = 0UL;
static volatile unsigned long g_tbl34_fast_boundary_sync_mask = 0UL;
static volatile unsigned long g_tbl34_fast_pwm_pin_config = 0UL;
static volatile unsigned char g_tbl34_fast_pwm_pin = 0U;

/* V35 MEMOPT2: no second RESET_PREPARE image.
 * The validated restore bytes remain in g_tbl34_blob[] / g_tbl34_gate_blob
 * throughout the armed execution and are referenced directly. */
#define TBL35_CANONICAL_CHANNEL_BYTES 18U

/* V35: a reset restore image may contain all four channels of one AD9959.
 * Each channel contributes the canonical 18-byte CSR/CFR/FTW/CPOW/ACR state.
 *
 * Memory optimization: the RESET_PREPARE blob already resides in the static
 * main/gate arena for the whole armed execution.  Keep a pointer to that
 * validated immutable blob instead of copying up to 72 bytes per DDS into a
 * second RAM buffer. */
#define TBL35_RESET_IMAGE_MAX_BYTES (4U * TBL35_CANONICAL_CHANNEL_BYTES)
static const uint8_t *g_tbl35_reset_prepared_source[TBL34_DDS_COUNT] = {
    (const uint8_t *)0, (const uint8_t *)0, (const uint8_t *)0
};
static volatile uint8_t g_tbl35_reset_prepared_len[TBL34_DDS_COUNT] = {0U,0U,0U};
static volatile bool g_tbl35_reset_prepared_ready[TBL34_DDS_COUNT] = {false,false,false};

/* Diagnostics: intentionally not reset at sequence start. */
volatile uint32_t g_tbl34_dbg_ttl_seen[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_start_accepted[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_start_ignored_active[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_start_not_ready[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_gate_seen[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_gate_release_accepted = 0U;
volatile uint32_t g_tbl34_dbg_gate_too_early = 0U;
volatile uint32_t g_tbl34_dbg_gate_wrong_ttl[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_sequence_started = 0U;
volatile uint32_t g_tbl34_dbg_sequence_completed = 0U;
volatile uint32_t g_tbl34_dbg_sequence_aborted = 0U;
volatile uint32_t g_tbl34_dbg_commit_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_isr_ticks[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_isr_max_ticks[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_late_ticks[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_late_max_ticks[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_preload_ticks[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_preload_max_ticks[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_preload_error[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_fast_points_committed = 0U;
volatile uint32_t g_tbl34_dbg_fast_period_ticks = 0U;
volatile uint32_t g_tbl34_dbg_fast_isr_ticks = 0U;
volatile uint32_t g_tbl34_dbg_fast_isr_max_ticks = 0U;
volatile uint32_t g_tbl34_dbg_fast_late_max_ticks = 0U;
volatile uint32_t g_tbl34_dbg_profile_pwm_start_count = 0U;
volatile uint32_t g_tbl34_dbg_profile_pwm_stop_count = 0U;
volatile uint32_t g_tbl34_dbg_profile_pwm_sync_ticks = 0U;
volatile uint32_t g_tbl34_dbg_arm_request_count = 0U;
volatile uint32_t g_tbl34_dbg_arm_ack_count = 0U;
volatile uint32_t g_tbl34_dbg_arm_nak_count = 0U;
volatile uint32_t g_tbl34_dbg_arm_preload_last_ticks = 0U;
volatile uint32_t g_tbl34_dbg_last_gate_release_ticks = 0U;
volatile uint32_t g_tbl34_dbg_main_trigger_masked[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_sweep_start_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_sweep_exit_count[3] = {0U,0U,0U};

/* V34.22: low-latency chip-level MASTER_RESET + exact table-blob restore.
 * The legacy reset_recovery_* and v3421_* names are kept so existing CCS
 * watch expressions remain useful while the new v3422_* counters expose the
 * fast serial-mode path in detail. */
volatile uint32_t g_tbl34_dbg_reset_recovery_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_reset_recovery_error[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_reset_recovery_master_high_ticks[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_reset_recovery_single_init_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_reset_recovery_quad_init_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_reset_recovery_program_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_reset_recovery_update_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_reset_recovery_total_ticks[3] = {0U,0U,0U};

/* V34.23: recovery blob is parsed in RAM and re-emitted in canonical
 * CSR -> CFR -> FTW -> CPOW -> ACR order.  This fixes merged-event blobs
 * whose compiler order starts with CPOW/ACR because of last-write-wins. */

/* V34.24: canonical restore image is prepared before the reset boundary. */

/* V34.25 explicit event-state and hardened-load diagnostics. */
volatile uint32_t g_tbl34_dbg_event_crc_error[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_event_duplicate_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl34_dbg_gate_event_crc_error = 0U;
volatile uint32_t g_tbl34_dbg_gate_event_duplicate_count = 0U;

/* V35 persistent diagnostics. */
volatile uint32_t g_tbl35_dbg_reset_prepare_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_reset_prepare_error[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_reset_prepare_bytes[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_reset_restore_channel_mask[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_reset_commit_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_reset_commit_error[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_reset_blackout_ticks[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_reset_restore_bytes[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_native_freq_start_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_native_freq_exit_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_native_phase_start_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_native_phase_exit_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_native_amp_start_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_native_amp_exit_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_gate_arm_count = 0U;
volatile uint32_t g_tbl35_dbg_auto_fast_start_count = 0U;
volatile uint32_t g_tbl35_dbg_fast_start_attempt_count = 0U;
volatile uint32_t g_tbl35_dbg_fast_start_ok_count = 0U;
volatile uint32_t g_tbl35_dbg_fast_start_fail_code = 0U;
volatile uint32_t g_tbl35_dbg_fast_timer_irq_count = 0U;
volatile uint32_t g_tbl35_dbg_fast_update_count = 0U;
volatile uint32_t g_tbl35_dbg_fast_preload_count = 0U;
volatile uint32_t g_tbl35_dbg_fast_preload_error_count = 0U;
volatile uint32_t g_tbl35_dbg_fast_final_count = 0U;
volatile uint32_t g_tbl35_dbg_fast_to_generic_count = 0U;
volatile uint32_t g_tbl35_dbg_triangle_config_count = 0U;
volatile uint32_t g_tbl35_dbg_triangle_config_error_count = 0U;
volatile uint32_t g_tbl35_dbg_triangle_period_ticks = 0U;
volatile uint32_t g_tbl35_dbg_triangle_half_ticks = 0U;
volatile uint32_t g_tbl35_dbg_triangle_pwm_load = 0U;
volatile uint32_t g_tbl35_dbg_triangle_pwm_match = 0U;
volatile uint32_t g_tbl35_dbg_event_cfr_frame_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_event_ftw_frame_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_event_cpow_frame_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl35_dbg_event_acr_frame_count[3] = {0U,0U,0U};

/* V35.1 diagnostics. */
volatile uint32_t g_tbl351_dbg_fast_handoff_count = 0U;
volatile uint32_t g_tbl351_dbg_fast_handoff_timer_kept_count = 0U;
volatile uint32_t g_tbl351_dbg_fast_handoff_error_count = 0U;
volatile uint32_t g_tbl351_dbg_fast_expected_events = 0U;
volatile uint32_t g_tbl351_dbg_fast_last_index = 0U;
volatile uint32_t g_tbl351_dbg_fast_ftw_frame_count = 0U;
volatile uint32_t g_tbl351_dbg_fast_cpow_frame_count = 0U;
volatile uint32_t g_tbl351_dbg_fast_acr_frame_count = 0U;
volatile uint32_t g_tbl351_dbg_native_desc_start_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl351_dbg_native_desc_profile_low_count[3] = {0U,0U,0U};
volatile uint32_t g_tbl351_dbg_phase_last_csr[3] = {0U,0U,0U};
volatile uint32_t g_tbl351_dbg_phase_last_cfr[3] = {0U,0U,0U};
volatile uint32_t g_tbl351_dbg_phase_last_cpow[3] = {0U,0U,0U};
volatile uint32_t g_tbl351_dbg_phase_last_cw1[3] = {0U,0U,0U};
volatile uint32_t g_tbl351_dbg_phase_last_rdw[3] = {0U,0U,0U};
volatile uint32_t g_tbl351_dbg_phase_last_fdw[3] = {0U,0U,0U};
volatile uint32_t g_tbl351_dbg_phase_last_lsrr[3] = {0U,0U,0U};
volatile uint32_t g_tbl351_dbg_phase_last_ftw[3] = {0U,0U,0U};
volatile uint32_t g_tbl351_dbg_phase_last_acr[3] = {0U,0U,0U};

/* V35.3 compact diagnostics. No volatile diagnostic is updated on every
 * staircase step; aggregate timing is published once at the end. */
volatile uint32_t g_tbl353_dbg_layout_prepare_count = 0U;
volatile uint32_t g_tbl353_dbg_layout_prepare_error_count = 0U;
volatile uint32_t g_tbl353_dbg_direct_used_count = 0U;
volatile uint32_t g_tbl353_dbg_direct_stream_addr = 0U;
volatile uint32_t g_tbl353_dbg_direct_stride = 0U;
volatile uint32_t g_tbl353_dbg_direct_chain_events = 0U;
volatile uint32_t g_tbl353_dbg_direct_layout_error_code = 0U;
volatile uint32_t g_tbl353_dbg_direct_timer_tailr = 0U;
volatile uint32_t g_tbl353_dbg_direct_expected_irqs = 0U;
volatile uint32_t g_tbl353_dbg_direct_irqs_done = 0U;
volatile uint32_t g_tbl353_dbg_direct_expected_runtime_ticks = 0U;
volatile uint32_t g_tbl353_dbg_direct_runtime_ticks = 0U;
volatile uint32_t g_tbl353_dbg_direct_excess_ticks = 0U;
volatile uint32_t g_tbl353_dbg_direct_average_period_ticks = 0U;
volatile uint32_t g_tbl353_dbg_direct_final_count = 0U;
volatile uint32_t g_tbl353_dbg_direct_to_generic_count = 0U;
volatile uint32_t g_tbl353_dbg_direct_fallback_count = 0U;

/* V35.4 compact diagnostics; none is updated on every periodic IRQ. */
volatile uint32_t g_tbl354_dbg_start_latency_ticks = 0U;
volatile uint32_t g_tbl354_dbg_gate_timer_rearm_count = 0U;
volatile uint32_t g_tbl354_dbg_gate_timer_imr_after_rearm = 0U;
volatile uint32_t g_tbl354_dbg_triangle_direct_count = 0U;
volatile uint32_t g_tbl354_dbg_triangle_exit_count = 0U;
volatile uint32_t g_tbl354_dbg_triangle_exit_late_ticks = 0U;

/* V35.5 compact diagnostics. */
volatile uint32_t g_tbl355_dbg_triangle_sync_count = 0U;
volatile uint32_t g_tbl355_dbg_triangle_sync_mask = 0U;

/* V35.6 compact diagnostics. */
volatile uint32_t g_tbl356_dbg_auto_reuse_count = 0U;
volatile uint32_t g_tbl358_dbg_triangle_lsrr = 0U;
volatile uint32_t g_tbl356_dbg_triangle_no_ioupdate_srr_count = 0U;
volatile uint32_t g_tbl356_dbg_triangle_cfr = 0U;
volatile uint32_t g_tbl356_dbg_triangle_exit_preload_count = 0U;
volatile uint32_t g_tbl356_dbg_triangle_exit_preload_ticks = 0U;
volatile uint32_t g_tbl356_dbg_triangle_exit_irq_count = 0U;
volatile uint32_t g_tbl356_dbg_triangle_exit_preload_error = 0U;

/* Descending native-sweep diagnostic retained because it is still useful on
 * the bench: measured PROFILE-HIGH hold before switching to PROFILE-LOW. */
volatile uint32_t g_tbl36_dbg_native_desc_high_hold_ticks = 0U;

volatile uint32_t g_tbl362_dbg_deferred_preload_count = 0U;
volatile uint32_t g_tbl362_dbg_service_preload_count = 0U;
volatile uint32_t g_tbl362_dbg_emergency_preload_count = 0U;
volatile uint32_t g_tbl362_dbg_inline_short_preload_count = 0U;

/* Current build marker. Older version-signature variables were removed in the
 * V36.5 handover cleanup; use this single signature in CCS. */
volatile uint32_t g_tbl365_signature = 0x56333605UL; /* 1446196741 */

/* V36.4 early-wake synchronized-commit diagnostics, still active in V36.5. */
volatile uint32_t g_tbl364_dbg_early_preload_irq_count = 0U;
volatile uint32_t g_tbl364_dbg_sync_group_count = 0U;
volatile uint32_t g_tbl364_dbg_sync_last_mask = 0U;
volatile uint32_t g_tbl364_dbg_sync_last_skew_ticks = 0U;
volatile uint32_t g_tbl364_dbg_sync_max_skew_ticks = 0U;
volatile uint32_t g_tbl364_dbg_preload_batch_ticks = 0U;
volatile uint32_t g_tbl364_dbg_preload_batch_max_ticks = 0U;
volatile uint32_t g_tbl364_dbg_preload_batch_late_count = 0U;

/* Private capture state: set only while a synchronized group is committed. */
static volatile uint8_t g_tbl364_sync_capture_mask = 0U;
static volatile uint32_t g_tbl364_sync_update_tick[3] = {0U,0U,0U};

volatile uint32_t g_tbl358_dbg_triangle_rdw = 0U;
volatile uint32_t g_tbl358_dbg_triangle_fdw = 0U;
volatile uint32_t g_tbl357_dbg_triangle_exit_commit_count = 0U;

static inline uint16_t Tbl34_Rd16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t Tbl34_Rd32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static uint32_t Tbl34_Crc32(const uint8_t *data, unsigned int len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    unsigned int i;
    while (len-- != 0U) {
        crc ^= (uint32_t)(*data++);
        for (i = 0U; i < 8U; i++) {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static uint32_t Tbl34_Crc32Byte(uint32_t crc, uint8_t b)
{
    unsigned int k;
    crc ^= (uint32_t)b;
    for (k = 0U; k < 8U; k++) {
        uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
        crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
    return crc;
}

static uint32_t Tbl34_Crc32EventsCanonical(const tbl34_event_t *ev, unsigned int n)
{
    uint32_t crc = 0xFFFFFFFFUL;
    unsigned int i;
    for (i = 0U; i < n; i++) {
        const tbl34_event_t *e = &ev[i];
        uint8_t w[12];
        unsigned int k;
        w[0]=(uint8_t)(e->at_ticks); w[1]=(uint8_t)(e->at_ticks>>8);
        w[2]=(uint8_t)(e->at_ticks>>16); w[3]=(uint8_t)(e->at_ticks>>24);
        w[4]=(uint8_t)(e->blob_offset); w[5]=(uint8_t)(e->blob_offset>>8);
        w[6]=e->blob_len; w[7]=e->action; w[8]=e->profile_ops; w[9]=e->reserved;
        w[10]=(uint8_t)(e->aux); w[11]=(uint8_t)(e->aux>>8);
        for (k=0U;k<12U;k++) crc=Tbl34_Crc32Byte(crc,w[k]);
    }
    return ~crc;
}

static void Tbl34_MarkEventSlotsUnseen(tbl34_event_t *events, unsigned int count)
{
    unsigned int i;
    if (events == (tbl34_event_t *)0 || count > TBL34_MAX_EVENTS) return;
    for (i = 0U; i < count; i++) events[i].blob_len = TBL34_EVENT_UNSEEN_BLOB_LEN;
}


static inline unsigned long Tbl34_TimerBase(unsigned int d)
{
    if (d == 0U) return TIMER4_BASE;
    if (d == 1U) return TIMER5_BASE;
    return TIMER6_BASE;
}

static inline uint32_t Tbl34_TimerInterrupt(unsigned int d)
{
    if (d == 0U) return INT_TIMER4A;
    if (d == 1U) return INT_TIMER5A;
    return INT_TIMER6A;
}

static inline unsigned long Tbl34_SsiBase(unsigned int d)
{
    if (d == 0U) return SSI0_BASE;
    if (d == 1U) return SSI1_BASE;
    return SSI2_BASE;
}

static inline unsigned long Tbl34_ProfilePortBase(unsigned int d)
{
    if (d == 0U) return GPIO_PORTH_BASE;
    if (d == 1U) return GPIO_PORTP_BASE;
    return GPIO_PORTM_BASE;
}

static inline uint8_t Tbl34_ProfilePinMask(unsigned int d, unsigned int c)
{
    if (c >= 4U) return 0U;
    if (d == 0U) return (uint8_t)(0x08U >> c); /* PH3..PH0 */
    if (d == 1U) return (uint8_t)(0x20U >> c); /* PP5..PP2 */
    if (d == 2U) return (uint8_t)(0x08U >> c); /* PM3..PM0 */
    return 0U;
}

static inline uint8_t Tbl34_TtlPinMask(unsigned int ttl_index)
{
    if (ttl_index == 0U) return GPIO_PIN_2; /* TTL6 / PL2 */
    if (ttl_index == 1U) return GPIO_PIN_1; /* TTL7 / PL1 */
    if (ttl_index == 2U) return GPIO_PIN_0; /* TTL8 / PL0 */
    return 0U;
}

static void Tbl34_ProfileLow(unsigned int d, unsigned int c)
{
    unsigned long base = Tbl34_ProfilePortBase(d);
    uint8_t pin = Tbl34_ProfilePinMask(d,c);
    if (pin == 0U) return;
    HWREG(base + GPIO_O_DATA + ((unsigned long)pin << 2)) = 0U;
}

static void Tbl34_ProfileHigh(unsigned int d, unsigned int c)
{
    unsigned long base = Tbl34_ProfilePortBase(d);
    uint8_t pin = Tbl34_ProfilePinMask(d,c);
    if (pin == 0U) return;
    HWREG(base + GPIO_O_DATA + ((unsigned long)pin << 2)) = pin;
}

static void Tbl34_ApplyProfileLowMask(unsigned int d, uint8_t profile_ops)
{
    uint8_t mask = (uint8_t)((profile_ops >> 4) & 0x0FU);
    unsigned int c;
    for (c = 0U; c < 4U; c++) if ((mask & (1U << c)) != 0U) Tbl34_ProfileLow(d,c);
}

static void Tbl34_ApplyProfileHighMask(unsigned int d, uint8_t profile_ops)
{
    uint8_t mask = (uint8_t)(profile_ops & 0x0FU);
    unsigned int c;
    for (c = 0U; c < 4U; c++) if ((mask & (1U << c)) != 0U) Tbl34_ProfileHigh(d,c);
}

static inline void Tbl34_IOUpdateOne(unsigned int d)
{
    /* V34.5: give I/O_UPDATE comfortable margin above the AD9959 minimum
     * one-SYNC_CLK pulse width.  Three Cortex-M4 NOPs are ~25 ns @120 MHz. */
    if (d == 0U) {
        HWREG(GPIO_PORTK_BASE + GPIO_O_DATA + (0x80UL << 2)) = 0x80U;
        if ((g_tbl364_sync_capture_mask & 0x01U) != 0U) g_tbl364_sync_update_tick[0] = TBL34_DWT_CYCCNT;
        __asm(" NOP"); __asm(" NOP"); __asm(" NOP");
        HWREG(GPIO_PORTK_BASE + GPIO_O_DATA + (0x80UL << 2)) = 0U;
    } else if (d == 1U) {
        HWREG(GPIO_PORTN_BASE + GPIO_O_DATA + (0x08UL << 2)) = 0x08U;
        if ((g_tbl364_sync_capture_mask & 0x02U) != 0U) g_tbl364_sync_update_tick[1] = TBL34_DWT_CYCCNT;
        __asm(" NOP"); __asm(" NOP"); __asm(" NOP");
        HWREG(GPIO_PORTN_BASE + GPIO_O_DATA + (0x08UL << 2)) = 0U;
    } else {
        HWREG(GPIO_PORTN_BASE + GPIO_O_DATA + (0x01UL << 2)) = 0x01U;
        if ((g_tbl364_sync_capture_mask & 0x04U) != 0U) g_tbl364_sync_update_tick[2] = TBL34_DWT_CYCCNT;
        __asm(" NOP"); __asm(" NOP"); __asm(" NOP");
        HWREG(GPIO_PORTN_BASE + GPIO_O_DATA + (0x01UL << 2)) = 0U;
    }
}

/* Recovery-only I/O_UPDATE.  The normal path stays fast, but native
 * frequency-sweep exit deliberately holds UPDATE high for 1 us so pulse
 * width/sampling by SYNC_CLK cannot be the limiting factor during diagnosis. */
static inline void Tbl34_IOUpdateOneWide(unsigned int d, uint32_t high_ticks)
{
    uint32_t t0;
    if (d == 0U) {
        HWREG(GPIO_PORTK_BASE + GPIO_O_DATA + (0x80UL << 2)) = 0x80U;
        t0 = TBL34_DWT_CYCCNT;
        while ((uint32_t)(TBL34_DWT_CYCCNT - t0) < high_ticks) { }
        HWREG(GPIO_PORTK_BASE + GPIO_O_DATA + (0x80UL << 2)) = 0U;
    } else if (d == 1U) {
        HWREG(GPIO_PORTN_BASE + GPIO_O_DATA + (0x08UL << 2)) = 0x08U;
        t0 = TBL34_DWT_CYCCNT;
        while ((uint32_t)(TBL34_DWT_CYCCNT - t0) < high_ticks) { }
        HWREG(GPIO_PORTN_BASE + GPIO_O_DATA + (0x08UL << 2)) = 0U;
    } else {
        HWREG(GPIO_PORTN_BASE + GPIO_O_DATA + (0x01UL << 2)) = 0x01U;
        t0 = TBL34_DWT_CYCCNT;
        while ((uint32_t)(TBL34_DWT_CYCCNT - t0) < high_ticks) { }
        HWREG(GPIO_PORTN_BASE + GPIO_O_DATA + (0x01UL << 2)) = 0U;
    }
}

static void Tbl34_WaitSsiIdle(unsigned int d)
{
    unsigned long base = Tbl34_SsiBase(d);
    while ((HWREG(base + SSI_O_SR) & SSI_SR_BSY) != 0U) { }
}

static void Tbl34_PutByte(unsigned int d, uint8_t value)
{
    unsigned long base = Tbl34_SsiBase(d);
    while ((HWREG(base + SSI_O_SR) & SSI_SR_TNF) == 0U) { }
    HWREG(base + SSI_O_DR) = value;
}

static unsigned int Tbl34_RegisterPayloadBytes(uint8_t addr)
{
    switch (addr & 0x7FU) {
    case 0x00U: return 1U; /* CSR */
    case 0x03U: return 3U; /* CFR */
    case 0x04U: return 4U; /* FTW */
    case 0x05U: return 2U; /* CPOW */
    case 0x06U: return 3U; /* ACR */
    case 0x07U: return 2U; /* LSRR */
    case 0x08U: return 4U; /* RDW */
    case 0x09U: return 4U; /* FDW */
    case 0x0AU: return 4U; /* CW1 */
    default: return 0U;
    }
}

static bool Tbl34_PushBlob(unsigned int d, const uint8_t *p, unsigned int n)
{
    while (n != 0U) {
        unsigned int payload;
        unsigned int total;
        unsigned int i;
        if (p == (const uint8_t *)0) return false;
        payload = Tbl34_RegisterPayloadBytes(p[0]);
        if (payload == 0U) return false;
        total = 1U + payload;
        if (total > n) return false;
        for (i = 0U; i < total; i++) Tbl34_PutByte(d,p[i]);
        p += total;
        n -= total;
    }
    return true;
}

/* Send exactly one register transaction from an already compiled event blob.
 * V34.6 uses this to separate the mode transition (CSR+CFR) from the
 * following CFTW0 takeover instead of latching both in one I/O_UPDATE. */
static bool Tbl34_ReadRegisterValueFromBlob(const uint8_t *p,
                                               unsigned int n,
                                               uint8_t wanted_addr,
                                               uint32_t *value_out)
{
    while (n != 0U) {
        unsigned int payload;
        unsigned int total;
        unsigned int i;
        uint32_t v = 0U;
        if (p == (const uint8_t *)0 || value_out == (uint32_t *)0) return false;
        payload = Tbl34_RegisterPayloadBytes(p[0]);
        if (payload == 0U || payload > 4U) return false;
        total = 1U + payload;
        if (total > n) return false;
        if ((p[0] & 0x7FU) == (wanted_addr & 0x7FU)) {
            for (i = 0U; i < payload; i++) v = (v << 8) | (uint32_t)p[1U + i];
            *value_out = v;
            return true;
        }
        p += total;
        n -= total;
    }
    return false;
}

/* V34.8 diagnostic path: exactly one AD9959 register frame at a time.
 * The bytes belonging to one register are queued, then the code waits until
 * SSI_SR_BSY is physically clear before any byte of the next register is
 * written.  This intentionally mirrors the historical ssiXPutData() rule. */
static bool Tbl34_PreloadEvent(unsigned int d, const tbl34_event_t *ev, const uint8_t *arena)
{
    uint32_t t0;
    uint32_t dt;
    bool ok;
    if (d >= TBL34_DDS_COUNT || ev == (const tbl34_event_t *)0) return false;
    if (ev->blob_len == 0U) return true;
    t0 = TBL34_DWT_CYCCNT;
    ok = Tbl34_PushBlob(d,&arena[ev->blob_offset],ev->blob_len);
    dt = TBL34_DWT_CYCCNT - t0;
    g_tbl34_dbg_preload_ticks[d] = dt;
    if (dt > g_tbl34_dbg_preload_max_ticks[d]) g_tbl34_dbg_preload_max_ticks[d] = dt;
    if (!ok) g_tbl34_dbg_preload_error[d]++;
    return ok;
}

static void Tbl34_DisableTimer(unsigned int d)
{
    unsigned long base = Tbl34_TimerBase(d);
    TimerIntDisable(base,TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
    HWREG(base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
}

static void Tbl34_ConfigOneShot(unsigned int d)
{
    unsigned long base = Tbl34_TimerBase(d);
    Tbl34_DisableTimer(d);
    TimerConfigure(base,TIMER_CFG_ONE_SHOT);
    TimerIntEnable(base,TIMER_TIMA_TIMEOUT);
}

static void Tbl34_ArmDeadlineWithLead(unsigned int d, uint32_t deadline, uint32_t wake_lead_ticks)
{
    unsigned long base = Tbl34_TimerBase(d);
    uint32_t now = TBL34_DWT_CYCCNT;
    uint32_t remain = deadline - now;
    uint32_t reload;
    if ((int32_t)remain <= 0) reload = TBL34_TIMER_MIN_TICKS;
    else if (remain > wake_lead_ticks + TBL34_TIMER_MIN_TICKS)
        reload = remain - wake_lead_ticks;
    else reload = TBL34_TIMER_MIN_TICKS;
    TimerIntDisable(base,TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
    HWREG(base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
    IntPendClear(Tbl34_TimerInterrupt(d));
    TimerLoadSet(base,TIMER_A,reload - 1U);
    TimerIntEnable(base,TIMER_TIMA_TIMEOUT);
    HWREG(base + TIMER_O_CTL) |= TIMER_CTL_TAEN;
}

static void Tbl34_ArmDeadline(unsigned int d, uint32_t deadline)
{
    Tbl34_ArmDeadlineWithLead(d,deadline,TBL34_WAKE_LEAD_TICKS);
}

/*
 * Native sweep exits are deliberately NOT serialized immediately after the
 * sweep start.  The current bench result points to CFR/CFTW0 being serialized too early
 * during the active ramp, so the exit preload is deferred near the endpoint.  The first timer wake
 * below is a PREPARE phase near the endpoint; a second wake performs the
 * exact-deadline I/O_UPDATE.
 */
static void Tbl34_ArmEventDeadline(unsigned int d,
                                   const tbl34_event_t *ev,
                                   uint32_t deadline)
{
    (void)ev;
    Tbl34_ArmDeadline(d,deadline);
}

static uint8_t Tbl34_PriorityRank(uint8_t ttl)
{
    unsigned int i;
    for (i = 0U; i < 3U; i++) if (g_tbl34_priority[i] == ttl) return (uint8_t)i;
    return 3U;
}

static void Tbl34_MaskMainTrigger(uint8_t ttl)
{
    uint8_t pin;
    if (ttl >= 3U || g_tbl34_policy != 0U) return;
    if (g_tbl34_gate_def.valid && g_tbl34_gate_def.gate_ttl_index == ttl) return;
    pin = Tbl34_TtlPinMask(ttl);
    HWREG(GPIO_PORTL_BASE + GPIO_O_IM) &= ~(uint32_t)pin;
    HWREG(GPIO_PORTL_BASE + GPIO_O_ICR) = pin;
    g_tbl34_masked_main_ttl_bits |= (uint8_t)(1U << ttl);
    g_tbl34_dbg_main_trigger_masked[ttl]++;
}

static void Tbl34_UnmaskMainTriggers(void)
{
    unsigned int ttl;
    for (ttl = 0U; ttl < 3U; ttl++) {
        if ((g_tbl34_masked_main_ttl_bits & (1U << ttl)) != 0U) {
            uint8_t pin = Tbl34_TtlPinMask(ttl);
            HWREG(GPIO_PORTL_BASE + GPIO_O_ICR) = pin;
            HWREG(GPIO_PORTL_BASE + GPIO_O_IM) |= pin;
        }
    }
    g_tbl34_masked_main_ttl_bits = 0U;
}

static bool Tbl34_ValidateEvents(const tbl34_event_t *ev, uint16_t n, uint16_t blob_len)
{
    uint16_t i;
    if (n == 0U || ev[0].at_ticks != 0U) return false;
    for (i = 0U; i < n; i++) {
        if ((unsigned int)ev[i].blob_offset + (unsigned int)ev[i].blob_len > blob_len) return false;
        if (ev[i].blob_len > TBL34_MAX_BLOB_PER_EVENT) return false;
        if (i != 0U && ev[i].at_ticks <= ev[i-1U].at_ticks) return false;
    }
    return (ev[n-1U].action & TBL34_ACTION_STREAM_END) != 0U;
}

static bool Tbl351_GateUsesFastTiming(void)
{
    return g_tbl34_gate_def.valid
        && (g_tbl34_gate_def.flags
            & (TBL34_GATE_FLAG_FAST_STAIRCASE | TBL35_GATE_FLAG_FAST_EVENT_STREAM)) != 0U;
}

static bool Tbl352_PrepareUltrafastLayout(void);

static bool Tbl34_ValidateFastGate(void)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    tbl34_event_t *events = Tbl365_ActiveFastEvents();
    uint16_t i;
    uint16_t fast_count = 0U;
    uint32_t period;
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_NONE || !def->valid || def->n_events < 2U) return false;
    while (fast_count < def->n_events
           && (events[fast_count].action & TBL34_ACTION_FAST_CHAIN) != 0U)
        fast_count++;
    if (fast_count < 2U) return false;
    period = events[1].at_ticks - events[0].at_ticks;
    if (period < TBL34_FAST_MIN_TICKS) return false;
    for (i = 2U; i < fast_count; i++) {
        if ((events[i].at_ticks - events[i-1U].at_ticks) != period) return false;
    }
    return true;
}

static bool Tbl354_PrepareFastGateAtArm(void)
{
    bool ok;
    if (g_tbl365_fast_def.valid) {
        g_tbl365_fast_source = TBL365_FAST_SOURCE_AUTO;
    } else if (Tbl351_GateUsesFastTiming()) {
        g_tbl365_fast_source = TBL365_FAST_SOURCE_GATE;
    } else {
        g_tbl365_fast_source = TBL365_FAST_SOURCE_NONE;
        return true;
    }
    g_tbl354_fast_gate_validated = false;
    g_tbl354_layout_cached = false;
    ok = Tbl34_ValidateFastGate();
    if (ok) {
        g_tbl354_fast_gate_validated = true;
        ok = Tbl352_PrepareUltrafastLayout();
    }
    return ok;
}

static void Tbl34_ParseWireEvent(tbl34_event_t *dst, const uint8_t *p)
{
    dst->at_ticks = Tbl34_Rd32(&p[0]);
    dst->blob_offset = Tbl34_Rd16(&p[4]);
    dst->blob_len = p[6];
    dst->action = p[7];
    dst->profile_ops = p[8];
    dst->reserved = p[9];
    dst->aux = Tbl34_Rd16(&p[10]);
}

static void Tbl34_StopProfilePwm(void)
{
    unsigned long base;
    if (!g_tbl34_profile_pwm_active) return;
    base = g_tbl34_profile_pwm_timer_base;
    TimerDisable(base,g_tbl34_profile_pwm_timer_half);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE,g_tbl34_profile_pwm_pin);
    GPIOPinWrite(GPIO_PORTM_BASE,g_tbl34_profile_pwm_pin,0U);
    g_tbl34_profile_pwm_active = false;
    g_tbl34_profile_pwm_timer_base = 0UL;
    g_tbl34_profile_pwm_timer_half = TIMER_A;
    g_tbl34_profile_pwm_pin = 0U;
    g_tbl34_profile_restore_pending_base = base;
    g_tbl34_dbg_profile_pwm_stop_count++;
}

static inline void Tbl355_ConfigureDirectPeriodicBoundary(unsigned long base, uint32_t period_ticks)
{

    HWREG(base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(base + TIMER_O_IMR) &= ~(TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
    HWREG(base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;

    /* Full-width 32-bit periodic Timer A.  TivaWare TIMER_CFG_PERIODIC is
     * encoded as CFG=0 and TAMR low byte=0x22. */
    HWREG(base + TIMER_O_CFG) = (TIMER_CFG_PERIODIC >> 24);
    HWREG(base + TIMER_O_TAMR) = (TIMER_CFG_PERIODIC & 0xFFUL);
    HWREG(base + TIMER_O_TAILR) = period_ticks - 1U;
    HWREG(base + TIMER_O_IMR) |= TIMER_TIMA_TIMEOUT;
}

static bool Tbl34_PrepareFastTimingHardware(unsigned int d,
                                             unsigned int c,
                                             uint32_t period_ticks,
                                             uint32_t half_ticks,
                                             bool triangle)
{
    unsigned long boundary = Tbl34_TimerBase(d);
    unsigned long pwm_base = 0UL;
    unsigned int pwm_half = TIMER_A;
    unsigned long pwm_config = 0UL;
    unsigned long pwm_sync = 0UL;
    unsigned long boundary_sync = 0UL;
    unsigned long pin_config = 0UL;
    unsigned char pin = 0U;
    unsigned long mode_offset = 0UL;

    if (period_ticks < TBL34_FAST_MIN_TICKS
        || (triangle && period_ticks > 0xFFFFUL)) {
        /* Timer4/5/6 are 32-bit on the fast boundary path.  Only the DDS3
         * PROFILE PWM uses a split 16-bit Timer2/3 half and therefore needs
         * the 0xFFFF limit. */
        if (triangle) g_tbl35_dbg_triangle_config_error_count++;
        return false;
    }
    if (triangle) {
        g_tbl35_dbg_triangle_config_count++;
        g_tbl35_dbg_triangle_period_ticks = period_ticks;
        g_tbl35_dbg_triangle_half_ticks = half_ticks;
    }
    if (g_tbl352_ultrafast_prepared) {
        /* V35.5: hot start.  The main one-shot timer has just delivered the
         * GATE_ARM boundary; morph it directly into the 32-bit periodic
         * timeout clock instead of calling several DriverLib functions. */
        Tbl355_ConfigureDirectPeriodicBoundary(boundary,period_ticks);
    } else {
        Tbl34_DisableTimer(d);
        TimerConfigure(boundary,TIMER_CFG_PERIODIC);
        TimerLoadSet(boundary,TIMER_A,period_ticks - 1U);
        TimerMatchSet(boundary,TIMER_A,TBL34_FAST_WAKE_LEAD_TICKS - 1U);
        TimerIntEnable(boundary,TIMER_TIMA_MATCH);
    }

    if (boundary == TIMER4_BASE) boundary_sync = TIMER_4A_SYNC;
    else if (boundary == TIMER5_BASE) boundary_sync = TIMER_5A_SYNC;
    else if (boundary == TIMER6_BASE) boundary_sync = TIMER_6A_SYNC;
    else return false;

    if (triangle) {
        if (d != 2U || c >= 4U || half_ticks == 0U || half_ticks >= period_ticks) {
            g_tbl35_dbg_triangle_config_error_count++;
            return false;
        }
        switch (c) {
        case 0U:
            pwm_base = TIMER3_BASE; pwm_half = TIMER_B;
            pwm_config = TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_PWM;
            pwm_sync = TIMER_3B_SYNC; pin_config = GPIO_PM3_T3CCP1; pin = GPIO_PIN_3; mode_offset = TIMER_O_TBMR; break;
        case 1U:
            pwm_base = TIMER3_BASE; pwm_half = TIMER_A;
            pwm_config = TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_ONE_SHOT;
            pwm_sync = TIMER_3A_SYNC; pin_config = GPIO_PM2_T3CCP0; pin = GPIO_PIN_2; mode_offset = TIMER_O_TAMR; break;
        case 2U:
            pwm_base = TIMER2_BASE; pwm_half = TIMER_B;
            pwm_config = TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_PWM;
            pwm_sync = TIMER_2B_SYNC; pin_config = GPIO_PM1_T2CCP1; pin = GPIO_PIN_1; mode_offset = TIMER_O_TBMR; break;
        default:
            pwm_base = TIMER2_BASE; pwm_half = TIMER_A;
            pwm_config = TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_ONE_SHOT;
            pwm_sync = TIMER_2A_SYNC; pin_config = GPIO_PM0_T2CCP0; pin = GPIO_PIN_0; mode_offset = TIMER_O_TAMR; break;
        }
        Tbl34_StopProfilePwm();
        GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE,pin);
        GPIOPinWrite(GPIO_PORTM_BASE,pin,0U);
        TimerDisable(pwm_base,TIMER_BOTH);
        HWREG(pwm_base + TIMER_O_IMR) = 0UL;
        HWREG(pwm_base + TIMER_O_ICR) = 0xFFFFFFFFUL;
        TimerConfigure(pwm_base,pwm_config);
        TimerLoadSet(pwm_base,pwm_half,period_ticks - 1U);
        TimerMatchSet(pwm_base,pwm_half,period_ticks - half_ticks - 1U);
        g_tbl35_dbg_triangle_pwm_load = period_ticks - 1U;
        g_tbl35_dbg_triangle_pwm_match = period_ticks - half_ticks - 1U;
        TimerControlLevel(pwm_base,pwm_half,false);
        HWREG(pwm_base + mode_offset) |= 0x00000C00UL;
    }

    g_tbl34_fast_boundary_base = boundary;
    g_tbl34_fast_boundary_sync_mask = boundary_sync;
    g_tbl34_fast_timing_triangle = triangle;
    g_tbl34_fast_pwm_base = pwm_base;
    g_tbl34_fast_pwm_half = pwm_half;
    g_tbl34_fast_pwm_sync_mask = pwm_sync;
    g_tbl34_fast_pwm_pin_config = pin_config;
    g_tbl34_fast_pwm_pin = pin;
    g_tbl34_fast_timing_prepared = true;
    return true;
}

static bool Tbl34_StartPreparedFastTiming(uint32_t *origin_ticks)
{
    uint32_t origin;
    if (!g_tbl34_fast_timing_prepared || g_tbl34_fast_boundary_base == 0UL) return false;

    /* V35.4: GATE_ARM may have ended the main stream and masked this timer.
     * Re-enable the exact interrupt source at the real release/start. */
    TimerIntDisable(g_tbl34_fast_boundary_base,TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
    HWREG(g_tbl34_fast_boundary_base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
    if (g_tbl352_ultrafast_prepared)
        TimerIntEnable(g_tbl34_fast_boundary_base,TIMER_TIMA_TIMEOUT);
    else
        TimerIntEnable(g_tbl34_fast_boundary_base,TIMER_TIMA_MATCH);
    g_tbl354_dbg_gate_timer_rearm_count++;
    g_tbl354_dbg_gate_timer_imr_after_rearm =
        HWREG(g_tbl34_fast_boundary_base + TIMER_O_IMR);

    if (g_tbl34_fast_timing_triangle) {
        GPIOPinConfigure(g_tbl34_fast_pwm_pin_config);
        GPIOPinTypeTimer(GPIO_PORTM_BASE,g_tbl34_fast_pwm_pin);
        TimerEnable(g_tbl34_fast_pwm_base,g_tbl34_fast_pwm_half);
        HWREG(g_tbl34_fast_boundary_base + TIMER_O_CTL) |= TIMER_CTL_TAEN;
        {
            uint32_t sync_mask = g_tbl34_fast_pwm_sync_mask | g_tbl34_fast_boundary_sync_mask;
            HWREG(TIMER0_BASE + TIMER_O_SYNC) = 0UL;
            HWREG(TIMER0_BASE + TIMER_O_SYNC) = sync_mask;
            g_tbl355_dbg_triangle_sync_mask = sync_mask;
            g_tbl355_dbg_triangle_sync_count++;
        }
        origin = TBL34_DWT_CYCCNT;
        g_tbl34_profile_pwm_active = true;
        g_tbl34_profile_pwm_timer_base = g_tbl34_fast_pwm_base;
        g_tbl34_profile_pwm_timer_half = g_tbl34_fast_pwm_half;
        g_tbl34_profile_pwm_pin = g_tbl34_fast_pwm_pin;
        g_tbl34_dbg_profile_pwm_sync_ticks = origin;
        g_tbl34_dbg_profile_pwm_start_count++;
    } else {
        HWREG(g_tbl34_fast_boundary_base + TIMER_O_CTL) |= TIMER_CTL_TAEN;
        origin = TBL34_DWT_CYCCNT;
    }
    if (origin_ticks != (uint32_t *)0) *origin_ticks = origin;
    return true;
}

/* V34.22 low-latency recovery.
 *
 * V34.21 proved that MASTER_RESET + reconstruction from the Tiva table is a
 * valid way to escape the AD9959 linear-sweep terminal state, but ~34.5 us of
 * its ~54 us recovery was spent re-running the board-startup functions
 * init_SPIx_mode_single_bit()/quad_bit().  Those functions re-enable already
 * enabled peripherals, re-run GPIO mux/pad configuration and reprogram the
 * SSI clock.  None of that is needed inside a real-time table boundary.
 *
 * V34.22 keeps the pins, clock divider and SSI peripheral configuration that
 * were established at boot.  It only changes the SSI advanced transfer mode:
 *
 *   QUAD_WRITE -> standard WRITE -> send CSR=F6 -> UPDATE -> QUAD_WRITE
 *
 * The extra XDAT2/XDAT3 pins stay muxed to SSI during the short standard-SPI
 * phase; the peripheral simply does not use them.  This mirrors the effective
 * CR1 values of the historical initializers (single: 0x00 before SSE enable,
 * quad: 0x80 before SSE enable) without repeating their slow GPIO/clock setup.
 *
 * The restore blob is also sent only once and through a trust-the-compiler
 * raw FIFO loop.  Its first frame is CSR, whose channel-select bits are active
 * immediately, so the following CFR/FTW/CPOW/ACR frames target the requested
 * channel before the final I/O_UPDATE.
 */
#define TBL34_V3422_RESET_HIGH_TICKS    60UL  /* 0.5 us. */
#define TBL34_V3422_RESET_SETTLE_TICKS  60UL  /* 0.5 us. */
#define TBL34_V3422_MODE_SETTLE_TICKS   24UL  /* 0.2 us after serial-mode UPDATE. */
#define TBL34_V3422_UPDATE_HIGH_TICKS   24UL  /* 0.2 us > one 8 ns SYNC_CLK. */
#define TBL34_V3422_CR1_SSE             0x00000002UL
#define TBL34_V3422_CR1_QUAD_BASE       0x00000080UL

static inline void Tbl34_V3422Delay(uint32_t ticks)
{
    uint32_t t0 = TBL34_DWT_CYCCNT;
    while ((uint32_t)(TBL34_DWT_CYCCNT - t0) < ticks) { }
}

static void Tbl34_V3422SetMasterReset(unsigned int d, bool high)
{
    unsigned long base;
    unsigned char pin;
    if (d == 0U) {
        base = GPIO_PORTK_BASE;
        pin = GPIO_PIN_5;
    } else if (d == 1U) {
        base = GPIO_PORTN_BASE;
        pin = GPIO_PIN_4;
    } else {
        base = GPIO_PORTN_BASE;
        pin = GPIO_PIN_1;
    }
    HWREG(base + GPIO_O_DATA + ((unsigned long)pin << 2)) = high ? pin : 0U;
}

static bool Tbl34_V3422SwitchToSingleFast(unsigned int d)
{
    unsigned long base;
    if (d >= 3U) return false;
    base = Tbl34_SsiBase(d);
    Tbl34_WaitSsiIdle(d);

    /* Disable SSE only while changing advanced mode. */
    HWREG(base + SSI_O_CR1) &= ~TBL34_V3422_CR1_SSE;
    SSIAdvModeSet(base,SSI_ADV_MODE_WRITE);
    SSIAdvFrameHoldEnable(base);

    /* Same effective CR1 state as the validated historical single-bit init. */
    HWREG(base + SSI_O_CR1) = 0x00000000UL;
    HWREG(base + SSI_O_CR1) = TBL34_V3422_CR1_SSE;
    return true;
}

static bool Tbl34_V3422SwitchToQuadFast(unsigned int d)
{
    unsigned long base;
    if (d >= 3U) return false;
    base = Tbl34_SsiBase(d);
    Tbl34_WaitSsiIdle(d);

    HWREG(base + SSI_O_CR1) &= ~TBL34_V3422_CR1_SSE;
    SSIAdvFrameHoldDisable(base);
    SSIAdvModeSet(base,SSI_ADV_MODE_QUAD_WRITE);

    /* Same effective CR1 state as the validated historical quad init. */
    HWREG(base + SSI_O_CR1) = TBL34_V3422_CR1_QUAD_BASE;
    HWREG(base + SSI_O_CR1) = TBL34_V3422_CR1_QUAD_BASE | TBL34_V3422_CR1_SSE;
    return true;
}

static bool Tbl34_V3422WriteF6Single(unsigned int d)
{
    if (d >= 3U) return false;
    Tbl34_PutByte(d,TBL34_CSR_ADDR);
    Tbl34_PutByte(d,0xF6U);
    Tbl34_WaitSsiIdle(d);
    return true;
}

static void Tbl35_InvalidateResetPrepared(unsigned int d)
{
    if (d >= TBL34_DDS_COUNT) return;
    g_tbl35_reset_prepared_ready[d] = false;
    g_tbl35_reset_prepared_len[d] = 0U;
    g_tbl35_reset_prepared_source[d] = (const uint8_t *)0;
}

static bool Tbl35_PrepareResetImage(unsigned int d,
                                    const uint8_t *blob,
                                    unsigned int blob_len)
{
    const uint8_t *p = blob;
    unsigned int n = blob_len;
    uint8_t selected = 0U;
    uint8_t channel_mask = 0U;
    uint8_t found[4] = {0U,0U,0U,0U};
    unsigned int i;

    if (d >= TBL34_DDS_COUNT || blob == (const uint8_t *)0 || blob_len == 0U
        || blob_len > TBL35_RESET_IMAGE_MAX_BYTES) return false;
    Tbl35_InvalidateResetPrepared(d);

    while (n != 0U) {
        uint8_t addr = (uint8_t)(p[0] & 0x7FU);
        unsigned int payload = Tbl34_RegisterPayloadBytes(p[0]);
        unsigned int total;
        uint8_t reg_bit = 0U;
        if (payload == 0U) return false;
        total = 1U + payload;
        if (total > n) return false;
        if (addr == TBL34_CSR_ADDR) {
            if (payload != 1U) return false;
            selected = (uint8_t)((p[1] >> 4) & 0x0FU);
            if (selected == 0U) return false;
            channel_mask |= selected;
            for (i = 0U; i < 4U; i++) if ((selected & (1U << i)) != 0U) found[i] |= 0x01U;
        } else {
            if (selected == 0U) return false;
            if (addr == TBL34_CFR_ADDR) reg_bit = 0x02U;
            else if (addr == TBL34_FTW_ADDR) reg_bit = 0x04U;
            else if (addr == TBL34_CPOW_ADDR) reg_bit = 0x08U;
            else if (addr == TBL34_ACR_ADDR) reg_bit = 0x10U;
            else return false;
            for (i = 0U; i < 4U; i++) if ((selected & (1U << i)) != 0U) found[i] |= reg_bit;
        }
        p += total;
        n -= total;
    }

    for (i = 0U; i < 4U; i++) {
        if ((channel_mask & (1U << i)) != 0U && found[i] != 0x1FU) return false;
    }

    /* The source points into g_tbl34_blob[] or g_tbl34_gate_blob.  Those
     * arenas cannot be reloaded while the corresponding table is active, so
     * the bytes remain stable until RESET_COMMIT consumes this image. */
    g_tbl35_reset_prepared_source[d] = blob;
    g_tbl35_reset_prepared_len[d] = (uint8_t)blob_len;
    g_tbl35_reset_prepared_ready[d] = true;
    g_tbl35_dbg_reset_prepare_bytes[d] = blob_len;
    g_tbl35_dbg_reset_restore_channel_mask[d] = channel_mask;
    return true;
}

static bool Tbl35_PushPreparedResetImage(unsigned int d,
                                         const uint8_t *image,
                                         unsigned int image_len)
{
    unsigned long base;
    unsigned int i;
    if (d >= TBL34_DDS_COUNT || image == (const uint8_t *)0 || image_len == 0U
        || image_len > TBL35_RESET_IMAGE_MAX_BYTES) return false;
    base = Tbl34_SsiBase(d);
    for (i = 0U; i < image_len; i++) {
        while ((HWREG(base + SSI_O_SR) & SSI_SR_TNF) == 0U) { }
        HWREG(base + SSI_O_DR) = (uint32_t)image[i];
    }
    Tbl34_WaitSsiIdle(d);
    return true;
}

static bool Tbl35_ResetRecoverPrepared(unsigned int d)
{
    uint32_t t0, tstage;
    const uint8_t *restore_blob;
    unsigned int restore_len;

    if (d >= TBL34_DDS_COUNT || !g_tbl35_reset_prepared_ready[d]) return false;
    restore_blob = g_tbl35_reset_prepared_source[d];
    restore_len = g_tbl35_reset_prepared_len[d];
    if (restore_blob == (const uint8_t *)0 || restore_len == 0U) return false;

    Tbl34_WaitSsiIdle(d);
    t0 = TBL34_DWT_CYCCNT;

    Tbl34_V3422SetMasterReset(d,true);
    tstage = TBL34_DWT_CYCCNT;
    Tbl34_V3422Delay(TBL34_V3422_RESET_HIGH_TICKS);
    g_tbl34_dbg_reset_recovery_master_high_ticks[d] = TBL34_DWT_CYCCNT - tstage;
    Tbl34_V3422SetMasterReset(d,false);

    tstage = TBL34_DWT_CYCCNT;
    Tbl34_V3422Delay(TBL34_V3422_RESET_SETTLE_TICKS);
    tstage = TBL34_DWT_CYCCNT;
    if (!Tbl34_V3422SwitchToSingleFast(d)) goto error_after_reset;
    g_tbl34_dbg_reset_recovery_single_init_count[d]++;

    tstage = TBL34_DWT_CYCCNT;
    if (!Tbl34_V3422WriteF6Single(d)) goto error_after_reset;

    tstage = TBL34_DWT_CYCCNT;
    Tbl34_IOUpdateOneWide(d,TBL34_V3422_UPDATE_HIGH_TICKS);

    tstage = TBL34_DWT_CYCCNT;
    Tbl34_V3422Delay(TBL34_V3422_MODE_SETTLE_TICKS);

    tstage = TBL34_DWT_CYCCNT;
    if (!Tbl34_V3422SwitchToQuadFast(d)) goto error_after_reset;
    g_tbl34_dbg_reset_recovery_quad_init_count[d]++;
    if (!Tbl35_PushPreparedResetImage(d,restore_blob,restore_len)) goto error_after_reset;
    g_tbl34_dbg_reset_recovery_program_count[d]++;

    tstage = TBL34_DWT_CYCCNT;
    Tbl34_IOUpdateOneWide(d,TBL34_V3422_UPDATE_HIGH_TICKS);
    g_tbl34_dbg_reset_recovery_update_count[d]++;
    g_tbl34_dbg_reset_recovery_count[d]++;

    g_tbl35_dbg_reset_blackout_ticks[d] = TBL34_DWT_CYCCNT - t0;
    g_tbl35_dbg_reset_restore_bytes[d] = restore_len;
    g_tbl34_dbg_reset_recovery_total_ticks[d] = g_tbl35_dbg_reset_blackout_ticks[d];
    g_tbl35_dbg_reset_commit_count[d]++;
    Tbl35_InvalidateResetPrepared(d);
    return true;

error_after_reset:
    g_tbl34_dbg_reset_recovery_error[d]++;
    g_tbl35_dbg_reset_commit_error[d]++;
    g_tbl35_dbg_reset_blackout_ticks[d] = TBL34_DWT_CYCCNT - t0;
    g_tbl34_dbg_reset_recovery_total_ticks[d] = g_tbl35_dbg_reset_blackout_ticks[d];
    Tbl35_InvalidateResetPrepared(d);
    (void)Tbl34_V3422SwitchToQuadFast(d);
    return false;
}

static uint32_t Tbl356_FindCfrInEvent(const tbl34_event_t *ev, const uint8_t *arena)
{
    const uint8_t *p;
    unsigned int n;
    if (ev == (const tbl34_event_t *)0 || arena == (const uint8_t *)0 || ev->blob_len == 0U) return 0U;
    p = &arena[ev->blob_offset];
    n = ev->blob_len;
    while (n != 0U) {
        unsigned int payload = Tbl34_RegisterPayloadBytes(p[0]);
        unsigned int total = 1U + payload;
        if (payload == 0U || total > n) return 0U;
        if ((p[0] & 0x7FU) == TBL34_CFR_ADDR && payload == 3U)
            return ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
        p += total;
        n -= total;
    }
    return 0U;
}

static void Tbl35_CountEventRegisterFrames(unsigned int d,
                                           const tbl34_event_t *ev,
                                           const uint8_t *arena)
{
    const uint8_t *p;
    unsigned int n;
    if (d >= TBL34_DDS_COUNT || ev == (const tbl34_event_t *)0
        || arena == (const uint8_t *)0 || ev->blob_len == 0U) return;
    p = &arena[ev->blob_offset];
    n = ev->blob_len;
    while (n != 0U) {
        uint8_t addr = (uint8_t)(p[0] & 0x7FU);
        unsigned int payload = Tbl34_RegisterPayloadBytes(p[0]);
        unsigned int total;
        if (payload == 0U) return;
        total = 1U + payload;
        if (total > n) return;
        if (addr == TBL34_CFR_ADDR) g_tbl35_dbg_event_cfr_frame_count[d]++;
        else if (addr == TBL34_FTW_ADDR) g_tbl35_dbg_event_ftw_frame_count[d]++;
        else if (addr == TBL34_CPOW_ADDR) g_tbl35_dbg_event_cpow_frame_count[d]++;
        else if (addr == TBL34_ACR_ADDR) g_tbl35_dbg_event_acr_frame_count[d]++;
        p += total;
        n -= total;
    }
}

static void Tbl351_CountFastRegisterFrames(const tbl34_event_t *ev)
{
    const uint8_t *p;
    unsigned int n;
    if (ev == (const tbl34_event_t *)0 || ev->blob_len == 0U) return;
    p = &Tbl365_ActiveFastBlob()[ev->blob_offset];
    n = ev->blob_len;
    while (n != 0U) {
        uint8_t addr = (uint8_t)(p[0] & 0x7FU);
        unsigned int payload = Tbl34_RegisterPayloadBytes(p[0]);
        unsigned int total;
        if (payload == 0U) return;
        total = 1U + payload;
        if (total > n) return;
        if (addr == TBL34_FTW_ADDR) g_tbl351_dbg_fast_ftw_frame_count++;
        else if (addr == TBL34_CPOW_ADDR) g_tbl351_dbg_fast_cpow_frame_count++;
        else if (addr == TBL34_ACR_ADDR) g_tbl351_dbg_fast_acr_frame_count++;
        p += total;
        n -= total;
    }
}

static void Tbl352_ClearUltrafastLayout(void)
{
    g_tbl352_ultrafast_prepared = false;
    g_tbl353_direct_active = false;
    g_tbl352_ultrafast_fast_count = 0U;
    g_tbl352_ultrafast_frame_len = 0U;
    g_tbl352_ultrafast_frame_addr = 0U;
    g_tbl352_ultrafast_next_frame = (const uint8_t *)0;
    g_tbl352_ultrafast_ssi_base = 0UL;
    g_tbl352_ultrafast_ioupdate_addr = 0UL;
    g_tbl352_ultrafast_ioupdate_mask = 0U;
    g_tbl353_direct_remaining = 0U;
    g_tbl353_direct_total_irqs = 0U;
    g_tbl353_direct_frame = (const uint8_t *)0;
    g_tbl353_direct_start_tick = 0U;
}

static void Tbl354_InvalidateFastCache(void)
{
    g_tbl354_fast_gate_validated = false;
    g_tbl354_layout_cached = false;
    g_tbl354_gate_arm_commit_tick = 0U;
    g_tbl356_auto_event0_committed = false;
    g_tbl356_triangle_exit_pending = false;
    g_tbl356_direct_fast_end_tick = 0U;
    Tbl352_ClearUltrafastLayout();
}

static bool Tbl352_PrepareUltrafastLayout(void)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    tbl34_event_t *events = Tbl365_ActiveFastEvents();
    uint8_t *blob = Tbl365_ActiveFastBlob();
    uint16_t fast_count = 0U;
    uint16_t i;
    uint8_t addr;
    uint8_t frame_len;
    unsigned int payload;
    uint16_t expected_offset;
    unsigned int d;

    g_tbl353_dbg_layout_prepare_count++;
    Tbl352_ClearUltrafastLayout();
    g_tbl353_dbg_direct_layout_error_code = 0U;

    if (!def->valid || def->n_events < 2U) {
        g_tbl353_dbg_direct_layout_error_code = 1U; goto fail;
    }
    while (fast_count < def->n_events
           && (events[fast_count].action & TBL34_ACTION_FAST_CHAIN) != 0U)
        fast_count++;
    if (fast_count < 2U) {
        g_tbl353_dbg_direct_layout_error_code = 2U; goto fail;
    }

    /* Event 0 may contain setup registers.  The hot stream begins at event 1. */
    if (events[1].blob_len == 0U) {
        g_tbl353_dbg_direct_layout_error_code = 3U; goto fail;
    }
    addr = (uint8_t)(blob[events[1].blob_offset] & 0x7FU);
    payload = Tbl34_RegisterPayloadBytes(addr);
    frame_len = (uint8_t)(1U + payload);
    if (payload == 0U || events[1].blob_len != frame_len
        || (addr != TBL34_FTW_ADDR && addr != TBL34_CPOW_ADDR && addr != TBL34_ACR_ADDR)) {
        g_tbl353_dbg_direct_layout_error_code = 4U; goto fail;
    }

    expected_offset = events[1].blob_offset;
    for (i = 1U; i < fast_count; i++) {
        const tbl34_event_t *ev = &events[i];
        const uint8_t *p;
        if ((ev->action & (TBL34_ACTION_FAST_CHAIN | TBL34_ACTION_IO_UPDATE))
            != (TBL34_ACTION_FAST_CHAIN | TBL34_ACTION_IO_UPDATE)) {
            g_tbl353_dbg_direct_layout_error_code = 5U; goto fail;
        }
        if (ev->blob_len != frame_len || ev->blob_offset != expected_offset) {
            g_tbl353_dbg_direct_layout_error_code = 6U; goto fail;
        }
        p = &blob[ev->blob_offset];
        if ((p[0] & 0x7FU) != addr) {
            g_tbl353_dbg_direct_layout_error_code = 7U; goto fail;
        }
        expected_offset = (uint16_t)(expected_offset + frame_len);
    }

    d = def->dds;
    if (d >= TBL34_DDS_COUNT) {
        g_tbl353_dbg_direct_layout_error_code = 8U; goto fail;
    }
    g_tbl352_ultrafast_fast_count = fast_count;
    g_tbl352_ultrafast_frame_len = frame_len;
    g_tbl352_ultrafast_frame_addr = addr;
    g_tbl352_ultrafast_next_frame = &blob[events[1].blob_offset];
    g_tbl352_ultrafast_ssi_base = Tbl34_SsiBase(d);
    if (d == 0U) {
        g_tbl352_ultrafast_ioupdate_addr = GPIO_PORTK_BASE + GPIO_O_DATA + (0x80UL << 2);
        g_tbl352_ultrafast_ioupdate_mask = 0x80U;
    } else if (d == 1U) {
        g_tbl352_ultrafast_ioupdate_addr = GPIO_PORTN_BASE + GPIO_O_DATA + (0x08UL << 2);
        g_tbl352_ultrafast_ioupdate_mask = 0x08U;
    } else {
        g_tbl352_ultrafast_ioupdate_addr = GPIO_PORTN_BASE + GPIO_O_DATA + (0x01UL << 2);
        g_tbl352_ultrafast_ioupdate_mask = 0x01U;
    }
    g_tbl352_ultrafast_prepared = true;
    g_tbl353_dbg_direct_stream_addr = addr;
    g_tbl353_dbg_direct_stride = frame_len;
    g_tbl353_dbg_direct_chain_events = fast_count;
    g_tbl354_layout_cached = true;
    return true;

fail:
    g_tbl354_layout_cached = false;
    g_tbl353_dbg_layout_prepare_error_count++;
    return false;
}

static inline void Tbl353_QueueDirectFrame(const uint8_t *p)
{
    unsigned long dr = g_tbl352_ultrafast_ssi_base + SSI_O_DR;
    if (g_tbl352_ultrafast_frame_len == 5U) {
        HWREG(dr)=p[0]; HWREG(dr)=p[1]; HWREG(dr)=p[2]; HWREG(dr)=p[3]; HWREG(dr)=p[4];
    } else if (g_tbl352_ultrafast_frame_len == 4U) {
        HWREG(dr)=p[0]; HWREG(dr)=p[1]; HWREG(dr)=p[2]; HWREG(dr)=p[3];
    } else {
        HWREG(dr)=p[0]; HWREG(dr)=p[1]; HWREG(dr)=p[2];
    }
}

static inline void Tbl352_PulseIOUpdateFast(void)
{
    unsigned long a = g_tbl352_ultrafast_ioupdate_addr;
    uint32_t m = g_tbl352_ultrafast_ioupdate_mask;
    HWREG(a) = m;
    __asm(" NOP"); __asm(" NOP"); __asm(" NOP");
    HWREG(a) = 0U;
}

static void Tbl351_RecordNativePhaseProgram(unsigned int d,
                                             const tbl34_event_t *ev,
                                             const uint8_t *arena)
{
    uint32_t v;
    const uint8_t *p;
    if (d >= TBL34_DDS_COUNT || ev == (const tbl34_event_t *)0
        || arena == (const uint8_t *)0 || ev->blob_len == 0U) return;
    p = &arena[ev->blob_offset];
    if (Tbl34_ReadRegisterValueFromBlob(p,ev->blob_len,TBL34_CSR_ADDR,&v)) g_tbl351_dbg_phase_last_csr[d]=v;
    if (Tbl34_ReadRegisterValueFromBlob(p,ev->blob_len,TBL34_CFR_ADDR,&v)) g_tbl351_dbg_phase_last_cfr[d]=v;
    if (Tbl34_ReadRegisterValueFromBlob(p,ev->blob_len,TBL34_CPOW_ADDR,&v)) g_tbl351_dbg_phase_last_cpow[d]=v;
    if (Tbl34_ReadRegisterValueFromBlob(p,ev->blob_len,0x0AU,&v)) g_tbl351_dbg_phase_last_cw1[d]=v;
    if (Tbl34_ReadRegisterValueFromBlob(p,ev->blob_len,0x08U,&v)) {
        g_tbl351_dbg_phase_last_rdw[d]=v;
        g_tbl358_dbg_triangle_rdw=v;
    }
    if (Tbl34_ReadRegisterValueFromBlob(p,ev->blob_len,0x09U,&v)) {
        g_tbl351_dbg_phase_last_fdw[d]=v;
        g_tbl358_dbg_triangle_fdw=v;
    }
    if (Tbl34_ReadRegisterValueFromBlob(p,ev->blob_len,0x07U,&v)) {
        g_tbl351_dbg_phase_last_lsrr[d]=v;
        g_tbl358_dbg_triangle_lsrr=v;
    }
    if (Tbl34_ReadRegisterValueFromBlob(p,ev->blob_len,TBL34_FTW_ADDR,&v)) g_tbl351_dbg_phase_last_ftw[d]=v;
    if (Tbl34_ReadRegisterValueFromBlob(p,ev->blob_len,TBL34_ACR_ADDR,&v)) g_tbl351_dbg_phase_last_acr[d]=v;
}

static void Tbl34_RecordCommit(unsigned int d, uint32_t deadline)
{
    uint32_t now = TBL34_DWT_CYCCNT;
    uint32_t late = (int32_t)(now - deadline) > 0 ? (now - deadline) : 0U;
    g_tbl34_dbg_commit_count[d]++;
    g_tbl34_dbg_late_ticks[d] = late;
    if (late > g_tbl34_dbg_late_max_ticks[d]) g_tbl34_dbg_late_max_ticks[d] = late;
}

static bool Tbl34_CommitGeneric(unsigned int d,
                                const tbl34_event_t *ev,
                                const uint8_t *arena,
                                uint32_t deadline)
{
    uint32_t update_ticks = 0U;
    while ((int32_t)(TBL34_DWT_CYCCNT - deadline) < 0) { }

    if (g_tbl34_gate_running
        && (g_tbl34_gate_def.flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U
        && g_tbl34_profile_pwm_active
        && (ev->action & TBL34_ACTION_FAST_CHAIN) == 0U
        && (ev->profile_ops & 0xF0U) != 0U) {
        uint32_t now = TBL34_DWT_CYCCNT;
        g_tbl354_dbg_triangle_exit_count++;
        g_tbl354_dbg_triangle_exit_late_ticks =
            (int32_t)(now - deadline) > 0 ? (now - deadline) : 0U;
        Tbl34_StopProfilePwm();
    }

    Tbl35_CountEventRegisterFrames(d,ev,arena);
    if (((ev->aux & TBL35_AUX_NATIVE_PHASE) != 0U
         && (ev->action & TBL34_ACTION_SWEEP_START) != 0U)
        || ((ev->action & TBL34_ACTION_GATE_ARM) != 0U
            && ((g_tbl34_gate_def.valid
                 && g_tbl34_gate_def.dds == d
                 && (g_tbl34_gate_def.flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U)
                || (g_tbl365_fast_def.valid
                    && g_tbl365_fast_def.dds == d
                    && (g_tbl365_fast_def.flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U))))
        Tbl351_RecordNativePhaseProgram(d,ev,arena);

    if ((ev->action & TBL34_ACTION_SWEEP_EXIT_PREPARE) != 0U) {
        if ((ev->action & TBL34_ACTION_IO_UPDATE) != 0U
            || arena == (const uint8_t *)0 || ev->blob_len == 0U) {
            if ((ev->aux & TBL35_AUX_RESET_PREPARE) != 0U) g_tbl35_dbg_reset_prepare_error[d]++;
            return false;
        }

        if ((ev->aux & TBL35_AUX_RESET_PREPARE) != 0U) {
            /* Frequency native sweep: PREPARE is RAM-only.  The blob already
             * contains the complete post-reset state of every channel that
             * must survive the chip-level MASTER_RESET. */
            if (!Tbl35_PrepareResetImage(d,&arena[ev->blob_offset],ev->blob_len)) {
                g_tbl35_dbg_reset_prepare_error[d]++;
                return false;
            }
            g_tbl35_dbg_reset_prepare_count[d]++;
        } else {
            /* Phase/amplitude native sweep: no chip reset unless testing later
             * proves it necessary.  Serialize buffered exit state only. */
            if (!Tbl34_PushBlob(d,&arena[ev->blob_offset],ev->blob_len)) {
                return false;
            }
            Tbl34_WaitSsiIdle(d);
        }
    } else if ((ev->action & TBL34_ACTION_SWEEP_EXIT_COMMIT) != 0U) {
        if (ev->blob_len != 0U || (ev->action & TBL34_ACTION_IO_UPDATE) == 0U) {
            if ((ev->aux & TBL35_AUX_RESET_COMMIT) != 0U) g_tbl35_dbg_reset_commit_error[d]++;
            return false;
        }

        if ((ev->aux & TBL35_AUX_RESET_COMMIT) != 0U) {
            Tbl34_ApplyProfileLowMask(d,ev->profile_ops);
            if (!Tbl35_ResetRecoverPrepared(d)) {
                return false;
            }
        } else {
            Tbl34_WaitSsiIdle(d);
            Tbl34_IOUpdateOne(d);
            update_ticks = TBL34_DWT_CYCCNT;
            while ((uint32_t)(TBL34_DWT_CYCCNT - update_ticks) < TBL34_SWEEP_SETTLE_TICKS) { }
            Tbl34_ApplyProfileLowMask(d,ev->profile_ops);
        }
        g_tbl34_dbg_sweep_exit_count[d]++;
        if ((ev->aux & TBL35_AUX_NATIVE_FREQ) != 0U) g_tbl35_dbg_native_freq_exit_count[d]++;
        if ((ev->aux & TBL35_AUX_NATIVE_PHASE) != 0U) g_tbl35_dbg_native_phase_exit_count[d]++;
        if ((ev->aux & TBL35_AUX_NATIVE_AMP) != 0U) g_tbl35_dbg_native_amp_exit_count[d]++;
    } else {
        Tbl34_WaitSsiIdle(d);
        Tbl34_ApplyProfileLowMask(d,ev->profile_ops);
        if ((ev->action & TBL34_ACTION_IO_UPDATE) != 0U) {
            Tbl34_IOUpdateOne(d);
            update_ticks = TBL34_DWT_CYCCNT;
            if ((ev->action & TBL34_ACTION_GATE_ARM) != 0U)
                g_tbl354_gate_arm_commit_tick = update_ticks;
        }
        if ((ev->action & TBL34_ACTION_SWEEP_START) != 0U) {
            uint32_t profile_high_tick = 0U;
            if ((ev->profile_ops & 0x0FU) != 0U && update_ticks != 0U) {
                while ((uint32_t)(TBL34_DWT_CYCCNT - update_ticks) < TBL34_SWEEP_SETTLE_TICKS) { }
            }
            g_tbl34_dbg_sweep_start_count[d]++;
            if ((ev->aux & TBL35_AUX_NATIVE_FREQ) != 0U) g_tbl35_dbg_native_freq_start_count[d]++;
            if ((ev->aux & TBL35_AUX_NATIVE_PHASE) != 0U) g_tbl35_dbg_native_phase_start_count[d]++;
            if ((ev->aux & TBL35_AUX_NATIVE_AMP) != 0U) g_tbl35_dbg_native_amp_start_count[d]++;
            if ((ev->aux & TBL351_AUX_NATIVE_DESCENDING) != 0U) g_tbl351_dbg_native_desc_start_count[d]++;

            Tbl34_ApplyProfileHighMask(d,ev->profile_ops);

            /* V36 descending-sweep fix.  V35.1 emitted a second table event
             * only 48 ticks (0.4 us) after SWEEP_START.  The generic ISR takes
             * orders of magnitude longer, so PROFILE LOW arrived late and the
             * falling ramp could be badly shifted or effectively unusable.
             * Generate the short HIGH pulse in this ISR instead. */
            if ((ev->aux & TBL351_AUX_NATIVE_DESCENDING) != 0U
                && (ev->profile_ops & 0x0FU) != 0U
                && (ev->profile_ops & 0xF0U) != 0U) {
                profile_high_tick = TBL34_DWT_CYCCNT;
                while ((uint32_t)(TBL34_DWT_CYCCNT - profile_high_tick)
                       < TBL36_DESC_PROFILE_HIGH_HOLD_TICKS) { }
                Tbl34_ApplyProfileLowMask(d,ev->profile_ops);
                g_tbl351_dbg_native_desc_profile_low_count[d]++;
                g_tbl36_dbg_native_desc_high_hold_ticks =
                    TBL34_DWT_CYCCNT - profile_high_tick;
            }
        } else {
            /* Non-sweep events keep the historical PROFILE-high handling. */
            Tbl34_ApplyProfileHighMask(d,ev->profile_ops);
        }
    }
    Tbl34_RecordCommit(d,deadline);
    return true;
}

static void Tbl34_ResetRuntimeOnly(void)
{
    unsigned int d;
    g_tbl362_main_preload_pending_mask = 0U;
    /* Clear any stale Timer7 pending state left by an older/previous runtime. */
    IntPendClear(INT_TIMER7A);
    for (d = 0U; d < 3U; d++) {
        Tbl34_DisableTimer(d);
        g_tbl34_rt[d].active = false;
                g_tbl34_rt[d].idx = 0U;
        g_tbl34_rt[d].origin_ticks = 0U;
        Tbl35_InvalidateResetPrepared(d);
    }
    Tbl34_StopProfilePwm();
    g_tbl34_fast_active = false;
    g_tbl34_fast_dds = 0xFFU;
    g_tbl34_fast_idx = 0U;
    g_tbl34_fast_timing_prepared = false;
    g_tbl34_fast_boundary_base = 0UL;
    Tbl354_InvalidateFastCache();
    g_tbl34_gate_running = false;
    g_tbl34_gate_armed = false;
    g_tbl34_gate_done = true;
    g_tbl34_gate_idx = 0U;
    g_tbl365_auto_fast_done = true;
    g_tbl365_auto_fast_running = false;
    g_tbl365_auto_fast_event0_prepared = false;
    g_tbl365_auto_fast_origin = 0U;
    g_tbl365_main_resume_mask = 0U;
    g_tbl365_fast_source = TBL365_FAST_SOURCE_NONE;
        for (d = 0U; d < 3U; d++) {
        g_tbl34_group[d].running = false;
        g_tbl34_group[d].active_dds_mask = 0U;
        g_tbl34_group[d].done_dds_mask = 0U;
    }
    g_tbl34_current_start_ttl = 0xFFU;
    Tbl34_UnmaskMainTriggers();
}

static bool Tbl34_PreloadMainEvent0(unsigned int d)
{
    if (!g_tbl34_def[d].valid || g_tbl34_def[d].n_events == 0U) return false;
    if (!Tbl34_PreloadEvent(d,&g_tbl34_ev[d][0],g_tbl34_blob[d])) return false;
    Tbl34_WaitSsiIdle(d);
    g_tbl34_rt[d].event0_prepared = true;
    return true;
}

static bool Tbl34_PreloadGateEvent0(void)
{
    unsigned int d;
    if (!g_tbl34_gate_def.valid || g_tbl34_gate_def.n_events == 0U) return false;
    d = g_tbl34_gate_def.dds;
    if (!Tbl34_PreloadEvent(d,&g_tbl34_gate_ev[0],g_tbl34_gate_blob)) return false;
    Tbl34_WaitSsiIdle(d);
    g_tbl34_gate_event0_prepared = true;
    return true;
}

static bool Tbl34_PreloadNextMain(unsigned int d)
{
    uint16_t idx = g_tbl34_rt[d].idx;
    const tbl34_event_t *ev;
    if (idx >= g_tbl34_def[d].n_events) return true;
    ev = &g_tbl34_ev[d][idx];
    /* PREPARE owns its serialization at its own explicit deadline. COMMIT has
     * no blob. Neither participates in the normal preload mechanism. */
    if ((ev->action & (TBL34_ACTION_SWEEP_EXIT_PREPARE | TBL34_ACTION_SWEEP_EXIT_COMMIT)) != 0U)
        return true;
    return Tbl34_PreloadEvent(d,ev,g_tbl34_blob[d]);
}

static bool Tbl34_PreloadNextGate(void)
{
    unsigned int d = g_tbl34_gate_def.dds;
    const tbl34_event_t *ev;
    if (g_tbl34_gate_idx >= g_tbl34_gate_def.n_events) return true;
    ev = &g_tbl34_gate_ev[g_tbl34_gate_idx];
    if ((ev->action & (TBL34_ACTION_SWEEP_EXIT_PREPARE | TBL34_ACTION_SWEEP_EXIT_COMMIT)) != 0U)
        return true;
    return Tbl34_PreloadEvent(d,ev,g_tbl34_gate_blob);
}


static bool Tbl365_PreloadActiveFastEvent0(void)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    tbl34_event_t *events = Tbl365_ActiveFastEvents();
    uint8_t *blob = Tbl365_ActiveFastBlob();
    unsigned int d;
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_NONE || !def->valid || def->n_events == 0U) return false;
    d = def->dds;
    if (!Tbl34_PreloadEvent(d,&events[0],blob)) return false;
    Tbl34_WaitSsiIdle(d);
    Tbl365_SetActiveFastEvent0Prepared(true);
    return true;
}

static bool Tbl365_PreloadNextFast(void)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    tbl34_event_t *events = Tbl365_ActiveFastEvents();
    uint8_t *blob = Tbl365_ActiveFastBlob();
    const tbl34_event_t *ev;
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_NONE) return false;
    if (g_tbl34_fast_idx >= def->n_events) return true;
    ev = &events[g_tbl34_fast_idx];
    if ((ev->action & (TBL34_ACTION_SWEEP_EXIT_PREPARE | TBL34_ACTION_SWEEP_EXIT_COMMIT)) != 0U)
        return true;
    return Tbl34_PreloadEvent(def->dds,ev,blob);
}

/* -------------------------------------------------------------------------
 * V36.3 deferred preload
 * -------------------------------------------------------------------------
 * Only ordinary main-table events are deferred.  Native sweep transitions,
 * gate/FAST handoffs and phase-pulse special actions retain the V36 inline
 * behavior.  This makes the optimization independent of register type:
 * ordinary FTW, CPOW and ACR blobs all use the same path.
 */
static bool Tbl362_MainEventNeedsPreload(const tbl34_event_t *ev)
{
    if (ev == (const tbl34_event_t *)0 || ev->blob_len == 0U) return false;
    if ((ev->action & (TBL34_ACTION_SWEEP_EXIT_PREPARE
                     | TBL34_ACTION_SWEEP_EXIT_COMMIT)) != 0U) return false;
    return true;
}

static bool Tbl362_MainDeferSafe(unsigned int d,
                                 const tbl34_event_t *current,
                                 const tbl34_event_t *next)
{
    const uint8_t special =
        TBL34_ACTION_GATE_ARM
      | TBL34_ACTION_FAST_CHAIN
      | TBL34_ACTION_PHASE_PULSE
      | TBL34_ACTION_SWEEP_START
      | TBL34_ACTION_SWEEP_EXIT_PREPARE
      | TBL34_ACTION_SWEEP_EXIT_COMMIT;

    if (current == (const tbl34_event_t *)0 || next == (const tbl34_event_t *)0)
        return false;
    if ((current->action & special) != 0U || (next->action & special) != 0U)
        return false;
    /* A gate/FAST stream owns only its own DDS timer/SSI.  V36.2 disabled
     * deferred preloading globally whenever any DDS was gated/FAST, which
     * reintroduced cross-DDS blocking in realistic mixed scenarios. */
    if (g_tbl34_gate_running && g_tbl34_gate_def.dds == d) return false;
    if (g_tbl34_fast_active && g_tbl34_fast_dds == (uint8_t)d) return false;
    return true;
}

/* Schedule the event currently selected by g_tbl34_rt[d].idx.
 *
 * Long ordinary gaps:
 *   mark preload pending -> arm the owning DDS timer for an early wake.
 *   The first due timer preloads every DDS that shares the same deadline.
 *
 * Short/special gaps:
 *   preserve the historical preload-before-arm sequence.
 */
static bool Tbl362_ScheduleNextMain(unsigned int d,
                                    const tbl34_event_t *current)
{
    volatile tbl34_rt_t *rt;
    const tbl34_event_t *next;
    uint32_t deadline;
    uint32_t now;
    uint32_t remain;
    uint8_t bit;

    if (d >= TBL34_DDS_COUNT) return false;
    rt = &g_tbl34_rt[d];
    bit = (uint8_t)(1U << d);

    if (rt->idx >= g_tbl34_def[d].n_events) {
        g_tbl362_main_preload_pending_mask &= (uint8_t)~bit;
        return true;
    }

    next = &g_tbl34_ev[d][rt->idx];
    deadline = rt->origin_ticks + next->at_ticks;

    if (!Tbl362_MainEventNeedsPreload(next)) {
        g_tbl362_main_preload_pending_mask &= (uint8_t)~bit;
        Tbl34_ArmEventDeadline(d,next,deadline);
        return true;
    }

    now = TBL34_DWT_CYCCNT;
    remain = deadline - now;

    if (!Tbl362_MainDeferSafe(d,current,next)
        || (int32_t)remain <= (int32_t)TBL364_DEFER_MIN_SLACK_TICKS) {
        g_tbl362_main_preload_pending_mask &= (uint8_t)~bit;
        g_tbl362_dbg_inline_short_preload_count++;
        if (!Tbl34_PreloadNextMain(d)) return false;
        Tbl34_ArmEventDeadline(d,next,deadline);
        return true;
    }

    /* V36.4: the pending bit means that the event is intentionally NOT yet
     * serialized.  Wake this DDS timer 75 us before the absolute deadline.
     * The first timer ISR then preloads every DDS sharing that deadline before
     * any of them is committed.  No background/Timer7 worker is required. */
    g_tbl362_main_preload_pending_mask |= bit;
    g_tbl362_dbg_deferred_preload_count++;
    Tbl34_ArmDeadlineWithLead(d,deadline,TBL364_EARLY_WAKE_LEAD_TICKS);
    return true;
}

static void Tbl34_CheckGroupFinish(uint8_t ttl)
{
    volatile tbl34_group_rt_t *grp;
    bool waits_gate;
    unsigned int d;
    if (ttl >= 3U) return;
    grp = &g_tbl34_group[ttl];
    if (!grp->running) return;
    if ((grp->done_dds_mask & grp->active_dds_mask) != grp->active_dds_mask) return;
    waits_gate = g_tbl34_gate_def.valid
              && g_tbl34_gate_def.start_ttl_index == ttl
              && !g_tbl34_gate_done;
    if (waits_gate) return;
    if (g_tbl365_fast_def.valid
        && g_tbl365_fast_def.start_ttl_index == ttl
        && !g_tbl365_auto_fast_done) return;

    grp->running = false;
    g_tbl34_dbg_sequence_completed++;
    g_tbl34_current_start_ttl = 0xFFU;

    /* Prepare event 0 before re-enabling an IGNORE-mode main trigger.  This
     * removes the V33 rearm window completely: a new edge cannot be accepted
     * until every participating DDS already contains its event-0 registers. */
    for (d = 0U; d < 3U; d++) {
        if ((grp->active_dds_mask & (1U << d)) != 0U && g_tbl34_def[d].valid) {
            (void)Tbl34_PreloadMainEvent0(d);
        }
    }
    Tbl34_UnmaskMainTriggers();
}

static void Tbl34_MarkMainDone(unsigned int d)
{
    uint8_t ttl = g_tbl34_rt[d].start_ttl_index;
    bool fast_owns_timer = g_tbl34_fast_active && g_tbl34_fast_dds == (uint8_t)d;
    uint8_t bit = (uint8_t)(1U << d);
    g_tbl362_main_preload_pending_mask &= (uint8_t)~bit;
    g_tbl365_main_resume_mask &= (uint8_t)~bit;
    g_tbl34_rt[d].active = false;
    if (fast_owns_timer) {
        unsigned long base = Tbl34_TimerBase(d);
        g_tbl351_dbg_fast_handoff_count++;
        if ((HWREG(base + TIMER_O_CTL) & TIMER_CTL_TAEN) != 0U)
            g_tbl351_dbg_fast_handoff_timer_kept_count++;
        else
            g_tbl351_dbg_fast_handoff_error_count++;
        /* V35.0 killed this same periodic timer here.  Ownership has already
         * moved to g_tbl34_fast_active; the fast final event will stop it. */
    } else {
        Tbl34_DisableTimer(d);
    }
    if (ttl < 3U) {
        g_tbl34_group[ttl].done_dds_mask |= (uint8_t)(1U << d);
        Tbl34_CheckGroupFinish(ttl);
    }
}

/* Select the pending DDS whose next absolute deadline is closest. */
/* Queue deferred main-event register blobs in the V36.3 low-priority worker.
 *
 * The timer itself keeps counting. Only this DDS timer interrupt is masked
 * while bytes are queued and SSI_BSY is drained, which prevents a same-DDS
 * timeout from pre-empting a half-written blob. Other DDS timer interrupts
 * remain enabled and can pre-empt this service routine.
 *
 * If the deadline expires while masked, the raw timeout remains pending and
 * is delivered immediately when TIMER_TIMA_TIMEOUT is re-enabled.
 */
void Tbl34_PreloadWorkerIntHandler(void)
{
    /* Compatibility entry point only. V36.4+ no longer uses Timer7 for main
     * preloads; main.c keeps the vector registered for the legacy engine. */
}

static bool Tbl34_StartFastGate(void);

static void Tbl34_MarkGateDone(void)
{
    uint8_t ttl = g_tbl34_gate_def.start_ttl_index;
    unsigned int d = g_tbl34_gate_def.dds;
    g_tbl34_gate_running = false;
    g_tbl34_gate_done = true;
    g_tbl34_gate_armed = false;
    g_tbl34_gate_idx = 0U;
    g_tbl34_gate_event0_prepared = false;
    g_tbl356_auto_event0_committed = false;
    g_tbl356_triangle_exit_pending = false;
        g_tbl34_rt[d].active = false;
    Tbl34_DisableTimer(d);
    (void)Tbl34_PreloadMainEvent0(d);
    Tbl34_CheckGroupFinish(ttl);
}


static void Tbl365_MarkActiveFastDone(void)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    uint8_t ttl = def->start_ttl_index;
    unsigned int d = def->dds;

    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_GATE) {
        Tbl34_MarkGateDone();
        g_tbl365_fast_source = TBL365_FAST_SOURCE_NONE;
        return;
    }
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_AUTO) {
        uint8_t bit = (uint8_t)(1U << d);
        g_tbl365_auto_fast_done = true;
        g_tbl365_auto_fast_running = false;
        g_tbl365_auto_fast_event0_prepared = false;
        g_tbl356_auto_event0_committed = false;
        g_tbl356_triangle_exit_pending = false;
        Tbl34_DisableTimer(d);
        g_tbl365_fast_source = TBL365_FAST_SOURCE_NONE;

        /* Same-DDS time multiplexing: if another channel has later main-table
         * boundaries, return Timer4/5/6 to that merged table now. The compiler
         * rejects any boundary/native sweep that overlaps the periodic interval. */
        if ((g_tbl365_main_resume_mask & bit) != 0U) {
            const tbl34_event_t *current;
            g_tbl365_main_resume_mask &= (uint8_t)~bit;
            if (g_tbl34_rt[d].active && g_tbl34_rt[d].idx < g_tbl34_def[d].n_events) {
                current = (g_tbl34_rt[d].idx > 0U)
                    ? &g_tbl34_ev[d][g_tbl34_rt[d].idx - 1U]
                    : &g_tbl34_ev[d][0];
                if (!Tbl362_ScheduleNextMain(d,current)) {
                    g_tbl34_dbg_preload_error[d]++;
                    g_tbl34_dbg_sequence_aborted++;
                    Tbl34_MarkMainDone(d);
                }
                return;
            }
        }

        (void)Tbl34_PreloadMainEvent0(d);
        Tbl34_CheckGroupFinish(ttl);
    }
}

static void Tbl34_ProcessMainEvent(unsigned int d)
{
    volatile tbl34_rt_t *rt = &g_tbl34_rt[d];
    const tbl34_event_t *ev;
    uint32_t deadline;
    uint8_t bit;
    if (!rt->active || rt->idx >= g_tbl34_def[d].n_events) return;
    ev = &g_tbl34_ev[d][rt->idx];
    deadline = rt->origin_ticks + ev->at_ticks;
    bit = (uint8_t)(1U << d);

    /* Sécurité : si un preload différé n'a pas été terminé avant la deadline,
     * this event. If the worker was delayed until the timer fired,
     * preserve correctness by preloading here and expose the fallback count.
     * This path may be late, but it can never commit a half/missing blob. */
    if ((g_tbl362_main_preload_pending_mask & bit) != 0U) {
        g_tbl362_dbg_emergency_preload_count++;
        g_tbl362_main_preload_pending_mask &= (uint8_t)~bit;
        if (!Tbl34_PreloadNextMain(d)) {
            g_tbl34_dbg_sequence_aborted++;
            Tbl34_MarkMainDone(d);
            return;
        }
        Tbl34_WaitSsiIdle(d);
    }

    /* V34.25: every phase is a distinct table event index. */
    if (!Tbl34_CommitGeneric(d,ev,g_tbl34_blob[d],deadline)) {
        g_tbl34_dbg_sequence_aborted++;
        Tbl34_MarkMainDone(d);
        return;
    }
    /* V36.5: GATED and periodic FAST are independent resources. */
    if ((ev->action & TBL34_ACTION_GATE_ARM) != 0U
        && g_tbl365_fast_def.valid
        && g_tbl365_fast_def.dds == d
        && g_tbl365_fast_def.start_ttl_index == rt->start_ttl_index
        && (g_tbl365_fast_def.flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U) {
        g_tbl356_dbg_triangle_cfr = Tbl356_FindCfrInEvent(ev,g_tbl34_blob[d]);
        if ((g_tbl356_dbg_triangle_cfr & 0x002000UL) == 0U)
            g_tbl356_dbg_triangle_no_ioupdate_srr_count++;
    }

    /* True secondary-TTL gate: arm now, release later from GPIO TTL. */
    if ((ev->action & TBL34_ACTION_GATE_ARM) != 0U
        && g_tbl34_gate_def.valid
        && g_tbl34_gate_def.dds == d
        && g_tbl34_gate_def.start_ttl_index == rt->start_ttl_index) {
        g_tbl35_dbg_gate_arm_count++;
        g_tbl34_gate_armed = true;
        g_tbl34_gate_done = false;
        g_tbl34_gate_running = false;
        g_tbl34_gate_idx = 0U;
        g_tbl34_gate_event0_prepared = Tbl34_PreloadGateEvent0();
        g_tbl356_auto_event0_committed = false;
        if (Tbl351_GateUsesFastTiming() && g_tbl34_gate_def.n_events >= 2U) {
            uint32_t p = g_tbl34_gate_ev[1].at_ticks - g_tbl34_gate_ev[0].at_ticks;
            g_tbl365_fast_source = TBL365_FAST_SOURCE_GATE;
            if (!g_tbl352_ultrafast_prepared && !g_tbl354_layout_cached)
                (void)Tbl352_PrepareUltrafastLayout();
            (void)Tbl34_PrepareFastTimingHardware(
                d, g_tbl34_gate_def.channel, p, g_tbl34_gate_def.phase_half_ticks,
                (g_tbl34_gate_def.flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U);
        }
    }

    /* Periodic AUTO_START stream: the main handoff event already committed
     * point 0, therefore start the periodic engine immediately. */
    if ((ev->action & TBL34_ACTION_GATE_ARM) != 0U
        && g_tbl365_fast_def.valid
        && g_tbl365_fast_def.dds == d
        && g_tbl365_fast_def.start_ttl_index == rt->start_ttl_index) {
        uint32_t p = Tbl365_AutoFastEvents()[1].at_ticks - Tbl365_AutoFastEvents()[0].at_ticks;
        g_tbl365_fast_source = TBL365_FAST_SOURCE_AUTO;
        g_tbl365_auto_fast_done = false;
        g_tbl365_auto_fast_event0_prepared = false;
        g_tbl356_auto_event0_committed = true;
        g_tbl354_gate_arm_commit_tick = TBL34_DWT_CYCCNT;
        if (!g_tbl352_ultrafast_prepared && !g_tbl354_layout_cached)
            (void)Tbl352_PrepareUltrafastLayout();
        (void)Tbl34_PrepareFastTimingHardware(
            d, g_tbl365_fast_def.channel, p, g_tbl365_fast_def.phase_half_ticks,
            (g_tbl365_fast_def.flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U);
        g_tbl35_dbg_auto_fast_start_count++;
        if (!Tbl34_StartFastGate()) {
            g_tbl34_dbg_sequence_aborted++;
            Tbl34_ResetRuntimeOnly();
            return;
        }
    }

    if ((ev->action & TBL34_ACTION_STREAM_END) != 0U) {
        Tbl34_MarkMainDone(d);
        return;
    }

    rt->idx++;
    if (g_tbl365_auto_fast_running && g_tbl365_fast_def.dds == d
        && rt->idx < g_tbl34_def[d].n_events) {
        g_tbl365_main_resume_mask |= (uint8_t)(1U << d);
        return;
    }
    if (!Tbl362_ScheduleNextMain(d,ev)) {
        g_tbl34_dbg_preload_error[d]++;
        Tbl34_MarkMainDone(d);
        return;
    }
}


/* V36.4 synchronized early-preload barrier.
 *
 * This routine is entered when a main DDS timer fires while its current event
 * is still marked "preload pending".  Because deferred timers are armed with
 * a 75-us wake lead, this normally happens well BEFORE the physical boundary.
 *
 * If other DDSs have a pending event at the same compiled absolute deadline,
 * all register blobs are serialized first.  Only after every participating
 * SSI is idle do we wait for the common deadline and call the ordinary event
 * processor for each DDS back-to-back.  Therefore the expensive preload time
 * is common-mode latency and no longer appears as DDS1->DDS2->DDS3 skew.
 */
static void Tbl364_ProcessDeferredDeadlineGroup(unsigned int anchor_d)
{
    uint8_t pending;
    uint8_t group_mask = 0U;
    uint8_t ready_mask = 0U;
    uint32_t anchor_deadline;
    uint32_t preload_t0;
    uint32_t preload_dt;
    unsigned int d;

    if (anchor_d >= TBL34_DDS_COUNT) return;
    if (!g_tbl34_rt[anchor_d].active
        || g_tbl34_rt[anchor_d].idx >= g_tbl34_def[anchor_d].n_events) return;

    pending = g_tbl362_main_preload_pending_mask;
    if ((pending & (1U << anchor_d)) == 0U) {
        Tbl34_ProcessMainEvent(anchor_d);
        return;
    }

    anchor_deadline = g_tbl34_rt[anchor_d].origin_ticks
                    + g_tbl34_ev[anchor_d][g_tbl34_rt[anchor_d].idx].at_ticks;

    /* Build a group only from events intentionally deferred by V36.4 and
     * sharing the same compiled deadline (within two 120-MHz ticks). */
    for (d = 0U; d < TBL34_DDS_COUNT; d++) {
        uint8_t bit = (uint8_t)(1U << d);
        uint32_t deadline;
        int32_t delta;
        if ((pending & bit) == 0U) continue;
        if (!g_tbl34_rt[d].active || g_tbl34_rt[d].idx >= g_tbl34_def[d].n_events) continue;
        if (g_tbl34_gate_running && g_tbl34_gate_def.dds == d) continue;
        if (g_tbl34_fast_active && g_tbl34_fast_dds == (uint8_t)d) continue;
        deadline = g_tbl34_rt[d].origin_ticks + g_tbl34_ev[d][g_tbl34_rt[d].idx].at_ticks;
        delta = (int32_t)(deadline - anchor_deadline);
        if (delta >= -(int32_t)TBL364_SYNC_DEADLINE_TOL_TICKS
            && delta <= (int32_t)TBL364_SYNC_DEADLINE_TOL_TICKS) {
            group_mask |= bit;
        }
    }

    if (group_mask == 0U) group_mask = (uint8_t)(1U << anchor_d);
    g_tbl364_dbg_early_preload_irq_count++;

    /* Suppress duplicate timer ISR delivery for members we are about to
     * service manually.  Scheduling of the NEXT event will re-arm each timer. */
    for (d = 0U; d < TBL34_DDS_COUNT; d++) {
        uint8_t bit = (uint8_t)(1U << d);
        if ((group_mask & bit) == 0U) continue;
        Tbl34_DisableTimer(d);
        IntPendClear(Tbl34_TimerInterrupt(d));
    }

    preload_t0 = TBL34_DWT_CYCCNT;
    for (d = 0U; d < TBL34_DDS_COUNT; d++) {
        uint8_t bit = (uint8_t)(1U << d);
        if ((group_mask & bit) == 0U) continue;

        if (!Tbl34_PreloadNextMain(d)) {
            g_tbl34_dbg_sequence_aborted++;
            g_tbl362_main_preload_pending_mask &= (uint8_t)~bit;
            Tbl34_MarkMainDone(d);
            continue;
        }
        Tbl34_WaitSsiIdle(d);
        g_tbl362_main_preload_pending_mask &= (uint8_t)~bit;
        g_tbl362_dbg_service_preload_count++;
        ready_mask |= bit;
    }
    preload_dt = TBL34_DWT_CYCCNT - preload_t0;
    g_tbl364_dbg_preload_batch_ticks = preload_dt;
    if (preload_dt > g_tbl364_dbg_preload_batch_max_ticks)
        g_tbl364_dbg_preload_batch_max_ticks = preload_dt;
    if ((int32_t)(TBL34_DWT_CYCCNT - anchor_deadline) > 0)
        g_tbl364_dbg_preload_batch_late_count++;

    /* Capture the physical I/O_UPDATE rising-edge ticks while the ordinary
     * per-DDS event processor performs all existing gate/sweep/bookkeeping. */
    g_tbl364_sync_update_tick[0] = 0U;
    g_tbl364_sync_update_tick[1] = 0U;
    g_tbl364_sync_update_tick[2] = 0U;
    g_tbl364_sync_capture_mask = ready_mask;

    for (d = 0U; d < TBL34_DDS_COUNT; d++) {
        if ((ready_mask & (1U << d)) != 0U) Tbl34_ProcessMainEvent(d);
    }
    g_tbl364_sync_capture_mask = 0U;

    g_tbl364_dbg_sync_last_mask = ready_mask;
    {
        bool have_tick = false;
        uint32_t first_tick = 0U;
        uint32_t last_tick = 0U;
        unsigned int count = 0U;
        for (d = 0U; d < TBL34_DDS_COUNT; d++) {
            uint32_t tick = g_tbl364_sync_update_tick[d];
            if (tick == 0U) continue;
            if (!have_tick) {
                first_tick = tick;
                last_tick = tick;
                have_tick = true;
            } else {
                if ((int32_t)(tick - first_tick) < 0) first_tick = tick;
                if ((int32_t)(tick - last_tick) > 0) last_tick = tick;
            }
            count++;
        }
        if (count >= 2U) {
            uint32_t skew = last_tick - first_tick;
            g_tbl364_dbg_sync_group_count++;
            g_tbl364_dbg_sync_last_skew_ticks = skew;
            if (skew > g_tbl364_dbg_sync_max_skew_ticks)
                g_tbl364_dbg_sync_max_skew_ticks = skew;
        } else {
            g_tbl364_dbg_sync_last_skew_ticks = 0U;
        }
    }
}

static void Tbl34_ProcessGateEvent(void)
{
    unsigned int d = g_tbl34_gate_def.dds;
    const tbl34_event_t *ev;
    uint32_t deadline;
    if (!g_tbl34_gate_running || g_tbl34_gate_idx >= g_tbl34_gate_def.n_events) return;
    ev = &g_tbl34_gate_ev[g_tbl34_gate_idx];
    deadline = g_tbl34_gate_origin + ev->at_ticks;

    /* Same explicit PREPARE/COMMIT semantics for gated native sweeps. */
    if (!Tbl34_CommitGeneric(d,ev,g_tbl34_gate_blob,deadline)) {
        g_tbl34_dbg_sequence_aborted++;
        Tbl34_MarkGateDone();
        return;
    }
    if ((ev->action & TBL34_ACTION_STREAM_END) != 0U) {
        Tbl34_MarkGateDone();
        return;
    }
    g_tbl34_gate_idx++;
    if (!Tbl34_PreloadNextGate()) {
        g_tbl34_dbg_preload_error[d]++;
        Tbl34_MarkGateDone();
        return;
    }
    if (g_tbl34_gate_idx < g_tbl34_gate_def.n_events) {
        const tbl34_event_t *next = &g_tbl34_gate_ev[g_tbl34_gate_idx];
        Tbl34_ArmEventDeadline(d, next, g_tbl34_gate_origin + next->at_ticks);
    }
}


static void Tbl365_ProcessAutoFastEvent(void)
{
    unsigned int d = g_tbl365_fast_def.dds;
    const tbl34_event_t *ev;
    uint32_t deadline;
    if (!g_tbl365_auto_fast_running || g_tbl34_fast_idx >= g_tbl365_fast_def.n_events) return;
    ev = &Tbl365_AutoFastEvents()[g_tbl34_fast_idx];
    deadline = g_tbl365_auto_fast_origin + ev->at_ticks;
    if (!Tbl34_CommitGeneric(d,ev,Tbl365_AutoFastBlob(),deadline)) {
        g_tbl34_dbg_sequence_aborted++;
        Tbl365_MarkActiveFastDone();
        return;
    }
    if ((ev->action & TBL34_ACTION_STREAM_END) != 0U) {
        Tbl365_MarkActiveFastDone();
        return;
    }
    g_tbl34_fast_idx++;
    if (!Tbl365_PreloadNextFast()) {
        g_tbl34_dbg_preload_error[d]++;
        Tbl365_MarkActiveFastDone();
        return;
    }
    if (g_tbl34_fast_idx < g_tbl365_fast_def.n_events) {
        const tbl34_event_t *next = &Tbl365_AutoFastEvents()[g_tbl34_fast_idx];
        Tbl34_ArmEventDeadline(d,next,g_tbl365_auto_fast_origin + next->at_ticks);
    }
}

static bool Tbl34_StartFastGate(void)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    tbl34_event_t *events = Tbl365_ActiveFastEvents();
    unsigned int d = def->dds;
    unsigned int c = def->channel;
    unsigned long base = Tbl34_TimerBase(d);
    const tbl34_event_t *ev0;
    uint32_t period;
    uint32_t origin;
    uint32_t update_tick;
    bool event0_already_committed;

    g_tbl35_dbg_fast_start_attempt_count++;
    g_tbl35_dbg_fast_start_fail_code = 0U;
    if (!g_tbl354_fast_gate_validated) {
        if (!Tbl34_ValidateFastGate()) {
            g_tbl35_dbg_fast_start_fail_code = 1U;
            return false;
        }
        g_tbl354_fast_gate_validated = true;
    }
    event0_already_committed = g_tbl356_auto_event0_committed
        && ((def->flags & TBL34_GATE_FLAG_AUTO_START) != 0U);
    if (!event0_already_committed
        && !Tbl365_ActiveFastEvent0Prepared()
        && !Tbl365_PreloadActiveFastEvent0()) {
        g_tbl35_dbg_fast_start_fail_code = 2U;
        return false;
    }
    ev0 = &events[0];
    period = events[1].at_ticks - ev0->at_ticks;

    /* V35.7: never rescan 250/450 FAST_CHAIN events on the real-time start
     * path.  Tbl352_PrepareUltrafastLayout() already validated the uniform
     * prefix and cached its exact length. */
    if (!g_tbl352_ultrafast_prepared) {
        if (!Tbl352_PrepareUltrafastLayout()) {
            g_tbl354_layout_cached = false;
            g_tbl35_dbg_fast_start_fail_code = 3U;
            return false;
        }
    }
    g_tbl351_dbg_fast_expected_events = g_tbl352_ultrafast_fast_count;
    if (event0_already_committed) {
        /* AUTO_START: point 0 became physically active at the GATE_ARM
         * I/O_UPDATE.  Reusing it removes the ~47-us redundant SSI round trip
         * measured in V35.4/V35.5. */
        update_tick = g_tbl354_gate_arm_commit_tick;
        g_tbl354_dbg_start_latency_ticks = 0U;
        g_tbl356_dbg_auto_reuse_count++;
    } else {
        Tbl34_WaitSsiIdle(d);
        Tbl34_ApplyProfileLowMask(d,ev0->profile_ops);
        if ((ev0->action & TBL34_ACTION_IO_UPDATE) != 0U
            && (def->flags & TBL3512_GATE_FLAG_SUPPRESS_FAST_IOUPDATE) == 0U)
            Tbl34_IOUpdateOne(d);
        /* In L0 there is intentionally no physical UPDATE here.  The timer/PWM
         * origin is still captured at gate release so L0/L1/L2 have the same
         * 2-us timing reference. */
        update_tick = TBL34_DWT_CYCCNT;
        if (g_tbl354_gate_arm_commit_tick != 0U)
            g_tbl354_dbg_start_latency_ticks = update_tick - g_tbl354_gate_arm_commit_tick;
    }
    g_tbl356_auto_event0_committed = false;
    Tbl351_CountFastRegisterFrames(ev0);
    g_tbl351_dbg_fast_last_index = 0U;
    g_tbl34_dbg_commit_count[d]++;
    g_tbl34_dbg_fast_points_committed++;

    g_tbl34_fast_active = true;
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_AUTO) g_tbl365_auto_fast_running = true;
    g_tbl34_fast_dds = (uint8_t)d;
    g_tbl34_fast_idx = 1U;
    g_tbl34_fast_period_ticks = period;
    g_tbl34_dbg_fast_period_ticks = period;
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_GATE) g_tbl34_gate_running = true;
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_GATE) g_tbl34_gate_idx = 1U;
    Tbl365_SetActiveFastOrigin(update_tick);
    Tbl365_SetActiveFastEvent0Prepared(false);

    if (!g_tbl34_fast_timing_prepared) {
        if (!Tbl34_PrepareFastTimingHardware(
                d, c, period, def->phase_half_ticks,
                (def->flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U)) {
            g_tbl35_dbg_fast_start_fail_code = 3U; return false;
        }
    }

    if (g_tbl352_ultrafast_prepared) {
        /* Queue event 1 while the timer is stopped. */
        Tbl353_QueueDirectFrame(g_tbl352_ultrafast_next_frame);
        g_tbl353_direct_frame = g_tbl352_ultrafast_next_frame;
        g_tbl353_direct_total_irqs = (uint16_t)(g_tbl352_ultrafast_fast_count - 1U);
        g_tbl353_direct_remaining = g_tbl353_direct_total_irqs;
        g_tbl353_direct_active = true;
        g_tbl353_dbg_direct_used_count++;
        if ((def->flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U)
            g_tbl354_dbg_triangle_direct_count++;
        g_tbl353_dbg_direct_expected_irqs = g_tbl353_direct_total_irqs;
        g_tbl353_dbg_direct_irqs_done = 0U;
        g_tbl353_dbg_direct_expected_runtime_ticks = (uint32_t)g_tbl353_direct_total_irqs * period;
        g_tbl353_dbg_direct_runtime_ticks = 0U;
        g_tbl353_dbg_direct_excess_ticks = 0U;
        g_tbl353_dbg_direct_average_period_ticks = 0U;
    } else {
        g_tbl353_dbg_direct_fallback_count++;
        if (!Tbl365_PreloadNextFast()) {
            g_tbl35_dbg_fast_start_fail_code = 5U;
            g_tbl35_dbg_fast_preload_error_count++;
            return false;
        }
        g_tbl35_dbg_fast_preload_count++;
    }

    if (!Tbl34_StartPreparedFastTiming(&origin)) {
        g_tbl353_direct_active = false;
        g_tbl35_dbg_fast_start_fail_code = 4U; return false;
    }
    g_tbl353_dbg_direct_timer_tailr = HWREG(base + TIMER_O_TAILR);

    /* Gate timestamps are defined from the physical point-0 I/O_UPDATE.
     * The periodic timer may be enabled a little later; direct runtime stats
     * retain that separate timer origin. */
    Tbl365_SetActiveFastOrigin(update_tick);
    g_tbl34_fast_next_deadline = origin + period;
    if (g_tbl353_direct_active) g_tbl353_direct_start_tick = origin;

    g_tbl35_dbg_fast_start_ok_count++;
    return true;
}

static void Tbl353_FinishDirectStream(unsigned int d, unsigned long base)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    tbl34_event_t *events = Tbl365_ActiveFastEvents();
    uint16_t fast_count = g_tbl352_ultrafast_fast_count;
    const tbl34_event_t *last_ev;
    uint32_t runtime;
    uint32_t expected;

    if (fast_count == 0U) return;
    last_ev = &events[fast_count - 1U];
    runtime = TBL34_DWT_CYCCNT - g_tbl353_direct_start_tick;
    expected = (uint32_t)g_tbl353_direct_total_irqs * g_tbl34_fast_period_ticks;

    /* Publish debug only after the repetitive hot loop. */
    g_tbl353_dbg_direct_irqs_done = g_tbl353_direct_total_irqs;
    g_tbl353_dbg_direct_runtime_ticks = runtime;
    g_tbl353_dbg_direct_expected_runtime_ticks = expected;
    g_tbl353_dbg_direct_excess_ticks = runtime > expected ? runtime - expected : 0U;
    if (g_tbl353_direct_total_irqs != 0U)
        g_tbl353_dbg_direct_average_period_ticks = runtime / g_tbl353_direct_total_irqs;

    g_tbl35_dbg_fast_timer_irq_count += g_tbl353_direct_total_irqs;
    if ((def->flags & TBL3512_GATE_FLAG_SUPPRESS_FAST_IOUPDATE) == 0U)
        g_tbl35_dbg_fast_update_count += g_tbl353_direct_total_irqs;
    g_tbl34_dbg_fast_points_committed += g_tbl353_direct_total_irqs;
    g_tbl34_dbg_commit_count[d] += g_tbl353_direct_total_irqs;
    if (g_tbl352_ultrafast_frame_addr == TBL34_FTW_ADDR)
        g_tbl351_dbg_fast_ftw_frame_count += g_tbl353_direct_total_irqs;
    else if (g_tbl352_ultrafast_frame_addr == TBL34_CPOW_ADDR)
        g_tbl351_dbg_fast_cpow_frame_count += g_tbl353_direct_total_irqs;
    else if (g_tbl352_ultrafast_frame_addr == TBL34_ACR_ADDR)
        g_tbl351_dbg_fast_acr_frame_count += g_tbl353_direct_total_irqs;

    g_tbl34_fast_idx = fast_count;
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_GATE) g_tbl34_gate_idx = fast_count;
    g_tbl351_dbg_fast_last_index = fast_count - 1U;
    g_tbl353_direct_active = false;
    g_tbl353_dbg_direct_final_count++;

    if ((def->flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U
        && (last_ev->action & TBL34_ACTION_STREAM_END) != 0U)
        Tbl34_StopProfilePwm();

    TimerIntDisable(base,TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
    HWREG(base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    g_tbl34_fast_active = false;
    g_tbl34_fast_dds = 0xFFU;
    g_tbl34_fast_timing_prepared = false;
    g_tbl34_fast_boundary_base = 0UL;
    /* Gate definition/layout is immutable until the next LOAD/ARM. */
    Tbl34_ConfigOneShot(d);

    if ((last_ev->action & TBL34_ACTION_STREAM_END) != 0U
        || fast_count >= def->n_events) {
        g_tbl35_dbg_fast_final_count++;
        Tbl365_MarkActiveFastDone();
    } else {
        const tbl34_event_t *next = &events[fast_count];
        g_tbl35_dbg_fast_to_generic_count++;
        g_tbl353_dbg_direct_to_generic_count++;
        if (!Tbl365_PreloadNextFast()) {
            g_tbl34_dbg_preload_error[d]++;
            g_tbl35_dbg_fast_preload_error_count++;
            Tbl365_MarkActiveFastDone();
        } else {
            g_tbl35_dbg_fast_preload_count++;
            Tbl34_ArmEventDeadline(d,next,Tbl365_ActiveFastOrigin() + next->at_ticks);
        }
    }
}

static __attribute__((noinline)) void Tbl357_CommitTriangleExitIRQ(unsigned int d, unsigned long base)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    tbl34_event_t *events = Tbl365_ActiveFastEvents();
    uint8_t *blob = Tbl365_ActiveFastBlob();
    const tbl34_event_t *exit_ev;
    uint16_t fast_count = g_tbl352_ultrafast_fast_count;
    uint32_t deadline;
    uint32_t now;
    uint32_t runtime;
    uint32_t expected;

    if (!g_tbl356_triangle_exit_pending || d != g_tbl34_fast_dds) return;
    if (fast_count >= def->n_events) {
        g_tbl356_triangle_exit_pending = false;
        Tbl353_FinishDirectStream(d,base);
        return;
    }

    exit_ev = &events[fast_count];
    deadline = Tbl365_ActiveFastOrigin() + exit_ev->at_ticks;
    Tbl34_WaitSsiIdle(d);
    now = TBL34_DWT_CYCCNT;
    g_tbl354_dbg_triangle_exit_count++;
    g_tbl354_dbg_triangle_exit_late_ticks =
        (int32_t)(now - deadline) > 0 ? (now - deadline) : 0U;

    if (g_tbl34_profile_pwm_active) Tbl34_StopProfilePwm();
    Tbl34_ApplyProfileLowMask(d,exit_ev->profile_ops);
    Tbl34_ApplyProfileHighMask(d,exit_ev->profile_ops);
    if ((exit_ev->action & TBL34_ACTION_IO_UPDATE) != 0U) Tbl34_IOUpdateOne(d);
    Tbl35_CountEventRegisterFrames(d,exit_ev,blob);
    g_tbl34_dbg_commit_count[d]++;
    g_tbl356_dbg_triangle_exit_irq_count++;
    g_tbl357_dbg_triangle_exit_commit_count++;

    /* The direct statistics stop at the last FAST point, not at the exit IRQ. */
    runtime = g_tbl356_direct_fast_end_tick - g_tbl353_direct_start_tick;
    expected = (uint32_t)g_tbl353_direct_total_irqs * g_tbl34_fast_period_ticks;
    g_tbl353_dbg_direct_irqs_done = g_tbl353_direct_total_irqs;
    g_tbl353_dbg_direct_runtime_ticks = runtime;
    g_tbl353_dbg_direct_expected_runtime_ticks = expected;
    g_tbl353_dbg_direct_excess_ticks = runtime > expected ? runtime - expected : 0U;
    if (g_tbl353_direct_total_irqs != 0U)
        g_tbl353_dbg_direct_average_period_ticks = runtime / g_tbl353_direct_total_irqs;
    g_tbl35_dbg_fast_timer_irq_count += g_tbl353_direct_total_irqs;
    if ((def->flags & TBL3512_GATE_FLAG_SUPPRESS_FAST_IOUPDATE) == 0U)
        g_tbl35_dbg_fast_update_count += g_tbl353_direct_total_irqs;
    g_tbl34_dbg_fast_points_committed += g_tbl353_direct_total_irqs;
    g_tbl34_dbg_commit_count[d] += g_tbl353_direct_total_irqs;
    if (g_tbl352_ultrafast_frame_addr == TBL34_FTW_ADDR)
        g_tbl351_dbg_fast_ftw_frame_count += g_tbl353_direct_total_irqs;
    else if (g_tbl352_ultrafast_frame_addr == TBL34_CPOW_ADDR)
        g_tbl351_dbg_fast_cpow_frame_count += g_tbl353_direct_total_irqs;
    else if (g_tbl352_ultrafast_frame_addr == TBL34_ACR_ADDR)
        g_tbl351_dbg_fast_acr_frame_count += g_tbl353_direct_total_irqs;
    g_tbl351_dbg_fast_last_index = fast_count - 1U;
    g_tbl353_dbg_direct_final_count++;
    g_tbl35_dbg_fast_to_generic_count++;
    g_tbl353_dbg_direct_to_generic_count++;

    g_tbl356_triangle_exit_pending = false;
    g_tbl353_direct_active = false;
    g_tbl34_fast_idx = (uint16_t)(fast_count + 1U);
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_GATE) g_tbl34_gate_idx = (uint16_t)(fast_count + 1U);
    TimerIntDisable(base,TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
    HWREG(base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    g_tbl34_fast_active = false;
    g_tbl34_fast_dds = 0xFFU;
    g_tbl34_fast_timing_prepared = false;
    g_tbl34_fast_boundary_base = 0UL;
    Tbl34_ConfigOneShot(d);

    if ((exit_ev->action & TBL34_ACTION_STREAM_END) != 0U
        || g_tbl34_fast_idx >= def->n_events) {
        g_tbl35_dbg_fast_final_count++;
        Tbl365_MarkActiveFastDone();
    } else if (!Tbl365_PreloadNextFast()) {
        g_tbl34_dbg_preload_error[d]++;
        g_tbl35_dbg_fast_preload_error_count++;
        Tbl365_MarkActiveFastDone();
    } else {
        const tbl34_event_t *next = &events[g_tbl34_fast_idx];
        g_tbl35_dbg_fast_preload_count++;
        Tbl34_ArmEventDeadline(d,next,Tbl365_ActiveFastOrigin() + next->at_ticks);
    }
}

static __attribute__((noinline)) bool Tbl357_PrepareTriangleExitAfterFast(unsigned int d)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    tbl34_event_t *events = Tbl365_ActiveFastEvents();
    uint8_t *blob = Tbl365_ActiveFastBlob();
    uint16_t fast_count = g_tbl352_ultrafast_fast_count;
    const tbl34_event_t *exit_ev;
    uint32_t t0;

    if ((def->flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) == 0U
        || fast_count >= def->n_events)
        return false;

    exit_ev = &events[fast_count];
    t0 = TBL34_DWT_CYCCNT;
    if ((exit_ev->action & TBL34_ACTION_FAST_CHAIN) == 0U
        && exit_ev->at_ticks == (uint32_t)fast_count * g_tbl34_fast_period_ticks
        && Tbl34_PreloadEvent(d,exit_ev,blob)) {
        g_tbl356_direct_fast_end_tick = t0;
        g_tbl356_triangle_exit_pending = true;
        g_tbl356_dbg_triangle_exit_preload_count++;
        g_tbl356_dbg_triangle_exit_preload_ticks = TBL34_DWT_CYCCNT - t0;
        return true;
    }

    g_tbl356_dbg_triangle_exit_preload_error++;
    return false;
}

static inline void Tbl353_DirectPeriodicIRQ(unsigned int d, unsigned long base)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    uint16_t remaining;
    const uint8_t *p;

    HWREG(base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;

    /* Keep the repetitive 2-us path tiny.  The large triangle-exit code lives
     * in a noinline cold helper so it cannot create spills/prologue overhead
     * on every normal FTW interrupt. */
    if (g_tbl356_triangle_exit_pending) {
        Tbl357_CommitTriangleExitIRQ(d,base);
        return;
    }
    if (!g_tbl353_direct_active || d != g_tbl34_fast_dds) return;

    if ((def->flags & TBL3512_GATE_FLAG_SUPPRESS_FAST_IOUPDATE) == 0U)
        Tbl352_PulseIOUpdateFast();
    remaining = (uint16_t)(g_tbl353_direct_remaining - 1U);

    if (remaining != 0U) {
        p = g_tbl353_direct_frame + g_tbl352_ultrafast_frame_len;
        g_tbl353_direct_frame = p;
        Tbl353_QueueDirectFrame(p);
        g_tbl353_direct_remaining = remaining;
        return;
    }

    g_tbl353_direct_remaining = 0U;
    if (Tbl357_PrepareTriangleExitAfterFast(d)) {
        /* Keep periodic Timer A running for exactly one more timeout.  The
         * wrapper routes that timeout back here even though direct_active is
         * now false. */
        g_tbl353_direct_active = false;
        return;
    }

    Tbl353_FinishDirectStream(d,base);
}

static void Tbl34_FastTimerIntHandler(unsigned int d)
{
    tbl34_gate_def_t *def = Tbl365_ActiveFastDef();
    tbl34_event_t *events = Tbl365_ActiveFastEvents();
    unsigned long base = Tbl34_TimerBase(d);
    uint32_t t0 = TBL34_DWT_CYCCNT;
    uint32_t late;
    const tbl34_event_t *ev;

    g_tbl35_dbg_fast_timer_irq_count++;
    HWREG(base + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
    if (!g_tbl34_fast_active || d != g_tbl34_fast_dds) return;
    if (g_tbl34_fast_idx >= def->n_events) return;
while ((int32_t)(TBL34_DWT_CYCCNT - g_tbl34_fast_next_deadline) < 0) { }
    late = (int32_t)(TBL34_DWT_CYCCNT - g_tbl34_fast_next_deadline) > 0
         ? (TBL34_DWT_CYCCNT - g_tbl34_fast_next_deadline) : 0U;
    if (late > g_tbl34_dbg_fast_late_max_ticks) g_tbl34_dbg_fast_late_max_ticks = late;

    ev = &events[g_tbl34_fast_idx];
    g_tbl351_dbg_fast_last_index = g_tbl34_fast_idx;
    Tbl34_WaitSsiIdle(d);
    Tbl351_CountFastRegisterFrames(ev);

    if ((ev->action & TBL34_ACTION_STREAM_END) != 0U
        && (def->flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U) {
        Tbl34_StopProfilePwm();
    } else if ((ev->action & TBL34_ACTION_FAST_CHAIN) != 0U
               && g_tbl34_fast_idx + 1U < def->n_events
               && (events[g_tbl34_fast_idx + 1U].action & TBL34_ACTION_FAST_CHAIN) == 0U
               && (def->flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U) {
        /* The last fast boundary may transition into generic suffix events. */
        Tbl34_StopProfilePwm();
    }
    if ((ev->action & TBL34_ACTION_IO_UPDATE) != 0U
        && (def->flags & TBL3512_GATE_FLAG_SUPPRESS_FAST_IOUPDATE) == 0U) {
        Tbl34_IOUpdateOne(d);
        g_tbl35_dbg_fast_update_count++;
    }
    if ((ev->action & TBL34_ACTION_STREAM_END) != 0U) {
        Tbl34_ApplyProfileLowMask(d,ev->profile_ops);
    }

    g_tbl34_dbg_commit_count[d]++;
    g_tbl34_dbg_fast_points_committed++;
    g_tbl34_fast_idx++;
    if (g_tbl365_fast_source == TBL365_FAST_SOURCE_GATE) g_tbl34_gate_idx = g_tbl34_fast_idx;

    if ((ev->action & TBL34_ACTION_STREAM_END) != 0U
        || g_tbl34_fast_idx >= def->n_events) {
        g_tbl35_dbg_fast_final_count++;
        TimerIntDisable(base,TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
        HWREG(base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
        g_tbl34_fast_active = false;
        g_tbl34_fast_dds = 0xFFU;
        g_tbl34_fast_timing_prepared = false;
        g_tbl34_fast_boundary_base = 0UL;
        Tbl34_ConfigOneShot(d);
        Tbl365_MarkActiveFastDone();
    } else if ((events[g_tbl34_fast_idx].action & TBL34_ACTION_FAST_CHAIN) == 0U) {
        const tbl34_event_t *next = &events[g_tbl34_fast_idx];
        g_tbl35_dbg_fast_to_generic_count++;
        TimerIntDisable(base,TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
        HWREG(base + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
        g_tbl34_fast_active = false;
        g_tbl34_fast_dds = 0xFFU;
        g_tbl34_fast_timing_prepared = false;
        g_tbl34_fast_boundary_base = 0UL;
        Tbl34_ConfigOneShot(d);
        if (!Tbl365_PreloadNextFast()) {
            g_tbl34_dbg_preload_error[d]++;
            g_tbl35_dbg_fast_preload_error_count++;
            Tbl365_MarkActiveFastDone();
        } else {
            g_tbl35_dbg_fast_preload_count++;
            Tbl34_ArmEventDeadline(d,next,Tbl365_ActiveFastOrigin() + next->at_ticks);
        }
    } else {
        if (!Tbl365_PreloadNextFast()) {
            g_tbl34_dbg_preload_error[d]++;
            g_tbl35_dbg_fast_preload_error_count++;
            g_tbl34_fast_active = false;
            Tbl365_MarkActiveFastDone();
        } else {
            g_tbl35_dbg_fast_preload_count++;
            g_tbl34_fast_next_deadline += g_tbl34_fast_period_ticks;
        }
    }

    g_tbl34_dbg_fast_isr_ticks = TBL34_DWT_CYCCNT - t0;
    if (g_tbl34_dbg_fast_isr_ticks > g_tbl34_dbg_fast_isr_max_ticks)
        g_tbl34_dbg_fast_isr_max_ticks = g_tbl34_dbg_fast_isr_ticks;
}

static bool Tbl34_ReleaseGate(uint8_t ttl)
{
    unsigned int d;
    const tbl34_event_t *ev0;
    if (!g_tbl34_gate_def.valid || g_tbl34_gate_def.gate_ttl_index >= 3U
        || ttl != g_tbl34_gate_def.gate_ttl_index) return false;
    g_tbl34_dbg_gate_seen[ttl]++;
    if (!g_tbl34_gate_armed) {
        if (g_tbl34_group[g_tbl34_gate_def.start_ttl_index].running) g_tbl34_dbg_gate_too_early++;
        else g_tbl34_dbg_gate_wrong_ttl[ttl]++;
        return false;
    }
    if (g_tbl34_gate_running) return false;
    if (Tbl351_GateUsesFastTiming() && g_tbl34_fast_active) return false;

    g_tbl34_dbg_gate_release_accepted++;
    g_tbl34_dbg_last_gate_release_ticks = TBL34_DWT_CYCCNT;
    g_tbl34_gate_armed = false;

    if (Tbl351_GateUsesFastTiming()) {
        g_tbl365_fast_source = TBL365_FAST_SOURCE_GATE;
        if (!Tbl34_StartFastGate()) {
            g_tbl34_dbg_sequence_aborted++;
            Tbl34_ResetRuntimeOnly();
            return false;
        }
        return true;
    }

    d = g_tbl34_gate_def.dds;
    if (!g_tbl34_gate_event0_prepared && !Tbl34_PreloadGateEvent0()) {
        g_tbl34_dbg_preload_error[d]++;
        return false;
    }
    ev0 = &g_tbl34_gate_ev[0];
    g_tbl34_gate_origin = TBL34_DWT_CYCCNT;
    g_tbl34_gate_idx = 0U;
    g_tbl34_gate_running = true;
    g_tbl34_gate_event0_prepared = false;
        g_tbl34_rt[d].active = true;
    g_tbl34_rt[d].start_ttl_index = g_tbl34_gate_def.start_ttl_index;

    if (!Tbl34_CommitGeneric(d,ev0,g_tbl34_gate_blob,g_tbl34_gate_origin + ev0->at_ticks)) {
        g_tbl34_dbg_sequence_aborted++;
        Tbl34_MarkGateDone();
        return false;
    }
    if ((ev0->action & TBL34_ACTION_STREAM_END) != 0U) {
        Tbl34_MarkGateDone();
        return true;
    }
    g_tbl34_gate_idx = 1U;
    if (!Tbl34_PreloadNextGate()) {
        g_tbl34_dbg_preload_error[d]++;
        Tbl34_MarkGateDone();
        return false;
    }
    Tbl34_ArmEventDeadline(d,&g_tbl34_gate_ev[1], g_tbl34_gate_origin + g_tbl34_gate_ev[1].at_ticks);
    return true;
}

static uint8_t Tbl34_StartMaskForTtl(uint8_t ttl)
{
    uint8_t mask = 0U;
    unsigned int d;
    for (d = 0U; d < 3U; d++) {
        if (g_tbl34_def[d].valid && g_tbl34_def[d].start_ttl_index == ttl) mask |= (uint8_t)(1U << d);
    }
    return mask;
}

static bool Tbl34_StartGroup(uint8_t ttl)
{
    uint8_t mask;
    uint32_t origin;
    unsigned int d;
    volatile tbl34_group_rt_t *grp;
    if (!g_tbl34_armed || ttl >= 3U) return false;
    mask = Tbl34_StartMaskForTtl(ttl);
    if (mask == 0U) return false;

    /* Retrigger policy applies to the complete table group. */
    if (g_tbl34_current_start_ttl < 3U && g_tbl34_group[g_tbl34_current_start_ttl].running) {
        bool accept_restart = false;
        if (g_tbl34_policy == 2U) accept_restart = true;
        else if (g_tbl34_policy == 1U
                 && Tbl34_PriorityRank(ttl) < Tbl34_PriorityRank(g_tbl34_current_start_ttl)) accept_restart = true;
        if (!accept_restart) {
            g_tbl34_dbg_start_ignored_active[ttl]++;
            return false;
        }
        g_tbl34_dbg_sequence_aborted++;
        Tbl34_ResetRuntimeOnly();
    }

    /* ARM preloads event 0. If a previous run did not manage to re-preload it,
     * recover synchronously but expose the fact in diagnostics. */
    for (d = 0U; d < 3U; d++) {
        if ((mask & (1U << d)) != 0U && !g_tbl34_rt[d].event0_prepared) {
            g_tbl34_dbg_start_not_ready[ttl]++;
            if (!Tbl34_PreloadMainEvent0(d)) return false;
        }
    }

    grp = &g_tbl34_group[ttl];
    grp->running = true;
    grp->active_dds_mask = mask;
    grp->done_dds_mask = 0U;
    g_tbl34_current_start_ttl = ttl;
    g_tbl34_gate_done = !(g_tbl34_gate_def.valid && g_tbl34_gate_def.start_ttl_index == ttl);
    g_tbl34_gate_armed = false;
    g_tbl34_gate_running = false;
    g_tbl34_gate_event0_prepared = false;
    g_tbl365_auto_fast_done = !(g_tbl365_fast_def.valid && g_tbl365_fast_def.start_ttl_index == ttl);
    g_tbl365_auto_fast_running = false;
    g_tbl365_auto_fast_event0_prepared = false;
        Tbl34_MaskMainTrigger(ttl);

    origin = TBL34_DWT_CYCCNT + TBL34_INITIAL_GUARD_TICKS;
    g_tbl34_dbg_start_accepted[ttl]++;
    g_tbl34_dbg_sequence_started++;

    for (d = 0U; d < 3U; d++) {
        const tbl34_event_t *ev0;
        if ((mask & (1U << d)) == 0U) continue;
        g_tbl34_rt[d].active = true;
        g_tbl34_rt[d].event0_prepared = false;
                g_tbl34_rt[d].idx = 0U;
        g_tbl34_rt[d].origin_ticks = origin;
        g_tbl34_rt[d].start_ttl_index = ttl;
        ev0 = &g_tbl34_ev[d][0];
        if (!Tbl34_CommitGeneric(d,ev0,g_tbl34_blob[d],origin + ev0->at_ticks)) {
            g_tbl34_dbg_sequence_aborted++;
            Tbl34_MarkMainDone(d);
            continue;
        }

        if ((ev0->action & TBL34_ACTION_GATE_ARM) != 0U
            && g_tbl365_fast_def.valid
            && g_tbl365_fast_def.dds == d
            && (g_tbl365_fast_def.flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U) {
            g_tbl356_dbg_triangle_cfr = Tbl356_FindCfrInEvent(ev0,g_tbl34_blob[d]);
            if ((g_tbl356_dbg_triangle_cfr & 0x002000UL) == 0U)
                g_tbl356_dbg_triangle_no_ioupdate_srr_count++;
        }

        if ((ev0->action & TBL34_ACTION_GATE_ARM) != 0U
            && g_tbl34_gate_def.valid && g_tbl34_gate_def.dds == d) {
            g_tbl35_dbg_gate_arm_count++;
            g_tbl34_gate_armed = true;
            g_tbl34_gate_done = false;
            g_tbl34_gate_running = false;
            g_tbl34_gate_idx = 0U;
            g_tbl34_gate_event0_prepared = Tbl34_PreloadGateEvent0();
            g_tbl356_auto_event0_committed = false;
            if (Tbl351_GateUsesFastTiming() && g_tbl34_gate_def.n_events >= 2U) {
                uint32_t pfast = g_tbl34_gate_ev[1].at_ticks - g_tbl34_gate_ev[0].at_ticks;
                g_tbl365_fast_source = TBL365_FAST_SOURCE_GATE;
                if (!g_tbl352_ultrafast_prepared && !g_tbl354_layout_cached)
                    (void)Tbl352_PrepareUltrafastLayout();
                (void)Tbl34_PrepareFastTimingHardware(
                    d, g_tbl34_gate_def.channel, pfast, g_tbl34_gate_def.phase_half_ticks,
                    (g_tbl34_gate_def.flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U);
            }
        }

        if ((ev0->action & TBL34_ACTION_GATE_ARM) != 0U
            && g_tbl365_fast_def.valid && g_tbl365_fast_def.dds == d) {
            uint32_t pfast = Tbl365_AutoFastEvents()[1].at_ticks - Tbl365_AutoFastEvents()[0].at_ticks;
            g_tbl365_fast_source = TBL365_FAST_SOURCE_AUTO;
            g_tbl365_auto_fast_done = false;
            g_tbl365_auto_fast_event0_prepared = false;
            g_tbl356_auto_event0_committed = true;
            g_tbl354_gate_arm_commit_tick = TBL34_DWT_CYCCNT;
            if (!g_tbl352_ultrafast_prepared && !g_tbl354_layout_cached)
                (void)Tbl352_PrepareUltrafastLayout();
            (void)Tbl34_PrepareFastTimingHardware(
                d, g_tbl365_fast_def.channel, pfast, g_tbl365_fast_def.phase_half_ticks,
                (g_tbl365_fast_def.flags & TBL34_GATE_FLAG_PHASE_TRIANGLE) != 0U);
            g_tbl35_dbg_auto_fast_start_count++;
            if (!Tbl34_StartFastGate()) {
                g_tbl34_dbg_sequence_aborted++;
                Tbl34_ResetRuntimeOnly();
                return false;
            }
        }
        if ((ev0->action & TBL34_ACTION_STREAM_END) != 0U) {
            Tbl34_MarkMainDone(d);
        } else {
            g_tbl34_rt[d].idx = 1U;
            if (g_tbl365_auto_fast_running && g_tbl365_fast_def.dds == d
                && g_tbl34_rt[d].idx < g_tbl34_def[d].n_events) {
                g_tbl365_main_resume_mask |= (uint8_t)(1U << d);
            } else if (!Tbl362_ScheduleNextMain(d,ev0)) {
                g_tbl34_dbg_preload_error[d]++;
                Tbl34_MarkMainDone(d);
            }
        }
    }
    return true;
}

/* =============================================================================
 * INITIALISATION ET CHANGEMENT DE MODE
 * =============================================================================
 * Ces fonctions préparent les buffers RAM, les timers et l'état du moteur.
 * Elles ne jouent pas encore de séquence.
 */
void Tbl34_Init(void)
{
    unsigned int d;

    g_tbl362_main_preload_pending_mask = 0U;

    /* V35.5: GPTMSYNC lives in GPTM0.  main.c initializes Timer1..7 but not
     * Timer0, so the triangle path must clock GPTM0 before touching SYNC. */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) { }
    HWREG(TIMER0_BASE + TIMER_O_SYNC) = 0UL;

    TBL34_DEMCR |= 0x01000000UL;
    TBL34_DWT_CYCCNT = 0UL;
    TBL34_DWT_CTRL |= 1UL;
    for (d = 0U; d < 3U; d++) {
        g_tbl34_def[d].valid = false;
        g_tbl34_def[d].loading = false;
        g_tbl34_rt[d].active = false;
        g_tbl34_rt[d].event0_prepared = false;
        Tbl35_InvalidateResetPrepared(d);
        g_tbl34_group[d].running = false;
    }
    g_tbl34_gate_def.valid = false;
    g_tbl34_gate_def.loading = false;
    g_tbl365_fast_def.valid = false;
    g_tbl365_fast_def.loading = false;
    g_tbl34_armed = false;
    g_tbl34_arm_pending = false;
}

void Tbl34_EnterMode(void)
{
    unsigned int d;
    for (d = 0U; d < 3U; d++) Tbl34_ConfigOneShot(d);
    TimerIntDisable(TIMER7_BASE,TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH);
    HWREG(TIMER7_BASE + TIMER_O_CTL) &= ~TIMER_CTL_TAEN;
    HWREG(TIMER7_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
    IntPendClear(INT_TIMER7A);
    IntEnable(INT_TIMER7A);
    IntPrioritySet(INT_TIMER4A,0x00);
    IntPrioritySet(INT_TIMER5A,0x00);
    IntPrioritySet(INT_TIMER6A,0x00);
    /* V36.3 worker: lower priority than DDS deadline timers (0x00) AND
     * GPIOL trigger/gate handling (0x40), but above UART housekeeping (0xC0).
     * This prevents a long SSI preload from adding latency to a secondary gate. */
    IntPrioritySet(INT_TIMER7A,0x60);
}

void Tbl34_StopAll(void)
{
    Tbl34_ResetRuntimeOnly();
    g_tbl34_armed = false;
    g_tbl34_arm_pending = false;
    for (unsigned int d = 0U; d < 3U; d++) {
        g_tbl34_rt[d].event0_prepared = false;
        Tbl35_InvalidateResetPrepared(d);
    }
    }

void Tbl34_ResetConfiguration(void)
{
    unsigned int d;
    Tbl34_StopAll();
    for (d = 0U; d < 3U; d++) {
        g_tbl34_def[d].valid = false;
        g_tbl34_def[d].loading = false;
        g_tbl34_def[d].n_events = 0U;
    }
    g_tbl34_gate_def.valid = false;
    g_tbl34_gate_def.loading = false;
    g_tbl34_gate_def.n_events = 0U;
    g_tbl34_gate_done = true;
    g_tbl365_fast_def.valid = false;
    g_tbl365_fast_def.loading = false;
    g_tbl365_fast_def.n_events = 0U;
    g_tbl365_fast_event_base = 0U;
    g_tbl365_fast_blob_base = 0U;
    g_tbl365_auto_fast_done = true;
    g_tbl365_auto_fast_running = false;
}

bool Tbl34_IsArmed(void) { return g_tbl34_armed; }
bool Tbl34_ArmAckPending(void) { return g_tbl34_arm_pending; }

bool Tbl34_AnyActive(void)
{
    unsigned int d;
    if (g_tbl34_fast_active || g_tbl34_gate_running || g_tbl365_auto_fast_running) return true;
    for (d = 0U; d < 3U; d++) if (g_tbl34_rt[d].active) return true;
    return false;
}

/* =============================================================================
 * ARMEMENT
 * =============================================================================
 * L'ARM valide la configuration reçue et prépare l'événement 0 de chaque DDS.
 * L'ACK n'est envoyé qu'une fois ce préchargement terminé.
 */
bool Tbl34_ServiceArm(void)
{
    uint32_t t0;
    unsigned int d;
    bool any = false;
    bool ok = true;
    if (!g_tbl34_arm_pending) return false;
    t0 = TBL34_DWT_CYCCNT;
    Tbl34_ResetRuntimeOnly();
    Tbl34_EnterMode();
    for (d = 0U; d < 3U; d++) {
        g_tbl34_rt[d].event0_prepared = false;
                if (g_tbl34_def[d].valid) {
            any = true;
            if (!Tbl34_PreloadMainEvent0(d)) ok = false;
        }
    }
    if ((g_tbl365_fast_def.valid || (g_tbl34_gate_def.valid && Tbl351_GateUsesFastTiming()))
        && !Tbl354_PrepareFastGateAtArm()) ok = false;
    if (!any) ok = false;
    g_tbl34_dbg_arm_preload_last_ticks = TBL34_DWT_CYCCNT - t0;
    g_tbl34_arm_pending = false;
    g_tbl34_armed = ok;
    if (ok) g_tbl34_dbg_arm_ack_count++;
    else g_tbl34_dbg_arm_nak_count++;
    return ok;
}

void Tbl34_Service(void)
{
    unsigned long base;

    /* V36.5: les preloads ordinaires sont pris en charge par le mécanisme
     * d'early-preload V36.4. Le foreground ne garde ici que le petit travail
     * historique de restauration PROFILE via Timer2/Timer3. */
    base = g_tbl34_profile_restore_pending_base;
    if (base == 0UL) return;
    g_tbl34_profile_restore_pending_base = 0UL;
    if (base == TIMER2_BASE) initTimer2();
    else if (base == TIMER3_BASE) initTimer3();
}

/* =============================================================================
 * RECEPTION DES TABLES DEPUIS LE PC
 * =============================================================================
 * Chaque trame est contrôlée avant d'être copiée dans les arènes statiques.
 * Les CRC protègent les événements et les blobs contre une trame incomplète.
 */
bool Tbl34_ParseFrame(uint8_t ftype, const uint8_t *p, unsigned int len)
{
    if (ftype == TBL34_FRAME_BEGIN) {
        unsigned int d;
        tbl34_def_t *def;
        if (len != 18U) return false;
        d = p[0];
        if (d >= 3U || g_tbl34_rt[d].active) return false;
        def = &g_tbl34_def[d];
        def->valid = false;
        def->loading = false;
        def->start_ttl_index = p[1];
        def->n_events = Tbl34_Rd16(&p[2]);
        def->loop_ticks = Tbl34_Rd32(&p[4]);
        def->blob_len = Tbl34_Rd16(&p[8]);
        def->crc_expected = Tbl34_Rd32(&p[10]);
        def->event_crc_expected = Tbl34_Rd32(&p[14]);
        def->ev_recv = 0U;
        def->blob_recv = 0U;
        g_tbl34_armed = false;
        g_tbl34_rt[d].event0_prepared = false;
        Tbl35_InvalidateResetPrepared(d);
        if (def->n_events == 0U) return true; /* explicit invalidation */
        if (def->start_ttl_index >= 3U || def->n_events > TBL34_MAX_EVENTS
            || def->blob_len > TBL34_ARENA_BYTES) return false;
        Tbl34_MarkEventSlotsUnseen(g_tbl34_ev[d],def->n_events);
        def->loading = true;
        return true;
    }

    if (ftype == TBL34_FRAME_EVENTS) {
        unsigned int d;
        uint16_t first, count, i;
        if (len < 5U) return false;
        d = p[0]; first = Tbl34_Rd16(&p[1]); count = Tbl34_Rd16(&p[3]);
        if (d >= 3U || !g_tbl34_def[d].loading || 5U + (unsigned int)count * 12U != len) return false;
        if ((unsigned int)first + count > g_tbl34_def[d].n_events) return false;
        for (i = 0U; i < count; i++) {
            unsigned int idx = (unsigned int)first + i;
            tbl34_event_t parsed;
            bool first_time = (g_tbl34_ev[d][idx].blob_len == TBL34_EVENT_UNSEEN_BLOB_LEN);
            Tbl34_ParseWireEvent(&parsed,&p[5U + (unsigned int)i * 12U]);
            if (parsed.blob_len > TBL34_MAX_BLOB_PER_EVENT
                || (unsigned int)parsed.blob_offset + (unsigned int)parsed.blob_len > g_tbl34_def[d].blob_len)
                return false;
            g_tbl34_ev[d][idx] = parsed;
            if (first_time)
                g_tbl34_def[d].ev_recv++;
            else
                g_tbl34_dbg_event_duplicate_count[d]++;
        }
        return true;
    }

    if (ftype == TBL34_FRAME_BLOB) {
        unsigned int d, n, i;
        uint16_t off;
        if (len < 3U) return false;
        d = p[0]; off = Tbl34_Rd16(&p[1]); n = len - 3U;
        if (d >= 3U || !g_tbl34_def[d].loading || (unsigned int)off + n > g_tbl34_def[d].blob_len) return false;
        for (i = 0U; i < n; i++) g_tbl34_blob[d][off+i] = p[3U+i];
        g_tbl34_def[d].blob_recv = (uint16_t)(g_tbl34_def[d].blob_recv + n);
        return true;
    }

    if (ftype == TBL34_FRAME_END) {
        unsigned int d;
        tbl34_def_t *def;
        if (len != 1U) return false;
        d = p[0]; if (d >= 3U) return false;
        def = &g_tbl34_def[d];
        if (!def->loading) return false;
        def->loading = false;
        if (def->ev_recv != def->n_events || def->blob_recv != def->blob_len
            || Tbl34_Crc32(g_tbl34_blob[d],def->blob_len) != def->crc_expected
            || Tbl34_Crc32EventsCanonical(g_tbl34_ev[d],def->n_events) != def->event_crc_expected
            || !Tbl34_ValidateEvents(g_tbl34_ev[d],def->n_events,def->blob_len)) {
            if (Tbl34_Crc32EventsCanonical(g_tbl34_ev[d],def->n_events) != def->event_crc_expected)
                g_tbl34_dbg_event_crc_error[d]++;
            return false;
        }
        def->valid = true;
        return true;
    }

    if (ftype == TBL34_FRAME_GATE_BEGIN) {
        Tbl354_InvalidateFastCache();
        if (len == 0U) {
            g_tbl34_gate_def.valid = false;
            g_tbl34_gate_def.loading = false;
            g_tbl34_gate_def.n_events = 0U;
            g_tbl34_gate_done = true;
            g_tbl34_gate_armed = false;
                        return true;
        }
        if (len != 25U || Tbl34_AnyActive()) return false;
        g_tbl34_gate_def.valid = false;
        g_tbl34_gate_def.loading = true;
        g_tbl34_gate_def.dds = p[0];
        g_tbl34_gate_def.channel = p[1];
        g_tbl34_gate_def.start_ttl_index = p[2];
        g_tbl34_gate_def.gate_ttl_index = p[3];
        g_tbl34_gate_def.n_events = Tbl34_Rd16(&p[4]);
        g_tbl34_gate_def.duration_ticks = Tbl34_Rd32(&p[6]);
        g_tbl34_gate_def.blob_len = Tbl34_Rd16(&p[10]);
        g_tbl34_gate_def.flags = p[12];
        g_tbl34_gate_def.phase_half_ticks = Tbl34_Rd32(&p[13]);
        g_tbl34_gate_def.crc_expected = Tbl34_Rd32(&p[17]);
        g_tbl34_gate_def.event_crc_expected = Tbl34_Rd32(&p[21]);
        g_tbl34_gate_def.ev_recv = 0U;
        g_tbl34_gate_def.blob_recv = 0U;
        g_tbl34_armed = false;
        if (g_tbl34_gate_def.dds >= 3U || g_tbl34_gate_def.channel >= 4U
            || g_tbl34_gate_def.start_ttl_index >= 3U
            || g_tbl34_gate_def.n_events == 0U || g_tbl34_gate_def.n_events > TBL34_MAX_EVENTS
            || g_tbl34_gate_def.blob_len > TBL34_ARENA_BYTES
            || (g_tbl34_gate_def.flags & ~(TBL34_GATE_FLAG_FAST_STAIRCASE | TBL34_GATE_FLAG_PHASE_TRIANGLE | TBL34_GATE_FLAG_NATIVE_LINEAR | TBL34_GATE_FLAG_AUTO_START | TBL35_GATE_FLAG_FAST_EVENT_STREAM | TBL35_GATE_FLAG_FAST_CPOW | TBL35_GATE_FLAG_FAST_ACR | TBL3512_GATE_FLAG_SUPPRESS_FAST_IOUPDATE)) != 0U
            || ((g_tbl34_gate_def.flags & TBL34_GATE_FLAG_AUTO_START) != 0U)
            || (((g_tbl34_gate_def.flags & TBL34_GATE_FLAG_AUTO_START) != 0U)
                && (((g_tbl34_gate_def.flags & (TBL34_GATE_FLAG_FAST_STAIRCASE | TBL35_GATE_FLAG_FAST_EVENT_STREAM)) == 0U)
                    || g_tbl34_gate_def.gate_ttl_index != 0xFFU))
            || (((g_tbl34_gate_def.flags & (TBL35_GATE_FLAG_FAST_CPOW | TBL35_GATE_FLAG_FAST_ACR)) != 0U)
                && ((g_tbl34_gate_def.flags & TBL35_GATE_FLAG_FAST_EVENT_STREAM) == 0U))
            || (((g_tbl34_gate_def.flags & TBL34_GATE_FLAG_AUTO_START) == 0U)
                && (g_tbl34_gate_def.gate_ttl_index >= 3U
                    || g_tbl34_gate_def.gate_ttl_index == g_tbl34_gate_def.start_ttl_index))) {
            g_tbl34_gate_def.loading = false; return false;
        }
        Tbl34_MarkEventSlotsUnseen(g_tbl34_gate_ev,g_tbl34_gate_def.n_events);
        return true;
    }

    if (ftype == TBL34_FRAME_GATE_EVENTS) {
        uint16_t first, count, i;
        if (len < 4U || !g_tbl34_gate_def.loading) return false;
        first = Tbl34_Rd16(&p[0]); count = Tbl34_Rd16(&p[2]);
        if (4U + (unsigned int)count * 12U != len || (unsigned int)first + count > g_tbl34_gate_def.n_events) return false;
        for (i = 0U; i < count; i++) {
            unsigned int idx = (unsigned int)first + i;
            tbl34_event_t parsed;
            bool first_time = (g_tbl34_gate_ev[idx].blob_len == TBL34_EVENT_UNSEEN_BLOB_LEN);
            Tbl34_ParseWireEvent(&parsed,&p[4U + (unsigned int)i * 12U]);
            if (parsed.blob_len > TBL34_MAX_BLOB_PER_EVENT
                || (unsigned int)parsed.blob_offset + (unsigned int)parsed.blob_len > g_tbl34_gate_def.blob_len)
                return false;
            g_tbl34_gate_ev[idx] = parsed;
            if (first_time)
                g_tbl34_gate_def.ev_recv++;
            else
                g_tbl34_dbg_gate_event_duplicate_count++;
        }
        return true;
    }

    if (ftype == TBL34_FRAME_GATE_BLOB) {
        uint16_t off;
        unsigned int n, i;
        if (len < 2U || !g_tbl34_gate_def.loading) return false;
        off = Tbl34_Rd16(&p[0]); n = len - 2U;
        if ((unsigned int)off + n > g_tbl34_gate_def.blob_len) return false;
        for (i = 0U; i < n; i++) g_tbl34_gate_blob[off+i] = p[2U+i];
        g_tbl34_gate_def.blob_recv = (uint16_t)(g_tbl34_gate_def.blob_recv + n);
        return true;
    }

    if (ftype == TBL34_FRAME_GATE_END) {
        if (len != 0U || !g_tbl34_gate_def.loading) return false;
        g_tbl34_gate_def.loading = false;
        if (g_tbl34_gate_def.ev_recv != g_tbl34_gate_def.n_events
            || g_tbl34_gate_def.blob_recv != g_tbl34_gate_def.blob_len
            || Tbl34_Crc32(g_tbl34_gate_blob,g_tbl34_gate_def.blob_len) != g_tbl34_gate_def.crc_expected
            || Tbl34_Crc32EventsCanonical(g_tbl34_gate_ev,g_tbl34_gate_def.n_events) != g_tbl34_gate_def.event_crc_expected
            || !Tbl34_ValidateEvents(g_tbl34_gate_ev,g_tbl34_gate_def.n_events,g_tbl34_gate_def.blob_len)) {
            if (Tbl34_Crc32EventsCanonical(g_tbl34_gate_ev,g_tbl34_gate_def.n_events) != g_tbl34_gate_def.event_crc_expected)
                g_tbl34_dbg_gate_event_crc_error++;
            return false;
        }
        g_tbl34_gate_def.valid = true;
        return true;
    }


    if (ftype == TBL365_FRAME_FAST_BEGIN) {
        Tbl354_InvalidateFastCache();
        if (len == 0U) {
            g_tbl365_fast_def.valid = false;
            g_tbl365_fast_def.loading = false;
            g_tbl365_fast_def.n_events = 0U;
            g_tbl365_fast_event_base = 0U;
            g_tbl365_fast_blob_base = 0U;
            g_tbl365_auto_fast_done = true;
            g_tbl365_auto_fast_running = false;
            return true;
        }
        if (len != 25U || Tbl34_AnyActive()) return false;
        g_tbl365_fast_def.valid = false;
        g_tbl365_fast_def.loading = true;
        g_tbl365_fast_def.dds = p[0];
        g_tbl365_fast_def.channel = p[1];
        g_tbl365_fast_def.start_ttl_index = p[2];
        g_tbl365_fast_def.gate_ttl_index = p[3];
        g_tbl365_fast_def.n_events = Tbl34_Rd16(&p[4]);
        g_tbl365_fast_def.duration_ticks = Tbl34_Rd32(&p[6]);
        g_tbl365_fast_def.blob_len = Tbl34_Rd16(&p[10]);
        g_tbl365_fast_def.flags = p[12];
        g_tbl365_fast_def.phase_half_ticks = Tbl34_Rd32(&p[13]);
        g_tbl365_fast_def.crc_expected = Tbl34_Rd32(&p[17]);
        g_tbl365_fast_def.event_crc_expected = Tbl34_Rd32(&p[21]);
        g_tbl365_fast_def.ev_recv = 0U;
        g_tbl365_fast_def.blob_recv = 0U;
        /* Pack AUTO_START FAST immediately after the true gate in the shared
         * special-stream pool. serialize_bundle() always sends GATE before FAST. */
        g_tbl365_fast_event_base = g_tbl34_gate_def.valid ? g_tbl34_gate_def.n_events : 0U;
        g_tbl365_fast_blob_base = g_tbl34_gate_def.valid ? g_tbl34_gate_def.blob_len : 0U;
        g_tbl34_armed = false;
        if (g_tbl365_fast_def.dds >= 3U || g_tbl365_fast_def.channel >= 4U
            || g_tbl365_fast_def.start_ttl_index >= 3U
            || g_tbl365_fast_def.gate_ttl_index != 0xFFU
            || g_tbl365_fast_def.n_events == 0U || g_tbl365_fast_def.n_events > TBL34_MAX_EVENTS
            || g_tbl365_fast_def.blob_len > TBL34_ARENA_BYTES
            || (uint32_t)g_tbl365_fast_event_base + (uint32_t)g_tbl365_fast_def.n_events > TBL34_MAX_EVENTS
            || (uint32_t)g_tbl365_fast_blob_base + (uint32_t)g_tbl365_fast_def.blob_len > TBL34_ARENA_BYTES
            || (g_tbl365_fast_def.flags & TBL34_GATE_FLAG_AUTO_START) == 0U
            || (g_tbl365_fast_def.flags & (TBL34_GATE_FLAG_FAST_STAIRCASE | TBL35_GATE_FLAG_FAST_EVENT_STREAM)) == 0U
            || (g_tbl365_fast_def.flags & ~(TBL34_GATE_FLAG_FAST_STAIRCASE | TBL34_GATE_FLAG_PHASE_TRIANGLE | TBL34_GATE_FLAG_AUTO_START | TBL35_GATE_FLAG_FAST_EVENT_STREAM | TBL35_GATE_FLAG_FAST_CPOW | TBL35_GATE_FLAG_FAST_ACR | TBL3512_GATE_FLAG_SUPPRESS_FAST_IOUPDATE)) != 0U
            || (g_tbl34_gate_def.valid && g_tbl34_gate_def.dds == g_tbl365_fast_def.dds)
            || (g_tbl34_gate_def.valid
                && (g_tbl34_gate_def.flags & (TBL34_GATE_FLAG_FAST_STAIRCASE | TBL35_GATE_FLAG_FAST_EVENT_STREAM)) != 0U)) {
            g_tbl365_fast_def.loading = false;
            return false;
        }
        Tbl34_MarkEventSlotsUnseen(Tbl365_AutoFastEvents(),g_tbl365_fast_def.n_events);
        return true;
    }

    if (ftype == TBL365_FRAME_FAST_EVENTS) {
        uint16_t first, count, i;
        if (len < 4U || !g_tbl365_fast_def.loading) return false;
        first = Tbl34_Rd16(&p[0]); count = Tbl34_Rd16(&p[2]);
        if (4U + (unsigned int)count * 12U != len || (unsigned int)first + count > g_tbl365_fast_def.n_events) return false;
        for (i = 0U; i < count; i++) {
            unsigned int idx2 = (unsigned int)first + i;
            tbl34_event_t parsed;
            bool first_time = (Tbl365_AutoFastEvents()[idx2].blob_len == TBL34_EVENT_UNSEEN_BLOB_LEN);
            Tbl34_ParseWireEvent(&parsed,&p[4U + (unsigned int)i * 12U]);
            if (parsed.blob_len > TBL34_MAX_BLOB_PER_EVENT
                || (unsigned int)parsed.blob_offset + (unsigned int)parsed.blob_len > g_tbl365_fast_def.blob_len)
                return false;
            Tbl365_AutoFastEvents()[idx2] = parsed;
            if (first_time) g_tbl365_fast_def.ev_recv++;
            else g_tbl34_dbg_gate_event_duplicate_count++;
        }
        return true;
    }

    if (ftype == TBL365_FRAME_FAST_BLOB) {
        uint16_t off;
        unsigned int n, i;
        if (len < 2U || !g_tbl365_fast_def.loading) return false;
        off = Tbl34_Rd16(&p[0]); n = len - 2U;
        if ((unsigned int)off + n > g_tbl365_fast_def.blob_len) return false;
        for (i = 0U; i < n; i++) Tbl365_AutoFastBlob()[off+i] = p[2U+i];
        g_tbl365_fast_def.blob_recv = (uint16_t)(g_tbl365_fast_def.blob_recv + n);
        return true;
    }

    if (ftype == TBL365_FRAME_FAST_END) {
        if (len != 0U || !g_tbl365_fast_def.loading) return false;
        g_tbl365_fast_def.loading = false;
        if (g_tbl365_fast_def.ev_recv != g_tbl365_fast_def.n_events
            || g_tbl365_fast_def.blob_recv != g_tbl365_fast_def.blob_len
            || Tbl34_Crc32(Tbl365_AutoFastBlob(),g_tbl365_fast_def.blob_len) != g_tbl365_fast_def.crc_expected
            || Tbl34_Crc32EventsCanonical(Tbl365_AutoFastEvents(),g_tbl365_fast_def.n_events) != g_tbl365_fast_def.event_crc_expected
            || !Tbl34_ValidateEvents(Tbl365_AutoFastEvents(),g_tbl365_fast_def.n_events,g_tbl365_fast_def.blob_len)) {
            if (Tbl34_Crc32EventsCanonical(Tbl365_AutoFastEvents(),g_tbl365_fast_def.n_events) != g_tbl365_fast_def.event_crc_expected)
                g_tbl34_dbg_gate_event_crc_error++;
            return false;
        }
        g_tbl365_fast_def.valid = true;
        return true;
    }

    if (ftype == TBL34_FRAME_ARM) {
        if (len != 4U || Tbl34_AnyActive()) return false;
        if (p[0] > 2U || p[1] > 2U || p[2] > 2U || p[3] > 2U
            || p[1] == p[2] || p[1] == p[3] || p[2] == p[3]) return false;
        g_tbl34_policy = p[0];
        g_tbl34_priority[0] = p[1]; g_tbl34_priority[1] = p[2]; g_tbl34_priority[2] = p[3];
        g_tbl34_armed = false;
        g_tbl34_arm_pending = true;
        g_tbl34_dbg_arm_request_count++;
        return true;
    }

    return false;
}

/* =============================================================================
 * DECLENCHEMENT TTL
 * =============================================================================
 * Le front TTL choisit les DDS armés à démarrer. Tous les DDS d'un même groupe
 * reçoivent la même origine temporelle absolue.
 */
void Tbl34_HandleTriggeredMask(unsigned int trigger_flags)
{
    unsigned int ttl;
    for (ttl = 0U; ttl < 3U; ttl++) if ((trigger_flags & (1U << ttl)) != 0U) g_tbl34_dbg_ttl_seen[ttl]++;

    /* A secondary edge is consumed only by an already-armed gate. */
    if (g_tbl34_gate_def.valid
        && g_tbl34_gate_def.gate_ttl_index < 3U
        && (trigger_flags & (1U << g_tbl34_gate_def.gate_ttl_index)) != 0U) {
        (void)Tbl34_ReleaseGate(g_tbl34_gate_def.gate_ttl_index);
        trigger_flags &= ~(1U << g_tbl34_gate_def.gate_ttl_index);
    }

    /* If several main TTLs arrive in the same ISR, evaluate in configured
     * priority order so HIGHER_PRIORITY is deterministic. */
    for (ttl = 0U; ttl < 3U; ttl++) {
        uint8_t t = g_tbl34_priority[ttl];
        if ((trigger_flags & (1U << t)) != 0U) (void)Tbl34_StartGroup(t);
    }
}

static void Tbl34_TimerHandler(unsigned int d)
{
    uint32_t t0 = TBL34_DWT_CYCCNT;
    uint8_t bit = (uint8_t)(1U << d);
    if (g_tbl34_fast_active && d == g_tbl34_fast_dds) {
        Tbl34_FastTimerIntHandler(d);
        return;
    }
    HWREG(Tbl34_TimerBase(d) + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT | TIMER_TIMA_MATCH;
    if (g_tbl365_auto_fast_running && g_tbl365_fast_def.dds == d) {
        Tbl365_ProcessAutoFastEvent();
    } else if (g_tbl34_gate_running && g_tbl34_gate_def.dds == d) {
        Tbl34_ProcessGateEvent();
    } else if ((g_tbl362_main_preload_pending_mask & bit) != 0U) {
        /* V36.4 long-gap path: this timeout is an EARLY preload wake. */
        Tbl364_ProcessDeferredDeadlineGroup(d);
    } else {
        Tbl34_ProcessMainEvent(d);
    }
    g_tbl34_dbg_isr_ticks[d] = TBL34_DWT_CYCCNT - t0;
    if (g_tbl34_dbg_isr_ticks[d] > g_tbl34_dbg_isr_max_ticks[d])
        g_tbl34_dbg_isr_max_ticks[d] = g_tbl34_dbg_isr_ticks[d];
}

/* =============================================================================
 * INTERRUPTIONS DE DEADLINE DDS1 / DDS2 / DDS3
 * =============================================================================
 * Les trois handlers appellent le même coeur temps réel avec l'indice du DDS.
 */
void Tbl34_Timer_DDS0_IntHandler(void)
{
    if ((g_tbl353_direct_active || g_tbl356_triangle_exit_pending)
        && g_tbl34_fast_dds == 0U) {
        Tbl353_DirectPeriodicIRQ(0U,TIMER4_BASE); return;
    }
    Tbl34_TimerHandler(0U);
}
void Tbl34_Timer_DDS1_IntHandler(void)
{
    if ((g_tbl353_direct_active || g_tbl356_triangle_exit_pending)
        && g_tbl34_fast_dds == 1U) {
        Tbl353_DirectPeriodicIRQ(1U,TIMER5_BASE); return;
    }
    Tbl34_TimerHandler(1U);
}
void Tbl34_Timer_DDS2_IntHandler(void)
{
    if ((g_tbl353_direct_active || g_tbl356_triangle_exit_pending)
        && g_tbl34_fast_dds == 2U) {
        Tbl353_DirectPeriodicIRQ(2U,TIMER6_BASE); return;
    }
    Tbl34_TimerHandler(2U);
}
