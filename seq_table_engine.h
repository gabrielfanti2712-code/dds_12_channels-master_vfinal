/*
 * API publique du moteur de tables DDS.
 *
 * Ce header décrit uniquement ce que ``main_ARM_TM4C1294.c`` et le debugger
 * CCS doivent connaître : format des événements, constantes de timing,
 * fonctions publiques et compteurs de diagnostic.
 *
 * Le format UART est stable : les numéros historiques TBL34/TBL35 dans les
 * noms des constantes et variables sont conservés pour ne pas casser le
 * protocole ni les Expressions CCS existantes.
 */
#ifndef SEQ_TABLE_ENGINE_H_
#define SEQ_TABLE_ENGINE_H_

#include <stdbool.h>
#include <stdint.h>

#define TBL34_DDS_COUNT              3U
#define TBL34_CHANNELS_PER_DDS       4U
#define TBL34_MAX_EVENTS             1024U
#define TBL34_ARENA_BYTES            (8U * 1024U)
#define TBL34_MAX_BLOB_PER_EVENT     200U

/* Capacités du moteur. Les optimisations SRAM sont internes au .c et ne
 * changent ni le format UART, ni la taille d'un événement, ni cette API. */

/* V34 frame types, carried inside the existing A5 5A / type / len16 / XOR
 * serial framing used by V33. */
#define TBL34_FRAME_BEGIN            0x20U
#define TBL34_FRAME_EVENTS           0x21U
#define TBL34_FRAME_BLOB             0x22U
#define TBL34_FRAME_END              0x23U
#define TBL34_FRAME_GATE_BEGIN       0x24U
#define TBL34_FRAME_GATE_EVENTS      0x25U
#define TBL34_FRAME_GATE_BLOB        0x26U
#define TBL34_FRAME_GATE_END         0x27U
#define TBL34_FRAME_ARM              0x28U
/* V36.5: AUTO_START periodic stream is transported independently from the true secondary-TTL gate. */
#define TBL365_FRAME_FAST_BEGIN       0x29U
#define TBL365_FRAME_FAST_EVENTS      0x2AU
#define TBL365_FRAME_FAST_BLOB        0x2BU
#define TBL365_FRAME_FAST_END         0x2CU

#define TBL34_ACTION_IO_UPDATE       0x01U
#define TBL34_ACTION_GATE_ARM        0x02U
#define TBL34_ACTION_STREAM_END      0x04U
#define TBL34_ACTION_FAST_CHAIN      0x08U
#define TBL34_ACTION_PHASE_PULSE     0x10U
#define TBL34_ACTION_SWEEP_EXIT_COMMIT 0x20U
#define TBL34_ACTION_SWEEP_EXIT      TBL34_ACTION_SWEEP_EXIT_COMMIT /* compatibility */
#define TBL34_ACTION_SWEEP_START     0x40U
#define TBL34_ACTION_SWEEP_EXIT_PREPARE 0x80U

#define TBL34_GATE_FLAG_FAST_STAIRCASE 0x01U
#define TBL34_GATE_FLAG_PHASE_TRIANGLE 0x02U
#define TBL34_GATE_FLAG_NATIVE_LINEAR  0x04U
#define TBL34_GATE_FLAG_AUTO_START     0x08U
/* V35.1: generic periodic fast streams use the same Timer4/5/6 runtime as
 * FTW staircases.  Subtype flags are diagnostic/validation metadata only. */
#define TBL35_GATE_FLAG_FAST_EVENT_STREAM 0x10U
#define TBL35_GATE_FLAG_FAST_CPOW         0x20U
#define TBL35_GATE_FLAG_FAST_ACR          0x40U
/* V35.12 L0 only: same PROFILE/timer/SSI activity, but no periodic I/O_UPDATE. */
#define TBL3512_GATE_FLAG_SUPPRESS_FAST_IOUPDATE 0x80U

/* V35 event aux flags.  The action byte is full; aux preserves the 12-byte
 * event format while making sweep kind and reset semantics explicit. */
#define TBL35_AUX_RESET_PREPARE        0x0001U
#define TBL35_AUX_RESET_COMMIT         0x0002U
#define TBL35_AUX_NATIVE_FREQ          0x0010U
#define TBL35_AUX_NATIVE_PHASE         0x0020U
#define TBL35_AUX_NATIVE_AMP           0x0040U
#define TBL351_AUX_NATIVE_DESCENDING     0x0080U

#define TBL34_TIMER_HZ               120000000UL
#define TBL34_SSI_HZ                 60000000UL
#define TBL34_WAKE_LEAD_TICKS        60UL   /* 0.5 us: wake, then DWT-spin. */
#define TBL34_FAST_WAKE_LEAD_TICKS   60UL
#define TBL34_FAST_MIN_TICKS         240UL  /* 2.0 us. */
#define TBL34_SWEEP_SETTLE_TICKS     24UL   /* 0.2 us between UPDATE and PROFILE high. */
#define TBL34_SWEEP_EXIT_PREP_LEAD_TICKS 2400UL /* legacy; V34.5 does not physically preload sweep exit. */
#define TBL34_SWEEP_EXIT_SETTLE_TICKS    60UL   /* legacy, retained for ABI/source compatibility */
#define TBL34_SWEEP_EXIT_PROFILE_LOW_HOLD_TICKS 720UL /* legacy V34.4 diagnostic constant */
#define TBL34_SWEEP_EXIT_HANDOFF_GUARD_TICKS    60UL  /* legacy V34.5 */
#define TBL34_SWEEP_EXIT_PROFILE_LOW_SETTLE_TICKS 240UL /* V34.6: 2.0 us */
#define TBL34_SWEEP_EXIT_MODE_SETTLE_TICKS        240UL /* V34.6: 2.0 us */
#define TBL34_SWEEP_EXIT_UPDATE_HIGH_TICKS         120UL /* recovery UPDATE high = 1.0 us */
#define TBL34_SWEEP_EXIT_CLEAR_SETTLE_TICKS         600UL /* V34.7: 5 us with sweep accumulator held at zero */
#define TBL34_SWEEP_EXIT_DISABLE_SETTLE_TICKS       600UL /* V34.7 legacy */
#define TBL34_SWEEP_EXIT_STRICT_PROFILE_SETTLE_TICKS 600UL /* V34.8: 5 us after PROFILE LOW */
#define TBL34_SWEEP_EXIT_STRICT_INTERREG_TICKS        120UL /* V34.8: 1 us FSS/SSI idle guard between register frames */
#define TBL34_SWEEP_EXIT_STRICT_UPDATE_HIGH_TICKS     120UL /* legacy V34.8 */
#define TBL34_V3421_FAST_RESET_RECOVERY 1U /* chip reset, table-driven restore, table runtime continues */
#define TBL34_TIMER_MIN_TICKS        24UL

/* Wire event is 12 bytes.  Do not memcpy a C struct from UART: the parser
 * decodes fields explicitly so ABI packing cannot change the protocol. */
typedef struct {
    uint32_t at_ticks;
    uint16_t blob_offset;
    uint8_t  blob_len;
    uint8_t  action;
    uint8_t  profile_ops;
    uint8_t  reserved;
    uint16_t aux;
} tbl34_event_t;

typedef struct {
    bool     valid;
    bool     loading;
    uint8_t  start_ttl_index;
    uint16_t n_events;
    uint32_t loop_ticks;
    uint16_t blob_len;
    uint32_t crc_expected;
    uint32_t event_crc_expected;
    uint16_t ev_recv;
    uint16_t blob_recv;
} tbl34_def_t;

typedef struct {
    bool     valid;
    bool     loading;
    uint8_t  dds;
    uint8_t  channel;
    uint8_t  start_ttl_index;
    uint8_t  gate_ttl_index;
    uint16_t n_events;
    uint32_t duration_ticks;
    uint16_t blob_len;
    uint8_t  flags;
    uint32_t phase_half_ticks;
    uint32_t crc_expected;
    uint32_t event_crc_expected;
    uint16_t ev_recv;
    uint16_t blob_recv;
} tbl34_gate_def_t;

typedef struct {
    bool     active;
    bool     event0_prepared;
    uint16_t idx;
    uint32_t origin_ticks;
    uint8_t  start_ttl_index;
} tbl34_rt_t;

typedef struct {
    bool     running;
    uint8_t  active_dds_mask;
    uint8_t  done_dds_mask;
} tbl34_group_rt_t;

/* Integration API. */
void Tbl34_Init(void);
void Tbl34_EnterMode(void);
void Tbl34_StopAll(void);
void Tbl34_ResetConfiguration(void);
bool Tbl34_ParseFrame(uint8_t frame_type, const uint8_t *payload, unsigned int len);
void Tbl34_HandleTriggeredMask(unsigned int trigger_flags);
bool Tbl34_IsArmed(void);
bool Tbl34_AnyActive(void);
bool Tbl34_ArmAckPending(void);
bool Tbl34_ServiceArm(void);     /* returns true on ACK, false on NAK; call only if pending */
void Tbl34_Service(void);        /* tâches non critiques : restaurations / housekeeping */
void Tbl34_PreloadWorkerIntHandler(void); /* symbole de compatibilité Timer7 ; V36.4 précharge via les timers DDS */

void Tbl34_Timer_DDS0_IntHandler(void);
void Tbl34_Timer_DDS1_IntHandler(void);
void Tbl34_Timer_DDS2_IntHandler(void);

extern const char g_tbl34_build_id[];

/* Persistent diagnostics.  They are cumulative until power cycle. */
extern volatile uint32_t g_tbl34_dbg_ttl_seen[3];
extern volatile uint32_t g_tbl34_dbg_start_accepted[3];
extern volatile uint32_t g_tbl34_dbg_start_ignored_active[3];
extern volatile uint32_t g_tbl34_dbg_start_not_ready[3];
extern volatile uint32_t g_tbl34_dbg_gate_seen[3];
extern volatile uint32_t g_tbl34_dbg_gate_release_accepted;
extern volatile uint32_t g_tbl34_dbg_gate_too_early;
extern volatile uint32_t g_tbl34_dbg_gate_wrong_ttl[3];
extern volatile uint32_t g_tbl34_dbg_sequence_started;
extern volatile uint32_t g_tbl34_dbg_sequence_completed;
extern volatile uint32_t g_tbl34_dbg_sequence_aborted;
extern volatile uint32_t g_tbl34_dbg_commit_count[3];
extern volatile uint32_t g_tbl34_dbg_isr_ticks[3];
extern volatile uint32_t g_tbl34_dbg_isr_max_ticks[3];
extern volatile uint32_t g_tbl34_dbg_late_ticks[3];
extern volatile uint32_t g_tbl34_dbg_late_max_ticks[3];
extern volatile uint32_t g_tbl34_dbg_preload_ticks[3];
extern volatile uint32_t g_tbl34_dbg_preload_max_ticks[3];
extern volatile uint32_t g_tbl34_dbg_preload_error[3];
extern volatile uint32_t g_tbl34_dbg_fast_points_committed;
extern volatile uint32_t g_tbl34_dbg_fast_period_ticks;
extern volatile uint32_t g_tbl34_dbg_fast_isr_ticks;
extern volatile uint32_t g_tbl34_dbg_fast_isr_max_ticks;
extern volatile uint32_t g_tbl34_dbg_fast_late_max_ticks;
extern volatile uint32_t g_tbl34_dbg_profile_pwm_start_count;
extern volatile uint32_t g_tbl34_dbg_profile_pwm_stop_count;
extern volatile uint32_t g_tbl34_dbg_profile_pwm_sync_ticks;
extern volatile uint32_t g_tbl34_dbg_arm_request_count;
extern volatile uint32_t g_tbl34_dbg_arm_ack_count;
extern volatile uint32_t g_tbl34_dbg_arm_nak_count;
extern volatile uint32_t g_tbl34_dbg_arm_preload_last_ticks;
extern volatile uint32_t g_tbl34_dbg_last_gate_release_ticks;
extern volatile uint32_t g_tbl34_dbg_main_trigger_masked[3];
extern volatile uint32_t g_tbl34_dbg_sweep_start_count[3];
extern volatile uint32_t g_tbl34_dbg_sweep_exit_count[3];

/* V34.24: low-latency chip-level MASTER_RESET recovery with RAM-prebuilt restore. */
extern volatile uint32_t g_tbl34_dbg_reset_recovery_count[3];
extern volatile uint32_t g_tbl34_dbg_reset_recovery_error[3];
extern volatile uint32_t g_tbl34_dbg_reset_recovery_master_high_ticks[3];
extern volatile uint32_t g_tbl34_dbg_reset_recovery_single_init_count[3];
extern volatile uint32_t g_tbl34_dbg_reset_recovery_quad_init_count[3];
extern volatile uint32_t g_tbl34_dbg_reset_recovery_program_count[3];
extern volatile uint32_t g_tbl34_dbg_reset_recovery_update_count[3];
extern volatile uint32_t g_tbl34_dbg_reset_recovery_total_ticks[3];


/* Hardened table-load diagnostics retained in V35 MEMOPT2. */
extern volatile uint32_t g_tbl34_dbg_event_crc_error[3];
extern volatile uint32_t g_tbl34_dbg_event_duplicate_count[3];
extern volatile uint32_t g_tbl34_dbg_gate_event_crc_error;
extern volatile uint32_t g_tbl34_dbg_gate_event_duplicate_count;

/* V35 diagnostics. */
extern volatile uint32_t g_tbl35_dbg_reset_prepare_count[3];
extern volatile uint32_t g_tbl35_dbg_reset_prepare_error[3];
extern volatile uint32_t g_tbl35_dbg_reset_prepare_bytes[3];
extern volatile uint32_t g_tbl35_dbg_reset_restore_channel_mask[3];
extern volatile uint32_t g_tbl35_dbg_reset_commit_count[3];
extern volatile uint32_t g_tbl35_dbg_reset_commit_error[3];
extern volatile uint32_t g_tbl35_dbg_reset_blackout_ticks[3];
extern volatile uint32_t g_tbl35_dbg_reset_restore_bytes[3];
extern volatile uint32_t g_tbl35_dbg_native_freq_start_count[3];
extern volatile uint32_t g_tbl35_dbg_native_freq_exit_count[3];
extern volatile uint32_t g_tbl35_dbg_native_phase_start_count[3];
extern volatile uint32_t g_tbl35_dbg_native_phase_exit_count[3];
extern volatile uint32_t g_tbl35_dbg_native_amp_start_count[3];
extern volatile uint32_t g_tbl35_dbg_native_amp_exit_count[3];
extern volatile uint32_t g_tbl35_dbg_gate_arm_count;
extern volatile uint32_t g_tbl35_dbg_auto_fast_start_count;
extern volatile uint32_t g_tbl35_dbg_fast_start_attempt_count;
extern volatile uint32_t g_tbl35_dbg_fast_start_ok_count;
extern volatile uint32_t g_tbl35_dbg_fast_start_fail_code;
extern volatile uint32_t g_tbl35_dbg_fast_timer_irq_count;
extern volatile uint32_t g_tbl35_dbg_fast_update_count;
extern volatile uint32_t g_tbl35_dbg_fast_preload_count;
extern volatile uint32_t g_tbl35_dbg_fast_preload_error_count;
extern volatile uint32_t g_tbl35_dbg_fast_final_count;
extern volatile uint32_t g_tbl35_dbg_fast_to_generic_count;
extern volatile uint32_t g_tbl35_dbg_triangle_config_count;
extern volatile uint32_t g_tbl35_dbg_triangle_config_error_count;
extern volatile uint32_t g_tbl35_dbg_triangle_period_ticks;
extern volatile uint32_t g_tbl35_dbg_triangle_half_ticks;
extern volatile uint32_t g_tbl35_dbg_triangle_pwm_load;
extern volatile uint32_t g_tbl35_dbg_triangle_pwm_match;
extern volatile uint32_t g_tbl35_dbg_event_cfr_frame_count[3];
extern volatile uint32_t g_tbl35_dbg_event_ftw_frame_count[3];
extern volatile uint32_t g_tbl35_dbg_event_cpow_frame_count[3];
extern volatile uint32_t g_tbl35_dbg_event_acr_frame_count[3];

/* V35.1 diagnostics: fast main->periodic handoff, dense microstreams and
 * exact native-phase programming.  Cumulative until power cycle. */
extern volatile uint32_t g_tbl351_dbg_fast_handoff_count;
extern volatile uint32_t g_tbl351_dbg_fast_handoff_timer_kept_count;
extern volatile uint32_t g_tbl351_dbg_fast_handoff_error_count;
extern volatile uint32_t g_tbl351_dbg_fast_expected_events;
extern volatile uint32_t g_tbl351_dbg_fast_last_index;
extern volatile uint32_t g_tbl351_dbg_fast_ftw_frame_count;
extern volatile uint32_t g_tbl351_dbg_fast_cpow_frame_count;
extern volatile uint32_t g_tbl351_dbg_fast_acr_frame_count;
extern volatile uint32_t g_tbl351_dbg_native_desc_start_count[3];
extern volatile uint32_t g_tbl351_dbg_native_desc_profile_low_count[3];
extern volatile uint32_t g_tbl351_dbg_phase_last_csr[3];
extern volatile uint32_t g_tbl351_dbg_phase_last_cfr[3];
extern volatile uint32_t g_tbl351_dbg_phase_last_cpow[3];
extern volatile uint32_t g_tbl351_dbg_phase_last_cw1[3];
extern volatile uint32_t g_tbl351_dbg_phase_last_rdw[3];
extern volatile uint32_t g_tbl351_dbg_phase_last_fdw[3];
extern volatile uint32_t g_tbl351_dbg_phase_last_lsrr[3];
extern volatile uint32_t g_tbl351_dbg_phase_last_ftw[3];
extern volatile uint32_t g_tbl351_dbg_phase_last_acr[3];


/* V35.4 compact direct-periodic diagnostics. */
extern volatile uint32_t g_tbl353_dbg_layout_prepare_count;
extern volatile uint32_t g_tbl353_dbg_layout_prepare_error_count;
extern volatile uint32_t g_tbl353_dbg_direct_used_count;
extern volatile uint32_t g_tbl353_dbg_direct_stream_addr;
extern volatile uint32_t g_tbl353_dbg_direct_stride;
extern volatile uint32_t g_tbl353_dbg_direct_chain_events;
extern volatile uint32_t g_tbl353_dbg_direct_layout_error_code;
extern volatile uint32_t g_tbl353_dbg_direct_timer_tailr;
extern volatile uint32_t g_tbl353_dbg_direct_expected_irqs;
extern volatile uint32_t g_tbl353_dbg_direct_irqs_done;
extern volatile uint32_t g_tbl353_dbg_direct_expected_runtime_ticks;
extern volatile uint32_t g_tbl353_dbg_direct_runtime_ticks;
extern volatile uint32_t g_tbl353_dbg_direct_excess_ticks;
extern volatile uint32_t g_tbl353_dbg_direct_average_period_ticks;
extern volatile uint32_t g_tbl353_dbg_direct_final_count;
extern volatile uint32_t g_tbl353_dbg_direct_to_generic_count;
extern volatile uint32_t g_tbl353_dbg_direct_fallback_count;

/* V35.4 compact diagnostics. */
extern volatile uint32_t g_tbl354_dbg_start_latency_ticks;
extern volatile uint32_t g_tbl354_dbg_gate_timer_rearm_count;
extern volatile uint32_t g_tbl354_dbg_gate_timer_imr_after_rearm;
extern volatile uint32_t g_tbl354_dbg_triangle_direct_count;
extern volatile uint32_t g_tbl354_dbg_triangle_exit_count;
extern volatile uint32_t g_tbl354_dbg_triangle_exit_late_ticks;

/* V35.5 compact diagnostics. */
extern volatile uint32_t g_tbl355_dbg_triangle_sync_count;
extern volatile uint32_t g_tbl355_dbg_triangle_sync_mask;

/* V35.6 compact diagnostics. */
extern volatile uint32_t g_tbl356_dbg_auto_reuse_count;
extern volatile uint32_t g_tbl358_dbg_triangle_lsrr;
extern volatile uint32_t g_tbl356_dbg_triangle_no_ioupdate_srr_count;
extern volatile uint32_t g_tbl356_dbg_triangle_cfr;
extern volatile uint32_t g_tbl356_dbg_triangle_exit_preload_count;
extern volatile uint32_t g_tbl356_dbg_triangle_exit_preload_ticks;
extern volatile uint32_t g_tbl356_dbg_triangle_exit_irq_count;
extern volatile uint32_t g_tbl356_dbg_triangle_exit_preload_error;

/* V35.12 compact phase-delta diagnostics; SRAM-neutral repurpose of old slots. */
extern volatile uint32_t g_tbl358_dbg_triangle_rdw;
extern volatile uint32_t g_tbl358_dbg_triangle_fdw;
extern volatile uint32_t g_tbl357_dbg_triangle_exit_commit_count;

/* V36 diagnostics.  The historical V35.x counters are intentionally retained
 * so existing CCS watch expressions remain valid. */
extern volatile uint32_t g_tbl36_dbg_native_desc_high_hold_ticks;

/* V36.2 deferred-preload / crosstalk diagnostics. */
extern volatile uint32_t g_tbl362_dbg_deferred_preload_count;
extern volatile uint32_t g_tbl362_dbg_service_preload_count;
extern volatile uint32_t g_tbl362_dbg_emergency_preload_count;
extern volatile uint32_t g_tbl362_dbg_inline_short_preload_count;

/* V36.3 interrupt-worker diagnostics retained for compatibility. */

/* V36.4 early-preload synchronized-commit diagnostics. */
extern volatile uint32_t g_tbl365_signature;
extern volatile uint32_t g_tbl364_dbg_early_preload_irq_count;
extern volatile uint32_t g_tbl364_dbg_sync_group_count;
extern volatile uint32_t g_tbl364_dbg_sync_last_mask;
extern volatile uint32_t g_tbl364_dbg_sync_last_skew_ticks;
extern volatile uint32_t g_tbl364_dbg_sync_max_skew_ticks;
extern volatile uint32_t g_tbl364_dbg_preload_batch_ticks;
extern volatile uint32_t g_tbl364_dbg_preload_batch_max_ticks;
extern volatile uint32_t g_tbl364_dbg_preload_batch_late_count;

#endif /* SEQ_TABLE_ENGINE_H_ */
