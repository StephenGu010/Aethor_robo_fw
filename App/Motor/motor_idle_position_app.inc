/** @file motor_idle_position_app.inc
 * @brief Mode-2 ID7 raw position monitoring; never substitutes for standard safety feedback.
 */
#if AETHOR_IDLE_FEEDBACK_ENABLE
/** @brief Admit only idle LCD-owned reads for the fully discovered fixed display axis. */
static uint8_t aethor_app_idle_position_allowed(uint64_t now_us)
{
    const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[6];
    const JointConfig *joint = &application_motor_runtime.configuration->joints[6];
    return (uint8_t)(application_initialized && !aethor_app_debug_ui_executor_busy() &&
        !application_motor_runtime.discovery_active &&
        !debug_ui_mailbox_busy(&application_debug_ui.mailbox, true) &&
        !application_controller.enabled && !application_controller.moving &&
        application_controller.fault == ARM_FAULT_NONE &&
        !application_debug_ui.unconfirmed_disable_mask && !application_action.control_group_ready &&
        application_emergency_disable_read_index >= application_emergency_disable_batch.count &&
        application_debug_ui.fallback_frame_read_index >= application_debug_ui.fallback_frames.count &&
        application_transport_fault_read_sequence == application_transport_fault_write_sequence &&
        !application_motor_runtime.position_read.pending &&
        now_us >= application_motor_runtime.position_read.not_before_us &&
        (application_motor_runtime.discovery.verified_joint_mask & 0x40U) &&
        discovery->verified_fields_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK &&
        discovery->observed_esc_id == joint->esc_id && discovery->observed_master_id == joint->master_id &&
        discovery->observed_control_mode == 2U
#if AETHOR_ADRC_LCD_INTEGRATED
        && application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD &&
        !aethor_app_mapping_locked() && application_coordinate_pair.state != 1U &&
        (!application_integrated_diag_requested_us ||
         (now_us >= application_integrated_diag_requested_us &&
          now_us - application_integrated_diag_requested_us > ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US))
#endif
        );
}

/** @brief Cancel before owner/maintenance early returns; keep the existing late-reply quarantine. */
static void aethor_app_idle_position_preempt(uint64_t now_us)
{
    CanFrame unused_frame;
    if (application_initialized && !aethor_app_idle_position_allowed(now_us))
    {
        application_motor_runtime.idle_position_read.seen_mask = 0U;
        application_motor_runtime.idle_position_read.estimated_velocity_valid = 0U;
    }
    if (application_initialized && application_motor_runtime.idle_position_read.pending &&
        !aethor_app_idle_position_allowed(now_us))
        (void)motor_runtime_next_idle_position_read(&application_motor_runtime, 6U, now_us, 0U, &unused_frame);
}

/** @brief Append independent raw register samples only to the matching successful motor query. */
static ProtocolEngineStatus aethor_app_append_idle_position(const char *line, size_t length,
    uint64_t now_us, ProtocolEngineStatus status, ProtocolOutputBatch *output)
{
    TextProtocolRequest request;
    TextProtocolSpan motor;
    ProtocolOutputMessage *message;
    const MotorRegisterPosition *reader = &application_motor_runtime.idle_position_read;
    char extra[320];
    size_t used = 0U, original_length;
    unsigned index;
    if (status != PROTOCOL_ENGINE_STATUS_OK || !output || output->count != 1U ||
        text_protocol_parse_request(line, length, &request) != TEXT_PROTOCOL_STATUS_OK ||
        !text_protocol_request_path_equals(&request, "show", "motor") ||
        request.positional_count != 1U || !text_protocol_get_positional(&request, 0U, &motor) ||
        motor.length != 1U || motor.data[0] != '7') return status;
    for (index = 0U; index < 2U; ++index) {
        uint64_t age = now_us >= reader->sample_us[index] ? (now_us - reader->sample_us[index]) / 1000ULL : UINT64_MAX;
        uint8_t seen = (uint8_t)(aethor_app_idle_position_allowed(now_us) &&
            reader->joint_index == 6U && (reader->seen_mask & (1U << index)) &&
            isfinite(reader->values[index]));
        int written = seen ? snprintf(extra + used, sizeof(extra) - used,
            " reg%02x_raw=%.9g reg%02x_age_ms=%lu", 0x50U + index, (double)reader->values[index],
            0x50U + index, (unsigned long)(age > UINT32_MAX ? UINT32_MAX : age)) :
            snprintf(extra + used, sizeof(extra) - used, " reg%02x_raw=? reg%02x_age_ms=4294967295", 0x50U + index, 0x50U + index);
        if (written < 0 || (size_t)written >= sizeof(extra) - used) return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        used += (size_t)written;
    }
    {
        uint8_t velocity_valid = (uint8_t)(aethor_app_idle_position_allowed(now_us) &&
            reader->joint_index == 6U && (reader->seen_mask & 1U) && reader->estimated_velocity_valid &&
            now_us >= reader->sample_us[0] && now_us - reader->sample_us[0] < 500000ULL &&
            isfinite(reader->estimated_native_rad_s));
        uint8_t contract_valid = (uint8_t)(reader->joint_index == 6U && aethor_app_idle_position_allowed(now_us));
        int written = velocity_valid ? snprintf(extra + used, sizeof(extra) - used,
            " pm_contract=%u pm_vel=%.7g", contract_valid, (double)reader->estimated_native_rad_s) :
            snprintf(extra + used, sizeof(extra) - used, " pm_contract=%u pm_vel=?", contract_valid);
        if (written < 0 || (size_t)written >= sizeof(extra) - used) return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        used += (size_t)written;
    }
    {
        /* Coordinate reference is shared by native/register telemetry and HOME, distinct from authorization epoch. */
        aethor_app_debug_ui_refresh_reference(now_us);
        int written = snprintf(extra + used, sizeof(extra) - used,
            " pm_boot=%lu pm_epoch=%lu coord_boot=%lu coord_ref=%lu",
            (unsigned long)application_protocol_engine.boot_id, (unsigned long)application_debug_ui.epoch,
            (unsigned long)application_protocol_engine.boot_id,
            (unsigned long)application_debug_ui.reference_generation[6]);
        if (written < 0 || (size_t)written >= sizeof(extra) - used) return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        used += (size_t)written;
    }
    message = &output->messages[0];
    original_length = message->length;
    while (original_length && (message->data[original_length - 1U] == '\r' || message->data[original_length - 1U] == '\n')) --original_length;
    if (original_length + used + 3U > sizeof(message->data)) return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    memcpy(message->data + original_length, extra, used);
    memcpy(message->data + original_length + used, "\r\n", 3U);
    message->length = (uint16_t)(original_length + used + 2U);
    return status;
}
#endif
