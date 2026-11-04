/** @file mono_render_cases.inc
 * @brief Whole-LCD monochrome contracts, rendered only from synthetic snapshots.
 * Keeps legacy semantic/security assertions and the eighteen-object limit intact.
 */

/** @brief Require semantic line art on every formerly plain list or fallback family. */
static void render_mono_page_families(const char *directory)
{
    static const DebugUiPage families[] = {
        DEBUG_UI_PAGE_ACTIONS, DEBUG_UI_PAGE_MOTORS, DEBUG_UI_PAGE_DETAIL,
        DEBUG_UI_PAGE_DIAGNOSTICS, DEBUG_UI_PAGE_REGISTERS, DEBUG_UI_PAGE_PREPARE,
        DEBUG_UI_PAGE_RECOVERY, DEBUG_UI_PAGE_MODES, DEBUG_UI_PAGE_DIAGNOSTIC_MENU,
        DEBUG_UI_PAGE_STATIC_LOAD, DEBUG_UI_PAGE_STATIC_ACTIONS, DEBUG_UI_PAGE_STATIC_RESULT,
        DEBUG_UI_PAGE_SINGLE_STATIC_LOAD, DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS,
        DEBUG_UI_PAGE_SINGLE_STATIC_IDS, DEBUG_UI_PAGE_SINGLE_STATIC_RESULT,
        DEBUG_UI_PAGE_REVIEW, DEBUG_UI_PAGE_RUNNING, DEBUG_UI_PAGE_RESULT,
        DEBUG_UI_PAGE_FAULT, DEBUG_UI_PAGE_NOTICE, DEBUG_UI_PAGE_EDIT
    };
    unsigned index;
    char name[96];
    for (index=0U; index<sizeof(families)/sizeof(families[0]); ++index) {
        astra_scenario_draft(DEBUG_UI_OPERATION_MIT_MOVE,families[index]);
        model.selected_motor=0U; model.draft.target_motor_id=1U;
        model.reviewed=model.draft;
        /* Existing named page fixtures export these families below. Contract-only
         * repeats must not create duplicate artifacts or relax uniqueness checks. */
        assert(debug_ui_graphics_run(render_graphics,NULL));
        assert(view.page.static_art.icon_count>0U);
        assert(lv_obj_get_child_cnt(view.page.root)==18U);
        assert(!model.outgoing_ready && !model.stop_ready);
    }
    astra_scenario_draft(DEBUG_UI_OPERATION_MIT_MOVE,DEBUG_UI_PAGE_ACTIONS);
    model.selected_motor=0U;
    model.snapshot.motors[0].feedback_valid=0U;
    model.snapshot.motors[0].mode_verified=0U;
    save_page(directory,"262_mono_actions_stale_unverified");
    assert(strstr(view.page.display_text[5],"过期")!=NULL);
    assert(strstr(view.page.display_text[5],"未校验")!=NULL);
    assert(strstr(view.page.display_text[6],"12.5")==NULL);
    model.page=DEBUG_UI_PAGE_DETAIL;
    model.snapshot.motors[0].position_rad=NAN;
    model.snapshot.motors[0].velocity_rad_s=INFINITY;
    model.snapshot.motors[0].torque_nm=NAN;
    save_page(directory,"263_mono_detail_invalid");
    assert(strstr(view.page.display_text[1],"--")!=NULL);
    assert(strstr(view.page.display_text[2],"--")!=NULL);
    assert(strstr(view.page.display_text[3],"--")!=NULL);
    assert((view.page.layout_row_mask&(1U<<6U))==0U);
    for (index=0U; index<3U; ++index) {
        simulated_data(); model.page=DEBUG_UI_PAGE_DIAGNOSTICS;
        model.diagnostic_page=(uint8_t)index;
        (void)snprintf(name,sizeof(name),"%u_mono_diagnostic_tab_%u",264U+index,index);
        diagnostics.spi_errors=UINT32_MAX;
        diagnostics.init_errors=UINT32_MAX;
        diagnostics.adc_raw=1234U;
        model.snapshot.diagnostics.control_deadline_miss_count=12345U;
        save_page(directory,name);
        assert(view.page.layout_row_mask==0x7FU);
        assert(strstr(view.page.footer_text,"上下/右")!=NULL);
    }
    astra_scenario_draft(DEBUG_UI_OPERATION_MIT_MOVE,DEBUG_UI_PAGE_DETAIL);
    {
        static const float invalid_values[]={NAN,INFINITY,-INFINITY,FLT_MAX};
        unsigned sample;
        for (sample=0U;sample<sizeof(invalid_values)/sizeof(invalid_values[0]);++sample) {
            model.snapshot.motors[6].position_rad=invalid_values[sample];
            model.snapshot.motors[6].velocity_rad_s=invalid_values[sample];
            model.snapshot.motors[6].torque_nm=invalid_values[sample];
            assert(debug_ui_graphics_run(render_graphics,NULL));
            assert(strstr(view.page.display_text[1],"--")!=NULL);
            if (sample<3U) {
                assert(strstr(view.page.display_text[2],"--")!=NULL);
                assert(strstr(view.page.display_text[3],"--")!=NULL);
            }
        }
    }
    save_page(directory,"267_mono_detail_fresh_extreme");
    simulated_data(); model.page=DEBUG_UI_PAGE_MOTORS;
    model.snapshot.motors[0].mode_verified=0U;
    save_page(directory,"268_mono_motors_unverified_mode");
    assert(strstr(view.page.display_text[0],"未校验")!=NULL);
    model.page=DEBUG_UI_PAGE_DETAIL;
    save_page(directory,"269_mono_detail_unverified_mode");
    assert(strstr(view.page.display_text[0],"未校验")!=NULL);
    model.page=DEBUG_UI_PAGE_ACTIONS;
    model.snapshot.motors[0].mode_verified=1U;
    model.snapshot.motors[0].velocity_rad_s=debug_ui_degrees_to_radians(1.25f);
    save_page(directory,"270_mono_actions_fresh_summary");
    assert(strstr(view.page.display_text[5],"已失能")!=NULL);
    assert(strstr(view.page.display_text[6],"12.5")!=NULL);
    model.now_us=model.snapshot.timestamp_us+DEBUG_UI_HEALTH_TIMEOUT_US;
    assert(debug_ui_graphics_run(render_graphics,NULL));
    assert(strstr(view.page.display_text[5],"过期")!=NULL);
    assert(strstr(view.page.display_text[6],"12.5")==NULL);
    model.page=DEBUG_UI_PAGE_DETAIL;
    save_page(directory,"272_mono_detail_snapshot_expired");
    assert(strstr(view.page.display_text[0],"快照过期")!=NULL);
    assert(strstr(view.page.display_text[1],"过期")!=NULL);
    assert(strstr(view.page.display_text[2],"--")!=NULL);
    assert(strstr(view.page.display_text[3],"--")!=NULL);
    model.page=DEBUG_UI_PAGE_MOTORS;
    assert(debug_ui_graphics_run(render_graphics,NULL));
    assert(strstr(view.page.display_text[0],"过期")!=NULL);
    astra_scenario_draft(DEBUG_UI_OPERATION_MIT_MOVE,DEBUG_UI_PAGE_EDIT);
    model.draft.delta_rad=debug_ui_degrees_to_radians(-180.0f);
    model.draft.speed_rad_s=debug_ui_degrees_to_radians(999.9f);
    model.draft.kp=500.0f; model.draft.kd=5.0f;
    model.draft.hold_duration_ms=60000U; model.focus=3U;
    save_page(directory,"271_mono_edit_large_parameters");
    astra_scenario_draft(DEBUG_UI_OPERATION_MIT_MOVE,DEBUG_UI_PAGE_DETAIL);
    {
        static const float temperatures[]={NAN,INFINITY,-INFINITY,FLT_MAX};
        unsigned sample;
        for (sample=0U;sample<sizeof(temperatures)/sizeof(temperatures[0]);++sample) {
            model.snapshot.motors[6].mos_temperature_c=temperatures[sample];
            model.snapshot.motors[6].rotor_temperature_c=43.0f;
            assert(debug_ui_graphics_run(render_graphics,NULL));
            assert(strstr(view.page.display_text[4],"43")!=NULL);
            assert(strstr(view.page.display_text[4],"nan")==NULL);
            assert(strstr(view.page.display_text[4],"inf")==NULL);
            if (sample<3U) assert(strstr(view.page.display_text[4],"--")!=NULL);
            model.snapshot.motors[6].mos_temperature_c=37.0f;
            model.snapshot.motors[6].rotor_temperature_c=temperatures[sample];
            assert(debug_ui_graphics_run(render_graphics,NULL));
            assert(strstr(view.page.display_text[4],"37")!=NULL);
            assert(strstr(view.page.display_text[4],"nan")==NULL);
            assert(strstr(view.page.display_text[4],"inf")==NULL);
            if (sample<3U) assert(strstr(view.page.display_text[4],"--")!=NULL);
        }
        model.snapshot.motors[6].mos_temperature_c=NAN;
        model.snapshot.motors[6].rotor_temperature_c=-INFINITY;
        save_page(directory,"273_mono_detail_temperatures_unknown");
        assert(strstr(view.page.display_text[4],"MOS --")!=NULL);
        assert(strstr(view.page.display_text[4],"转子 --")!=NULL);
        model.snapshot.motors[6].mos_temperature_c=FLT_MAX;
        model.snapshot.motors[6].rotor_temperature_c=FLT_MAX;
        save_page(directory,"274_mono_detail_temperatures_extreme");
    }
    puts("LCD_MONO_ALL_FAMILIES_OK families=22 stale_nonfinite=1 diagnostics_rows=7 children=18");
}
