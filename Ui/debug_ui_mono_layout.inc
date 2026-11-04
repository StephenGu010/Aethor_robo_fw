/** @file debug_ui_mono_layout.inc
 * @brief Allocation-free monochrome templates for all legacy LCD page families.
 * Reuses semantic rows and fixed labels; presentation never mutates control state.
 */

/** @brief Select a semantic menu icon from the existing page and immutable item index. */
static unsigned mono_menu_icon(const DebugUiModel *model, unsigned item)
{
    static const uint8_t action_icons[]={STATIC_ICON_MOTOR,STATIC_ICON_ANGLE,STATIC_ICON_SPEED,
        STATIC_ICON_TIMER,STATIC_ICON_CONTROL,STATIC_ICON_STOP};
    static const uint8_t recovery_icons[]={STATIC_ICON_WARNING,STATIC_ICON_STOP,STATIC_ICON_REGISTER,STATIC_ICON_CONTROL};
    static const uint8_t field_icons[]={STATIC_ICON_ANGLE,STATIC_ICON_SPEED,STATIC_ICON_CONTROL,STATIC_ICON_CONTROL,STATIC_ICON_TIMER};
    if (model->page==DEBUG_UI_PAGE_ACTIONS) return action_icons[item%6U];
    if (model->page==DEBUG_UI_PAGE_RECOVERY) return recovery_icons[item%4U];
    if (model->page==DEBUG_UI_PAGE_DIAGNOSTIC_MENU)
        return item==0U?STATIC_ICON_DISPLAY:(item==1U?STATIC_ICON_COMM:STATIC_ICON_MEMORY);
    if (model->page==DEBUG_UI_PAGE_EDIT)
        return item<debug_ui_model_parameter_count(model)?field_icons[debug_ui_model_parameter_field(model,(uint8_t)item)%5U]:STATIC_ICON_PLAY;
    if (model->page==DEBUG_UI_PAGE_MODES) return STATIC_ICON_CONTROL;
    if (model->page==DEBUG_UI_PAGE_STATIC_ACTIONS || model->page==DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS) {
        unsigned offset=model->page==DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS?1U:0U;
        if (offset && item==0U) return STATIC_ICON_MOTOR;
        item-=offset;
        return item==0U?STATIC_ICON_CONTROL:(item==1U?STATIC_ICON_LOCK:(item==2U?STATIC_ICON_TIMER:
            (item==3U?STATIC_ICON_PLAY:(item==4U?STATIC_ICON_STOP:STATIC_ICON_REGISTER))));
    }
    return STATIC_ICON_MOTOR;
}

/** @brief Fit a semantic line to its fixed box using the existing sparse typefaces. */
static void mono_label(DebugUiViewPage *page,unsigned index,int x,int y,int width,
                        const char *text,const lv_font_t *font,uint8_t selected)
{
    uint32_t offset=0U;
    /* Scientific fallback and explicit unknown/status strings may exceed the
     * numeric subset even when their measured width fits the field. */
    while (text[offset]!='\0') {
        uint32_t codepoint=_lv_txt_encoded_next(text,&offset);
        lv_font_glyph_dsc_t glyph;
        if (codepoint>32U && (!lv_font_get_glyph_dsc(font,&glyph,codepoint,0U) || glyph.is_placeholder)) {
            font=&ui_font_16;
            break;
        }
    }
    pos_load_label(page,index,x,y,width,text,font,selected);
}

/** @brief Require current snapshot provenance before summarizing fresh motor values. */
static uint8_t mono_motor_current(const DebugUiModel *model,const DebugUiMotorView *motor)
{
    return (uint8_t)(model->snapshot_seen && model->now_us>=model->snapshot.timestamp_us &&
        model->now_us-model->snapshot.timestamp_us<DEBUG_UI_HEALTH_TIMEOUT_US &&
        motor->feedback_seen && motor->feedback_valid);
}

/** @brief Render icon lists with an inverse focus and a fresh-evidence action summary. */
static void mono_menu_layout(DebugUiViewPage *page,const DebugUiModel *model,unsigned count)
{
    unsigned index,first=model->list_first;
    uint8_t actions=(uint8_t)(model->page==DEBUG_UI_PAGE_ACTIONS);
    int stride=actions?26:32;
    char text[DEBUG_UI_VIEW_TEXT_BYTES];
    const DebugUiMotorView *motor=&model->snapshot.motors[model->selected_motor<7U?model->selected_motor:0U];
    if (count<=5U) first=0U; else if (first>count-5U) first=count-5U;
    page->scrollbar_count=(uint8_t)count; page->scrollbar_first=(uint8_t)first;
    for (index=0U;index<5U && first+index<count;++index) {
        unsigned item=first+index;
        int top=35+(int)index*stride;
        uint8_t selected=(uint8_t)(item==model->focus);
        static_panel_add(page,12,top,268,top+stride-3,selected);
        static_icon_add(page,mono_menu_icon(model,item),18,top,1U,selected);
        if (model->page==DEBUG_UI_PAGE_EDIT && item<debug_ui_model_parameter_count(model)) {
            unsigned field=debug_ui_model_parameter_field(model,(uint8_t)item);
            astra_number(text,sizeof(text),&model->draft,field);
            pos_load_caption(page,index,51,top+8,87,astra_fields[field],selected);
            mono_label(page,index,139,top+6,121,text,&ui_font_16,selected);
        } else {
            astra_entry(text,sizeof(text),model,model->page,item);
            mono_label(page,index,51,top+(actions?4:7),209,text,&ui_font_16,selected);
        }
    }
    if (actions) {
        (void)snprintf(text,sizeof(text),"%s / %s",motor->mode_verified?mode_text(motor->actual_mode):"模式未校验",
            motor->feedback_seen?(mono_motor_current(model,motor)?(motor->enabled?"已使能":"已失能"):"反馈过期"):"状态未知");
        mono_label(page,5U,16,172,248,text,&ui_font_popup_12,0U);
        if (mono_motor_current(model,motor) && isfinite(motor->position_rad) &&
            isfinite(motor->velocity_rad_s) && isfinite(motor->torque_nm) &&
            fabsf(debug_ui_radians_to_degrees(motor->position_rad))<10000.0f &&
            fabsf(debug_ui_radians_to_degrees(motor->velocity_rad_s))<10000.0f && fabsf(motor->torque_nm)<1000.0f)
            (void)snprintf(text,sizeof(text),"%.1f° / %.1f°/s / %.2f Nm",
                (double)debug_ui_radians_to_degrees(motor->position_rad),
                (double)debug_ui_radians_to_degrees(motor->velocity_rad_s),(double)motor->torque_nm);
        else (void)snprintf(text,sizeof(text),"位置 -- / 速度 -- / 转矩 --");
        mono_label(page,6U,16,192,248,text,&ui_font_popup_12,0U);
    }
}

/** @brief Retain all seven semantic diagnostic/status rows with bounded original icons. */
static void mono_evidence_layout(DebugUiViewPage *page,const DebugUiModel *model)
{
    unsigned index;
    for (index=0U;index<7U;++index) {
        unsigned icon=STATIC_ICON_MOTOR;
        if (model->page==DEBUG_UI_PAGE_DIAGNOSTICS) {
            static const uint8_t diagnostic_icons[3][7]={
                {STATIC_ICON_CONTROL,STATIC_ICON_CONTROL,STATIC_ICON_WARNING,STATIC_ICON_DISPLAY,STATIC_ICON_DISPLAY,STATIC_ICON_MEMORY,STATIC_ICON_TIMER},
                {STATIC_ICON_TIMER,STATIC_ICON_COMM,STATIC_ICON_COMM,STATIC_ICON_WARNING,STATIC_ICON_STOP,STATIC_ICON_REGISTER,STATIC_ICON_WARNING},
                {STATIC_ICON_REGISTER,STATIC_ICON_COMM,STATIC_ICON_COMM,STATIC_ICON_MEMORY,STATIC_ICON_MEMORY,STATIC_ICON_DISPLAY,STATIC_ICON_WARNING}};
            icon=diagnostic_icons[model->diagnostic_page<3U?model->diagnostic_page:2U][index];
        } else if (model->page==DEBUG_UI_PAGE_REGISTERS)
            icon=index==0U?STATIC_ICON_REGISTER:(index==1U || index==3U?STATIC_ICON_ANGLE:(index==2U || index==4U?STATIC_ICON_TIMER:STATIC_ICON_WARNING));
        else if (model->page==DEBUG_UI_PAGE_PREPARE)
            icon=index==0U?STATIC_ICON_MOTOR:(index==3U?STATIC_ICON_STOP:(index==5U?STATIC_ICON_CONTROL:STATIC_ICON_LOCK));
        else if (model->page==DEBUG_UI_PAGE_SINGLE_STATIC_LOAD || model->page==DEBUG_UI_PAGE_SINGLE_STATIC_RESULT)
            icon=index==0U?STATIC_ICON_MOTOR:(index==1U?STATIC_ICON_ANGLE:(index==2U?STATIC_ICON_TORQUE:
                (index==3U?STATIC_ICON_TEMP:(index==4U?STATIC_ICON_TIMER:(index==5U?STATIC_ICON_STOP:STATIC_ICON_CONTROL)))));
        static_icon_add(page,icon,16,35+(int)index*24,1U,0U);
        mono_label(page,index,49,39+(int)index*24,215,page->row_text[index],&ui_font_popup_12,0U);
    }
}

/** @brief Render output angle, velocity and torque without promoting stale/nonfinite samples. */
static void mono_detail_layout(DebugUiViewPage *page,const DebugUiModel *model)
{
    const DebugUiMotorView *motor=&model->snapshot.motors[model->selected_motor<7U?model->selected_motor:0U];
    uint8_t current=mono_motor_current(model,motor);
    uint8_t snapshot_expired=(uint8_t)(!current && motor->feedback_seen && motor->feedback_valid);
    char text[DEBUG_UI_VIEW_TEXT_BYTES];
    unsigned index;
    if (snapshot_expired) {
        (void)snprintf(text,sizeof(text),"ID%u %s / 快照过期",model->selected_motor+1U,
            motor->mode_verified?mode_text(motor->actual_mode):"模式未校验");
        mono_label(page,0U,16,36,248,text,&ui_font_16,0U);
    } else mono_label(page,0U,16,36,248,page->row_text[0],&ui_font_16,0U);
    static_icon_add(page,STATIC_ICON_ANGLE,18,68,1U,0U);
    debug_ui_format_value(text,sizeof(text),debug_ui_radians_to_degrees(motor->position_rad),
        motor->feedback_seen,current,"°");
    if (!current && debug_ui_raw_position_current(model,motor))
        (void)snprintf(text,sizeof(text),"%.2f°",(double)debug_ui_register_output_degrees(motor));
    mono_label(page,1U,55,64,207,text,current?&ui_font_numeric_28:&ui_font_16,0U);
    for (index=2U;index<=3U;++index) {
        int x=index==2U?16:145;
        static_icon_add(page,index==2U?STATIC_ICON_SPEED:STATIC_ICON_TORQUE,x,104,1U,0U);
        pos_load_caption(page,index,x+32,111,84,index==2U?
            (!current && debug_ui_estimated_speed_current(model,motor)?"估算 °/s":"速度 °/s"):"转矩 Nm",0U);
        if (current) {
            float value=index==2U?debug_ui_radians_to_degrees(motor->velocity_rad_s):motor->torque_nm;
            pos_load_number(text,sizeof(text),value,index==2U?1U:2U);
        } else if (index==2U && debug_ui_estimated_speed_current(model,motor)) {
            pos_load_number(text,sizeof(text),debug_ui_radians_to_degrees(
                motor->register_estimated_native_rad_s/motor->single_static_native_per_output),1U);
        } else (void)snprintf(text,sizeof(text),"--");
        mono_label(page,index,x,135,119,text,&ui_font_numeric_28,0U);
    }
    if (current) {
        char mos_temperature[32],rotor_temperature[32];
        /* Each sensor retains its own validity; a missing sample must not hide
         * the other sensor, and finite extremes use bounded scientific text. */
        pos_load_number(mos_temperature,sizeof(mos_temperature),motor->mos_temperature_c,0U);
        pos_load_number(rotor_temperature,sizeof(rotor_temperature),motor->rotor_temperature_c,0U);
        (void)snprintf(text,sizeof(text),"温度 MOS %s / 转子 %s°C",mos_temperature,rotor_temperature);
        mono_label(page,4U,16,174,248,text,&ui_font_popup_12,0U);
    } else mono_label(page,4U,16,174,248,snapshot_expired?"温度 -- / 快照过期":page->row_text[4],&ui_font_popup_12,0U);
    mono_label(page,5U,16,194,248,snapshot_expired?"快照过期 / 状态未知":page->row_text[5],&ui_font_popup_12,0U);
    /* DETAIL is read-only: obsolete semantic row 6 is intentionally not displayed. */
}

/** @brief Preserve the review popup geometry and isolated long-press confirmation surface. */
static void mono_review_layout(DebugUiViewPage *page,const DebugUiModel *model)
{
    unsigned index;
    object_set_pos(page->overlay,16,38); object_set_size(page->overlay,248,170);
    style_set_number(page->overlay,LV_STYLE_BG_OPA,LV_OPA_TRANSP);
    page->layout_overlay_visible=1U;
    static_icon_add(page,STATIC_ICON_PLAY,24,43,1U,0U);
    mono_label(page,0U,58,47,194,page->row_text[0],&ui_font_popup_12,0U);
    for (index=1U;index<6U;++index)
        mono_label(page,index,24,72+(int)(index-1U)*21,232,page->row_text[index],&ui_font_popup_12,0U);
    static_panel_add(page,22,180,258,203,1U);
    mono_label(page,6U,28,184,224,page->row_text[6],&ui_font_popup_12,1U);
    if (model->review_released && model->confirm_elapsed_ms>0U) {
        uint32_t elapsed=model->confirm_elapsed_ms>800U?800U:model->confirm_elapsed_ms;
        object_set_pos(page->confirm_bar,24,204); object_set_size(page->confirm_bar,(int)(232U*elapsed/800U),3);
        page->layout_confirm_visible=1U;
    }
}

/** @brief Separate outcome, lifecycle stage and disable evidence with an explicit return action. */
static void mono_result_layout(DebugUiViewPage *page,const DebugUiModel *model)
{
    uint8_t confirmed=(uint8_t)(model->stop_latch_state==DEBUG_UI_STOP_LATCH_CONFIRMED ||
        (model->stop_latch_state==DEBUG_UI_STOP_LATCH_NONE && model->completion_seen && model->completion.disabled_confirmed));
    static_icon_add(page,model->page==DEBUG_UI_PAGE_FAULT?STATIC_ICON_WARNING:STATIC_ICON_REGISTER,16,36,1U,0U);
    mono_label(page,0U,54,39,208,page->row_text[0],&ui_font_16,0U);
    mono_label(page,1U,16,70,248,page->row_text[1],&ui_font_popup_12,0U);
    mono_label(page,5U,16,96,248,page->row_text[2],&ui_font_popup_12,0U);
    static_icon_add(page,confirmed?STATIC_ICON_CHECK:STATIC_ICON_WARNING,16,119,1U,0U);
    mono_label(page,2U,54,124,208,page->row_text[3],&ui_font_popup_12,0U);
    mono_label(page,3U,16,154,248,page->row_text[4],&ui_font_popup_12,0U);
    mono_label(page,6U,16,174,248,page->row_text[5],&ui_font_popup_12,0U);
    static_panel_add(page,12,189,268,210,1U);
    mono_label(page,4U,24,192,232,debug_ui_model_result_requires_check(model)?"进入控制检查":"已读返回",&ui_font_popup_12,1U);
}

/** @brief Apply a whole-LCD family template, preserving accepted POS/number/guided templates. */
static uint8_t mono_layout(DebugUiViewPage *page,const DebugUiModel *model,unsigned count)
{
    if (model->page==DEBUG_UI_PAGE_OVERVIEW || model->page==DEBUG_UI_PAGE_NUMBER) return 0U;
    page->decoration=4U;
    if (count!=0U) mono_menu_layout(page,model,count);
    else if (model->page==DEBUG_UI_PAGE_DETAIL) mono_detail_layout(page,model);
    else if (model->page==DEBUG_UI_PAGE_REVIEW) mono_review_layout(page,model);
    else if (model->page==DEBUG_UI_PAGE_RESULT || model->page==DEBUG_UI_PAGE_FAULT) mono_result_layout(page,model);
    else if (model->page==DEBUG_UI_PAGE_NOTICE) {
        static_icon_add(page,STATIC_ICON_WARNING,20,48,2U,0U);
        mono_label(page,0U,88,58,174,"操作未提交",&ui_font_16,0U);
        mono_label(page,1U,20,118,242,debug_ui_reason_text(model->reason),&ui_font_16,0U);
        static_panel_add(page,12,181,268,210,1U);
        static_icon_add(page,STATIC_ICON_LEFT,20,182,1U,1U);
        mono_label(page,2U,55,188,202,"返回",&ui_font_16,1U);
    } else if (model->page==DEBUG_UI_PAGE_RUNNING && !static_load_running(model)) {
        uint8_t target=debug_ui_model_active_target(model);
        const DebugUiMotorView *motor=&model->snapshot.motors[target>0U && target<=7U?target-1U:0U];
        uint8_t seen=(uint8_t)(target!=0U && motor->feedback_seen && isfinite(debug_ui_radians_to_degrees(motor->position_rad)));
        char text[DEBUG_UI_VIEW_TEXT_BYTES];
        static_icon_add(page,STATIC_ICON_MOTOR,16,35,1U,0U);
        mono_label(page,0U,50,39,214,page->row_text[0],&ui_font_16,0U);
        mono_label(page,1U,16,66,248,page->row_text[1],&ui_font_popup_12,0U);
        mono_label(page,2U,16,86,248,page->row_text[2],&ui_font_popup_12,0U);
        static_icon_add(page,STATIC_ICON_ANGLE,16,111,1U,0U);
        debug_ui_format_value(text,sizeof(text),debug_ui_radians_to_degrees(motor->position_rad),seen,seen,"°");
        mono_label(page,3U,50,108,214,text,&ui_font_numeric_28,0U);
        if (seen) (void)snprintf(text,sizeof(text),"%s / %lums",mono_motor_current(model,motor)?"输出轴当前位置":"上次输出轴位置",(unsigned long)motor->feedback_age_ms);
        else (void)snprintf(text,sizeof(text),"输出轴位置未知");
        mono_label(page,6U,16,144,248,text,&ui_font_popup_12,0U);
        static_panel_add(page,12,165,268,190,1U);
        static_icon_add(page,STATIC_ICON_STOP,16,165,1U,1U);
        mono_label(page,4U,50,169,214,page->row_text[4],&ui_font_16,1U);
        mono_label(page,5U,16,195,248,page->row_text[6],&ui_font_popup_12,0U);
    } else mono_evidence_layout(page,model);
    return 1U;
}
