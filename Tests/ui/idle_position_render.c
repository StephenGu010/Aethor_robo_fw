/** @file idle_position_render.c
 * @brief Render real LCD widgets with independent raw position and absent standard feedback.
 */
#define main existing_position_render_main
#include "ui_render_smoke.c"
#undef main

/** @brief Assert raw units, stale handling and standard-feedback precedence in real LVGL. */
int main(int argc, char **argv)
{
    DebugUiMotorView *motor;
    assert(argc == 3);
    assert(debug_ui_graphics_run(initialize_graphics, NULL));
    verify_font_inventory(argv[2]);
    simulated_data();
    model.selected_motor = 6U;
    motor = &model.snapshot.motors[6];
    motor->feedback_seen = motor->feedback_valid = 0U;
    motor->register_seen_mask = 3U;
    motor->register_position[0] = 19.2032F;
    motor->register_position[1] = 0.25F;
    motor->single_static_native_per_output = 19.2032F;
    motor->register_position_contract = 1U;
    motor->register_estimated_native_rad_s = 19.2032F;
    motor->register_estimated_velocity_valid = 1U;
    motor->register_age_ms[0] = 101U;
    motor->register_age_ms[1] = 1U;
    model.page = DEBUG_UI_PAGE_DETAIL;
    save_page(argv[1], "mode2_idle_detail");
    assert(strstr(lv_label_get_text(view.page.rows[1]), "57.30") != NULL);
    assert(strstr(lv_label_get_text(view.page.rows[2]), "57.3") != NULL);
    model.page = DEBUG_UI_PAGE_REGISTERS;
    save_page(argv[1], "mode2_idle_registers");
    motor->register_age_ms[0] = 500U;
    model.page = DEBUG_UI_PAGE_DETAIL;
    save_page(argv[1], "mode2_idle_expired");
    assert(strstr(lv_label_get_text(view.page.rows[1]), "57.30") == NULL);
    assert(strstr(lv_label_get_text(view.page.rows[2]), "--") != NULL);
    puts("MODE2_IDLE_LVGL_PASS engineering_angle_estimated_speed_and_expiry");
    return 0;
}
