/** @file static_gain_render.c
 * @brief Real LVGL rendering of gain setup, numeric editing and unchanged ID scope.
 */
#define main gain_existing_render_main
#include "ui_render_smoke.c"
#undef main
/** @brief Render the actual shared widgets and exercise label/geometry assertions. */
int main(int argc, char **argv)
{
    assert(argc == 3);
    assert(debug_ui_graphics_run(initialize_graphics, NULL));
    verify_font_inventory(argv[2]);
    simulated_data();
    model.single_static_flow = 1U; model.selected_motor = 6U;
    model.single_static_kp = DEBUG_UI_STATIC_KP_DEFAULT; model.single_static_kd_centi = 30U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP; model.focus = 4U;
    save_page(argv[1], "gain_setup");
    assert(strstr(lv_label_get_text(view.page.rows[0]), "Kp 120") != NULL);
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_GAINS; model.focus = 0U;
    save_page(argv[1], "gain_parameters");
    assert(strstr(lv_label_get_text(view.page.rows[1]), "120") != NULL);
    model.gain_editing = 1U; model.gain_edit_value = 100U; model.gain_digit = 1U;
    save_page(argv[1], "gain_kp_digit");
    assert(strstr(lv_label_get_text(view.page.rows[1]), "[0]") != NULL);
    model.focus = 1U; model.gain_edit_value = 30U; model.gain_digit = 2U;
    save_page(argv[1], "gain_kd_digit");
    assert(strstr(lv_label_get_text(view.page.rows[2]), "0.3[0]") != NULL);
    puts("STATIC_GAIN_LVGL_PASS");
    return 0;
}
