/** @file debug_ui_view.h
 * @brief One shared LVGL view and bounded semantic/display text storage.
 */
#ifndef UI_DEBUG_UI_VIEW_H
#define UI_DEBUG_UI_VIEW_H
#include "lvgl.h"
#include "../App/DebugUi/debug_ui_model.h"
#define DEBUG_UI_VIEW_ROWS 7U
#define DEBUG_UI_VIEW_TEXT_BYTES 160U

/** @brief UI/platform diagnostics copied into the view in UiTask only. */
typedef struct {
    uint16_t adc_raw;
    uint32_t adc_age_ms;
    DebugUiKey key;
    uint8_t adc_seen;
    uint8_t input_valid;
    uint32_t input_fault;
    uint32_t dma_errors;
    uint32_t spi_errors; /**< Cumulative platform count, never a current health flag. */
    uint32_t init_errors; /**< Cumulative initialization failures, retained after recovery. */
    uint32_t flush_ms;
    uint32_t stack_free_words;
    uint32_t control_execution_max_us;
    uint32_t control_period_max_us;
    uint8_t simulated;
    const char *firmware_version; /**< Process-lifetime build metadata, or NULL. */
} DebugUiViewDiagnostics;

/** @brief One reusable screen; semantic rows and displayed strings have stable storage. */
typedef struct {
    lv_obj_t *root;
    lv_obj_t *title;
    lv_obj_t *rows[DEBUG_UI_VIEW_ROWS];
    lv_obj_t *footer;
    lv_obj_t *icons[DEBUG_UI_VIEW_ROWS];
    lv_obj_t *confirm_bar;
    lv_obj_t *overlay;
    char background_text[5][80];
    uint8_t decoration;
    uint8_t tile_focus;
    int16_t tile_offset;
    int16_t animation_from;
    uint32_t animation_start;
    uint32_t animation_tick;
    uint8_t animation_active;
    uint8_t scrollbar_count;
    uint8_t scrollbar_first;
    char title_text[80];
    char row_text[DEBUG_UI_VIEW_ROWS][DEBUG_UI_VIEW_TEXT_BYTES];
    char footer_text[112];
    char display_text[DEBUG_UI_VIEW_ROWS][DEBUG_UI_VIEW_TEXT_BYTES];
} DebugUiViewPage;

/** @brief Single screen allocated at startup; model state never owns LVGL objects. */
typedef struct {
    DebugUiViewPage page;
    lv_group_t *group;
    DebugUiPage visible;
    uint8_t initialized;
} DebugUiView;

LV_FONT_DECLARE(ui_font_16);
LV_FONT_DECLARE(ui_font_numeric_28);
/** @brief Advance idle-only decorations without delaying input or control. */
void debug_ui_view_animate(DebugUiView *view);
/** @brief Create every object up front; fail closed on pool/object allocation loss. */
uint8_t debug_ui_view_init(DebugUiView *view);
/** @brief Update changed strings/styles only; must never run in flush wait or ISR. */
void debug_ui_view_update(DebugUiView *view, const DebugUiModel *model,
                          const DebugUiViewDiagnostics *diagnostics);
/** @brief Inspect the local LVGL assertion/OOM latch without a fatal handler. */
uint8_t debug_ui_view_healthy(void);
/** @brief Verify every UI character against actual bitmap/descriptor font callbacks. */
uint32_t debug_ui_view_missing_glyphs(const char *utf8);
#endif
