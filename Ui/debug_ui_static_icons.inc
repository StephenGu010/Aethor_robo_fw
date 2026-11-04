/** @file debug_ui_static_icons.inc
 * @brief Original, bounded monochrome static-test line art; no imported code/assets.
 * Included after Astra drawing primitives; every path stays within a 24-unit square.
 */
enum {
    STATIC_ICON_MOTOR = 0, STATIC_ICON_ARM, STATIC_ICON_LOCK, STATIC_ICON_TIMER,
    STATIC_ICON_PLAY, STATIC_ICON_CHECK, STATIC_ICON_EMPTY, STATIC_ICON_STOP,
    STATIC_ICON_ANGLE, STATIC_ICON_TORQUE, STATIC_ICON_TEMP, STATIC_ICON_WARNING,
    STATIC_ICON_LEFT, STATIC_ICON_RIGHT, STATIC_ICON_CONTROL, STATIC_ICON_SPEED,
    STATIC_ICON_DISPLAY, STATIC_ICON_REGISTER, STATIC_ICON_MEMORY, STATIC_ICON_COMM
};

/** @brief Retain one original icon and assert its full stroke remains in content bounds. */
static void static_icon_add(DebugUiViewPage *page, unsigned kind, int x, int y,
                            unsigned scale, uint8_t inverse)
{
    DebugUiStaticIcon *icon;
    assert(page->static_art.icon_count < 12U && kind <= STATIC_ICON_COMM);
    assert(scale >= 1U && scale <= 2U && x >= 12 && y >= 33);
    assert(x + 24 * (int)scale + 2 <= 269 && y + 24 * (int)scale + 2 <= 211);
    icon = &page->static_art.icons[page->static_art.icon_count++];
    icon->x = (int16_t)x; icon->y = (int16_t)y;
    icon->kind = (uint8_t)kind; icon->scale = (uint8_t)scale; icon->inverse = inverse;
}

/** @brief Retain one selection/group surface without allocating another LVGL object. */
static void static_panel_add(DebugUiViewPage *page, int left, int top, int right, int bottom,
                             uint8_t selected)
{
    DebugUiStaticPanel *panel;
    assert(page->static_art.panel_count < 6U);
    assert(left >= 12 && top >= 33 && right <= 268 && bottom <= 210 && right > left && bottom > top);
    panel = &page->static_art.panels[page->static_art.panel_count++];
    panel->left = (int16_t)left; panel->top = (int16_t)top;
    panel->right = (int16_t)right; panel->bottom = (int16_t)bottom; panel->selected = selected;
}

/** @brief Rasterize original diagonals explicitly because LV_DRAW_COMPLEX=0 omits skew lines. */
static void static_segment(lv_draw_ctx_t *context, const DebugUiStaticIcon *icon,
                            const int8_t segment[4], uint32_t color)
{
    int x=icon->x+segment[0]*icon->scale, y=icon->y+segment[1]*icon->scale;
    int end_x=icon->x+segment[2]*icon->scale, end_y=icon->y+segment[3]*icon->scale;
    int delta_x=abs(end_x-x), step_x=x<end_x?1:-1;
    int delta_y=-abs(end_y-y), step_y=y<end_y?1:-1;
    int error=delta_x+delta_y;
    int thickness=icon->scale>1U?2:1;
    if (x==end_x || y==end_y) {
        astra_line(context,icon->x,icon->y,icon->scale,
            segment[0],segment[1],segment[2],segment[3],color);
        return;
    }
    for (;;) {
        int twice_error;
        astra_fill(context,x,y,x+thickness,y+thickness,color);
        if (x==end_x && y==end_y) break;
        twice_error=2*error;
        if (twice_error>=delta_y) { error+=delta_y; x+=step_x; }
        if (twice_error<=delta_x) { error+=delta_x; y+=step_y; }
    }
}

/** @brief Draw an original semantic path using only LVGL basic line descriptors. */
static void static_icon_draw(lv_draw_ctx_t *context, const DebugUiStaticIcon *icon)
{
    static const int8_t motor[][4]={{3,6,21,6},{21,6,21,18},{21,18,3,18},{3,18,3,6},{0,10,3,10},{0,14,3,14},{21,12,24,12},{7,2,17,2},{7,2,7,6},{17,2,17,6},{8,10,16,14},{16,10,8,14}};
    static const int8_t lock[][4]={{5,10,20,10},{20,10,20,23},{20,23,5,23},{5,23,5,10},{8,10,8,4},{8,4,11,1},{11,1,15,1},{15,1,18,4},{18,4,18,10},{12,14,12,19}};
    static const int8_t arm[][4] = {{1,23,22,23},{5,23,5,18},{2,18,9,18},{6,18,6,14},{6,14,13,9},{13,9,19,12},{19,12,22,8},{22,8,24,9},{18,5,22,5},{22,5,22,8},{4,11,8,11},{4,11,4,15},{4,15,8,15},{8,15,8,11},{11,6,15,6},{11,6,11,10},{11,10,15,10},{15,10,15,6}};
    static const int8_t timer[][4] = {{8,0,16,0},{12,0,12,3},{6,5,18,5},{18,5,23,10},{23,10,23,18},{23,18,18,23},{18,23,6,23},{6,23,1,18},{1,18,1,10},{1,10,6,5},{12,8,12,15},{12,15,17,15},{20,3,23,6}};
    static const int8_t play[][4] = {{6,2,6,22},{6,22,23,12},{23,12,6,2}};
    static const int8_t check[][4] = {{3,12,9,19},{9,19,22,4}};
    static const int8_t empty[][4] = {{5,2,19,2},{19,2,23,6},{23,6,23,19},{23,19,19,23},{19,23,5,23},{5,23,1,19},{1,19,1,6},{1,6,5,2}};
    static const int8_t stop[][4] = {{3,3,21,3},{21,3,21,21},{21,21,3,21},{3,21,3,3},{8,8,16,8},{16,8,16,16},{16,16,8,16},{8,16,8,8}};
    static const int8_t angle[][4] = {{2,2,2,22},{2,22,23,22},{2,22,20,5},{10,22,10,18},{10,18,8,16}};
    static const int8_t torque[][4] = {{4,7,4,18},{4,18,9,23},{9,23,18,23},{18,23,23,18},{23,18,23,9},{23,9,18,4},{18,4,7,4},{7,4,10,1},{7,4,10,7},{12,10,16,10},{16,10,16,15},{16,15,12,15},{12,15,12,10}};
    static const int8_t temp[][4] = {{10,16,10,3},{10,3,13,1},{13,1,16,3},{16,3,16,16},{16,16,19,19},{19,19,19,22},{19,22,16,24},{16,24,10,24},{10,24,7,22},{7,22,7,19},{7,19,10,16},{13,7,13,20},{18,7,22,7},{18,11,21,11}};
    static const int8_t warning[][4] = {{12,1,24,23},{24,23,0,23},{0,23,12,1},{12,8,12,15},{12,19,12,20}};
    static const int8_t left[][4] = {{15,5,8,12},{8,12,15,19}};
    static const int8_t right[][4] = {{9,5,16,12},{16,12,9,19}};
    static const int8_t control[][4] = {{1,8,23,8},{23,8,23,21},{23,21,1,21},{1,21,1,8},{5,1,5,8},{12,1,12,8},{19,1,19,8},{5,12,5,17},{12,12,12,17},{19,12,19,17}};
    static const int8_t speed[][4] = {{2,21,2,10},{2,10,7,4},{7,4,17,4},{17,4,22,10},{22,10,22,21},{2,21,22,21},{12,17,19,8},{6,14,8,14},{11,8,11,10}};
    static const int8_t display[][4] = {{1,3,23,3},{23,3,23,18},{23,18,1,18},{1,18,1,3},{12,18,12,23},{7,23,17,23},{5,8,9,8},{5,12,18,12}};
    static const int8_t registers[][4] = {{3,1,21,1},{21,1,21,23},{21,23,3,23},{3,23,3,1},{7,6,17,6},{7,12,17,12},{7,18,17,18}};
    static const int8_t memory[][4] = {{5,5,19,5},{19,5,19,19},{19,19,5,19},{5,19,5,5},{9,1,9,5},{15,1,15,5},{9,19,9,23},{15,19,15,23},{1,9,5,9},{1,15,5,15},{19,9,23,9},{19,15,23,15},{9,9,15,9},{15,9,15,15},{15,15,9,15},{9,15,9,9}};
    static const int8_t communication[][4] = {{2,6,22,6},{22,6,18,2},{22,6,18,10},{22,18,2,18},{2,18,6,14},{2,18,6,22}};
    const int8_t (*segments)[4] = NULL;
    unsigned count = 0U, index;
    uint32_t color = icon->inverse ? 0U : 0xFFFFFFU;
    lv_area_t bounds={(lv_coord_t)(icon->x-2),(lv_coord_t)(icon->y-2),
        (lv_coord_t)(icon->x+24*icon->scale+2),(lv_coord_t)(icon->y+24*icon->scale+2)};
    lv_area_t clipped;
    /* Skip the complete path when neither this clip nor this partial draw buffer can see it. */
    if (!_lv_area_intersect(&clipped,&bounds,context->clip_area) ||
        !_lv_area_intersect(&clipped,&clipped,context->buf_area)) return;
#define STATIC_PATH(name) do { segments = name; count = sizeof(name)/sizeof(name[0]); } while (0)
    switch (icon->kind) {
    case STATIC_ICON_MOTOR: STATIC_PATH(motor); break;
    case STATIC_ICON_LOCK: STATIC_PATH(lock); break;
    case STATIC_ICON_ARM: STATIC_PATH(arm); break;
    case STATIC_ICON_TIMER: STATIC_PATH(timer); break;
    case STATIC_ICON_PLAY: STATIC_PATH(play); break;
    case STATIC_ICON_CHECK: STATIC_PATH(check); break;
    case STATIC_ICON_EMPTY: STATIC_PATH(empty); break;
    case STATIC_ICON_STOP: STATIC_PATH(stop); break;
    case STATIC_ICON_ANGLE: STATIC_PATH(angle); break;
    case STATIC_ICON_TORQUE: STATIC_PATH(torque); break;
    case STATIC_ICON_TEMP: STATIC_PATH(temp); break;
    case STATIC_ICON_WARNING: STATIC_PATH(warning); break;
    case STATIC_ICON_LEFT: STATIC_PATH(left); break;
    case STATIC_ICON_RIGHT: STATIC_PATH(right); break;
    case STATIC_ICON_SPEED: STATIC_PATH(speed); break;
    case STATIC_ICON_DISPLAY: STATIC_PATH(display); break;
    case STATIC_ICON_REGISTER: STATIC_PATH(registers); break;
    case STATIC_ICON_MEMORY: STATIC_PATH(memory); break;
    case STATIC_ICON_COMM: STATIC_PATH(communication); break;
    default: STATIC_PATH(control); break;
    }
#undef STATIC_PATH
    for (index = 0U; index < count; ++index)
        static_segment(context, icon, segments[index], color);
}

/** @brief Draw the retained monochrome surfaces/icons before transparent label children. */
static void static_art_draw(lv_draw_ctx_t *context, const DebugUiViewPage *page)
{
    unsigned index;
    for (index = 0U; index < page->static_art.panel_count; ++index) {
        const DebugUiStaticPanel *panel = &page->static_art.panels[index];
        uint32_t edge = panel->selected ? 0xFFFFFFU : 0x555555U;
        astra_fill(context, panel->left, panel->top, panel->right, panel->bottom, edge);
        if (!panel->selected)
            astra_fill(context, panel->left+1, panel->top+1, panel->right-1, panel->bottom-1, 0U);
    }
    for (index = 0U; index < page->static_art.icon_count; ++index)
        static_icon_draw(context, &page->static_art.icons[index]);
    if (page->decoration == 5U) {
        for (index = 0U; index < 24U; ++index)
            astra_fill(context, 16+(int)index*10, 101, 22+(int)index*10, 104,
                index < page->static_art.progress_steps ? 0xFFFFFFU : 0x444444U);
        astra_line(context,0,0,1,140,111,140,146,0x555555U);
        astra_parent_text(context,&ui_font_popup_12,"输出轴偏差",42,112,0U,0xAAAAAAU);
        astra_parent_text(context,&ui_font_popup_12,"反馈转矩",173,112,0U,0xAAAAAAU);
    }
}
