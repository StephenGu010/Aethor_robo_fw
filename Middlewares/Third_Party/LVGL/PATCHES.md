# Local compatibility patches on LVGL v8.3.11

Upstream tag `v8.3.11`, commit `74d0a816a440eea53e030c4f1af842a94f7ce3d3`.
Archive SHA-256 `a992bdb8a13b9b706857889ac88d28e76d23c0aa57e1da8ddfc2816a093171c1`.
Upstream MIT license remains in `LICENCE.txt`; original and current file hashes are recorded in `SOURCE_MANIFEST.json`.

2026-09-08: narrow ARMCC5 compatibility changes requested during the real Keil read-only build. No global warning suppression, allocator algorithm change, version upgrade, or new external dependency.

| File | Change | Preserved behavior |
|---|---|---|
| `src/core/lv_event.c` | Explicit `lv_event_code_t` cast after masking `LV_EVENT_PREPROCESS`. | Same event bits and returned value. |
| `src/core/lv_obj_draw.c` | Explicit `lv_layer_type_t` cast from the stored layer bit field. | Same layer value; no layout change. |
| `src/hal/lv_hal_disp.c` | Explicit `lv_disp_rot_t` cast from the stored rotation bit field. | Same rotation value; no display-transform change. |
| `src/misc/lv_style.c` | Explicit `lv_style_prop_t` casts for custom property IDs, count and property/meta combination. | Same integer representation at the enum API boundary. |
| `src/misc/lv_tlsf.c` | At the three pool-header negative offsets, use `(size_t)(-(tlsfptr_t)block_header_overhead)`. | Makes the existing signed-to-unsigned conversion explicit. The offset function still receives the identical modulo-`size_t` negative offset; its signature and arithmetic remain unchanged. Both 32-bit ARMCC and 64-bit host allocator paths are exercised. |
| `src/font/lv_font_fmt_txt.c` | Remove the final unreachable `return NULL` after exhaustive returning branches. | Missing glyph, plain bitmap, compressed bitmap and compressed-disabled paths already return inside their respective branch; glyph lookup behavior is unchanged. |
| `src/misc/lv_mem.c` | Optional `LV_MEM_FAILURE_HANDLER()` hook at the `malloc` and `realloc` NULL branches. | Undefined hook preserves upstream behavior. This UI defines the hook to latch graphics failure and escape its UiTask-local boundary before unchecked upstream object-array allocation paths dereference NULL. The real OOM smoke test exposed that an assertion-only handler was insufficient. Successful allocation and allocator accounting are unchanged. |

Validation entrypoints: `Tests/ui/run_ui_render_smoke.ps1` renders the real patched LVGL through the production double-buffer port, checks all UI glyphs and page widths, exercises pool allocation, and deliberately forces allocation failure through the local graphics escape boundary. ARMCC5 per-file probes compile the six patched translation units with C99 and no disabled diagnostics. Full Keil target builds and linked memory maps are owned by the parent integration task.
