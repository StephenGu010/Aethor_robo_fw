# LCD UI Refresh Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce key-to-pixel latency and eliminate avoidable full-screen animation transfers while preserving the low-memory DMA design and all motor-safety behavior.

**Architecture:** Keep UiTask as the sole LVGL owner and SPI1/DMA as the asynchronous display transport. Add a UiTask-local dirty flag for immediate model-to-view mapping, restrict the home animation invalidation rectangle, and track complete LVGL refreshes in the display port without changing the platform health contract.

**Tech Stack:** STM32H723, C99, FreeRTOS/CMSIS-RTOS, LVGL 8.3.11, STM32 HAL SPI/DMA, PowerShell host regression scripts, Keil MDK targets.

---

### Task 1: Lock the performance contract with failing tests

**Files:**
- Modify: `Tests/host/lcd_fakes/lcd_fake_hal.h`
- Modify: `Tests/host/lcd_fakes/platform_test_main.c`
- Modify: `Tests/ui/task_display_regression.c`

- [ ] **Step 1: Define the 10 MHz divider in the fake HAL**

Add the exact HAL value used by production:

```c
#define SPI_BAUDRATEPRESCALER_8 2U
#define SPI_BAUDRATEPRESCALER_16 3U
```

- [ ] **Step 2: Change the platform expectation before production code**

```c
CHECK(fake_hal.spi->Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_8);
```

- [ ] **Step 3: Add task/display expectations**

Add a regression that requires an input event to set the UiTask-local `view_dirty` flag and requires complete-refresh statistics after the fake display releases the last LVGL block:

```c
LvPortDispRefreshStatus refresh_status;
view_dirty = 0U;
/* Feed one stable directional key through service_control_io(). */
assert(view_dirty == 1U);
graphics_service(NULL);
assert(view_dirty == 0U && last_view_ms == fake_ms);
lv_port_disp_refresh_status(&refresh_status);
assert(refresh_status.completed > 0U);
assert(refresh_status.last_bytes > 0U);
assert(refresh_status.last_duration_ms > 0U);
assert(refresh_status.fps_tenths == 10000U / refresh_status.last_duration_ms);
```

- [ ] **Step 4: Run the focused tests and verify RED**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_lcd_transport_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/ui/run_review_regressions.ps1 -Suite task
```

Expected: platform test fails on divider 16 versus 8; UI regression fails because `view_dirty`, `LvPortDispRefreshStatus`, and `lv_port_disp_refresh_status()` do not exist.

- [ ] **Step 5: Keep the failing contract uncommitted until its implementation is green**

Do not create a deliberately failing commit. Commit each contract together with the production slice that makes it pass.

### Task 2: Make model-driven view changes immediate

**Files:**
- Modify: `Ui/debug_ui_task.c`
- Test: `Tests/ui/task_display_regression.c`

- [ ] **Step 1: Add the task-owned dirty flag**

```c
static uint8_t graphics_initialized, graphics_running;
static uint8_t redraw_after_recovery, view_dirty;
```

- [ ] **Step 2: Mark actual model mutations**

After each delivered emergency/ordinary input event and each consumed admission/completion, set `view_dirty = 1U`. Set it once after graphics initialization and after successful display recovery.

- [ ] **Step 3: Consume the flag in graphics service**

```c
if (view_dirty || task_view.visible != task_model.page ||
    (uint32_t)(now_ms - last_view_ms) >= 100U) {
    debug_ui_view_update(&task_view, &task_model, &task_diagnostics);
    last_view_ms = now_ms;
    view_dirty = 0U;
}
```

- [ ] **Step 4: Run the task regression and verify GREEN**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/ui/run_review_regressions.ps1 -Suite task
```

Expected: `TASK_VIEW_DIRTY_PASS` plus all existing task display passes.

- [ ] **Step 5: Commit the scheduling slice**

```powershell
git add Ui/debug_ui_task.c Tests/ui/task_display_regression.c
git commit -m "perf: refresh LCD view immediately after model events"
```

### Task 3: Restrict the home animation invalidation area

**Files:**
- Modify: `Ui/debug_ui_astra_layout.inc`
- Modify: `Tests/host/test_debug_ui_build_contract.py`

- [ ] **Step 1: Add a static source contract before production modification**

Read `Ui/debug_ui_astra_layout.inc` and require the animation function to call `lv_obj_invalidate_area` with a named fixed area while rejecting `lv_obj_invalidate(page->root)` inside that function.

- [ ] **Step 2: Verify the contract fails**

Run:

```powershell
python Tests/host/test_debug_ui_build_contract.py
```

Expected: FAIL because the animation still invalidates the root object.

- [ ] **Step 3: Implement the fixed animation rectangle**

Inside `debug_ui_view_animate()` use a rectangle that covers every old/new icon and bracket position:

```c
static const lv_area_t home_animation_area = {12, 58, 268, 180};
lv_obj_invalidate_area(page->root, &home_animation_area);
```

Keep the full root invalidation in `layout()` because page composition changes can affect the whole screen.

- [ ] **Step 4: Run build-contract and render regressions**

Run:

```powershell
python Tests/host/test_debug_ui_build_contract.py
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/ui/run_review_regressions.ps1
```

Expected: build contract and all UI render/task checks pass.

- [ ] **Step 5: Commit the dirty-area slice**

```powershell
git add Ui/debug_ui_astra_layout.inc Tests/host/test_debug_ui_build_contract.py
git commit -m "perf: limit LCD home animation to its dirty region"
```

### Task 4: Publish complete-refresh diagnostics

**Files:**
- Modify: `Ui/lv_port_disp.h`
- Modify: `Ui/lv_port_disp.c`
- Modify: `Ui/debug_ui_view.h`
- Modify: `Ui/debug_ui_task.c`
- Modify: `Ui/debug_ui_view.c`
- Test: `Tests/ui/task_display_regression.c`

- [ ] **Step 1: Define the public diagnostic copy**

```c
typedef struct {
    uint32_t last_bytes;
    uint32_t last_duration_ms;
    uint32_t fps_tenths;
    uint32_t completed;
    uint32_t failed;
} LvPortDispRefreshStatus;

void lv_port_disp_refresh_status(LvPortDispRefreshStatus *status);
```

- [ ] **Step 2: Accumulate one LVGL refresh**

At the first accepted flush, capture `DebugUiNowMs()` and clear pending bytes. Add each block byte count and capture `lv_disp_flush_is_last(driver)`. On successful completion of the last block, publish all fields under an odd/even sequence counter; on failure, increment `failed` and discard the pending refresh.

- [ ] **Step 3: Copy coherent statistics in UiTask**

Use a retrying sequence read in `lv_port_disp_refresh_status()` so task context never observes a partially published ISR update. Copy the result into `DebugUiViewDiagnostics` during the existing 50 ms health block.

- [ ] **Step 4: Display compact frame metrics**

Update the input/display diagnostic page to retain DMA errors and block time while adding complete-refresh bytes, duration and fixed-point FPS:

```c
(void)snprintf(text, sizeof(text), "整帧 %luB  %lums", ...);
(void)snprintf(text, sizeof(text), "有效 %lu.%lu FPS  栈 %lu", ...);
(void)snprintf(text, sizeof(text), "控制 %lu/%lu us", ...);
```

- [ ] **Step 5: Run UI regressions and verify GREEN**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/ui/run_review_regressions.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/ui/run_ui_render_smoke.ps1
```

Expected: complete-refresh assertions pass; all generated page validation checks pass without text overflow.

- [ ] **Step 6: Commit the statistics slice**

```powershell
git add Ui/lv_port_disp.h Ui/lv_port_disp.c Ui/debug_ui_view.h Ui/debug_ui_task.c Ui/debug_ui_view.c Tests/ui/task_display_regression.c
git commit -m "feat: report complete LCD refresh performance"
```

### Task 5: Raise SPI1 to the conservative 10 MHz stage

**Files:**
- Modify: `App/Platform/lcd_st7789.c`
- Test: `Tests/host/lcd_fakes/lcd_fake_hal.h`
- Test: `Tests/host/lcd_fakes/platform_test_main.c`

- [ ] **Step 1: Implement the minimal divider change**

```c
lcd_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
```

Do not change the SPI source clock, CPOL/CPHA, DMA settings, transfer timeout, or panel `C6=0x0F` configuration.

- [ ] **Step 2: Run the LCD platform regression and verify GREEN**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Tests/host/run_lcd_transport_tests.ps1
```

Expected: the divider assertion and all timeout/EOT/cache/stop-order tests pass.

- [ ] **Step 3: Commit the clock slice**

```powershell
git add App/Platform/lcd_st7789.c Tests/host/lcd_fakes/lcd_fake_hal.h Tests/host/lcd_fakes/platform_test_main.c
git commit -m "perf: raise LCD SPI stage to 10 MHz"
```

### Task 6: Full software verification

**Files:**
- Modify only if a test exposes a task-scoped defect.

- [ ] **Step 1: Run the complete host and four-target verification**

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify_debug_ui.ps1 -IncludeKeil
```

Expected: every host check passes and all four Keil targets finish with 0 errors and 0 warnings.

- [ ] **Step 2: Verify scope and repository state**

Run:

```powershell
git diff --check
git status --short
git log --oneline -6
```

Expected: no whitespace errors; only explicitly retained evidence files may remain untracked; commits are limited to the approved LCD performance scope.

- [ ] **Step 3: Report the evidence boundary**

Report software checks and theoretical 10 MHz bandwidth separately from hardware evidence. State explicitly that no firmware was flashed and real-panel stability/FPS remain pending bench validation.
