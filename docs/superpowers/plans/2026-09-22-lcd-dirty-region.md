# LCD Dirty Region Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop unchanged or text-only LCD view updates from forcing a 280×240 refresh.

**Architecture:** Keep the shared LVGL object tree and existing model-to-text mapping. Make repeated object configuration idempotent, apply visibility once per layout, and invalidate only root-drawn regions whose retained state actually changed.

**Tech Stack:** C99, LVGL 8.3.11, STM32H723, PowerShell host tests, Keil MDK.

---

### Task 1: Lock the pixel-transfer contract

**Files:**
- Modify: `Tests/ui/ui_render_smoke.c`

- [x] Add a function-level-commented regression that settles the overview page, repeats an identical update, and requires zero additional `framebuffer_writes`.
- [x] In the same regression, toggle one overview status string and require a positive pixel count below `280 * 240`.
- [x] Run `powershell -NoProfile -ExecutionPolicy Bypass -File Tests/ui/run_ui_render_smoke.ps1` and verify it fails on the unchanged-update assertion because the current layout invalidates the root.

### Task 2: Make repeated layout application idempotent

**Files:**
- Modify: `Ui/debug_ui_view.h`
- Modify: `Ui/debug_ui_view.c`
- Modify: `Ui/debug_ui_astra_layout.inc`

- [x] Add bounded per-layout visibility masks to `DebugUiViewPage`.
- [x] Change visibility and style helpers so invalidating LVGL setters run only when the requested value differs; retain LVGL's own local-value checks for position and size.
- [x] Accumulate desired row/icon/overlay/confirmation visibility and apply it once at the end of `layout()`.
- [x] Replace unconditional root invalidation with page/template and bounded custom-draw invalidation.
- [x] Run the focused render regression and require the new pixel-transfer assertions to pass: unchanged `0`, one overview row `11592`, full screen `67200` pixels.

### Task 3: Verify the complete firmware scope

**Files:**
- Modify only if a task-scoped regression exposes a defect.

- [x] Run `powershell -NoProfile -ExecutionPolicy Bypass -File Tests/ui/run_review_regressions.ps1`.
- [x] Run `powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify_debug_ui.ps1 -IncludeKeil`: 28 checks passed; all four Keil targets reported 0 errors and 0 warnings.
- [x] Run `git diff --check` and inspect `git status --short` plus the final diff.
- [x] Commit the implementation locally without pushing, creating a pull request, or flashing hardware.
