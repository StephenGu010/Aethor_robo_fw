"""Verify real LVGL PPM output and convert it to reviewable PNG files.

This is deterministic file-format conversion of actual raster output. It also
checks the versioned font inventory against every current UI/model C literal.
"""
from pathlib import Path
import hashlib
import io
import json
import os
import re
import sys
import time
from PIL import Image, ImageDraw


def save_png_if_changed(raster: Image.Image, path: Path, source_modified_ns: int) -> None:
    """Keep exact PNG bytes and bind freshness to the rendered PPM, with bounded Windows lock retries."""
    encoded = io.BytesIO()
    raster.save(encoded, format="PNG")
    payload = encoded.getvalue()
    unchanged = path.exists() and path.read_bytes() == payload
    for attempt in range(3):
        try:
            if not unchanged:
                path.write_bytes(payload)
            # Even unchanged pixels must belong to this PPM render, never the verification wall clock.
            os.utime(path, ns=(source_modified_ns, source_modified_ns))
            return
        except OSError as error:
            if attempt == 2 or error.errno not in (13, 22):
                raise
            time.sleep(0.1)


def save_contact_sheets(directory: Path, images: list[dict]) -> None:
    """Compose current verified rasters into readable, numbered review sheets."""
    preferred = ["01_", "02_", "03_", "04_", "05_", "06_", "16_", "07_", "08_",
                 "09_", "11_", "13_", "14_", "24_", "25_", "33_", "69_", "76_",
                 "90_", "93_", "94_", "98_", "112_", "116_", "117_", "122_",
                 "132_", "136_", "137_", "142_", "145_", "149_", "150_", "154_",
                 "163_", "167_", "170_", "178_", "210_", "212_", "213_", "216_",
                 "63_", "78_", "82_", "95_", "172_", "211_", "178_lcd_",
                 "220_", "225_", "226_", "262_", "263_", "266_", "267_", "270_", "271_", "272_", "273_", "274_"]
    selected = [item for prefix in preferred for item in images if item["image"].startswith(prefix)]
    groups = [("contact_sheet_mono_overview.png", selected)]
    groups += [(f"contact_sheet_mono_all_{first // 60 + 1}.png", images[first:first + 60])
               for first in range(0, len(images), 60)]
    for filename, group in groups:
        rows = (len(group) + 5) // 6
        sheet = Image.new("RGB", (6 * 288, rows * 266), "#303030")
        drawing = ImageDraw.Draw(sheet)
        for index, item in enumerate(group):
            left, top = index % 6 * 288 + 4, index // 6 * 266 + 4
            with Image.open(directory / item["image"]) as raster:
                sheet.paste(raster, (left, top))
            drawing.text((left, top + 242), item["image"][:42], fill="white")
        source_modified_ns = max((directory / item["image"]).stat().st_mtime_ns for item in group)
        save_png_if_changed(sheet, directory / filename, source_modified_ns)


def verify(directory: Path) -> None:
    """Check image size/content/uniqueness and every source Unicode codepoint."""
    root = Path(__file__).resolve().parents[2]
    inventory = set((root / "Ui/fonts/charset.txt").read_text(encoding="utf-8"))
    for source in (root / "Ui/debug_ui_view.c", root / "Ui/debug_ui_astra_layout.inc", root / "Ui/debug_ui_static_icons.inc",
                   root / "Ui/debug_ui_mono_layout.inc",
                   root / "Ui/debug_ui_load_trial_layout.inc", root / "App/DebugUi/debug_ui_load_trial_pages.inc",
                   root / "Ui/debug_ui_pos_load_layout.inc",
                   root / "App/DebugUi/debug_ui_model.c"):
        for literal in re.findall(r'"(?:\\.|[^"\\])*"', source.read_text(encoding="utf-8")):
            missing = {character for character in literal if ord(character) > 127 and character not in inventory}
            assert not missing, f"{source}: missing font inventory {missing}"
    manifest = []
    hashes = set()
    monochrome_static_pages = 0
    maximum_static_channel_spread = 0
    for path in sorted(directory.glob("*.ppm")):
        with Image.open(path) as raster:
            assert raster.size == (280, 240)
            assert len(raster.getcolors(280 * 240) or []) > 15, f"Blank/limited raster: {path}"
            spread = max(max(pixel)-min(pixel) for pixel in raster.get_flattened_data())
            assert spread <= 8, f"Chromatic LCD raster: {path}: spread={spread}"
            maximum_static_channel_spread = max(maximum_static_channel_spread, spread)
            monochrome_static_pages += 1
            digest = hashlib.sha256(raster.tobytes()).hexdigest()
            assert digest not in hashes, f"Duplicate rendered page: {path}"
            hashes.add(digest)
            png = path.with_suffix(".png")
            save_png_if_changed(raster, png, path.stat().st_mtime_ns)
            manifest.append({"image": png.name, "rgb_sha256": digest, "width": 280, "height": 240})
    required = {"17_review_set_pos.png", "18_review_set_mit.png", "19_remote_id3.png",
                "20_stop_latched.png", "21_stop_latch_confirmed.png", "22_stop_latch_unconfirmed.png",
                "23_remote_multiple_targets.png",
                "90_static_default_locked.png", "91_static_seven_fresh.png", "92_static_one_stale.png",
                "93_static_actions.png", "94_static_duration.png", "95_static_duration_number.png",
                "96_static_review.png", "97_static_running.png", "98_static_result_unconfirmed.png",
                "99_static_result_confirmed.png", "100_static_result_temperatures.png",
                "101_static_result_samples.png", "102_static_result_missing_axis.png",
                "103_static_failed_partial.png", "104_static_stopped_confirmed.png",
                "105_static_old_report_hidden.png", "106_static_explicit_history.png",
                "107_static_bound_stop_identity.png", "108_static_unrelated_stop_reason_hidden.png",
                "109_static_fallback_stop_confirmed.png", "110_static_confirmed_control_locked.png",
                "111_static_bound_stop_failed.png",
                "112_single_default_locked.png", "113_single_unconfigured.png",
                "114_single_monitor_id3.png", "115_single_stale_id3.png",
                "116_single_actions.png", "117_single_select_id7.png",
                "118_single_duration_60s.png", "119_single_review.png",
                "120_single_running.png", "121_single_stop_pending.png",
                "122_single_result_unconfirmed.png", "123_single_result_confirmed.png",
                "124_single_wrong_mode_hidden.png", "125_single_wrong_id_hidden.png",
                "126_single_explicit_id4_history.png", "127_single_missing_sample_control_locked.png",
                "128_single_lock_not_seven_preview.png", "129_single_invalid_history_mask.png",
                "130_single_lock_not_seven_actions.png",
                "131_home_static_launcher.png", "132_static_hub_single.png", "133_static_hub_seven.png",
                "134_lcd_default_locked.png", "136_lcd_setup_id7_60s.png", "137_lcd_config_review.png",
                "138_lcd_config_pending.png", "139_lcd_config_snapshot_pending.png", "140_lcd_config_failure.png",
                "141_lcd_ready_remote.png", "142_lcd_ready_capture.png", "143_lcd_ready_locked.png",
                "144_lcd_hold_review.png", "145_lcd_running.png", "146_lcd_running_unknown.png",
                "147_lcd_stop_pending.png", "148_lcd_result_unconfirmed.png", "149_lcd_result_confirmed.png",
                "150_lcd_relock_pending.png", "151_lcd_relock_failure.png", "152_lcd_acquire_review.png",
                "153_lcd_cleanup_failure.png", "154_lcd_old_target_handoff.png"}
    required.update(f"135_lcd_setup_id{motor_id}.png" for motor_id in range(1, 8))
    required.update({"155_lcd_ready_stale_feedback.png", "156_lcd_ready_enabled.png",
                     "157_lcd_ready_wrong_mode.png", "158_lcd_relock_retry.png",
                     "159_lcd_running_small_error.png", "160_lcd_ready_unverified_mode.png",
                     "161_lcd_ready_other_target.png", "162_lcd_ready_checks_complete.png"})
    required.update({"163_load_default_250mm_unconfirmed.png", "164_load_250mm_100g_confirmed.png",
                     "165_load_minimum_unconfirmed.png", "166_load_maximum_unconfirmed.png",
                     "167_load_fine_lever_boundary.png", "168_load_fine_mass_boundary.png",
                     "169_load_history_empty.png", "170_load_history_latest_16.png", "171_load_history_last_page.png",
                     "172_load_hold_failed_stop_separate.png", "173_load_measurements_disable_unconfirmed.png",
                     "174_load_old_id4_without_report.png", "175_load_without_report_unknown.png",
                     "176_load_running_unknown_metadata.png", "177_load_review_frozen_100g.png",
                     "178_load_complete_measurements.png", "179_load_result_frozen_summary.png"})
    required.update({"180_load_failed_single_confirmed.png", "181_load_failed_single_unfilled.png",
                     "182_load_failed_result_confirmed.png", "183_load_failed_fault_unfilled.png",
                     "184_load_lifecycle_without_hold.png", "185_load_timer_wrong_axis.png",
                     "186_load_timer_wrong_duration.png", "187_load_timer_old_identity.png",
                     "188_load_timer_old_start.png", "189_load_timer_stale_snapshot.png",
                     "190_load_timer_cross_operation.png", "191_load_timer_other_boot.png",
                     "192_load_timer_future_snapshot.png", "193_load_timer_matched_zero_samples.png"})
    required.update({"194_load_timer_regressive_elapsed.png", "195_load_timer_duplicate_changed_elapsed.png",
                     "196_load_timer_finalized_reactivation.png", "197_load_timer_duplicate_accepted.png"})
    required.update({"210_pos_load_setup.png", "211_pos_load_number.png", "212_pos_load_review.png",
                     "213_pos_load_running.png", "214_pos_load_recovered.png", "215_pos_load_no_disturbance.png",
                     "216_pos_load_peaks.png", "217_pos_load_recovery_timeout.png", "218_pos_load_old_report_hidden.png",
                     "219_pos_load_default_locked.png"})
    required.update({"251_pos_digit_sign.png", "252_pos_digit_hundreds.png", "253_pos_digit_tenths.png", "254_pos_digit_seconds.png"})
    required.update({"220_pos_focus_angle.png", "221_pos_focus_speed.png", "222_pos_focus_time.png",
                     "223_pos_focus_record.png", "224_pos_focus_start.png", "225_pos_focus_control.png",
                     "226_pos_parameters_large.png", "227_pos_parameters_minimum.png", "228_pos_review_released.png",
                     "229_pos_stop_pending.png", "230_pos_stopped_confirmed.png", "231_pos_stopped_disable_unknown.png",
                     "232_pos_nonfinite_report_hidden.png", "233_pos_peak_boundaries.png", "234_pos_terminal_waiting.png",
                     "235_pos_prepare_owned.png", "236_pos_prepare_stop_required.png", "237_pos_prepare_stopping.png", "238_pos_prepare_other_axis.png",
                     "300_pos_cleanup_latch_check.png", "301_pos_cleanup_result_check.png",
                     "302_pos_flange_negative180.png", "303_pos_flange_driver_blocked.png",
                     "304_pos_flange_angle_number.png", "305_pos_flange_review.png",
                     "306_pos_flange_running_relative.png", "307_pos_flange_recovered.png",
                     "308_pos_flange_result_detail.png"})
    required.update({"262_mono_actions_stale_unverified.png", "263_mono_detail_invalid.png",
                     "264_mono_diagnostic_tab_0.png", "265_mono_diagnostic_tab_1.png",
                     "266_mono_diagnostic_tab_2.png", "267_mono_detail_fresh_extreme.png",
                     "268_mono_motors_unverified_mode.png", "269_mono_detail_unverified_mode.png",
                     "270_mono_actions_fresh_summary.png", "271_mono_edit_large_parameters.png",
                     "272_mono_detail_snapshot_expired.png", "273_mono_detail_temperatures_unknown.png",
                     "274_mono_detail_temperatures_extreme.png"})
    family_coverage = dict(zip([
        "OVERVIEW", "MOTORS", "DETAIL", "DIAGNOSTICS", "EDIT", "REVIEW", "RUNNING", "RESULT",
        "FAULT", "PREPARE", "REGISTERS", "ACTIONS", "NUMBER", "MODES", "RECOVERY", "DIAGNOSTIC_MENU",
        "NOTICE", "STATIC_LOAD", "STATIC_ACTIONS", "STATIC_RESULT", "SINGLE_STATIC_LOAD",
        "SINGLE_STATIC_ACTIONS", "SINGLE_STATIC_RESULT", "SINGLE_STATIC_IDS", "STATIC_HUB",
        "SINGLE_STATIC_SETUP", "SINGLE_STATIC_CONFIG_REVIEW", "SINGLE_STATIC_READY", "SINGLE_STATIC_RELOCK",
        "LOAD_METADATA", "LOAD_NUMBER", "LOAD_HISTORY", "LOAD_DETAIL", "SINGLE_STATIC_PREFLIGHT",
        "POS_LOAD_SETUP", "POS_LOAD_NUMBER", "POS_LOAD_RESULT"
    ], [
        "01_overview.png", "02_motors_scrolled.png", "03_motor_detail.png", "05_diagnostics.png",
        "08_mit_draft.png", "09_review_release.png", "11_running.png", "13_result.png", "14_fault.png",
        "24_control_prepare.png", "25_position_registers.png", "32_actions_row_0.png",
        "95_static_duration_number.png", "63_modes_select_pos.png", "69_recovery_row_2.png",
        "82_diagnostic_menu_0.png", "78_notice_out_of_range.png", "90_static_default_locked.png",
        "93_static_actions.png", "98_static_result_unconfirmed.png", "112_single_default_locked.png",
        "116_single_actions.png", "122_single_result_unconfirmed.png", "117_single_select_id7.png",
        "132_static_hub_single.png", "136_lcd_setup_id7_60s.png", "137_lcd_config_review.png",
        "142_lcd_ready_capture.png", "150_lcd_relock_pending.png", "163_load_default_250mm_unconfirmed.png",
        "167_load_fine_lever_boundary.png", "170_load_history_latest_16.png", "172_load_hold_failed_stop_separate.png",
        "178_lcd_preflight_pending.png", "210_pos_load_setup.png", "211_pos_load_number.png", "216_pos_load_peaks.png"
    ]))
    enum_names = set(re.findall(r"DEBUG_UI_PAGE_([A-Z_]+)",
                     (root / "App/DebugUi/debug_ui_model.h").read_text(encoding="utf-8"))) - {"COUNT"}
    assert len(family_coverage) == 37 and set(family_coverage) == enum_names, "Incomplete LCD enum family coverage"
    required.update(family_coverage.values())
    missing_renders = required - {item["image"] for item in manifest}
    assert not missing_renders, f"Missing review-regression renders: {sorted(missing_renders)}"
    assert len(manifest) >= 23
    save_contact_sheets(directory, manifest)
    (directory / "render_manifest.json").write_text(json.dumps({
        "renderer": "actual LVGL 8.3.11 software raster + production double-buffer flush port",
        "data": "SIMULATED ONLY; no hardware validation", "images": manifest,
        "all_current_literal_codepoints_in_font_inventory": True,
        "monochrome_static_pages": monochrome_static_pages,
        "maximum_static_rgb565_channel_spread": maximum_static_channel_spread,
        "page_enum_coverage": family_coverage,
    }, indent=2) + "\n", encoding="utf-8")
    print(f"RENDER_ARTIFACTS_OK: {len(manifest)} distinct 280x240 pages; current Unicode inventory complete")


if __name__ == "__main__":
    verify(Path(sys.argv[1]))
