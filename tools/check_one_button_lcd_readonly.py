"""Read the running one-button LCD model using the pinned release and its own AXF offsets.

Default execution verifies offline only. --read delegates the previously checked
attach-only reader: no halt, reset, resume, RAM writes or motor commands.
"""
import check_engineering_lcd_readonly as lcd_reader
import deploy_one_button_poweroff as release


def main():
    """Bind exact release identity and linker evidence before the original reader parses arguments."""
    lcd_reader.release = release
    lcd_reader.EVIDENCE = release.BUILD
    lcd_reader.main()


if __name__ == '__main__':
    main()
