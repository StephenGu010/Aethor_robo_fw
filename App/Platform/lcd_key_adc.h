/** @file lcd_key_adc.h
 * @brief Private ADC1 PA5/INP19, 12-bit single conversion, 5-ms IT sampler.
 * This driver publishes raw samples only; classification belongs to the UI.
 * Software verified only; actual ADC distributions/physical key mapping untested.
 * Forward ADC_IRQHandler to lcd_key_adc_irq; no duplicate ADC1/MSP owner.
 */
#ifndef LCD_KEY_ADC_H
#define LCD_KEY_ADC_H
#include <stdint.h>
/** IRQ-published raw sample; seq wraps, ms is HAL_GetTick at conversion completion.
 * valid is false before first sample, on error or when age reaches 50 ms.
 */
typedef struct {
    uint16_t raw;
    uint32_t seq, ms;
    uint8_t valid;
} LcdKeyAdcSample;
/** Task-readable acquisition health; no inferred key state. */
typedef struct {
    uint32_t init_errors, start_errors, timeouts, irq_errors, late_irqs;
    uint8_t initialized, conversion_pending;
} LcdKeyAdcStatus;
/** Initialize and perform vendor offset calibration in UiTask, bounded to 10 ms
 * plus a finite poll limit if the HAL tick fails. Return 1 on success.
 */
int lcd_key_adc_init(void);
/** Request at most one conversion per 5 ms; no catch-up bursts. */
void lcd_key_adc_service(uint32_t now_ms);
/** Return latest consistent sample and apply 50-ms validity deadline. */
void lcd_key_adc_latest(uint32_t now_ms, LcdKeyAdcSample *sample);
/** Copy acquisition diagnostics while only ADC_IRQn is masked. */
void lcd_key_adc_get_status(LcdKeyAdcStatus *status);
/** Forward ADC1 interrupt; ISR only publishes raw/seq/time/error state. */
void lcd_key_adc_irq(void);
#endif
