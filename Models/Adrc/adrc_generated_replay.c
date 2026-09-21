/** @file adrc_generated_replay.c
 * @brief Offline replay harness; all control arithmetic comes from generated C.
 * Input CSV has nine controller inputs followed by three Simulink outputs.
 */
#include "adrc_controller.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/** @brief Replay every sample and reject normalized disagreement over 1e-4. */
int main(int argument_count, char **arguments)
{
    FILE *vectors;
    FILE *report;
    float fields[12];
    double maximum_error = 0.0;
    unsigned long samples = 0UL;
    int parsed_fields;
    int output_index;
    if (argument_count != 3) return 2;
    vectors = fopen(arguments[1], "r");
    if (vectors == NULL) return 3;
    adrc_controller_initialize();
    for (;;) {
        parsed_fields = fscanf(vectors,
            "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
            &fields[0], &fields[1], &fields[2], &fields[3], &fields[4],
            &fields[5], &fields[6], &fields[7], &fields[8], &fields[9],
            &fields[10], &fields[11]);
        if (parsed_fields == EOF) break;
        if (parsed_fields != 12) { fclose(vectors); return 4; }
        adrc_controller_U.mode = (uint8_T)fields[0];
        adrc_controller_U.reference_rad_s = fields[1];
        adrc_controller_U.velocity_rad_s = fields[2];
        adrc_controller_U.prev_sent_nm = fields[3];
        adrc_controller_U.b0 = fields[4];
        adrc_controller_U.wc_rad_s = fields[5];
        adrc_controller_U.wo_rad_s = fields[6];
        adrc_controller_U.reset = (uint8_T)fields[7];
        adrc_controller_U.new_sample = (uint8_T)fields[8];
        adrc_controller_step();
        for (output_index = 0; output_index < 3; ++output_index) {
            double actual = output_index == 0 ? adrc_controller_Y.torque_raw_nm :
                (output_index == 1 ? adrc_controller_Y.z1 : adrc_controller_Y.z2);
            double expected = fields[9 + output_index];
            double normalized_error;
            if (!isfinite(actual) || !isfinite(expected)) {
                fclose(vectors); return 5;
            }
            normalized_error = fabs(actual - expected) / fmax(1.0, fabs(expected));
            if (normalized_error > maximum_error) maximum_error = normalized_error;
        }
        ++samples;
    }
    fclose(vectors);
    report = fopen(arguments[2], "w");
    if (report == NULL) return 6;
    fprintf(report, "{\"samples\":%lu,\"max_normalized_error\":%.17g,\"tolerance\":0.0001,\"passed\":%s}\n",
        samples, maximum_error, samples > 0UL && maximum_error <= 1e-4 ? "true" : "false");
    fclose(report);
    return samples > 0UL && maximum_error <= 1e-4 ? 0 : 7;
}
