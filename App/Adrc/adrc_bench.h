/** @file adrc_bench.h
 * @brief HAL-independent single-owner bridge from bounded protocol mailboxes to experiment supervision.
 * Caller allocates this context statically. All APIs, gateway callbacks, result publication and trace
 * reads must share the application's critical hooks; no concurrent supervisor access is permitted.
 * Inputs are coherent selected-axis output-shaft SI feedback; this bridge performs no scaling or I/O.
 */
#ifndef APP_ADRC_BENCH_H
#define APP_ADRC_BENCH_H

#include "adrc_protocol.h"

/** @brief Holds one completion until the reserved protocol channel accepts it. */
typedef struct
{
    uint32_t request_id;
    uint64_t timestamp_us;
    AdrcExperimentResult result;
} AdrcBenchTerminal;

/** @brief Caller-owned fixed storage; trusted qualification can only enter through local evidence API. */
typedef struct
{
    AdrcExperiment experiment;
    AdrcProtocolGateway gateway;
    AdrcExperimentConfig draft_config;
    AdrcExperimentQualification trusted_qualification;
    /** @brief Draft position, velocity and torque scales; changes revoke all measured evidence. */
    float mapping[3];
    AdrcBenchTerminal pending_terminal[3];
    uint32_t run_request_id;
    /** @brief Sticky history prevents revoked qualification from recreating the first-boot diagnostic exception. */
    uint8_t evidence_installed;
    uint8_t initialized;
} AdrcBench;

/** @brief Initializes conservative defaults, unit draft scales and zero qualification; never drives. */
AdrcExperimentResult adrc_bench_init(AdrcBench *bench, AdrcExperimentEnvironment environment,
    AdrcControllerCallback controller, void *controller_context);
/** @brief Returns the gateway for protocol_engine_set_adrc_handler under caller serialization. */
AdrcProtocolGateway *adrc_bench_gateway(AdrcBench *bench);
/** @brief Installs trusted local measured evidence only while safely disabled, never from USB fields. */
AdrcExperimentResult adrc_bench_set_evidence(AdrcBench *bench, const AdrcExperimentConfig *config,
    const AdrcExperimentQualification *qualification, const AdrcExperimentInput *input);
/** @brief Consumes at most three priority events and advances control; caller delivers actual send receipts. */
AdrcExperimentResult adrc_bench_service(AdrcBench *bench, const AdrcExperimentInput *input,
    AdrcExperimentOutput *output);

#endif
