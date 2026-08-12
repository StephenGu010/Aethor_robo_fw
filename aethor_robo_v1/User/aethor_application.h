/**
 * @file aethor_application.h
 * @brief Hardware integration entry points for USB CDC and PA15 dual-motor control.
 */

#ifndef AETHOR_APPLICATION_H
#define AETHOR_APPLICATION_H

#ifdef __cplusplus
extern "C" {
#endif

int aethor_application_init(void);
void aethor_application_service(void);

#ifdef __cplusplus
}
#endif

#endif /* AETHOR_APPLICATION_H */
