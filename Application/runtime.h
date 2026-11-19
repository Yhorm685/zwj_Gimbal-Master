#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void Gimbal_CreateQueues(void);
void Gimbal_Comm(void);
void StartControl(void *argument);
#ifdef __cplusplus
}
#endif
