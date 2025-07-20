#include "FreeRTOS.h"
#include "task.h"

#ifndef configUSE_TRACE_FACILITY
    #warning "configUSE_TRACE_FACILITY must be defined to use trace facility"
#endif

uint32_t *vTaskStackAddr(void) {
#if ( configUSE_TRACE_FACILITY == 1 )
    TaskStatus_t status;
    vTaskGetInfo(NULL, &status, pdFALSE, eInvalid); // NULL表示当前任务
    return (uint32_t *)status.pxStackBase;
#else
    return NULL; // 如果没有启用跟踪设施，则返回NULL
#endif
}

uint32_t vTaskStackSize(void) {
#if ( configUSE_TRACE_FACILITY == 1 )
    TaskStatus_t status;
    vTaskGetInfo(NULL, &status, pdFALSE, eInvalid);
    return status.usStackHighWaterMark; // 注意：这是剩余栈空间，不是总栈大小
    // 若要总栈大小，需用 status.usStackDepth 或 status.usStackSize（不同版本成员名不同）
#else
    return 0; // 如果没有启用跟踪设施，则返回0
#endif
}

char *vTaskName(void) {
#if ( configUSE_TRACE_FACILITY == 1 )
    TaskStatus_t status;
    vTaskGetInfo(NULL, &status, pdFALSE, eInvalid);
    return (char *)status.pcTaskName;
#else
    return NULL; // 如果没有启用跟踪设施，则返回NULL
#endif
}