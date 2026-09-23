#include "status.hpp"
#include "FreeRTOSConfig.h"

osEventFlagsId_t sysEvent = NULL;
const osEventFlagsAttr_t sysEvent_Attr = {
    .name = "SysEvent"
};

Status SysFlagInit(void)
{
    sysEvent = osEventFlagsNew(&sysEvent_Attr);
    if (sysEvent == NULL)
    {
        return Status::FULL;
    }
    osEventFlagsSet(sysEvent, (1U << (uint8_t)Status::OK));
    return Status::OK;
}

Status SysFlagSet(Status statu)
{
    if (sysEvent == NULL)
    {
        return Status::NOT_INIT;
    }
    if (statu == Status::OK)
    {
        return Status::BAD_ARG;
    }
    osEventFlagsSet(sysEvent, (1U << (uint8_t)statu));
    osEventFlagsClear(sysEvent, (1U << (uint8_t)Status::OK));
    return Status::OK;
}

void SysCompleteInit(void)
{
    if ( (osEventFlagsGet(sysEvent) & SYS_FLAG_INIT_FAIL_BIT) == 0 )
    {
        osEventFlagsSet(sysEvent, SYS_FLAG_RUNNING_BIT);
    }
}

void SysInitError(void)
{
    osEventFlagsSet(sysEvent, SYS_FLAG_INIT_FAIL_BIT);
    osEventFlagsClear(sysEvent, SYS_FLAG_RUNNING_BIT);
}

Status SysInitError(Status statu)
{
    SysInitError();
    return SysFlagSet(statu);
}

uint32_t SysFlagWait(Status statu, uint32_t timeout)
{
    return osEventFlagsWait(sysEvent, (1U << (uint8_t)statu), osFlagsWaitAll | osFlagsNoClear, timeout);
}

void SysFlagWaitRunning(void)
{
    if (sysEvent == NULL)   // 连事件本身都挂了那就别运行了，当然这个可能性还是太低了
    {
        osThreadExit();
    }

    const uint32_t flags = osEventFlagsWait(sysEvent,
                                            SYS_FLAG_RUNNING_BIT,
                                            osFlagsWaitAll | osFlagsNoClear,
                                            osWaitForever);
    if ((flags & osFlagsError) != 0U)
    {
        osThreadExit();
    }
}
