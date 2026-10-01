#include "flash_port_stm32h7.h"

#include "flash_device.h"
#include "octospi.h"

#define FLASH_PORT_TIMEOUT_MS 100U
#define FLASH_PORT_BACKGROUND_REGION MPU_REGION_NUMBER6
#define FLASH_PORT_ACCESS_REGION MPU_REGION_NUMBER7

static uint32_t flash_port_mpu_begin(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  __DMB();
  HAL_MPU_Disable();
  return primask;
}

static void flash_port_mpu_end(uint32_t primask)
{
  HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);
  __DSB();
  __ISB();
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void flash_port_memory_window_protect(void)
{
  MPU_Region_InitTypeDef region  = {0};
  uint32_t               primask = flash_port_mpu_begin();

  region.Enable           = MPU_REGION_ENABLE;
  region.Number           = FLASH_PORT_BACKGROUND_REGION;
  region.BaseAddress      = FLASH_DEVICE_MAPPED_BASE;
  region.Size             = MPU_REGION_SIZE_256MB;
  region.SubRegionDisable = 0U;
  region.TypeExtField     = MPU_TEX_LEVEL0;
  region.AccessPermission = MPU_REGION_NO_ACCESS;
  region.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
  region.IsShareable      = MPU_ACCESS_SHAREABLE;
  region.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
  region.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&region);
  flash_port_mpu_end(primask);
}

void flash_port_prepare(void)
{
  flash_port_memory_window_protect();

  if (hospi2.hmdma != NULL)
  {
    HAL_NVIC_SetPriority(OCTOSPI2_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(OCTOSPI2_IRQn);
  }
}

void OCTOSPI2_IRQHandler(void)
{
  HAL_OSPI_IRQHandler(&hospi2);
}

static void flash_port_init_command(OSPI_RegularCmdTypeDef *command,
                                    uint8_t                 instruction,
                                    uint32_t                data_length)
{
  *command                       = (OSPI_RegularCmdTypeDef) {0};
  command->OperationType         = HAL_OSPI_OPTYPE_COMMON_CFG;
  command->FlashId               = HAL_OSPI_FLASH_ID_1;
  command->Instruction           = instruction;
  command->InstructionMode       = HAL_OSPI_INSTRUCTION_1_LINE;
  command->InstructionSize       = HAL_OSPI_INSTRUCTION_8_BITS;
  command->InstructionDtrMode    = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  command->AddressMode           = HAL_OSPI_ADDRESS_NONE;
  command->AddressSize           = HAL_OSPI_ADDRESS_24_BITS;
  command->AddressDtrMode        = HAL_OSPI_ADDRESS_DTR_DISABLE;
  command->AlternateBytesMode    = HAL_OSPI_ALTERNATE_BYTES_NONE;
  command->AlternateBytesSize    = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  command->AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE;
  command->DataMode              = data_length == 0U ? HAL_OSPI_DATA_NONE : HAL_OSPI_DATA_1_LINE;
  command->NbData                = data_length;
  command->DataDtrMode           = HAL_OSPI_DATA_DTR_DISABLE;
  command->DummyCycles           = 0U;
  command->DQSMode               = HAL_OSPI_DQS_DISABLE;
  command->SIOOMode              = HAL_OSPI_SIOO_INST_EVERY_CMD;
}

int flash_port_command(uint8_t instruction)
{
  OSPI_RegularCmdTypeDef command;

  flash_port_init_command(&command, instruction, 0U);
  return HAL_OSPI_Command(&hospi2, &command, FLASH_PORT_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

int flash_port_receive(uint8_t instruction, void *data, size_t length)
{
  OSPI_RegularCmdTypeDef command;

  if (data == NULL || length == 0U || length > UINT32_MAX)
  {
    return -1;
  }
  flash_port_init_command(&command, instruction, (uint32_t)length);
  if (HAL_OSPI_Command(&hospi2, &command, FLASH_PORT_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }
  return HAL_OSPI_Receive(&hospi2, data, FLASH_PORT_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

int flash_port_transmit(uint8_t instruction, const void *data, size_t length)
{
  OSPI_RegularCmdTypeDef command;

  if (data == NULL || length == 0U || length > UINT32_MAX)
  {
    return -1;
  }
  flash_port_init_command(&command, instruction, (uint32_t)length);
  if (HAL_OSPI_Command(&hospi2, &command, FLASH_PORT_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }
  return HAL_OSPI_Transmit(&hospi2, (uint8_t *)data, FLASH_PORT_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

void flash_port_delay_ms(uint32_t milliseconds)
{
  HAL_Delay(milliseconds);
}

uint32_t flash_port_get_tick_ms(void)
{
  return HAL_GetTick();
}

static void flash_port_init_address_command(OSPI_RegularCmdTypeDef *command,
                                            uint8_t                 instruction,
                                            uint32_t                address,
                                            uint32_t                data_length)
{
  flash_port_init_command(command, instruction, data_length);
  command->Address     = address;
  command->AddressMode = HAL_OSPI_ADDRESS_1_LINE;
  command->AddressSize = HAL_OSPI_ADDRESS_24_BITS;
}

int flash_port_address_command(uint8_t instruction, uint32_t address)
{
  OSPI_RegularCmdTypeDef command;

  flash_port_init_address_command(&command, instruction, address, 0U);
  return HAL_OSPI_Command(&hospi2, &command, FLASH_PORT_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

int flash_port_address_receive(uint8_t instruction, uint32_t address, void *data, size_t length)
{
  OSPI_RegularCmdTypeDef command;

  if (data == NULL || length == 0U || length > UINT32_MAX)
  {
    return -1;
  }
  flash_port_init_address_command(&command, instruction, address, (uint32_t)length);
  if (HAL_OSPI_Command(&hospi2, &command, FLASH_PORT_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }
  return HAL_OSPI_Receive(&hospi2, data, FLASH_PORT_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

int flash_port_address_transmit(uint8_t instruction, uint32_t address, const void *data, size_t length)
{
  OSPI_RegularCmdTypeDef command;

  if (data == NULL || length == 0U || length > UINT32_MAX)
  {
    return -1;
  }
  flash_port_init_address_command(&command, instruction, address, (uint32_t)length);
  if (HAL_OSPI_Command(&hospi2, &command, FLASH_PORT_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }
  return HAL_OSPI_Transmit(&hospi2, (uint8_t *)data, FLASH_PORT_TIMEOUT_MS) == HAL_OK ? 0 : -1;
}

static int flash_port_wait_ready(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (HAL_OSPI_GetState(&hospi2) != HAL_OSPI_STATE_READY)
  {
    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      (void)HAL_OSPI_Abort(&hospi2);
      return -1;
    }
  }
  return HAL_OSPI_GetError(&hospi2) == HAL_OSPI_ERROR_NONE ? 0 : -1;
}

int flash_port_address_receive_mdma(uint8_t instruction, uint32_t address, void *data, size_t length, uint8_t dummy_cycles, flash_port_data_mode_t data_mode)
{
  OSPI_RegularCmdTypeDef command;

  if (data == NULL || length == 0U || length > UINT32_MAX)
  {
    return -1;
  }
  flash_port_init_address_command(&command, instruction, address, (uint32_t)length);
  command.DataMode    = data_mode == FLASH_PORT_DATA_4_LINES ? HAL_OSPI_DATA_4_LINES : HAL_OSPI_DATA_1_LINE;
  command.DummyCycles = dummy_cycles;
  if (HAL_OSPI_Command(&hospi2, &command, FLASH_PORT_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }
  if (hospi2.hmdma == NULL)
  {
    return HAL_OSPI_Receive(&hospi2, data, FLASH_PORT_TIMEOUT_MS) == HAL_OK ? 0 : -1;
  }
  if (HAL_OSPI_Receive_DMA(&hospi2, data) != HAL_OK)
  {
    return -1;
  }
  return flash_port_wait_ready(FLASH_PORT_TIMEOUT_MS);
}

int flash_port_address_transmit_mdma(uint8_t instruction, uint32_t address, const void *data, size_t length, flash_port_data_mode_t data_mode)
{
  OSPI_RegularCmdTypeDef command;

  if (data == NULL || length == 0U || length > UINT32_MAX)
  {
    return -1;
  }
  flash_port_init_address_command(&command, instruction, address, (uint32_t)length);
  command.DataMode = data_mode == FLASH_PORT_DATA_4_LINES ? HAL_OSPI_DATA_4_LINES : HAL_OSPI_DATA_1_LINE;
  if (HAL_OSPI_Command(&hospi2, &command, FLASH_PORT_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }
  if (hospi2.hmdma == NULL)
  {
    return HAL_OSPI_Transmit(&hospi2, (uint8_t *)data, FLASH_PORT_TIMEOUT_MS) == HAL_OK ? 0 : -1;
  }
  if (HAL_OSPI_Transmit_DMA(&hospi2, (uint8_t *)data) != HAL_OK)
  {
    return -1;
  }
  return flash_port_wait_ready(FLASH_PORT_TIMEOUT_MS);
}

static int flash_port_memory_mapped_configure(uint8_t  read_instruction,
                                              uint8_t  write_instruction,
                                              uint8_t  dummy_cycles,
                                              uint32_t data_mode)
{
  OSPI_RegularCmdTypeDef   read_command;
  OSPI_RegularCmdTypeDef   write_command;
  OSPI_MemoryMappedTypeDef config = {0};

  flash_port_init_address_command(&read_command, read_instruction, 0U, 1U);
  read_command.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
  read_command.DataMode      = data_mode;
  read_command.DummyCycles   = dummy_cycles;
  if (HAL_OSPI_Command(&hospi2, &read_command, FLASH_PORT_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }

  flash_port_init_address_command(&write_command, write_instruction, 0U, 1U);
  write_command.OperationType = HAL_OSPI_OPTYPE_WRITE_CFG;
  write_command.DataMode      = data_mode;
  if (HAL_OSPI_Command(&hospi2, &write_command, FLASH_PORT_TIMEOUT_MS) != HAL_OK)
  {
    return -1;
  }
  config.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
  config.TimeOutPeriod     = 0U;
  return HAL_OSPI_MemoryMapped(&hospi2, &config) == HAL_OK ? 0 : -1;
}

int flash_port_memory_mapped_enable(uint8_t read_instruction,
                                    uint8_t write_instruction,
                                    uint8_t dummy_cycles)
{
  return flash_port_memory_mapped_configure(read_instruction,
                                            write_instruction,
                                            dummy_cycles,
                                            HAL_OSPI_DATA_4_LINES);
}

int flash_port_memory_mapped_spi_enable(uint8_t read_instruction,
                                        uint8_t write_instruction)
{
  return flash_port_memory_mapped_configure(read_instruction,
                                            write_instruction,
                                            0U,
                                            HAL_OSPI_DATA_1_LINE);
}

int flash_port_memory_mapped_disable(void)
{
  return HAL_OSPI_Abort(&hospi2) == HAL_OK ? 0 : -1;
}

int flash_port_memory_mapped_access_enable(int executable)
{
  MPU_Region_InitTypeDef region  = {0};
  uint32_t               primask = flash_port_mpu_begin();

  region.Enable           = MPU_REGION_ENABLE;
  region.Number           = FLASH_PORT_ACCESS_REGION;
  region.BaseAddress      = FLASH_DEVICE_MAPPED_BASE;
  region.Size             = MPU_REGION_SIZE_8MB;
  region.SubRegionDisable = 0U;
  region.TypeExtField     = MPU_TEX_LEVEL0;
  region.AccessPermission = MPU_REGION_PRIV_RO_URO;
  region.DisableExec      = executable != 0 ? MPU_INSTRUCTION_ACCESS_ENABLE : MPU_INSTRUCTION_ACCESS_DISABLE;
  region.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
  region.IsCacheable      = MPU_ACCESS_CACHEABLE;
  region.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&region);
  flash_port_mpu_end(primask);
  return 0;
}

void flash_port_memory_mapped_access_disable(void)
{
  MPU_Region_InitTypeDef region  = {0};
  uint32_t               primask = flash_port_mpu_begin();

  region.Enable = MPU_REGION_DISABLE;
  region.Number = FLASH_PORT_ACCESS_REGION;
  HAL_MPU_ConfigRegion(&region);
  flash_port_mpu_end(primask);
}
