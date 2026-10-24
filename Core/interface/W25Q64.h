#ifndef SOLAR_LIGHT_TRACKING_SYSTEM_W25Q64_H
#define SOLAR_LIGHT_TRACKING_SYSTEM_W25Q64_H

#include "main.h"

/* 8 MiB 总容量；末尾两个 4 KB 扇区专放参数，其余空间可留给固件。 */
#define W25Q64_CAPACITY_BYTES       0x800000UL
#define W25Q64_SECTOR_BYTES         4096UL
#define W25Q64_PAGE_BYTES           256UL
#define W25Q64_PARAMETER_MAX_BYTES  (W25Q64_SECTOR_BYTES - 16UL)
#define W25Q64_FIRMWARE_END         (W25Q64_CAPACITY_BYTES - 2UL * W25Q64_SECTOR_BYTES)

/* SPI2 和 DMA 初始化完成后调用；检查芯片 JEDEC ID。 */
HAL_StatusTypeDef W25Q64_Init(void);
/* 原始读写供后续固件缓存使用；擦写不允许覆盖末尾两个参数扇区。写入前须擦除。 */
HAL_StatusTypeDef W25Q64_Read(uint32_t address, void *data, uint32_t length);
HAL_StatusTypeDef W25Q64_Program(uint32_t address, const void *data, uint32_t length);
HAL_StatusTypeDef W25Q64_EraseSector(uint32_t address);
/* 两个 4 KB 扇区各存一份完整参数包，含序号和 CRC32。 */
/* 读取返回 HAL_BUSY 表示两份均无效；保存成功表示两份均已校验。 */
HAL_StatusTypeDef W25Q64_LoadParameters(void *data, uint32_t length);
HAL_StatusTypeDef W25Q64_SaveParameters(const void *data, uint32_t length);

#endif
