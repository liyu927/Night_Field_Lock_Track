#ifndef SOLAR_LIGHT_TRACKING_SYSTEM_W25Q64_H
#define SOLAR_LIGHT_TRACKING_SYSTEM_W25Q64_H

#include "main.h"

/* 8 MiB 总容量；末尾两个 4 KB 扇区专放参数，其余空间可留给固件。 */
#define W25Q64_CAPACITY_BYTES       0x800000UL//8 MiB
#define W25Q64_SECTOR_BYTES         4096UL//4 KB 擦除单位(扇区)
#define W25Q64_PAGE_BYTES           256UL//页编程上限
#define W25Q64_PARAMETER_MAX_BYTES  (W25Q64_SECTOR_BYTES - 16UL)//	4096 − 16（去掉包头）
#define W25Q64_FIRMWARE_END         (W25Q64_CAPACITY_BYTES - 2UL * W25Q64_SECTOR_BYTES)//0x800000 − 2×4096

/* SPI2 和 DMA 初始化完成后调用；检查芯片 JEDEC ID。 */
HAL_StatusTypeDef W25Q64_Init(void);
/* 原始读写供后续固件缓存使用；擦写不允许覆盖末尾两个参数扇区。写入前须擦除。 */
//从任意地址读任意长度（可跨页，芯片内部地址自动递增）
HAL_StatusTypeDef W25Q64_Read(uint32_t address, void *data, uint32_t length);
//W25Q64_Program：页编程，内部会自动分页、写使能、等待完成。
HAL_StatusTypeDef W25Q64_Program(uint32_t address, const void *data, uint32_t length);
//擦除一个 4 KB 扇区，地址必须 4096 对齐。
HAL_StatusTypeDef W25Q64_EraseSector(uint32_t address);
/* 两个 4 KB 扇区各存一份完整参数包，含序号和 CRC32。 */
/* 读取返回 HAL_BUSY 表示两份均无效；保存成功表示两份均已校验。 */
//把参数包读进 data，内部完成选新、补写备份。
HAL_StatusTypeDef W25Q64_LoadParameters(void *data, uint32_t length);
//把参数包写入两个扇区，两份都校验通过才返回 OK。
HAL_StatusTypeDef W25Q64_SaveParameters(const void *data, uint32_t length);

#endif
