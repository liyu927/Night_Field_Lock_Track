#ifndef NIGHT_FIELD_LOCK_TRACK_W25Q64_H
#define NIGHT_FIELD_LOCK_TRACK_W25Q64_H

#include "main.h"

/* 8 MiB 总容量；末尾五个 4 KB 扇区依次为用户 A、用户 B、出厂、电机 A、电机 B。 */
#define W25Q64_CAPACITY_BYTES       0x800000UL /* 芯片总字节数。 */
#define W25Q64_SECTOR_BYTES         4096UL     /* 最小擦除单位，一个参数槽占一扇区。 */
#define W25Q64_PAGE_BYTES           256UL      /* 单次页编程的最大字节数。 */
#define W25Q64_PARAMETER_MAX_BYTES  (W25Q64_SECTOR_BYTES - 16UL) /* 去掉 16 字节记录头后的最大载荷。 */
/* 固件缓存可用区的结束地址，原始编程和擦除接口不得写入后五个参数扇区。 */
#define W25Q64_FIRMWARE_END         (W25Q64_CAPACITY_BYTES - 5UL * W25Q64_SECTOR_BYTES)

/* SPI2 和 DMA 初始化完成后调用；检查芯片 JEDEC ID。 */
HAL_StatusTypeDef W25Q64_Init(void);
/* 原始读写供后续固件缓存使用；擦写不允许覆盖末尾五个参数扇区。写入前须擦除。 */
//从任意地址读任意长度（可跨页，芯片内部地址自动递增）
HAL_StatusTypeDef W25Q64_Read(uint32_t address, void *data, uint32_t length);
//W25Q64_Program：页编程，内部会自动分页、写使能、等待完成。
HAL_StatusTypeDef W25Q64_Program(uint32_t address, const void *data, uint32_t length);
//擦除一个 4 KB 扇区，地址必须 4096 对齐。
HAL_StatusTypeDef W25Q64_EraseSector(uint32_t address);
/* 电机偏移量专用双槽：读取序号最新的有效记录，保存时保留上一份完整记录。 */
/* data 指向接收缓冲或待写缓冲；length 是数据字节数，不能超过单槽载荷上限。 */
HAL_StatusTypeDef W25Q64_LoadParameters(void *data, uint32_t length);
HAL_StatusTypeDef W25Q64_SaveParameters(const void *data, uint32_t length);
/* 用户参数专用双槽：previous=0 读取最新版，previous!=0 读取较旧版。 */
/* 读取不补写另一槽；旧版不存在或资源正忙时返回 HAL_BUSY，SPI 故障返回 HAL_ERROR。 */
HAL_StatusTypeDef W25Q64_LoadUserParameters(void *data, uint32_t length, uint8_t previous);
/* 将新用户参数写到较旧或无效槽；内容与最新版相同时直接返回成功、不擦写。 */
HAL_StatusTypeDef W25Q64_SaveUserParameters(const void *data, uint32_t length);
/* 从独立出厂扇区读取整份默认参数，供“恢复出厂设置”使用。 */
HAL_StatusTypeDef W25Q64_LoadFactoryParameters(void *data, uint32_t length);
/* 上电用固件默认值核对出厂扇区；仅在空白、校验失败或内容变化时擦写。 */
HAL_StatusTypeDef W25Q64_SyncFactoryParameters(const void *data, uint32_t length);

#endif
