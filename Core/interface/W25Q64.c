#include "W25Q64.h"
#include "spi.h"
#include <string.h>

/* W25Q64 指令与状态位；读写均通过 SPI2 全双工 DMA 完成。 */
#define W25_CMD_JEDEC_ID       0x9FU
#define W25_CMD_READ           0x03U
#define W25_CMD_WRITE_ENABLE   0x06U
#define W25_CMD_STATUS         0x05U
#define W25_CMD_PROGRAM        0x02U
#define W25_CMD_ERASE_4K       0x20U
#define W25_STATUS_BUSY        0x01U
#define W25_STATUS_WEL         0x02U
#define W25_DMA_TIMEOUT_MS     100U
#define W25_PROGRAM_TIMEOUT_MS 10U
#define W25_ERASE_TIMEOUT_MS   3000U
#define W25_RECORD_MAGIC      0x36345157UL
/* 芯片末尾两个扇区专用于参数，固件缓存只能使用前面的空间。 */
#define W25_SLOT_A            (W25Q64_CAPACITY_BYTES - 2UL * W25Q64_SECTOR_BYTES)
#define W25_SLOT_B            (W25Q64_CAPACITY_BYTES - W25Q64_SECTOR_BYTES)

/* DMA 必须使用在传输完成前始终有效的缓冲区；锁防止任务交叉使用它们。 */
static uint8_t dma_tx[W25Q64_PAGE_BYTES];
static uint8_t dma_rx[W25Q64_PAGE_BYTES];
static uint8_t flash_busy;

/* 短暂关中断只用于原子地取得缓冲区使用权，不包住实际 SPI 传输。 */
static HAL_StatusTypeDef flash_lock(void)
{
    uint32_t irq_state = __get_PRIMASK();
    __disable_irq();
    if (flash_busy != 0U)
    {
        if (irq_state == 0U) __enable_irq();
        return HAL_BUSY;
    }
    flash_busy = 1U;
    if (irq_state == 0U) __enable_irq();
    return HAL_OK;
}

static void flash_unlock(void)
{
    flash_busy = 0U;
}

/* SPI 发出多少字节就同时收到多少字节；纯读取时发送 0xFF 提供时钟。 */
static HAL_StatusTypeDef flash_transfer(const uint8_t *tx, uint8_t *rx, uint16_t length)
{
    uint32_t started = HAL_GetTick();
    if (length == 0U || length > W25Q64_PAGE_BYTES) return HAL_ERROR;
    if (tx != NULL) memcpy(dma_tx, tx, length);
    else memset(dma_tx, 0xFF, length);
    if (HAL_SPI_TransmitReceive_DMA(&hspi2, dma_tx, dma_rx, length) != HAL_OK)
        return HAL_ERROR;
    /* DMA 中断会让 HAL 恢复 READY；超时则中止本次传输。 */
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY)
    {
        if ((uint32_t)(HAL_GetTick() - started) >= W25_DMA_TIMEOUT_MS)
        {
            (void)HAL_SPI_Abort(&hspi2);
            return HAL_TIMEOUT;
        }
    }
    if (HAL_SPI_GetError(&hspi2) != HAL_SPI_ERROR_NONE) return HAL_ERROR;
    if (rx != NULL) memcpy(rx, dma_rx, length);
    return HAL_OK;
}

static HAL_StatusTypeDef flash_command(const uint8_t *command, uint16_t length)
{
    return flash_transfer(command, NULL, length);
}

/* 片选在指令与返回的状态字节之间保持低电平。 */
static HAL_StatusTypeDef flash_status(uint8_t *status)
{
    const uint8_t command = W25_CMD_STATUS;
    HAL_StatusTypeDef result;
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    result = flash_command(&command, 1U);
    if (result == HAL_OK) result = flash_transfer(NULL, status, 1U);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    return result;
}

/* 页编程、扇区擦除完成前，状态寄存器的 BUSY 位会保持为 1。 */
static HAL_StatusTypeDef flash_wait(uint32_t timeout_ms)
{
    uint32_t started = HAL_GetTick();
    uint8_t status;
    do
    {
        if (flash_status(&status) != HAL_OK) return HAL_ERROR;
        if ((status & W25_STATUS_BUSY) == 0U) return HAL_OK;
        HAL_Delay(1U);
    } while ((uint32_t)(HAL_GetTick() - started) < timeout_ms);
    return HAL_TIMEOUT;
}

/* 每次页编程或擦除前都要重新置位写使能，并确认 WEL 位。 */
static HAL_StatusTypeDef flash_write_enable(void)
{
    const uint8_t command = W25_CMD_WRITE_ENABLE;
    uint8_t status;
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = flash_command(&command, 1U);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    if (result != HAL_OK || flash_status(&status) != HAL_OK) return HAL_ERROR;
    return (status & W25_STATUS_WEL) != 0U ? HAL_OK : HAL_ERROR;
}

static void flash_address_command(uint8_t command[4], uint8_t opcode, uint32_t address)
{
    command[0] = opcode;
    command[1] = (uint8_t)(address >> 16);
    command[2] = (uint8_t)(address >> 8);
    command[3] = (uint8_t)address;
}

/* 一次 READ 指令可连续读取任意长度，DMA 按 256 字节分段。 */
static HAL_StatusTypeDef flash_read_raw(uint32_t address, uint8_t *data, uint32_t length)
{
    uint8_t command[4];
    HAL_StatusTypeDef result;
    flash_address_command(command, W25_CMD_READ, address);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    result = flash_command(command, sizeof(command));
    while (result == HAL_OK && length != 0U)
    {
        uint16_t count = length > W25Q64_PAGE_BYTES ? W25Q64_PAGE_BYTES : (uint16_t)length;
        result = flash_transfer(NULL, data, count);
        data += count;
        length -= count;
    }
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    return result;
}

/* 页编程不能跨越 256 字节页边界；每一页单独写使能并等待完成。 */
static HAL_StatusTypeDef flash_program_raw(uint32_t address, const uint8_t *data, uint32_t length)
{
    uint8_t command[4];
    while (length != 0U)
    {
        uint32_t room = W25Q64_PAGE_BYTES - (address % W25Q64_PAGE_BYTES);
        uint16_t count = length < room ? (uint16_t)length : (uint16_t)room;
        HAL_StatusTypeDef result = flash_write_enable();
        if (result != HAL_OK) return result;
        flash_address_command(command, W25_CMD_PROGRAM, address);
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
        result = flash_command(command, sizeof(command));
        if (result == HAL_OK) result = flash_transfer(data, NULL, count);
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
        if (result != HAL_OK) return result;
        result = flash_wait(W25_PROGRAM_TIMEOUT_MS);
        if (result != HAL_OK) return result;
        address += count;
        data += count;
        length -= count;
    }
    return HAL_OK;
}

/* W25Q64 的最小擦除单位为 4 KB。 */
static HAL_StatusTypeDef flash_erase_raw(uint32_t address)
{
    uint8_t command[4];
    HAL_StatusTypeDef result = flash_write_enable();
    if (result != HAL_OK) return result;
    flash_address_command(command, W25_CMD_ERASE_4K, address);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    result = flash_command(command, sizeof(command));
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    if (result != HAL_OK) return result;
    return flash_wait(W25_ERASE_TIMEOUT_MS);
}

/* 标准 CRC32：先以 0xFFFFFFFF 初始化，最后再异或 0xFFFFFFFF。 */
static uint32_t flash_crc(uint32_t crc, const uint8_t *data, uint32_t length)
{
    while (length-- != 0U)
    {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
            crc = (crc >> 1) ^ ((crc & 1U) != 0U ? 0xEDB88320UL : 0U);
    }
    return crc;
}

/* 参数包头的整数固定按小端字节序存放。 */
static uint32_t flash_u32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8)
        | ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static void flash_put_u32(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
    bytes[2] = (uint8_t)(value >> 16);
    bytes[3] = (uint8_t)(value >> 24);
}

/* 包头为标记、序号、长度、CRC 各 4 字节；CRC 覆盖前 12 字节与全部参数。 */
/* HAL_OK 表示有效，HAL_BUSY 表示空白或校验不符，HAL_ERROR 表示总线读取失败。 */
static HAL_StatusTypeDef flash_check_slot(uint32_t slot, uint32_t length, uint32_t *sequence)
{
    uint8_t header[16];
    uint8_t block[W25Q64_PAGE_BYTES];
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t remaining = length;
    uint32_t address = slot + sizeof(header);
    if (flash_read_raw(slot, header, sizeof(header)) != HAL_OK) return HAL_ERROR;
    if (flash_u32(header) != W25_RECORD_MAGIC || flash_u32(header + 8) != length)
        return HAL_BUSY;
    crc = flash_crc(crc, header, 12U);
    while (remaining != 0U)
    {
        uint32_t count = remaining > sizeof(block) ? sizeof(block) : remaining;
        if (flash_read_raw(address, block, count) != HAL_OK) return HAL_ERROR;
        crc = flash_crc(crc, block, count);
        address += count;
        remaining -= count;
    }
    if ((crc ^ 0xFFFFFFFFUL) != flash_u32(header + 12)) return HAL_BUSY;
    *sequence = flash_u32(header + 4);
    return HAL_OK;
}

/* 先擦除目标扇区，再写包头和数据，最后读回计算 CRC 确认。 */
static HAL_StatusTypeDef flash_save_slot(uint32_t slot, uint32_t sequence,
                                         const uint8_t *data, uint32_t length)
{
    uint8_t header[16];
    uint32_t checked_sequence;
    uint32_t crc;
    flash_put_u32(header, W25_RECORD_MAGIC);
    flash_put_u32(header + 4, sequence);
    flash_put_u32(header + 8, length);
    crc = flash_crc(0xFFFFFFFFUL, header, 12U);
    crc = flash_crc(crc, data, length) ^ 0xFFFFFFFFUL;
    flash_put_u32(header + 12, crc);
    if (flash_erase_raw(slot) != HAL_OK) return HAL_ERROR;
    if (flash_program_raw(slot, header, sizeof(header)) != HAL_OK) return HAL_ERROR;
    if (flash_program_raw(slot + sizeof(header), data, length) != HAL_OK) return HAL_ERROR;
    return flash_check_slot(slot, length, &checked_sequence);
}

/* 两份 CRC 都有效时仍需比较内容，处理断电后留下的新旧版本。 */
static HAL_StatusTypeDef flash_compare_slot(uint32_t slot,
                                            const uint8_t *data, uint32_t length)
{
    uint8_t block[W25Q64_PAGE_BYTES];
    uint32_t offset = 0U;
    while (offset < length)
    {
        uint32_t count = length - offset;
        if (count > sizeof(block)) count = sizeof(block);
        if (flash_read_raw(slot + 16U + offset, block, count) != HAL_OK)
            return HAL_ERROR;
        if (memcmp(block, data + offset, count) != 0) return HAL_BUSY;
        offset += count;
    }
    return HAL_OK;
}

/* 读取 JEDEC ID，确认连接的是 Winbond 64 Mbit 容量芯片。 */
HAL_StatusTypeDef W25Q64_Init(void)
{
    const uint8_t command = W25_CMD_JEDEC_ID;
    uint8_t id[3];
    HAL_StatusTypeDef result = flash_lock();
    if (result != HAL_OK) return result;
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    result = flash_command(&command, 1U);
    if (result == HAL_OK) result = flash_transfer(NULL, id, sizeof(id));
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    flash_unlock();
    if (result != HAL_OK) return result;
    return (id[0] == 0xEFU && id[2] == 0x17U) ? HAL_OK : HAL_ERROR;
}

/* 原始读取允许访问整个芯片，包括参数扇区。 */
HAL_StatusTypeDef W25Q64_Read(uint32_t address, void *data, uint32_t length)
{
    HAL_StatusTypeDef result;
    if (data == NULL || length == 0U || address >= W25Q64_CAPACITY_BYTES
        || length > W25Q64_CAPACITY_BYTES - address) return HAL_ERROR;
    result = flash_lock();
    if (result != HAL_OK) return result;
    result = flash_read_raw(address, data, length);
    flash_unlock();
    return result;
}

/* 原始编程限制在参数区之前；调用者须先擦除目标扇区。 */
HAL_StatusTypeDef W25Q64_Program(uint32_t address, const void *data, uint32_t length)
{
    HAL_StatusTypeDef result;
    if (data == NULL || length == 0U || address >= W25Q64_FIRMWARE_END
        || length > W25Q64_FIRMWARE_END - address) return HAL_ERROR;
    result = flash_lock();
    if (result != HAL_OK) return result;
    result = flash_program_raw(address, data, length);
    flash_unlock();
    return result;
}

/* 只允许擦除 4 KB 对齐的非参数扇区。 */
HAL_StatusTypeDef W25Q64_EraseSector(uint32_t address)
{
    HAL_StatusTypeDef result;
    if ((address % W25Q64_SECTOR_BYTES) != 0U || address >= W25Q64_FIRMWARE_END)
        return HAL_ERROR;
    result = flash_lock();
    if (result != HAL_OK) return result;
    result = flash_erase_raw(address);
    flash_unlock();
    return result;
}

/* 分别验证两份记录，优先采用序号较新的有效记录。 */
HAL_StatusTypeDef W25Q64_LoadParameters(void *data, uint32_t length)
{
    uint32_t sequence_a = 0U, sequence_b = 0U;
    HAL_StatusTypeDef a, b, result;
    if (data == NULL || length == 0U || length > W25Q64_PARAMETER_MAX_BYTES)
        return HAL_ERROR;
    result = flash_lock();
    if (result != HAL_OK) return result;
    a = flash_check_slot(W25_SLOT_A, length, &sequence_a);
    b = flash_check_slot(W25_SLOT_B, length, &sequence_b);
    if (a == HAL_ERROR || b == HAL_ERROR) result = HAL_ERROR;
    else if (a != HAL_OK && b != HAL_OK) result = HAL_BUSY;
    else
    {
        /* 序号较新者是最近一次写入；序号差用有符号比较以兼容回绕。 */
        uint32_t slot = (b == HAL_OK && (a != HAL_OK
            || (int32_t)(sequence_b - sequence_a) > 0)) ? W25_SLOT_B : W25_SLOT_A;
        result = flash_read_raw(slot + 16U, data, length);
        if (result == HAL_OK)
        {
            uint32_t other = slot == W25_SLOT_A ? W25_SLOT_B : W25_SLOT_A;
            uint32_t sequence = slot == W25_SLOT_A ? sequence_a : sequence_b;
            HAL_StatusTypeDef comparison = a != b ? HAL_BUSY
                : flash_compare_slot(other, data, length);
            if (comparison == HAL_ERROR) result = HAL_ERROR;
            /* 一份损坏或内容较旧时补齐备份；补写失败也保留已读出的有效记录。 */
            else if (comparison == HAL_BUSY)
                (void)flash_save_slot(other, sequence + 1U, data, length);
        }
    }
    flash_unlock();
    return result;
}

/* 先写无效或较旧副本，验证通过后才更新原来的有效副本。 */
/* 已有有效记录时，中途断电仍可保留写入前或写入后的完整参数包。 */
HAL_StatusTypeDef W25Q64_SaveParameters(const void *data, uint32_t length)
{
    uint32_t sequence_a = 0U, sequence_b = 0U;
    HAL_StatusTypeDef a, b, result;
    if (data == NULL || length == 0U || length > W25Q64_PARAMETER_MAX_BYTES)
        return HAL_ERROR;
    result = flash_lock();
    if (result != HAL_OK) return result;
    a = flash_check_slot(W25_SLOT_A, length, &sequence_a);
    b = flash_check_slot(W25_SLOT_B, length, &sequence_b);
    if (a == HAL_ERROR || b == HAL_ERROR) result = HAL_ERROR;
    else if (a != HAL_OK && b != HAL_OK)
    {
        /* 空白芯片或两份都无效：先建 A，再建 B。 */
        result = flash_save_slot(W25_SLOT_A, 1U, data, length);
        if (result == HAL_OK) result = flash_save_slot(W25_SLOT_B, 2U, data, length);
    }
    else if (a != HAL_OK || (b == HAL_OK && (int32_t)(sequence_b - sequence_a) > 0))
    {
        /* B 较新或 A 无效：先更新 A，成功后再更新 B。 */
        result = flash_save_slot(W25_SLOT_A, sequence_b + 1U, data, length);
        if (result == HAL_OK)
            result = flash_save_slot(W25_SLOT_B, sequence_b + 2U, data, length);
    }
    else
    {
        /* A 较新或 B 无效：先更新 B，成功后再更新 A。 */
        result = flash_save_slot(W25_SLOT_B, sequence_a + 1U, data, length);
        if (result == HAL_OK)
            result = flash_save_slot(W25_SLOT_A, sequence_a + 2U, data, length);
    }
    flash_unlock();
    return result;
}
