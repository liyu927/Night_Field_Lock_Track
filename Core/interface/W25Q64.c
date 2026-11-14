#include "W25Q64.h"
#include "spi.h"
#include <string.h>

/* W25Q64 指令与状态位；读写均通过 SPI2 全双工 DMA 完成。 */
#define W25_CMD_JEDEC_ID       0x9FU//读 JEDEC ID 指令 0x9F：回 3 字节（厂商 + 存储类型 + 容量）。
#define W25_CMD_READ           0x03U//普通读指令 0x03：后面跟 3 字节地址，然后可以一直读下去（称为 Read Data）。
#define W25_CMD_WRITE_ENABLE   0x06U//写使能指令 0x06：每次编程/擦除前都必须发，否则芯片拒绝写入。
#define W25_CMD_STATUS         0x05U//读状态寄存器-1 指令 0x05：后面持续读可以不断拿到状态字节。
#define W25_CMD_PROGRAM        0x02U//页编程指令 0x02：后面跟 3 字节地址，再跟最多 256 字节数据。
#define W25_CMD_ERASE_4K       0x20U//4 KB 扇区擦除指令 0x20：后面跟 3 字节地址（芯片会自动对齐到扇区边界）。
#define W25_STATUS_BUSY        0x01U//状态寄存器 BUSY 位掩码 bit0：1 = 芯片正在内部擦/写，此时不响应多数指令。
#define W25_STATUS_WEL         0x02U//状态寄存器 WEL 位掩码 bit1：Write Enable Latch，1 = 写使能已生效。
#define W25_DMA_TIMEOUT_MS     100U//一次 DMA 传输的超时上限 100 ms。256 字节 @21 MHz 只要约 0.1 ms，100 ms 是极宽松的保护。
#define W25_PROGRAM_TIMEOUT_MS 10U//页编程典型耗时上限 10 ms（手册典型值 0.4~3 ms）。
#define W25_ERASE_TIMEOUT_MS   3000U//扇区擦除超时 3 s（手册典型值 45~400 ms，取 3 s 保险）。
#define W25_RECORD_MAGIC      0x36345157UL//包头的标识字段（包头共 16 字节：标识、序号、数据长度、CRC32），记录标记 0x36345157。小端存进 Flash 后字节顺序是 57 51 34 36，正好是 ASCII W Q 4 6——用来判断"这块扇区是不是我们写过的参数"。
/* 芯片末尾两个扇区专用于参数，固件缓存只能使用前面的空间。 */
#define W25_SLOT_A            (W25Q64_CAPACITY_BYTES - 2UL * W25Q64_SECTOR_BYTES)//A 槽地址 = 0x800000 − 8192 = 0x7FE000。
#define W25_SLOT_B            (W25Q64_CAPACITY_BYTES - W25Q64_SECTOR_BYTES)//B 槽地址 = 0x800000 − 4096 = 0x7FF000。两个槽紧挨着，各 4 KB。

/* DMA 必须使用在传输完成前始终有效的缓冲区；锁防止任务交叉使用它们。 */
static uint8_t dma_tx[W25Q64_PAGE_BYTES];//静态发送缓冲 256 字节。必须是 static：DMA 控制器直接读这块内存的物理地址，不能用栈上的临时数组（函数返回就失效）
static uint8_t dma_rx[W25Q64_PAGE_BYTES];//静态发送缓冲 256 字节。必须是 static：DMA 控制器直接读这块内存的物理地址，不能用栈上的临时数组（函数返回就失效）
static uint8_t flash_busy;//互斥标志 flash_busy：0 = 空闲，1 = 已被某个任务占用。

/* 短暂关中断只用于原子地取得缓冲区使用权，不包住实际 SPI 传输。 */
static HAL_StatusTypeDef flash_lock(void)//抢锁，抢到返回 HAL_OK，抢不到返回 HAL_BUSY。
{
    uint32_t irq_state = __get_PRIMASK();//先读出 PRIMASK（中断屏蔽寄存器），记住"进来之前中断到底是开着的还是关着的"。
    __disable_irq();//关中断进入临界区，保证下面"检查 + 置位"两步不会被打断。
    if (flash_busy != 0U)
    {
        if (irq_state == 0U) __enable_irq();//若进来前中断是开的（irq_state==0 表示 PRIMASK 为 0，即没被屏蔽）就恢复开中断——不能粗暴地一律打开。
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
    uint32_t started = HAL_GetTick();//记录起始时刻（毫秒），用于超时判断
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
            (void)HAL_SPI_Abort(&hspi2);//超过 100 ms 还没完成：调用 HAL_SPI_Abort 中止 DMA 和 SPI。
            return HAL_TIMEOUT;
        }
    }
    if (HAL_SPI_GetError(&hspi2) != HAL_SPI_ERROR_NONE) return HAL_ERROR;
    if (rx != NULL) memcpy(rx, dma_rx, length);
    return HAL_OK;
}

static HAL_StatusTypeDef flash_command(const uint8_t *command, uint16_t length)//只发不收的封装（例如发指令字节）。
{
    return flash_transfer(command, NULL, length);
}

/* 片选在指令与返回的状态字节之间保持低电平。 */
static HAL_StatusTypeDef flash_status(uint8_t *status)
{
    const uint8_t command = W25_CMD_STATUS;//局部常量：读状态指令 0x05。用 const 是为了让它有确定的存储地址，可以取地址传给下面的函数。
    HAL_StatusTypeDef result;
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    result = flash_command(&command, 1U);
    if (result == HAL_OK) result = flash_transfer(NULL, status, 1U);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    return result;
}

/* 页编程、扇区擦除完成前，状态寄存器的 BUSY 位会保持为 1。读取到1说明W25Q64的工作已经完成，可以继续下一步 */
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

/* 每次页编程或擦除前都要重新置位写使能，并确认 WEL 位。flash_status读状态寄存器，该寄存器每一位bit为1都代表一种功能使能成功 */
static HAL_StatusTypeDef flash_write_enable(void)
{
    const uint8_t command = W25_CMD_WRITE_ENABLE;
    uint8_t status;
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = flash_command(&command, 1U);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    if (result != HAL_OK || flash_status(&status) != HAL_OK) return HAL_ERROR;
    return (status & W25_STATUS_WEL) != 0U ? HAL_OK : HAL_ERROR;//读取W25Q64发回来的状态值确认是否收到数据
}

static void flash_address_command(uint8_t command[4], uint8_t opcode, uint32_t address)//把指令码 + 24 位地址拼成 4 字节命令。
{
    command[0] = opcode;
    command[1] = (uint8_t)(address >> 16);//A23~A16
    command[2] = (uint8_t)(address >> 8);//A15~A8
    command[3] = (uint8_t)address;//A7~A0   4 字节命令缓冲（1 指令 + 3 地址）。
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
    while (length != 0U)//前面一页写满了，还会回来继续编辑下一页
    {
        uint32_t room = W25Q64_PAGE_BYTES - (address % W25Q64_PAGE_BYTES);//room = 当前页还剩多少字节
        uint16_t count = length < room ? (uint16_t)length : (uint16_t)room;//本次写入长度 = min(剩余数据长度, 本页剩余空间)。
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
static HAL_StatusTypeDef flash_check_slot(uint32_t slot, uint32_t length, uint32_t *sequence)//检查某一个槽（slot）里的记录是否完整可信。sequence是序号
{
    uint8_t header[16];//16 字节包头缓冲区。
    uint8_t block[W25Q64_PAGE_BYTES];//256 字节分块缓冲，用于分段喂给 CRC。
    uint32_t crc = 0xFFFFFFFFUL;//CRC 初值 0xFFFFFFFF（标准 CRC32 的规定初值）。
    uint32_t remaining = length;//还剩多少字节参数没参与 CRC。
    uint32_t address = slot + sizeof(header);//参数数据的起始地址 = 槽地址 + 16（跳过包头）。
    if (flash_read_raw(slot, header, sizeof(header)) != HAL_OK) return HAL_ERROR;//读包头
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
    uint8_t header[16];//16 字节包头。
    uint32_t checked_sequence;//用于接收"回读校验"时得到的序号（本函数不关心，但接口要求给个变量）。
    uint32_t crc;//CRC 值。
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
        if (flash_read_raw(slot + 16U + offset, block, count) != HAL_OK)//+16U跳过包头
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

/* 分别读取验证两份记录，优先采用序号较新的有效记录。并且检查另一份状态 */
HAL_StatusTypeDef W25Q64_LoadParameters(void *data, uint32_t length)//读取参数包,这个data是一个电机结构体来的，要用void*来指内存地址
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
        if (result == HAL_OK)//检查AB区是否相同，如果AB区不相同，若A区序号更大，那么B区复制一遍A区的数据，并且序号是A区的+1
        {
            uint32_t other = slot == W25_SLOT_A ? W25_SLOT_B : W25_SLOT_A;//另一份（没被选中的那个槽）的地址。序号小
            uint32_t sequence = slot == W25_SLOT_A ? sequence_a : sequence_b;//被选中那份的序号。序号大
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
HAL_StatusTypeDef W25Q64_SaveParameters(const void *data, uint32_t length)//写入参数包
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
