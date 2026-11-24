#include "communication/bootloader.hpp"

#include <stddef.h>

#include <stm32f405xx.h>
#include <stm32f4xx_hal_flash_ex.h>

namespace {

constexpr uint32_t FLASH_ERROR_FLAGS =
    FLASH_SR_SOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
    FLASH_SR_PGPERR | FLASH_SR_PGSERR;
constexpr uint32_t FLASH_UNLOCK_KEY1 = 0x45670123;
constexpr uint32_t FLASH_UNLOCK_KEY2 = 0xCDEF89AB;
constexpr uint32_t FLASH_WAIT_LIMIT = 100000000;
constexpr uint32_t SRAM_START = 0x20000000;
constexpr uint32_t SRAM_END = 0x20020000;

uint32_t crc32(const uint8_t* data, uint32_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint32_t bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
    }
    return ~crc;
}

bool wait_for_flash() {
    uint32_t timeout = FLASH_WAIT_LIMIT;
    while ((FLASH->SR & FLASH_SR_BSY) && timeout)
        --timeout;
    return timeout != 0;
}

void unlock_flash() {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_UNLOCK_KEY1;
        FLASH->KEYR = FLASH_UNLOCK_KEY2;
    }
}

void lock_flash() {
    FLASH->CR |= FLASH_CR_LOCK;
}

bool flash_operation_ok() {
    const uint32_t errors = FLASH->SR & FLASH_ERROR_FLAGS;
    FLASH->SR = errors | FLASH_SR_EOP;
    return errors == 0;
}

bool erase_sector(uint32_t sector) {
    if (!wait_for_flash())
        return false;
    FLASH->SR = FLASH_ERROR_FLAGS | FLASH_SR_EOP;
    FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_SER | FLASH_CR_SNB | FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos) | FLASH_CR_PSIZE_1;
    FLASH->CR |= FLASH_CR_STRT;
    const bool completed = wait_for_flash();
    FLASH->CR &= ~FLASH_CR_SER;
    return completed && flash_operation_ok();
}

bool program_word(uint32_t address, uint32_t value) {
    if (!wait_for_flash())
        return false;
    FLASH->SR = FLASH_ERROR_FLAGS | FLASH_SR_EOP;
    FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_SER | FLASH_CR_SNB | FLASH_CR_PSIZE);
    FLASH->CR |= FLASH_CR_PG | FLASH_CR_PSIZE_1;
    *reinterpret_cast<volatile uint32_t*>(address) = value;
    const bool completed = wait_for_flash();
    FLASH->CR &= ~FLASH_CR_PG;
    return completed && flash_operation_ok();
}

bool valid_metadata(const OtaMetadata& metadata) {
    const bool app = metadata.magic == OTA_METADATA_MAGIC;
    return ((app && metadata.image_type == OtaCanProtocol::APPLICATION) ||
                   (metadata.magic == OTA_CONFIG_MAGIC &&
                    metadata.image_type == OtaCanProtocol::PARAMETERS)) &&
           metadata.format == OTA_METADATA_FORMAT &&
           metadata.state == OTA_STATE_PENDING &&
           valid_image_size(metadata.image_type, metadata.firmware_size) &&
           metadata.chunk_count ==
               (metadata.firmware_size + OTA_CHUNK_SIZE - 1) / OTA_CHUNK_SIZE &&
           metadata.metadata_crc32 ==
               crc32(reinterpret_cast<const uint8_t*>(&metadata),
                     offsetof(OtaMetadata, metadata_crc32));
}

bool valid_application_vectors(uint32_t address, uint32_t image_size) {
    if (image_size < 8)
        return false;
    const volatile uint32_t* vectors =
        reinterpret_cast<const volatile uint32_t*>(address);
    const uint32_t stack_pointer = vectors[0];
    const uint32_t reset_handler = vectors[1];
    return valid_app_vectors(stack_pointer, reset_handler, image_size);
}

bool set_metadata_state(uint32_t state) {
    const uint32_t address =
        BOOTLOADER_DATA_START + offsetof(OtaMetadata, state);
    unlock_flash();
    const bool result = program_word(address, state);
    lock_flash();
    return result;
}

bool install_pending_image(const OtaMetadata& metadata) {
    const uint8_t* source =
        reinterpret_cast<const uint8_t*>(APPLICATION_BUFFER_START);
    const bool app = metadata.magic == OTA_METADATA_MAGIC;
    const uint32_t destination = app ? APPLICATION_START : APPLICATION_CONFIG_A_START;
    const uint32_t first = app ? FLASH_SECTOR_4 : FLASH_SECTOR_10;
    const uint32_t last = app ? FLASH_SECTOR_6 : FLASH_SECTOR_11;
    if (crc32(source, metadata.firmware_size) != metadata.firmware_crc32 ||
        (app && !valid_application_vectors(APPLICATION_BUFFER_START, metadata.firmware_size))) {
        set_metadata_state(OTA_STATE_FAILED);
        return false;
    }

    unlock_flash();
    for (uint32_t sector = first; sector <= last; ++sector) {
        if (!erase_sector(sector)) {
            lock_flash();
            return false;
        }
    }

    const uint32_t word_count = (metadata.firmware_size + 3) / 4;
    const volatile uint32_t* source_words =
        reinterpret_cast<const volatile uint32_t*>(APPLICATION_BUFFER_START);
    for (uint32_t i = 0; i < word_count; ++i) {
        if (!program_word(destination + i * 4, source_words[i])) {
            lock_flash();
            return false;
        }
    }
    lock_flash();

    if (crc32(reinterpret_cast<const uint8_t*>(destination),
              metadata.firmware_size) != metadata.firmware_crc32 ||
        (app && !valid_application_vectors(destination, metadata.firmware_size))) {
        return false;
    }

    return set_metadata_state(OTA_STATE_INSTALLED);
}

bool application_vectors_present() {
    const volatile uint32_t* vectors =
        reinterpret_cast<const volatile uint32_t*>(APPLICATION_START);
    const uint32_t stack_pointer = vectors[0];
    const uint32_t reset_handler = vectors[1];
    return stack_pointer >= SRAM_START && stack_pointer <= SRAM_END &&
           (reset_handler & 1U) != 0 &&
           (reset_handler & ~1U) >= APPLICATION_START &&
           (reset_handler & ~1U) < APPLICATION_START + APPLICATION_SIZE;
}

void jump_to_application() {
    if (!application_vectors_present())
        return;

    const volatile uint32_t* vectors =
        reinterpret_cast<const volatile uint32_t*>(APPLICATION_START);
    const uint32_t stack_pointer = vectors[0];
    const uint32_t reset_handler = vectors[1];
    using ApplicationEntry = void (*)();
    const ApplicationEntry entry =
        reinterpret_cast<ApplicationEntry>(reset_handler);

    __disable_irq();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    for (uint32_t i = 0; i < 8; ++i) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    SCB->VTOR = APPLICATION_START;
    __DSB();
    __ISB();
    __set_CONTROL(0);
    __set_MSP(stack_pointer);
    __enable_irq();
    entry();
}

void halt() {
    __disable_irq();
    for (;;)
        __WFI();
}

}

extern "C" void SystemInit(void) {
    SCB->VTOR = BOOTLOADER_START;
    SCB->CPACR |= (0xFU << 20);
    __DSB();
    __ISB();
}

extern "C" void early_start_checks(void) {}

extern "C" int main(void) {
    const OtaMetadata* metadata =
        reinterpret_cast<const OtaMetadata*>(BOOTLOADER_DATA_START);
    if (metadata->state == OTA_STATE_PENDING) {
        if (!valid_metadata(*metadata)) {
            set_metadata_state(OTA_STATE_FAILED);
        } else if (!install_pending_image(*metadata)) {
            if (metadata->state == OTA_STATE_PENDING)
                halt();
        } else {
            NVIC_SystemReset();
            halt();
        }
    }

    jump_to_application();
    halt();
}
