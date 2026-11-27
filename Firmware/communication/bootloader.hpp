#ifndef FIRMWARE_BOOTLOADER_HPP
#define FIRMWARE_BOOTLOADER_HPP

#include <stdint.h>

#define BASE_ADDRESS        0x08000000
#define SECTOR_0_START      (BASE_ADDRESS)
#define SECTOR_1_START      (SECTOR_0_START + 0x4000)
#define SECTOR_2_START      (SECTOR_1_START + 0x4000)
#define SECTOR_3_START      (SECTOR_2_START + 0x4000)
#define SECTOR_4_START      (SECTOR_3_START + 0x4000)
#define SECTOR_5_START      (SECTOR_4_START + 0x10000)
#define SECTOR_6_START      (SECTOR_5_START + 0x20000)
#define SECTOR_7_START      (SECTOR_6_START + 0x20000)
#define SECTOR_8_START      (SECTOR_7_START + 0x20000)
#define SECTOR_9_START      (SECTOR_8_START + 0x20000)
#define SECTOR_10_START     (SECTOR_9_START + 0x20000)
#define SECTOR_11_START     (SECTOR_10_START + 0x20000)


#define BOOTLOADER_START             SECTOR_0_START
#define BOOTLOADER_SIZE              (SECTOR_3_START - SECTOR_0_START)
#define BOOTLOADER_DATA_START        SECTOR_3_START
#define BOOTLOADER_DATA_SIZE         (SECTOR_4_START - SECTOR_3_START)
#define APPLICATION_START            SECTOR_4_START
#define APPLICATION_SIZE             (SECTOR_7_START - SECTOR_4_START)
#define APPLICATION_BUFFER_START     SECTOR_7_START
#define APPLICATION_BUFFER_SIZE      (SECTOR_10_START - SECTOR_7_START)
#define APPLICATION_CONFIG_A_START   SECTOR_10_START
#define APPLICATION_CONFIG_B_START   SECTOR_11_START
#define APPLICATION_CONFIG_SIZE      (SECTOR_11_START - SECTOR_10_START)

constexpr uint32_t OTA_METADATA_MAGIC = 0x4F54414D;
constexpr uint32_t OTA_CONFIG_MAGIC = 0x4346474D;
constexpr uint32_t OTA_METADATA_FORMAT = 2;
constexpr uint32_t OTA_STATE_EMPTY = 0xFFFFFFFF;
constexpr uint32_t OTA_STATE_PENDING = 0xFFFFFFFE;
constexpr uint32_t OTA_STATE_INSTALLED = 0xFFFFFFFC;
constexpr uint32_t OTA_STATE_FAILED = 0xFFFFFFF8;
constexpr uint32_t OTA_CHUNK_SIZE = 1024;
constexpr uint32_t OTA_MAX_FIRMWARE_SIZE = APPLICATION_SIZE;
constexpr uint32_t OTA_MAX_CHUNKS =
    (OTA_MAX_FIRMWARE_SIZE + OTA_CHUNK_SIZE - 1) / OTA_CHUNK_SIZE;

struct OtaMetadata {
    uint32_t magic;
    uint32_t format;
    uint32_t image_type;
    uint32_t firmware_size;
    uint32_t firmware_crc32;
    uint32_t chunk_count;
    uint32_t metadata_crc32;
    uint32_t state;
};

static_assert(sizeof(OtaMetadata) == 32, "OTA metadata layout must be 32 bytes");

namespace OtaCanProtocol {

constexpr uint32_t CAN_ID = 0xABC;
constexpr uint32_t RESPONSE_CAN_ID = 0xDEF;
constexpr uint8_t FRAME_DATA = 0x01;
constexpr uint8_t FRAME_DATA_ACK = 0x02;
constexpr uint8_t FRAME_CHUNK_CRC_REQUEST = 0x03;
constexpr uint8_t FRAME_CHUNK_CRC_RESPONSE = 0x04;
constexpr uint8_t FRAME_IMAGE_CRC_REQUEST = 0x05;
constexpr uint8_t FRAME_IMAGE_CRC_RESPONSE = 0x06;
constexpr uint8_t FRAME_VERSION_REQUEST = 0x07;
constexpr uint8_t FRAME_VERSION_RESPONSE = 0x08;
constexpr uint8_t FRAME_APPLICATION_REQUEST = 0x10;
constexpr uint8_t FRAME_PARAMETERS_REQUEST = 0x11;
constexpr uint8_t FRAME_READY = 0x13;
enum ImageType : uint8_t { APPLICATION = 1, PARAMETERS = 2 };

enum ReadyStatus : uint8_t {
    READY_PENDING = 0,
    READY_SUCCESS = 1,
    READY_REJECTED = 2
};

}

inline bool image_fits_flash(uint32_t type, uint32_t size) {
    return type == OtaCanProtocol::APPLICATION
        ? size > 0 && size <= APPLICATION_SIZE
        : type == OtaCanProtocol::PARAMETERS &&
          size > 0 && size <= APPLICATION_CONFIG_SIZE;
}

inline bool valid_app_reset_entry(uint32_t entry) {
    return (entry & 1) && (entry & ~1U) >= APPLICATION_START &&
           (entry & ~1U) < APPLICATION_START + APPLICATION_SIZE;
}

#endif
