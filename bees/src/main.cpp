#include <zephyr/kernel.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(emmc_test, LOG_LEVEL_DBG);

namespace {
    constexpr const char* DISK_NAME   = "MMC";
    constexpr uint32_t    SECTOR_SIZE = 512U;
    constexpr uint32_t    TEST_SECTOR = 0U;

    uint8_t buf[SECTOR_SIZE];
}

static bool init_disk()
{
    const int ret = disk_access_init(DISK_NAME);
    if (ret != 0) {
        LOG_ERR("disk_access_init failed: %d", ret);
        return false;
    }
    return true;
}

static void print_disk_info()
{
    uint32_t sector_count = 0U;
    uint32_t sector_size  = 0U;

    if (disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count) == 0) {
        LOG_INF("Sector count: %u", sector_count);
        LOG_INF("Total size:   %u MB", 
                static_cast<uint32_t>((static_cast<uint64_t>(sector_count) * SECTOR_SIZE) 
                / (1024U * 1024U)));
    } else {
        LOG_WRN("Could not read sector count");
    }

    if (disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size) == 0) {
        LOG_INF("Sector size: %u bytes", sector_size);
    } else {
        LOG_WRN("Could not read sector size");
    }
}

static bool read_sector(const uint32_t sector)
{
    const int ret = disk_access_read(DISK_NAME, buf, sector, 1U);
    if (ret != 0) {
        LOG_ERR("Read of sector %u failed: %d", sector, ret);
        return false;
    }
    return true;
}

static void check_mbr()
{
    constexpr uint8_t  MBR_SIG_BYTE0    = 0x55U;
    constexpr uint8_t  MBR_SIG_BYTE1    = 0xAAU;
    constexpr uint32_t MBR_SIG_OFFSET_0 = 510U;
    constexpr uint32_t MBR_SIG_OFFSET_1 = 511U;

    LOG_INF("First 16 bytes of sector 0:");
    LOG_HEXDUMP_INF(buf, 16U, "sector0:");

    if (buf[MBR_SIG_OFFSET_0] == MBR_SIG_BYTE0 &&
        buf[MBR_SIG_OFFSET_1] == MBR_SIG_BYTE1) {
        LOG_INF("Valid MBR signature found (0x55 0xAA) — eMMC is formatted");
    } else {
        LOG_WRN("No MBR signature found — eMMC may be unformatted or raw");
        LOG_WRN("Bytes at 510-511: 0x%02X 0x%02X",
                buf[MBR_SIG_OFFSET_0], buf[MBR_SIG_OFFSET_1]);
    }
}

int main()
{
    LOG_INF("eMMC test starting...");

    if (!init_disk()) {
        return -EIO;
    }

    print_disk_info();

    if (!read_sector(TEST_SECTOR)) {
        return -EIO;
    }

    check_mbr();

    LOG_INF("eMMC test complete");
    return 0;
}