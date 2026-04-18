/*
 * MCP23017 I2C Communication Proof Test
 *
 * Target board: ranger_1 (Teensy 4.1-based LPL board)
 * Bus: LPI2C1 (Teensy pins 18 SDA, 19 SCL)
 * Device: Waveshare MCP23017 IO Expansion Board at address 0x27
 *
 * Every printed value is a byte actually received from the chip over I2C.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

#define MCP23017_ADDR    0x27

/* MCP23017 registers (IOCON.BANK=0 default) */
#define REG_IODIR_A      0x00   /* Pin direction: 1=input, 0=output. POR default: 0xFF */
#define REG_IODIR_B      0x01   /* Port B direction */
#define REG_GPIO_A       0x12   /* Port A pin values */
#define REG_GPIO_B       0x13   /* Port B pin values */
#define REG_OLAT_A       0x14   /* Port A output latch */
#define REG_OLAT_B       0x15   /* Port B output latch */

#define I2C_BUS_NODE     DT_NODELABEL(lpi2c1)

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *val)
{
	return i2c_write_read(dev, MCP23017_ADDR, &reg, 1, val, 1);
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = {reg, val};
	return i2c_write(dev, buf, 2, MCP23017_ADDR);
}

int main(void)
{
	int ret;
	uint8_t val;

	/* Wait for USB CDC serial to connect */
	k_msleep(3000);

	printk("\n============================================\n");
	printk("  MCP23017 I2C Communication Proof Test\n");
	printk("  ranger_1 (Teensy 4.1) <-> Waveshare MCP23017\n");
	printk("  Bus: LPI2C1 (pins 19 SCL, 18 SDA)\n");
	printk("============================================\n\n");

	const struct device *i2c_dev = DEVICE_DT_GET(I2C_BUS_NODE);

	if (!device_is_ready(i2c_dev)) {
		printk("FAIL: LPI2C1 not ready. Check overlay.\n");
		return -1;
	}
	printk("[OK] LPI2C1 bus is ready.\n\n");

	/* ---- TEST 1: Bus scan ---- */
	printk("--- TEST 1: I2C Bus Scan ---\n");
	int found = 0;
	for (uint8_t addr = 0x03; addr < 0x78; addr++) {
		uint8_t dummy;
		if (i2c_write_read(i2c_dev, addr, NULL, 0, &dummy, 1) == 0) {
			printk("  Device ACK at 0x%02X", addr);
			if (addr == MCP23017_ADDR) {
				printk("  <-- MCP23017 (expected)");
			}
			printk("\n");
			found++;
		}
	}
	if (found == 0) {
		printk("FAIL: No devices found. Check wiring (SDA=18, SCL=19, VCC=3.3V, GND=GND).\n");
		return -1;
	}
	printk("Found %d device(s).\n\n", found);

	/* ---- TEST 2: Read default register ---- */
	printk("--- TEST 2: Read IODIR_A (datasheet default: 0xFF) ---\n");
	ret = reg_read(i2c_dev, REG_IODIR_A, &val);
	if (ret != 0) {
		printk("FAIL: read error %d\n\n", ret);
		return -1;
	}
	printk("  Read: 0x%02X %s\n\n", val,
	       (val == 0xFF) ? "PASS (matches datasheet default)" : "(device responded, not at POR default)");

	/* ---- TEST 3: Write and read back ---- */
	printk("--- TEST 3: Write/Readback Verification ---\n");
	printk("  Writing 0xFE to IODIR_A (PA0 = output, rest = inputs)\n");
	ret = reg_write(i2c_dev, REG_IODIR_A, 0xFE);
	if (ret != 0) {
		printk("FAIL: write error %d\n\n", ret);
		return -1;
	}
	ret = reg_read(i2c_dev, REG_IODIR_A, &val);
	if (ret != 0) {
		printk("FAIL: readback error %d\n\n", ret);
		return -1;
	}
	printk("  Readback: 0x%02X %s\n\n", val,
	       (val == 0xFE) ? "PASS (bidirectional I2C confirmed)" : "FAIL");
	if (val != 0xFE) {
		return -1;
	}

	/* ---- TEST 4: Toggle PA0 6 times ---- */
	printk("--- TEST 4: GPIO Toggle via I2C (6 cycles) ---\n");
	for (int i = 0; i < 6; i++) {
		uint8_t set_val = (i % 2 == 0) ? 0x01 : 0x00;
		reg_write(i2c_dev, REG_GPIO_A, set_val);
		reg_read(i2c_dev, REG_OLAT_A, &val);
		printk("  PA0 = %s  (wrote 0x%02X, OLAT readback: 0x%02X) %s\n",
		       set_val ? "HIGH" : "LOW ", set_val, val,
		       (val == set_val) ? "OK" : "MISMATCH");
		k_msleep(1000);
	}

	/* ---- TEST 5: Blink ALL Port A and Port B pins ---- */
	printk("\n--- TEST 5: Blink All Pins (PA0-PA7 + PB0-PB7) ---\n");
	printk("  Probe ANY pin with multimeter. Should see 3.3V/0V toggling.\n");
	printk("  If PA0 reads 0V, try PA1, PA2, PB0, etc.\n\n");

	/* Set ALL Port A pins as outputs */
	ret = reg_write(i2c_dev, REG_IODIR_A, 0x00);
	if (ret != 0) {
		printk("  FAIL: Could not set IODIR_A: %d\n", ret);
	}

	/* Set ALL Port B pins as outputs */
	ret = reg_write(i2c_dev, REG_IODIR_B, 0x00);
	if (ret != 0) {
		printk("  FAIL: Could not set IODIR_B: %d\n", ret);
	}

	/* Verify direction registers */
	reg_read(i2c_dev, REG_IODIR_A, &val);
	printk("  IODIR_A = 0x%02X (expect 0x00 = all outputs)\n", val);
	reg_read(i2c_dev, REG_IODIR_B, &val);
	printk("  IODIR_B = 0x%02X (expect 0x00 = all outputs)\n\n", val);

	int blink_count = 0;
	while (1) {
		/* ALL pins HIGH */
		reg_write(i2c_dev, REG_GPIO_A, 0xFF);
		reg_write(i2c_dev, REG_GPIO_B, 0xFF);
		reg_read(i2c_dev, REG_OLAT_A, &val);
		printk("  [%d] ALL HIGH  (OLAT_A: 0x%02X", blink_count, val);
		reg_read(i2c_dev, REG_OLAT_B, &val);
		printk(", OLAT_B: 0x%02X)\n", val);
		k_msleep(500);

		/* ALL pins LOW */
		reg_write(i2c_dev, REG_GPIO_A, 0x00);
		reg_write(i2c_dev, REG_GPIO_B, 0x00);
		reg_read(i2c_dev, REG_OLAT_A, &val);
		printk("  [%d] ALL LOW   (OLAT_A: 0x%02X", blink_count, val);
		reg_read(i2c_dev, REG_OLAT_B, &val);
		printk(", OLAT_B: 0x%02X)\n", val);
		k_msleep(500);

		blink_count++;
	}

	return 0;
}
