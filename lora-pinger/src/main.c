#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <string.h>

#define LORA_FREQUENCY       915000000
#define LORA_TX_POWER        2        /* Low power, benchtop test */
#define LORA_PREAMBLE_LEN    8

#define LATENCY_ITERATIONS   100
#define THROUGHPUT_PACKETS   200
#define THROUGHPUT_PKT_SIZE  64
#define RECV_TIMEOUT_MS      3000

#define PKT_PING  0x01
#define PKT_PONG  0x02
#define PKT_DATA  0x03
#define PKT_DONE  0x04

struct test_pkt {
	uint8_t  type;
	uint16_t seq;
} __packed;

static const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));

static struct lora_modem_config tx_cfg = {
	.frequency = LORA_FREQUENCY,
	.bandwidth = BW_125_KHZ,
	.datarate = SF_7,
	.coding_rate = CR_4_5,
	.preamble_len = LORA_PREAMBLE_LEN,
	.tx_power = LORA_TX_POWER,
	.tx = true,
};

static struct lora_modem_config rx_cfg = {
	.frequency = LORA_FREQUENCY,
	.bandwidth = BW_125_KHZ,
	.datarate = SF_7,
	.coding_rate = CR_4_5,
	.preamble_len = LORA_PREAMBLE_LEN,
	.tx_power = LORA_TX_POWER,
	.tx = false,
};

static void wait_for_usb(void)
{
	const struct device *dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	uint32_t dtr = 0;

	usb_enable(NULL);

	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	k_sleep(K_MSEC(500));
}

static void run_latency_test(void)
{
	struct test_pkt ping = { .type = PKT_PING };
	uint8_t rx_buf[255];
	int16_t rssi;
	int8_t  snr;
	uint32_t rtt_us[LATENCY_ITERATIONS];
	int good = 0;

	printk("\n=== Latency Test (%d iterations) ===\n\n", LATENCY_ITERATIONS);

	for (int i = 0; i < LATENCY_ITERATIONS; i++) {
		ping.seq = i;

		lora_config(lora_dev, &tx_cfg);
		uint32_t t_start = k_cycle_get_32();
		int ret = lora_send(lora_dev, (uint8_t *)&ping, sizeof(ping));
		if (ret < 0) {
			printk("[%3d] TX failed: %d\n", i, ret);
			continue;
		}

		lora_config(lora_dev, &rx_cfg);
		ret = lora_recv(lora_dev, rx_buf, sizeof(rx_buf),
				K_MSEC(RECV_TIMEOUT_MS), &rssi, &snr);
		uint32_t t_end = k_cycle_get_32();

		if (ret < (int)sizeof(struct test_pkt)) {
			printk("[%3d] RX timeout or too short\n", i);
			continue;
		}

		struct test_pkt *pong = (struct test_pkt *)rx_buf;
		if (pong->type != PKT_PONG || pong->seq != i) {
			printk("[%3d] Bad pong: type=%d seq=%d\n",
			       i, pong->type, pong->seq);
			continue;
		}

		uint32_t rtt = k_cyc_to_us_floor32(t_end - t_start);
		rtt_us[good] = rtt;
		good++;

		printk("[%3d] RTT: %u us  RSSI: %d dBm  SNR: %d dB\n",
		       i, rtt, rssi, snr);
	}

	if (good == 0) {
		printk("\nNo successful round trips!\n");
		return;
	}

	uint32_t min_rtt = rtt_us[0], max_rtt = rtt_us[0];
	uint64_t sum = 0;
	for (int i = 0; i < good; i++) {
		if (rtt_us[i] < min_rtt) min_rtt = rtt_us[i];
		if (rtt_us[i] > max_rtt) max_rtt = rtt_us[i];
		sum += rtt_us[i];
	}

	printk("\n--- Latency Summary ---\n");
	printk("  Successful: %d / %d\n", good, LATENCY_ITERATIONS);
	printk("  Min RTT:    %u us\n", min_rtt);
	printk("  Max RTT:    %u us\n", max_rtt);
	printk("  Avg RTT:    %u us\n", (uint32_t)(sum / good));
	printk("  Avg one-way: ~%u us\n", (uint32_t)(sum / good / 2));
}

static void run_throughput_test(void)
{
	uint8_t tx_buf[THROUGHPUT_PKT_SIZE];
	uint8_t rx_buf[255];
	int16_t rssi;
	int8_t  snr;

	printk("\n=== Throughput Test (%d packets, %d bytes each) ===\n\n",
	       THROUGHPUT_PACKETS, THROUGHPUT_PKT_SIZE);

	memset(tx_buf, 0, sizeof(tx_buf));

	uint32_t t_start = k_cycle_get_32();

	for (int i = 0; i < THROUGHPUT_PACKETS; i++) {
		struct test_pkt *pkt = (struct test_pkt *)tx_buf;
		pkt->type = PKT_DATA;
		pkt->seq = i;

		lora_config(lora_dev, &tx_cfg);
		int ret = lora_send(lora_dev, tx_buf, THROUGHPUT_PKT_SIZE);
		if (ret < 0) {
			printk("[%3d] TX failed: %d\n", i, ret);
		}

		if ((i + 1) % 20 == 0) {
			printk("  Sent %d / %d\n", i + 1, THROUGHPUT_PACKETS);
		}
	}

	uint32_t t_end = k_cycle_get_32();

	struct test_pkt done = { .type = PKT_DONE, .seq = THROUGHPUT_PACKETS };
	lora_config(lora_dev, &tx_cfg);
	lora_send(lora_dev, (uint8_t *)&done, sizeof(done));

	lora_config(lora_dev, &rx_cfg);
	int ret = lora_recv(lora_dev, rx_buf, sizeof(rx_buf),
			    K_MSEC(5000), &rssi, &snr);

	uint32_t elapsed_us = k_cyc_to_us_floor32(t_end - t_start);
	uint32_t total_bytes = THROUGHPUT_PACKETS * THROUGHPUT_PKT_SIZE;

	printk("\n--- Throughput Summary ---\n");
	printk("  Sent:      %d packets (%u bytes total)\n",
	       THROUGHPUT_PACKETS, total_bytes);
	printk("  Elapsed:   %u ms\n", elapsed_us / 1000);

	if (elapsed_us > 0) {
		uint32_t bps = (uint64_t)total_bytes * 8 * 1000000 / elapsed_us;
		printk("  TX rate:   %u bps\n", bps);
	}

	if (ret >= (int)sizeof(struct test_pkt)) {
		struct test_pkt *resp = (struct test_pkt *)rx_buf;
		printk("  Received:  %d / %d packets\n",
		       resp->seq, THROUGHPUT_PACKETS);
		printk("  Loss:      %d%%\n",
		       100 * (THROUGHPUT_PACKETS - resp->seq) / THROUGHPUT_PACKETS);
	} else {
		printk("  (No response from responder with RX count)\n");
	}
}

int main(void)
{
	wait_for_usb();

	printk("=== LoRa Pinger ===\n");

	if (!device_is_ready(lora_dev)) {
		printk("LoRa device not ready!\n");
		return 0;
	}
	printk("LoRa radio ready.\n");

	run_latency_test();
	run_throughput_test();

	printk("\n=== All tests complete ===\n");

	k_sleep(K_FOREVER);
}
