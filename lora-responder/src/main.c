#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <string.h>

#define LORA_FREQUENCY       915000000
#define LORA_TX_POWER        2
#define LORA_PREAMBLE_LEN    8

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

static void init_usb(void)
{
	usb_enable(NULL);
	k_sleep(K_MSEC(1000));
}

int main(void)
{
	init_usb();

	printk("=== LoRa Responder ===\n");

	if (!device_is_ready(lora_dev)) {
		printk("LoRa device not ready!\n");
		return 0;
	}
	printk("LoRa radio ready. Waiting for packets...\n");

	uint8_t rx_buf[255];
	int16_t rssi;
	int8_t  snr;
	int throughput_rx_count = 0;

	while (1) {
		lora_config(lora_dev, &rx_cfg);
		int len = lora_recv(lora_dev, rx_buf, sizeof(rx_buf),
				    K_FOREVER, &rssi, &snr);

		if (len < (int)sizeof(struct test_pkt)) {
			printk("RX: too short (%d bytes)\n", len);
			continue;
		}

		struct test_pkt *pkt = (struct test_pkt *)rx_buf;

		if (pkt->type == PKT_PING) {
			struct test_pkt pong = {
				.type = PKT_PONG,
				.seq = pkt->seq,
			};

			lora_config(lora_dev, &tx_cfg);
			int ret = lora_send(lora_dev, (uint8_t *)&pong,
					    sizeof(pong));

			printk("[%3d] PING -> PONG  RSSI: %d  SNR: %d  %s\n",
			       pkt->seq, rssi, snr,
			       ret < 0 ? "TX FAIL" : "OK");

		} else if (pkt->type == PKT_DATA) {
			throughput_rx_count++;

			if (throughput_rx_count % 20 == 0) {
				printk("  Throughput RX: %d packets so far\n",
				       throughput_rx_count);
			}

		} else if (pkt->type == PKT_DONE) {
			printk("Throughput test done. Received %d / %d packets.\n",
			       throughput_rx_count, pkt->seq);

			struct test_pkt resp = {
				.type = PKT_DONE,
				.seq = throughput_rx_count,
			};

			k_sleep(K_MSEC(100));
			lora_config(lora_dev, &tx_cfg);
			lora_send(lora_dev, (uint8_t *)&resp, sizeof(resp));

			throughput_rx_count = 0;
			printk("Waiting for packets...\n");
		}
	}
}
