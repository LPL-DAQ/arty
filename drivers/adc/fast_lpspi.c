/// Faster LPSPI tranceive.
///
/// The built-in nxp,lpspi driver's transceive function incurs overhead in each transaction, primarily from:
/// - Acquiring a lock on the SPI context
/// - Yielding after initiating the SPI frame, thus requiring the initiating thread to be rescheduled
///
/// Given our use case for the ADS7953 driver, where we initiate many small, rapid transactions, one after
/// the other, the aforementioned overhead builds up quite a bit. We thus implement our own variant of the
/// transceive function, which mitigates these sources of delay by busy waiting and by locking the SPI
/// context once for multiple transactions.

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(fast_lpspi, CONFIG_LOG_DEFAULT_LEVEL);

#include "../../../modules/hal/nxp/mcux/mcux-sdk-ng/drivers/lpspi/fsl_lpspi.h"
#include "../../../zephyr/drivers/spi/spi_context.h"
#include "../../../zephyr/drivers/spi/spi_nxp_lpspi/spi_nxp_lpspi_priv.h"

/* simple macro for readability of the equations used in the clock configuring */
#define TWO_EXP(power) BIT(power)

struct lpspi_driver_data {
    size_t total_words_to_clock;
    size_t words_clocked;
    uint8_t word_size_bytes;
    uint8_t lpspi_op_mode;
};

static inline uint8_t rx_fifo_cur_len(LPSPI_Type* base)
{
    return (base->FSR & LPSPI_FSR_RXCOUNT_MASK) >> LPSPI_FSR_RXCOUNT_SHIFT;
}

static inline uint8_t tx_fifo_cur_len(LPSPI_Type* base)
{
    return (base->FSR & LPSPI_FSR_TXCOUNT_MASK) >> LPSPI_FSR_TXCOUNT_SHIFT;
}

/* Reads a word from the RX fifo and handles writing it into the RX spi buf */
static inline void lpspi_rx_word_write_bytes(const struct device* dev, size_t offset)
{
    LPSPI_Type* base = (LPSPI_Type*)DEVICE_MMIO_NAMED_GET(dev, reg_base);
    struct lpspi_data* data = dev->data;
    struct lpspi_driver_data* lpspi_data = (struct lpspi_driver_data*)data->driver_data;
    struct spi_context* ctx = &data->ctx;
    uint8_t num_bytes = lpspi_data->word_size_bytes;
    uint8_t* buf = ctx->rx_buf + offset;
    uint32_t word = base->RDR;

    if (!spi_context_rx_buf_on(ctx) && spi_context_rx_on(ctx)) {
        /* receive no actual data if rx buf is NULL */
        return;
    }

    for (uint8_t i = 0; i < num_bytes; i++) {
        buf[i] = (uint8_t)(word >> (BITS_PER_BYTE * i));
    }
}

/* Reads a maximum number of words from RX fifo and writes them to the remainder of the RX buf */
static inline size_t lpspi_rx_buf_write_words(const struct device* dev, uint8_t max_read)
{
    struct lpspi_data* data = dev->data;
    struct lpspi_driver_data* lpspi_data = (struct lpspi_driver_data*)data->driver_data;
    struct spi_context* ctx = &data->ctx;
    size_t buf_len = ctx->rx_len;
    uint8_t words_read = 0;

    size_t offset = 0;

    while (buf_len-- > 0 && max_read-- > 0) {
        lpspi_rx_word_write_bytes(dev, offset);
        offset += lpspi_data->word_size_bytes;
        words_read++;
    }

    return words_read;
}

static inline void lpspi_handle_rx_irq(const struct device* dev)
{
    LPSPI_Type* base = (LPSPI_Type*)DEVICE_MMIO_NAMED_GET(dev, reg_base);
    struct lpspi_data* data = dev->data;
    struct lpspi_driver_data* lpspi_data = (struct lpspi_driver_data*)data->driver_data;
    struct spi_context* ctx = &data->ctx;
    uint8_t total_words_written = 0;
    uint8_t total_words_read = 0;
    uint8_t words_read;
    uint8_t rx_fsr;

    base->SR = LPSPI_SR_RDF_MASK;

    LOG_DBG("RX FIFO: %d, RX BUF: %p", rx_fsr, ctx->rx_buf);

    while ((rx_fsr = rx_fifo_cur_len(base)) > 0 && spi_context_rx_on(ctx)) {
        words_read = lpspi_rx_buf_write_words(dev, rx_fsr);
        total_words_read += words_read;
        total_words_written += (spi_context_rx_buf_on(ctx) ? words_read : 0);
        spi_context_update_rx(ctx, lpspi_data->word_size_bytes, words_read);
    }

    LOG_DBG("RX done %d words to spi buf", total_words_written);
}

/* constructs the next word from the buffer */
static inline uint32_t lpspi_next_tx_word(const struct device* dev, const uint8_t* buf, int offset, size_t max_bytes)
{
    const uint8_t* byte = buf + offset;
    uint32_t next_word = 0;

    for (uint8_t i = 0; i < max_bytes; i++) {
        next_word |= byte[i] << (BITS_PER_BYTE * i);
    }

    return next_word;
}

/* fills the TX fifo with specified amount of data from the specified buffer */
static inline void lpspi_fill_tx_fifo(const struct device* dev, const uint8_t* buf, size_t buf_len, size_t fill_len)
{
    LPSPI_Type* base = (LPSPI_Type*)DEVICE_MMIO_NAMED_GET(dev, reg_base);
    struct lpspi_data* data = dev->data;
    struct lpspi_driver_data* lpspi_data = (struct lpspi_driver_data*)data->driver_data;
    uint8_t word_size = lpspi_data->word_size_bytes;
    size_t offset = 0;
    uint32_t next_word;
    uint32_t next_word_bytes;

    for (int word_count = 0; word_count < fill_len; word_count++) {
        next_word_bytes = MIN(word_size, buf_len);
        next_word = lpspi_next_tx_word(dev, buf, offset, next_word_bytes);
        base->TDR = next_word;
        offset += word_size;
        buf_len -= word_size;
    }

    lpspi_data->words_clocked += fill_len;
    LOG_DBG("Filled TX FIFO to %d words (%d bytes)", fill_len, offset);
}

/* This is the equation for the sck frequency given a div and prescaler. */
static uint32_t lpspi_calc_sck_freq(uint32_t src_clk_hz, uint16_t sckdiv, uint8_t prescaler)
{
    return (uint32_t)(src_clk_hz / (TWO_EXP(prescaler) * (sckdiv + 2)));
}

static inline uint8_t lpspi_calc_best_div_for_prescaler(uint32_t src_clk_hz, uint8_t prescaler, uint32_t req_freq)
{
    uint64_t prescaled_req_freq = TWO_EXP(prescaler) * req_freq;
    uint64_t ratio;

    if (prescaled_req_freq == 0) {
        ratio = UINT8_MAX + 2;
    }
    else {
        ratio = DIV_ROUND_UP(src_clk_hz, prescaled_req_freq);
    }

    ratio = MAX(ratio, 2);
    ratio -= 2;
    ratio = MIN(ratio, UINT8_MAX);

    return (uint8_t)ratio;
}

static inline int lpspi_validate_xfer_args(const struct spi_config* spi_cfg)
{
    uint32_t word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
    uint32_t pcs = spi_cfg->slave;

    if (spi_cfg->operation & SPI_HALF_DUPLEX) {
        /* the IP DOES support half duplex, need to implement driver support */
        LOG_WRN("Half-duplex not supported");
        return -ENOTSUP;
    }

    if (word_size < 2 || (word_size % 32 == 1)) {
        /* Zephyr word size == hardware FRAME size (not word size)
         * Max frame size: 4096 bits
         *   (zephyr field is 6 bit wide for max 64 bit size, no need to check)
         * Min frame size: 8 bits.
         * Minimum hardware word size is 2. Since this driver is intended to work
         * for 32 bit platforms, and 64 bits is max size, then only 33 and 1 are invalid.
         */
        LOG_WRN("Word size %d not allowed", word_size);
        return -EINVAL;
    }

    if (pcs > LPSPI_CHIP_SELECT_COUNT - 1) {
        LOG_WRN("Peripheral %d select exceeds max %d", pcs, LPSPI_CHIP_SELECT_COUNT - 1);
        return -EINVAL;
    }

    return 0;
}

static uint8_t lpspi_calc_delay_scaler(uint32_t desired_delay_ns, uint32_t prescaled_clock, uint32_t min_cycles)
{
    uint64_t delay_cycles;

    /* calculates the number of functional clock cycles needed to achieve delay */
    delay_cycles = (uint64_t)prescaled_clock * desired_delay_ns;
    delay_cycles = DIV_ROUND_UP(delay_cycles, NSEC_PER_SEC);

    /* what the min_cycles parameter is about is that
     * PCSSCK and SCKPSC are +1 cycles of the programmed value,
     * while DBT is +2 cycles of the programmed value.
     * So this calculates the value to program to the register.
     */
    // PATCH: Prevent underflow.
    delay_cycles = MAX(delay_cycles, min_cycles) - min_cycles;

    /* Don't overflow */
    delay_cycles = MIN(delay_cycles, UINT8_MAX);

    return (uint8_t)delay_cycles;
}

/* This function configures everything except the TCR and the clock scaler */
static void lpspi_basic_config(const struct device* dev, const struct spi_config* spi_cfg)
{
    const struct lpspi_config* config = dev->config;
    LPSPI_Type* base = (LPSPI_Type*)DEVICE_MMIO_NAMED_GET(dev, reg_base);
    uint32_t pcs_control_bit = 1 << (LPSPI_CFGR1_PCSPOL_SHIFT + spi_cfg->slave);
    uint32_t cfgr1_val = 0;

    if (spi_cfg->operation & SPI_CS_ACTIVE_HIGH) {
        cfgr1_val |= pcs_control_bit;
    }
    else {
        cfgr1_val &= ~pcs_control_bit;
    }

    if (SPI_OP_MODE_GET(spi_cfg->operation) == SPI_OP_MODE_MASTER) {
        cfgr1_val |= LPSPI_CFGR1_MASTER_MASK;
    }

    if (config->tristate_output) {
        cfgr1_val |= LPSPI_CFGR1_OUTCFG_MASK;
    }

    cfgr1_val |= config->data_pin_config << LPSPI_CFGR1_PINCFG_SHIFT;

    base->CFGR1 = cfgr1_val;

    if (IS_ENABLED(CONFIG_DEBUG)) {
        /* DEBUG mode makes it so the lpspi does not keep
         * running while debugger has halted the chip.
         * This makes debugging spi transfers easier.
         */
        base->CR |= LPSPI_CR_DBGEN_MASK;
    }
}

/* This function configures the clock control register (CCR) for the desired frequency
 * It does a binary search for the optimal CCR divider and TCR prescaler.
 * The prescale_value parameter is changed to the best value of the prescaler,
 * for use in setting the TCR outside this function.
 * The return value is the mask of the CCR (bits 0-7) required to set SCKDIV for best result.
 */
static inline uint32_t lpspi_set_sckdiv(uint32_t desired_freq, uint32_t clock_freq, uint8_t* prescale_value)
{
    uint8_t best_prescaler = 0, best_div = 0;
    uint32_t best_freq = 0;

    for (int8_t prescaler = 7U; prescaler >= 0; prescaler--) {
        /* if maximum freq (div = 0) won't get better than what we got with
         * previous prescaler, then we can fast path exit this loop.
         */
        if (lpspi_calc_sck_freq(clock_freq, 0, prescaler) < best_freq) {
            break;
        }

        /* the algorithm approaches the desired freq from below intentionally,
         * therefore the min is our previous best and the max is the desired.
         */
        uint8_t new_div = lpspi_calc_best_div_for_prescaler(clock_freq, prescaler, desired_freq);
        uint32_t new_freq = lpspi_calc_sck_freq(clock_freq, new_div, prescaler);

        if (new_freq >= best_freq && new_freq <= desired_freq) {
            best_div = new_div;
            best_freq = new_freq;
            best_prescaler = prescaler;
        }
    }

    *prescale_value = best_prescaler;

    return LPSPI_CCR_SCKDIV(best_div);
}

/* returns CCR mask of the bits 8-31 */
static inline uint32_t lpspi_set_delays(const struct device* dev, uint32_t prescaled_clock)
{
    const struct lpspi_config* config = dev->config;

    // LOG_ERR("sck delay: %d", lpspi_calc_delay_scaler(config->pcs_sck_delay,
    // 						prescaled_clock, 1));
    // LOG_ERR("sck_pcs_delay delay: %d", config->sck_pcs_delay);
    // LOG_ERR("transfer_delay", config->transfer_delay);

    return LPSPI_CCR_PCSSCK(lpspi_calc_delay_scaler(config->pcs_sck_delay, prescaled_clock, 1)) |
           LPSPI_CCR_SCKPCS(lpspi_calc_delay_scaler(config->sck_pcs_delay, prescaled_clock, 1)) |
           LPSPI_CCR_DBT(lpspi_calc_delay_scaler(config->transfer_delay, prescaled_clock, 2));
}

/// Checks if the previously used SPI config is identical to the config for the upcoming frame.
/// Unlike the original spi_context_configured, we *ignore* the chip select/slave fields as we handle
/// that independently.
static inline bool fast_lpspi_context_configured(struct spi_context* ctx, const struct spi_config* new_config)
{
    const struct spi_config* old_config = ctx->config;

    return old_config->frequency == new_config->frequency && old_config->operation == new_config->operation;
}

/// Identical to lpspi_configure, but uses fast_lpspi_context_configured instead of spi_context_configured
/// when checking if reconfiguration can be skipped.
int fast_lpspi_configure(const struct device* dev, const struct spi_config* spi_cfg)
{
    struct lpspi_data* data = dev->data;
    struct spi_context* ctx = &data->ctx;
    bool already_configured = fast_lpspi_context_configured(ctx, spi_cfg);
    LPSPI_Type* base = (LPSPI_Type*)DEVICE_MMIO_NAMED_GET(dev, reg_base);
    uint32_t word_size = SPI_WORD_SIZE_GET(spi_cfg->operation);
    uint32_t clock_freq = 0;
    uint8_t prescaler = 0;
    int ret = 0;

    /* fast path to avoid reconfigure */
    /* TODO: S32K3 errata ERR050456 requiring module reset before every transfer,
     * investigate alternative workaround so we don't have this latency for S32.
     */
    if (already_configured && !IS_ENABLED(CONFIG_SOC_FAMILY_NXP_S32)) {
        return 0;
    }

    ret = lpspi_validate_xfer_args(spi_cfg);
    if (ret) {
        return ret;
    }

    /* For the purpose of configuring the LPSPI, 8 is the minimum frame size for the hardware */
    word_size = MAX(word_size, 8);

    /* specific driver implementation should set up watermarks and interrupts.
     * we reset them here to avoid any unexpected events during configuring.
     */
    base->FCR = 0;
    base->IER = 0;

    /* this is workaround for ERR050456 */
    base->CR |= LPSPI_CR_RST_MASK;
    base->CR |= LPSPI_CR_RRF_MASK | LPSPI_CR_RTF_MASK;

    /* Setting the baud rate requires module to be disabled. */
    base->CR = 0;
    while ((base->CR & LPSPI_CR_MEN_MASK) != 0) {
        /* According to datasheet, should wait for this MEN bit to clear once idle */
    }

    data->ctx.config = spi_cfg;

    lpspi_basic_config(dev, spi_cfg);

    clock_freq = data->clock_freq;

    if (SPI_OP_MODE_GET(spi_cfg->operation) == SPI_OP_MODE_MASTER) {
        uint32_t ccr = 0;

        /* sckdiv algorithm must run *before* delays are set in order to know prescaler */
        ccr |= lpspi_set_sckdiv(spi_cfg->frequency, clock_freq, &prescaler);
        ccr |= lpspi_set_delays(dev, clock_freq / TWO_EXP(prescaler));

        /* note that not all bits of the register are readable on some platform,
         * that's why we update it on one write
         */
        base->CCR = ccr;
    }

    base->CR |= LPSPI_CR_MEN_MASK;

    base->TCR = LPSPI_TCR_CPOL(!!(spi_cfg->operation & SPI_MODE_CPOL)) | LPSPI_TCR_CPHA(!!(spi_cfg->operation & SPI_MODE_CPHA)) |
                LPSPI_TCR_LSBF(!!(spi_cfg->operation & SPI_TRANSFER_LSB)) | LPSPI_TCR_FRAMESZ(word_size - 1) | LPSPI_TCR_PRESCALE(prescaler) |
                LPSPI_TCR_PCS(spi_cfg->slave);

    return lpspi_wait_tx_fifo_empty(dev);
}

/// Initiate an SPI frame, blocking till it is completed. Based on nxp,lpspi's implementation.
/// fast_lpspi_lock_spi must be called beforehand.
int fast_lpspi_transceive_dt(const struct spi_dt_spec* spec, const struct spi_buf_set* tx_bufs, const struct spi_buf_set* rx_bufs)
{
    const struct device* dev = spec->bus;
    const struct spi_config* spi_cfg = &spec->config;
    LPSPI_Type* base = (LPSPI_Type*)DEVICE_MMIO_NAMED_GET(dev, reg_base);
    const struct lpspi_config* config = dev->config;
    struct lpspi_data* data = dev->data;
    struct lpspi_driver_data* lpspi_data = (struct lpspi_driver_data*)data->driver_data;
    struct spi_context* ctx = &data->ctx;
    uint8_t op_mode = SPI_OP_MODE_GET(spi_cfg->operation);
    int ret = 0;

    lpspi_data->word_size_bytes = DIV_ROUND_UP(SPI_WORD_SIZE_GET(spi_cfg->operation), BITS_PER_BYTE);
    if (lpspi_data->word_size_bytes > 4) {
        LOG_ERR("Maximum 4 byte word size");
        ret = -EINVAL;
        goto error;
    }

    if (op_mode == SPI_OP_MODE_SLAVE && !(spi_cfg->operation & SPI_MODE_CPHA)) {
        LOG_ERR("CPHA=0 not supported with LPSPI peripheral mode");
        ret = -ENOTSUP;
        goto error;
    }

    if (data->major_version < 2 && spi_cfg->operation & SPI_HOLD_ON_CS) {
        /* on this version of LPSPI, due to errata in design
         * CS must be deasserted in order to clock all words,
         * so HOLD_ON_CS flag cannot be supported.
         */
        return -EINVAL;
    }

    // spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, lpspi_data->word_size_bytes);
    lpspi_data->lpspi_op_mode = op_mode;

    ret = fast_lpspi_configure(dev, spi_cfg);
    if (ret) {
        goto error;
    }

    if (ctx->config && spi_cs_is_gpio(ctx->config)) {
        gpio_pin_set_dt(&ctx->config->cs.gpio, 1);
    }

    if (op_mode == SPI_OP_MODE_MASTER) {
        /* set watermarks to 0 so get tx interrupt when fifo empty
         * and rx interrupt when any data received
         */
        base->FCR = 0;
    }
    else {
        /* set watermarks so that we are as responsive to master as possible and don't
         * miss any communication. This means RX interrupt at 0 so that if we ever
         * get any data, we get interrupt and handle immediately, and TX interrupt
         * to one less than the max of the fifo (-2 of size) so that we have as much
         * data ready to send to master as possible at any time
         */
        base->FCR = LPSPI_FCR_TXWATER(config->tx_fifo_size - 1);
        base->CFGR1 |= LPSPI_CFGR1_AUTOPCS_MASK;
    }

    base->CR |= LPSPI_CR_MEN_MASK;

    if (tx_bufs->count != 1) {
        LOG_ERR("Only one tx buf may be specified, no scatter-gather");
        goto error;
    }
    if (rx_bufs->count != 1) {
        LOG_ERR("Only one rx buf may be specified, no scatter-gather");
        goto error;
    }
    const struct spi_buf* tx_buf = &(tx_bufs->buffers[0]);
    const struct spi_buf* rx_buf = &(rx_bufs->buffers[0]);
    if (tx_buf->len != rx_buf->len) {
        LOG_ERR("tx and rx bufs must have same length");
        goto error;
    }

    lpspi_transfer_t transfer = {.txData = tx_buf->buf, .rxData = rx_buf->buf, .dataSize = tx_buf->len, .configFlags = kLPSPI_MasterPcs0};
    ret = LPSPI_MasterTransferBlocking(base, &transfer);

    if (ret) {
        LOG_ERR("Failed to perform lpspi transfer blocking, see MCUXpresso API for error code: %d", ret);
    }

    if (ctx->config && spi_cs_is_gpio(ctx->config)) {
        gpio_pin_set_dt(&ctx->config->cs.gpio, 0);
    }

error:
    return ret;
}

// Reserve the LPSPI bus for a series of fast transceives.
void fast_lpspi_lock_spi(const struct spi_dt_spec* spec)
{
    struct lpspi_data* data = spec->bus->data;

    spi_context_lock(&data->ctx, false, NULL, NULL, &spec->config);
}

// Release the LPSPI bus after a series of fast transceives. Must be called after fast_lpspi_lock_spi.
void fast_lpspi_release_spi(const struct spi_dt_spec* spec)
{
    struct lpspi_data* data = spec->bus->data;

    spi_context_release(&data->ctx, 0);
}
