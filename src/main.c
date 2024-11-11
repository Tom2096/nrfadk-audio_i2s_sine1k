/*
	Author: Tom He
	Email: sy4he@uwaterloo.ca
*/

#include <zephyr/kernel.h>
#include <nrfx_clock.h>
#include <audio_i2s.h>
#include <audio_i2s_macros.h>
#include <zephyr/sys/ring_buffer.h>
#include <pcm_stream_channel_modifier.h>
#include <macros_common.h>
#include <hw_codec.h>
#include <tone.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

#define CONFIG_LOGGING

static uint8_t *tx_buf;
static uint32_t *rx_buf;

static uint8_t sine_1000hz_16bits_mono[BLK_MONO_SIZE_OCTETS] = { 0 };
static uint8_t sine_1000hz_16bits_stereo[BLK_STEREO_SIZE_OCTETS] = { 0 };

K_SEM_DEFINE(sem_ringbuf_space_available, 1, 1);
K_MUTEX_DEFINE(mtx_ringbuf);
RING_BUF_DECLARE(ringbuf_audio_data, RING_BUF_SIZE);

/* Alternate-buffers used when there is no active audio stream.
 * Used interchangeably by I2S.
 */
static struct {
	uint8_t __aligned(WB_UP(1)) buf_0[BLK_MONO_SIZE_OCTETS];
	uint8_t __aligned(WB_UP(1)) buf_1[BLK_MONO_SIZE_OCTETS];
	bool buf_0_in_use;
	bool buf_1_in_use;
} alt;

/**
 * @brief	Get first available alternative-buffer.
 *
 * @param	p_buffer	Double pointer to populate with buffer.
 *
 * @retval	0 if success.
 * @retval	-ENOMEM No available buffers.
 */
static int alt_buffer_get(void **p_buffer)
{
	if (!alt.buf_0_in_use) {
		alt.buf_0_in_use = true;
		*p_buffer = alt.buf_0;
	} else if (!alt.buf_1_in_use) {
		alt.buf_1_in_use = true;
		*p_buffer = alt.buf_1;
	} else {
		return -ENOMEM;
	}

	return 0;
}

/**
 * @brief	Frees both alternative buffers.
 */
static void alt_buffer_free_both(void)
{
	alt.buf_0_in_use = false;
	alt.buf_1_in_use = false;
}

static int ringbuf_write()
{
	int ret;
	uint8_t *buf;
	size_t write_size = RING_BUF_FILL_SIZE;

	/* The ringbuffer is read every 10 ms by audio datapath when SD card playback is enabled.
	 * Timeout value should therefore not be less than 10 ms
	 */
	ret = k_sem_take(&sem_ringbuf_space_available, K_MSEC(20));
	if (ret) {
		LOG_ERR("Sem take err: %d. Skipping frame", ret);
		return ret;
	}

	write_size = ring_buf_put_claim(&ringbuf_audio_data, &buf, write_size);
	memcpy(buf, sine_1000hz_16bits_stereo, write_size);
	ret = ring_buf_put_finish(&ringbuf_audio_data, write_size);
	if (ret) {
		LOG_ERR("Ring buf put finish err: %d", ret);
		return ret;
	}

	return 0;
}

static void ringbuf_read()
{
	size_t read_size = BLK_MONO_SIZE_OCTETS;

	read_size = ring_buf_get_claim(&ringbuf_audio_data, &tx_buf, read_size);
	if (read_size != BLK_MONO_SIZE_OCTETS) {
		LOG_WRN("Read size (%d) not equal requested size (%d)", read_size, BLK_MONO_SIZE_OCTETS);
	}

	/*** Data exchange ***/
	audio_i2s_set_next_buf(tx_buf, rx_buf);

	ring_buf_get_finish(&ringbuf_audio_data, read_size);

	if (ring_buf_space_get(&ringbuf_audio_data) >= RING_BUF_FILL_SIZE) {
		k_sem_give(&sem_ringbuf_space_available);
	}
}

static int hfclock_config_and_start(void)
{
	int ret;

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	nrfx_clock_hfclk_start();
	while (!nrfx_clock_hfclk_is_running()) {
	}

	return 0;
}

int main(void) 
{
    int ret;

#ifdef CONFIG_LOGGING
    printk("\nLogging I2S Info ...\n");
    printk("\nCONFIG_I2S_LRCK_FREQ_HZ:     %d\n", CONFIG_I2S_LRCK_FREQ_HZ);
    printk("\nCONFIG_I2S_CH_NUM:           %d\n", CONFIG_I2S_CH_NUM);
    printk("\nCONFIG_AUDIO_BIT_DEPTH_BITS: %d\n", CONFIG_AUDIO_BIT_DEPTH_BITS);
#endif

	ret = hfclock_config_and_start();
	ERR_CHK(ret);

	audio_i2s_blk_comp_cb_register(ringbuf_read);
	audio_i2s_init();

	ret = hw_codec_init();
	if (ret) {
		LOG_ERR("Failed to initialize HW codec: %d", ret);
		return ret;
	}
	hw_codec_volume_set(100);

	printk("\nStarting I2S!\n");

	/* Starting I2S */

	/********** I2S TX **********/
	ret = alt_buffer_get((void **)&tx_buf);
	ERR_CHK(ret);

	/********** I2S RX **********/
	ret = alt_buffer_get((void **)&rx_buf);
	ERR_CHK(ret);

	/*
		BLK_MONO_SIZE_OCTETS = 32, therefore if every sample is 16bits, then 
		one buffer will hold exactly 
		
			32 / 2 = 16 samples
		
		Since I2S expects interleaving left/right audio data, this means we can fit 
		8 unique samples in a single buffer.

		If we want to hear 1000Hz sine wave, we need to play two alternating buffers,
		each holding 8 unique samples representing half a period.  
	*/
	size_t tone_size;
	ret = tone_gen((int16_t *)sine_1000hz_16bits_mono, &tone_size, 1000, CONFIG_I2S_LRCK_FREQ_HZ, 1.0);
	if (ret || tone_size != BLK_MONO_SIZE_OCTETS) {
		LOG_ERR("Failed to generate test tone");
		return ret;
	}
	
	ret = pscm_copy_pad(sine_1000hz_16bits_mono, BLK_MONO_SIZE_OCTETS, 
							CONFIG_AUDIO_BIT_DEPTH_BITS,
							sine_1000hz_16bits_stereo,
							&tone_size);
	if (ret || tone_size != BLK_STEREO_SIZE_OCTETS) {
		LOG_ERR("Failed to generate dual tone");
	}

	memset(tx_buf, 0, BLK_MONO_SIZE_OCTETS);
	ringbuf_write();

	ret = hw_codec_default_conf_enable();
	ERR_CHK(ret);
	audio_i2s_start(tx_buf, rx_buf);
	// Need to run this manually once to trigger the callback
	audio_i2s_set_next_buf(tx_buf, rx_buf);
	
	int64_t start_time = k_uptime_get();
	while (true) {

		int64_t current_time = k_uptime_get();
		int64_t elapsed_time_ms = current_time - start_time;
		
        if (elapsed_time_ms >= 5000) { // Check if 5000ms has passed
            break; // Exit the loop after 5 seconds
        }

		ringbuf_write();
	}

	ret = hw_codec_soft_reset();
	ERR_CHK(ret);
	audio_i2s_stop();
	
	alt_buffer_free_both();

	printk("\nI2S Stopped!\n");

    return 0;
}