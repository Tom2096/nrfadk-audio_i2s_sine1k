/*
	Author: Tom He
	Email: sy4he@uwaterloo.ca
*/

#include <zephyr/kernel.h>
#include <nrfx_clock.h>
#include <audio_i2s.h>
#include <pcm_stream_channel_modifier.h>
#include <macros_common.h>
#include <hw_codec.h>
#include <tone.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

#define CONFIG_LOGGING

#define BLK_PERIOD_US 1000

/* Number of audio blocks given a duration */
#define NUM_BLKS(d) ((d) / BLK_PERIOD_US)
/* Single audio block size in number of samples (stereo) */
/* clang-format off */
#define BLK_SIZE_SAMPLES(r) (((r)*BLK_PERIOD_US) / 1000000)

#define NUM_BLKS_IN_FRAME      NUM_BLKS(CONFIG_AUDIO_FRAME_DURATION_US)
#define BLK_MONO_NUM_SAMPS     BLK_SIZE_SAMPLES(CONFIG_AUDIO_SAMPLE_RATE_HZ)

/* Number of octets in a single audio block */
#define BLK_MONO_SIZE_OCTETS   (BLK_MONO_NUM_SAMPS * CONFIG_AUDIO_BIT_DEPTH_OCTETS)
#define BLK_STEREO_SIZE_OCTETS (BLK_MONO_SIZE_OCTETS * 2)

static uint8_t *tx_buf;
static uint32_t *rx_buf;

static uint8_t alt_pos = 0;
static uint8_t sine_1000hz_16bits_mono[BLK_MONO_SIZE_OCTETS] = { 0 };
static uint8_t sine_1000hz_16bits_stereo[BLK_STEREO_SIZE_OCTETS] = { 0 };

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

static void i2s_block_complete_callback()
{
	uint8_t* input_ptr;
	
	if (alt_pos == 0) {
		input_ptr = sine_1000hz_16bits_stereo;
	} else {
		input_ptr = &(sine_1000hz_16bits_stereo[BLK_MONO_SIZE_OCTETS]);
	}
	
	/*** Data exchange ***/
	audio_i2s_set_next_buf(input_ptr, rx_buf);
	
	alt_pos = 1 - alt_pos;
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

	audio_i2s_blk_comp_cb_register(i2s_block_complete_callback);
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

	for (size_t i = 0; i < BLK_STEREO_SIZE_OCTETS / 2; ++i) {
		printk("%d\n", ((int16_t *)sine_1000hz_16bits_stereo)[i]);
	}

	ret = hw_codec_default_conf_enable();
	ERR_CHK(ret);
	audio_i2s_start(tx_buf, rx_buf);
	// Need to run this manually once to trigger the callback
	audio_i2s_set_next_buf(tx_buf, rx_buf);
	
	k_msleep(5000);

	ret = hw_codec_soft_reset();
	ERR_CHK(ret);
	audio_i2s_stop();
	
	alt_buffer_free_both();

	printk("\nI2S Stopped!\n");

    return 0;
}