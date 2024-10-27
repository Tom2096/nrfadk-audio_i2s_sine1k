#include <audio_i2s.h>

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

/* Size of ring buffer in bytes */
#define RING_BUF_SIZE 960

/* How many bytes we can fill at a time */
#define RING_BUF_FILL_SIZE 64