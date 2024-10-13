# nRF5340 Audio I2S - Sine 1000HZ

This example leverages the existing I2S functions in the nrf5340_audio application to play a 1000Hz sine wave. It relies on `audio_i2s_set_next_buf()` and `audio_i2s_blk_comp_cb_register` to register a callback and continuously fills the I2S buffers with new audio data. 

## References

- [nRF5340 Audio Application](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/applications/nrf5340_audio/index.html)
