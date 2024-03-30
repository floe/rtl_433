/** @file
    Decoder for Watts Vision devices.

    Copyright (C) 2024 Florian 'floe' Echtler <floe@butterbrot.org>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
 */
/**
Decoder for Watts Vision devices.

Note: work in progress.

- Modulation: FSK PCM
- Frequency: 868.3MHz
- 26 us bit time
- based on TI CC1100

Payload format:
- Preamble          {32} 0xaaaaaaaa
- Syncword          {32} 0xd391d391
- Length            {8}
- Payload           {n}
- Checksum          {16} CRC16 poly=0x8005 init=0xffff
- Checksum          {16} CRC16 poly=0x8005 init=0xffff

Note that there are two CRCs, one calculated by the transceiver over the whole message
including length byte, and one calculated by the controller over just the payload.

Usual payload lengths seem to be 20 (0x14) and 34 (0x22).

To get raw data:

    ./rtl_433 -f 868.25M -X 'n=WattsVision,m=FSK_PCM,s=26,l=26,r=1000,preamble=aad391d391'
*/

#include "decoder.h"

static int watts_vision_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble[] = {
            /*0xaa, 0xaa, */ 0xaa, 0xaa, // preamble
            0xd3, 0x91, 0xd3, 0x91       // sync word
    };

    data_t *data;

    if (bitbuffer->num_rows != 1) {
        return DECODE_ABORT_EARLY;
    }

    int row = 0;
    // Validate message and reject it as fast as possible : check for preamble
    unsigned start_pos = bitbuffer_search(bitbuffer, row, 0, preamble, sizeof (preamble) * 8);

    if (start_pos == bitbuffer->bits_per_row[row]) {
        return DECODE_ABORT_EARLY; // no preamble detected
    }

    // check min length
    if (bitbuffer->bits_per_row[row] < 14 * 8) { //sync(4) + preamble(4) + len(1) + data(1) + crc(2) + crc(2)
        return DECODE_ABORT_LENGTH;
    }

    uint8_t len;
    bitbuffer_extract_bytes(bitbuffer, row, start_pos + sizeof (preamble) * 8, &len, 8);

    // usual lengths seem to be 20 (0x14) and 34 (0x22).
    if (len > 50) {
        decoder_logf(decoder, 1, __func__, "packet to large (%d bytes), drop it", len);
        return DECODE_ABORT_LENGTH;
    }

    uint8_t frame[50] = {0}; // arbitrary limit of 1 len byte + 45 data bytes + 2*2 bytes crc
    frame[0] = len;

    // Get frame (len doesn't include the length byte or the crc16 bytes)
    bitbuffer_extract_bytes(bitbuffer, row,
            start_pos + (sizeof (preamble) + 1) * 8,
            &frame[1], (len + 2) * 8);

    decoder_log_bitrow(decoder, 2, __func__, frame, (len + 1) * 8, "frame data");

    // use the inner CRC to verify the message, as the outer CRC at the very end of the 
    // message may sometimes lose a few bits off the end
    uint16_t crc = crc16(frame + 1, len - 1, 0x8005, 0xffff);

    if ((frame[len + 1] << 8 | frame[len + 2]) != crc) {
        decoder_logf(decoder, 1, __func__, "CRC invalid %04x != %04x",
                frame[len + 1] << 8 | frame[len + 2], crc);
        return DECODE_FAIL_MIC;
    }

    char frame_str[sizeof(frame) * 2 + 1]   = {0};
    for (int i = 0; i < len; ++i)
        sprintf(&frame_str[i * 2], "%02x", frame[i + 1]);

    /* clang-format off */
    data = data_make(
            "model",            "",             DATA_STRING, "WattsVision",
            "raw",              "Raw data",     DATA_STRING, frame_str,
            "mic",              "Integrity",    DATA_STRING, "CRC",
            NULL);
    /* clang-format on */
    decoder_output_data(decoder, data);
    return 1;
}

static char const *const output_fields[] = {
        "model",
        "raw",
        "mic",
        NULL,
};

r_device const watts_vision = {
        .name        = "Watts Vision thermostats",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 26,
        .long_width  = 26,
        .reset_limit = 1000,
        .decode_fn   = &watts_vision_decode,
        .fields      = output_fields,
};
