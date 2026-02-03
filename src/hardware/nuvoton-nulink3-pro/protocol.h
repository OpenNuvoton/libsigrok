/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2026 Nuvoton Technology Corp.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_NUVOTON_NULINK3_PRO_PROTOCOL_H
#define LIBSIGROK_HARDWARE_NUVOTON_NULINK3_PRO_PROTOCOL_H

#include <glib.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX         "nuvoton-nulink3-pro"

#define USB_PACKET_SIZE    (512)
#define USB_TIMEOUT        (5 * 1000)

#define MINOR_VERSION      0x0100

enum nubridge_edge_modes
{
    EDGE_RISING,
    EDGE_FALLING,
    EDGE_BOTH,
};

struct nubridge_version
{
    uint32_t major;
    uint32_t minor;
};

struct nubridge_mode
{
    uint8_t flags;
};

struct nubridge_config
{
    uint32_t mode;
    uint32_t sample_rate;
    uint32_t buffer_size;
    uint32_t channel_en;

    uint32_t trig_en;
    uint16_t trig_mask0;
    uint16_t trig_mask1;
    uint16_t trig_value0;
    uint16_t trig_value1;
    uint16_t trig_edge0;
    uint16_t trig_edge1;
};

struct nubridge_trigger_pos
{
    uint32_t check_id;
    uint32_t channel_en;
    uint32_t real_pos;
    uint32_t buffer_saddr;
    uint32_t buffer_size;
    uint32_t status;
    uint8_t reserved[488];
};

struct nubridge_profile
{
    uint16_t vid;
    uint16_t pid;
    uint32_t interface;

    const char *vendor;
    const char *model;
    const char *model_version;

    uint32_t dev_caps;
    uint64_t mem_depth;

    uint32_t channel_count;
};

struct dev_context
{
    const struct nubridge_profile *profile;

    const uint64_t *samplerates;
    uint32_t num_samplerates;

    uint64_t cur_samplerate;
    uint64_t limit_samples;

    gboolean acq_aborted;

    uint32_t sent_samples;
    uint32_t submitted_transfers;

    uint32_t num_transfers;
    struct libusb_transfer **transfers;
    struct sr_context *ctx;

    uint8_t *deinterleave_buffer;

    uint16_t mode;
    uint32_t trigger_pos;
    uint32_t clock_edge;

    uint32_t channel_mask;
    uint32_t channel_count;

    uint8_t endpoint_in;
    uint8_t endpoint_out;
};

SR_PRIV int32_t nubridge_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di);
SR_PRIV struct dev_context *nubridge_dev_new(void);
SR_PRIV int32_t nubridge_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int32_t nubridge_acquisition_stop(struct sr_dev_inst *sdi);

#endif
