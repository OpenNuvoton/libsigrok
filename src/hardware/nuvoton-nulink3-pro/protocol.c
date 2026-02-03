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

#include <config.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>
#include <glib.h>
#include <glib/gstdio.h>
#include <glib-2.0/glib.h>
#include "protocol.h"

#ifdef _WIN32
#include <windows.h>
#endif

#define CMD_GET_FW_VERSION      0xA0
#define CMD_START               0xA1
#define CMD_CONFIG              0xA2
#define CMD_SETTING             0xA3

#define START_FLAGS_START       (1 << 0)
#define START_FLAGS_STOP        (1 << 1)

#define MODE_TRIG_EN            (1 << 0)

#define MAX_LOGIC_SAMPLERATE    SR_MHZ(22) //2CH

#define ATOMIC_SAMPLES          (sizeof(uint8_t) * 8)
#define ATOMIC_BYTES            sizeof(uint8_t)

#define TRIGGER_HEADER_ID       0x55AAAA55

static uint32_t _get_tick_ms()
{
#ifdef _WIN32
    return GetTickCount();
#else
    struct timespec time = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (time.tv_sec * 1000) + (time.tv_nsec / 1000000);
#endif
}

static int32_t _command_write_op(libusb_device_handle *devhdl, uint8_t endpoint_out, uint32_t index, uint32_t command, uint8_t *data, uint32_t size)
{
    int32_t ret, transferred;
    uint32_t send[USB_PACKET_SIZE / 4] = {index, command, size};

    if (data != NULL)
        memcpy(&send[3], data, MIN(size, USB_PACKET_SIZE - 12));

    if ((ret = libusb_bulk_transfer(devhdl, endpoint_out, (uint8_t *)send, USB_PACKET_SIZE, &transferred, USB_TIMEOUT)) < 0)
        return -1;

    return 0;
}

static int32_t _command_read_op(libusb_device_handle *devhdl, uint8_t endpoint_in, uint32_t index, uint32_t command, uint8_t *data, uint32_t *size, uint32_t timeout)
{
    int32_t ret, transferred;
    uint32_t len;
    uint32_t recv[USB_PACKET_SIZE / 4] = {0};
    uint32_t tick = _get_tick_ms();

    while (1)
    {
        if ((ret = libusb_bulk_transfer(devhdl, endpoint_in, (uint8_t *)recv, USB_PACKET_SIZE, &transferred, USB_TIMEOUT)) < 0)
            return -1;

        if ((transferred == USB_PACKET_SIZE) && (recv[0] == index))
        {
            if (recv[1] == command)
                break;
            else if (recv[1] == 0)
                return -1;
        }

        if ((_get_tick_ms() - tick) > timeout)
            return -1;
    }

    len = MIN(recv[2], USB_PACKET_SIZE - 12);

    if (data != NULL)
        memcpy(data, &recv[3], len);

    if (size != NULL)
        *size = len;

    return 0;
}

static int32_t command_proc(const struct dev_context *devc, libusb_device_handle *devhdl, uint32_t command, uint8_t *data, uint32_t size, uint8_t *res, uint32_t *rsize, bool wait)
{
    static uint32_t index = 0x1;

    index++;

    if (_command_write_op(devhdl, devc->endpoint_out, index, command, data, size) < 0)
        return -1;

    if (wait && (_command_read_op(devhdl, devc->endpoint_in, index, command, res, rsize, USB_TIMEOUT * 2) < 0))
        return -1;

    return 0;
}

static int32_t command_get_version(const struct dev_context *devc, libusb_device_handle *devhdl, struct nubridge_version *vi)
{
    int32_t ret;
    uint32_t size;;

    ret = command_proc(devc, devhdl, CMD_GET_FW_VERSION, NULL, 0, (uint8_t *)vi, &size, true);

    if (ret < 0 || size < sizeof(struct nubridge_version))
    {
        sr_err("Failed to get version.");
        return SR_ERR;
    }

    return SR_OK;
}

static int32_t command_start_acquisition(const struct sr_dev_inst *sdi)
{
    int32_t ret;
    const struct dev_context *devc;
    struct sr_usb_dev_inst *usb;
    struct nubridge_mode mode;

    mode.flags = START_FLAGS_START;

    devc = sdi->priv;
    usb = sdi->conn;

    ret = command_proc(devc, usb->devhdl, CMD_START, (uint8_t *)&mode, sizeof(struct nubridge_mode), NULL, NULL, true);

    if (ret < 0)
    {
        sr_err("Failed to start acquisition.");
        return SR_ERR;
    }

    return SR_OK;
}

static int32_t command_stop_acquisition(const struct sr_dev_inst *sdi)
{
    int32_t ret;
    const struct dev_context *devc;
    struct sr_usb_dev_inst *usb;
    struct nubridge_mode mode;

    mode.flags = START_FLAGS_STOP;

    devc = sdi->priv;
    usb = sdi->conn;

    ret = command_proc(devc, usb->devhdl, CMD_START, (uint8_t *)&mode, sizeof(struct nubridge_mode), NULL, NULL, false);

    if (ret < 0)
    {
        sr_err("Failed to stop acquisition.");
        return SR_ERR;
    }

    return SR_OK;
}

static uint32_t enabled_channel_count(const struct sr_dev_inst *sdi)
{
    unsigned int count = 0;

    for (const GSList *l = sdi->channels; l; l = l->next)
    {
        const struct sr_channel *const probe = (struct sr_channel *)l->data;
        if (probe->enabled)
            count++;
    }

    return count;
}

static uint32_t enabled_channel_mask(const struct sr_dev_inst *sdi)
{
    uint32_t mask = 0;

    for (const GSList *l = sdi->channels; l; l = l->next)
    {
        const struct sr_channel *const probe = (struct sr_channel *)l->data;
        if (probe->enabled)
            mask |= 1 << probe->index;
    }

    return mask;
}

static bool set_trigger(const struct sr_dev_inst *sdi, struct nubridge_config *cfg)
{
    struct sr_trigger *trigger;
    struct sr_trigger_stage *stage;
    struct sr_trigger_match *match;
    const GSList *l, *m;
    int32_t num_trigger_stages = 0;
    int32_t channel_bit;

    cfg->channel_en = enabled_channel_mask(sdi);
    cfg->trig_en = 0;

    cfg->trig_mask0 = 0xFFFF;
    cfg->trig_mask1 = 0xFFFF;
    cfg->trig_value0 = 0;
    cfg->trig_value1 = 0;
    cfg->trig_edge0 = 0;
    cfg->trig_edge1 = 0;

    if (!(trigger = sr_session_trigger_get(sdi->session)))
        return false;

    for (l = trigger->stages; l; l = l->next)
    {
        stage = l->data;
        num_trigger_stages++;

        for (m = stage->matches; m; m = m->next)
        {
            match = m->data;
            if (!match->channel->enabled)
                continue;

            cfg->trig_en |= 1 << (match->channel->index);
            channel_bit = 1 << (match->channel->index);

            if (match->match == SR_TRIGGER_FALLING)
            {
                cfg->trig_mask0 &= ~channel_bit;
                cfg->trig_mask1 &= ~channel_bit;
                cfg->trig_edge0 |= channel_bit;
                cfg->trig_edge1 |= channel_bit;
            }
            else if (match->match == SR_TRIGGER_RISING)
            {
                cfg->trig_mask0 &= ~channel_bit;
                cfg->trig_mask1 &= ~channel_bit;
                cfg->trig_value0 |= channel_bit;
                cfg->trig_value1 |= channel_bit;
                cfg->trig_edge0 |= channel_bit;
                cfg->trig_edge1 |= channel_bit;
            }
            else if (match->match == SR_TRIGGER_EDGE)
            {
                cfg->trig_edge0 |= channel_bit;
                cfg->trig_edge1 |= channel_bit;
            }
        }
    }

    return num_trigger_stages != 0;
}

static int32_t nubridge_configure(const struct sr_dev_inst *sdi)
{
    const struct dev_context *devc;
    const struct sr_usb_dev_inst *usb;
    struct nubridge_config cfg;
    uint16_t mode = 0;
    uint32_t count = enabled_channel_count(sdi);
    int32_t ret = -1;

    devc = sdi->priv;
    usb = sdi->conn;

    if (set_trigger(sdi, &cfg))
        mode |= MODE_TRIG_EN;

    cfg.mode = mode;
    cfg.sample_rate = devc->cur_samplerate;
    cfg.buffer_size = devc->profile->mem_depth;

    do
    {
        if (cfg.sample_rate > MAX_LOGIC_SAMPLERATE)
            break;

        if (count <= 2)
        {
            if (cfg.sample_rate > (MAX_LOGIC_SAMPLERATE / 1))
                break;
        }
        else if (count <= 4)
        {
            if (cfg.sample_rate > (MAX_LOGIC_SAMPLERATE / 2))
                break;
        }
        else if (count <= 8)
        {
            if (cfg.sample_rate > (MAX_LOGIC_SAMPLERATE / 4))
                break;
        }

        ret = command_proc(devc, usb->devhdl, CMD_SETTING, (uint8_t *)&cfg, sizeof(struct nubridge_config), NULL, NULL, true);
    }
    while(0);

    if (ret < 0)
    {
        sr_err("Failed to configure device.");
        return SR_ERR;
    }

    return SR_OK;
}

SR_PRIV int32_t nubridge_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di)
{
    libusb_device **devlist;
    struct sr_usb_dev_inst *usb;
    struct libusb_interface intf;
    struct libusb_device_descriptor des;
    struct libusb_config_descriptor *conf_desc;
    const struct libusb_interface_descriptor *intf_desc;
    struct dev_context *devc;
    struct drv_context *drvc;
    struct nubridge_version vi;
    int32_t ret = SR_ERR, i, j, k, z, device_count;
    char connection_id[64];
    bool intf_found;

    drvc = di->context;
    devc = sdi->priv;
    usb = sdi->conn;

    if ((device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist)) < 0)
    {
        sr_err("Failed to get device list.");
        return SR_ERR;
    }

    for (i = 0; i < device_count; i++)
    {
        if (libusb_get_device_descriptor(devlist[i], &des) < 0)
        {
            sr_err("Failed to get device descriptor.");
            ret = SR_ERR;
            break;
        }

        if (des.idVendor != devc->profile->vid || des.idProduct != devc->profile->pid)
            continue;

        if ((sdi->status == SR_ST_INITIALIZING) || (sdi->status == SR_ST_INACTIVE))
        {
            if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
                continue;

            if (strcmp(sdi->connection_id, connection_id) != 0)
                continue;
        }

        if (!(ret = libusb_open(devlist[i], &usb->devhdl)))
        {
            if (usb->address == 0xFF)
                usb->address = libusb_get_device_address(devlist[i]);
        }
        else
        {
            sr_err("Failed to open device.");
            ret = SR_ERR;
            break;
        }

        if (libusb_get_active_config_descriptor(devlist[i], &conf_desc) < 0)
        {
            sr_err("Failed to get config descriptor.");
            ret = SR_ERR;
            break;
        }
        else
        {
            intf_found = false;

            for (j = 0; j < conf_desc->bNumInterfaces; j++)
            {
                intf = conf_desc->interface[j];

                for (k = 0; k < intf.num_altsetting; k++)
                {
                    intf_desc = &intf.altsetting[k];

                    if (intf_desc->bInterfaceNumber != devc->profile->interface)
                        continue;

                    if (intf_desc->bInterfaceClass != 0xFF)
                        continue;

                    if (intf_desc->bNumEndpoints != 2)
                        continue;

                    for (z = 0; z < intf_desc->bNumEndpoints; z++)
                    {
                        if (intf_desc->endpoint[z].bEndpointAddress & LIBUSB_ENDPOINT_IN)
                            devc->endpoint_in = intf_desc->endpoint[z].bEndpointAddress;
                        else
                            devc->endpoint_out = intf_desc->endpoint[z].bEndpointAddress;
                    }

                    intf_found = true;
                }
            }

            if (!intf_found)
            {
                sr_err("Failed to get interface descriptor.");
                ret = SR_ERR;
                break;
            }
        }

        if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER))
        {
            if (libusb_kernel_driver_active(usb->devhdl, devc->profile->interface) == 1)
            {
                if ((ret = libusb_detach_kernel_driver(usb->devhdl, devc->profile->interface)) < 0)
                {
                    sr_err("Failed to detach kernel driver.");
                    ret = SR_ERR;
                    break;
                }
            }
        }

        if ((ret = libusb_claim_interface(usb->devhdl, devc->profile->interface)) < 0)
        {
            ret = SR_ERR;
            break;
        }

        if ((ret = command_get_version(devc, usb->devhdl, &vi)) != SR_OK)
        {
            ret = SR_ERR;
            break;
        }

        if (vi.minor != MINOR_VERSION)
        {
            sr_err("Device version mismatch: %d (!= %d)", vi.minor, MINOR_VERSION);
            ret = SR_ERR;
            break;
        }

        sr_info("Opened device on %d.%d (logical) / %s (physical), version %d.", usb->bus, usb->address, connection_id, vi.minor);

        ret = SR_OK;
        break;
    }

    libusb_free_device_list(devlist, 1);

    return ret;
}

SR_PRIV struct dev_context *nubridge_dev_new(void)
{
    struct dev_context *devc;

    devc = g_malloc0(sizeof(struct dev_context));
    devc->profile = NULL;
    devc->cur_samplerate = 0;
    devc->limit_samples = 0;
    devc->clock_edge = EDGE_RISING;

    return devc;
}

static void abort_acquisition(struct dev_context *devc)
{
    int32_t i;

    devc->acq_aborted = TRUE;

    for (i = devc->num_transfers - 1; i >= 0; i--)
    {
        if (devc->transfers[i])
            libusb_cancel_transfer(devc->transfers[i]);
    }
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
    struct dev_context *devc;

    devc = sdi->priv;

    std_session_send_df_end(sdi);
    usb_source_remove(sdi->session, devc->ctx);

    devc->num_transfers = 0;
    g_free(devc->transfers);
    g_free(devc->deinterleave_buffer);
}

static void free_transfer(struct libusb_transfer *transfer)
{
    struct sr_dev_inst *sdi;
    struct dev_context *devc;
    uint32_t i;

    sdi = transfer->user_data;
    devc = sdi->priv;

    g_free(transfer->buffer);
    transfer->buffer = NULL;
    libusb_free_transfer(transfer);

    for (i = 0; i < devc->num_transfers; i++)
    {
        if (devc->transfers[i] == transfer)
        {
            devc->transfers[i] = NULL;
            break;
        }
    }

    devc->submitted_transfers--;
    if (devc->submitted_transfers == 0)
        finish_acquisition(sdi);
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
    if (libusb_submit_transfer(transfer) != LIBUSB_SUCCESS)
        free_transfer(transfer);
}

static void deinterleave_buffer(const uint8_t *src, uint32_t length, uint8_t *dst_ptr, uint32_t channel_count, uint32_t channel_mask)
{
    uint8_t sample;

    for (const uint8_t *src_ptr = src; src_ptr < (src + length); src_ptr += channel_count)
    {
        for (int32_t bit = 0; bit != 8; bit++)
        {
            const uint8_t *channel_ptr = src_ptr;

            sample = 0;

            for (uint32_t channel = 0; channel != 8; channel++)
            {
                const uint32_t m = channel_mask >> channel;

                if (!m)
                    break;

                if ((m & 1) && ((*channel_ptr++ >> bit) & UINT8_C(1)))
                    sample |= 1 << channel;
            }

            *dst_ptr++ = sample;
        }
    }
}

static void send_data(struct sr_dev_inst *sdi, uint8_t *data, uint32_t sample_count)
{
    const struct sr_datafeed_logic logic =
    {
        .length = sample_count * sizeof(uint8_t),
        .unitsize = sizeof(uint8_t),
        .data = data
    };

    const struct sr_datafeed_packet packet =
    {
        .type = SR_DF_LOGIC,
        .payload = &logic
    };

    sr_session_send(sdi, &packet);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
    struct sr_dev_inst *const sdi = transfer->user_data;
    struct dev_context *const devc = sdi->priv;

    const uint32_t cur_sample_count = ATOMIC_SAMPLES * transfer->actual_length / (ATOMIC_BYTES * devc->channel_count);

    gboolean packet_has_error = FALSE;
    uint32_t num_samples;
    int32_t trigger_offset;

    if (devc->acq_aborted)
    {
        free_transfer(transfer);
        return;
    }

    switch (transfer->status)
    {
        case LIBUSB_TRANSFER_NO_DEVICE:
            abort_acquisition(devc);
            free_transfer(transfer);
            return;
        case LIBUSB_TRANSFER_COMPLETED:
        case LIBUSB_TRANSFER_TIMED_OUT:
            break;
        default:
            packet_has_error = TRUE;
            break;
    }

    if (transfer->actual_length == 0 || packet_has_error)
    {
        resubmit_transfer(transfer);
        return;
    }

    if (!devc->limit_samples || devc->sent_samples < devc->limit_samples)
    {
        if (devc->limit_samples && devc->sent_samples + cur_sample_count > devc->limit_samples)
            num_samples = devc->limit_samples - devc->sent_samples;
        else
            num_samples = cur_sample_count;

        //if (transfer->actual_length % (ATOMIC_BYTES * devc->channel_count) != 0)
        //    sr_err("Invalid transfer length!");

        deinterleave_buffer(transfer->buffer, transfer->actual_length, devc->deinterleave_buffer, devc->channel_count, devc->channel_mask);

        if (devc->trigger_pos > devc->sent_samples && devc->trigger_pos <= devc->sent_samples + num_samples)
        {
            trigger_offset = devc->trigger_pos - devc->sent_samples;

            send_data(sdi, devc->deinterleave_buffer, trigger_offset);
            devc->sent_samples += trigger_offset;

            devc->trigger_pos = 0;
            std_session_send_df_trigger(sdi);

            num_samples -= trigger_offset;
            send_data(sdi, devc->deinterleave_buffer + trigger_offset, num_samples);

            devc->sent_samples += num_samples;
        }
        else
        {
            send_data(sdi, devc->deinterleave_buffer, num_samples);
            devc->sent_samples += num_samples;
        }
    }

    if (devc->limit_samples && devc->sent_samples >= devc->limit_samples)
    {
        abort_acquisition(devc);
        free_transfer(transfer);
    }
    else
    {
        resubmit_transfer(transfer);
    }
}

static int32_t receive_data(int32_t fd, int32_t revents, void *cb_data)
{
    struct timeval tv;
    struct drv_context *drvc;

    (void)fd;
    (void)revents;

    drvc = (struct drv_context *)cb_data;

    tv.tv_sec = tv.tv_usec = 0;
    libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

    return TRUE;
}

static int32_t start_transfers(const struct sr_dev_inst *sdi, uint32_t channel_mask, uint32_t transfer_size)
{
    struct dev_context *devc;
    struct sr_usb_dev_inst *usb;
    struct libusb_transfer *transfer;
    int32_t ret;
    unsigned char *buf;
    uint32_t i, channel_count, num_transfers;

    for (i = 0, channel_count = 0; i < 8; i++)
    {
        if (channel_mask & (1 << i))
            channel_count++;
    }

    devc = sdi->priv;
    usb = sdi->conn;

    transfer_size = MIN(transfer_size, devc->profile->mem_depth);
    num_transfers = transfer_size / USB_PACKET_SIZE;

    devc->channel_mask = channel_mask;
    devc->channel_count = channel_count;
    devc->limit_samples = (uint64_t) (transfer_size * 8 / channel_count);

    devc->sent_samples = 0;
    devc->acq_aborted = FALSE;
    devc->submitted_transfers = 0;

    g_free(devc->transfers);

    devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * num_transfers);
    if (!devc->transfers)
        return SR_ERR_MALLOC;

    devc->deinterleave_buffer = g_try_malloc(ATOMIC_SAMPLES * (USB_PACKET_SIZE / (channel_count * ATOMIC_BYTES)) * sizeof(uint8_t));
    if (!devc->deinterleave_buffer)
    {
        g_free(devc->deinterleave_buffer);
        return SR_ERR_MALLOC;
    }

    devc->num_transfers = num_transfers;
    for (i = 0; i < num_transfers; i++)
    {
        if (!(buf = g_try_malloc(USB_PACKET_SIZE)))
            return SR_ERR_MALLOC;

        transfer = libusb_alloc_transfer(0);
        libusb_fill_bulk_transfer(transfer, usb->devhdl, devc->endpoint_in, buf, USB_PACKET_SIZE, receive_transfer, (void *)sdi, USB_TIMEOUT);

        if ((ret = libusb_submit_transfer(transfer)) != LIBUSB_SUCCESS)
        {
            libusb_free_transfer(transfer);
            g_free(buf);
            abort_acquisition(devc);
            return SR_ERR;
        }
        devc->transfers[i] = transfer;
        devc->submitted_transfers++;
    }

    std_session_send_df_header(sdi);

    return SR_OK;
}

static void LIBUSB_CALL trigger_receive(struct libusb_transfer *transfer)
{
    const struct sr_dev_inst *sdi;
    struct nubridge_trigger_pos *tpos = NULL;
    struct dev_context *devc;
    int32_t ret;

    sdi = transfer->user_data;
    devc = sdi->priv;

    if (transfer->status == LIBUSB_TRANSFER_CANCELLED)
    {
        std_session_send_df_end(sdi);
        usb_source_remove(sdi->session, devc->ctx);

        devc->num_transfers = 0;
        g_free(devc->transfers);
    }
    else if (transfer->status == LIBUSB_TRANSFER_COMPLETED && transfer->actual_length == sizeof(struct nubridge_trigger_pos))
    {
        tpos = (struct nubridge_trigger_pos *)transfer->buffer;
        devc->trigger_pos = tpos->real_pos;

        if ((tpos->check_id != TRIGGER_HEADER_ID))
        {
            sr_err("Failed to trigger session.");

            std_session_send_df_end(sdi);
            usb_source_remove(sdi->session, devc->ctx);
        }
        else if ((ret = start_transfers(sdi, tpos->channel_en, tpos->buffer_size)) != SR_OK)
        {
            sr_err("Failed to start transfers");
        }

        g_free(tpos);
    }

    libusb_free_transfer(transfer);
}

SR_PRIV int32_t nubridge_acquisition_start(const struct sr_dev_inst *sdi)
{
    struct sr_dev_driver *di;
    struct drv_context *drvc;
    struct dev_context *devc;
    struct sr_usb_dev_inst *usb;
    struct nubridge_trigger_pos *tpos;
    struct libusb_transfer *transfer;
    int32_t ret;

    di = sdi->driver;
    drvc = di->context;
    devc = sdi->priv;
    usb = sdi->conn;

    devc->ctx = drvc->sr_ctx;
    devc->sent_samples = 0;
    devc->acq_aborted = FALSE;

    if ((ret = command_stop_acquisition(sdi)) != SR_OK)
        return ret;

    if ((ret = nubridge_configure(sdi)) != SR_OK)
        return ret;

    if ((ret = command_start_acquisition(sdi)) != SR_OK)
        return ret;

    usb_source_add(sdi->session, devc->ctx, 100, receive_data, drvc);

    tpos = g_malloc(sizeof(struct nubridge_trigger_pos));

    transfer = libusb_alloc_transfer(0);
    transfer->actual_length = 0;

    libusb_fill_bulk_transfer(transfer, usb->devhdl, devc->endpoint_in, (unsigned char *)tpos, sizeof(struct nubridge_trigger_pos), trigger_receive, (void *)sdi, 0);

    if ((ret = libusb_submit_transfer(transfer)) != 0)
    {
        sr_err("Failed to request trigger.");
        libusb_free_transfer(transfer);
        g_free(tpos);
        return SR_ERR;
    }

    devc->transfers = g_try_malloc0(sizeof(*devc->transfers));
    if (!devc->transfers)
        return SR_ERR_MALLOC;

    devc->num_transfers = 1;
    devc->submitted_transfers = 1;
    devc->transfers[0] = transfer;

    return ret;
}

SR_PRIV int32_t nubridge_acquisition_stop(struct sr_dev_inst *sdi)
{
    command_stop_acquisition(sdi);
    abort_acquisition(sdi->priv);

    return SR_OK;
}

