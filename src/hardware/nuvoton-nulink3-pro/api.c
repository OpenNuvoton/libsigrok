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
#include "protocol.h"

#define DEFAULT_SAMPLE_RATE    SR_MHZ(4)

static const struct nubridge_profile supported_device[] =
{
    { 0x0416, 0x200A, 7, "Nuvoton", "Nu-Link3-Pro", NULL, 0, 1024 * 1024, 6},
    ALL_ZERO
};

static const uint32_t scanopts[] =
{
    SR_CONF_CONN,
};

static const uint32_t drvopts[] =
{
    SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] =
{
    SR_CONF_CONN          | SR_CONF_GET,
    SR_CONF_SAMPLERATE    | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
    SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
};

static const int32_t trigger_matches[] =
{
    SR_TRIGGER_RISING,
    SR_TRIGGER_FALLING,
    SR_TRIGGER_EDGE,
};

static const uint64_t samplerates[] =
{
    SR_MHZ(1),
    SR_MHZ(2),
    SR_MHZ(4),
    SR_MHZ(8),
    SR_MHZ(10),
    SR_MHZ(22),
};

static gboolean is_plausible(const struct libusb_device_descriptor *des)
{
    int32_t i;

    for (i = 0; supported_device[i].vid; i++)
    {
        if (des->idVendor != supported_device[i].vid)
            continue;
        if (des->idProduct == supported_device[i].pid)
            return TRUE;
    }

    return FALSE;
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
    struct drv_context *drvc;
    struct dev_context *devc;
    struct sr_dev_inst *sdi;
    struct sr_usb_dev_inst *usb;
    struct sr_channel *ch;
    struct sr_channel_group *cg;
    struct sr_config *src;
    const struct nubridge_profile *prof;
    GSList *l, *devices, *conn_devices;
    struct libusb_device_descriptor des;
    libusb_device **devlist;
    struct libusb_device_handle *hdl;
    int32_t ret, i;
    uint32_t j;
    const char *conn;
    char product[64], serial_num[64], connection_id[64];
    char channel_name[16];

    drvc = di->context;

    conn = NULL;
    for (l = options; l; l = l->next)
    {
        src = l->data;
        switch (src->key)
        {
            case SR_CONF_CONN:
                conn = g_variant_get_string(src->data, NULL);
                break;
        }
    }

    if (conn)
        conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
    else
        conn_devices = NULL;

    devices = NULL;
    libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
    for (i = 0; devlist[i]; i++)
    {
        if (conn)
        {
            usb = NULL;
            for (l = conn_devices; l; l = l->next)
            {
                usb = l->data;
                if (usb->bus == libusb_get_bus_number(devlist[i]) && usb->address == libusb_get_device_address(devlist[i]))
                    break;
            }
            if (!l)
                continue;
        }

        libusb_get_device_descriptor(devlist[i], &des);

        if (!is_plausible(&des))
            continue;

        if ((ret = libusb_open(devlist[i], &hdl)) < 0)
        {
            sr_warn("Failed to open device with VID:PID %04X:%04X. (It may be already open or in use by another program.)", des.idVendor, des.idProduct);
            continue;
        }

        if (des.iProduct == 0)
        {
            product[0] = '\0';
        }
        else if ((ret = libusb_get_string_descriptor_ascii(hdl, des.iProduct, (unsigned char *) product, sizeof(product))) < 0)
        {
            sr_warn("Failed to get product string descriptor.");
            continue;
        }

        if (des.iSerialNumber == 0)
        {
            serial_num[0] = '\0';
        }
        else if ((ret = libusb_get_string_descriptor_ascii(hdl, des.iSerialNumber, (unsigned char *) serial_num, sizeof(serial_num))) < 0)
        {
            sr_warn("Failed to get serial number string descriptor.");
            continue;
        }

        libusb_close(hdl);

        if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
            continue;

        prof = NULL;
        for (j = 0; supported_device[j].vid; j++)
        {
            if (des.idVendor == supported_device[j].vid && des.idProduct == supported_device[j].pid)
            {
                prof = &supported_device[j];
                break;
            }
        }

        if (!prof)
            continue;

        sdi = g_malloc0(sizeof(struct sr_dev_inst));
        sdi->status = SR_ST_INITIALIZING;
        sdi->vendor = g_strdup(prof->vendor);
        sdi->model = g_strdup(prof->model);
        sdi->version = g_strdup(prof->model_version);
        sdi->serial_num = g_strdup(serial_num);
        sdi->connection_id = g_strdup(connection_id);

        cg = sr_channel_group_new(sdi, "Logic", NULL);
        for (j = 0; j < prof->channel_count; j++)
        {
            sprintf(channel_name, "%d", j);
            ch = sr_channel_new(sdi, j, SR_CHANNEL_LOGIC, TRUE, channel_name);
            cg->channels = g_slist_append(cg->channels, ch);
        }

        devc = nubridge_dev_new();
        devc->profile = prof;
        sdi->priv = devc;
        devices = g_slist_append(devices, sdi);

        devc->samplerates = samplerates;
        devc->num_samplerates = ARRAY_SIZE(samplerates);

        sdi->inst_type = SR_INST_USB;
        sdi->conn = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]), 0xff, NULL);
    }

    libusb_free_device_list(devlist, 1);
    g_slist_free_full(conn_devices, (GDestroyNotify)sr_usb_dev_inst_free);

    return std_scan_complete(di, devices);
}

static int32_t dev_open(struct sr_dev_inst *sdi)
{
    struct sr_dev_driver *di = sdi->driver;
    struct dev_context *devc;
    int32_t ret;

    devc = sdi->priv;

    if ((ret = nubridge_dev_open(sdi, di)) != SR_OK)
    {
        sr_err("Failed to open device.");
        return SR_ERR;
    }

#if 0
    struct sr_usb_dev_inst *usb = sdi->conn;

    if ((ret = libusb_claim_interface(usb->devhdl, devc->profile->interface)) != 0)
    {
        switch (ret)
        {
            case LIBUSB_ERROR_BUSY:
                sr_err("Failed to claim USB interface. Another program or driver has already claimed it.");
                break;
            case LIBUSB_ERROR_NO_DEVICE:
                sr_err("Device has been disconnected.");
                break;
            default:
                sr_err("Failed to claim interface.");
                break;
        }

        return SR_ERR;
    }
#endif

    if (devc->cur_samplerate == 0)
        devc->cur_samplerate = DEFAULT_SAMPLE_RATE;

    devc->limit_samples = 0 ; //devc->profile->mem_depth * 8 / NUM_CHANNELS;

    return SR_OK;
}

static int32_t dev_close(struct sr_dev_inst *sdi)
{
    struct sr_usb_dev_inst *usb;
    struct dev_context *devc;

    devc = sdi->priv;
    usb = sdi->conn;

    if (!usb->devhdl)
        return SR_ERR_BUG;

    sr_info("Closing device on %d.%d (logical) / %s (physical).", usb->bus, usb->address, sdi->connection_id);

    libusb_release_interface(usb->devhdl, devc->profile->interface);
    libusb_close(usb->devhdl);
    usb->devhdl = NULL;

    return SR_OK;
}

static int32_t config_get(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
    struct dev_context *devc;
    struct sr_usb_dev_inst *usb;

    (void)cg;

    if (!sdi)
        return SR_ERR_ARG;

    devc = sdi->priv;

    switch (key)
    {
        case SR_CONF_CONN:
            if (!sdi->conn)
                return SR_ERR_ARG;
            usb = sdi->conn;
            if (usb->address == 0xFF)
                return SR_ERR;
            *data = g_variant_new_printf("%d.%d", usb->bus, usb->address);
            break;
        case SR_CONF_SAMPLERATE:
            *data = g_variant_new_uint64(devc->cur_samplerate);
            break;
        default:
            return SR_ERR_NA;
    }

    return SR_OK;
}

static int32_t config_set(uint32_t key, GVariant *data, const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
    struct dev_context *devc;
    int32_t idx;
    (void)cg;

    if (!sdi)
        return SR_ERR_ARG;

    devc = sdi->priv;

    switch (key)
    {
        case SR_CONF_SAMPLERATE:
            if ((idx = std_u64_idx(data, devc->samplerates, devc->num_samplerates)) < 0)
                return SR_ERR_ARG;
            devc->cur_samplerate = devc->samplerates[idx];
            break;
        default:
            return SR_ERR_NA;
    }

    return SR_OK;
}

static int32_t config_list(uint32_t key, GVariant **data, const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
    struct dev_context *devc;

    devc = (sdi) ? sdi->priv : NULL;

    if (cg)
        return SR_ERR_NA;

    switch (key)
    {
        case SR_CONF_SCAN_OPTIONS:
        case SR_CONF_DEVICE_OPTIONS:
            return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
        case SR_CONF_SAMPLERATE:
            if (!devc)
                return SR_ERR_ARG;
            *data = std_gvar_samplerates(devc->samplerates, devc->num_samplerates);
            break;
        case SR_CONF_TRIGGER_MATCH:
            *data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
            break;
        default:
            return SR_ERR_NA;
    }

    return SR_OK;
}

static struct sr_dev_driver nulink3_pro_driver_info =
{
    .name = "nuvoton-nulink3-pro",
    .longname = "Nuvoton Nu-Link3-Pro",
    .api_version = 1,
    .init = std_init,
    .cleanup = std_cleanup,
    .scan = scan,
    .dev_list = std_dev_list,
    .dev_clear = std_dev_clear,
    .config_get = config_get,
    .config_set = config_set,
    .config_list = config_list,
    .dev_open = dev_open,
    .dev_close = dev_close,
    .dev_acquisition_start = nubridge_acquisition_start,
    .dev_acquisition_stop = nubridge_acquisition_stop,
    .context = NULL,
};
SR_REGISTER_DEV_DRIVER(nulink3_pro_driver_info);

