/*
 * Copyright (C) 2023 David Guillen Fandos <david@davidgf.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/stm32/flash.h>

#include "devconf.h"
#include "stm32-bootloader/reboot.h"

#include "board_stm32prog.h"

#define LED_BLINK_MS    1500    // Blink for 1.5 seconds
#define LED_BLINK_SPEED  256    // Blink ~4 times per second

#define FLASH_CONFIG_ADDR      0x0800C000   // Base block number 48
#define FLASH_CONFIG_NBLK              16   // 16 blocks for now
#define FLASH_CONFIG_SIZE             216

_Static_assert(FLASH_CONFIG_SIZE == sizeof(t_devconf), "sanity check");

#define MAGIC_KEY         0xd1bd     // To avoid random driver probes causing mayhem

#define START_OFFSET_MSECS  1000     // Wait 1 second before doing its thing

#define MOUSEMV_LEFTRIGHT      0
#define MOUSEMV_UPDOWN         1
#define MOUSEMV_CIRCLE         2
#define MOUSEMV_RANDOM         3
#define MOUSEMV_WUPDOWN        4

#define KEYBD_KEYTYPE          0

#define REQ_DFU_REBOOT      0xff   // Reboots to bootloader
#define REQ_CONFIG          0xfe   // Reads/Writes config
#define REQ_INFO            0xfd   // Reads FW information
#define REQ_RESET           0xfc   // Wipes and factory-resets the device

#define TOGGLE_MODE_4k_1s      0
#define TOGGLE_MODE_6k_2s      1
#define TOGGLE_MODE_8k_3s      2

#define HDR_VERSION   0x00010000

static uint32_t last_activity = 0;
static uint64_t mscounter64 = 0;
static uint32_t mscounter32 = 0;
static uint8_t  faker_enabled;

const uint8_t toggle_limits[3][2] = {
	{ 4, 1 << 4 },
	{ 6, 2 << 4 },
	{ 8, 3 << 4 },
};

const struct {
	uint32_t magic;
	uint32_t version;
	char fw_version[56];
} device_fw_info = {
	.magic = HDR_MAGIC,
	.version = HDR_VERSION,
	.fw_version = VERSION
};

const t_devconf default_cfg = {
	.magic = HDR_MAGIC,
	.version = HDR_VERSION,
	.cfg_rev = 0,
	.checksum = 0,
	.usb_vidpid = 0x045e07fd,
	.manufacturer = "Microsoft",
	.product = "Microsoft Nano Transceiver 1.1",
	.toggle_key = 1,
	.toggle_mode = TOGGLE_MODE_6k_2s,
	.def_enabled = 1,
	.rule_count = 1,
	.rules = {
		{.period = 30*1000, .rtype = MOUSEMV_LEFTRIGHT, .rdata = 1, .rmode = 0, .rflags = 0}
	}
};

// Points to the current config (can be the default or a previously flashed config)
const t_devconf *current_config;
// Working structures, inferred from the config.
struct {
	uint64_t trigger_ts[MAX_RULES];     // Next rule trigger
	uint32_t state[MAX_RULES];          // State for movement and key up/down
} rcfg;

static usbd_device *usbd_dev;

static struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0,
	.idProduct = 0,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};
const static char *usb_strings[2];

static const uint8_t mouse_report_descriptor[] = {
	0x05, 0x01, /* USAGE_PAGE (Generic Desktop)         */
	0x09, 0x02, /* USAGE (Mouse)                        */
	0xa1, 0x01, /* COLLECTION (Application)             */
	0x09, 0x01, /*   USAGE (Pointer)                    */
	0xa1, 0x00, /*   COLLECTION (Physical)              */
	0x05, 0x09, /*     USAGE_PAGE (Button)              */
	0x19, 0x01, /*     USAGE_MINIMUM (Button 1)         */
	0x29, 0x03, /*     USAGE_MAXIMUM (Button 3)         */
	0x15, 0x00, /*     LOGICAL_MINIMUM (0)              */
	0x25, 0x01, /*     LOGICAL_MAXIMUM (1)              */
	0x95, 0x03, /*     REPORT_COUNT (3)                 */
	0x75, 0x01, /*     REPORT_SIZE (1)                  */
	0x81, 0x02, /*     INPUT (Data,Var,Abs)             */
	0x95, 0x01, /*     REPORT_COUNT (1)                 */
	0x75, 0x05, /*     REPORT_SIZE (5)                  */
	0x81, 0x01, /*     INPUT (Cnst,Ary,Abs)             */
	0x05, 0x01, /*     USAGE_PAGE (Generic Desktop)     */
	0x09, 0x30, /*     USAGE (X)                        */
	0x09, 0x31, /*     USAGE (Y)                        */
	0x09, 0x38, /*     USAGE (Wheel)                    */
	0x15, 0x81, /*     LOGICAL_MINIMUM (-127)           */
	0x25, 0x7f, /*     LOGICAL_MAXIMUM (127)            */
	0x75, 0x08, /*     REPORT_SIZE (8)                  */
	0x95, 0x03, /*     REPORT_COUNT (3)                 */
	0x81, 0x06, /*     INPUT (Data,Var,Rel)             */
	0xc0,       /*   END_COLLECTION                     */
	0x09, 0x3c, /*   USAGE (Motion Wakeup)              */
	0x05, 0xff, /*   USAGE_PAGE (Vendor Defined Page 1) */
	0x09, 0x01, /*   USAGE (Vendor Usage 1)             */
	0x15, 0x00, /*   LOGICAL_MINIMUM (0)                */
	0x25, 0x01, /*   LOGICAL_MAXIMUM (1)                */
	0x75, 0x01, /*   REPORT_SIZE (1)                    */
	0x95, 0x02, /*   REPORT_COUNT (2)                   */
	0xb1, 0x22, /*   FEATURE (Data,Var,Abs,NPrf)        */
	0x75, 0x06, /*   REPORT_SIZE (6)                    */
	0x95, 0x01, /*   REPORT_COUNT (1)                   */
	0xb1, 0x01, /*   FEATURE (Cnst,Ary,Abs)             */
	0xc0        /* END_COLLECTION                       */
};

static const uint8_t kbd_report_descriptor[] = {
	0x05, 0x01, /* USAGE_PAGE (Generic Desktop)         */
	0x09, 0x06, /* USAGE (Keyboard                      */
	0xa1, 0x01, /* COLLECTION (Application)             */
	0x05, 0x07, /*   USAGE_PAGE (Key Codes)             */
	0x19, 0xe0, /*   USAGE_MINIMUM (224)                */
	0x29, 0xe7, /*   USAGE_MAXIMUM (231)                */
	0x15, 0x00, /*   LOGICAL_MINIMUM (0)                */
	0x25, 0x01, /*   LOGICAL_MAXIMUM (1)                */
	0x75, 0x01, /*   REPORT_SIZE (1)                    */
	0x95, 0x08, /*   REPORT_COUNT (8)                   */
	0x81, 0x02, /*   INPUT (Data,Var,Abs)   [Mod keys]  */
	0x95, 0x01, /*   REPORT_COUNT (1)                   */
	0x75, 0x08, /*   REPORT_SIZE (8)                    */
	0x81, 0x01, /*   INPUT (Constant)       [Reserved]  */
	0x95, 0x03, /*   REPORT_COUNT (3)                   */
	0x75, 0x01, /*   REPORT_SIZE (1)                    */
	0x05, 0x08, /*   USAGE_PAGE (LEDs)                  */
	0x19, 0x01, /*   USAGE_MINIMUM (1)                  */
	0x29, 0x03, /*   USAGE_MAXIMUM (3)                  */
	0x91, 0x02, /*   OUTPUT (Data,Var,Abs)  [LED bits]  */
	0x95, 0x05, /*   REPORT_COUNT (5)                   */
	0x75, 0x01, /*   REPORT_SIZE (1)                    */
	0x91, 0x01, /*   OUTPUT (Constant)      [Padding]   */
	0x95, 0x06, /*   REPORT_COUNT (6)                   */
	0x75, 0x08, /*   REPORT_SIZE (8)                    */
	0x15, 0x00, /*   LOGICAL_MINIMUM (0)                */
	0x26, 0x90, 0x00, /*   LOGICAL_MAXIMUM (144)        */
	0x05, 0x07, /*   USAGE_PAGE (Key Codes)             */
	0x19, 0x00, /*   USAGE_MINIMUM (0)                  */
	0x29, 0x90, /*   USAGE_MAXIMUM (144)                */
	0x81, 0x00, /*   INPUT (Array)          [Keys(6)]   */
	0xc0        /* END_COLLECTION                       */
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) mouse_fn = {
	.hid_descriptor = {
		.bLength = sizeof(mouse_fn),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(mouse_report_descriptor),
	}
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) kbd_fn = {
	.hid_descriptor = {
		.bLength = sizeof(kbd_fn),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(kbd_report_descriptor),
	}
};

const struct usb_endpoint_descriptor mouse_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x20,
};

const struct usb_endpoint_descriptor kbd_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 8,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor mouse_hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 2, /* mouse */
	.iInterface = 0,

	.endpoint = &mouse_endpoint,

	.extra = &mouse_fn,
	.extralen = sizeof(mouse_fn),
};

const struct usb_interface_descriptor kbd_hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 1, /* keyboard */
	.iInterface = 0,

	.endpoint = &kbd_endpoint,

	.extra = &kbd_fn,
	.extralen = sizeof(kbd_fn),
};

const struct usb_interface_descriptor config_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 2,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 255,
	.bInterfaceSubClass = 255,
	.bInterfaceProtocol = 255,
	.iInterface = 0,
};

const struct usb_interface ifaces[] = {
	{.num_altsetting = 1, .altsetting = &mouse_hid_iface},
	{.num_altsetting = 1, .altsetting = &kbd_hid_iface},
	{.num_altsetting = 1, .altsetting = &config_iface},
};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 3,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[1024];

// Some custom sutff int the EP0 for rebooting and poking
void reboot_info_dfu() {
	// Reboot into DFU!
	reboot_into_bootloader();
	scb_reset_system();
}

void reboot_handler() {
	// Regular reboot
	scb_reset_system();
}

void wipe_and_reboot() {
	// Wipe the storage and reboot
	flash_unlock();
	for (unsigned i = 0; i < FLASH_CONFIG_NBLK; i++)
		flash_erase_page(FLASH_CONFIG_ADDR + i*1024);
	flash_lock();

	scb_reset_system();
}

int wiped_entry(const t_devconf *cfg) {
	for (unsigned i = 0; i < sizeof(t_devconf)/sizeof(uint32_t); i++)
		if (((uint32_t*)cfg)[i] != 0xFFFFFFFFU)
			return 0;
	return 1;
}

uint32_t calc_checksum(const t_devconf *cfg) {
	uint32_t ret = 0xc1f94e12;
	for (unsigned i = offsetof(t_devconf, checksum)/sizeof(uint32_t) + 1; i < sizeof(t_devconf)/sizeof(uint32_t); i++)
		ret ^= ((uint32_t*)cfg)[i];
	return ret;
}

int config_valid(const t_devconf* cfg) {
	if (cfg->magic != HDR_MAGIC || cfg->version != HDR_VERSION)
		return 0;

	if (cfg->checksum != calc_checksum(cfg))
		return 0;

	if (cfg->manufacturer[sizeof(cfg->manufacturer)-1] ||
	    cfg->product[sizeof(cfg->product)-1])
	    return 0;

	if (cfg->rule_count > MAX_RULES)
		return 0;

	// Only accepts three LEDs for now
	if (cfg->toggle_key != 0xFF && cfg->toggle_key > 2)
		return 0;

	return 1;
}

static enum usbd_request_return_codes ctrl_req_in(
	usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)complete;
	(void)dev;

	// Special handler to reboot straight into DFU
	switch (req->bRequest) {
	case REQ_CONFIG:      // Read config
		*buf = (uint8_t*)current_config;
		*len = sizeof(*current_config);
		return USBD_REQ_HANDLED;
	case REQ_INFO:
		*buf = (uint8_t*)&device_fw_info;
		*len = sizeof(device_fw_info);
		return USBD_REQ_HANDLED;
	case USB_REQ_GET_DESCRIPTOR:
		// Return the HID decriptor for the requested iface
		if ((req->bmRequestType != 0x81) || (req->wValue != 0x2200))
			return USBD_REQ_NOTSUPP;

		if (req->wIndex) {
			*buf = (uint8_t *)kbd_report_descriptor;
			*len = sizeof(kbd_report_descriptor);
		} else {
			*buf = (uint8_t *)mouse_report_descriptor;
			*len = sizeof(mouse_report_descriptor);
		}

		return USBD_REQ_HANDLED;
	};

	return USBD_REQ_NOTSUPP;
}

static enum usbd_request_return_codes ctrl_req_out(
	usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)dev;

	if (req->wValue != MAGIC_KEY)
		return USBD_REQ_NOTSUPP;

	switch (req->bRequest) {
	case REQ_DFU_REBOOT:   	// Special handler to reboot straight into DFU
		*len = 0;
		*complete = reboot_info_dfu;
		return USBD_REQ_HANDLED;
	case REQ_RESET:
		*len = 0;
		*complete = wipe_and_reboot;
		return USBD_REQ_HANDLED;
	case REQ_CONFIG:        // Write new config
		// Validate the config
		if (!config_valid((t_devconf*)*buf))
			return USBD_REQ_NOTSUPP;

		// It needs to have a higher revision than the current config (last)
		if (current_config->cfg_rev >= ((t_devconf*)*buf)->cfg_rev)
			return USBD_REQ_NOTSUPP;

		// Go flash it. Find an empty or old entry.
		const t_devconf *eempty = NULL, *ebad = NULL, *eold = NULL;
		for (unsigned i = 0; i < FLASH_CONFIG_NBLK; i++) {
			const t_devconf *entry = (t_devconf*)(FLASH_CONFIG_ADDR + i*1024);
			if (wiped_entry(entry))
				eempty = entry;
			else if (!config_valid(entry))
				ebad = entry;
			else if (!eold || entry->cfg_rev < eold->cfg_rev)
				eold = entry;
		}

		// Pick empty entries first, then corrupted ones, finally old & valid ones
		const t_devconf *picked = eempty ? eempty :
		                          ebad   ? ebad   :
		                          eold;

		uint32_t wpage = (uintptr_t)picked;

		flash_unlock();
		// Erase page if necessary only
		if (!wiped_entry(picked))
			flash_erase_page(wpage);

		// Flash config now
		for (unsigned i = 0; i < FLASH_CONFIG_SIZE; i += 2)
			flash_program_half_word(wpage + i, *(uint16_t*)(&(*buf)[i]));
		flash_lock();

		*len = 0;
		*complete = reboot_handler;

		return USBD_REQ_HANDLED;
	};

	return USBD_REQ_NOTSUPP;
}

static uint32_t toggle_hist[16] = {0};
static uint8_t toggle_ptr = 0;
static uint8_t pmask = 0;

static enum usbd_request_return_codes ctrl_req_classout(
	usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)complete;
	(void)dev;

	// Handle keyboard LEDs
	if (req->bRequest == USB_HID_REQ_TYPE_SET_REPORT && *len == 1) {
		uint8_t mask = (*buf)[0];
		// Mask represents numlock, capslock, scroll lock
		if (current_config->toggle_key <= 2 &&
		    (mask ^ pmask) & (1 << current_config->toggle_key)) {
			// Record toggle history
			toggle_hist[toggle_ptr] = mscounter32;

			// Check whether we went over the trigger condition
			const uint8_t *lim = toggle_limits[current_config->toggle_mode];
			uint8_t keycount = lim[0];
			uint32_t msdiff  = (lim[1] * 1000) >> 4;

			uint32_t startp = toggle_hist[(toggle_ptr - keycount + 1) & 15];
			uint32_t timediff = (mscounter32 - startp);
			if (timediff < msdiff) {
				// The last N toggles happened in less than K msecs, trigger!
				memset(toggle_hist, 0, sizeof(toggle_hist));
				// Enable/disable if necessary
				faker_enabled ^= 1;
				update_led_enable(faker_enabled);
			}

			// Advance window
			toggle_ptr = (toggle_ptr + 1) & 15;
		}

		pmask = mask;
		return USBD_REQ_HANDLED;
	}

	return USBD_REQ_NOTSUPP;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue) {
	(void)wValue;
	(void)dev;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);
	usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, 8, NULL);

	usbd_register_control_callback(dev,
		USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE | USB_REQ_TYPE_OUT,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT | USB_REQ_TYPE_DIRECTION,
		ctrl_req_out);

	usbd_register_control_callback(dev,
		USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE | USB_REQ_TYPE_IN,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT | USB_REQ_TYPE_DIRECTION,
		ctrl_req_in);

	usbd_register_control_callback(dev,
		USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE | USB_REQ_TYPE_OUT,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT | USB_REQ_TYPE_DIRECTION,
		ctrl_req_classout);
}

#define MAX_TX_QUEUE    16
static struct {
	uint8_t epn, size;
	uint8_t data[8];
}
usb_tx_buffer[16] = {0};
volatile unsigned buf_rdptr = 0, buf_wrptr = 0;

int push_usb(uint8_t ep, const uint8_t *data, unsigned len) {
	unsigned curr_ptr = buf_wrptr;
	unsigned next_ptr = (curr_ptr + 1) % MAX_TX_QUEUE;
	if (next_ptr == buf_rdptr)
		return 0;   // Overflow (keep 1 guard element)
	memcpy(usb_tx_buffer[curr_ptr].data, data, len);
	usb_tx_buffer[curr_ptr].epn = ep;
	usb_tx_buffer[curr_ptr].size = len;
	buf_wrptr = next_ptr;
	return 1;
}

int main(void) {
	// Setup fast USB clock
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	// Parse the config, bail on corrupted data and use the default config
	const t_devconf *rom_cfg = NULL;
	for (unsigned i = 0; i < FLASH_CONFIG_NBLK; i++) {
		const t_devconf *entry = (t_devconf*)(FLASH_CONFIG_ADDR + i *1024);
		if (config_valid(entry))
		    // Pick the most recent one!
		    if (!rom_cfg || rom_cfg->cfg_rev < entry->cfg_rev)
				rom_cfg = entry;
	}

    current_config = rom_cfg ? rom_cfg : &default_cfg;

	// Init state machine
	for (unsigned i = 0; i < current_config->rule_count; i++) {
		rcfg.trigger_ts[i] = START_OFFSET_MSECS + current_config->rules[i].period;
		rcfg.state[i] = 0;
	}

	// Prepare USB info from the config
	dev_descr.idVendor  = current_config->usb_vidpid >> 16;
	dev_descr.idProduct = current_config->usb_vidpid & 0xffff;
	usb_strings[0] = &current_config->manufacturer[0];
	usb_strings[1] = &current_config->product[0];

	faker_enabled = current_config->def_enabled;

	// Force re-enumeration
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (unsigned i = 0; i < 800000; i++)
		__asm__ ("nop");

	iwdg_reset();  // Watchdog happy plz
	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 2,
		usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);

	// Configure millisecond counter
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	systick_set_reload(8999);
	systick_interrupt_enable();
	systick_counter_enable();

	update_led_enable(faker_enabled);

	while (1) {
		usbd_poll(usbd_dev);
		iwdg_reset();

		// Send interrupt packets if queued
		if (buf_rdptr != buf_wrptr) {
			if (usbd_ep_write_packet(usbd_dev,
				usb_tx_buffer[buf_rdptr].epn,
				usb_tx_buffer[buf_rdptr].data,
				usb_tx_buffer[buf_rdptr].size)) {

				last_activity = mscounter32;
				buf_rdptr = (buf_rdptr + 1) % MAX_TX_QUEUE;
			}
		}
	}
}

void sys_tick_handler(void) {
	mscounter64++;
	mscounter32++;

	// LED blinky blink!
	if (mscounter32 - last_activity < LED_BLINK_MS)
		update_led_activity((mscounter32 % LED_BLINK_SPEED) < LED_BLINK_SPEED/2);
	else
		update_led_activity(0);

	// Ingore if disabled!
	if (!faker_enabled)
		return;

	for (unsigned i = 0; i < current_config->rule_count; i++) {
		if (rcfg.trigger_ts[i] <= mscounter64) {
			// Reschedule the rule again
			rcfg.trigger_ts[i] = mscounter64 + current_config->rules[i].period;
			// Trigger event
			if (current_config->rules[i].rtype) {
				// Simulate a keyboard
				uint8_t upd[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
				uint8_t updr[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

				switch (current_config->rules[i].rmode) {
				case KEYBD_KEYTYPE:
					upd[2] = current_config->rules[i].rdata;
					push_usb(0x82, upd, sizeof(upd));
					push_usb(0x82, updr, sizeof(updr));
					break;
				};
			} else {
				// Simulate a mouse
				int amount = current_config->rules[i].rdata;
				int x = 0, y = 0, w = 0;
				switch (current_config->rules[i].rmode) {
				case MOUSEMV_UPDOWN:
					y = rcfg.state[i] ? amount : -amount;
					rcfg.state[i] ^= 1;
					break;
				case MOUSEMV_LEFTRIGHT:
					x = rcfg.state[i] ? amount : -amount;
					rcfg.state[i] ^= 1;
					break;
				case MOUSEMV_WUPDOWN:
					w = rcfg.state[i] ? amount : -amount;
					rcfg.state[i] ^= 1;
					break;
				case MOUSEMV_CIRCLE:
					// Move in a circle using 8 steps
					{
						const struct {
							uint32_t cos, sin;
						} sc_tbl[8] = {
							{  256,   0 },
							{  181, 181 },
							{   0,  256 },
							{ -181, 181 },
							{ -256,   0 },
							{ -181,-181 },
							{   0, -256 },
							{  181,-181 },
						};
						uint32_t s = rcfg.state[i]++;
						x = amount * (sc_tbl[s].cos - sc_tbl[(s - 1) & 7].cos) >> 8;
						y = amount * (sc_tbl[s].sin - sc_tbl[(s - 1) & 7].sin) >> 8;
					}
					break;
				case MOUSEMV_RANDOM:
					x = (amount * (rand() - RAND_MAX/2)) / (RAND_MAX/2);
					y = (amount * (rand() - RAND_MAX/2)) / (RAND_MAX/2);
					break;
				}
				// Send update!
				if (x || y) {
					uint8_t upd[4] = { 0, x, y, w };
					push_usb(0x81, upd, sizeof(upd));
				}
			}
		}
	}
}

