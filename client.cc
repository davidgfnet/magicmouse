/*
 * Author: David Guillen Fandos (2023) <david@davidgf.net>
 */

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <libusb-1.0/libusb.h>

#include "devconf.h"

#define DEVICE_VID_PID(vid, pid)  ((vid << 16) | pid)

#define CMD_GO_DFU          0xff
#define CMD_CONFIG          0xfe
#define CMD_INFO            0xfd
#define CMD_RESET           0xfc
#define IFACE_NUMBER         0x2
#define TIMEOUT_MS          5000

#define MAGIC_KEY         0xd1bd    // To avoid random driver probes causing mayhem

const std::unordered_set<uint32_t> fakedevs = {
	DEVICE_VID_PID(0x045e, 0x07fd),   // Microsoft / Microsoft Nano Transceiver 1.1
	DEVICE_VID_PID(0x046d, 0xc52e),   // Logitech / USB Receiver     [MK260 Wireless Combo Receiver]
	DEVICE_VID_PID(0x046d, 0xc534),   // Logitech / Unifying Receiver
	DEVICE_VID_PID(0x248a, 0x8367),   // Telink / Wireless Receiver  [Telink Wireless Receiver from Maxxter]
	DEVICE_VID_PID(0x062a, 0x4101),   // MOSART Semi. / 2.4G Keyboard Mouse
};

const char *toggle_modes[] = {
	"4 presses in < 1s",
	"6 presses in < 2s",
	"8 presses in < 3s",
};

static const int CTRL_REQ_TYPE_IN  = LIBUSB_ENDPOINT_IN  | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_INTERFACE;
static const int CTRL_REQ_TYPE_OUT = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_INTERFACE;

void fatal_error(const char * errmsg, int code) {
	fprintf(stderr, "ERROR! %d %s\n", code, errmsg);
	exit(1);
}

int main(int argc, char ** argv) {
	if (argc >= 2 && !strcmp(argv[1], "-h")) {
		std::cout << "Usage: " << argv[0] << " Option" << std::endl;
		std::cout << "  -h     Print this help message" << std::endl;
		std::cout << "  -w     Wipe (factory reset) the device" << std::endl;
		std::cout << "  -r     Reboot the device into DFU mode" << std::endl;
		return 1;
	}

	libusb_context *usbctx = NULL;
	libusb_device **devlist = NULL;

	int result = libusb_init(&usbctx);
	if (result < 0)
		fatal_error("libusb_init failed!", result);

	// Look for devices that could be fake mouse/keyboards :)
	std::vector<std::pair<int, uint32_t>> mdevs;
	auto dcnt = libusb_get_device_list(usbctx, &devlist);
	for (unsigned i = 0; i < dcnt; i++) {
		libusb_device_descriptor dsc;
		if (!libusb_get_device_descriptor(devlist[i], &dsc)) {
			uint32_t usbid = DEVICE_VID_PID(dsc.idVendor, dsc.idProduct);
			if (fakedevs.count(usbid)) {
				// The device matched, now try and connect to it to verify it's a fake one
				libusb_device_handle *devh = NULL;
				if (!libusb_open(devlist[i], &devh)) {
					libusb_set_auto_detach_kernel_driver(devh, 1);
					if (!libusb_claim_interface(devh, IFACE_NUMBER)) {
						uint32_t info[256];
						int ret = libusb_control_transfer(devh, CTRL_REQ_TYPE_IN, CMD_INFO, MAGIC_KEY,
						                                  IFACE_NUMBER, (uint8_t*)info, sizeof(info), TIMEOUT_MS);
						if (ret > 0) {
							// Check whether the config seems reasonable
							bool seemsok = (info[0] == HDR_MAGIC);
							if (seemsok) {
								std::cout << "Found fake kbd/mouse device " << std::endl;
								mdevs.emplace_back(i, usbid);
							}
							else
								std::cout << "Found device but seems legit" << std::endl;
						}
						libusb_release_interface(devh, 0);
					}
					libusb_close(devh);
				}
			}
		}
	}

	// Only proceed if we have more than one device, otherwise there's ambiguity
	if (mdevs.empty())
		std::cerr << "Found no matching devices!" << std::endl;
	else if (mdevs.size() > 1)
		std::cerr << "Found " << mdevs.size() << " devices, not supported atm." << std::endl;
	else {
		uint32_t devid = mdevs[0].second;
		std::cerr << "Found device " << std::hex << (devid >> 16) << ":" << (devid & 0xffff) << std::endl;

		libusb_device_handle *devh = NULL;
		if (!libusb_open(devlist[mdevs[0].first], &devh)) {
			libusb_set_auto_detach_kernel_driver(devh, 1);
			if (!libusb_claim_interface(devh, IFACE_NUMBER)) {
				uint8_t cfgb[1024];
				int ret = libusb_control_transfer(devh, CTRL_REQ_TYPE_IN, CMD_CONFIG, MAGIC_KEY,
				                                  IFACE_NUMBER, cfgb, sizeof(cfgb), TIMEOUT_MS);
				if (ret > 0) {
					const t_devconf *cfg = (t_devconf *)cfgb;
					std::cout << "Device version " << std::hex << cfg->version << std::endl;
					std::cout << "Revision " << cfg->cfg_rev << std::endl;

					std::cout << "Emulating " << std::hex << cfg->usb_vidpid << " ";
					std::cout << cfg->manufacturer << " / " << cfg->product << std::endl;

					if (cfg->toggle_key == 0xff)
						std::cout << "Enable/Disable key is disabled" << std::endl;
					else
						std::cout << "Enable/Disable key is enabled with mode " << toggle_modes[cfg->toggle_mode] << std::endl;

					for (unsigned i = 0; i < cfg->rule_count; i++) {
						std::cout << "Rule [" << i << "]:" << std::endl;
						std::cout << "  Period: " << std::dec << cfg->rules[i].period << "ms" << std::endl;
					}
				}
				// Perform an action?
				if (argc >= 2 && !strcmp(argv[1], "-w")) {
					
				}
				else if (argc >= 2 && !strcmp(argv[1], "-r")) {
					if (libusb_control_transfer(devh, CTRL_REQ_TYPE_OUT, CMD_GO_DFU, MAGIC_KEY, IFACE_NUMBER, 0, 0, TIMEOUT_MS) < 0)
						std::cout << "Reboot into DFU mode failed!" << std::endl;
					else
						std::cout << "Reboot into DFU mode succesful!" << std::endl;
				}
				libusb_release_interface(devh, 0);
			}
			else
				std::cerr << "Error. Could not claim the device interface" << std::endl;
			libusb_close(devh);
		}
		else
			std::cerr << "Error. Could not open the USB device" << std::endl;
	}

	libusb_free_device_list(devlist, 1);
	libusb_exit(usbctx);
	return 0;
}

