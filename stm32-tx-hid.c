/*
 * Copyright (C) 2016 Paul Fertser <fercerpav@gmail.com>
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencmsis/core_cm3.h>

#define BUTTONS_PINS	0xff00
#define BUTTONS_SHIFT	8

#define NSRST_PIN	GPIO0

/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[5*64];

#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dfu.h>

static usbd_device *usbd_dev;

const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5710,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
	 0x05, 0x01, // USAGE_PAGE (Generic Desktop)
	 0x09, 0x04, // USAGE (Joystick)

	 0xa1, 0x01, // COLLECTION (Application)
	 0xa1, 0x00, // COLLECTION (Physical)

	 0x05, 0x09, // USAGE_PAGE (Button)
	 0x19, 0x01, // USAGE_MINIMUM (Button 1)
	 0x29, 0x08, // USAGE_MAXIMUM (Button 8)
	 0x15, 0x00, // LOGICAL_MINIMUM (0)
	 0x25, 0x01, // LOGICAL_MAXIMUM (1)
	 0x95, 0x08, // REPORT_COUNT (8)
	 0x75, 0x01, // REPORT_SIZE (1)
	 0x81, 0x02, // INPUT (Data,Var,Abs)

	 0x05, 0x01, // USAGE_PAGE (Generic Desktop)
	 0x15, 0x00, // LOGICAL_MINIMUM (0)
	 0x26, 0xff, 0x0f, // LOGICAL_MAXIMUM
	 0x75, 0x10, // REPORT_SIZE

	 0x09, 0x30, // USAGE (X)
	 0x09, 0x31, // USAGE (Y)
	 0x09, 0x32, // USAGE (Z)
	 0x09, 0x33, // USAGE (Rx)
	 0x09, 0x34, // USAGE (Ry)
	 0x09, 0x35, // USAGE (Rz)
	 0x09, 0x36, // USAGE (Throttle)
	 0x09, 0x37, // USAGE (Rudder)
	 0x95, 0x08, // REPORT_COUNT
	 0x81, 0x82, // INPUT (Data,Var,Abs,Vol)

	 0xc0, // END_COLLECTION
	 0xc0 // END_COLLECTION
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
	.hid_descriptor = {
		.bLength = sizeof(hid_function),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	}
};

const struct usb_endpoint_descriptor hid_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 17,
	.bInterval = 0x02,
};

const struct usb_interface_descriptor hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 0, /* boot */
	.bInterfaceProtocol = 0, /* mouse */
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor dfu_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	/* The ST Microelectronics DfuSe application needs this string.
	 * The format isn't documented... */
	.iInterface = 4,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}, {
	.num_altsetting = 1,
	.altsetting = &dfu_iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static int hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x81) ||
	   (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
	   (req->wValue != 0x2200))
		return 0;

	/* Handle the HID report descriptor. */
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

	return 1;
}

static void dfu_detach_complete(usbd_device *dev, struct usb_setup_data *req)
{
	(void)req;
	(void)dev;

	scb_reset_core();
}

static int dfu_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	(void)buf;
	(void)len;
	(void)dev;

	if ((req->bmRequestType != 0x21) || (req->bRequest != DFU_DETACH))
		return 0; /* Only accept class request. */

	*complete = dfu_detach_complete;

	return 1;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				hid_control_request);
	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				dfu_control_request);

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(99999);
	systick_interrupt_enable();
	systick_counter_enable();
}

void sys_tick_handler(void)
{
	static uint8_t buf[1 + 16];
	static uint8_t channels[] = { 0, 1, 2, 3, 4, 5, 6, 7 };

	for (unsigned int i = 0; i < 8; i++) {
		uint8_t channel_array[] = { channels[i] };
		adc_set_regular_sequence(ADC1, 1, channel_array);
		adc_start_conversion_direct(ADC1);
		/* Wait for end of conversion. */
		while (!(adc_eoc(ADC1)))
			;

		uint16_t res = adc_read_regular(ADC1);
		buf[1 + i * 2] = res & 0xff;
		buf[1 + i * 2 + 1] = res >> 8;
	}

	buf[0] = ~(gpio_get(GPIOB, BUTTONS_PINS) >> BUTTONS_SHIFT);
	usbd_ep_write_packet(usbd_dev, 0x81, buf, sizeof(buf));
}

static char serial_no[25];

static const char *usb_strings[] = {
	"libopencm3",
	"STM32 Tx HID adapter",
	serial_no,
	/* This string is used by ST Microelectronics' DfuSe utility. */
	"@Internal Flash   /0x08000000/8*001Ka,56*001Kg",
};

static char *get_dev_unique_id(char *s)
{
	volatile uint8_t *unique_id = (volatile uint8_t *)0x1FFFF7E8;
	int i;

	/* Fetch serial number from chip's unique ID */
	for(i = 0; i < 24; i+=2) {
		s[i] = ((*unique_id >> 4) & 0xF) + '0';
		s[i+1] = (*unique_id++ & 0xF) + '0';
	}
	for(i = 0; i < 24; i++)
		if(s[i] > '9')
			s[i] += 'A' - '9' - 1;

	return s;
}

static void gpio_setup(void)
{
	/* Button inputs */
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_PULL_UPDOWN, BUTTONS_PINS);
	gpio_set(GPIOB, BUTTONS_PINS);

	/* Pull down for nSRST */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, NSRST_PIN);
	gpio_clear(GPIOB, NSRST_PIN);
}

static void adc_setup(void)
{
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, 0xff);

	rcc_periph_clock_enable(RCC_ADC1);

	/* Make sure the ADC doesn't run during config. */
	adc_off(ADC1);

	/* We configure everything for one single conversion. */
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (unsigned int i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibration(ADC1);
}

static void usb_suspend_callback(void)
{
	gpio_set(GPIOB, NSRST_PIN);
	*USB_CNTR_REG |= USB_CNTR_FSUSP;
	*USB_CNTR_REG |= USB_CNTR_LP_MODE;
	SCB_SCR |= SCB_SCR_SLEEPDEEP;
	__WFI();
}

void usb_wakeup_isr(void)
{
	exti_reset_request(EXTI18);
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	*USB_CNTR_REG &= ~USB_CNTR_FSUSP;
	gpio_clear(GPIOB, NSRST_PIN);
}

int main(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	rcc_periph_clock_enable(RCC_GPIOA);

	exti_set_trigger(EXTI18, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI18);
	nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);

	adc_setup();
	gpio_setup();

	/*
	 * Vile hack to reenumerate, physically _drag_ d+ low.
	 * (need at least 2.5us to trigger usb disconnect)
	 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (unsigned int i = 0; i < 800000; i++)
		__asm__("nop");

	get_dev_unique_id(serial_no);

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config,
			     usb_strings, 4,
			     usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);
	usbd_register_suspend_callback(usbd_dev, usb_suspend_callback);

	while (1)
		usbd_poll(usbd_dev);
}
