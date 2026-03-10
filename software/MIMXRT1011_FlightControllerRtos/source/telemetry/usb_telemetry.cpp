// source/telemetry/usb_telemetry.cpp
#include "telemetry/usb_telemetry.h"
#include <cstring>

extern "C" {
    #include "fsl_device_registers.h"
    #include "clock_config.h"
    #include "board.h"
    #include "usb_device_config.h"
    #include "usb.h"
    #include "usb_device.h"
    #include "usb_device_class.h"
    #include "usb_device_cdc_acm.h"
    #include "usb_device_descriptor.h"
    #include "usb_phy.h"
    #include "virtual_com.h"
}

// Ensure CONTROLLER_ID is defined for the RT1011 EHCI controller
#ifndef CONTROLLER_ID
#define CONTROLLER_ID kUSB_ControllerEhci0
#endif

static usb_cdc_vcom_struct_t s_cdcVcom;
static usb_cdc_acm_info_t s_usbCdcAcmInfo;
static volatile bool s_txBusy = false;
static volatile bool s_isConnected = false;

// Allocate a static buffer for transmission aligned for USB DMA
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_sendBuffer[DATA_BUFF_SIZE];

// Forward declarations for C callbacks
extern "C" {
    usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param);
    usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);
    void USB_OTG1_IRQHandler(void);
}

extern usb_device_class_struct_t g_UsbDeviceCdcVcomConfig;

static usb_device_class_config_struct_t s_cdcAcmConfig[1] = {{
    USB_DeviceCdcVcomCallback,
    0,
    &g_UsbDeviceCdcVcomConfig,
}};

static usb_device_class_config_list_struct_t s_cdcAcmConfigList = {
    s_cdcAcmConfig,
    USB_DeviceCallback,
    1,
};

void UsbTelemetry::init() {
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };

    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
    CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);

    s_cdcVcom.speed = USB_SPEED_FULL;
    s_cdcVcom.attach = 0;
    s_cdcVcom.cdcAcmHandle = nullptr;
    s_cdcVcom.deviceHandle = nullptr;

    USB_DeviceClassInit(CONTROLLER_ID, &s_cdcAcmConfigList, &s_cdcVcom.deviceHandle);
    s_cdcVcom.cdcAcmHandle = s_cdcAcmConfigList.config->classHandle;

    const IRQn_Type usbDeviceEhciIrqs[] = USBHS_IRQS;
    uint8_t irqNumber = usbDeviceEhciIrqs[CONTROLLER_ID - kUSB_ControllerEhci0];

    NVIC_SetPriority(static_cast<IRQn_Type>(irqNumber), USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ(static_cast<IRQn_Type>(irqNumber));

    USB_DeviceRun(s_cdcVcom.deviceHandle);
}

bool UsbTelemetry::isConnected() {
    return s_isConnected && (s_cdcVcom.attach == 1);
}

bool UsbTelemetry::isTxBusy() {
    return s_txBusy;
}

void UsbTelemetry::send(const uint8_t* data, std::size_t length) {
    if (length > sizeof(s_sendBuffer)) {
        length = sizeof(s_sendBuffer);
    }

    std::memcpy(s_sendBuffer, data, length);
    s_txBusy = true;

    USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_sendBuffer, length);
}

extern "C" void USB_OTG1_IRQHandler(void) {
    USB_DeviceEhciIsrFunction(s_cdcVcom.deviceHandle);
}

extern "C" usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param) {
    usb_status_t error = kStatus_USB_InvalidRequest;
    usb_device_cdc_acm_request_param_struct_t *acmReqParam = static_cast<usb_device_cdc_acm_request_param_struct_t *>(param);

    switch (event) {
        case kUSB_DeviceCdcEventSendResponse:
            s_txBusy = false;
            error = kStatus_USB_Success;
            break;

        case kUSB_DeviceCdcEventSetControlLineState:
            s_usbCdcAcmInfo.dteStatus = acmReqParam->setupValue;
            s_isConnected = (s_usbCdcAcmInfo.dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) != 0;

            if (s_cdcVcom.attach == 1) {
                s_cdcVcom.startTransactions = 1;
            }
            error = kStatus_USB_Success;
            break;

        case kUSB_DeviceCdcEventSetLineCoding:
            error = kStatus_USB_Success;
            break;

        default:
            break;
    }

    return error;
}

extern "C" usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param) {
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint8_t *temp8 = static_cast<uint8_t *>(param);

    switch (event) {
        case kUSB_DeviceEventBusReset:
            s_cdcVcom.attach = 0;
            s_cdcVcom.currentConfiguration = 0U;
            s_isConnected = false;
            error = kStatus_USB_Success;

            if (kStatus_USB_Success == USB_DeviceClassGetSpeed(CONTROLLER_ID, &s_cdcVcom.speed)) {
                USB_DeviceSetSpeed(handle, s_cdcVcom.speed);
            }
            break;

        case kUSB_DeviceEventSetConfiguration:
            if (*temp8 == 0U) {
                s_cdcVcom.attach = 0;
                s_cdcVcom.currentConfiguration = 0U;
                error = kStatus_USB_Success;
            } else if (*temp8 == USB_CDC_VCOM_CONFIGURE_INDEX) {
                s_cdcVcom.attach = 1;
                s_cdcVcom.currentConfiguration = *temp8;
                error = kStatus_USB_Success;
            }
            break;

        case kUSB_DeviceEventGetDeviceDescriptor:
            if (param != nullptr) {
                error = USB_DeviceGetDeviceDescriptor(handle, static_cast<usb_device_get_device_descriptor_struct_t *>(param));
            }
            break;

        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (param != nullptr) {
                error = USB_DeviceGetConfigurationDescriptor(handle, static_cast<usb_device_get_configuration_descriptor_struct_t *>(param));
            }
            break;

        case kUSB_DeviceEventGetStringDescriptor:
            if (param != nullptr) {
                error = USB_DeviceGetStringDescriptor(handle, static_cast<usb_device_get_string_descriptor_struct_t *>(param));
            }
            break;

        default:
            break;
    }

    return error;
}
