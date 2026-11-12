#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"
#include "drivers/serial.h"
#include "drivers/serial_usb_vcp.h"
#include "drivers/time.h"
#include "usbd_core.h"
#include "usbd_cdc_acm.h"
#include "vcp_hal/cdc_acm.h"

#define TX_CAPACITY 2048U
#define CDC_PACKET_SIZE 64U
#define TX_STALL_MS 200U

static usbd_event_handler_t notifyUsb;
static struct usbd_endpoint *inEndpoint;
static struct usbd_endpoint *outEndpoint;
static serialPort_t *expectEmptyOnOutArm;
static bool configured;
static bool pending;
static bool outArmed;
static bool refuseStart;
static bool autoComplete;
static uint32_t clockMs;
static uint32_t resetAt;
static const uint8_t *inData;
static uint8_t *outData;
static uint32_t inLength;
static uint8_t submitted[TX_CAPACITY];
static uint8_t received[TX_CAPACITY * 3];
static uint32_t receivedLength;
static uint32_t dataStarts;
static uint32_t zlpStarts;
static uint32_t outStarts;

bool usb_device_is_configured(uint8_t busid)
{
    assert(busid == 0);
    return configured;
}

uint16_t usbd_get_ep_mps(uint8_t busid, uint8_t ep)
{
    assert(busid == 0 && (ep == 0x81 || ep == 0x02));
    return CDC_PACKET_SIZE;
}

void usbd_desc_register(uint8_t busid, const struct usb_descriptor *desc)
{
    assert(busid == 0 && desc);
}

void usbd_add_interface(uint8_t busid, struct usbd_interface *intf)
{
    assert(busid == 0 && intf);
}

struct usbd_interface *usbd_cdc_acm_init_intf(uint8_t busid, struct usbd_interface *intf)
{
    assert(busid == 0 && intf);
    return intf;
}

void usbd_add_endpoint(uint8_t busid, struct usbd_endpoint *ep)
{
    assert(busid == 0 && ep);
    if (ep->ep_addr & 0x80) {
        inEndpoint = ep;
    } else {
        outEndpoint = ep;
    }
}

int usbd_initialize(uint8_t busid, uintptr_t reg_base, usbd_event_handler_t event_handler)
{
    assert(busid == 0 && reg_base == USB_BASE);
    notifyUsb = event_handler;
    notifyUsb(0, USBD_EVENT_INIT);
    return 0;
}

int usbd_ep_start_read(uint8_t busid, const uint8_t ep, uint8_t *data, uint32_t length)
{
    assert(configured && !outArmed && busid == 0 && ep == 0x02 && data && length == CDC_PACKET_SIZE);
    if (expectEmptyOnOutArm) {
        assert(expectEmptyOnOutArm->vTable->serialTotalRxWaiting(expectEmptyOnOutArm) == 0);
    }
    outArmed = true;
    outData = data;
    outStarts++;
    return 0;
}

int usbd_ep_start_write(uint8_t busid, const uint8_t ep, const uint8_t *data, uint32_t length)
{
    assert(configured && !pending && busid == 0 && ep == 0x81);
    assert(length <= sizeof(submitted));
    if (refuseStart) {
        return -1;
    }
    if (length) {
        assert(data);
        memcpy(submitted, data, length);
        dataStarts++;
    } else {
        assert(!data);
        zlpStarts++;
    }
    inData = data;
    inLength = length;
    pending = true;
    return 0;
}

static void completeIn(void)
{
    assert(pending);
    const uint32_t length = inLength;
    if (length) {
        assert(memcmp(submitted, inData, length) == 0);
        assert(receivedLength + length <= sizeof(received));
        memcpy(received + receivedLength, inData, length);
        receivedLength += length;
    }
    pending = false;
    inEndpoint->ep_cb(0, inEndpoint->ep_addr, length);
}

static void configureUsb(void)
{
    pending = false;
    outArmed = false;
    configured = true;
    notifyUsb(0, USBD_EVENT_CONFIGURED);
}

static void resetUsb(void)
{
    configured = false;
    pending = false;
    outArmed = false;
    notifyUsb(0, USBD_EVENT_RESET);
    assert(!usbVcpIsActive());
    configureUsb();
}

static void sendOut(const uint8_t *data, uint32_t count)
{
    assert(outArmed && outEndpoint && outData && count <= CDC_PACKET_SIZE);
    memcpy(outData, data, count);
    outArmed = false;
    outEndpoint->ep_cb(0, outEndpoint->ep_addr, count);
}

timeMs_t millis(void)
{
    clockMs++;
    if (resetAt && clockMs == resetAt) {
        resetUsb();
    }
    if (autoComplete && pending) {
        completeIn();
    }
    return clockMs;
}

static serialPort_t *freshPort(void)
{
    configured = pending = outArmed = refuseStart = autoComplete = false;
    clockMs = resetAt = dataStarts = zlpStarts = outStarts = receivedLength = 0;
    serialPort_t *port = usbVcpOpen();
    assert(!usbVcpIsActive());
    configureUsb();
    assert(usbVcpIsConnected() && usbVcpIsActive());
    assert(port->vTable->isSerialTransmitBufferEmpty(port));
    assert(port->vTable->serialTotalTxFree(port) == TX_CAPACITY);
    return port;
}

static void checkBusyBeforeSubmission(void)
{
    serialPort_t *port = freshPort();
    const struct serialPortVTable *vcp = port->vTable;

    const uint32_t rawStart = clockMs;
    vcp->serialWrite(port, 'X');
    assert(pending && inLength == 1 && dataStarts == 1);
    vcp->serialWrite(port, 'Y');
    assert(clockMs - rawStart < 10);
    assert(pending && inLength == 1 && dataStarts == 1);
    vcp->beginWrite(port);
    vcp->serialWrite(port, 'a');
    vcp->writeBuf(port, "BC", 2);
    vcp->serialWrite(port, 'd');
    const uint32_t flushStart = clockMs;
    vcp->endWrite(port);
    assert(clockMs - flushStart >= 50 && clockMs - flushStart < 55);
    assert(dataStarts == 1 && receivedLength == 0);
    assert(vcp->serialTotalTxFree(port) == TX_CAPACITY - 6);

    completeIn();
    assert(pending && dataStarts == 2 && inLength == 5);
    completeIn();
    assert(receivedLength == 6 && memcmp(received, "XYaBCd", 6) == 0);
    assert(vcp->isSerialTransmitBufferEmpty(port));
}

static void checkTimeoutAfterSubmission(void)
{
    serialPort_t *port = freshPort();
    const struct serialPortVTable *vcp = port->vTable;
    uint8_t bytes[CDC_PACKET_SIZE];
    memset(bytes, 0x47, sizeof(bytes));

    vcp->beginWrite(port);
    vcp->writeBuf(port, bytes, sizeof(bytes));
    assert(dataStarts == 0);
    vcp->endWrite(port);
    vcp->endWrite(port);
    assert(dataStarts == 1 && pending && !zlpStarts);

    completeIn();
    assert(pending && inLength == 0 && zlpStarts == 1);
    assert(vcp->serialTotalTxFree(port) == TX_CAPACITY - sizeof(bytes));
    vcp->serialWrite(port, 'Y');
    assert(dataStarts == 1 && zlpStarts == 1);
    completeIn();
    assert(dataStarts == 2 && inLength == 1);
    completeIn();
    assert(receivedLength == sizeof(bytes) + 1);
    assert(memcmp(received, bytes, sizeof(bytes)) == 0 && received[sizeof(bytes)] == 'Y');
    assert(vcp->isSerialTransmitBufferEmpty(port));
}

static void checkRefusedSubmission(void)
{
    serialPort_t *port = freshPort();
    const struct serialPortVTable *vcp = port->vTable;

    refuseStart = true;
    vcp->serialWrite(port, 'R');
    assert(dataStarts == 0 && !pending);
    assert(vcp->serialTotalTxFree(port) == TX_CAPACITY - 1);
    refuseStart = false;
    assert(!vcp->isSerialTransmitBufferEmpty(port));
    assert(dataStarts == 1 && pending);
    completeIn();
    assert(receivedLength == 1 && received[0] == 'R');
}

static void checkQueueFullTimeout(void)
{
    serialPort_t *port = freshPort();
    const struct serialPortVTable *vcp = port->vTable;
    uint8_t bytes[TX_CAPACITY];
    memset(bytes, 0xA2, sizeof(bytes));

    vcp->beginWrite(port);
    vcp->writeBuf(port, bytes, sizeof(bytes));
    assert(dataStarts == 1 && vcp->serialTotalTxFree(port) == 0);
    const uint32_t start = clockMs;
    vcp->serialWrite(port, 'F');
    assert(clockMs - start >= TX_STALL_MS - 2 && clockMs - start < TX_STALL_MS + 5);
    assert(dataStarts == 1 && vcp->serialTotalTxFree(port) == 0);

    completeIn();
    assert(zlpStarts == 1 && vcp->serialTotalTxFree(port) == 0);
    completeIn();
    assert(receivedLength == sizeof(bytes) && memcmp(received, bytes, sizeof(bytes)) == 0);
    assert(vcp->serialTotalTxFree(port) == TX_CAPACITY);

    vcp->writeBuf(port, "suffix", 6);
    assert(vcp->serialTotalTxFree(port) == TX_CAPACITY);
    vcp->endWrite(port);
    assert(!pending && dataStarts == 1);

    vcp->beginWrite(port);
    vcp->writeBuf(port, "new", 3);
    vcp->endWrite(port);
    assert(pending && dataStarts == 2);
    completeIn();
    assert(receivedLength == sizeof(bytes) + 3 && memcmp(received + sizeof(bytes), "new", 3) == 0);
}

static void checkSingleByteWritesShareTheFullQueueDeadline(void)
{
    serialPort_t *port = freshPort();
    const struct serialPortVTable *vcp = port->vTable;
    uint8_t bytes[TX_CAPACITY];
    for (uint32_t i = 0; i < sizeof(bytes); i++) {
        bytes[i] = (uint8_t)(i ^ (i >> 8));
    }

    vcp->writeBuf(port, bytes, sizeof(bytes));
    assert(dataStarts == 1 && vcp->serialTotalTxFree(port) == 0);
    const uint32_t stallStart = clockMs;
    vcp->serialWrite(port, 'A');
    assert(clockMs - stallStart >= TX_STALL_MS - 2 && clockMs - stallStart < TX_STALL_MS + 5);
    const uint32_t repeatedStart = clockMs;
    for (uint8_t i = 'B'; i <= 'H'; i++) {
        vcp->serialWrite(port, i);
    }
    assert(clockMs - repeatedStart < 30);

    completeIn();
    assert(pending && !inLength && zlpStarts == 1 && !vcp->serialTotalTxFree(port));
    completeIn();
    assert(vcp->serialTotalTxFree(port) == TX_CAPACITY);

    const uint32_t resumeStart = clockMs;
    vcp->serialWrite(port, 'R');
    assert(clockMs - resumeStart < 5);
    completeIn();
    assert(receivedLength == sizeof(bytes) + 1);
    assert(memcmp(received, bytes, sizeof(bytes)) == 0 && received[sizeof(bytes)] == 'R');

    vcp->writeBuf(port, bytes, sizeof(bytes));
    assert(vcp->serialTotalTxFree(port) == 0);
    const uint32_t nextStall = clockMs;
    vcp->serialWrite(port, 'I');
    assert(clockMs - nextStall >= TX_STALL_MS - 2 && clockMs - nextStall < TX_STALL_MS + 5);
    resetUsb();
    assert(vcp->isSerialTransmitBufferEmpty(port));
}

static void checkResetDuringBufferedWrite(void)
{
    serialPort_t *port = freshPort();
    const struct serialPortVTable *vcp = port->vTable;

    vcp->serialWrite(port, 'X');
    vcp->beginWrite(port);
    vcp->writeBuf(port, "old", 3);
    resetUsb();
    vcp->serialWrite(port, 'Y');
    vcp->endWrite(port);
    assert(dataStarts == 1 && !pending && vcp->isSerialTransmitBufferEmpty(port));

    vcp->beginWrite(port);
    vcp->writeBuf(port, "new", 3);
    vcp->endWrite(port);
    assert(dataStarts == 2 && pending);
    completeIn();
    assert(receivedLength == 3 && memcmp(received, "new", 3) == 0);
}

static void checkResetDuringBulkWrite(void)
{
    serialPort_t *port = freshPort();
    const struct serialPortVTable *vcp = port->vTable;
    uint8_t bytes[TX_CAPACITY + 64];
    memset(bytes, 0xA5, sizeof(bytes));

    vcp->beginWrite(port);
    resetAt = 4;
    vcp->writeBuf(port, bytes, sizeof(bytes));
    vcp->endWrite(port);
    assert(clockMs >= resetAt && dataStarts == 1 && receivedLength == 0 && !pending);
    assert(vcp->serialTotalTxFree(port) == TX_CAPACITY);
    vcp->serialWrite(port, 'Z');
    completeIn();
    assert(receivedLength == 1 && received[0] == 'Z');
}

static void checkQueueWrapAndBulkOrder(void)
{
    serialPort_t *port = freshPort();
    const struct serialPortVTable *vcp = port->vTable;
    uint8_t bytes[TX_CAPACITY * 2 + 16];
    for (uint32_t i = 0; i < sizeof(bytes); i++) {
        bytes[i] = (uint8_t)(i ^ (i >> 8));
    }

    autoComplete = true;
    vcp->serialWrite(port, 'W');
    vcp->beginWrite(port);
    vcp->serialWrite(port, '<');
    vcp->writeBuf(port, bytes, sizeof(bytes));
    vcp->serialWrite(port, '>');
    vcp->endWrite(port);
    assert(vcp->isSerialTransmitBufferEmpty(port));
    assert(receivedLength == sizeof(bytes) + 3);
    assert(memcmp(received, "W<", 2) == 0);
    assert(memcmp(received + 2, bytes, sizeof(bytes)) == 0 && received[sizeof(bytes) + 2] == '>');
}

static void checkConfiguredClearsWrappedRxAndDeferredOut(void)
{
    serialPort_t *port = freshPort();
    const struct serialPortVTable *vcp = port->vTable;
    uint8_t oldData[CDC_PACKET_SIZE];
    memset(oldData, 0xA2, sizeof(oldData));
    sendOut(oldData, 55);
    for (unsigned i = 0; i < 30; i++) {
        sendOut(oldData, sizeof(oldData));
    }
    for (unsigned i = 0; i < 100; i++) {
        assert(vcp->serialRead(port) == 0xA2);
    }
    sendOut(oldData, sizeof(oldData));
    sendOut(oldData, sizeof(oldData)); // The final packet crosses the RX ring end.
    const uint32_t wrappedCount = 55 + 32 * sizeof(oldData) - 100;
    assert(vcp->serialTotalRxWaiting(port) == wrappedCount);
    memset(oldData, 0xB3, sizeof(oldData));
    sendOut(oldData, sizeof(oldData)); // Held outside the ring until it has space.
    assert(!outArmed && vcp->serialTotalRxWaiting(port) == wrappedCount);

    const uint32_t armsBefore = outStarts;
    expectEmptyOnOutArm = port;
    configureUsb(); // A CONFIGURED event without an intervening RESET.
    expectEmptyOnOutArm = NULL;
    assert(outArmed && outStarts == armsBefore + 1);
    assert(vcp->serialTotalRxWaiting(port) == 0);

    const uint8_t newData[] = {0x04, 0x9A, 0x00, 0x0D, 0x16};
    sendOut(newData, sizeof(newData));
    assert(vcp->serialTotalRxWaiting(port) == sizeof(newData));
    for (unsigned i = 0; i < sizeof(newData); i++) {
        assert(vcp->serialRead(port) == newData[i]);
    }
    assert(outArmed && outStarts == armsBefore + 2);
    assert(vcp->serialTotalRxWaiting(port) == 0);
}

int main(void)
{
    checkBusyBeforeSubmission();
    checkTimeoutAfterSubmission();
    checkRefusedSubmission();
    checkQueueFullTimeout();
    checkSingleByteWritesShareTheFullQueueDeadline();
    checkResetDuringBufferedWrite();
    checkResetDuringBulkWrite();
    checkQueueWrapAndBulkOrder();
    checkConfiguredClearsWrappedRxAndDeferredOut();
    puts("UM32 VCP checks passed: asynchronous raw writes, batch flush, shared stall deadline, ordered resume, ZLP, reset, TX wrap, and wrapped/deferred RX on CONFIGURED");
    return 0;
}
