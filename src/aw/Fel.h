#pragma once

// Allwinner FEL protocol (PhoenixSuit / sunxi-tools compatible).
//
// FEL runs in the BootROM of Allwinner SoCs (VID 1F3A, PID 0xEFE8).
// The USB framing follows sunxi-tools:
//
//   1) host -> device: a 32-byte aw_usb_request
//        magic "AWUC", length, 0x0c000000, request type (0x11 read / 0x12 write),
//        length2, 10 bytes padding
//   2) for a FEL_WRITE phase: a 16-byte aw_fel_request
//        request u32, address u32, length u32, pad u32
//   3) optional data phase
//   4) device -> host: a 13-byte "AWUS" acknowledgement
//
// FEL command results are read with a separate AW_USB_READ(8) transaction.
// FEL_VER returns a 32-byte AWUSBFEX payload.
//
// TODO(protocols): response size (8 vs 4 bytes) differs on a few BROM builds;
// the client accepts a short response and only validates the status word.
// Verify against hardware, see docs/PROTOCOLS.md.

#include <QByteArray>
#include <QString>

#include "usb/ProtocolSupport.h"
#include "usb/UsbTransport.h"

namespace gf::aw {

inline constexpr quint32 kHello = 0x001; // FEL_VERSION
inline constexpr quint32 kVer = 0x001;
inline constexpr quint32 kWrite = 0x101; // FEL_1_WRITE
inline constexpr quint32 kExec = 0x102;  // FEL_1_EXEC
inline constexpr quint32 kRead = 0x103;  // FEL_1_READ
inline constexpr quint32 kRun = 0x104;

inline constexpr quint32 kRequestSize = 16;   // aw_fel_request
inline constexpr quint32 kUsbRequestSize = 32; // aw_usb_request
inline constexpr quint32 kResponseSize = 8;   // read status payload
inline constexpr quint32 kAckSize = 13;       // "AWUS" + 9 zero bytes
inline constexpr quint32 kVersionSize = 32;
inline constexpr quint8 kUsbRead = 0x11;
inline constexpr quint8 kUsbWrite = 0x12;
inline constexpr qsizetype kMaxChunk = 16 * 1024;

struct Request {
    quint32 request = 0;
    quint32 length = 0;
    quint32 address = 0;
    quint32 reserved = 0;

    QByteArray encode() const;
    static Request decode(const QByteArray& raw);
};

struct Response {
    bool valid = false;
    quint32 status = 0;
    quint32 tag = 0;
    QByteArray raw;

    bool ok() const { return valid && status == 0; }
    static Response decode(const QByteArray& raw);
};

QByteArray encodeRequest(quint32 request, quint32 length = 0, quint32 address = 0);

struct VersionInfo {
    bool valid = false;
    QString magic;
    quint32 version = 0;
    quint32 id = 0; // SoC id
    QByteArray firmware;
    QString error;
};

VersionInfo parseVersion(const QByteArray& payload);
QString socName(quint32 id);

class Client {
public:
    explicit Client(usb::IUsbTransport& transport, proto::Reporter reporter = {});

    /// FEL_VER / HELLO: returns the BootROM version + SoC id.
    VersionInfo hello(int timeoutMs = 5000);

    QByteArray read(quint32 address, quint32 length, int timeoutMs = 30000);
    void write(quint32 address, const QByteArray& data, qsizetype chunkSize = kMaxChunk,
               int progressBase = 0, int progressSpan = 0, int timeoutMs = 30000);
    void exec(quint32 address, int timeoutMs = 5000);

    /// write(addr, image) followed by exec(addr).
    void run(quint32 address, const QByteArray& image, int timeoutMs = 30000);

private:
    void sendUsbRequest(quint8 type, quint32 length, int timeoutMs);
    void readAck(int timeoutMs);
    QByteArray readData(quint32 length, int timeoutMs, bool resyncVersion = false);
    void sendFelRequest(quint32 request, quint32 address, quint32 length, int timeoutMs);
    QByteArray readUsb(quint32 length, int timeoutMs);
    void writeUsb(const QByteArray& data, int timeoutMs);
    void readFelStatus(int timeoutMs);
    usb::IUsbTransport& m_transport;
    proto::Reporter m_reporter;
    QByteArray m_rx;
};

} // namespace gf::aw
