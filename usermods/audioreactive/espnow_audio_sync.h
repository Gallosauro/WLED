/**
 * @file    espnow_audio_sync.h
 * @brief   Low-latency ESP-NOW transport for WLED audioreactive sync.
 *
 * Drop-in replacement for the UDP multicast path used by WLED's
 * audioreactive usermod. Keeps the exact same V2 audioSyncPacket
 * wire-payload (44 bytes), so receivers process audio identically to
 * before.
 *
 * Features:
 *   - ESP-NOW broadcast (~2-5 ms one-way vs ~8-30 ms UDP/Wi-Fi)
 *   - 16-bit sequence number + receiver-side dedup
 *   - Each packet sent twice back-to-back (cheap packet-loss insurance)
 *
 * Design:
 *   This module DOES NOT initialise ESP-NOW itself. It piggy-backs on
 *   WLED's existing QuickEspNow stack, which WLED brings up when
 *   `enableESPNow == true` in the user's configuration. Required steps
 *   on the user's side:
 *     1. Build WLED without `-D WLED_DISABLE_ESPNOW` (the default).
 *     2. Enable "ESP-NOW" in WLED's Sync Interfaces page (sets
 *        `enableESPNow = true`).
 *     3. Select "ESP-NOW (low-latency)" as the Audio Sync Transport in
 *        WLED's Sound Settings page (sets `audioSyncTransport = 1`).
 *
 *   Reception: WLED's core ESP-NOW dispatcher (espNowReceiveCB in
 *   wled00/udp.cpp) calls UsermodManager::onEspNowMessage() on every
 *   incoming packet. The audioreactive usermod's onEspNowMessage()
 *   override forwards each packet to handleIncomingPacket() here -- we
 *   claim audio-sync packets by magic-byte match and let everything
 *   else (WiZmote remote, ESP-NOW state-sync) fall through to WLED's
 *   normal handlers untouched.
 *
 *   Transmission: send() calls esp_now_send() directly. This works
 *   because WLED's QuickEspNow has already initialised the ESP-NOW
 *   stack and added the broadcast peer; we just emit a packet on top
 *   of that.
 *
 * @note Targets ESP32 / ESP32-S3 (Arduino-ESP32 v2.x / IDF v4.4+).
 *       On non-ESP32 targets, or when WLED is built with
 *       `-D WLED_DISABLE_ESPNOW`, the API compiles to no-op stubs so
 *       the rest of the audioreactive usermod still builds cleanly.
 * @note API is intentionally raw bytes (void* + size_t) so this header
 *       does not require visibility of WLED's nested
 *       `AudioReactive::audioSyncPacket` type.
 *
 * Wire-up (3 calls from audio_reactive.cpp):
 *   1. begin(channel, isSender) - in connected(), after WLED's normal
 *      multicast init. Safe to call on every reconnect; the broadcast
 *      peer is refreshed.
 *   2. send(&pkt, sizeof(pkt))  - in transmitAudioData(), as an
 *                                  alternative to fftUdp.write(...).
 *   3. poll(&pkt, sizeof(pkt))  - in receiveAudioData(), as an
 *                                  alternative to fftUdp.parsePacket().
 *   plus a one-line onEspNowMessage() override in the AudioReactive
 *   class that calls handleIncomingPacket() to receive packets via
 *   WLED's UsermodManager dispatch.
 *
 * @see EDITS_COMPLETE.md for the precise call-site changes.
 */

#pragma once

#include <Arduino.h>   // base types + FreeRTOS portMUX + memcpy/memset (via <cstring>)

#if defined(ARDUINO_ARCH_ESP32) && !defined(WLED_DISABLE_ESPNOW)
// -----------------------------------------------------------------------
// Full implementation. Active only when ESP-NOW is available -- the
// firmware target is an ESP32 family chip AND the user has not built
// with -D WLED_DISABLE_ESPNOW (which would strip WLED's own QuickEspNow
// stack and the WiZmote / ESP-NOW-state-sync features along with it).
// Each of the four IDF includes below pulls in a distinct ESP-IDF
// component; there is no umbrella header that covers them all.
// -----------------------------------------------------------------------
#include <WiFi.h>             // WiFi.mode(), WiFi.disconnect(), WiFi.channel()
#include <esp_now.h>          // esp_now_add_peer / mod_peer / send (init done by WLED)
#include <esp_wifi.h>         // WIFI_IF_STA

/**
 * @namespace espnowAudioSync
 * @brief    Encapsulates the ESP-NOW audio-sync transport state and API.
 *
 * The namespace is meant to be included from exactly one .cpp per
 * firmware image, since it uses file-scope statics for its state.
 */
namespace espnowAudioSync {

/** @brief Magic number identifying audio-sync packets on the wire. */
static constexpr uint16_t EN_MAGIC      = 0xA53E;

/** @brief On-the-wire protocol version. Bump on incompatible changes. */
static constexpr uint8_t  EN_VERSION    = 1;

/** @brief Raw payload size in bytes (= sizeof V2 audioSyncPacket). */
static constexpr size_t   EN_PAYLOAD    = 44;

/** @brief Preamble size in bytes (magic + version + reserved + seq). */
static constexpr size_t   EN_PREAMBLE   = 6;

/** @brief Total on-the-wire packet size (preamble + payload). */
static constexpr size_t   EN_PACKET     = EN_PREAMBLE + EN_PAYLOAD;

/** @brief ESP-NOW broadcast destination MAC (FF:FF:FF:FF:FF:FF). */
static const     uint8_t  BROADCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/**
 * @brief  On-the-wire ESP-NOW packet layout.
 *
 * Packed to ensure byte-identical layout across senders/receivers.
 * Carries a 6-byte preamble followed by the raw 44-byte audioSyncPacket
 * payload from WLED's audioreactive usermod.
 */
struct __attribute__((packed)) ENPacket {
    uint16_t magic;                ///< Equals EN_MAGIC (0xA53E).
    uint8_t  version;              ///< Equals EN_VERSION (1).
    uint8_t  reserved;             ///< Reserved for future protocol use; senders MUST set to 0.
    uint16_t seq;                  ///< Monotonically increasing per sender.
    uint8_t  payload[EN_PAYLOAD];  ///< Raw bytes of the V2 audioSyncPacket.
};
static_assert(sizeof(ENPacket) == EN_PACKET, "ENPacket layout drift");

/** @brief Configured channel for the broadcast peer (0 = follow current Wi-Fi channel). */
static uint8_t   _channel = 0;

/** @brief True on the master (FFT source), false on slaves. */
static bool      _isSender = false;

/** @brief True once begin() has succeeded. Subsequent begin() calls refresh state. */
static bool      _initialised = false;

/**
 * @brief  True if begin() found ESP-NOW uninitialised and brought it up
 *         itself (standalone mode). False if begin() found ESP-NOW already
 *         initialised by WLED and just attached as a peer (hosted mode).
 *
 *         Standalone mode lets the user run ESP-NOW audio sync WITHOUT
 *         ticking "Enable ESP-NOW" in WLED Network Settings. This is a
 *         direct workaround for the WLED 16.0.0 bug where ticking that
 *         checkbox crashes the master on effect change. Cost: in
 *         standalone mode we own the ESP-NOW receive callback outright,
 *         so WLED's WiZmote-remote and ESP-NOW-state-sync features will
 *         not work on this board. (Audio sync is what the user wanted —
 *         the other ESP-NOW features stay disabled along with the bug.)
 */
static bool      _standaloneMode = false;

/** @brief Sender's transmit sequence counter (wraps at 65535). */
static uint16_t  _txSeq = 0;

/** @brief Set to true by handleIncomingPacket() when a fresh frame is latched. */
static volatile bool _rxReady = false;

/** @brief Latched copy of the most recent audio frame's raw bytes. */
static uint8_t   _rxPayload[EN_PAYLOAD];

/** @brief Sequence number of the most recently accepted frame (for dedup). */
static uint16_t  _lastRxSeq = 0;

/** @brief False until the first frame has been received post-boot. */
static bool      _hasLastRxSeq = false;

/** @brief Idle window (ms) after which the slave will accept a new master MAC.
 *
 *         Once a slave has accepted a valid audio-sync packet, it sticks to
 *         that sender's MAC and ignores anyone else on the same channel. If
 *         no packets arrive from the locked sender for this many ms, the lock
 *         is released and the slave will accept the next valid sender it
 *         hears (so swapping the master physically still works after ~30 s).
 */
static const uint32_t _MAC_UNLOCK_MS = 30000;

/** @brief Locked sender's MAC address (only valid while _hasLockedSender). */
static uint8_t   _lockedSenderMac[6] = {0};

/** @brief True once the slave has accepted its first valid audio-sync packet. */
static bool      _hasLockedSender = false;

/** @brief millis() of the most recent accepted packet from the locked sender. */
static uint32_t  _lastLockedSenderMs = 0;

/** @brief Counter: packets matching magic + version but coming from a
 *         different MAC than the locked one. Surfaces neighbour WLED setups
 *         or rogue senders on the same channel. */
static uint32_t  _stat_foreignSender = 0;

/** @brief Master switch for the receive-side MAC lock. When false, packets
 *         from any valid sender are accepted (legacy behaviour). When true
 *         (default), the first valid sender wins and others are ignored.
 *         Toggled from the usermod settings page (sync:macLock).
 */
static bool      _macLockEnabled = true;

/**
 * @brief FreeRTOS mutex protecting the receiver's latched state.
 *
 * Created lazily in begin(). We deliberately use a FreeRTOS mutex
 * (xSemaphoreCreateMutex) here instead of the more obvious portMUX +
 * portENTER_CRITICAL: the latter disables interrupts on the current
 * core, which can stall the LED RMT/I2S drivers and cause visible
 * pixel glitches. A mutex yields the task cooperatively and leaves
 * interrupts enabled, so LED timing stays clean. The critical sections
 * here are tiny (44-byte memcpy + a few field writes), so contention
 * is essentially zero in practice.
 */
static SemaphoreHandle_t _rxMutex = nullptr;

/** @brief Counter: unique frames delivered to poll(). */
static uint32_t _stat_rx = 0;

/** @brief Counter: duplicate copies discarded by the dedup logic. */
static uint32_t _stat_dupes = 0;

/** @brief Counter: sequence-number gaps observed (i.e. lost frames). */
static uint32_t _stat_gaps = 0;

/** @brief Counter: packets successfully queued for TX. */
static uint32_t _stat_tx = 0;

/** @brief Counter: TX attempts where the send to the IDF queue failed. */
static uint32_t _stat_tx_err = 0;

/**
 * @brief  TX backoff counter. Set to N after a failed esp_now_send() so the
 *         next N calls to send() skip transmission, allowing the IDF driver
 *         queue to drain. Prevents audio sync at 50 Hz from monopolising the
 *         ESP-NOW radio and starving WLED's own state-sync / WiZmote traffic
 *         (which would otherwise lock up the master when the user changes an
 *         effect or colour).
 */
static uint8_t _txSkipNextN = 0;

// Forward declaration: receive callback used in standalone mode (when WLED's
// ESP-NOW is disabled and we own the radio). Body is below, after
// handleIncomingPacket so it can call into it directly. The signature varies
// across ESP-IDF major versions; we pick the one matching the toolchain.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void _audioSyncStandaloneRecvCb(const esp_now_recv_info_t * info, const uint8_t * data, int len);
#else
static void _audioSyncStandaloneRecvCb(const uint8_t * mac, const uint8_t * data, int len);
#endif

/**
 * @brief  Initialise (or refresh) the audio-sync transport.
 *
 * Call from the usermod's `connected()` callback after Wi-Fi is up.
 * Safe to call on every Wi-Fi (re)connect: the first call creates the
 * receive-state mutex and registers the broadcast peer; subsequent
 * calls refresh the peer's channel so the module survives mesh
 * roaming to a new AP on a different channel.
 *
 * @note This function does NOT initialise ESP-NOW itself -- it expects
 *       WLED's QuickEspNow stack to have already done so (because the
 *       user has enabled `enableESPNow` in WLED's settings). If
 *       ESP-NOW is not initialised, begin() returns false and the
 *       audioreactive usermod silently falls back to the UDP transport.
 *
 * @param channel   Wi-Fi channel for the broadcast peer (0 = follow
 *                  the current Wi-Fi channel). Use bit 0 of WLED's
 *                  Sound Settings espnow-channel field if you've
 *                  enabled the channel selector; otherwise pass 0.
 * @param isSender  true on the FFT-producing master; false on receivers.
 *                  Use bit 0 of WLED's `audioSyncEnabled`.
 * @return true on success (broadcast peer is ready), false otherwise
 *         (typically because WLED's ESP-NOW stack is not initialised).
 */
inline bool begin(uint8_t channel, bool isSender) {
    _isSender = isSender;
    _channel  = channel;

    // Lazy-create the rx mutex on first call; reused on subsequent
    // begin() calls (Wi-Fi reconnects).
    if (_rxMutex == nullptr) {
        _rxMutex = xSemaphoreCreateMutex();
        if (_rxMutex == nullptr) return false;
    }

    // Register or refresh the broadcast peer on top of WLED's already-
    // initialised ESP-NOW stack.
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = _channel;   // 0 = current Wi-Fi channel
    peer.encrypt = false;
    peer.ifidx   = WIFI_IF_STA;

    esp_err_t res = esp_now_add_peer(&peer);

    if (res == ESP_ERR_ESPNOW_NOT_INIT) {
        // WLED's ESP-NOW stack is NOT up (the user has not ticked
        // "Enable ESP-NOW" in Network Settings, or that init crashed and
        // they backed it out). Bring ESP-NOW up ourselves in standalone
        // mode so audio sync still works without that checkbox.
        //
        // Wi-Fi must already be running (esp_wifi_start has been called)
        // for esp_now_init to succeed -- and it will be, because this
        // function is invoked from the usermod's connected() callback
        // which only fires after Wi-Fi has connected.
        if (esp_now_init() != ESP_OK) return false;

        // Install our own receive callback. This overwrites any previous
        // recv cb in the IDF (there shouldn't be one, since WLED's
        // ESP-NOW is off, but esp_now_register_recv_cb is idempotent).
        if (esp_now_register_recv_cb(_audioSyncStandaloneRecvCb) != ESP_OK) {
            // If callback registration fails we can't receive, but we
            // can still transmit. Continue rather than failing outright.
        }
        _standaloneMode = true;

        // Now retry adding the broadcast peer.
        res = esp_now_add_peer(&peer);
    }

    if (res == ESP_ERR_ESPNOW_EXIST) {
        // Peer was already registered (e.g. from a previous begin()
        // call). Update its channel in case we roamed to a different AP.
        esp_now_mod_peer(&peer);
    } else if (res != ESP_OK) {
        return false;
    }

    _initialised = true;
    return true;
}

/**
 * @brief  Broadcast one audio-sync frame from the master.
 *
 * Sends two back-to-back copies of the same packet for packet-loss
 * resilience; receivers dedup on the sequence number. Caller passes the
 * raw 44-byte V2 audioSyncPacket via `payload` + `payloadLen`.
 *
 * @param payload     Pointer to a buffer containing the V2 audio sync
 *                    packet (typically `&transmitData`).
 * @param payloadLen  Must equal `EN_PAYLOAD` (44). Other values are
 *                    rejected.
 * @return true if at least one of the two copies was successfully
 *         queued for transmit; false otherwise.
 */
inline bool send(const void * payload, size_t payloadLen) {
    if (!_initialised || !_isSender) return false;
    if (payloadLen != EN_PAYLOAD) return false;

    // Backoff: if the previous send failed (queue full), skip a couple of
    // 20 ms slots so the IDF driver queue can drain. This is what allows
    // WLED's own ESP-NOW traffic (state-sync packets fired when the user
    // changes an effect or colour, plus WiZmote remote handling) to get
    // through. Without it, audio sync at 50 Hz monopolises the radio and
    // the state-notify packet starves -- eventually locking up the master.
    if (_txSkipNextN > 0) {
        _txSkipNextN--;
        return false;
    }

    ENPacket ep;
    memset(&ep, 0, sizeof(ep));
    ep.magic   = EN_MAGIC;
    ep.version = EN_VERSION;
    ep.seq     = ++_txSeq;
    memcpy(ep.payload, payload, EN_PAYLOAD);

    // Send ONE copy only. We used to send two back-to-back for cheap
    // packet-loss resilience, but at 50 Hz that's 100 ESP-NOW broadcasts
    // per second, which saturates the IDF driver's tiny TX queue (~3 slots)
    // and crowds out everything else on the radio. Sequence-number dedup on
    // the receiver still works fine with a single packet per slot.
    esp_err_t r = esp_now_send(BROADCAST_MAC, (uint8_t*)&ep, sizeof(ep));
    if (r == ESP_OK) {
        _stat_tx++;
        return true;
    }

    // Send failed (typically ESP_ERR_ESPNOW_NO_MEM = TX queue full). Skip
    // the next two send() calls (~40 ms at 50 Hz) so the queue drains and
    // other ESP-NOW traffic can squeeze through.
    _stat_tx_err++;
    _txSkipNextN = 2;
    return false;
}

/**
 * @brief  Retrieve the most-recent received audio frame, if any.
 *
 * Returns true exactly once per arriving frame; subsequent calls
 * return false until another frame arrives. The raw 44-byte payload is
 * copied into `out`, suitable for passing directly to WLED's
 * `decodeAudioData()` cast as `uint8_t*`.
 *
 * @param[out] out     Buffer to fill with the V2 audioSyncPacket
 *                     payload (must be at least 44 bytes).
 * @param      outLen  Capacity of `out` in bytes (must be >= EN_PAYLOAD).
 * @return true if a fresh frame was returned in `out`; false otherwise.
 */
inline bool poll(void * out, size_t outLen) {
    if (!_rxReady) return false;
    if (outLen < EN_PAYLOAD) return false;
    if (_rxMutex == nullptr) return false;
    if (xSemaphoreTake(_rxMutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;

    memcpy(out, _rxPayload, EN_PAYLOAD);
    _rxReady = false;

    xSemaphoreGive(_rxMutex);
    return true;
}

/**
 * @brief  4-arg overload accepted for compatibility with the working
 *         audio_reactive.cpp that passes a deadline + renderDelayUs.
 *         The extra parameters are accepted but not currently used —
 *         behaviour is identical to the 2-arg poll() above.
 */
inline bool poll(void * out, size_t outLen, uint32_t * /*outDeadline*/, uint32_t /*renderDelayUs*/) {
    return poll(out, outLen);
}

/**
 * @brief  Inspect and consume an incoming ESP-NOW packet.
 *
 * Call from the audioreactive usermod's `onEspNowMessage()` override:
 * @code
 *   bool onEspNowMessage(uint8_t* sender, uint8_t* data, uint8_t len) override {
 *       return espnowAudioSync::handleIncomingPacket(data, len);
 *   }
 * @endcode
 *
 * If the packet matches the audio-sync magic bytes it is dedup'd and
 * latched for later retrieval by poll(); the function returns true,
 * signalling WLED's core dispatcher to skip its own WiZmote /
 * state-sync handling for this packet. Otherwise returns false and the
 * packet is left for WLED to process normally (preserving WiZmote
 * remote support and ESP-NOW state-sync).
 *
 * @param data  Raw bytes received from ESP-NOW.
 * @param len   Length of `data` in bytes.
 * @return true if the packet was an audio-sync frame (consumed);
 *         false otherwise.
 */
inline bool handleIncomingPacket(const uint8_t * sender, const uint8_t * data, int len) {
    if (len != (int)sizeof(ENPacket)) return false;
    const ENPacket * ep = reinterpret_cast<const ENPacket *>(data);
    if (ep->magic != EN_MAGIC || ep->version != EN_VERSION) return false;
    if (_isSender) return true;  // ours but we ignore -- we sent it

    // MAC locking. First valid sender wins; everyone else on the channel
    // with matching magic+version gets dropped (= different WLED master
    // nearby). Auto-unlocks if no traffic from the locked sender for
    // _MAC_UNLOCK_MS so physically swapping the master still works.
    // Skipped entirely when _macLockEnabled is false (user setting).
    if (sender != nullptr && _macLockEnabled) {
        uint32_t now = millis();
        if (_hasLockedSender && (now - _lastLockedSenderMs > _MAC_UNLOCK_MS)) {
            _hasLockedSender = false;
        }
        if (_hasLockedSender) {
            if (memcmp(_lockedSenderMac, sender, 6) != 0) {
                _stat_foreignSender++;
                return true;  // consume so WLED's dispatcher doesn't see it
            }
        } else {
            memcpy(_lockedSenderMac, sender, 6);
            _hasLockedSender = true;
        }
        _lastLockedSenderMs = now;
    }

    if (_hasLastRxSeq) {
        int16_t diff = (int16_t)((int16_t)ep->seq - (int16_t)_lastRxSeq);
        if (diff <= 0) { _stat_dupes++; return true; }
        if (diff > 1)  { _stat_gaps += (diff - 1); }
    }

    // Mutex must exist before any callback can fire (begin() creates it
    // first), but we check defensively in case of unusual timing.
    if (_rxMutex == nullptr) return true;
    if (xSemaphoreTake(_rxMutex, pdMS_TO_TICKS(5)) != pdTRUE) return true;

    _lastRxSeq        = ep->seq;
    _hasLastRxSeq     = true;
    _stat_rx++;
    memcpy(_rxPayload, ep->payload, EN_PAYLOAD);
    _rxReady          = true;

    xSemaphoreGive(_rxMutex);
    return true;
}

/**
 * @brief  Backwards-compatible overload for callers that don't have the
 *         sender's MAC at hand. Skips the MAC-locking filter, behaving
 *         the same as before this feature was added.
 */
inline bool handleIncomingPacket(const uint8_t * data, int len) {
    return handleIncomingPacket(nullptr, data, len);
}

/**
 * @brief Diagnostic snapshot of transport-layer counters.
 */
struct Stats {
    uint32_t rx;             ///< Unique frames delivered to poll().
    uint32_t dupes;          ///< Duplicate copies dropped by dedup.
    uint32_t gaps;           ///< Sequence gaps observed (lost frames).
    uint32_t tx;             ///< Packets successfully queued for TX.
    uint32_t tx_err;         ///< TX attempts where the IDF queue refused.
    uint32_t foreignSender;  ///< Packets dropped because they came from a non-locked MAC.
};

/**
 * @brief  Snapshot the diagnostic counters.
 *
 * @return Stats struct containing the current counter values.
 */
inline Stats stats() {
    return Stats{_stat_rx, _stat_dupes, _stat_gaps, _stat_tx, _stat_tx_err, _stat_foreignSender};
}

/**
 * @brief  True iff the slave has locked onto a specific master's MAC and is
 *         filtering out any other senders on the same channel.
 */
inline bool hasLockedSender() { return _hasLockedSender; }

/**
 * @brief  Pointer to the 6-byte MAC of the locked sender, or nullptr if
 *         no lock has been established yet. The pointer remains valid for
 *         the lifetime of the firmware (it references a file-scope buffer);
 *         do NOT free it.
 */
inline const uint8_t * lockedSenderMac() { return _hasLockedSender ? _lockedSenderMac : nullptr; }

/**
 * @brief  Enable/disable the receive-side MAC lock at runtime. Called from
 *         the usermod's `connected()` after reading the user's setting.
 *         Disabling clears any existing lock so the next valid packet
 *         from any sender is accepted.
 */
inline void setMacLockEnabled(bool enabled) {
    _macLockEnabled = enabled;
    if (!enabled) {
        _hasLockedSender = false;  // clear stale lock so any sender is accepted
    }
}

/** @brief Read-back of the current MAC-lock master switch. */
inline bool isMacLockEnabled() { return _macLockEnabled; }

/**
 * @brief  Whether this transport is running in "hosted" mode (sharing
 *         WLED's ESP-NOW stack via the onEspNowMessage hook) vs
 *         "standalone" mode (owning the ESP-NOW radio outright).
 *
 *         Decided at begin() time based on whether WLED's ESP-NOW was
 *         already initialised. Read this in the Info page to surface
 *         which mode the board is currently using.
 */
inline bool isHosted() { return !_standaloneMode; }

// --- Standalone-mode receive callback ---
// Bridges ESP-NOW's C-style recv callback into our existing
// handleIncomingPacket(), which already does magic-byte filtering,
// sequence-number dedup, and mutex-protected payload latching. Defined
// here, after handleIncomingPacket, so the call below resolves cleanly.
//
// The ESP-IDF callback signature changed between IDF 4.x and IDF 5.x
// (5.x adds an esp_now_recv_info_t* in place of the raw MAC pointer).
// Both branches do the same thing: forward to handleIncomingPacket.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void _audioSyncStandaloneRecvCb(const esp_now_recv_info_t * info, const uint8_t * data, int len) {
    handleIncomingPacket(info ? info->src_addr : nullptr, data, len);
}
#else
static void _audioSyncStandaloneRecvCb(const uint8_t * mac, const uint8_t * data, int len) {
    handleIncomingPacket(mac, data, len);
}
#endif

/**
 * @brief  Currently-active Wi-Fi channel for ESP-NOW broadcast.
 *
 * Returns the *live* radio channel (queried from WiFi.channel()) rather
 * than the cached `_channel` value, because Wi-Fi may roam to a new
 * channel between begin() calls and we want the info page to reflect
 * reality. Returns 0 if Wi-Fi is not yet up.
 */
inline uint8_t channel() {
    uint8_t live = (uint8_t)WiFi.channel();
    if (live != 0) return live;
    return _channel;
}

} // namespace espnowAudioSync

#elif defined(ESP8266) && !defined(WLED_DISABLE_ESPNOW)
// -----------------------------------------------------------------------
// ESP8266 RX-only implementation. ESP8266 boards can act as audio sync
// SLAVES (Sync = Receive) but not masters -- the FFT processing is
// ESP32-only, so there's no point implementing TX on ESP8266.
//
// We piggy-back on WLED's existing ESP-NOW stack the same way the ESP32
// branch does: receives arrive via the usermod's onEspNowMessage()
// override, which forwards each packet to handleIncomingPacket() here.
//
// Differences from the ESP32 branch:
//   * No FreeRTOS mutex (ESP8266 doesn't have FreeRTOS). The shared
//     state is small and ESP-NOW callbacks on ESP8266 run from the
//     main loop context (WLED's QuickEspNow drains the queue cooperatively),
//     so volatile flags are sufficient.
//   * No send() / esp_now_send() -- we never transmit from ESP8266.
//   * No FFT task or audio sampling exists on this MCU anyway, so the
//     master is always an ESP32.
// -----------------------------------------------------------------------
#include <Arduino.h>

namespace espnowAudioSync {

// Same wire-format constants as the ESP32 branch -- they MUST match
// for cross-platform compatibility (ESP32 master <-> ESP8266 slave).
static constexpr uint16_t EN_MAGIC      = 0xA53E;
static constexpr uint8_t  EN_VERSION    = 1;
static constexpr size_t   EN_PAYLOAD    = 44;
static constexpr size_t   EN_PREAMBLE   = 6;
static constexpr size_t   EN_PACKET     = EN_PREAMBLE + EN_PAYLOAD;

struct __attribute__((packed)) ENPacket {
    uint16_t magic;
    uint8_t  version;
    uint8_t  reserved;
    uint16_t seq;
    uint8_t  payload[EN_PAYLOAD];
};
static_assert(sizeof(ENPacket) == EN_PACKET, "ENPacket layout drift");

// Receiver state (volatile since handleIncomingPacket() may be invoked
// from a different task context than poll()).
static volatile bool _rxReady = false;
static uint8_t       _rxPayload[EN_PAYLOAD];
static uint16_t      _lastRxSeq = 0;
static bool          _hasLastRxSeq = false;

// Diagnostic counters (volatile for the same reason).
static volatile uint32_t _stat_rx     = 0;
static volatile uint32_t _stat_dupes  = 0;
static volatile uint32_t _stat_gaps   = 0;

/**
 * @brief  Initialise the receiver. On ESP8266 this is a near-no-op:
 *         WLED's ESP-NOW stack is already up, and we just need to be
 *         ready to handle incoming packets via the onEspNowMessage()
 *         hook. Returns true so audio_reactive.cpp does not fall back
 *         to UDP (which would defeat the purpose).
 *
 * @param channel   Ignored on ESP8266 -- the radio is always on the
 *                  current Wi-Fi channel.
 * @param isSender  Ignored on ESP8266 -- always treated as receiver.
 */
inline bool begin(uint8_t /*channel*/, bool /*isSender*/) {
    return true;
}

/**
 * @brief  No-op on ESP8266: this MCU never transmits audio sync.
 *         The master is always an ESP32 because the FFT processing
 *         only runs on ESP32 family chips.
 */
inline bool send(const void * /*payload*/, size_t /*payloadLen*/) {
    return false;
}

/**
 * @brief  Same semantics as the ESP32 poll(). Returns the latched
 *         receive payload if a fresh frame has arrived since the last
 *         call; otherwise returns false.
 */
inline bool poll(void * out, size_t outLen) {
    if (!_rxReady) return false;
    if (outLen < EN_PAYLOAD) return false;

    // Brief interrupt-disable window to copy the latched payload
    // atomically. Cheaper than any heavier sync primitive on ESP8266.
    noInterrupts();
    memcpy(out, _rxPayload, EN_PAYLOAD);
    _rxReady = false;
    interrupts();
    return true;
}

/** @brief 4-arg overload for compatibility with audio_reactive.cpp. */
inline bool poll(void * out, size_t outLen, uint32_t * /*outDeadline*/, uint32_t /*renderDelayUs*/) {
    return poll(out, outLen);
}

/**
 * @brief  Inspect an incoming ESP-NOW packet. If it's one of ours
 *         (magic + version match), latch the 44-byte payload for the
 *         next poll() and return true so WLED skips its own dispatch
 *         on this packet. Otherwise return false and let WLED process
 *         it normally (WiZmote / state-sync support stays intact).
 */
inline bool handleIncomingPacket(const uint8_t * data, int len) {
    if (len != (int)sizeof(ENPacket)) return false;
    const ENPacket * ep = reinterpret_cast<const ENPacket *>(data);
    if (ep->magic != EN_MAGIC || ep->version != EN_VERSION) return false;

    // Dedup using the same sequence-number logic as the ESP32 branch.
    if (_hasLastRxSeq) {
        int16_t diff = (int16_t)((int16_t)ep->seq - (int16_t)_lastRxSeq);
        if (diff <= 0) { _stat_dupes++; return true; }
        if (diff > 1)  { _stat_gaps += (diff - 1); }
    }

    noInterrupts();
    _lastRxSeq    = ep->seq;
    _hasLastRxSeq = true;
    _stat_rx++;
    memcpy(_rxPayload, ep->payload, EN_PAYLOAD);
    _rxReady      = true;
    interrupts();

    return true;
}

/** @brief Sender-MAC-aware overload. On ESP8266 the MAC isn't useful in
 *         our setup (master is always an ESP32), so we just ignore the
 *         MAC and forward to the original handler. Provided so the
 *         calling code can be the same on both platforms. */
inline bool handleIncomingPacket(const uint8_t * /*sender*/, const uint8_t * data, int len) {
    return handleIncomingPacket(data, len);
}

/** @brief Diagnostic snapshot of receive counters (TX counters are 0). */
struct Stats {
    uint32_t rx;
    uint32_t dupes;
    uint32_t gaps;
    uint32_t tx;
    uint32_t tx_err;
    uint32_t foreignSender;  ///< Always 0 on ESP8266 (no MAC filtering).
};
inline Stats stats() {
    return Stats{_stat_rx, _stat_dupes, _stat_gaps, 0, 0, 0};
}

/** @brief Stub on ESP8266: MAC locking is not implemented here. */
inline bool hasLockedSender() { return false; }
inline const uint8_t * lockedSenderMac() { return nullptr; }
inline void setMacLockEnabled(bool /*enabled*/) { /* no-op */ }
inline bool isMacLockEnabled() { return false; }

/** @brief Always hosted on ESP8266 (we use WLED's existing ESP-NOW stack). */
inline bool isHosted() { return true; }

/** @brief Currently-active Wi-Fi channel (ESP8266 always uses the radio's
 *         current Wi-Fi channel for ESP-NOW). Returns 0 if Wi-Fi is down. */
inline uint8_t channel() { return (uint8_t)WiFi.channel(); }

} // namespace espnowAudioSync

#else // !(ARDUINO_ARCH_ESP32 || ESP8266) || WLED_DISABLE_ESPNOW
// -----------------------------------------------------------------------
// Compile-time stubs. Active on builds with -D WLED_DISABLE_ESPNOW or
// on MCUs other than ESP32 / ESP8266. Exposing the same API surface
// as no-ops lets audio_reactive.cpp compile cleanly without further
// #ifdef guards at every call site -- every call just returns false /
// zeroed Stats and the usermod silently falls back to the existing UDP
// transport.
// -----------------------------------------------------------------------
namespace espnowAudioSync {

/** @brief Stub Stats struct (always returns zeros). */
struct Stats { uint32_t rx, dupes, gaps, tx, tx_err, foreignSender; };

/** @brief Stub: always returns false. */
inline bool begin(uint8_t /*channel*/, bool /*isSender*/) { return false; }

/** @brief Stub: always returns false. */
inline bool send(const void * /*payload*/, size_t /*payloadLen*/) { return false; }

/** @brief Stub: always returns false. */
inline bool poll(void * /*out*/, size_t /*outLen*/) { return false; }

/** @brief Stub: always returns false. */
inline bool handleIncomingPacket(const uint8_t * /*data*/, int /*len*/) { return false; }

/** @brief Stub: sender-aware overload, also always returns false. */
inline bool handleIncomingPacket(const uint8_t * /*sender*/, const uint8_t * /*data*/, int /*len*/) { return false; }

/** @brief Stub: always returns zeroed counters. */
inline Stats stats() { return Stats{0, 0, 0, 0, 0, 0}; }

/** @brief Stubs: no MAC locking on disabled builds. */
inline bool hasLockedSender() { return false; }
inline const uint8_t * lockedSenderMac() { return nullptr; }
inline void setMacLockEnabled(bool /*enabled*/) { /* no-op */ }
inline bool isMacLockEnabled() { return false; }

/** @brief Stub: always returns false on non-ESP32 builds. */
inline bool isHosted() { return false; }

/** @brief Stub: returns 0 when ESP-NOW transport is not available. */
inline uint8_t channel() { return 0; }

} // namespace espnowAudioSync

#endif // ARDUINO_ARCH_ESP32 || ESP8266 (with WLED_DISABLE_ESPNOW guarded)
