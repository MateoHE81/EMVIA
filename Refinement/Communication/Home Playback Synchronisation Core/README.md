Home Playback Synchronisation Core/README.md

Responsibilities

Cloud Relay Emotion Stream
        ↓
UTC-ordered temporal buffer
        ↓
AVPlayer live-content date
        ↓
Playback-time alignment
        ↓
CES + Spike onset decision
        ↓
Emotion persistence tracking
        ↓
START_EVENT / UPDATE_EVENT / STOP_EVENT


The module does not calculate motor PWM, LED PWM or PTC output. Those
belong to the later Home-side ESP32 feedback engine.

Files

- `CloudEmotionEvent.swift`
  - Matches the JavaScript Cloud Relay JSON contract.
  - Validates protocol version, match ID and normalized sensor values.

- `CloudEmotionStreamReceiver.swift`
  - Connects with `URLSessionWebSocketTask`.
  - Subscribes to a `matchID` and receives timestamped Emotion events.
  - Supports Ping, reconnect and cached-message resume.

- `PlaybackTimelineClock.swift`
  - Defines the playback-clock abstraction.
  - `AVPlayerPlaybackTimeline` uses `AVPlayerItem.currentDate()` when the HLS
    stream supplies `EXT-X-PROGRAM-DATE-TIME`.
  - Supports a manual media-time to UTC anchor when that mapping is absent.
  - Detects playback pause and time jumps.

- `EmotionTimelineBuffer.swift`
  - Orders events by `eventUtcMs` and removes duplicates.

- `MatchContext.swift`
  - Supplies match minute and final/stoppage context.
  - Applies the selected maximum event durations:
    - `<15 min`: 27.2 s
    - `<45 min`: 30.0 s
    - `<75 min`: 30.5 s
    - `<90 min`: 30.5 s
    - `90+ min`: 45.2 s
    - critical final stoppage moment: 67.5 s

- `FeedbackDecisionEngine.swift`
  - Trigger: high CES + high Spike.
  - Sustain: CES remains above the sustain threshold.
  - Stop: CES remains below sustain, duration limit is reached, or input times
    out.
  - Cooldown requires CES to return below a lower reset threshold.
  - Spike is used only for onset, not for the sustain phase.

- `FeedbackCommand.swift`
  - Defines high-level `START_EVENT`, `UPDATE_EVENT` and `STOP_EVENT` commands.
  - Contains no direct hardware values.

- `HomePlaybackSyncManager.swift`
  - Connects all modules.
  - Aligns Emotion events to the live video timeline before decision-making.
  - Normal pause retains buffered events.
  - A deliberate backward seek suppresses live feedback until the viewer
    returns to the live edge.

Integration example

swift
import AVFoundation
import HomePlaybackSynchronisationCore

let player = AVPlayer(url: liveStreamURL)

let receiver = CloudEmotionStreamReceiver(
    endpoint: URL(string: "wss://example.com/ws")!,
    matchID: "demo-match"
)

let playbackTimeline = AVPlayerPlaybackTimeline(player: player)
let matchContext = MutableMatchContextProvider()

let commandSink = ClosureFeedbackCommandSink { command in
    // The next module will encode and send this command to Home-side ESP32.
    print(command.kind, command.ces, command.peakCES)
}

let synchroniser = HomePlaybackSyncManager(
    receiver: receiver,
    playbackTimeline: playbackTimeline,
    matchContextProvider: matchContext,
    commandSink: commandSink
)

synchroniser.start()

Important implementation boundary

The numerical thresholds in `FeedbackDecisionConfiguration.prototypeDefault`
are prototype calibration values. The fixed interaction logic is:


IDLE
→ high CES + high Spike
→ START_EVENT
→ CES remains elevated
→ UPDATE_EVENT
→ CES returns to baseline or maximum duration is reached
→ STOP_EVENT
→ cooldown


The Home-side ESP32 remains responsible for:

- `IDLE → ONSET → SUSTAIN → DECAY → COOLDOWN`
- vibration amplitude and low-rate envelope
- LED peak-brightness hold and fade-out
- closed-loop PTC temperature safety
- communication-loss shutdown
