# Feedback-repair FPV link — architecture proposal

**Status: proposal. Nothing on this page has run end to end.** Every number
below is a component measurement the design rests on, given with its source and
its adversarial counterpart. The rollout ([§10](#10-rollout-each-step-gated-on-a-measurement))
is ordered so the cheapest measurement that could kill the idea runs first.

devourer is the mechanism, not the policy
([building blocks](adaptive-link-building-blocks.md)). The link layer described
here lives upstream in **mabur**, the drone / ground-station daemon pair that
uses devourer as a library (read at `f51f8e0`, see [References](#references)).
This page records the design, the measurements that shaped it, and what devourer
itself has to provide.

Terms: an **AU** (access unit) is one encoded video frame; mabur sends each AU
as one **burst** of aggregated radio frames. The **GS** is the ground station
(two receive cards); the **drone** has one card.

## The question

A community discussion proposed adding a software acknowledgement to the
one-way FEC video link used by OpenIPC-style digital FPV:

- the ground reports reception to the drone once per ~10 packets rather than
  per packet;
- the drone retransmits only the lost packets, and only when the loss exceeds
  what FEC can recover (recover at 7-of-12 without help, retransmit at 8-of-12);
- in exchange, static FEC overhead can come down, because retransmission covers
  the tail the FEC used to be sized for.

Claims raised alongside it: the retransmission round costs "well under a frame
at 120 fps", even "nanoseconds", because radio travels at light speed; an
unrecoverable group cannot be displayed before a round trip (the objection);
and probing the MCS rungs either side of the current one keeps the link off the
cliff, so a whole group is never lost in the first place.

**Short answer: feasible and worth building, but not in the proposed shape.**
On a half-duplex, carrier-sense-off link, a report every ~10 packets costs more
airtime than it saves. What fits is one cumulative report per frame, sent into a
listen window the drone reserves after every burst. The report says how many FEC
symbols are missing, not which packets. The drone answers with fresh repair
symbols rather than retransmitted originals.

## 1. What the measurements say about the claims

**"Retransmission takes nanoseconds."** Propagation is about 3.3 µs/km, so
0.17 ms one way at 50 km, which is negligible. The cost is the turnaround
through host, USB and chip queue at both ends. devourer's submit→air floor is
11–22 µs RMS, but its p99.9 is 0.76–3.2 ms on every transport
([scheduled-mac.md](scheduled-mac.md)). The counterpart: that tail is
ambient-dependent, and the same Jaguar1 cell has read anywhere from 0.8 to
3.2 ms across runs. mabur measures its control-path round trip at **7.2 ms**
with its uplink slotter and 7.8 ms without (bench, mabur
`docs/gs-uplink-self-blanking-findings-2026-09-02.md`). The counterpart: it
reads high under saturation, because the drone's reply queues behind video. So
one repair round is **about one frame at 120 fps and half a frame at 60**, not
"well under".

**"ACK every ~10 packets."** Both ends are half-duplex and run with carrier
sense off. Every GS transmission blinds *both* GS cards for ~180 µs, and a drone
aggregate whose preamble starts inside that window is lost whole; the drone
hears nothing during its own burst (mabur `docs/tx-rx-timing.md`). mabur
already struggles to land 10–20 uplink sends/s: its slotter needs ≥ 4 ms of
drone idle, and at ~76% duty **22–43% of sends time out** into random-phase
blasts. A 20 Mb/s stream at 1400-byte packets is ~1800 packets/s, so one report
per 10 packets would be ~180 more sends/s.

**"An unrecoverable group can't be shown before a round trip."** This is
correct. But static FEC is paid on *every* frame. mabur flies 1.0× overhead at
its mcs5 rung and 2.0× at rung 0, so air per video byte is 2.0–3.0× (mabur
`docs/airtime-model.md` §2). When base carried twice the enhancement layer's
overhead, that alone cost ~7–8 ms of p50 latency plus most of the jitter (mabur
`docs/latency-budget-findings-2026-08-31.md`). The counterpart: that protection
works. On the loss-sim rig, 5% injected loss leaves 0.03–0.10% residual. The
alternative to a late frame is a corrupted reference frame, not an on-time one.

**"MCS probing prevents whole-group loss."** It does for slow fades. mabur's
first fade flights recorded 26 demote episodes, all gradual range fades,
stepped down about every 410–440 ms, with zero mid-flight video-damage windows.
It does not for bursts, because the loop is two orders of magnitude slower than
a burst. Ten packets at mcs4 are ~3 ms of air, while in mabur today:

- a demote was observed applied ~35 ms after the GS decided (one observation,
  mabur `docs/switch-loss-findings-2026-09-05.md`);
- ~7% of feedback frames are lost, each costing a 50–100 ms feedback period;
- the predictive fade trigger needs 0.3–1.3 s, and never fires on ramps slower
  than ~0.45 dB/s;
- "a genuine FAST fade (obstruction, multipath null) has still never been
  recorded against this trigger" (mabur `docs/link-adaptation.md`).

Probing covers promotions only. mabur probes rung +1 as a promote gate, and a
probe-driven demote is explicitly out of scope. Probes also do not predict the
next second: in flight 20 the +1 probe passed 14 promotes to rung 5, and all
14 were demoted, 8 within 1 s (mabur
`docs/probe-stream-flight-findings-2026-09-05.md`). Losses already arrive a
whole aggregate at a time (4–6 packets), from GS self-blanking and from
interferers. A lower MCS does not help against those: a jammed slice loses
frames before the FCS stage ([pseudo-preamble-puncturing.md](pseudo-preamble-puncturing.md)),
and a lower MCS stretches airtime and so the collision exposure. A rung-5
enhancement AU survives one lost aggregate but not two, so "all 10 packets
lost" is two aggregates, close to the normal failure unit.

**"Hardware ACK is what makes AP-mode FPV slow on lossy links."** Bounded
hardware ARQ is not inherently slow. devourer's retry-limit sweep delivered
99.72% at limit 3, 99.97% at 8 and 100% at 16, with queue-time p99 flat across
limits ([scheduled-mac.md](scheduled-mac.md), `tests/arq_retry_sweep.sh`). The
counterpart: that is a near-field collision regime, not a range fade. The
penalty an AP-mode stack shows is more plausibly its kernel retry, backoff and
rate-control policy; that is not measured here.

**"We can push 3000–4000 packets/s over USB."** On a PC, yes; devourer benches
2.4k fps. On a camera SoC, packets/s is CPU-bound: one bulk submission costs
~248 µs of CPU on a CV610 craft against ~22 µs on x86, falling to ~148 µs with
3:1 USB TX aggregation ([aggregation.md](aggregation.md); one RTL8733B unit).
Chopping video into small packets is not free.

## 2. Constraints that shape the design

- **Half-duplex at both ends, carrier sense off at both** (mabur since
  2026-08-05). The only hardware arbitration left is that a chip will not start
  a TX in the middle of an RX PPDU.
- **Every uplink send is a blast** that blinds both GS cards for ~180 µs.
- **The drone is deaf for its whole burst.** Without slotting, uplink delivery
  tracks 1 − air% (51–69% measured; ~93% with mabur's slotter).
- **Turnaround is milliseconds; propagation is microseconds.** Timeouts must be
  derived from measured round trip, and nothing may depend on SIFS-scale
  timing if range is to be unbounded (§7).
- **The loss unit is an aggregate**, not a packet.
- **The GS has two cards.** Anything that decides "lost" must decide after
  merging both.
- **Per-packet CPU on the camera SoC is expensive** (§1).

## 3. Architecture: one mechanism per timescale

| Layer | Timescale | Job | Status in mabur |
|---|---|---|---|
| FEC floor | 0 round trips | absorb the loss that is always there | exists; overhead comes down |
| **Repair loop** | 1 round trip (~5–8 ms) | fix a frame the floor could not | **new** |
| Rate / power ladder | 100 ms – seconds | follow mean SNR so the repair loop stays rare | exists; gains inputs (§6) |
| Source adaptation | frames – seconds | bitrate vs airtime budget, enhancement shed, IDR / intra-refresh | exists |
| Channel | seconds – minutes | escape interference | exists (mabur channel select; devourer [channel migration](adaptive-channel-migration.md)) |

Each layer covers what the layer above it cannot. The ladder keeps repairs
rare; repairs let the FEC floor be thin; a thin floor returns airtime as bitrate
or latency. The structural change underneath is making the drone's idle
**scheduled** rather than predicted (§4.2).

## 4. Components

### 4.1 Downlink: broadcast plus a thin FEC floor

Keep what mabur has: SVC-T base and enhancement layers, per-layer systematic
sliding-window RLC FEC (symbol 332 B, window 32, 4 FEC blocks per body),
sub-block salvage of FCS-failed frames ([fused-fec.md](fused-fec.md)), and
A-MPDU broadcast with QoS No-Ack and retry 0. Video does not use hardware ACK
(§7).

**Size the floor to one lost aggregate per frame, not to fades.** Measure it
from the residual gap distribution the way `tests/arq_fec_dimension.py` does
for the hardware-ARQ case, on real flights, per rung. Illustration only: at
mcs5 today air per byte is 2.0×; a 0.25 floor makes it ~1.25×, which is 37%
shorter bursts or up to 60% more video at the same airtime. That holds only if
repair demand stays low, which is what rollout step 1 measures.

### 4.2 A scheduled listen window after every burst

```
drone  [ AU burst .................. ][probe][ LISTEN W ][repairs?][ next AU burst ...
GS                                       └─ deficit known → [status frame]
```

- **The drone reserves W after each burst and its trailing probe**, and the
  bitrate policy budgets `burst + probe + W ≤ AU period`. Today the budget is
  an open-loop 60% that measures 66–76% (mabur `docs/tx-rx-timing.md` §5 gap 2).
- **Anchor W to the chip TSF.** devourer exposes `ReadTsf()` and a MAC-latched
  `tsfl` on every received frame ([time-distribution.md](time-distribution.md)).
  The drone publishes its burst phase in its own TSF, and the GS maps it
  through the `tsfl` of received frames. That replaces mabur's first-body
  cadence predictor (its gap #5); the predictor is already within ~1 ms, so
  the gain is determinism, not accuracy.
- **Size W** as GS send latency (1–1.5 ms measured) + status airtime (~0.2 ms at
  MCS0) + 2 × one-way propagation (0.33 ms at 50 km) + margin, so ~1.5–2 ms,
  scaled with range. The cost at 60 AU/s is ~12% of airtime for a 2 ms W; the
  FEC-floor reduction is what pays for it.
- **Inside W an uplink blast collides with nothing:** the drone is silent by
  construction, so blinding both GS cards costs no video.

### 4.3 The status frame: the software ACK

One GS → drone frame per AU period, inside W, ~20–40 bytes:

| Field | Meaning |
|---|---|
| `status_seq` | this frame's sequence number |
| `burst_echo` | the drone burst this report closes |
| per layer: `complete_through` | newest AU fully decoded |
| per layer: up to N `(a, b, k)` | open shortfall: symbol range `[a, b]`, short by `k`; repeated until filled |
| existing RCF fields | rung / power / probe profile, as today |

Its properties are the design:

- **Cumulative and idempotent.** It carries state, not events, so a lost status
  frame costs one AU period (16.7 ms at 60 AU/s). Today a lost op-changing RCF
  costs a whole `feedback_ms` of 50–100 ms.
- **A count, not a bitmap.** Sub-block salvage means an FCS-failed frame may
  still have delivered most of its symbols, so a per-packet bitmap overstates
  the loss. The decoder's own count of missing symbols (net of repair rows it
  already holds) is exact. With an RLC code the drone needs only "k more", not
  which ones.
- **Computed after merging both GS cards**, which a hardware ACK cannot do.
- **Generated after FEC decode**, so it confirms delivery to the application.
  A hardware ACK confirms chip-FIFO admission only; see the ACKed-but-undelivered
  field report behind `tests/arq_e2e_delivery.sh`.
- **A heartbeat.** Its absence is how the drone learns the uplink is down,
  feeding mabur's existing `failsafe_ms` / rendezvous path.

**Why not reuse `RxReceipt` directly.** `src/cell/RxReceipt.h` is a working
software ACK. It uses overlapping bitmap receipts with an idempotent merge, and
was measured frame-exact against the receiver's ledger: 126,594 frames clean,
and 349,455 under 150 ms consumer stalls at 2.4k fps. It is sized as an
accounting tier, though: the 8192-bit default is ~1 KB, ~1.4 ms at 6M, too
heavy per AU, and it counts frames, not symbols. Reuse its patterns instead:
overlap for loss tolerance, magic / version / TA / exact-length validation,
and a hard cap on the indices an over-the-air report may claim. Keep
`RxReceipt` as the delivery truth in benches.

### 4.4 The repair loop: coded ARQ

**GS side.**

- At burst end, read the deficit from the FEC decoder. The probe body already
  marks the end: it lands 0.9 ms p50 / 4 ms p99 after the AU completion stamp
  on the bench, and 1.6–1.8 / 5.3–5.6 ms in flight.
- Put the deficit in the next status frame.
- Hold an incomplete AU for at least one round trip (gap timeout, repair-row
  expiry) instead of abandoning it.

**Drone side.**

- Retain sealed symbols for at least one round trip plus one burst. The current
  encoder window of 32 symbols covers only about a third of a 30 KB AU.
- On a shortfall, emit **k + 1 fresh repair symbols over `[a, b]`**. This needs
  a new envelope type, because the wire window is a u8 (≤ 255).
- Queue repairs ahead of the next AU's bodies.
- Fly repairs one rung lower than the video, and optionally at higher
  per-packet power; Jaguar3 has per-packet TX power (`src/AdapterCaps.h`).

**Why repairs and not retransmitted originals.** Any RLC repair symbol can
substitute for any missing source symbol in its window. So the report never
names packets, a lost repair is just a smaller count in the next report, and
loss the FEC already covered generates nothing. The "retransmit only when loss
exceeds FEC capacity" rule falls out of the decoder's rank for free.

**Timers and budgets.**

- The timeout comes from measured round trip (mabur's `RttEstimator`: minimum
  plus a jitter margin), never a constant.
- The timeout is capped by the AU's display deadline.
- At most two rounds per AU.
- A per-second cap on repair airtime.

**Deadline policy.**

- Drop enhancement-layer repairs once the player has moved past that AU.
- Repair base-layer AUs even when late: they are references, and a late repair
  keeps the chain decodable.
- A base AU that still fails falls back to intra-refresh or an IDR request,
  which already exist.

### 4.5 Uplink confirmation: an echo in the burst header

The drone echoes the last `status_seq`, and the rung it applied, in the header
of its next burst. That costs no extra transmission, and the GS knows within
one period whether its frame landed. mabur already carries `rcf_seq_echo`, but
in the 1 Hz `T_TELEM`; moving it into the burst header makes it per-period.
Uplink reliability then comes from **frequency × cumulative content**, not from
retries.

## 5. Latency of one repair round

| Step | Estimate | Source |
|---|---|---|
| burst end → deficit known | ~1 ms p50, 4–6 ms p99 | probe arrival, bench / flight |
| GS host → air | 1–1.5 ms | mabur `docs/tx-rx-timing.md` §3.2 |
| status frame on air | ~0.2 ms | RCF-sized frame at MCS0 |
| propagation, both ways | 6.7 µs per km of range | physics |
| drone USB RX → decision | **unmeasured** | none |
| repair submit → air | tens of µs at queue head; behind the chip FIFO (up to ~65–70 KB) otherwise | [scheduled-mac.md](scheduled-mac.md); mabur latency budget |
| k repair bodies on air | ~0.2–0.3 ms each at mcs4 | mabur airtime model |
| **anchor** | control-path RTT 7.2–7.8 ms | mabur bench (reads high under load) |

Only AUs with a deficit pay a round; every AU saves the FEC air the floor no
longer spends.

## 6. Coupling to the ladder

- **Keep scoring pre-repair loss.** mabur already books it at arrival time.
  Repairs must never hide a rung that only survives on retransmission.
- **Add repair demand per second at the current rung as a demote input.** It is
  measured every AU at the rate being flown, which makes it an earlier warning
  than a probe of a neighbouring rate.
- **Leave the rung +1 probe unchanged as the promote gate.** A rung −1 probe
  earns its airtime only for interference-shaped loss, where a lower rate can
  do worse.
- **W and the repair cap belong to the airtime budget** the bitrate policy
  divides.

## 7. Why not hardware ACK

| | Hardware ACK | Software ACK (this design) |
|---|---|---|
| Range | ~30 km ceiling (255 µs window); ~12 km at the 128 µs default | no hard limit; round trip grows 6.7 µs/km |
| Time to retry | µs | ms (~5–8 ms round) |
| What it confirms | chip-FIFO admission | application delivery, after FEC and salvage |
| Two GS cards | one responder card | merged before deciding |
| FEC-aware | no; retries what FEC would recover | yes; asks only for the shortfall |
| Lost acknowledgement | spurious retry | healed by the next cumulative report |

**The range ceiling.** The ACK window (`DEVOURER_ACK_TIMEOUT_US`,
`src/DeviceConfig.h`) is an 8-bit µs register, clamped to 1–255. Sizing is
~6.7 µs per km of range plus ~50 µs of ACK flight and detection margin, so the
ceiling is ~30 km at 255 µs and ~12 km at the 128 µs default. A wider window
is not free: every retry of a genuinely lost frame waits the full window
(measured with a dead RA at retry 8 and max duty: 2719 / 2015 / 1507 write-offs
per 8 s at 33 / 128 / 255 µs).

**Past the ceiling nothing fails loudly.** The frame still arrives, but the
ACK lands after the window closes. The sender then believes every frame failed
and retries to the limit:

- the receiver gets duplicates;
- each retry blinds both GS cards again, and two or three retries at full
  windows can overrun W;
- `tx.report` marks delivered frames as failed, so the telemetry is wrong too.

**It cannot be switched off mid-flight today.** Jaguar3 writes the session's
`retry_limit` into every frame's descriptor (`src/jaguar3/RtlJaguar3Device.cpp`).
The radiotap `DATA_RETRIES` field is parsed but not honoured on TX.

**On the video downlink specifically,** hardware ARQ is wrong for three more
reasons:

- It is blind to FEC.
- Retries walk the rate down the firmware ladder (Jaguar3 at 5 GHz:
  MCS3 ×4 → MCS2 → 6M ×4, [scheduled-mac.md](scheduled-mac.md)), which makes
  burst length unpredictable and breaks both the airtime budget and W.
- ACKed A-MPDU measured −8% delivered at MCS3 against the un-aggregated feed
  ([aggregation.md](aggregation.md)).

**Allowed only as an optional short-range speed-up for the status frame.** Use
it only if shadow data shows uplink loss still costs video once §4.3–§4.5 are
in place. It would need devourer to honour radiotap `DATA_RETRIES` per packet;
the field is already per-descriptor. The GS then disables it automatically when
`tx.report` retries pin at the limit while burst-header echoes prove delivery,
because that combination is the signature of ACKs landing outside the window.
GPS range from telemetry is the backup trigger. The Jaguar3 hardware works in
both roles (8812EU responder 98% single-shot; 8822CU soliciting 1.00 delivered
at 0.24 mean retries), but this has not been measured inside mabur's timing.

## 8. What software ACK does not fix

- **Airtime.** Every report is a transmission on a half-duplex link. It is
  cheap only inside W; without W it is back to predicted gaps and 22–43%
  timeouts.
- **Long outages.** A report sent into a deep fade is lost like everything
  else. Heartbeat silence, the ladder, the failsafe and IDR handle those.
- **Drone CPU.** Each received status frame is USB and parse work on the camera
  SoC. One per AU is negligible; one per packet would not be.

## 9. Where the work lives

**devourer:**

- **Per-packet TX queue selection,** so repairs and control frames jump the
  chip's video backlog. Today `DEVOURER_TX_QSEL` is a global experimental
  override (default 0x12, the MGMT queue). Whether a different queue actually
  overtakes queued MGMT frames on the 8822E is unmeasured.
- **A turnaround bench:** status on air → repair on air, timed at a passive
  witness from `tsfl`, with the drone's queue loaded. The TD v2 tag and
  `tests/txegress_analyze.py` fitting already exist.
- **Optional:** honour radiotap `DATA_RETRIES` per packet (§7).

**mabur:**

- `common/`: the status-frame wire format; a "k repairs over `[a, b]`" envelope
  on `SwEncoder`.
- `gs/`: the deficit tracker, a W-anchored status sender replacing
  `RcfSlotter`, AU hold ≥ one round trip, and the ladder's repair-demand input.
- `drone/`: the retained-symbol ring, a front-of-queue lane in `TxQueue`, W in
  the bitrate policy, the burst-phase publication, and the burst-header echo.

**Not needed:**

- **waybeam.** mabur folded waybeam's encoder into `maburd` on 2026-08-29. The
  standalone waybeam + waybeam-link path, via waybeam's `frame-shm://` output,
  is an alternative host for the same design.
- **wfb-ng.** It is the no-feedback baseline, useful as an A/B control.

## 10. Rollout, each step gated on a measurement

1. **Shadow mode, GS only, no new transmissions.** Log the per-AU symbol
   deficit on real flights and bucket it: covered, short by ≤ one aggregate,
   short by two, more. **Kill criterion:** if shortfalls are mostly long
   outages rather than one-to-two-aggregate bursts, the ladder and IDR already
   do the useful work; stop.
2. **devourer turnaround bench with a loaded queue.** **Kill criterion (for a
   120 fps target):** status-to-repair p99 well above ~8 ms.
3. **W and a per-period status frame,** with FEC unchanged. A/B uplink
   delivery and video loss against `RcfSlotter`.
4. **The repair loop on top of today's FEC.** Measure residual loss and
   latency with mabur's gates (`ausniff.py`, the `lat` segments).
5. **Lower the FEC floor in steps** (1.0 → 0.5 → 0.25 at mcs5), each A/B'd on
   the same gates plus `aucadence.py` and air%. Stop where residual or repair
   demand climbs.
6. **Ladder coupling** (§6).

## 11. Risks and open questions

- **Idle is the scarce resource.** W costs ~12% of airtime at 60 AU/s and a
  larger share at 120 fps (8.3 ms period). Repairs and status share it.
- **The round trip under saturation** will exceed the 7 ms bench anchor, and the
  drone's USB RX leg is unmeasured.
- **Chip-FIFO depth.** Without a queue that genuinely overtakes, a repair waits
  behind up to ~65–70 KB of queued video.
- **Fast fades are unrecorded** in mabur's data, so the burst-versus-outage
  split step 1 measures is genuinely unknown.
- **TSF mapping.** Two free-running TSFs; the mapping carries a one-way
  propagation offset (~167 µs at 50 km). That is fine at millisecond W
  resolution, or correct it with half the round trip.

## References

- mabur (read at `f51f8e0`): <https://github.com/notsudogood/mabur> —
  `docs/tx-rx-timing.md` (half-duplex timing, slotter, self-blanking),
  `docs/link-adaptation.md` (ladder, probe stream, fade trigger),
  `docs/airtime-model.md` (overhead policy),
  `docs/latency-budget-findings-2026-08-31.md`,
  `docs/probe-stream-flight-findings-2026-09-05.md`,
  `docs/switch-loss-findings-2026-09-05.md`,
  `docs/gs-uplink-self-blanking-findings-2026-09-02.md`,
  `common/include/mabur/sw_encoder.h` / `sw_decoder.h`, `gs/src/rtt_estimator.h`.
- waybeam (read at `0c880d8`): <https://github.com/notsudogood/waybeam> —
  `frame-shm://` output, resilience presets.
- devourer: [scheduled-mac.md](scheduled-mac.md) (submit→air guard time, ACK /
  TxReport matrix, retry sweep, RX receipts),
  [aggregation.md](aggregation.md) (A-MPDU, BlockAck responder, USB TX
  aggregation), [fused-fec.md](fused-fec.md),
  [adaptive-link.md](adaptive-link.md),
  [time-distribution.md](time-distribution.md), `src/cell/RxReceipt.h`,
  `src/DeviceConfig.h` (`ack_timeout_us`, `retry_limit`, `tx_qsel`),
  `tests/arq_e2e_delivery.sh`, `tests/arq_fec_dimension.py`.
