# Signal observation and display timing after the USB overlay drive

The user requested collection of the fresh drive and an explanation of visual
recognition, temporal memory and flickering. Before leaving again, the user then
requested vehicle installation. The bounded follow-up changes display semantics
only; model weights, control settings and display retention policy stay unchanged.
The original internal ONNX remains selected; the helper is comparison-only and
`assist_enabled` remains OFF.

## Collection and source identity

The selected closed driving segments are 1096--37 through 47 and 1097--0 through
10, 22 segments in total. The vehicle HEAD at collection is 335c1c2ffc; these
recordings report the running implementation 64151b60ff in initData. The intervening
commit changes documentation only. Original internal ONNX SHA-256 is
`f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5`.

All 110 selected qlog/rlog/qcamera/road/wide files (2,036,831,563 bytes) were
preserved using hard links outside normal log rotation on the same vehicle disk.
The links were checked for identical device/inode, stable size/mtime and SHA-256.
This prevents normal route-file unlinking from discarding these originals; it
is not an independent disk backup. Local acquisition and checksums are recorded
in the private archive manifest. No captures, vehicle settings or raw logs are
published to Git.

Local collection completed with matching SHA-256 for 108 files (64 qlogs plus
22 rlogs and 22 qcamera videos), 423,929,674 bytes, and the 37,514,604-byte
full-resolution road video for 1096--37. Across 22 decoded segments there are
5,367 trial observations at 3.23--4.93 Hz by segment. Result age is 178 ms median
and 247 ms at the 95th percentile; 2,345/5,345 within-segment observation gaps
exceed 250 ms. The 26,062 recorded modelV2 messages are valid, with reported
frameDropPerc zero. This does not establish helper accuracy or control safety.

## Confirmed mechanisms

The road camera and the original driving model operate around 20 Hz. The helper
runs two observers sequentially, at low priority on cores 0--3, with an explicit
50% single-CPU duty target. The second observer supplies the USB overlay. Its
actual observation rate must be measured, rather than inferred from the 20 Hz
maximum in its startup message.

There are three independent reasons for an intermittent or unknown box:

1. `SignalTracker` resets color confirmation across observations more than
   250 ms apart. Confirmation requires three observations. Identity can remain
   for up to 500 ms with image support after confirmed red, while color
   confirmation repeatedly resets. Losing color does not always mean losing
   the physical-object track.
2. `SignalDisplayReader` shows only currently detected tracks, and hides an
   observation older than 350 ms or more than 250 ms/five frames away from the
   displayed camera frame. Processing latency uses part of that budget before
   the result can first be drawn. Stable raw color can therefore coexist with
   a disappearing overlay between publications.
3. The displayed state inherits the helper's red-before-green requirement.
   A signal first encountered green can have raw green evidence but remain
   `unknown`, even after repeated observations. `G` is not currently a pure
   green-color perception label. The display's additional raw/state agreement
   requirement also downgrades contradictory or missing evidence.

The initial diagnostic display thus mixed recognition and departure-confirmation
semantics. The departure-time follow-up displays current raw color as R?/G? when
the observer has not confirmed it, retaining R/G for matching confirmed states
and ? for unknown evidence. The caption explicitly says signal candidates.
Missing current evidence never becomes remembered green. Freshness, frame skew,
lost-track rejection and control transport remain unchanged. This clarifies
what the detector saw; it does not fix the remaining timing-related flicker.
Renderer status adds display_revision=2 for runtime activation verification.

Installed implementation c4702e4238 on the parked vehicle by guarded fast-forward.
The initial SIGTERM-based restart did not produce revision 2 within its timeout.
Temporarily setting ClusterHud to zero and restoring its saved value 1 restarted
the display successfully. Fresh revision-2 status and progressing road frames
were then observed, with valid CAN/model state and unchanged modeld/camerad/card/
plannerd/selfdrived PIDs. No whole-device reboot occurred. This confirms activation,
not a new road-accuracy result. 68 focused display/camera/performance tests pass
with one existing Linux-only skip; desktop tests substitute Windows Params.

## A concrete continuity failure

In 1096--37, trial track 1267 covers a forward signal from encoded frame 467 to
937. It has 83 current-image observations: 69 raw red, 13 raw green and one
unknown. Only one observation has confirmed red and none has confirmed green.
The same ID and its red history survive, but color confirmation repeatedly
restarts. At green frames 864, 870, 875, 881, 887, 893, 899, 905 and 910 the
confirmation count remains one. It reaches two at frame 931, then resets to one
at 937. This directly establishes a timing/confirmation failure in this track;
it does not require assuming that the camera repeatedly saw a different light.
Full-resolution road-video crops show red at frames 841/858 and green lamps at
864/870/910. Lamp brightness varies in later crops; these images alone do not
identify LED pulse frequency or exposure as the cause. Road EncodeIdx supplies
the encoded-frame mapping; these are offline diagnostic crops, not USB captures.

Across that segment, 150 of 228 inter-observation gaps exceed 250 ms. Of those,
62 are between 250 and 255 ms; 88 exceed 255 ms. A tiny tolerance alone cannot
resolve all the gaps. Scene review also shows candidate boxes on signs/background
features. More persistence would preserve false candidates too, so extending
timeouts alone is not a demonstrated recognition fix.

The private scripts estimate display eligibility from recorded observation
arrival/EOF and encoded camera timestamps, using a stated 50 ms camera
presentation delay and ten phases of a 10 Hz display. These estimates are not
measurements of the physical panel: per-render overlay status was not retained
in the route logs. Do not report their percentages as actual user-visible blink
rates. LED/exposure aliasing remains possible in general, but is not established
as the cause of this drive's overlay flicker.
Pulsed LEDs and short camera exposures can produce apparent flicker in general;
see the image-sensor manufacturer's
[technical explanation](https://www.onsemi.com/company/newsroom/blog/automotive/autonomous-driving-imaging-in-lfm-and-hdr-situations-on-the-road).

## Recognition and temporal memory

The installed helper is hand-written OpenCV proposal, color and tracking logic,
not a retrained neural detector. The original ONNX is a learned driving policy:
camera features and temporal context contribute to future position/speed outputs.
The downstream x/v-derived traffic state is not an exposed traffic-light color
classification. Comma's published temporal-policy architecture is useful general
context, not proof that every published architecture matches this internal
checkpoint: <https://blog.comma.ai/0103release/>.

Human object recognition uses both object appearance and scene context; temporal
context can assist recognition, as tested in
[Zhang, Tseng and Kreiman](https://arxiv.org/abs/1911.07349). This supports the
engineering idea of maintaining a tracked-object belief, without claiming an
exact model of the brain. The assistant can inspect crops and multiple recorded
frames with a learned vision model; that model is not the installed helper and
is not an infallible labeling oracle. OpenAI documents visual/spatial limitations
in its [vision guide](https://developers.openai.com/api/docs/guides/images-vision).

A bounded remembered state is feasible for visualization if identity remains
supported. It should visibly indicate that the color is remembered, expire on
loss of identity or contradiction, and not silently become a new departure
permission. A small dedicated neural detector is also a testable option; this
drive does not establish that the device lacks the capacity to run one.

The next useful comparison is improved candidate rejection/object tracking,
observation cadence aligned with confirmation, and separate display semantics,
evaluated on these unchanged recordings. It is not merely a request for more
examples or permission to lengthen every timeout.
