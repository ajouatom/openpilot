# eGPU Web status guidance (2026-10-09)

The reported Cinque v3 card displayed `Needs attention` and a raw
`Last worker stage: load_model` suffix, but did not explain what to do.
The precompiled worker's startup response has a 110-second deadline.
`load_model` covers artifact loading and associated device initialization;
it does not identify a faulty cable or prove a particular software defect.

Read-only inspection of the reported device found a matching historical
Cinque v3 load timeout, with about 106.7 seconds in that stage. At inspection,
the device selected Mountain Dew v1, reported `UsbGpuActive=1`,
`UsbGpuLoading=0`, and no startup-failed flag. A separate prior MDM inference
timeout was also recorded. Current recovery is not evidence that the
underlying intermittent fault has been resolved. Private captures stay in
`.analysis/archive/2026-10-09-egpu-guidance/`.

The card now separates the brief failure explanation from the next action:

- Runtime failures say that the eGPU is unavailable and the internal model
  was selected. This does not claim that internal inference is healthy.
- After parking, the user is directed to the existing User / System > Reboot
  control once; recurrence calls for a screenshot for support. Korean labels
  use `기기 관리` and `재부팅` consistently.
- Raw worker stages and exceptions remain in the API and saved diagnostics,
  rather than being appended to the main explanation in English.
- Unknown error codes use a readable support message. Recovery instructions
  clear when the status changes to healthy, downloading, or Jetson-only.

No reboot is requested automatically. Existing restart eligibility, model
selection, rejection policy, timeouts, fallback and control logic are unchanged.
Reboot clears the runtime startup-failure latch and permits a fresh attempt;
it is an attempt, not a promised repair. Korean and English UI regression
tests cover the failed compiled-model case (whose compile button is hidden),
recovery, unknown errors and unchanged download/Jetson presentation.
