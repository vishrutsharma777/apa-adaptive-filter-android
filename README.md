# APA Adaptive Filter on Android

A real-time **Affine Projection Algorithm (APA)** adaptive filter, built in Simulink entirely from
primitive matrix blocks — no built-in adaptive-filter block — and targeted at an Android device
using live microphone audio as the excitation signal.

![MATLAB](https://img.shields.io/badge/MATLAB-R2025b-orange)
![Simulink](https://img.shields.io/badge/Simulink-model-blue)
![Toolbox](https://img.shields.io/badge/DSP%20System%20Toolbox-required-lightgrey)
![Target](https://img.shields.io/badge/target-Android%20device-green)

---

## Overview

The model solves an **online system-identification** problem. An unknown 6-tap FIR "plant" sits
between the excitation and the desired signal, and the adaptive filter has to estimate that plant's
impulse response from the two signals alone:

- **Excitation** `x[n]` — captured live from the Android device microphone.
- **Desired signal** `d[n]` — the excitation passed through one of *two* unknown 6-tap FIR plants.
- **Plant switching** — a button in the on-device UI selects which plant is active *while the model
  is running*, so the filter must detect the change and re-identify the new system on the fly.
- **Monitoring** — the a-priori error is tracked as a sliding-window RMS value.

What makes this implementation different from calling a library block is that every stage of the
algorithm is exposed as explicit signal routing: the data matrix is physically assembled from
tapped delays and a buffer, the regularized Gram matrix is formed with matrix multiplies, and its
inverse is computed by **direct LU decomposition** rather than a closed-form update rule.

## Repository contents

| File | Description |
| :--- | :--- |
| `Assignment_2.slx` | The Simulink model — top-level harness plus the `Affine Projection Algorithm` subsystem |
| `ASPA2.pdf` | Write-up covering system architecture, signal flow, and parameterization |

## Requirements

| Component | Notes |
| :--- | :--- |
| MATLAB / Simulink **R2025b** | The `.slx` is saved in R2025b. Simulink cannot open a model in an *older* release — see [Opening in an earlier release](#opening-in-an-earlier-release) |
| DSP System Toolbox | `Buffer`, `LU Inverse`, `Moving Average` |
| Simulink Support Package for Android Devices | `Audio Capture` and the UI `Button`. Without it the model will not compile — the two blocks fail to resolve their libraries |
| An Android device | Microphone for excitation, touchscreen for the plant-switch button |

## The algorithm

At each step the filter assembles a data matrix from the $K$ most recent length-$L$ input vectors:

$$\mathbf{X}(n) = \big[\, \mathbf{x}(n) \;\; \mathbf{x}(n-1) \;\; \cdots \;\; \mathbf{x}(n-K+1) \,\big]$$

The a-priori output and error vectors follow, and the weights are updated along the projection of
the error onto the subspace spanned by those input vectors:

$$\mathbf{y}(n) = \mathbf{X}^{T}(n)\,\mathbf{w}(n)$$

$$\mathbf{e}(n) = \mathbf{d}(n) - \mathbf{y}(n)$$

$$\mathbf{w}(n+1) = \mathbf{w}(n) + \mu\,\mathbf{X}(n)\big[\mathbf{X}^{T}(n)\mathbf{X}(n) + \delta\mathbf{I}\big]^{-1}\mathbf{e}(n)$$

The $\delta\mathbf{I}$ term regularizes the Gram matrix so the inverse stays conditioned when the
input history becomes nearly collinear. In this model the bracketed term is inverted explicitly by
LU decomposition, and because the projection order equals the filter length ($K = L = 6$) every
matrix in the update is square and $6 \times 6$.

## Parameters

Values as committed in the model:

| Parameter | Block | Value |
| :--- | :--- | :--- |
| Filter length $L$ | `Tapped Delay` (number of delays) | 6 |
| Projection order $K$ | `Buffer` (output buffer size, no overlap) | 6 |
| Step size $\mu$ | `Step Size` constant | 0.05 |
| Regularization $\delta$ | `Constant` | 0.001 |
| Plant A | `Discrete FIR Filter2` coefficients | `[0.1  0.2  -0.3  0.8  0.4  -0.1]` |
| Plant B | `Discrete FIR Filter3` coefficients | `[-0.1  0.3  0.9  0.5  0.8  -0.3]` |
| Plant select | `Switch` criterion | `u2 > 0`, driven by the UI button |
| RMS window | `Moving Average` | sliding window 4, overlap 3 |
| Audio capture | `Audio Capture` | 44100 Hz, frame size 4410 → one frame per 0.1 s |
| Stop time / solver | Configuration | 1000 s, variable-step auto |

## Model architecture

### 1. Excitation

`Audio Capture` delivers a 4410-sample frame every 0.1 s. A `Sum` block configured to collapse
**all dimensions** reduces each frame to a single value, which is cast to `double` to form the
scalar excitation `x[n]`. The adaptive filter therefore updates once per audio frame.

### 2. Unknown plant and runtime switching

`x[n]` feeds both 6-tap FIR plants in parallel. The `Switch` block passes exactly one of their
outputs as the desired signal `d[n]`, and its control input comes from the Android UI `Button`.
Pressing the button mid-run swaps the plant underneath the filter — the event the algorithm is
meant to track.

### 3. APA subsystem

Inside `Affine Projection Algorithm`:

| Stage | Implementation |
| :--- | :--- |
| Data matrix | `x[n]` → `Tapped Delay` (6 delays, oldest first) → `Buffer` (N = 6) assembles $\mathbf{X}$ |
| Desired alignment | `d[n]` → a matching `Tapped Delay` so the target vector is time-aligned with $\mathbf{X}$ |
| Output | `Math Function (transpose)` → `Matrix Multiply` against the current weights gives $\mathbf{y}$ |
| Error | `Sum` (`-+`) forms $\mathbf{e} = \mathbf{d} - \mathbf{y}$ |
| Gram matrix | `Matrix Multiply` forms $\mathbf{X}^{T}\mathbf{X}$; `Constant` × `Identity Matrix` forms $\delta\mathbf{I}$; a `Sum` adds them |
| Inverse | `LU Inverse` inverts $\mathbf{X}^{T}\mathbf{X} + \delta\mathbf{I}$ |
| Weight update | the inverse is multiplied by $\mu\mathbf{X}$ and then by $\mathbf{e}$; a `Sum` accumulates onto the weight vector held in a unit `Delay` |
| Outputs | `Selector` blocks take the leading element of the output and error vectors as the scalar `y` and `e` |

### 4. Error monitoring

`e` → `Math Function (square)` → `Moving Average` (window 4) → `Sqrt` produces a sliding-window RMS
error, routed to scopes alongside the filter output and the desired signal.

## Running the model

1. Install the **Simulink Support Package for Android Devices**
   (*Home → Add-Ons → Get Add-Ons*), then run its **Hardware Setup**.
2. Enable **USB debugging** on the Android device and connect it.
3. Open `Assignment_2.slx`.
4. In *Configuration Parameters → Hardware Implementation*, set **Hardware board** to
   **Android Device**. The model is committed with the default desktop setting, so this step is
   required before deployment.
5. Build and deploy with **Monitor & Tune** (*Hardware* tab) to run on the device while streaming
   signals back to the scopes.
6. Speak or play audio near the microphone to excite the filter, then tap the on-screen button to
   switch plants.

### Opening in an earlier release

Simulink only opens models saved in the same or an earlier release. To use this model in an older
MATLAB, open it in R2025b and choose
**Save → Export Model to → Previous Version**, selecting the target release.

## Results

**No captured results, plots, or demo output are included in this repository, by design.**

The excitation is live microphone audio and the plant switch is a physical touchscreen button, so
there is no meaningful output unless the model is deployed and running on an Android device. Any
figure generated without that hardware — by substituting a synthetic source for the microphone —
would describe a different experiment, not this one. Deploy the model to see the real behaviour.

When it runs on-device, the scopes show:

- the filter output tracking the desired signal,
- the error settling toward the noise floor as the plant is identified,
- an RMS error transient at each button press, decaying as the new plant is re-identified.

## Design notes

- **Projection order equals filter length.** With $K = L = 6$ the update solves a 6×6 system in 6
  unknowns each step, so the algorithm behaves closer to an exact Newton-type solve than to a
  gradient method — convergence is fast, at the cost of inverting a full matrix every sample.
- **Regularization matters here.** $\delta = 0.001$ is small. Because the inverse is taken
  explicitly rather than through a numerically-guarded update, a nearly-collinear input history
  makes $\mathbf{X}^{T}\mathbf{X}$ ill-conditioned and can produce large transient weight jumps.
  Raising $\delta$ trades steady-state accuracy for robustness.
- **Frame-summed excitation.** Collapsing each 4410-sample audio frame to one value means the filter
  operates on a 10 Hz frame-rate signal, not on the 44.1 kHz waveform.

## Author

**Vishrut Sharma** — Advanced Signal Processing, IIT Gandhinagar.
