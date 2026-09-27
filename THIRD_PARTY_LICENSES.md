# Third-party licenses

This project includes material derived from third parties. The notices below
apply to the corresponding portions of this repository.

## Raspberry Pi "RP2350B Minimal" reference design

Applies to:

- `eda files/explorer-robot/rp235xb.pretty/` — footprints
- `eda files/explorer-robot/rp235xb.3dshapes/` — 3D STEP models

Source: Raspberry Pi Ltd, *RP2350B Minimal* reference design, release R4-S1
(upstream directory `RPI-RP2350B-MINIMAL_R4-S1_public`). Portions were copied
into this project and modified (folder names, library references, and 3D model
paths).

License: MIT

Copyright (c) 2026 Raspberry Pi Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Upstream disclaimer (retained from the original design): the design is
provided for guidance only and without guarantee; anyone intending to use it
for appearance and fitment purposes must refer to manufacturer information and
conduct suitable measurements of the physical product.

## KiCad official libraries

The schematic references symbols from KiCad's official libraries (`Device`,
`power`, `MCU_RaspberryPi`) and caches copies of them inside
`explorer-robot.kicad_sch`. These libraries are distributed under Creative
Commons CC-BY-SA-4.0 with an exception permitting unrestricted use within
designs. No license obligations propagate to this project's own design files
from that use; this notice is included for completeness.
