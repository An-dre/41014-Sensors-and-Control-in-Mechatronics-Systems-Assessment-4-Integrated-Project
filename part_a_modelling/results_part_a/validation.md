# Part A model check

Made by `python -m part_a_modelling.validatemodel`. The model integrates the encoder rates every 2 ms physics step; the true pose is read from the simulator only to score it.

| Robot model | Manoeuvre | Time (s) | Model: distance, turn | True: distance, turn | Position error | Heading error |
|---|---|---|---|---|---|---|
| as the xml loads | straight | 7.5 | 0.980 m, 0.0° | 0.829 m, -0.7° | 15.0 cm | +0.7° |
| as the xml loads | spin on the spot | 9 | 0.000 m, 348.9° | 0.052 m, 242.9° | 5.2 cm | +106.0° |
| as the xml loads | arc (left 2, right 4 rad/s) | 10 | 0.779 m, 130.6° | 0.747 m, 72.0° | 38.1 cm | +58.5° |
| with the contact correction | straight | 7.5 | 0.985 m, -0.0° | 0.981 m, -0.0° | 0.4 cm | -0.0° |
| with the contact correction | spin on the spot | 9 | 0.000 m, 351.9° | 0.000 m, 350.8° | 0.0 cm | +1.0° |
| with the contact correction | arc (left 2, right 4 rad/s) | 10 | 0.782 m, 131.1° | 0.782 m, 130.1° | 0.8 cm | +0.9° |
