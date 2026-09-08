# Disclaimer

This project is unofficial, independent reverse-engineering work. It is
not affiliated with, endorsed by, or supported by the maker of Protocol
Dura VR or any related product. All product names are used for
identification purposes only.

## Physical safety

This software sends real flight commands to a real drone over WiFi.
Propellers can cause injury and the drone can cause property damage.

- The sign of each movement key (which direction it actually moves the
  drone) is not fully flight-verified. See README.md's "Axis mapping"
  table for the current verification status of each key.
- The `e` key is an emergency motor cutoff: it drops the drone
  immediately from whatever height it is at. Only use it once already on
  the ground, or in an actual emergency.
- The `--max-deflection` flag is not verified to actually limit axis
  travel; do not rely on it as a safety limit. Test one direction at a
  time, near the ground, with room to fail safely.
- Do not use this software to fly indoors, near people or animals, or
  anywhere a sudden uncommanded movement or drop could cause harm.

## Liability

Use this software entirely at your own risk. The author is not liable
for any damage, injury, or loss arising from its use.

## Accuracy of reverse-engineered information

The protocol details, byte meanings, and field interpretations in this
repository were derived by observing traffic from the original app and a
real device, not from any official specification. Anything not marked
"confirmed" or checked off as verified is a best-effort interpretation
and may be wrong.
