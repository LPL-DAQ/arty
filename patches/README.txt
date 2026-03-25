=== Zephyr Patches Repository ===

This folder contains backport patches for Zephyr v4.2.0.
Please apply them in the following order:

[ 01-adc-ads79xx-backport.patch ]
- Purpose: Support TI ADS7953 driver on STM32H7.
- Path: /workspaces/arty/zephyr
- Command: git am /workspaces/arty/patches/01-adc-ads79xx-backport.patch
- Revert: git reset --hard v4.2.0

[ 02-future-fix.patch ] (If any)
- Purpose: Describe what this does...
- Command: ...
- Revert: ...
